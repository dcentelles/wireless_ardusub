#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/Geoid.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include <chrono>
#include <cpplogging/cpplogging.h>
#include <dynamic_reconfigure/server.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Pose.h>
#include <mavlink_cpp/GCSv1.h>
#include <ros/publisher.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <telerobotics/Constants.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <thread>
#include <wireless_ardusub/pid.h>
#include <wireless_ardusub/wireless_teleop_joyConfig.h>

using namespace mavlink_cpp;
using namespace cpplogging;
using namespace std::chrono_literals;
using namespace telerobotics;

class OperatorController : public Logger {
public:
  OperatorController(ros::NodeHandle &nh);
  void Start();

private:
  struct ControlState {
    double x, y, z, r;
    ARDUSUB_NAV_MODE mode;
    bool arm;
  };
  ControlState _controlState;
  // ATTRIBUTES
  ros::NodeHandle &_nh;
  dynamic_reconfigure::Server<wireless_ardusub::wireless_teleop_joyConfig>
      _server;
  wireless_ardusub::wireless_teleop_joyConfig _config;
  ros::Subscriber _joy_sub;
  std::vector<int> _previous_buttons;

  tf::TransformListener listener;
  tf2_ros::StaticTransformBroadcaster static_broadcaster;
  geometry_msgs::TransformStamped static_transformStamped;
  std::vector<geometry_msgs::TransformStamped> static_transforms;

  tf::StampedTransform nedMrov, nedMtarget, rovMtarget, cameraMrov;
  tf::Transform rovMned;
  std::shared_ptr<GCSv1> control;

  wireless_ardusub::PID yawPID = wireless_ardusub::PID(0.1, 3.14, -3.14, 0.5, 1,
                                                       0.05),
                        vPID = wireless_ardusub::PID(0.1, 3, -3, 0.5, 1, 0.05);

  // FUNCTIONS
  bool RisingEdge(const sensor_msgs::Joy::ConstPtr &joy, int index);
  void CmdTakeoffLand(bool takeoff);
  int8_t ComputeAxisValue(const sensor_msgs::Joy::ConstPtr &joy, int index);
  uint16_t MapToPpm(double in);
  void ConfigCallback(wireless_ardusub::wireless_teleop_joyConfig &update,
                      uint32_t level);
  void JoyCallback(const sensor_msgs::Joy::ConstPtr &joy);

  double keepHeadingIteration(double diff);
  double ArduSubXYR(double per);
  double ArduSubZ(double per);
  void StopRobot();
  void Saturate(const double &max, const double &x, const double &y,
                const double &z, double &vx, double &vy, double &vz);
  void GetLinearVel(const double &diffx, const double &diffy,
                    const double &diffz, double &vx, double &vy, double &vz);

  void Loop();
  void ResetPID();

  std::thread _mainLoop;
};

OperatorController::OperatorController(ros::NodeHandle &nh) : _nh(nh) {
  // connect dynamic reconfigure
  dynamic_reconfigure::Server<
      wireless_ardusub::wireless_teleop_joyConfig>::CallbackType f;
  f = boost::bind(&OperatorController::ConfigCallback, this, _1, _2);
  _server.setCallback(f);

  // connects subs and pubs
  _joy_sub = _nh.subscribe<sensor_msgs::Joy>(
      "/joy", 1, &OperatorController::JoyCallback, this);
  SetLogName("GCS");
  SetLogLevel(info);
  SetAsyncMode();
  FlushLogOn(debug);
  SetLogFormatter(std::make_shared<spdlog::pattern_formatter>("[%T.%F] %v"));
  SetAsyncMode(true);
  uint16_t localPort = 14550;
  control = std::shared_ptr<GCSv1>(new GCSv1(localPort));
  control->SetLogName("GCS");
  control->SetLogLevel(info);

  control->EnableGPSMock(false);
  control->SetManualControl(0, 0, 0, 0);
  control->EnableManualControl(true);
  control->Start();
}

void OperatorController::ConfigCallback(
    wireless_ardusub::wireless_teleop_joyConfig &update, uint32_t level) {
  Info("reconfigure request received");
  _config = update;
}

void OperatorController::Start() {
  _mainLoop = std::thread([this]() {
    while (1) {
      try {
        listener.lookupTransform("bluerov2_camera", "sitl", ros::Time(0),
                                 cameraMrov);
        static_transformStamped.header.stamp = ros::Time::now();
        static_transformStamped.header.frame_id = "camera";
        static_transformStamped.child_frame_id = "erov";
        static_transformStamped.transform.translation.x =
            cameraMrov.getOrigin().getX();
        static_transformStamped.transform.translation.y =
            cameraMrov.getOrigin().getY();
        static_transformStamped.transform.translation.z =
            cameraMrov.getOrigin().getZ();
        static_transformStamped.transform.rotation.x =
            cameraMrov.getRotation().x();
        static_transformStamped.transform.rotation.y =
            cameraMrov.getRotation().y();
        static_transformStamped.transform.rotation.z =
            cameraMrov.getRotation().z();
        static_transformStamped.transform.rotation.w =
            cameraMrov.getRotation().w();
        static_transforms.push_back(static_transformStamped);
        break;
      } catch (tf::TransformException &ex) {
        Warn("TF: {}", ex.what());
      }
    }
    Loop();
  });
}

void OperatorController::ResetPID() {
  yawPID.Reset();
  vPID.Reset();
}

void OperatorController::Loop() {
  //  _controlState.mode = NAV_MANUAL;
  //  control->SetFlyMode(FLY_MODE_R::MANUAL);
  static_broadcaster.sendTransform(static_transforms);
  bool manual = true;

  while (ros::ok()) {
    if (_controlState.mode == NAV_GUIDED) {
      if (manual) {
        ResetPID();
        manual = false;
      }
      try {
        listener.lookupTransform("local_origin_ned", "sitl", ros::Time(0),
                                 nedMrov);
        listener.lookupTransform("local_origin_ned", "bluerov2_ghost",
                                 ros::Time(0), nedMtarget);
        listener.lookupTransform("erov", "bluerov2_ghost", ros::Time(0),
                                 rovMtarget);
      } catch (tf::TransformException &ex) {
        Warn("TF: {}", ex.what());
        control->Arm(false);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        continue;
      }
      control->SetFlyMode(FLY_MODE_R::STABILIZE);
      control->Arm(true);

      rovMned = nedMrov.inverse();
      tf::Vector3 rovTned = rovMned.getOrigin();
      tf::Quaternion rovRned = rovMned.getRotation();

      tf::Vector3 rovTtarget = rovMtarget.getOrigin();
      tf::Quaternion rovRtarget = rovMtarget.getRotation();

      tf::Vector3 nedTrov = nedMrov.getOrigin();
      tf::Quaternion nedRrov = nedMrov.getRotation();
      tf::Vector3 nedTtarget = nedMtarget.getOrigin();
      tf::Quaternion nedRtarget = nedMtarget.getRotation();

      double vx, vy, vz;
      double vTlpX = rovTtarget.getX(), vTlpY = rovTtarget.getY(),
             vTlpZ = rovTtarget.getZ();
      GetLinearVel(vTlpX, vTlpY, -vTlpZ, vx, vy, vz);

      double rdiff = tf::getYaw(rovRtarget);
      double rv = keepHeadingIteration(rdiff);
      double mr = 100 / 3.14;
      rv = mr * rv;

      double baseZ = -40;
      double newZ = vz + baseZ;
      auto x = ceil(ArduSubXYR(vx));
      auto y = ceil(ArduSubXYR(vy));
      auto z = ceil(ArduSubZ(newZ));
      auto r = ceil(ArduSubXYR(rv));

      Info("[ {} , {} , {} ] ----> [ {} , {} , {} ] ----> [ {} ({} -- {}) , "
           "{} ({} -- {}) , {} ({} -- {})] ===== [ {} : {} : {} ]",
           nedTrov.getX(), nedTrov.getY(), nedTrov.getZ(), nedTtarget.getX(),
           nedTtarget.getY(), nedTtarget.getZ(), x, vTlpX, vx, y, vTlpY, vy, z,
           vTlpZ, newZ, rdiff, rv, r);

      control->SetManualControl(x, y, z, r);
    } else {
      manual = true;
      FLY_MODE_R mode;
      switch (_controlState.mode) {
      case ARDUSUB_NAV_MODE::NAV_DEPTH_HOLD:
        mode = FLY_MODE_R::DEPTH_HOLD;
        break;
      case ARDUSUB_NAV_MODE::NAV_STABILIZE:
        mode = FLY_MODE_R::STABILIZE;
        break;
      case ARDUSUB_NAV_MODE::NAV_MANUAL:
        mode = FLY_MODE_R::MANUAL;
        break;
      default:
        mode = FLY_MODE_R::STABILIZE;
        break;
      }
      control->SetFlyMode(mode);
      control->Arm(_controlState.arm);
      auto x = ceil(ArduSubXYR(_controlState.x));
      auto y = ceil(ArduSubXYR(_controlState.y));
      auto z = ceil(ArduSubZ(_controlState.z));
      auto r = ceil(ArduSubXYR(_controlState.r));
      control->SetManualControl(x, y, z, r);
      Info("Send order: X: {} ; Y: {} ; Z: {} ; R: {} ; Arm: {}", x, y, z, r,
           _controlState.arm ? "true" : "false");
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

// from -100,100 to -1000,1000
double OperatorController::ArduSubXYR(double per) { return per * 10; }
double OperatorController::ArduSubZ(double per) { return (per + 100) / 0.2; }

double OperatorController::keepHeadingIteration(double diff) {
  double vel = yawPID.calculate(0, -1 * diff);

  return vel;
}

void OperatorController::StopRobot() {
  int x = ceil(ArduSubXYR(0));
  int y = ceil(ArduSubXYR(0));
  int z = ceil(ArduSubZ(0));
  int r = ceil(ArduSubXYR(0));
  control->SetManualControl(x, y, z, r);
}

void OperatorController::Saturate(const double &max, const double &x,
                                  const double &y, const double &z, double &vx,
                                  double &vy, double &vz) {
  double rx, ry, rz;
  rx = x / max;
  ry = y / max;
  rz = y / max;
  if (rx > 1 || ry > 1 || rz > 1) {
    double alpha = rx;
    if (alpha < ry)
      alpha = ry;
    if (alpha < rz)
      alpha = rz;
    vx = x / alpha;
    vy = y / alpha;
    vz = z / alpha;
  } else {
    vx = x;
    vy = y;
    vz = z;
  }
}

void OperatorController::GetLinearVel(const double &diffx, const double &diffy,
                                      const double &diffz, double &vx,
                                      double &vy, double &vz) {
  double mod = std::sqrt(diffx * diffx + diffy * diffy + diffz * diffz);
  double vel = vPID.calculate(0, mod);
  Saturate(vel, diffx, diffy, diffz, vx, vy, vz);
  double m = 100 / 3.;
  vx = m * vx;
  vy = m * vy;
  vz = m * vz;
}
bool OperatorController::RisingEdge(const sensor_msgs::Joy::ConstPtr &joy,
                                    int index) {
  return (joy->buttons[index] == 1 && _previous_buttons[index] == 0);
}

int8_t
OperatorController::ComputeAxisValue(const sensor_msgs::Joy::ConstPtr &joy,
                                     int index) {
  // return 0 if axis index is invalid
  if (index < 0 || index >= joy->axes.size()) {
    return 0.0;
  }

  double raw = joy->axes[index]; // raw in [-1,1]
  double dvalue = 100 * raw;
  int8_t value = ceil(dvalue);
  Log->debug("{}: Raw: {} ; DValue: {} ; Value: {}", index, raw, dvalue, value);
  return value;
}

void OperatorController::JoyCallback(const sensor_msgs::Joy::ConstPtr &joy) {
  // init previous_buttons
  if (_previous_buttons.size() != joy->buttons.size()) {
    _previous_buttons = std::vector<int>(joy->buttons);
  }

  // arm, disarm
  if (RisingEdge(joy, _config.disarm_button)) {
    _controlState.arm = false;
    Info("disarm");
  } else if (RisingEdge(joy, _config.arm_button)) {
    _controlState.arm = true;
    Info("arm");
  }

  _controlState.x = ComputeAxisValue(joy, _config.x_axis);
  _controlState.y = -1 * ComputeAxisValue(joy, _config.y_axis);
  _controlState.z = ComputeAxisValue(joy, _config.z_axis);
  _controlState.r = -1 * ComputeAxisValue(joy, _config.wz_axis);

  // mode switching
  if (RisingEdge(joy, _config.stabilize_button)) {
    _controlState.mode = ARDUSUB_NAV_MODE::NAV_STABILIZE;
  } else if (RisingEdge(joy, _config.alt_hold_button)) {
    _controlState.mode = ARDUSUB_NAV_MODE::NAV_DEPTH_HOLD;
  } else if (RisingEdge(joy, 0)) {
    _controlState.mode = ARDUSUB_NAV_MODE::NAV_MANUAL;
  } else if (RisingEdge(joy, 2)) {
    _controlState.mode = ARDUSUB_NAV_MODE::NAV_POS_HOLD;
  } else if (RisingEdge(joy, 5)) {
    _controlState.mode = ARDUSUB_NAV_MODE::NAV_GUIDED;
  }
  std::string modeName = "";
  switch (_controlState.mode) {
  case ARDUSUB_NAV_MODE::NAV_DEPTH_HOLD:
    modeName = "DEPTH HOLD";
    break;
  case ARDUSUB_NAV_MODE::NAV_STABILIZE:
    modeName = "STABILIZE";
    break;
  case ARDUSUB_NAV_MODE::NAV_MANUAL:
    modeName = "MANUAL";
    break;
  case ARDUSUB_NAV_MODE::NAV_POS_HOLD:
    modeName = "POS HOLD";
    break;
  case ARDUSUB_NAV_MODE::NAV_GUIDED:
    modeName = "GUIDED";
    break;
  default:
    break;
  }

  Info("Send order: X: {} ; Y: {} ; Z: {} ; R: {} ; Arm: {} ; Mode: {}",
       _controlState.x, _controlState.y, _controlState.z, _controlState.r,
       _controlState.arm ? "true" : "false", modeName);

  // remember current button states for future comparison
  _previous_buttons = std::vector<int>(joy->buttons);
}

int main(int argc, char **argv) {
  auto log = CreateLogger("Main");
  log->SetLogLevel(info);
  log->FlushLogOn(debug);
  log->LogToConsole(true);
  log->SetAsyncMode();

  ros::init(argc, argv, "custom_setpoint");
  ros::NodeHandle nh;
  ros::Rate rate(10);

  OperatorController op(nh);
  op.Start();

  while (1) {
    ros::spinOnce();
    rate.sleep();
  }
  // op.Loop();
}
