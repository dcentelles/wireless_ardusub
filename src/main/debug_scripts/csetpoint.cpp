#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/Geoid.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include <chrono>
#include <cpplogging/cpplogging.h>
#include <dccomms/Utils.h>
#include <dynamic_reconfigure/server.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Pose.h>
#include <mavlink_cpp/GCSv1.h>
#include <merbots_whrov_msgs/debug.h>
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

  tf::StampedTransform nedMerov, nedMtarget, cameraMrov;
  tf::Transform rovMtarget;
  tf::Transform rovMned;
  std::shared_ptr<GCSv1> control;
  ros::Publisher debugPublisher0;

  dccomms::Timer timer;

  double vmax = 1000, vmin = -1000;
  wireless_ardusub::PID yawPID = wireless_ardusub::PID(vmax, vmin, 10, 20, 0.05),
                        xPID = wireless_ardusub::PID(vmax, vmin, 15, 60, 0.05),
                        yPID = wireless_ardusub::PID(vmax, vmin, 15, 60, 0.05),
                        zPID = wireless_ardusub::PID(vmax, vmin, 20, 10, 0.05);

  // FUNCTIONS
  bool RisingEdge(const sensor_msgs::Joy::ConstPtr &joy, int index);
  void CmdTakeoffLand(bool takeoff);
  int8_t ComputeAxisValue(const sensor_msgs::Joy::ConstPtr &joy, int index);
  uint16_t MapToPpm(double in);
  void ConfigCallback(wireless_ardusub::wireless_teleop_joyConfig &update,
                      uint32_t level);
  void JoyCallback(const sensor_msgs::Joy::ConstPtr &joy);

  double keepHeadingIteration(const double &dt, double diff);
  double ArduSubXYR(double per);
  double ArduSubZ(double per);
  void StopRobot();
  void Saturate(const double &max, const double &x, const double &y,
                const double &z, double &vx, double &vy, double &vz);
  void GetLinearXVel(const double &dt, const double &diff, double &v);
  void GetLinearYVel(const double &dt, const double &diff, double &v);
  void GetLinearZVel(const double &dt, const double &diff, double &v);
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

  _controlState.arm = false;
  _controlState.mode = NAV_MANUAL;

  debugPublisher0 =
      _nh.advertise<merbots_whrov_msgs::debug>("/rov_controller", 1);
}

void OperatorController::ConfigCallback(
    wireless_ardusub::wireless_teleop_joyConfig &update, uint32_t level) {
  Info("reconfigure request received");
  _config = update;
}

void OperatorController::Start() {
  _mainLoop = std::thread([this]() { Loop(); });
}

void OperatorController::ResetPID() {
  yawPID.Reset();
  xPID.Reset();
  yPID.Reset();
  zPID.Reset();
  timer.Reset();
}

void OperatorController::Loop() {
  static_broadcaster.sendTransform(static_transforms);
  bool manual = true;

  merbots_whrov_msgs::debug debugMsg;

  double tyaw, cyaw;
  while (ros::ok()) {
    if (_controlState.mode == NAV_GUIDED) {
      if (manual) {
        ResetPID();
        manual = false;
      }
      try {
        listener.lookupTransform("local_origin_ned", "erov", ros::Time(0),
                                 nedMerov);
        listener.lookupTransform("local_origin_ned", "bluerov2_ghost",
                                 ros::Time(0), nedMtarget);
      } catch (tf::TransformException &ex) {
        Warn("TF: {}", ex.what());
        control->Arm(false);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        continue;
      }
      control->SetFlyMode(FLY_MODE_R::STABILIZE);
      control->Arm(true);

      tf::Vector3 nedTerov = nedMerov.getOrigin();
      cyaw = tf::getYaw(nedMerov.getRotation());

      tf::Vector3 nedTtarget = nedMtarget.getOrigin();
      tyaw = tf::getYaw(nedMtarget.getRotation());

      rovMtarget = nedMerov.inverse() * nedMtarget;

      tf::Vector3 erovTtarget = rovMtarget.getOrigin();
      tf::Quaternion erovRtarget = rovMtarget.getRotation();

      double vx, vy, vz;
      double vTlpX = erovTtarget.getX(), vTlpY = erovTtarget.getY(),
             vTlpZ = erovTtarget.getZ();

      double elapsedSecs = timer.Elapsed() / 1000.;

      GetLinearXVel(elapsedSecs, vTlpX, vx);
      GetLinearYVel(elapsedSecs, vTlpY, vy);
      GetLinearZVel(elapsedSecs, -vTlpZ, vz);

      if (vx > 100)
        vx = 100;
      if (vx < -100)
        vx = -100;
      if (vy > 100)
        vy = 100;
      if (vy < -100)
        vy = -100;
      if (vz > 100)
        vz = 100;
      if (vz < -100)
        vz = -100;

      double rdiff = tf::getYaw(erovRtarget);
      double rv0 = keepHeadingIteration(elapsedSecs, rdiff);

      double baseZ = 0; //-38;
      double newZ = vz + baseZ;
      auto x = ceil(ArduSubXYR(vx));
      auto y = ceil(ArduSubXYR(vy));
      auto z = ceil(ArduSubZ(newZ));
      auto r = ceil(ArduSubXYR(rv0));

      double yoffset = 90;
      double xoffset = 90;
      double roffset = 460;
      double zoffset = 10;
      double deadband = 0;

      if (y > deadband)
        y += yoffset;
      else if (y < -deadband)
        y -= yoffset;

      if (x > deadband)
          x += xoffset;
      else if (x < -deadband)
          x -= xoffset;

      if (z > 500)
          z += 150;
      else if (z < 500)
          z -= zoffset;

      if (r > deadband)
          r += roffset + 5;
      else if (r < -deadband)
          r -= roffset;


//      if (r > 10)
//          z += roffset;
//      else if (r < -10)
//          z -= roffset;


      Info("Send order: X: {} ({}) ; Y: {} ({}) ; Z: {} ({}) ; R: {} ;  rdiff: "
           "{} ; rout: {} "
           "; rinput: {} ; Arm: {}",
           x, vx, y, vy, z, vz, r, rdiff, rv0, rv0,
           _controlState.arm ? "true" : "false");

      debugMsg.pout_yaw = r;
      debugMsg.pout_x = x;
      debugMsg.pout_y = y;
      debugMsg.pout_z = z;
      debugMsg.error_yaw = rdiff;
      debugMsg.error_x = vTlpX;
      debugMsg.error_y = vTlpY;
      debugMsg.error_z = -vTlpZ;
      debugMsg.target_yaw = tyaw;
      debugMsg.target_x = nedTtarget.getX();
      debugMsg.target_y = nedTtarget.getY();
      debugMsg.target_z = nedTtarget.getZ();
      debugMsg.current_yaw = cyaw;
      debugMsg.current_x = nedTerov.getX();
      debugMsg.current_y = nedTerov.getY();
      debugMsg.current_z = nedTerov.getZ();

      debugPublisher0.publish(debugMsg);

      control->SetManualControl(x, y, z, r);
      timer.Reset();
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

double OperatorController::keepHeadingIteration(const double &dt, double diff) {
  double vel = yawPID.calculate(dt, 0, -1 * diff);

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

void OperatorController::GetLinearXVel(const double &dt, const double &diffx,
                                       double &vx) {
  vx = xPID.calculate(dt, 0, -diffx);
}
void OperatorController::GetLinearYVel(const double &dt, const double &diffy,
                                       double &vy) {
  vy = yPID.calculate(dt, 0, -diffy);
}
void OperatorController::GetLinearZVel(const double &dt, const double &diffz,
                                       double &vz) {
  vz = zPID.calculate(dt, 0, -diffz);
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

  //  Info("Send order: X: {} ; Y: {} ; Z: {} ; R: {} ; Arm: {} ; Mode: {}",
  //       _controlState.x, _controlState.y, _controlState.z, _controlState.r,
  //       _controlState.arm ? "true" : "false", modeName);

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
}
