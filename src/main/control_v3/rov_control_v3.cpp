#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/Geoid.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include <chrono>
#include <cpplogging/cpplogging.h>
#include <dccomms_utils/S100Stream.h>
#include <eigen_conversions/eigen_msg.h>
#include <image_utils_ros_msgs/EncodedImg.h>
#include <image_utils_ros_msgs/EncodingConfig.h>
#include <mavlink_cpp/GCSv1.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <telerobotics/HROVMessageV2.h>
#include <telerobotics/OperatorMessageV2.h>
#include <telerobotics/ROV.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <wireless_ardusub/Constants.h>
#include <wireless_ardusub/utils.hpp>

#include <mavros/frame_tf.h>
#include <merbots_whrov_msgs/debug.h>
#include <tf_conversions/tf_eigen.h>
#include <wireless_ardusub/pid.h>

using namespace cpplogging;
using namespace std::chrono_literals;
using namespace telerobotics;
using namespace mavlink_cpp;
using namespace std;
using namespace telerobotics;
using namespace wireless_ardusub;

struct Params {
  std::string serialPort, masterUri, dccommsId;
  bool log2Console, log2File;
};

struct HROVPose {
  double yaw, pitch, roll, Z, X, Y, heading;
};

static LoggerPtr Log;
static Params params;

static uint16_t localPort = 14550;
static dccomms::Ptr<GCSv1> control = dccomms::CreateObject<GCSv1>(localPort);

static dccomms::Ptr<OperatorMessageV2> currentOperatorMessage =
    dccomms::CreateObject<OperatorMessageV2>();
static bool currentOperatorMessage_updated;
static std::mutex currentOperatorMessage_mutex;
static std::condition_variable currentOperatorMessage_cond;
static TeleopOrderPtr lastOrder;
static int xVel;
static int yVel;
static int zVel;
static int rVel;

static dccomms::Ptr<HROVMessageV2> currentHROVMessageV2 =
    dccomms::CreateObject<HROVMessageV2>();
static bool currentHROVMessage_updated;
static std::mutex currentHROVMessage_mutex;
static std::condition_variable currentHROVMessage_cond;

static std::thread operatorMsgParserWorker;
static std::thread messageSenderWorker;
static std::thread keepOrientationWorker;
static std::thread holdChannelWorker;

static ros::Subscriber encodedImage_sub;
static ros::Publisher encodingConfig_pub;
static ros::Publisher setenupos_pub;
static ros::Publisher debugPublisher0;
static dccomms::Ptr<ROV> commsNode;

static image_utils_ros_msgs::EncodingConfig emsg;
static int lastImageSize = -1;

static int desiredOrientation;
static bool keepOrientation;
static std::condition_variable keepOrientation_cond;

static uint16_t holdChannelSeconds;
static bool holdChannelReceived;
static std::condition_variable holdChannel_cond;

static bool cancelLastOrderFlag;

static bool executingOrder;
static std::mutex executingOrder_mutex;
static std::condition_variable executingOrder_cond;
static std::condition_variable currentHROVPose_cond;
static bool armed;
static std::mutex ned_mutex;
static std::condition_variable ned_cond;
static double ned_x, ned_y, ned_z;
static std::mutex attitude_mutex;
static tfScalar g_yaw, g_pitch, g_roll;

static ARDUSUB_NAV_MODE lastReceivedMode;

struct PoseRegister {
  tf::Vector3 position;
  tf::Quaternion orientation;
};

static bool communication_lost = false;
static std::list<PoseRegister> poseStack;
static int maxPoseStackSize = 4096;
static ros::Publisher pose_pub;

void notifyHROVMessageUpdated() {
  currentHROVMessage_updated = true;
  currentHROVMessage_cond.notify_one();
}
void notifyLastOrderCancellationAndRobotReady() {
  executingOrder_mutex.lock();
  executingOrder = false;
  executingOrder_cond.notify_one();
  executingOrder_mutex.unlock();

  currentHROVMessage_mutex.lock();
  currentHROVMessageV2->LastOrderCancelledFlag(true);
  currentHROVMessageV2->Ready(true);
  notifyHROVMessageUpdated();
  currentHROVMessage_mutex.unlock();
}

void notifyROVBusy() {
  executingOrder_mutex.lock();
  executingOrder = true;
  executingOrder_cond.notify_one();
  executingOrder_mutex.unlock();

  currentHROVMessage_mutex.lock();
  currentHROVMessageV2->Ready(false);
  notifyHROVMessageUpdated();
  currentHROVMessage_mutex.unlock();
}

void notifyROVReady() {
  executingOrder_mutex.lock();
  executingOrder = false;
  executingOrder_cond.notify_one();
  executingOrder_mutex.unlock();

  currentHROVMessage_mutex.lock();
  currentHROVMessageV2->Ready(true);
  notifyHROVMessageUpdated();
  currentHROVMessage_mutex.unlock();
}

static double vmax = 1000, vmin = -1000;
static wireless_ardusub::PID yawPID = wireless_ardusub::PID(vmax, vmin, 15, 20,
                                                            0.05),
                             xPID =
                                 wireless_ardusub::PID(vmax, vmin, 10, 20, 0.4),
                             yPID =
                                 wireless_ardusub::PID(vmax, vmin, 10, 20, 0.4),
                             zPID =
                                 wireless_ardusub::PID(vmax, vmin, 10, 10, 0.4);

static dccomms::Timer timer;

void ResetPID() {
  yawPID.Reset();
  xPID.Reset();
  yPID.Reset();
  zPID.Reset();
  timer.Reset();
}

// from -127,127  to -100,100
double getJoyAxisNormalized(int x) { return 200. / 256 * x; }
// from -100,100 to -1000,1000
double ArduSubXYR(double per) { return per * 10; }
double ArduSubZ(double per) { return (per + 100) / 0.2; }

void stopRobot() {
  int x = ceil(ArduSubXYR(0));
  int y = ceil(ArduSubXYR(0));
  int z = ceil(ArduSubZ(0));
  int r = ceil(ArduSubXYR(0));
  control->SetManualControl(x, y, z, r);
}

void moveYaw(double per) {
  if (lastOrder) {
    rVel = ceil(ArduSubXYR(per));
    control->SetManualControl(xVel, yVel, zVel, rVel);
  }
}

static tf::Transform nedMtarget;
static bool guidedMode = false;
static bool goToMission = false;
static std::mutex goToMission_mutex;
static std::condition_variable goToMission_cond;

void updateImageSettings(const HROVSettingsV2Ptr &lastSettings) {
  notifyROVBusy();
  Log->Info("Current image settings:\n"
            "\t(x0,y0): ({},{})\n"
            "\t(x1,y1): ({},{})\n)"
            "\tsize: {} bytes"
            "\tROI shift: {}"
            "\tEncode mono version: {}",
            lastSettings->GetROIX0(), lastSettings->GetROIY0(),
            lastSettings->GetROIX1(), lastSettings->GetROIY1(),
            lastSettings->GetImgSize(), lastSettings->GetROIShift(),
            lastSettings->EncodeMonoVersion() ? "true" : "false");

  emsg.max_size = lastSettings->GetImgSize();
  emsg.shift = lastSettings->GetROIShift();
  emsg.x0 = lastSettings->GetROIX0();
  emsg.y0 = lastSettings->GetROIY0();
  emsg.x1 = lastSettings->GetROIX1();
  emsg.y1 = lastSettings->GetROIY1();
  emsg.encode8bversion = lastSettings->EncodeMonoVersion();

  encodingConfig_pub.publish(emsg);
  notifyROVReady();
}

void cancelLastOrder() {
  std::unique_lock<std::mutex> lock(executingOrder_mutex);
  while (executingOrder) {
    cancelLastOrderFlag = true;
    executingOrder_cond.wait(lock);
  }
  cancelLastOrderFlag = false;
  lock.unlock();
}

void sendGoToLocalNED(double x, double y, double z, double yaw) {

  tf::Quaternion ned_orientation;
  ned_orientation.setRPY(0, 0, yaw);
  tf::Vector3 ned_position(x, y, z);
  nedMtarget.setOrigin(ned_position);
  nedMtarget.setRotation(ned_orientation.normalize());
  goToMission = true;
}

void handleNewOrder() {
  currentHROVMessage_mutex.lock();
  auto eSeq = currentHROVMessageV2->GetExpectedOrderSeqNumber();
  currentHROVMessage_mutex.unlock();

  auto orderType = currentOperatorMessage->GetOrderType();
  auto cSeq = currentOperatorMessage->GetOrderSeqNumber();
  auto _cancelLastOrder = currentOperatorMessage->CancelLastOrder();

  if (eSeq == cSeq) {
    if (_cancelLastOrder) {
      Log->Debug("Cancelation requested");
      cancelLastOrder();
      Log->Debug("Cancelled");
    } else if (orderType != OperatorMessageV2::OrderType::NoOrder) {
      Log->Debug("New order requested. Cancel current order (if any)");
      cancelLastOrder();
      Log->Debug("Cancelled");
      switch (orderType) {
      case OperatorMessageV2::OrderType::HoldChannel: {
        holdChannelSeconds = currentOperatorMessage->GetHoldChannelDuration();
        Log->Info("Received hold channel duration order: {} seconds",
                  holdChannelSeconds);
        holdChannelReceived = true;
        holdChannel_cond.notify_one();
        break;
      }
      case OperatorMessageV2::OrderType::KeepOrientation: {
        desiredOrientation = currentOperatorMessage->GetKeepOrientationValue();
        Log->Info("Received keep orientation order: {} degrees",
                  desiredOrientation);
        keepOrientation = true;
        keepOrientation_cond.notify_one();
        break;
      }
      case OperatorMessageV2::OrderType::DisableKeepOrientation: {
        Log->Info("Received disable keep orientation order");
        keepOrientation = false;
        keepOrientation_cond.notify_one();
        break;
      }
      case OperatorMessageV2::OrderType::UpdateImageSettings: {
        auto lastSettings = currentOperatorMessage->GetImageSettingsOrderCopy();
        updateImageSettings(lastSettings);
        break;
      }
      case OperatorMessageV2::OrderType::GoTo: {
        int x, y, z, heading;

        currentOperatorMessage->GetGoToOrder(x, y, z, heading);
        // We received NED coordinates.
        // see
        // https://github.com/mavlink/mavros/blob/de9f39a719b091b8448214a17d27b3b1c415d0dc/mavros/src/lib/uas_data.cpp#L55

        double x2 = x / 100.;
        double y2 = y / 100.;
        double z2 = z / 100.;

        // heading: 0-360
        double yaw = heading - 360;
        yaw = yaw * M_PI / 180.;

        if (guidedMode) {
          Log->Info("GoTo: {} ; {} ; {} ; {} ({})", x2, y2, z2, heading, yaw);
          sendGoToLocalNED(x2, y2, z2, yaw);
        } else {
          Log->Info("Not in guided mode");
        }
        break;
      }
      }
      currentHROVMessage_mutex.lock();
      currentHROVMessageV2->IncExpectedOrderSeqNumber();
      currentHROVMessage_mutex.unlock();
      notifyHROVMessageUpdated();
    }
  }
}

void arm(bool v) {
  // We avoid set control->Arm(false) because there is a bug in the ArduSub
  // firmware
  // that adds an offset to the NED position when rearming.
  if (v) {
    control->Arm(true);
  } else {
    // control->Arm(false);
    stopRobot();
    control->SetFlyMode(FLY_MODE_R::MANUAL);
  }
  armed = v;
}
void messageSenderWork() {
  while (1) {
    std::unique_lock<std::mutex> lock(currentHROVMessage_mutex);
    while (!currentHROVMessage_updated) {
      currentHROVMessage_cond.wait(lock);
    }

    auto oid = currentHROVMessageV2->GetExpectedOrderSeqNumber();
    auto cancelled = currentHROVMessageV2->LastOrderCancelledFlag();
    auto ready = currentHROVMessageV2->Ready();
    auto roll = currentHROVMessageV2->GetRoll();
    auto pitch = currentHROVMessageV2->GetPitch();
    auto x = currentHROVMessageV2->GetX();
    auto y = currentHROVMessageV2->GetY();
    auto altitude = currentHROVMessageV2->GetZ();
    auto heading = currentHROVMessageV2->GetHeading();
    auto navMode = (int)(currentHROVMessageV2->GetNavMode());

    commsNode->SetCurrentTxState(currentHROVMessageV2->GetBuffer(),
                                 currentHROVMessageV2->GetMsgSize());
    auto armed = currentHROVMessageV2->Armed();
    Log->Info("OWN STATE - OID: {} ; CC: {} ; RDY: {} ; HD: {} ; NAV: {} ; "
              "ARMED: {} ; x:y:z: "
              "{} : {} : {}",
              oid, cancelled ? 1 : 0, ready ? 1 : 0, heading, navMode,
              armed ? 1 : 0, x, y, altitude);
    currentHROVMessage_updated = false;
  }
}

void operatorMsgParserWork() {
  while (1) {
    std::unique_lock<std::mutex> lock(currentOperatorMessage_mutex);
    while (!currentOperatorMessage_updated) {
      currentOperatorMessage_cond.wait_for(lock, chrono::milliseconds(5000));
      if (!currentOperatorMessage_updated && !commsNode->HoldingChannel()) {
        if (!lastReceivedMode != ARDUSUB_NAV_MODE::NAV_GUIDED) {
          //          arm(false);
          //          Log->Warn("Heartbeat lost. Stopping robot to avoid
          //          thruster "
          //                    "interferences!");
          Log->Warn("Heartbeat lost! Considering communication lost and "
                    "launching pose recovery");
        }
        // control->SetFlyMode(FLY_MODE_R::GUIDED);
        // communication_lost = true;
        control->SetFlyMode(FLY_MODE_R::MANUAL);
        stopRobot();
      }
    }
    communication_lost = false;
    currentOperatorMessage_updated = false;

    OperatorMessageV2::OrderType lastOrderType =
        currentOperatorMessage->GetOrderType();
    if (lastOrderType == OperatorMessageV2::OrderType::Move) {
      lastOrder = currentOperatorMessage->GetMoveOrderCopy();
      arm(lastOrder->Arm());
      auto currentMode = lastReceivedMode;
      lastReceivedMode = lastOrder->GetFlyMode();
      if (armed) {
        std::string modeName = "";
        switch (lastReceivedMode) {
        case ARDUSUB_NAV_MODE::NAV_DEPTH_HOLD:
          modeName = "DEPTH HOLD";
          control->SetDepthHoldMode();
          break;
        case ARDUSUB_NAV_MODE::NAV_STABILIZE:
          modeName = "STABILIZE";
          control->SetStabilizeMode();
          break;
        case ARDUSUB_NAV_MODE::NAV_MANUAL:
          modeName = "MANUAL";
          control->SetManualMode();
          break;
        case ARDUSUB_NAV_MODE::NAV_POS_HOLD:
          modeName = "POS HOLD";
          control->SetFlyMode(FLY_MODE_R::POS_HOLD);
          break;
        case ARDUSUB_NAV_MODE::NAV_GUIDED:
          modeName = "GUIDED";
          guidedMode = true;
          control->SetManualMode();
          if (currentMode != ARDUSUB_NAV_MODE::NAV_GUIDED) {
            ResetPID();
          }
          break;
        default:
          break;
        }
        Log->Info(
            "Send order: X: {} ; Y: {} ; Z: {} ; R: {} ; Arm: {} ; Mode: {}",
            lastOrder->GetX(), lastOrder->GetY(), lastOrder->GetZ(),
            lastOrder->GetR(), lastOrder->Arm() ? "true" : "false", modeName);
        int x = lastOrder->GetX();
        int y = lastOrder->GetY();
        int z = lastOrder->GetZ();
        int r = lastOrder->GetR();
        // order:  y = 200/256x
        // z control: y = 200/1000x - 100
        // x,y and r control: y = 200/2000x

        double xNorm = getJoyAxisNormalized(x);
        double yNorm = getJoyAxisNormalized(y);
        double zNorm = getJoyAxisNormalized(z);
        double rNorm = getJoyAxisNormalized(r);

        xVel = ceil(ArduSubXYR(xNorm));
        yVel = ceil(ArduSubXYR(yNorm));
        zVel = ceil(ArduSubZ(zNorm));
        if (!keepOrientation)
          rVel = ceil(ArduSubXYR(rNorm));

        //        Log->Info("Manual control: X: {} ; Y: {} ; Z: {} ; R: {} ;
        //        Arm: {} ; "
        //                  "Mode  : {} ",
        //                  xVel, yVel, zVel, rVel, lastOrder->Arm() ? "true" :
        //                  "false",
        //                  modeName);

        if (lastReceivedMode != ARDUSUB_NAV_MODE::NAV_GUIDED) {
          guidedMode = false;
          goToMission = false;
          control->SetManualControl(xVel, yVel, zVel, rVel);
        }
      }
    } else {
      handleNewOrder();
    }
  }
}

void startWorkers() {
  operatorMsgParserWorker = std::thread(operatorMsgParserWork);
  messageSenderWorker = std::thread(messageSenderWork);

  std::thread poseWorker([&]() {
    PoseRegister reg;
    double error = 0.5;
    bool recoveringLastPose = false;
    while (1) {
      std::unique_lock<std::mutex> lock(ned_mutex);
      ned_cond.wait(lock);
      if (!communication_lost) {
        recoveringLastPose = false;
        // save current position
        reg.position = tf::Vector3(ned_x, ned_y, ned_z);
        reg.orientation.setRPY(0, 0, g_yaw);
        poseStack.push_back(reg);
        if (poseStack.size() > maxPoseStackSize) {
          poseStack.pop_front();
        }
        lock.unlock();
        std::this_thread::sleep_for(chrono::seconds(2));
        Log->Info("Pose stack size: {}", poseStack.size());
      } else {
        // Pose recovery
        double x, y, z, yaw;
        x = ned_x;
        y = ned_y;
        z = ned_z;
        yaw = g_yaw;
        lock.unlock();
        if (poseStack.size() > 0) {
          tf::Vector3 position;
          double yaw, xerr, yerr, zerr;
          bool reached = false;
          do {
            position = poseStack.rbegin()->position;
            yaw = tf::getYaw(poseStack.rbegin()->orientation);
            xerr = abs(position.x() - x);
            yerr = abs(position.y() - y);
            zerr = abs(position.z() - z);
            reached = xerr <= error && yerr <= error && zerr <= error;
            if (reached)
              poseStack.pop_back();
          } while (reached && poseStack.size() > 0);
          Log->Warn("Recovering position: [ {} ; {} ; {} ]", position.x(),
                    position.y(), position.z());
          sendGoToLocalNED(position.x(), position.y(), position.z(), yaw);
          std::this_thread::sleep_for(chrono::seconds(2));
        }
      }
    }
  });
  poseWorker.detach();
}

void handleNewImage(image_utils_ros_msgs::EncodedImgConstPtr msg) {
  if (!commsNode->SendingCurrentImage()) {
    lastImageSize =
        emsg.max_size <= msg->img.size() ? emsg.max_size : msg->img.size();
    if (lastImageSize > 0) {
      commsNode->SendImage((void *)msg->img.data(), lastImageSize);
    }
  } else {
    if (lastImageSize != emsg.max_size) {
      Log->Warn("TX CANCEL LAST IMAGE {} {} {}", lastImageSize, emsg.max_size,
                msg->img.size());
      commsNode->CancelLastImage();
    }
  }
}

void handleNewHUDData(const mavlink_vfr_hud_t &msg) {
  //  currentHROVMessage_mutex.lock();
  //  currentHROVMessageV2->SetHeading(msg.heading);
  //  // currentHROVMessageV2->SetZ(-1 * msg.alt * 100);
  //  currentHROVMessage_updated = true;
  //  currentHROVMessage_mutex.unlock();
  //  currentHROVMessage_cond.notify_one();
}

void handleNewNavigationData(const mavlink_attitude_t &attitude) {
  attitude_mutex.lock();
  // g_yaw = attitude.yaw;
  g_pitch = attitude.pitch;
  g_roll = attitude.roll;
  attitude_mutex.unlock();

  // Log->Info("yaw: {} ; pitch: {} ; roll: {}", g_yaw, g_pitch, g_roll);
  int rx, ry, rz;
  rx = telerobotics::utils::GetDiscreteYaw(g_roll);
  ry = telerobotics::utils::GetDiscreteYaw(g_pitch);
  // rz = telerobotics::utils::GetDiscreteYaw(g_yaw);

  rx = rx > 180 ? -(360 - rx) : rx;
  ry = ry > 180 ? -(360 - ry) : ry;
  // rz = rz > 180 ? -(360-rz) : rz;

  currentHROVMessage_mutex.lock();
  currentHROVMessageV2->SetRoll(rx);
  currentHROVMessageV2->SetPitch(ry);
  // currentHROVMessage_updated = true;
  currentHROVMessage_mutex.unlock();
  // currentHROVMessage_cond.notify_one();
}

void handleNewArdusubState(const mavlink_heartbeat_t &msg) {
  ARDUSUB_NAV_MODE mode;
  if (armed) {
    if (guidedMode)
      mode = ARDUSUB_NAV_MODE::NAV_GUIDED;
    else {
      switch (msg.custom_mode) {
      case FLY_MODE_R::MANUAL:
        mode = ARDUSUB_NAV_MODE::NAV_MANUAL;
        break;
      case FLY_MODE_R::STABILIZE:
        mode = ARDUSUB_NAV_MODE::NAV_STABILIZE;
        break;
      case FLY_MODE_R::DEPTH_HOLD:
        mode = ARDUSUB_NAV_MODE::NAV_DEPTH_HOLD;
        break;
      case FLY_MODE_R::POS_HOLD:
        mode = ARDUSUB_NAV_MODE::NAV_POS_HOLD;
        break;
      case FLY_MODE_R::GUIDED:
        mode = ARDUSUB_NAV_MODE::NAV_GUIDED;
        break;
      default:
        mode = ARDUSUB_NAV_MODE::NAV_UNKNOWN;
        break;
      }
    }
  } else {
    mode = lastReceivedMode;
  }

  // bool armed = msg.base_mode & MAV_MODE_FLAG_SAFETY_ARMED;
  // armed = msg.base_mode & MAV_MODE_FLAG_SAFETY_ARMED;

  currentHROVMessage_mutex.lock();
  currentHROVMessageV2->SetNavMode(mode);
  currentHROVMessageV2->Armed(armed);
  // currentHROVMessage_updated = true;
  currentHROVMessage_mutex.unlock();
  // currentHROVMessage_cond.notify_one();
}

void initROSInterface(int argc, char **argv) {
  ros::NodeHandle nh;
  encodedImage_sub = nh.subscribe<image_utils_ros_msgs::EncodedImg>(
      "encoded_image", 1, boost::bind(handleNewImage, _1));
  encodingConfig_pub =
      nh.advertise<image_utils_ros_msgs::EncodingConfig>("encoding_config", 1);

  setenupos_pub = nh.advertise<geometry_msgs::PoseStamped>(
      "/mavros/setpoint_position/local", 1);

  debugPublisher0 =
      nh.advertise<merbots_whrov_msgs::debug>("/rov_controller", 1);

  control->SetVfrHudCb(handleNewHUDData);
  control->SetAttitudeCb(handleNewNavigationData);
  control->SetHeartbeatCb(handleNewArdusubState);
}

int getParams() {
  ros::NodeHandle nh("~");
  std::string dccommsId;
  if (nh.getParam("dccommsId", dccommsId)) {
    Log->Info("dccommsId: {}", dccommsId);
    params.dccommsId = dccommsId;
  } else
    params.dccommsId = "rov";

  std::string serialPort;
  if (!nh.getParam("port", serialPort)) {
    Log->Info("Using comms service with dccomms Id '{}'", params.dccommsId);
  } else {
    if (serialPort == "service")
      Log->Info("Using comms service with dccomms Id '{}'", params.dccommsId);
    else
      Log->Info("Serial port: {}", serialPort);
  }
  params.serialPort = serialPort;

  char *cmasterUri = getenv("ROS_MASTER_URI");
  std::string masterUri = cmasterUri;
  Log->Info("ROS MASTER URI: {}", masterUri);

  params.masterUri = masterUri;

  bool log2Console;
  if (!nh.getParam("log2Console", log2Console)) {
    bool defaultValue = true;
    Log->Info("log2Console set to default => {}", defaultValue);
    params.log2Console = defaultValue;
  } else {
    Log->Info("log2Console: {}", log2Console);
    params.log2Console = log2Console;
  }

  bool log2File;
  if (!nh.getParam("log2File", log2File)) {
    bool defaultValue = false;
    Log->Info("log2File set to default => {}", defaultValue);
    params.log2File = defaultValue;
  } else {
    Log->Info("log2File: {}", log2File);
    params.log2File = log2File;
  }

  return 0;
}

double keepHeadingIteration(const double &dt, double diff) {
  double vel = yawPID.calculate(dt, 0, -1 * diff);
  return vel;
}

void GetLinearXVel(const double &dt, const double &diffx, double &vx) {
  vx = xPID.calculate(dt, 0, -diffx);
}
void GetLinearYVel(const double &dt, const double &diffy, double &vy) {
  vy = yPID.calculate(dt, 0, -diffy);
}
void GetLinearZVel(const double &dt, const double &diffz, double &vz) {
  vz = zPID.calculate(dt, 0, -diffz);
}

int main(int argc, char **argv) {
  Log = CreateLogger("ROVMain");
  Log->Info("Init");
  Log->SetAsyncMode(true);

  //// GET PARAMS
  ros::init(argc, argv, "rov_control");

  if (getParams())
    return 1;

  Log->SetLogLevel(cpplogging::LogLevel::info);
  // Log->FlushLogOn(cpplogging::LogLevel::info);
  Log->LogToConsole(params.log2Console);

  control->SetLogName("GCS");
  control->SetAsyncMode();
  control->SetLogLevel(info);
  arm(false);
  control->LogToConsole(params.log2Console);
  control->EnableGPSMock(false);

  initROSInterface(argc, argv);
  startWorkers();
  commsNode = dccomms::CreateObject<ROV>();

  commsNode->SetImageTrunkLength(DEFAULT_IMG_TRUNK_LENGTH);
  commsNode->SetAsyncMode();

  if (params.log2File) {
    Log->SetLogFormatter(
        std::make_shared<spdlog::pattern_formatter>("[%T.%F] %v"));
    Log->LogToFile("rov_v3_control");
    control->LogToFile("rov_v3_gcs");
    commsNode->LogToFile("rov_v3_comms_node");
    commsNode->SetLogFormatter(
        std::make_shared<spdlog::pattern_formatter>("[%T.%F] %v"));
  }

  if (params.serialPort != "service") {
    Log->Info("CommsDevice type: dccomms::SerialPortStream");
    dccomms::Ptr<SerialPortStream> stream =
        dccomms::CreateObject<dccomms::SerialPortStream>(
            params.serialPort, SerialPortStream::BAUD_9600);
    stream->SetHwFlowControl(true);
    stream->Open();
    commsNode->SetComms(stream);
  } else {
    Log->Info("CommsDevice type: dccomms::CommsDeviceService");

    std::string dccommsId = params.dccommsId;
    Log->Info("dccomms ID: {}", dccommsId);

    dccomms::Ptr<IPacketBuilder> pb =
        dccomms::CreateObject<VariableLengthPacketBuilder>();

    dccomms::Ptr<CommsDeviceService> commsService;
    commsService = dccomms::CreateObject<CommsDeviceService>(pb);
    commsService->SetCommsDeviceId(dccommsId);
    commsService->SetLogLevel(LogLevel::info);
    commsService->Start();
    commsNode->SetComms(commsService);
  }

  commsNode->LogToConsole(params.log2Console);
  commsNode->SetLogLevel(LogLevel::info);

  commsNode->SetOrdersReceivedCallback([](ROV &receiver) {
    Log->Info("Orders received!");
    uint8_t state[300];
    receiver.GetCurrentRxState(state);

    currentOperatorMessage_mutex.lock();
    currentOperatorMessage->UpdateFromBuffer(state);
    currentOperatorMessage_mutex.unlock();
    currentOperatorMessage_updated = true;
    currentOperatorMessage_cond.notify_one();

    // Log->Info("Orders updated");
  });
  commsNode->Start();

  currentHROVMessageV2->Ready(true);
  commsNode->SetCurrentTxState(currentHROVMessageV2->GetBuffer(),
                               currentHROVMessageV2->GetMsgSize());

  std::thread guidedWorker([&]() {
    tf::TransformListener listener;
    tf::StampedTransform nedMerov;
    tf::Transform rovMtarget;
    tf::Vector3 nedTerov;
    tf::Quaternion nedRerov;
    merbots_whrov_msgs::debug debugMsg;

    double tyaw, cyaw, croll, cpitch;
    bool firstPIDIteration = true;
    while (1) {
      try {
        listener.lookupTransform("local_origin_ned", "erov", ros::Time(0),
                                 nedMerov);
      } catch (tf::TransformException &ex) {
        Log->Warn("TF: {}", ex.what());
        control->Arm(false);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        continue;
      }

      ned_mutex.lock();

      ned_x = nedMerov.getOrigin().getX();
      ned_y = nedMerov.getOrigin().getY();
      ned_z = nedMerov.getOrigin().getZ();
      nedMerov.getBasis().getRPY(croll, cpitch, cyaw);
      g_yaw = cyaw;
      auto heading = telerobotics::utils::GetDiscreteYaw(g_yaw);
      ned_cond.notify_all();
      ned_mutex.unlock();

      currentHROVMessage_mutex.lock();
      currentHROVMessageV2->SetX(nedMerov.getOrigin().getX() * 100);
      currentHROVMessageV2->SetY(nedMerov.getOrigin().getY() * 100);
      currentHROVMessageV2->SetZ(nedMerov.getOrigin().getZ() * 100);
      currentHROVMessageV2->SetHeading(heading);
      currentHROVMessage_updated = true;
      currentHROVMessage_mutex.unlock();
      currentHROVMessage_cond.notify_one();

      if (guidedMode && goToMission) {
        if (firstPIDIteration)
          ResetPID();
        firstPIDIteration = false;
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

        double baseZ = -38; //-38;
        double newZ = vz + baseZ;
        auto x = ceil(ArduSubXYR(vx));
        auto y = ceil(ArduSubXYR(vy));
        auto z = ceil(ArduSubZ(newZ));
        auto r = ceil(ArduSubXYR(rv0));

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
        firstPIDIteration = true;
      }
      std::this_thread::sleep_for(chrono::milliseconds(100));
    }
  });
  guidedWorker.detach();

  control->Start();
  try {
    ros::Rate rate(20); // 20 hz

    while (ros::ok()) {
      ;
      ros::spinOnce();
      rate.sleep();
    }
  } catch (std::exception &e) {
    Log->Error("Exception: {}", e.what());
    exit(1);
  }
  return 0;
}
