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
#include <tf/transform_listener.h>
#include <wireless_ardusub/Constants.h>
#include <wireless_ardusub/utils.hpp>

using namespace cpplogging;
using namespace std::chrono_literals;
using namespace telerobotics;
using namespace mavlink_cpp;
using namespace std;
using namespace telerobotics;

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
static dccomms::Ptr<ROV> commsNode;

static image_utils_ros_msgs::EncodingConfig emsg;
static int lastImageSize = -1;

static uint16_t desiredOrientation;
static bool keepOrientation;
static std::mutex keepOrientation_mutex;
static std::condition_variable keepOrientation_cond;

static uint16_t holdChannelSeconds;
static bool holdChannelReceived;
static std::mutex holdChannel_mutex;
static std::condition_variable holdChannel_cond;

static bool cancelLastOrderFlag;

static bool executingOrder;
static std::mutex executingOrder_mutex;
static std::condition_variable executingOrder_cond;

static HROVPose currentHROVPose;
static std::mutex currentHROVPose_mutex;
static std::condition_variable currentHROVPose_cond;
static bool currentHROVPose_updated;
static bool home_set;

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

double getJoyAxisNormalized(int x) { return 200. / 256 * x; }
double arduSubXYR(double per) { return per / 0.1; }
double arduSubZ(double per) { return (per + 100) / 0.2; }

void stopRobot() {
  int x = ceil(arduSubXYR(0));
  int y = ceil(arduSubXYR(0));
  int z = ceil(arduSubZ(0));
  int r = ceil(arduSubXYR(0));
  control->SetManualControl(x, y, z, r);
}

void moveYaw(double per) {
  if (lastOrder) {
    rVel = ceil(arduSubXYR(per));
    control->SetManualControl(xVel, yVel, zVel, rVel);
  }
}

void mockOrderWork() {
  notifyROVBusy();

  // work step 0
  this_thread::sleep_for(chrono::milliseconds(2000));
  if (cancelLastOrderFlag) {
    notifyLastOrderCancellationAndRobotReady();
    return;
  }

  // work step 1
  this_thread::sleep_for(chrono::milliseconds(2000));
  if (cancelLastOrderFlag) {
    notifyLastOrderCancellationAndRobotReady();
    return;
  }

  // work step 2
  this_thread::sleep_for(chrono::milliseconds(2000));
  if (cancelLastOrderFlag) {
    notifyLastOrderCancellationAndRobotReady();
    return;
  }

  // work step 3
  this_thread::sleep_for(chrono::milliseconds(2000));

  notifyROVReady();
}

void holdChannelWork() {
  notifyROVBusy();
  int seconds = holdChannelSeconds + 2;
  commsNode->HoldChannel(true);
  this_thread::sleep_for(chrono::seconds(seconds));
  commsNode->HoldChannel(false);
  notifyROVReady();
}

/**
   Length (angular) of a shortest way between two angles.
  It will be in range [0, 180].

 private int distance(int alpha, int beta) {
     int phi = Math.abs(beta - alpha) % 360;       // This is either the
 distance or 360 - distance
     int distance = phi > 180 ? 360 - phi : phi;
     return distance;
 }
*/

int angleDistance(int alpha, int beta) {
  int phi = std::abs(beta - alpha) %
            360; // This is either the distance or 360 - distance
  int distance = phi > 180 ? 360 - phi : phi;
  return distance;
}

double getKeepHeadingDecrease(int ahdiff) {
  uint32_t diff = abs(ahdiff);
  double m = diff / 180. * 100;
  return m;
}

void keepHeadingIteration(void) {
  int currentHeading = std::round(currentHROVPose.heading);
  int ahdiff = angleDistance(currentHeading, desiredOrientation);

  bool right;
  if (ahdiff + currentHeading % 360 == desiredOrientation)
    right = false;
  else
    right = true;

  double vel = getKeepHeadingDecrease(ahdiff);
  if (ahdiff > 1) {
    if (right) {
      Log->Debug("Turn left");
      moveYaw(-vel);
    } else {
      Log->Debug("Turn right");
      moveYaw(vel);
    }
  } else {
    Log->Debug("do not turn");
    moveYaw(0);
  }
}

void keepHeadingWorkLoop() {
  bool keepingHeading = false;
  while (1) {
    std::unique_lock<std::mutex> lock(keepOrientation_mutex);
    while (!keepOrientation) {
      currentHROVMessage_mutex.lock();
      keepingHeading = false;
      currentHROVMessageV2->KeepingHeadingFlag(false);
      currentHROVMessage_mutex.unlock();
      currentHROVMessage_cond.notify_one();
      Log->Debug("Keep orientation disabled");
      keepOrientation_cond.wait(lock);
      Log->Debug("Keep orientation received!");
    }
    if (!keepingHeading) {
      currentHROVMessage_mutex.lock();
      keepingHeading = true;
      currentHROVMessageV2->KeepingHeadingFlag(true);
      currentHROVMessage_mutex.unlock();
      currentHROVMessage_cond.notify_one();
    }

    keepHeadingIteration();
    this_thread::sleep_for(chrono::milliseconds(200));
  }
}

void holdChannelWorkLoop() {
  while (1) {
    Log->Debug("Waiting hold channel order");
    std::unique_lock<std::mutex> lock(holdChannel_mutex);
    while (!holdChannelReceived)
      holdChannel_cond.wait(lock);
    Log->Debug("Hold channel received");
    holdChannelReceived = false;
    holdChannelWork();
    Log->Debug("Hold channel finished");
  }
}

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
        // auto goTo = currentOperatorMessage->GetGoToOrder();
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
void messageSenderWork() {
  while (1) {
    std::unique_lock<std::mutex> lock(currentHROVMessage_mutex);
    while (!currentHROVMessage_updated) {
      currentHROVMessage_cond.wait(lock);
    }
    commsNode->SetCurrentTxState(currentHROVMessageV2->GetBuffer(),
                                 currentHROVMessageV2->GetMsgSize());
    currentHROVMessage_updated = false;
  }
}

void operatorMsgParserWork() {
  while (1) {
    std::unique_lock<std::mutex> lock(currentOperatorMessage_mutex);
    while (!currentOperatorMessage_updated) {
      currentOperatorMessage_cond.wait_for(lock, chrono::milliseconds(2200));
      if (!currentOperatorMessage_updated && !commsNode->HoldingChannel()) {
        stopRobot();
        control->Arm(false);
        Log->Warn(
            "Heartbeat lost. Stopping robot to avoid thruster interferences!");
      }
    }
    currentOperatorMessage_updated = false;

    OperatorMessageV2::OrderType lastOrderType =
        currentOperatorMessage->GetOrderType();
    if (lastOrderType == OperatorMessageV2::OrderType::Move) {
      lastOrder = currentOperatorMessage->GetMoveOrderCopy();

      std::string modeName = "";
      switch (lastOrder->GetFlyMode()) {
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
        control->SetFlyMode(FLY_MODE_R::GUIDED);
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

      xVel = ceil(arduSubXYR(xNorm));
      yVel = ceil(arduSubXYR(yNorm));
      zVel = ceil(arduSubZ(zNorm));
      if (!keepOrientation)
        rVel = ceil(arduSubXYR(rNorm));

      //      Log->Info(
      //          "Manual control: X: {} ; Y: {} ; Z: {} ; R: {} ; Arm: {} ;
      //          Mode: {}",
      //          xVel, yVel, zVel, rVel, lastOrder->Arm() ? "true" : "false",
      //          modeName);

      control->SetManualControl(xVel, yVel, zVel, rVel);
      control->Arm(lastOrder->Arm());
    } else {
      handleNewOrder();
    }
  }
}

void startWorkers() {
  operatorMsgParserWorker = std::thread(operatorMsgParserWork);
  messageSenderWorker = std::thread(messageSenderWork);
  holdChannelWorker = std::thread(holdChannelWorkLoop);
  keepOrientationWorker = std::thread(keepHeadingWorkLoop);
}

void handleNewImage(image_utils_ros_msgs::EncodedImgConstPtr msg) {
  if (!commsNode->SendingCurrentImage()) {
    lastImageSize =
        emsg.max_size <= msg->img.size() ? emsg.max_size : msg->img.size();
    if (lastImageSize > 0) {
      Log->Info("Sending the new captured image... ({} bytes)", lastImageSize);
      commsNode->SendImage((void *)msg->img.data(), lastImageSize);
    }
  } else {
    if (lastImageSize != emsg.max_size) {
      commsNode->CancelLastImage();
    }
  }
}

void handleNewHUDData(const mavlink_vfr_hud_t &msg) {
  currentHROVMessage_mutex.lock();
  currentHROVMessageV2->SetHeading(msg.heading);
  currentHROVMessageV2->SetZ(msg.alt * 100);
  currentHROVMessage_updated = true;
  currentHROVMessage_mutex.unlock();
  currentHROVMessage_cond.notify_one();

  currentHROVPose_mutex.lock();
  currentHROVPose.heading = msg.heading;
  currentHROVPose.Z = msg.alt;
  currentHROVPose_updated = true;
  currentHROVPose_mutex.unlock();
  currentHROVPose_cond.notify_one();
}

void handleNewNavigationData(const mavlink_attitude_t &attitude) {
  tfScalar yaw, pitch, roll;
  yaw = attitude.yaw;
  pitch = attitude.pitch;
  roll = attitude.roll;

  Log->Debug("yaw: {} ; pitch: {} ; roll: {}", yaw, pitch, roll);
  int rx, ry, rz;
  rx = telerobotics::utils::GetDiscreteYaw(roll);
  ry = telerobotics::utils::GetDiscreteYaw(pitch);
  rz = telerobotics::utils::GetDiscreteYaw(yaw);

  rx = rx > 180 ? -(360 - rx) : rx;
  ry = ry > 180 ? -(360 - ry) : ry;
  // rz = rz > 180 ? -(360-rz) : rz;

  currentHROVMessage_mutex.lock();
  // currentHROVMessage->SetYaw (rz);
  currentHROVMessageV2->SetRoll(rx);
  currentHROVMessageV2->SetPitch(ry);
  currentHROVMessage_updated = true;
  currentHROVMessage_mutex.unlock();
  currentHROVMessage_cond.notify_one();

  currentHROVPose_mutex.lock();
  // currentHROVPose.yaw = 0;
  currentHROVPose_updated = true;
  currentHROVPose_mutex.unlock();
  currentHROVPose_cond.notify_one();
}

void handleNewArdusubState(const mavlink_heartbeat_t &msg) {
  ARDUSUB_NAV_MODE mode;
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
  bool armed = msg.base_mode & MAV_MODE_FLAG_SAFETY_ARMED;

  currentHROVMessage_mutex.lock();
  currentHROVMessageV2->SetNavMode(mode);
  currentHROVMessageV2->Armed(armed);
  currentHROVMessage_updated = true;
  currentHROVMessage_mutex.unlock();
  currentHROVMessage_cond.notify_one();
}

void initROSInterface(int argc, char **argv) {
  ros::NodeHandle nh;
  encodedImage_sub = nh.subscribe<image_utils_ros_msgs::EncodedImg>(
      "encoded_image", 1, boost::bind(handleNewImage, _1));
  encodingConfig_pub =
      nh.advertise<image_utils_ros_msgs::EncodingConfig>("encoding_config", 1);

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
int main(int argc, char **argv) {
  Log = CreateLogger("TeleopROV");
  Log->Info("Init");
  Log->SetAsyncMode(true);

  //// GET PARAMS
  ros::init(argc, argv, "rov_control");

  if (getParams())
    return 1;

  Log->SetLogLevel(cpplogging::LogLevel::info);
  Log->FlushLogOn(cpplogging::LogLevel::info);
  Log->LogToConsole(params.log2Console);

  control->SetLogName("GCS");
  control->SetLogLevel(info);
  control->Arm(false);
  control->LogToConsole(params.log2Console);
  control->Start();

  startWorkers();
  commsNode = dccomms::CreateObject<ROV>();

  commsNode->SetImageTrunkLength(DEFAULT_IMG_TRUNK_LENGTH);

  if (params.log2File) {
    Log->LogToFile("rov_v3_main");
    control->LogToFile("rov_v3_control");
    commsNode->LogToFile("rov_v3_comms_node");
  }

  if (params.serialPort != "service") {
    Log->Info("CommsDevice type: dccomms_utils::S100Stream");
    dccomms::Ptr<CommsDevice> stream =
        dccomms::CreateObject<dccomms_utils::S100Stream>(
            params.serialPort, SerialPortStream::BAUD_2400, S100_MAX_BITRATE);
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

    Log->Info("Orders updated");
  });
  commsNode->Start();

  currentHROVMessageV2->Ready(true);
  commsNode->SetCurrentTxState(currentHROVMessageV2->GetBuffer(),
                               currentHROVMessageV2->GetMsgSize());

  ///// GPS
  Eigen::Vector3d map_origin; //!< geodetic origin [lla]

  // Constructor for a ellipsoid
  GeographicLib::Geocentric earth(GeographicLib::Constants::WGS84_a(),
                                  GeographicLib::Constants::WGS84_f());

  GeographicLib::LocalCartesian localNED;

  std::shared_ptr<GeographicLib::Geoid> egm96_5;
  egm96_5 = std::make_shared<GeographicLib::Geoid>("egm96-5", "", true, true);

  control->SetHomeUpdatedCb([&](const mavlink_home_position_t &msg) {

    map_origin = {msg.latitude / 1e7, msg.longitude / 1e7, msg.altitude / 1e3};

    localNED = GeographicLib::LocalCartesian(map_origin.x(), map_origin.y(),
                                             map_origin.z(), earth);
    home_set = true;
  });

  control->SetLocalPositionNEDCb([&](const mavlink_local_position_ned_t &msg) {
    Log->Info("LOCAL_POSITION_NED:"
              "\ttime_boot_ms: {}\n"
              "\tx: {}\n"
              "\ty: {}\n"
              "\tz: {}\n"
              "\tvx: {}\n"
              "\tvy: {}\n"
              "\tvz: {}\n",
              msg.time_boot_ms, msg.x, msg.y, msg.z, msg.vx, msg.vy, msg.vz);
    currentHROVMessage_mutex.lock();
    currentHROVMessageV2->SetX(msg.x * 100);
    currentHROVMessageV2->SetY(msg.y * 100);
    currentHROVMessage_updated = true;
    currentHROVMessage_mutex.unlock();
    currentHROVMessage_cond.notify_one();

  });

  control->SetGlobalPositionInt([&](const mavlink_global_position_int_t &msg) {
    double ecef_x, ecef_y, ecef_z, ned_x, ned_y, ned_z;
    int32_t latitude, longitude;
    auto alt =
        (msg.alt / (double)1e3) -
        GeographicLib::Geoid::ELLIPSOIDTOGEOID * (*egm96_5)(msg.lat, msg.lon);

    earth.Forward(msg.lat / 1e7, msg.lon / 1e7, alt, ecef_x, ecef_y, ecef_z);

    if (home_set) {

      localNED.Forward(msg.lat / 1e7, msg.lon / 1e7, (msg.alt / (double)1e3),
                       ned_x, ned_y, ned_z);
      Log->Info("lat,lon,alt: {} : {} : {} ---- NED x,y,z {} : {} : {}",
                msg.lat, msg.lon, msg.alt, ned_x, ned_y, ned_z);

      latitude = msg.lat;
      longitude = msg.lon;

      //      currentHROVMessage_mutex.lock();
      //      currentHROVMessageV2->SetX(ned_x);
      //      currentHROVMessageV2->SetY(ned_y);
      //      currentHROVMessage_updated = true;
      //      currentHROVMessage_mutex.unlock();
      //      currentHROVMessage_cond.notify_one();
    }

    //        Log->Info("lat,lon,alt: {} : {} : {} ---- NED x,y,z {} : {} : {}
    //        ----
    //        ECEF "
    //                  "x,y,z {} : {} : {}",
    //                  msg.lat, msg.lon, msg.alt, ned_x, ned_y, ned_z, ecef_x,
    //                  ecef_y,
    //                  ecef_z);

    //    Log->Info("lat,lon,alt: {} : {} : {} ---- NED x,y,z {} : {} : {}",
    //    msg.lat,
    //              msg.lon, msg.alt, ned_x, ned_y, ned_z);

  });
  try {

    ros::Rate rate(30); // 30 hz
    initROSInterface(argc, argv);

    while (ros::ok()) {
      ros::spinOnce();
      rate.sleep();
    }
  } catch (std::exception &e) {
    Log->Error("Exception: {}", e.what());
    exit(1);
  }
  return 0;
}
