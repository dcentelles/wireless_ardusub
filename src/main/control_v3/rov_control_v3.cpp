/*
 * Adaptation of:
 *  File: bluerov_apps/src/teleop_joy.cpp
 *  Author: Josh Villbrandt <josh@javconcepts.com>
 *  Date: February 2016
 *  Description: Manual remote control of ROVs like the bluerov_apps.
 * By centelld@uji.es for the wireless bluerov project
 */

#include <chrono>
#include <cpplogging/cpplogging.h>
#include <dccomms_utils/S100Stream.h>
#include <image_utils_ros_msgs/EncodedImg.h>
#include <image_utils_ros_msgs/EncodingConfig.h>
#include <mavlink_cpp/mavlink_cpp.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/VFR_HUD.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_listener.h>
#include <wireless_ardusub/Constants.h>
#include <wireless_ardusub/HROVMessageV2.h>
#include <wireless_ardusub/OperatorMessageV2.h>
#include <wireless_ardusub/nodes/ROV.h>
#include <wireless_ardusub/utils.hpp>

using namespace cpplogging;
using namespace std::chrono_literals;
using namespace wireless_ardusub;
using namespace mavlink_cpp;
using namespace std;

struct Params {
  std::string serialPort, masterUri;
  bool log2Console;
};

struct HROVPose {
  double yaw, pitch, roll, Z, X, Y, heading;
};

static LoggerPtr Log;
static Params params;

static uint16_t localPort = 14550;
static mavlink_cpp::Ptr<GCS> control =
    mavlink_cpp::CreateObject<GCS>(localPort);

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

static ros::Subscriber ardusubState_sub;

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

static bool cancelLastOrder;

static bool executingOrder;
static std::mutex executingOrder_mutex;
static std::condition_variable executingOrder_cond;

static ros::Subscriber ardusubNav_sub;
static ros::Subscriber ardusubHUD_sub;

static HROVPose currentHROVPose;
static std::mutex currentHROVPose_mutex;
static std::condition_variable currentHROVPose_cond;
static bool currentHROVPose_updated;

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
  if (cancelLastOrder) {
    notifyLastOrderCancellationAndRobotReady();
    return;
  }

  // work step 1
  this_thread::sleep_for(chrono::milliseconds(2000));
  if (cancelLastOrder) {
    notifyLastOrderCancellationAndRobotReady();
    return;
  }

  // work step 2
  this_thread::sleep_for(chrono::milliseconds(2000));
  if (cancelLastOrder) {
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
  double m = 30. / 180;
  return diff * m;
}

void keepHeadingIteration(void) {
  int currentHeading = std::round(currentHROVPose.heading);
  int ahdiff = angleDistance(currentHeading, desiredOrientation);

  bool right;
  if (ahdiff + currentHeading % 360 == desiredOrientation)
    right = false;
  else
    right = true;

  double vel = getKeepHeadingDecrease(30);
  if (ahdiff > 1) {
    if (right) {
      Log->Info("Turn left");
      moveYaw(-vel);
    } else {
      Log->Info("Turn right");
      moveYaw(vel);
    }
  } else {
    Log->Info("do not turn");
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

void CancelLastOrder() {
  std::unique_lock<std::mutex> lock(executingOrder_mutex);
  while (executingOrder) {
    cancelLastOrder = true;
    executingOrder_cond.wait(lock);
  }
  cancelLastOrder = false;
  lock.unlock();
}

void handleNewOrder() {
  currentHROVMessage_mutex.lock();
  auto eSeq = currentHROVMessageV2->GetExpectedOrderSeqNumber();
  currentHROVMessage_mutex.unlock();

  auto orderType = currentOperatorMessage->GetOrderType();
  auto cSeq = currentOperatorMessage->GetOrderSeqNumber();
  auto cancelLastOrder = currentOperatorMessage->CancelLastOrderFlag();

  if (eSeq == cSeq) {
    if (cancelLastOrder) {
      Log->Debug("Cancelation requested");
      CancelLastOrder();
      Log->Debug("Cancelled");
    } else if (orderType != OperatorMessageV2::OrderType::NoOrder) {
      Log->Debug("New order requested. Cancel current order (if any)");
      CancelLastOrder();
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
    commsNode->SetCurrentTxState(currentHROVMessageV2->GetBuffer());
    currentHROVMessage_updated = false;
  }
}

void operatorMsgParserWork() {
  while (1) {
    std::unique_lock<std::mutex> lock(currentOperatorMessage_mutex);
    while (!currentOperatorMessage_updated) {
      currentOperatorMessage_cond.wait_for(lock, chrono::milliseconds(2000));
      stopRobot();
    }
    currentOperatorMessage_updated = false;

    OperatorMessageV2::OrderType lastOrderType =
        currentOperatorMessage->GetOrderType();
    if (lastOrderType == OperatorMessageV2::OrderType::Move) {
      lastOrder = currentOperatorMessage->GetMoveOrderCopy();

      std::string modeName = "";
      switch (lastOrder->GetFlyMode()) {
      case FLY_MODE::DEPTH_HOLD:
        modeName = "DEPTH HOLD";
        control->SetDepthHoldMode();
        break;
      case FLY_MODE::STABILIZE:
        modeName = "STABILIZE";
        control->SetStabilizeMode();
        break;
      case FLY_MODE::MANUAL:
        modeName = "MANUAL";
        control->SetManualMode();
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

      Log->Info(
          "Manual control: X: {} ; Y: {} ; Z: {} ; R: {} ; Arm: {} ; Mode: {}",
          xVel, yVel, zVel, rVel, lastOrder->Arm() ? "true" : "false",
          modeName);

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
    Log->Info("Sending the new captured image... ({} bytes)", msg->img.size());
    lastImageSize =
        emsg.max_size <= msg->img.size() ? emsg.max_size : msg->img.size();
    commsNode->SendImage((void *)msg->img.data(), lastImageSize);
  } else {
    if (lastImageSize != emsg.max_size) {
      commsNode->CancelLastImage();
    }
  }
}

void HandleNewHUDData(const mavros_msgs::VFR_HUD::ConstPtr &msg) {
  currentHROVMessage_mutex.lock();
  currentHROVMessageV2->SetHeading(msg->heading);
  currentHROVMessage_updated = true;
  currentHROVMessage_mutex.unlock();
  currentHROVMessage_cond.notify_one();

  currentHROVPose_mutex.lock();
  currentHROVPose.heading = msg->heading;
  currentHROVPose_updated = true;
  currentHROVPose_mutex.unlock();
  currentHROVPose_cond.notify_one();
}

void HandleNewNavigationData(const sensor_msgs::Imu::ConstPtr &msg) {
  // the following is not correct! x, y z is the components of a Quaternion, not
  // the Euler angles.

  tf::Quaternion rotation(msg->orientation.x, msg->orientation.y,
                          msg->orientation.z, msg->orientation.w);

  tf::Matrix3x3 rotMat(rotation);

  tfScalar yaw, pitch, roll;
  rotMat.getEulerYPR(yaw, pitch, roll);

  int rx, ry, rz;
  rx = wireless_ardusub::utils::GetDiscreteYaw(roll);
  ry = wireless_ardusub::utils::GetDiscreteYaw(pitch);
  rz = wireless_ardusub::utils::GetDiscreteYaw(yaw);

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

void HandleNewArdusubState(const mavros_msgs::State::ConstPtr &msg) {
  ARDUSUB_NAV_MODE mode;
  if (msg->mode == "MANUAL") {
    mode = ARDUSUB_NAV_MODE::NAV_MANUAL;
  } else if (msg->mode == "STABILIZE") {
    mode = ARDUSUB_NAV_MODE::NAV_STABILIZE;
  } else if (msg->mode == "ALT_HOLD") {
    mode = ARDUSUB_NAV_MODE::NAV_DEPTH_HOLD;
  } else
    mode = ARDUSUB_NAV_MODE::NAV_UNKNOWN;

  bool armed = msg->armed;
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

  ardusubNav_sub = nh.subscribe<sensor_msgs::Imu>(
      "/mavros/imu/data", 1, boost::bind(HandleNewNavigationData, _1));

  ardusubHUD_sub = nh.subscribe<mavros_msgs::VFR_HUD>(
      "/mavros/vfr_hud", 1, boost::bind(HandleNewHUDData, _1));

  ardusubState_sub = nh.subscribe<mavros_msgs::State>(
      "/mavros/state", 1, boost::bind(HandleNewArdusubState, _1));
}

int GetParams() {
  ros::NodeHandle nh("~");
  std::string serialPort;
  if (!nh.getParam("port", serialPort)) {
    ROS_ERROR("Failed to get param port");
    return 1;
  } else {
    Log->Info("port topic: {}", serialPort);
  }
  params.serialPort = serialPort;

  char *cmasterUri = getenv("ROS_MASTER_URI");
  std::string masterUri = cmasterUri;
  Log->Info("ROS MASTER URI: {}", masterUri);

  params.masterUri = masterUri;

  bool log2Console;
  if (!nh.getParam("log2Console", log2Console)) {
    ROS_ERROR("Failed to get param log2Console");
    return 1;
  } else {
    Log->Info("log2Console: {}", log2Console);
  }

  params.log2Console = log2Console;

  return 0;
}
int main(int argc, char **argv) {
  Log = CreateLogger("TeleopROV");
  Log->Info("Init");

  //// GET PARAMS
  ros::init(argc, argv, "rov_control");

  if (GetParams())
    return 1;

  Log->SetLogLevel(cpplogging::LogLevel::debug);
  Log->FlushLogOn(cpplogging::LogLevel::info);
  Log->LogToConsole(params.log2Console);

  control->SetLogName("GCS");
  control->SetLogLevel(info);
  control->Arm(false);
  control->Start();
  control->LogToConsole(params.log2Console);

  Log->SetLogLevel(LogLevel::debug);

  startWorkers();

  auto stream = dccomms::CreateObject<dccomms_utils::S100Stream>(
      params.serialPort, SerialPortStream::BAUD_2400, S100_MAX_BITRATE);
  stream->Open();

  commsNode = dccomms::CreateObject<ROV>(stream);
  commsNode->SetOrdersReceivedCallback([](ROV &receiver) {
    Log->Info("Orders received!");
    uint8_t state[OperatorMessageV2::MessageLength];
    receiver.GetCurrentRxState(state);

    currentOperatorMessage_mutex.lock();
    currentOperatorMessage->UpdateFromBuffer(state);
    currentOperatorMessage_mutex.unlock();
    currentOperatorMessage_updated = true;
    currentOperatorMessage_cond.notify_one();

    Log->Info("Orders updated");
  });

  commsNode->LogToConsole(params.log2Console);
  commsNode->SetLogLevel(LogLevel::info);
  commsNode->SetRxStateSize(OperatorMessageV2::MessageLength);
  commsNode->SetTxStateSize(HROVMessageV2::MessageLength);
  commsNode->SetMaxImageTrunkLength(100);
  commsNode->Start();

  currentHROVMessageV2->Ready(true);
  commsNode->SetCurrentTxState(currentHROVMessageV2->GetBuffer());

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
