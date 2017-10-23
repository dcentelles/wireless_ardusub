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
#include <ros/ros.h>
#include <wireless_ardusub/HROVMessage.h>
#include <wireless_ardusub/OperatorMessageV2.h>
#include <wireless_ardusub/nodes/ROV.h>

using namespace cpplogging;
using namespace std::chrono_literals;
using namespace wireless_ardusub;
using namespace mavlink_cpp;
using namespace std;

struct Params {
  std::string serialPort, masterUri;
  bool log2Console;
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

static dccomms::Ptr<HROVMessage> currentHROVMessage =
    dccomms::CreateObject<HROVMessage>();
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
static bool keepOrientationReceived;
static std::mutex keepOrientation_mutex;
static std::condition_variable keepOrientation_cond;

static uint16_t holdChannelSeconds;
static bool holdChannelReceived;
static std::mutex holdChannel_mutex;
static std::condition_variable holdChannel_cond;

static bool cancelLastOrder;
static std::mutex executingOrder_mutex;

void notifyHROVMessageUpdated() {
  currentHROVMessage_updated = true;
  currentHROVMessage_cond.notify_one();
}
void notifyLastOrderCancellation() {
  currentHROVMessage_mutex.lock();
  currentHROVMessage->LastOrderCancelledFlag(true);
  currentHROVMessage->Ready(true);
  notifyHROVMessageUpdated();
  currentHROVMessage_mutex.unlock();
}
void notifyROVBusy() {
  currentHROVMessage_mutex.lock();
  currentHROVMessage->Ready(false);
  notifyHROVMessageUpdated();
  currentHROVMessage_mutex.unlock();
}

void notifyROVReady() {
  currentHROVMessage_mutex.lock();
  currentHROVMessage->Ready(true);
  notifyHROVMessageUpdated();
  currentHROVMessage_mutex.unlock();
}

void mockOrderWork() {
  notifyROVBusy();

  // work step 0
  this_thread::sleep_for(chrono::milliseconds(2000));
  if (cancelLastOrder) {
    notifyLastOrderCancellation();
    return;
  }

  // work step 1
  this_thread::sleep_for(chrono::milliseconds(2000));
  if (cancelLastOrder) {
    notifyLastOrderCancellation();
    return;
  }

  // work step 2
  this_thread::sleep_for(chrono::milliseconds(2000));
  if (cancelLastOrder) {
    notifyLastOrderCancellation();
    return;
  }

  // work step 3
  this_thread::sleep_for(chrono::milliseconds(2000));

  notifyROVReady();
}

void keepOrientationWork() {
  while (1) {
    std::unique_lock<std::mutex> lock(keepOrientation_mutex);
    while (!keepOrientationReceived)
      keepOrientation_cond.wait(lock);
    keepOrientationReceived = false;
    executingOrder_mutex.lock();
    mockOrderWork();
    cancelLastOrder = false;
    executingOrder_mutex.unlock();
  }
}

void holdChannelWork() {
  while (1) {
    std::unique_lock<std::mutex> lock(holdChannel_mutex);
    while (!holdChannelReceived)
      holdChannel_cond.wait(lock);
    holdChannelReceived = false;
    executingOrder_mutex.lock();
    mockOrderWork();
    cancelLastOrder = false;
    executingOrder_mutex.unlock();
  }
}

void handleNewOrder() {
  currentHROVMessage_mutex.lock();
  auto orderType = currentOperatorMessage->GetOrderType();
  auto eSeq = currentHROVMessage->GetExpectedOrderSeqNumber();
  auto cSeq = currentOperatorMessage->GetOrderSeqNumber();
  if (eSeq == cSeq) {
    if (currentOperatorMessage->CancelLastOrderFlag()) {
      cancelLastOrder = true;
    } else if (!orderType == OperatorMessageV2::OrderType::NoOrder) {
      cancelLastOrder = true;
      switch (orderType) {
      case OperatorMessageV2::OrderType::HoldChannel: {
        holdChannel_mutex.lock();
        holdChannelSeconds = currentOperatorMessage->GetHoldChannelDuration();
        Log->Info("Received hold channel order: {} seconds",
                  holdChannelSeconds);
        holdChannelSeconds = true;
        holdChannel_mutex.unlock();
        holdChannel_cond.notify_one();
        break;
      }
      case OperatorMessageV2::OrderType::KeepOrientation: {
        keepOrientation_mutex.lock();
        desiredOrientation = currentOperatorMessage->GetKeepOrientationOrder();
        Log->Info("Received keep orientation order: {} degrees",
                  desiredOrientation);
        keepOrientationReceived = true;
        keepOrientation_mutex.unlock();
        keepOrientation_cond.notify_one();
        break;
      }
      }
      currentHROVMessage->IncExpectedOrderSeqNumber();
    }
    notifyHROVMessageUpdated();
    currentHROVMessage_mutex.unlock();
  }
}
void messageSenderWork() {
  while (1) {
    std::unique_lock<std::mutex> lock(currentHROVMessage_mutex);
    while (!currentHROVMessage_updated) {
      currentHROVMessage_cond.wait(lock);
    }
    commsNode->SetCurrentTxState(currentHROVMessage->GetBuffer());
    currentHROVMessage_updated = false;
  }
}
void operatorMsgParserWork() {
  while (1) {
    std::unique_lock<std::mutex> lock(currentOperatorMessage_mutex);
    while (!currentOperatorMessage_updated) {
      currentOperatorMessage_cond.wait(lock);
    }
    currentOperatorMessage_updated = false;

    auto lastSettings = currentOperatorMessage->GetSettingsCopy();

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

    OperatorMessageV2::OrderType lastOrderType =
        currentOperatorMessage->GetOrderType();
    if (lastOrderType == OperatorMessageV2::OrderType::Move) {
      auto lastOrder = currentOperatorMessage->GetMoveOrderCopy();
      Log->Info("Last orders:\n"
                "\tx: {}\n"
                "\ty: {}\n"
                "\tz: {}\n"
                "\tr: {}\n",
                lastOrder->GetX(), lastOrder->GetY(), lastOrder->GetZ(),
                lastOrder->GetR());
      break;
    } else {
      handleNewOrder();
    }
  }
}

void startWorkers() {
  operatorMsgParserWorker = std::thread(operatorMsgParserWork);
  messageSenderWorker = std::thread(messageSenderWork);
  holdChannelWorker = std::thread(holdChannelWork);
  keepOrientationWorker = std::thread(keepOrientationWork);
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

void initROSInterface(int argc, char **argv) {
  ros::NodeHandle nh;
  encodedImage_sub = nh.subscribe<image_utils_ros_msgs::EncodedImg>(
      "encoded_image", 1, boost::bind(handleNewImage, _1));
  encodingConfig_pub =
      nh.advertise<image_utils_ros_msgs::EncodingConfig>("encoding_config", 1);
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
  control->Start();
  control->LogToConsole(params.log2Console);

  Log->SetLogLevel(LogLevel::info);

  startWorkers();

  auto stream = dccomms::CreateObject<dccomms_utils::S100Stream>(
      params.serialPort, SerialPortStream::BAUD_2400);
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
  });

  commsNode->LogToConsole(params.log2Console);
  commsNode->SetLogLevel(LogLevel::info);
  commsNode->SetRxStateSize(OperatorMessageV2::MessageLength);
  commsNode->SetTxStateSize(HROVMessage::MessageLength);
  commsNode->SetMaxImageTrunkLength(50);
  commsNode->Start();

  commsNode->SetCurrentTxState(currentHROVMessage->GetBuffer());

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
