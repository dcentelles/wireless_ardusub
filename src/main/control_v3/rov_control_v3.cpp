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
#include <wireless_ardusub/HROVMessage.h>
#include <wireless_ardusub/OperatorMessageV2.h>
#include <wireless_ardusub/nodes/ROV.h>

#include <ros/ros.h>

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

static auto operatorMessage = OperatorMessageV2::Build();

static uint16_t localPort = 14550;
static mavlink_cpp::Ptr<GCS> control =
    mavlink_cpp::CreateObject<GCS>(localPort);

static bool currentOperatorMessage_updated;
static std::mutex currentOperatorMessage_mutex;
static std::condition_variable currentOperatorMessage_cond;

static std::thread operatorMsgParserWorker;
static ros::Subscriber encodedImage_sub;
static ros::Publisher encodingConfig_pub;
static dccomms::Ptr<ROV> commsNode;

static image_utils_ros_msgs::EncodingConfig emsg;
static int lastImageSize = -1;

void operatorMsgParserWork() {
  while (1) {
    std::unique_lock<std::mutex> lock(currentOperatorMessage_mutex);
    while (!currentOperatorMessage_updated) {
      currentOperatorMessage_cond.wait(lock);
    }

    OperatorMessageV2::OrderType lastOrderType =
        operatorMessage->GetOrderType();
    switch (lastOrderType) {
    case OperatorMessageV2::OrderType::Move: {
      auto lastOrder = operatorMessage->GetMoveOrderCopy();
      Log->Info("Last orders:\n"
                "\tx: {}\n"
                "\ty: {}\n"
                "\tz: {}\n"
                "\tr: {}\n",
                lastOrder->GetX(), lastOrder->GetY(), lastOrder->GetZ(),
                lastOrder->GetR());
      break;
    }
    case OperatorMessageV2::OrderType::HoldChannel: {
      auto duration = operatorMessage->GetHoldChannelDuration();
      Log->Info("Received hold channel order: {} seconds", duration);
      break;
    }
    }

    auto lastSettings = operatorMessage->GetSettingsCopy();

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

    currentOperatorMessage_updated = false;
  }
}

void startOperatorMsgParserWorker() {
  operatorMsgParserWorker = std::thread(operatorMsgParserWork);
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

  startOperatorMsgParserWorker();

  auto stream = dccomms::CreateObject<dccomms_utils::S100Stream>(
      params.serialPort, SerialPortStream::BAUD_2400);
  stream->Open();

  commsNode = dccomms::CreateObject<ROV>(stream);
  commsNode->SetOrdersReceivedCallback([](ROV &receiver) {
    Log->Info("Orders received!");
    uint8_t state[OperatorMessageV2::MessageLength];
    receiver.GetCurrentRxState(state);

    currentOperatorMessage_mutex.lock();
    operatorMessage->UpdateFromBuffer(state);
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

  auto cstate = dccomms::CreateObject<HROVMessage>();
  commsNode->SetCurrentTxState(cstate->GetBuffer());

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
