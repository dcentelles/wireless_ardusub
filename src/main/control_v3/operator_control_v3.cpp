/*
 * Adaptation of:
 *  File: bluerov_apps/src/teleop_joy.cpp
 *  Author: Josh Villbrandt <josh@javconcepts.com>
 *  Date: February 2016
 *  Description: Manual remote control of ROVs like the bluerov_apps.
 * By centelld@uji.es for the wireless bluerov project
 */

#include <cmath>
#include <cpplogging/cpplogging.h>
#include <dccomms_utils/S100Stream.h>
#include <dynamic_reconfigure/server.h>
#include <image_utils_ros_msgs/EncodedImg.h>
#include <merbots_whrov_msgs/hrov_settings.h>
#include <merbots_whrov_msgs/position.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <vector>
#include <wireless_ardusub/HROVMessage.h>
#include <wireless_ardusub/HROVSettingsV2.h>
#include <wireless_ardusub/TeleopOrder.h>
#include <wireless_ardusub/nodes/Constants.h>
#include <wireless_ardusub/nodes/Operator.h>
#include <wireless_ardusub/wireless_teleop_joyConfig.h>

using namespace cpplogging;
using namespace wireless_ardusub;

class Teleop : public Logger {
public:
  Teleop(Ptr<ICommsLink> stream);
  void spin();

private:
  // functions
  bool risingEdge(const sensor_msgs::Joy::ConstPtr &joy, int index);
  void setArming(bool armed);
  // void setMode(uint8_t mode);
  void cmdTakeoffLand(bool takeoff);
  int8_t computeAxisValue(const sensor_msgs::Joy::ConstPtr &joy, int index);
  uint16_t mapToPpm(double in);
  void configCallback(wireless_ardusub::wireless_teleop_joyConfig &update,
                      uint32_t level);
  void joyCallback(const sensor_msgs::Joy::ConstPtr &joy);
  void newSettingsReceived(
      const merbots_whrov_msgs::hrov_settingsConstPtr &settings);

  TeleopOrderPtr order;
  Ptr<HROVSettingsV2> settings;
  // state transmitter
  Ptr<Operator> sender;
  // node handle
  ros::NodeHandle nh;

  ros::Publisher currentHROVState_pub;
  ros::Publisher encodedImage_pub;
  ros::Subscriber currentSettings_sub;

  // dynamic reconfigure
  dynamic_reconfigure::Server<wireless_ardusub::wireless_teleop_joyConfig>
      server;
  wireless_ardusub::wireless_teleop_joyConfig config;

  // pubs and subs
  ros::Subscriber joy_sub;

  // constants
  enum { COMPONENT_ARM_DISARM = 400 }; // https://pixhawk.ethz.ch/mavlink/
  enum {
    NAV_LAND_LOCAL = 23,
    NAV_TAKEOFF_LOCAL = 24
  }; // https://pixhawk.ethz.ch/mavlink/
  enum {
    MODE_STABILIZE = 1000,
    MODE_ALT_HOLD = 2000
  };                                       // ppm in uS; from ArduSub/radio.cpp
  enum { PPS_MIN = 1000, PPS_MAX = 2000 }; // uS

  // state
  uint16_t camera_tilt;
  bool initLT;
  bool initRT;
  std::vector<int> previous_buttons;
  image_utils_ros_msgs::EncodedImg encodedImgMsg;
  uint8_t imgBuffer[wireless_ardusub::teleop_v3::MAX_IMG_SIZE];

  uint8_t state[TeleopOrder::Size + HROVSettingsV2::SettingsSize];
  uint8_t *orderPtr, *settingsPtr;
};

Teleop::Teleop(Ptr<ICommsLink> stream) {
  orderPtr = state;
  settingsPtr = orderPtr + TeleopOrder::Size;
  // connect dynamic reconfigure
  dynamic_reconfigure::Server<
      wireless_ardusub::wireless_teleop_joyConfig>::CallbackType f;
  f = boost::bind(&Teleop::configCallback, this, _1, _2);
  server.setCallback(f);

  // connects subs and pubs
  joy_sub =
      nh.subscribe<sensor_msgs::Joy>("/joy", 1, &Teleop::joyCallback, this);

  // initialize state variables
  camera_tilt = 1500;
  initLT = false;
  initRT = false;
  SetLogName("TeleopJoy");
  Log->info("Sender initialized");

  encodedImgMsg.img.reserve(wireless_ardusub::teleop_v3::MAX_IMG_SIZE);

  currentHROVState_pub =
      nh.advertise<merbots_whrov_msgs::position>("current_hrov_position", 1);

  encodedImage_pub =
      nh.advertise<image_utils_ros_msgs::EncodedImg>("encoded_image", 1);

  currentSettings_sub = nh.subscribe<merbots_whrov_msgs::hrov_settings>(
      "desired_hrov_settings", 1,
      boost::bind(&Teleop::newSettingsReceived, this, _1)); //,

  sender = CreateObject<Operator>(stream);
  sender->SetLogLevel(LogLevel::info);
  sender->SetMaxImageTrunkLength(50);
  sender->SetRxStateSize(HROVMessage::MessageLength);
  sender->SetTxStateSize(TeleopOrder::Size + HROVSettingsV2::SettingsSize);
  sender->SetImageReceivedCallback([this](Operator &op) {
    Log->info("New Image received!");
    int encodedImgSize;
    encodedImgSize = op.GetLastReceivedImage(imgBuffer);
    encodedImgMsg.img.resize(encodedImgSize);
    memcpy(encodedImgMsg.img.data(), imgBuffer, encodedImgSize);
    encodedImage_pub.publish(encodedImgMsg);
  });

  sender->SetStateReceivedCallback(
      [this](Operator &op) { Log->info("New state received!"); });

  settings = HROVSettingsV2::Build();
  order = TeleopOrder::Build();

  sender->Start();
}

void Teleop::spin() {
  ros::Rate loop(config.pub_rate);

  while (ros::ok()) {
    // call all waiting callbacks
    ros::spinOnce();

    // enforce a max publish rate
    loop.sleep();
  }
}

void Teleop::newSettingsReceived(
    const merbots_whrov_msgs::hrov_settingsConstPtr &msg) {
  Log->info("New settings received");
  settings->SetFromROSMsg(msg);
  memcpy(settingsPtr, settings->GetBuffer(), HROVSettingsV2::SettingsSize);
  sender->SetDesiredState(state);
}

void Teleop::configCallback(wireless_ardusub::wireless_teleop_joyConfig &update,
                            uint32_t level) {
  ROS_INFO("reconfigure request received");
  config = update;
}

bool Teleop::risingEdge(const sensor_msgs::Joy::ConstPtr &joy, int index) {
  return (joy->buttons[index] == 1 && previous_buttons[index] == 0);
}

void Teleop::setArming(bool arm) {
  // Arm/disarm method following:
  // https://github.com/mavlink/qgroundcontrol/issues/590
  // https://pixhawk.ethz.ch/mavlink/#MAV_CMD_COMPONENT_ARM_DISARM
  if (arm) {
    Debug("armed");
    order->Arm(true);
    order->DisArm(false);
  } else {
    Debug("disarmed");
    order->Arm(false);
    order->DisArm(true);
  }
}

void Teleop::cmdTakeoffLand(bool takeoff) {
  // https://pixhawk.ethz.ch/mavlink/#MAV_CMD_NAV_LAND_LOCAL
  if (takeoff) {
    Debug("takeoff");
  } else {
    Debug("land");
  }
}

int8_t Teleop::computeAxisValue(const sensor_msgs::Joy::ConstPtr &joy,
                                int index) {
  // return 0 if axis index is invalid
  if (index < 0 || index >= joy->axes.size()) {
    return 0.0;
  }

  double raw = joy->axes[index]; // raw in [-1,1]
  double dvalue = 127 * raw;
  int8_t value = ceil(dvalue);
  Log->debug("{}: Raw: {} ; DValue: {} ; Value: {}", index, raw, dvalue, value);
  return value;
}

void Teleop::joyCallback(const sensor_msgs::Joy::ConstPtr &joy) {
  // init previous_buttons
  if (previous_buttons.size() != joy->buttons.size()) {
    previous_buttons = std::vector<int>(joy->buttons);
  }

  // arm, disarm
  if (risingEdge(joy, config.disarm_button)) {
    setArming(false);
  } else if (risingEdge(joy, config.arm_button)) {
    setArming(true);
  }

  auto x = computeAxisValue(joy, config.x_axis);
  auto y = computeAxisValue(joy, config.y_axis);
  auto z = computeAxisValue(joy, config.z_axis);
  auto r = computeAxisValue(joy, config.wz_axis);

  order->SetX(x);
  order->SetY(-1 * y);
  order->SetZ(z);
  order->SetR(-1 * r);

  // mode switching
  if (risingEdge(joy, config.stabilize_button)) {
    order->SetFlyMode(FLY_MODE::STABILIZE);
  } else if (risingEdge(joy, config.alt_hold_button)) {
    order->SetFlyMode(FLY_MODE::DEPTH_HOLD);
  } else if (risingEdge(joy, 0)) {
    order->SetFlyMode(FLY_MODE::MANUAL);
  }
  std::string modeName = "";
  switch (order->GetFlyMode()) {
  case FLY_MODE::DEPTH_HOLD:
    modeName = "DEPTH HOLD";
    break;
  case FLY_MODE::STABILIZE:
    modeName = "STABILIZE";
    break;
  case FLY_MODE::MANUAL:
    modeName = "MANUAL";
    break;
  default:
    break;
  }

  Log->info("Send order: X: {} ; Y: {} ; Z: {} ; R: {} ; Arm: {} ; Mode: {}",
            order->GetX(), order->GetY(), order->GetZ(), order->GetR(),
            order->Arm() ? "true" : "false", modeName);

  memcpy(orderPtr, order->GetBuffer(), TeleopOrder::Size);

  sender->SetDesiredState(state);
  // remember current button states for future comparison
  previous_buttons = std::vector<int>(joy->buttons);
}

struct Params {
  std::string serialPort, masterUri;
  bool log2Console;
};

static Params params;
static LoggerPtr Log;

int GetParams(ros::NodeHandle &nh) {
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
  ros::init(argc, argv, "operator_control_v3");
  Log = CreateLogger("operator_control_v3:main");
  Log->Info("Init");
  ros::NodeHandle nh("~");
  GetParams(nh);

  Log->LogToConsole(params.log2Console);
  auto stream = CreateObject<dccomms_utils::S100Stream>(
      params.serialPort, SerialPortStream::BAUD_2400);
  stream->Open();

  Teleop teleop(stream);
  teleop.LogToConsole(params.log2Console);

  teleop.spin();
  return 0;
}
