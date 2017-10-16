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
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <vector>
#include <wireless_ardusub/HROVMessage.h>
#include <wireless_ardusub/TeleopOrder.h>
#include <wireless_ardusub/nodes/Operator.h>
#include <wireless_ardusub/wireless_teleop_joyConfig.h>

using namespace cpplogging;
using namespace wireless_ardusub;

class TeleopJoy : public Logger {
public:
  TeleopJoy();
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

  TeleopOrderPtr order;
  // state transmitter
  Ptr<Operator> sender;
  // node handle
  ros::NodeHandle nh;

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
};

TeleopJoy::TeleopJoy() {
  // connect dynamic reconfigure
  dynamic_reconfigure::Server<
      wireless_ardusub::wireless_teleop_joyConfig>::CallbackType f;
  f = boost::bind(&TeleopJoy::configCallback, this, _1, _2);
  server.setCallback(f);

  // connects subs and pubs
  joy_sub =
      nh.subscribe<sensor_msgs::Joy>("joy", 1, &TeleopJoy::joyCallback, this);

  // initialize state variables
  camera_tilt = 1500;
  initLT = false;
  initRT = false;
  SetLogName("TeleopJoy");
  Log->info("Sender initialized");

  auto stream = CreateObject<dccomms_utils::S100Stream>(
      "/dev/ttyUSB0", SerialPortStream::BAUD_2400, 2000);
  stream->Open();

  sender = CreateObject<Operator>(stream);
  sender->SetLogLevel(LogLevel::info);
  sender->SetRxStateSize(HROVMessage::MessageLength);
  sender->SetTxStateSize(TeleopOrder::Size);
  order = TeleopOrder::Build();

  sender->Start();
}

void TeleopJoy::spin() {
  ros::Rate loop(config.pub_rate);

  while (ros::ok()) {
    // call all waiting callbacks
    ros::spinOnce();

    // enforce a max publish rate
    loop.sleep();
  }
}

void TeleopJoy::configCallback(
    wireless_ardusub::wireless_teleop_joyConfig &update, uint32_t level) {
  ROS_INFO("reconfigure request received");
  config = update;
}

bool TeleopJoy::risingEdge(const sensor_msgs::Joy::ConstPtr &joy, int index) {
  return (joy->buttons[index] == 1 && previous_buttons[index] == 0);
}

void TeleopJoy::setArming(bool arm) {
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

void TeleopJoy::cmdTakeoffLand(bool takeoff) {
  // https://pixhawk.ethz.ch/mavlink/#MAV_CMD_NAV_LAND_LOCAL
  if (takeoff) {
    Debug("takeoff");
  } else {
    Debug("land");
  }
}

int8_t TeleopJoy::computeAxisValue(const sensor_msgs::Joy::ConstPtr &joy,
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

void TeleopJoy::joyCallback(const sensor_msgs::Joy::ConstPtr &joy) {
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
  sender->SetDesiredState(order->GetBuffer());
  // remember current button states for future comparison
  previous_buttons = std::vector<int>(joy->buttons);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "wireless_teleop_joy");
  TeleopJoy teleop_joy;
  teleop_joy.spin();
  return 0;
}
