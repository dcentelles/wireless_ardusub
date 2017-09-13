/*
 * Adaptation of:
 *  File: bluerov_apps/src/teleop_joy.cpp
 *  Author: Josh Villbrandt <josh@javconcepts.com>
 *  Date: February 2016
 *  Description: Manual remote control of ROVs like the bluerov_apps.
 * By centelld@uji.es for the wireless bluerov project
 */

#include <cpplogging/cpplogging.h>
#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <telerobotics/StateSender.h>
#include <vector>
#include <wireless_ardusub/wireless_teleop_joyConfig.h>

using namespace cpplogging;
using namespace dcauv;

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
  double computeAxisValue(const sensor_msgs::Joy::ConstPtr &joy, int index,
                          double expo);
  uint16_t mapToPpm(double in);
  void configCallback(wireless_ardusub::wireless_teleop_joyConfig &update,
                      uint32_t level);
  void joyCallback(const sensor_msgs::Joy::ConstPtr &joy);

  // state transmitter
  StateSender sender;
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
  uint16_t mode;
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
  mode = MODE_STABILIZE;
  camera_tilt = 1500;
  initLT = false;
  initRT = false;
  SetLogName("TeleopJoy");
  sender.Start();
  Log->info("Sender initialized");

  sender.SetLogLevel(LogLevel::info);
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
    Info("armed");
  } else {
    Info("disarmed");
  }
}

void TeleopJoy::cmdTakeoffLand(bool takeoff) {
  // https://pixhawk.ethz.ch/mavlink/#MAV_CMD_NAV_LAND_LOCAL
  if (takeoff) {
    Info("takeoff");
  } else {
    Info("land");
  }
}

double TeleopJoy::computeAxisValue(const sensor_msgs::Joy::ConstPtr &joy,
                                   int index, double expo) {
  // return 0 if axis index is invalid
  if (index < 0 || index >= joy->axes.size()) {
    return 0.0;
  }
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
  int state;
  float raw = joy->axes[0];
  if (raw > 0.4)
    state = 1;
  else if (raw < -0.4)
    state = -1;
  else
    state = 0;
  Log->info("State: {} (raw: {})", state, raw);
  sender.SetState(sizeof(state), &state);

  // mode switching
  if (risingEdge(joy, config.stabilize_button)) {
    mode = MODE_STABILIZE;
  } else if (risingEdge(joy, config.alt_hold_button)) {
    mode = MODE_ALT_HOLD;
  }

  // takeoff and land
  if (risingEdge(joy, config.land_button)) {
    cmdTakeoffLand(false);
  } else if (risingEdge(joy, config.takeoff_button)) {
    cmdTakeoffLand(true);
  }

  // change camera_tilt
  if (risingEdge(joy, config.cam_tilt_reset)) {
    camera_tilt = 1500;
  } else if (risingEdge(joy, config.cam_tilt_up)) {
    camera_tilt = camera_tilt + config.cam_tilt_step;
    if (camera_tilt > PPS_MAX) {
      camera_tilt = PPS_MAX;
    }
  } else if (risingEdge(joy, config.cam_tilt_down)) {
    camera_tilt = camera_tilt - config.cam_tilt_step;
    if (camera_tilt < PPS_MIN) {
      camera_tilt = PPS_MIN;
    }
  }

  // remember current button states for future comparison
  previous_buttons = std::vector<int>(joy->buttons);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "wireless_teleop_joy");
  TeleopJoy teleop_joy;
  teleop_joy.spin();
  return 0;
}
