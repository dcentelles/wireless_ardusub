#ifndef INCLUDE_WIRELESS_ARDUSUB_JOY_CONTROLLER_H_
#define INCLUDE_WIRELESS_ARDUSUB_JOY_CONTROLLER_H_

#include <cpplogging/Logger.h>
#include <dynamic_reconfigure/server.h>
#include <ros/publisher.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <wireless_ardusub/wireless_teleop_joyConfig.h>

namespace wireless_ardusub {

using namespace cpplogging;

class JoyController : public Logger {
public:
  void ConfigCallback(wireless_ardusub::wireless_teleop_joyConfig &update,
                      uint32_t level) {
    Info("reconfigure request received");
    Config = update;
  }

  JoyController(ros::NodeHandle &nh) : _nh(nh) {
    // connect dynamic reconfigure
    dynamic_reconfigure::Server<
        wireless_ardusub::wireless_teleop_joyConfig>::CallbackType f;
    f = boost::bind(&JoyController::ConfigCallback, this, _1, _2);
    _server.setCallback(f);

    // connects subs and pubs
    _joy_sub = _nh.subscribe<sensor_msgs::Joy>(
        "/joy", 1, &JoyController::JoyCallback, this);
  }

  int8_t ComputeAxisValue(const sensor_msgs::Joy::ConstPtr &joy, int index) {
    // return 0 if axis index is invalid
    if (index < 0 || index >= joy->axes.size()) {
      return 0.0;
    }

    double raw = joy->axes[index]; // raw in [-1,1]
    double dvalue = 100 * raw;
    int8_t value = ceil(dvalue);
    Log->debug("{}: Raw: {} ; DValue: {} ; Value: {}", index, raw, dvalue,
               value);
    return value;
  }

  bool RisingEdge(const sensor_msgs::Joy::ConstPtr &joy, int index) {
    return (joy->buttons[index] == 1 && _previous_buttons[index] == 0);
  }
  void SetJoyCb(std::function<void(JoyController &,
                                   const sensor_msgs::Joy::ConstPtr &joy)>
                    handler) {
    _joy_cb = handler;
  }

  void JoyCallback(const sensor_msgs::Joy::ConstPtr &joy) {
    _joy_cb(*this, joy);
  }

  // ATTRIBUTES
  ros::NodeHandle &_nh;
  dynamic_reconfigure::Server<wireless_ardusub::wireless_teleop_joyConfig>
      _server;
  wireless_ardusub::wireless_teleop_joyConfig Config;
  ros::Subscriber _joy_sub;
  std::vector<int> _previous_buttons;
  std::function<void(JoyController &, const sensor_msgs::Joy::ConstPtr &joy)>
      _joy_cb = [](JoyController &, const sensor_msgs::Joy::ConstPtr &joy) {};
};

} // namespace wireless_ardusub

#endif /* INCLUDE_WIRELESS_ARDUSUB_CONSTANTS_H_ */
