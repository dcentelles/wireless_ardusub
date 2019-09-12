#include <chrono>
#include <cpplogging/cpplogging.h>
#include <cpputils/SignalManager.h>
#include <merbots_whrov_msgs/debug.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <thread>
#include <wireless_ardusub/JoyController.h>
#include <wireless_ardusub/OperatorController.h>

using namespace cpputils;
using namespace mavlink_cpp;
using namespace cpplogging;
using namespace std::chrono_literals;
using namespace control;
using namespace wireless_ardusub;

int main(int argc, char **argv) {
  auto log = CreateLogger("Main");
  log->SetLogLevel(info);
  log->FlushLogOn(debug);
  log->LogToConsole(true);
  log->SetAsyncMode();

  ros::init(argc, argv, "custom_setpoint");
  ros::NodeHandle nh("~");
  ros::Rate rate(10);

  OperatorController::Params params;
  params.sitl = true;

  nh.getParam("sitl", params.sitl);
  if (params.sitl)
    log->Info("Using params for SITL");
  else
    log->Info("Using params for HIL");

  OperatorController op(params);
  JoyController joyCtr(nh);
  op.Start();

  auto joyCb = [&](JoyController &joy_ctr,
                   const sensor_msgs::Joy::ConstPtr &joy) {
    // init previous_buttons
    if (joy_ctr._previous_buttons.size() != joy->buttons.size()) {
      joy_ctr._previous_buttons = std::vector<int>(joy->buttons);
    }

    // arm, disarm
    if (joy_ctr.RisingEdge(joy, joy_ctr.Config.disarm_button)) {
      op.ControlState.arm = false;
      op.Info("disarm");
    } else if (joy_ctr.RisingEdge(joy, joy_ctr.Config.arm_button)) {
      op.ControlState.arm = true;
      op.Info("arm");
    }

    op.ControlState.x = joy_ctr.ComputeAxisValue(joy, joy_ctr.Config.x_axis);
    op.ControlState.y =
        -1 * joy_ctr.ComputeAxisValue(joy, joy_ctr.Config.y_axis);
    op.ControlState.z = joy_ctr.ComputeAxisValue(joy, joy_ctr.Config.z_axis);
    op.ControlState.r =
        -1 * joy_ctr.ComputeAxisValue(joy, joy_ctr.Config.wz_axis);

    // mode switching
    if (joy_ctr.RisingEdge(joy, joy_ctr.Config.stabilize_button)) {
      op.ControlState.mode = FLY_MODE_R::STABILIZE;
    } else if (joy_ctr.RisingEdge(joy, joy_ctr.Config.alt_hold_button)) {
      op.ControlState.mode = FLY_MODE_R::DEPTH_HOLD;
    } else if (joy_ctr.RisingEdge(joy, 0)) {
      op.ControlState.mode = FLY_MODE_R::MANUAL;
    } else if (joy_ctr.RisingEdge(joy, 2)) {
      op.ControlState.mode = FLY_MODE_R::POS_HOLD;
    } else if (joy_ctr.RisingEdge(joy, 5)) {
      op.ControlState.mode = FLY_MODE_R::GUIDED;
    }
    std::string modeName = "";
    switch (op.ControlState.mode) {
    case FLY_MODE_R::DEPTH_HOLD:
      modeName = "DEPTH HOLD";
      break;
    case FLY_MODE_R::STABILIZE:
      modeName = "STABILIZE";
      break;
    case FLY_MODE_R::MANUAL:
      modeName = "MANUAL";
      break;
    case FLY_MODE_R::POS_HOLD:
      modeName = "POS HOLD";
      break;
    case FLY_MODE_R::GUIDED:
      modeName = "GUIDED";
      break;
    default:
      break;
    }
    joy_ctr._previous_buttons = std::vector<int>(joy->buttons);
  };

  joyCtr.SetJoyCb(joyCb);

  SignalManager::SetLastCallback(SIGINT, [&](int sig) {
    printf("Received %d signal.\nFlushing log messages...", sig);
    fflush(stdout);
    log->FlushLog();
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    printf("Log messages flushed.\n");
    exit(0);
  });

  while (1) {
    ros::spinOnce();
    rate.sleep();
  }
}
