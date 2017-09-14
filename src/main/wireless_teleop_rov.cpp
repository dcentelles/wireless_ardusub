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
#include <mavlink_cpp/mavlink_cpp.h>
#include <telerobotics/StateReceiver.h>
#include <wireless_ardusub/TeleopOrder.h>

using namespace cpplogging;
using namespace dcauv;
using namespace std::chrono_literals;
using namespace wireless_ardusub;
using namespace mavlink_cpp;

static LoggerPtr Log;

int main(int argc, char **argv) {
  Log = CreateLogger("TeleopROV");
  Log->Info("Init");
  TeleopOrderPtr order = TeleopOrder::Build();
  StateReceiver stateReceiver;
  uint16_t localPort = 14550;
  Ptr<GCS> control = CreateObject<GCS>(localPort);
  control->SetLogName("GCS");
  control->SetLogLevel(debug);
  control->Start();

  stateReceiver.SetStateReceivedCallback([order,
                                          control](StateReceiver &receiver) {
    int32_t state;
    receiver.GetState(order->GetBuffer(), TeleopOrder::Size);

    std::string modeName = "";
    switch (order->GetFlyMode()) {
    case FLY_MODE::DEPTH_HOLD:
      modeName = "DEPTH HOLD";
      //  control->SetDepthHoldMode();
      break;
    case FLY_MODE::STABILIZE:
      modeName = "STABILIZE";
      //  control->SetStabilizeMode();
      break;
    case FLY_MODE::MANUAL:
      modeName = "MANUAL";
      break;
    default:
      break;
    }

    Log->Info("Send order: X: {} ; Y: {} ; Z: {} ; R: {} ; Arm: {} ; Mode: {}",
              order->GetX(), order->GetY(), order->GetZ(), order->GetR(),
              order->Arm() ? "true" : "false", modeName);
    int x = order->GetX();
    int y = order->GetY();
    int z = order->GetZ();
    int r = order->GetR();

    control->SetManualControl(x, y, z, r);
    control->Arm(order->Arm());
  });

  stateReceiver.Start();
  Log->SetLogLevel(LogLevel::info);
  stateReceiver.SetLogLevel(LogLevel::info);
  while (1) {
    Log->Debug("I'm alive");
    std::this_thread::sleep_for(2000ms);
  }
  return 0;
}
