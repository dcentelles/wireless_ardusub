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
#include <mavlink_cpp/GCS.h>
#include <mavlink_cpp/utils.h>
#include <telerobotics/StateReceiver.h>
#include <telerobotics/TeleopOrder.h>

using namespace cpplogging;
using namespace telerobotics;
using namespace std::chrono_literals;
using namespace telerobotics;
using namespace mavlink_cpp;

static LoggerPtr Log;

double getJoyAxisNormalized(int x) { return 200. / 256 * x; }
double arduSubXYR(double per) { return per / 0.1; }
double arduSubZ(double per) { return (per + 100) / 0.2; }

int main(int argc, char **argv) {
  Log = CreateLogger("TeleopROV");
  Log->Info("Init");
  TeleopOrderPtr order = TeleopOrder::Build();
  StateReceiver stateReceiver;
  uint16_t localPort = 14550;
  mavlink_cpp::Ptr<GCS> control = mavlink_cpp::CreateObject<GCS>(localPort);
  control->SetLogName("GCS");
  control->SetLogLevel(debug);
  control->Start();

  stateReceiver.SetStateReceivedCallback([order,
                                          control](StateReceiver &receiver) {
    int32_t state;
    receiver.GetState(order->GetBuffer(), TeleopOrder::Size);

    std::string modeName = "";
    switch (order->GetFlyMode()) {
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
    // order:  y = 200/256x
    // z control: y = 200/1000x - 100
    // x,y and r control: y = 200/2000x

    double xNorm = getJoyAxisNormalized(x);
    double yNorm = getJoyAxisNormalized(y);
    double zNorm = getJoyAxisNormalized(z);
    double rNorm = getJoyAxisNormalized(r);
    x = ceil(arduSubXYR(xNorm));
    y = ceil(arduSubXYR(yNorm));
    z = ceil(arduSubZ(zNorm));
    r = ceil(arduSubXYR(rNorm));

    Log->Info(
        "Manual control: X: {} ; Y: {} ; Z: {} ; R: {} ; Arm: {} ; Mode: {}", x,
        y, z, r, order->Arm() ? "true" : "false", modeName);

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
