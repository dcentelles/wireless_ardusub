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
#include <telerobotics/StateReceiver.h>

using namespace cpplogging;
using namespace dcauv;
using namespace std::chrono_literals;

static LoggerPtr Log;

int main(int argc, char **argv) {
  Log = CreateLogger("TeleopROV");
  Log->Info("Init");

  StateReceiver stateReceiver;
  stateReceiver.SetStateReceivedCallback([](StateReceiver &receiver) {
    int32_t state;
    receiver.GetState(&state, sizeof(state));
    Log->Info("New state: {}", state);
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
