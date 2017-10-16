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
#include <wireless_ardusub/TeleopOrder.h>
#include <wireless_ardusub/nodes/ROV.h>

#include <ros/ros.h>

using namespace cpplogging;
using namespace std::chrono_literals;
using namespace wireless_ardusub;
using namespace mavlink_cpp;
using namespace std;

struct Params {
  std::string cameraTopic, masterUri;
  bool log2Console;
};

struct ProtocolConfig {
  unsigned int width;
  unsigned int height;
  unsigned int frameSize;
  ProtocolConfig() {
    width = 352;
    height = 288;
    frameSize = 100;
  }
};

static LoggerPtr Log;
static Params params;
static ProtocolConfig pconfig;
static std::mutex config_mutex;

int GetParams(ros::NodeHandle &nh) {
  std::string cameraTopic;
  if (!nh.getParam("image", cameraTopic)) {
    ROS_ERROR("Failed to get param image");
    return 1;
  } else {
    ROS_INFO("camera topic: %s", cameraTopic.c_str());
  }
  params.cameraTopic = cameraTopic;

  char *cmasterUri = getenv("ROS_MASTER_URI");
  std::string masterUri = cmasterUri;
  ROS_INFO("ROS MASTER URI: %s", masterUri.c_str());

  params.masterUri = masterUri;

  bool log2Console;
  if (!nh.getParam("log2Console", log2Console)) {
    ROS_ERROR("Failed to get param log2Console");
    return 1;
  } else {
    ROS_INFO("log2Console: %d", log2Console);
  }

  params.log2Console = log2Console;

  return 0;
}

void initROSInterface(ros::NodeHandle &nh, int argc, char **argv,
                      dccomms::Ptr<ROV> commsNode) {}

int main(int argc, char **argv) {
  Log = CreateLogger("TeleopROV");
  Log->Info("Init");

  //// GET PARAMS
  ros::init(argc, argv, "rov_control");
  ros::NodeHandle nh("~");

  if (GetParams(nh))
    return 1;

  Log->SetLogLevel(cpplogging::LogLevel::debug);
  Log->FlushLogOn(cpplogging::LogLevel::info);

  TeleopOrderPtr order = TeleopOrder::Build();

  dccomms::Ptr<ROV> commsNode;
  uint16_t localPort = 14550;
  mavlink_cpp::Ptr<GCS> control = mavlink_cpp::CreateObject<GCS>(localPort);
  control->SetLogName("GCS");
  control->SetLogLevel(debug);
  control->Start();

  commsNode->SetOrdersReceivedCallback([order, control](ROV &receiver) {});

  Log->SetLogLevel(LogLevel::info);
  commsNode->SetLogLevel(LogLevel::info);
  commsNode->Start();

  try {

    ros::Rate rate(30); // 30 hz
    initROSInterface(nh, argc, argv, commsNode);

    while (ros::ok()) {
      if (!commsNode->SendingCurrentImage()) {
        config_mutex.lock(); // At the moment, only the compression parameters
                             // will be dynamic

        Log->Info("Sending the new captured image...");
        config_mutex.unlock();
      }
      ros::spinOnce();
      rate.sleep();
    }
  } catch (std::exception &e) {
    Log->Error("Exception: {}", e.what());
    exit(1);
  }
  return 0;
}
