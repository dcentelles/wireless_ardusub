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
#include <mavlink_cpp/mavlink_cpp.h>
#include <wireless_ardusub/HROVMessage.h>
#include <wireless_ardusub/TeleopOrder.h>
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

int GetParams(ros::NodeHandle &nh) {
  std::string serialPort;
  if (!nh.getParam("port", serialPort)) {
    ROS_ERROR("Failed to get param port");
    return 1;
  } else {
    ROS_INFO("port topic: %s", serialPort.c_str());
  }
  params.serialPort = serialPort;

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

  uint16_t localPort = 14550;
  mavlink_cpp::Ptr<GCS> control = mavlink_cpp::CreateObject<GCS>(localPort);
  control->SetLogName("GCS");
  control->SetLogLevel(info);
  control->Start();

  Log->SetLogLevel(LogLevel::info);

  auto stream = dccomms::CreateObject<dccomms_utils::S100Stream>(
      params.serialPort, SerialPortStream::BAUD_2400);
  stream->Open();

  dccomms::Ptr<ROV> commsNode = dccomms::CreateObject<ROV>(stream);
  commsNode->SetOrdersReceivedCallback(
      [order, control](ROV &receiver) { Log->Info("Orders received!"); });
  commsNode->SetLogLevel(LogLevel::info);
  commsNode->SetRxStateSize(TeleopOrder::Size);
  commsNode->SetTxStateSize(HROVMessage::MessageLength);
  commsNode->SetMaxImageTrunkLength(50);
  commsNode->Start();

  auto cstate = dccomms::CreateObject<HROVMessage>();
  commsNode->SetCurrentTxState(cstate->GetBuffer());

  try {

    ros::Rate rate(30); // 30 hz
    initROSInterface(nh, argc, argv, commsNode);

    while (ros::ok()) {
      if (!commsNode->SendingCurrentImage()) {
        std::string msg =
            "En un lugar de la Mancha, de cuyo nombre no quiero"
            " acordarme, no ha mucho tiempo que vivía un hidalgo de l"
            "os de lanza en astillero, adarga antigua, rocín flaco y galg"
            "o corredor. Una olla de algo más vaca que carnero, salpicón las "
            "más"
            " noches, duelos y quebrantos los sábados, lentejas los viernes, "
            "algún pa"
            "lomino de añadidura los domingos, consumían las tres partes de su "
            "hacien"
            "da. El resto della concluían ";

        Log->Info("Sending the new captured image... ({} bytes)", msg.length());
        commsNode->SendImage((void *)msg.c_str(), msg.length());
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
