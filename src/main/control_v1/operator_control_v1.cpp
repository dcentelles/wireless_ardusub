#include <chrono> // std::chrono::seconds
#include <condition_variable>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <mutex>
#include <pthread.h>
#include <stdint.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <telerobotics/ROVOperator.h>
#include <thread> // std::this_thread::sleep_for
#include <unistd.h>

// ROS and image_transport
#include "ros/ros.h"
#include <image_utils_ros_msgs/EncodedImg.h>

#include <merbots_whrov_msgs/MoveOrderAction.h>
#include <merbots_whrov_msgs/hrov_settings.h>
#include <merbots_whrov_msgs/movement.h>
#include <merbots_whrov_msgs/position.h>

#include "std_msgs/String.h"
#include <actionlib/server/simple_action_server.h>

// end ROS

// Merbots
#include <telerobotics/HROVMessage.h>
#include <telerobotics/OperatorMessage.h>
// EndMerbots

// Logging
#include <cpplogging/Logger.h>

using namespace std;
using namespace telerobotics;

static LoggerPtr Log = cpplogging::CreateLogger("OperatorMain");
uint8_t imgBuffer[20000];
static unsigned int g_iwidth;
static unsigned int g_iheight;
struct imgBuffer *img;
static image_utils_ros_msgs::EncodedImg encodedImgMsg;
static ros::Publisher encodedImage_pub;

// static image_transport::CameraPublisher camera_pub;
static ros::Publisher currentHROVState_pub;
static ros::Subscriber desiredHROVState_sub;

static uint8_t *y, *u, *v;
static telerobotics::OperatorMessagePtr currentOperatorMessage;
static telerobotics::HROVMessagePtr currentHROVMessage;
static std::mutex currentOperatorMessage_mutex;
static std::mutex currentHROVMessage_mutex;
static std::condition_variable currentOperatorMessage_cond;
static std::condition_variable currentHROVMessage_cond;
static bool currentOperatorMessage_updated;
static bool currentHROVMessage_updated;

// actionlib objects

class MoveOrderActionServer {
public:
  void NotifyNewHROVState(bool _hrovReady, int _requestedOID, bool cancelled) {
    hrovState_mutex.lock();
    hrovReady = _hrovReady;
    requestedOID = _requestedOID;
    lastOrderCancelled = cancelled;
    hrovState_mutex.unlock();
    hrovState_updated_cond.notify_one();
  }

  // http://wiki.ros.org/actionlib_tutorials/Tutorials/SimpleActionServer%28ExecuteCallbackMethod%29
protected:
  std::string actionName;
  actionlib::SimpleActionServer<merbots_whrov_msgs::MoveOrderAction>
      actionServer;
  merbots_whrov_msgs::MoveOrderFeedback moveOrderFeedback;
  merbots_whrov_msgs::MoveOrderResult moveOrderResult;

  int oid, requestedOID;
  bool hrovReady, lastOrderCancelled;
  bool transmittingOrder;

  std::mutex hrovState_mutex;
  std::condition_variable hrovState_updated_cond;

public:
  MoveOrderActionServer(std::string name, ros::NodeHandle nh)
      : actionServer(
            nh, name,
            boost::bind(&MoveOrderActionServer::actionWorker, this, _1), false),
        actionName(name), oid(0), requestedOID(0), hrovReady(false),
        transmittingOrder(false) {}
  ~MoveOrderActionServer(void) {}
  void Start() { actionServer.start(); }

  bool CancelRequested(void) {
    if (actionServer.isPreemptRequested() || !ros::ok()) {
      Log->Info("{}: Preempted", actionName);
      actionServer.setPreempted();
      return true;
    }
    return false;
  }

  void CancelLastOrder(void) {
    // Puede que la orden se haya enviado ya, por lo que hay que cancelarla
    currentOperatorMessage_mutex.lock();
    currentOperatorMessage->CancelLastOrderFlag(true);
    currentOperatorMessage->SetOrderSeqNumber(requestedOID);
    currentOperatorMessage->SetOrderType(
        telerobotics::OperatorMessage::NoOrder);
    currentOperatorMessage_updated = true;
    currentOperatorMessage_mutex.unlock();
    currentOperatorMessage_cond.notify_one();
  }

  void actionWorker(const merbots_whrov_msgs::MoveOrderGoalConstPtr &goal) {
    auto moveOrder = telerobotics::HROVMoveOrder::BuildHROVMoveOrder();

    moveOrder->SetYaw(goal->order.yaw);
    moveOrder->SetZ(goal->order.Z);
    moveOrder->SetX(goal->order.X);
    moveOrder->SetY(goal->order.Y);
    moveOrder->Relative(goal->order.relative);

    std::unique_lock<std::mutex> lock(hrovState_mutex);

    moveOrderFeedback.percent_complete = 5;
    moveOrderFeedback.message =
        "Waiting for the ROV to get ready to handle a new order...";
    actionServer.publishFeedback(moveOrderFeedback);

    while (transmittingOrder) {
      hrovState_updated_cond.wait_for(lock, std::chrono::seconds(1));
      if (CancelRequested()) {
        moveOrderFeedback.percent_complete = 100;
        moveOrderFeedback.message = "Last order cancelled";
        actionServer.publishFeedback(moveOrderFeedback);
        lock.unlock();
        return;
      }
    }
    // Si el robot recibe una nueva orden con el OID
    // que espera y está en curso alguna orden, cancelará la orden actual para
    // ejecutar la nueva.
    transmittingOrder = true;
    oid = requestedOID;
    lock.unlock();

    moveOrderFeedback.percent_complete = 10;
    moveOrderFeedback.message = "Sending the new position request...";
    actionServer.publishFeedback(moveOrderFeedback);

    currentOperatorMessage_mutex.lock();
    currentOperatorMessage->SetOrderSeqNumber(oid);
    currentOperatorMessage->SetOrderType(
        telerobotics::OperatorMessage::Move);
    currentOperatorMessage->SetMoveOrder(moveOrder);
    currentOperatorMessage->CancelLastOrderFlag(false);
    currentOperatorMessage_updated = true;
    currentOperatorMessage_mutex.unlock();
    currentOperatorMessage_cond.notify_one();

    moveOrderFeedback.percent_complete = 15;
    moveOrderFeedback.message = "Waiting for the order acknowledgment...";
    actionServer.publishFeedback(moveOrderFeedback);

    int nextOID = telerobotics::HROVMessage::GetNextOrderSeqNumber(oid);
    lock.lock();
    while (requestedOID != nextOID) {
      hrovState_updated_cond.wait_for(lock, std::chrono::seconds(1));
      if (CancelRequested()) {
        moveOrderFeedback.percent_complete = 50;
        moveOrderFeedback.message = "Cancelling last order...";
        actionServer.publishFeedback(moveOrderFeedback);
        do {
          CancelLastOrder();
          // Hay que esperar la confirmación de que el ROV ha recibido la orden
          // de cancelación de la requestedOID
          /*Si esto no se hace, y se da otra orden antes de que el rov haya
             recibido la orden anterior, la nuevo orden
                      puede llegar a enviarse con el mismo OID que la anterior.
             Que pasaría? Veamos un ejemplo:
                        1. Se envía una orden con OID 1.
                        2. Antes de que el ROV reciba la orden con OID 1, se
             cancela la orden para enviar otra.
                        3. Como el ROV todavía no ha recibido la orden anterior,
             su estado es de Ready, y su requested OID sigue siendo 1.
                        4. Al tener el estado de Ready, se envía la nueva orden
             con OID 1 y se espera la recepción de requested OID igual a 2
                        5. En ese momento llega la antigua orden al ROV (la que
             tiene OID 1, al igual que la nueva orden). El robot la ejecuta, ya
             que tiene la misma OID
                            que la que espera. Mientras, cambia la requested OID
             a la 2
                        6. Seguidamente llega al ROV la nueva orden, con OID 1.
             Sin embargo, como el ROV espera la OID = 2 la descarta.
                        7. Cuando el operador reciba un paquete con requested
             OID = 2, pensará que la nueva orden se ha enviado correctamente.
             Sin embargo,
                           lo que ha pasado es que el robot ha ejecutado la
             orden anterior.
                        */
          hrovState_updated_cond.wait_for(lock, std::chrono::seconds(1));
        } while (!lastOrderCancelled);

        currentOperatorMessage_mutex.lock();
        currentOperatorMessage->CancelLastOrderFlag(false);
        currentOperatorMessage_updated = true;
        currentOperatorMessage_mutex.unlock();
        currentOperatorMessage_cond.notify_one();

        moveOrderFeedback.percent_complete = 100;
        moveOrderFeedback.message = "Last order cancelled";
        actionServer.publishFeedback(moveOrderFeedback);
        transmittingOrder = false;
        lock.unlock();
        return;
      }
    }
    currentOperatorMessage_mutex.lock();
    currentOperatorMessage->SetOrderType(
        telerobotics::OperatorMessage::NoOrder);
    currentOperatorMessage_updated = true;
    currentOperatorMessage_mutex.unlock();
    currentOperatorMessage_cond.notify_one();

    moveOrderFeedback.percent_complete = 60;
    moveOrderFeedback.message = "ROV received the order.";
    actionServer.publishFeedback(moveOrderFeedback);

    while (!hrovReady) {
      hrovState_updated_cond.wait_for(lock, std::chrono::seconds(1));
      if (CancelRequested()) {
        moveOrderFeedback.percent_complete = 50;
        moveOrderFeedback.message = "Cancelling last order...";
        actionServer.publishFeedback(moveOrderFeedback);
        do {
          CancelLastOrder();
          hrovState_updated_cond.wait_for(lock, std::chrono::seconds(1));
        } while (!lastOrderCancelled);

        currentOperatorMessage_mutex.lock();
        currentOperatorMessage->CancelLastOrderFlag(false);
        currentOperatorMessage_updated = true;
        currentOperatorMessage_mutex.unlock();
        currentOperatorMessage_cond.notify_one();

        moveOrderFeedback.percent_complete = 100;
        moveOrderFeedback.message = "Last order cancelled";
        actionServer.publishFeedback(moveOrderFeedback);
        transmittingOrder = false;
        lock.unlock();
        return;
      }
    }
    transmittingOrder = false;
    lock.unlock();

    moveOrderFeedback.percent_complete = 100;
    moveOrderFeedback.message = "Received last order completion confirmation";
    actionServer.publishFeedback(moveOrderFeedback);
    moveOrderResult.success = true;
    actionServer.setSucceeded(moveOrderResult);
  }
};
static MoveOrderActionServer *orderActionServer;

// end actionlib objects

// messageSenderWorker vars
static bool messageSenderWorker_mustContinue_flag;
static std::thread messageSenderWorker;
void messageSenderWork(ROVOperator *rovOperator) {
  while (1) {
    std::unique_lock<std::mutex> lock(currentOperatorMessage_mutex);
    while (!currentOperatorMessage_updated) {
      if (messageSenderWorker_mustContinue_flag)
        currentOperatorMessage_cond.wait(lock);
      else
        return;
    }
    rovOperator->SetDesiredState(currentOperatorMessage->GetBuffer());
    currentOperatorMessage_updated = false;
  }
}
// end messageSenderWorker vars

// rovMsgParserWorker vars
static bool hrovMsgParserWorker_mustContinue_flag;
static std::thread hrovMsgParserWorker;
void hrovMsgParserWork() {
  while (1) {
    std::unique_lock<std::mutex> lock(currentHROVMessage_mutex);
    while (!currentHROVMessage_updated) {
      if (hrovMsgParserWorker_mustContinue_flag)
        currentHROVMessage_cond.wait(lock);
      else
        return;
    }
    auto oid = currentHROVMessage->GetExpectedOrderSeqNumber();
    auto cancelled = currentHROVMessage->LastOrderCancelledFlag();
    auto ready = currentHROVMessage->Ready();
    auto X = currentHROVMessage->GetX();
    auto Y = currentHROVMessage->GetY();
    auto Z = currentHROVMessage->GetZ();
    auto yaw = currentHROVMessage->GetYaw();
    currentHROVMessage_updated = false;
    lock.unlock();

    merbots_whrov_msgs::position position;
    position.X = X;
    position.Y = Y;
    position.Z = Z;
    position.yaw = yaw;

    currentHROVState_pub.publish(position);
    orderActionServer->NotifyNewHROVState(ready, oid, cancelled);
  }
}

// end rovMsgParserWorker vars
void startMessageSenderWorker(ROVOperator *rovOperator) {
  messageSenderWorker_mustContinue_flag = true;
  messageSenderWorker = std::thread(messageSenderWork, rovOperator);
}

void terminateMessageSenderWorker(void) {
  messageSenderWorker_mustContinue_flag = false;
  currentOperatorMessage_cond.notify_one();
}

void startHROVMsgParserWorker() {
  hrovMsgParserWorker_mustContinue_flag = true;
  hrovMsgParserWorker = std::thread(hrovMsgParserWork);
}

void terminateHROVMsgParserWorker(void) {
  hrovMsgParserWorker_mustContinue_flag = false;
  currentHROVMessage_cond.notify_one();
}

struct ProtocolConfig {
  unsigned int maxPacketLength;
  unsigned int txStateSize, rxStateSize;
  unsigned int width;
  unsigned int height;
  std::string ltype;

  ProtocolConfig() {
    txStateSize = 20;
    rxStateSize = 20;
    maxPacketLength = 200;
    width = 0;
    height = 0;
    ltype = "fulld"; // link type
  }
};

telerobotics::HROVSettingsPtr
BuildHROVSettings(const merbots_whrov_msgs::hrov_settings::ConstPtr &msg) {
  auto settings = telerobotics::HROVSettings::BuildHROVSettings();
  settings->SetROIConf(msg->image_config.roi_x0, msg->image_config.roi_y0,
                       msg->image_config.roi_x1, msg->image_config.roi_y1,
                       msg->image_config.roi_shift);

  settings->SetImgSize(msg->image_config.size);
  settings->SetImgResolution(msg->image_config.resolution);
  settings->SetMaxPacketLength(msg->protocol_config.max_packet_length);
  return settings;
}

void HandleNewDesiredSettings(
    const merbots_whrov_msgs::hrov_settings::ConstPtr &msg) {
  auto desiredSettings = BuildHROVSettings(msg);

  currentOperatorMessage_mutex.lock();
  currentOperatorMessage->SetSettings(desiredSettings);
  currentOperatorMessage_mutex.unlock();

  currentOperatorMessage_updated = true;
  currentOperatorMessage_cond.notify_one();
}

void HandleCurrentState(ROVOperator &rovOperator) {
  currentHROVMessage_mutex.lock();
  rovOperator.GetLastConfirmedState(currentHROVMessage->GetBuffer());
  currentHROVMessage_mutex.unlock();

  currentHROVMessage_updated = true;
  currentHROVMessage_cond.notify_one();
}

void HandleNewImage(ROVOperator &rovOperator) {
  int encodedImgSize;
  encodedImgSize = rovOperator.GetLastReceivedImage(imgBuffer);
  encodedImgMsg.img.resize(encodedImgSize);
  memcpy(encodedImgMsg.img.data(), imgBuffer, encodedImgSize);
  encodedImage_pub.publish(encodedImgMsg);
}

void initMessages(void) {
  currentHROVMessage = telerobotics::HROVMessage::BuildHROVMessage();
  currentOperatorMessage =
      telerobotics::OperatorMessage::BuildOperatorMessage();

  currentOperatorMessage_updated = false;
  currentHROVMessage_updated = false;
}

void initROSInterface(ros::NodeHandle &nh, int argc, char **argv,
                      ROVOperator &rovOperator) {
  initMessages();
  orderActionServer = new MoveOrderActionServer("actions/move_order", nh);
  orderActionServer->Start();
  desiredHROVState_sub = nh.subscribe<merbots_whrov_msgs::hrov_settings>(
      "desired_hrov_settings", 1, HandleNewDesiredSettings); //,
  // boost::bind(HandleNewDesiredSettings, _1, &rovOperator));

  encodedImage_pub =
      nh.advertise<image_utils_ros_msgs::EncodedImg>("encoded_image", 1);

  currentHROVState_pub =
      nh.advertise<merbots_whrov_msgs::position>("current_hrov_position", 1);

  startMessageSenderWorker(&rovOperator);
  startHROVMsgParserWorker();
}

void startROSInterface() { ros::spin(); }

int main(int argc, char **argv) {

  ros::init(argc, argv, "operator_control");
  ros::NodeHandle nh("~");

  bool log2Console;
  if (!nh.getParam("log2Console", log2Console)) {
    ROS_ERROR("Failed to get param log2Console");
    return 1;
  } else {
    ROS_INFO("Log2Console: %d", log2Console);
  }
  int remoteAddr;
  if (!nh.getParam("remoteAddr", remoteAddr)) {
    ROS_ERROR("Failed to get param remoteAddr");
    return 1;
  } else {
    ROS_INFO("remoteAddr: %d", remoteAddr);
  }
  int localAddr;
  if (!nh.getParam("localAddr", localAddr)) {
    ROS_ERROR("Failed to get param localAddr");
    return 1;
  } else {
    ROS_INFO("localAddr: %d", localAddr);
  }

  std::string log2File;
  bool logging2File = false;
  if (!nh.getParam("log2File", log2File)) {
    ROS_ERROR("Failed to get param log2File");
    return 1;
  } else {
    ROS_INFO("log2File: %s", log2File.c_str());
    if (log2File == "") {
      ROS_INFO("No log to file");
    } else {
      ROS_INFO("Logging to file '%s'", log2File.c_str());
      logging2File = true;
    }
  }

  LinkType linkType;
  std::string slinkType;
  if (!nh.getParam("linkType", slinkType)) {
    ROS_ERROR("Failed to get param linkType");
    return 1;
  } else {
    if (slinkType == "fulld") {
      linkType = LinkType::fullDuplex;
    } else if (slinkType == "halfd") {
      linkType = LinkType::halfDuplex;
    } else {
      ROS_ERROR("wrong linkType value '%s'", slinkType.c_str());
      return 1;
    }
    ROS_INFO("linkType: %s", slinkType.c_str());
  }

  ROVOperator rovOperator(linkType);

  rovOperator.SetLogLevel(cpplogging::LogLevel::debug);
  rovOperator.FlushLogOn(cpplogging::LogLevel::info);
  rovOperator.LogToConsole(log2Console);
  if (logging2File) {
    rovOperator.LogToFile(log2File);
  }

  rovOperator.SetLocalAddr(localAddr);
  rovOperator.SetRemoteAddr(remoteAddr);

  Log->SetLogLevel(cpplogging::LogLevel::info);
  Log->FlushLogOn(cpplogging::LogLevel::info);

  Log->Info("remote addr: {}", remoteAddr);

  try {
    rovOperator.SetRxStateSize(telerobotics::HROVMessage::MessageLength);
    rovOperator.SetTxStateSize(
        telerobotics::OperatorMessage::MessageLength);
    rovOperator.SetImageReceivedCallback(&HandleNewImage);
    rovOperator.SetStateReceivedCallback(&HandleCurrentState);

    initROSInterface(nh, argc, argv, rovOperator);

    rovOperator.Start();

    startROSInterface();

  } catch (std::exception &e) {
    Log->Error("Radio Exception: {}", e.what());
    exit(1);
  }

  return 0;
}
