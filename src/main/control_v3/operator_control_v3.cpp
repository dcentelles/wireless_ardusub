/*
 * Adaptation of:
 *  File: bluerov_apps/src/teleop_joy.cpp
 *  Author: Josh Villbrandt <josh@javconcepts.com>
 *  Date: February 2016
 *  Description: Manual remote control of ROVs like the bluerov_apps.
 * By centelld@uji.es for the wireless bluerov project
 */

#include <actionlib/server/simple_action_server.h>
#include <cmath>
#include <cpplogging/cpplogging.h>
#include <dccomms_utils/S100Stream.h>
#include <dynamic_reconfigure/server.h>
#include <image_utils_ros_msgs/EncodedImg.h>
#include <merbots_whrov_msgs/OrderAction.h>
#include <merbots_whrov_msgs/hrov_settings.h>
#include <merbots_whrov_msgs/position.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <vector>
#include <wireless_ardusub/HROVMessage.h>
#include <wireless_ardusub/OperatorMessageV2.h>
#include <wireless_ardusub/nodes/Constants.h>
#include <wireless_ardusub/nodes/Operator.h>
#include <wireless_ardusub/wireless_teleop_joyConfig.h>

using namespace cpplogging;
using namespace wireless_ardusub;

struct Params {
  std::string serialPort, masterUri;
  bool log2Console;
};

static Params params;
static LoggerPtr Log;

int GetParams() {
  ros::NodeHandle nh("~");
  std::string serialPort;
  if (!nh.getParam("port", serialPort)) {
    ROS_ERROR("Failed to get param port");
    return 1;
  } else {
    Log->Info("port topic: {}", serialPort);
  }
  params.serialPort = serialPort;

  char *cmasterUri = getenv("ROS_MASTER_URI");
  std::string masterUri = cmasterUri;
  Log->Info("ROS MASTER URI: {}", masterUri);

  params.masterUri = masterUri;

  bool log2Console;
  if (!nh.getParam("log2Console", log2Console)) {
    ROS_ERROR("Failed to get param log2Console");
    return 1;
  } else {
    Log->Info("log2Console: {}", log2Console);
  }

  params.log2Console = log2Console;

  return 0;
}

class OperatorController : public Logger {
public:
  OperatorController(ros::NodeHandle &nh, Ptr<CommsDevice> stream);
  void Spin();

private:
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

  // ATTRIBUTES
  std::string _actionName;
  actionlib::SimpleActionServer<merbots_whrov_msgs::OrderAction>
      _orderActionServer;
  merbots_whrov_msgs::OrderFeedback _orderFeedback;
  merbots_whrov_msgs::OrderResult _orderResult;
  int _oid, _requestedOID;
  bool _hrovReady, _lastOrderCancelled;
  bool _transmittingOrder;

  std::mutex _hrovState_mutex;
  std::condition_variable _hrovState_updated_cond;

  std::mutex _currentOperatorMessage_mutex;
  Ptr<OperatorMessageV2> _currentOperatorMessage;
  std::condition_variable _currentOperatorMessage_cond;
  bool _currentOperatorMessage_updated;

  std::mutex _currentHROVMessage_mutex;
  Ptr<HROVMessage> _currentHROVMessage;
  std::condition_variable _currentHROVMessage_cond;
  bool _currentHROVMessage_updated;

  TeleopOrderPtr _teleopOrder;
  Ptr<HROVSettingsV2> _settings;
  Ptr<Operator> _node;
  ros::NodeHandle &_nh;
  ros::Publisher _currentHROVState_pub;
  ros::Publisher _encodedImage_pub;
  ros::Subscriber _currentSettings_sub;
  dynamic_reconfigure::Server<wireless_ardusub::wireless_teleop_joyConfig>
      _server;
  wireless_ardusub::wireless_teleop_joyConfig _config;
  ros::Subscriber _joy_sub;
  uint16_t _camera_tilt;
  bool _initLT;
  bool _initRT;
  std::vector<int> _previous_buttons;
  image_utils_ros_msgs::EncodedImg _encodedImgMsg;
  uint8_t _imgBuffer[wireless_ardusub::teleop_v3::MAX_IMG_SIZE];

  std::thread _hrovMsgParserWorker, _msgSenderWorker;

  // FUNCTIONS
  bool RisingEdge(const sensor_msgs::Joy::ConstPtr &joy, int index);
  void SetArming(bool armed);
  void CmdTakeoffLand(bool takeoff);
  int8_t ComputeAxisValue(const sensor_msgs::Joy::ConstPtr &joy, int index);
  uint16_t MapToPpm(double in);
  void ConfigCallback(wireless_ardusub::wireless_teleop_joyConfig &update,
                      uint32_t level);
  void JoyCallback(const sensor_msgs::Joy::ConstPtr &joy);
  void NewSettingsReceived(
      const merbots_whrov_msgs::hrov_settingsConstPtr &_settings);
  void NotifyNewHROVState(bool _hrovReady, int _requestedOID, bool cancelled);
  void StartOrderActionServer() { _orderActionServer.start(); }
  bool CancelRequested(void);
  void CancelLastOrder(void);
  void ActionWorker(const merbots_whrov_msgs::OrderGoalConstPtr &goal);
  void StartWorkers();
};

void OperatorController::StartWorkers() {
  _hrovMsgParserWorker = std::thread([this]() {
    while (1) {
      std::unique_lock<std::mutex> lock(_currentHROVMessage_mutex);
      while (!_currentHROVMessage_updated) {
        _currentHROVMessage_cond.wait(lock);
      }
      merbots_whrov_msgs::position position;

      auto oid = _currentHROVMessage->GetExpectedOrderSeqNumber();
      auto cancelled = _currentHROVMessage->LastOrderCancelledFlag();
      auto ready = _currentHROVMessage->Ready();
      position.roll = _currentHROVMessage->GetX();
      position.pitch = _currentHROVMessage->GetY();
      position.altitude = _currentHROVMessage->GetZ();
      position.orientation = _currentHROVMessage->GetYaw();
      _currentHROVMessage_updated = false;
      lock.unlock();

      _currentHROVState_pub.publish(position);
      NotifyNewHROVState(ready, oid, cancelled);
    }
  });
  _hrovMsgParserWorker.detach();

  _msgSenderWorker = std::thread([this]() {
    while (1) {
      std::unique_lock<std::mutex> lock(_currentOperatorMessage_mutex);
      while (!_currentOperatorMessage_updated) {
        _currentOperatorMessage_cond.wait(lock);
      }
      _node->SetDesiredState(_currentOperatorMessage->GetBuffer());
      _currentOperatorMessage_updated = false;
    }
  });
  _msgSenderWorker.detach();
}

OperatorController::OperatorController(ros::NodeHandle &nh,
                                       Ptr<CommsDevice> stream)
    : _orderActionServer(
          nh, "order", boost::bind(&OperatorController::ActionWorker, this, _1),
          false),
      _nh(nh) {
  // connect dynamic reconfigure
  dynamic_reconfigure::Server<
      wireless_ardusub::wireless_teleop_joyConfig>::CallbackType f;
  f = boost::bind(&OperatorController::ConfigCallback, this, _1, _2);
  _server.setCallback(f);

  // connects subs and pubs
  _joy_sub = _nh.subscribe<sensor_msgs::Joy>(
      "/joy", 1, &OperatorController::JoyCallback, this);

  // initialize state variables
  _camera_tilt = 1500;
  _initLT = false;
  _initRT = false;
  SetLogName("TeleopJoy");
  Log->info("Sender initialized");

  _encodedImgMsg.img.reserve(wireless_ardusub::teleop_v3::MAX_IMG_SIZE);

  _currentHROVState_pub =
      _nh.advertise<merbots_whrov_msgs::position>("current_hrov_position", 1);

  _encodedImage_pub =
      _nh.advertise<image_utils_ros_msgs::EncodedImg>("encoded_image", 1);

  _currentSettings_sub = _nh.subscribe<merbots_whrov_msgs::hrov_settings>(
      "desired_hrov_settings", 1,
      boost::bind(&OperatorController::NewSettingsReceived, this, _1)); //,

  _currentOperatorMessage = OperatorMessageV2::Build();
  _currentHROVMessage = HROVMessage::BuildHROVMessage();

  _settings = HROVSettingsV2::Build();
  _teleopOrder = TeleopOrder::Build();
  _node = CreateObject<Operator>(stream);
  _node->SetLogLevel(LogLevel::info);
  _node->SetMaxImageTrunkLength(50);
  _node->SetRxStateSize(HROVMessage::MessageLength);
  _node->SetTxStateSize(OperatorMessageV2::MessageLength);

  _node->SetImageReceivedCallback([this](Operator &op) {
    Log->info("New Image received!");
    int encodedImgSize;
    encodedImgSize = op.GetLastReceivedImage(_imgBuffer);
    _encodedImgMsg.img.resize(encodedImgSize);
    memcpy(_encodedImgMsg.img.data(), _imgBuffer, encodedImgSize);
    _encodedImage_pub.publish(_encodedImgMsg);
  });

  _node->SetStateReceivedCallback([this](Operator &op) {
    Log->info("New state received!");
    _currentHROVMessage_mutex.lock();
    _node->GetLastConfirmedState(_currentHROVMessage->GetBuffer());
    _currentHROVMessage_mutex.unlock();
    _currentHROVMessage_updated = true;
    _currentHROVMessage_cond.notify_one();
  });

  _node->Start();
  StartOrderActionServer();
  StartWorkers();
}

void OperatorController::NotifyNewHROVState(bool _hrovReady, int _requestedOID,
                                            bool cancelled) {
  _hrovState_mutex.lock();
  _hrovReady = _hrovReady;
  _requestedOID = _requestedOID;
  _lastOrderCancelled = cancelled;
  _hrovState_mutex.unlock();
  _hrovState_updated_cond.notify_one();
}

void OperatorController::Spin() {
  ros::Rate loop(_config.pub_rate);

  while (ros::ok()) {
    // call all waiting callbacks
    ros::spinOnce();

    // enforce a max publish rate
    loop.sleep();
  }
}

void OperatorController::NewSettingsReceived(
    const merbots_whrov_msgs::hrov_settingsConstPtr &msg) {
  Log->info("New settings received");
  _settings->SetFromROSMsg(msg);
  _currentOperatorMessage_mutex.lock();
  _currentOperatorMessage->SetSettings(_settings);
  _currentOperatorMessage_mutex.unlock();
  _currentOperatorMessage_updated = true;
  _currentOperatorMessage_cond.notify_one();
}

void OperatorController::ConfigCallback(
    wireless_ardusub::wireless_teleop_joyConfig &update, uint32_t level) {
  ROS_INFO("reconfigure request received");
  _config = update;
}

bool OperatorController::RisingEdge(const sensor_msgs::Joy::ConstPtr &joy,
                                    int index) {
  return (joy->buttons[index] == 1 && _previous_buttons[index] == 0);
}

void OperatorController::SetArming(bool arm) {
  // Arm/disarm method following:
  // https://github.com/mavlink/qgroundcontrol/issues/590
  // https://pixhawk.ethz.ch/mavlink/#MAV_CMD_COMPONENT_ARM_DISARM
  if (arm) {
    Debug("armed");
    _teleopOrder->Arm(true);
    _teleopOrder->DisArm(false);
  } else {
    Debug("disarmed");
    _teleopOrder->Arm(false);
    _teleopOrder->DisArm(true);
  }
}

void OperatorController::CmdTakeoffLand(bool takeoff) {
  // https://pixhawk.ethz.ch/mavlink/#MAV_CMD_NAV_LAND_LOCAL
  if (takeoff) {
    Debug("takeoff");
  } else {
    Debug("land");
  }
}

int8_t
OperatorController::ComputeAxisValue(const sensor_msgs::Joy::ConstPtr &joy,
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

void OperatorController::JoyCallback(const sensor_msgs::Joy::ConstPtr &joy) {
  // init previous_buttons
  if (_previous_buttons.size() != joy->buttons.size()) {
    _previous_buttons = std::vector<int>(joy->buttons);
  }

  // arm, disarm
  if (RisingEdge(joy, _config.disarm_button)) {
    SetArming(false);
  } else if (RisingEdge(joy, _config.arm_button)) {
    SetArming(true);
  }

  auto x = ComputeAxisValue(joy, _config.x_axis);
  auto y = ComputeAxisValue(joy, _config.y_axis);
  auto z = ComputeAxisValue(joy, _config.z_axis);
  auto r = ComputeAxisValue(joy, _config.wz_axis);

  _teleopOrder->SetX(x);
  _teleopOrder->SetY(-1 * y);
  _teleopOrder->SetZ(z);
  _teleopOrder->SetR(-1 * r);

  // mode switching
  if (RisingEdge(joy, _config.stabilize_button)) {
    _teleopOrder->SetFlyMode(FLY_MODE::STABILIZE);
  } else if (RisingEdge(joy, _config.alt_hold_button)) {
    _teleopOrder->SetFlyMode(FLY_MODE::DEPTH_HOLD);
  } else if (RisingEdge(joy, 0)) {
    _teleopOrder->SetFlyMode(FLY_MODE::MANUAL);
  }
  std::string modeName = "";
  switch (_teleopOrder->GetFlyMode()) {
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
            _teleopOrder->GetX(), _teleopOrder->GetY(), _teleopOrder->GetZ(),
            _teleopOrder->GetR(), _teleopOrder->Arm() ? "true" : "false",
            modeName);

  // remember current button states for future comparison
  _previous_buttons = std::vector<int>(joy->buttons);
}

bool OperatorController::CancelRequested(void) {
  if (_orderActionServer.isPreemptRequested() || !ros::ok()) {
    _orderActionServer.setPreempted();
    return true;
  }
  return false;
}

void OperatorController::CancelLastOrder(void) {
  // Puede que la orden se haya enviado ya, por lo que hay que cancelarla
  _currentOperatorMessage_mutex.lock();
  _currentOperatorMessage->CancelLastOrderFlag(true);
  _currentOperatorMessage->SetOrderSeqNumber(_requestedOID);
  _currentOperatorMessage->SetNoOrder();
  _currentOperatorMessage_updated = true;
  _currentOperatorMessage_mutex.unlock();
  _currentOperatorMessage_cond.notify_one();
}

void OperatorController::ActionWorker(
    const merbots_whrov_msgs::OrderGoalConstPtr &goal) {
  switch (goal->type) {
  case 0: // HEADING
  {
    Log->info("Heading order received");

    std::unique_lock<std::mutex> lock(_hrovState_mutex);
    _orderFeedback.percent_complete = 5;
    _orderFeedback.message =
        "Waiting for the ROV to get ready to handle a new order...";
    _orderActionServer.publishFeedback(_orderFeedback);

    while (_transmittingOrder) {
      _hrovState_updated_cond.wait_for(lock, std::chrono::seconds(1));
      if (CancelRequested()) {
        _orderFeedback.percent_complete = 100;
        _orderFeedback.message = "Last order cancelled";
        _orderActionServer.publishFeedback(_orderFeedback);
        lock.unlock();
        return;
      }
    }
    // Si el robot recibe una nueva orden con el OID
    // que espera y está en curso alguna orden, cancelará la orden actual para
    // ejecutar la nueva.
    _transmittingOrder = true;
    _oid = _requestedOID;
    lock.unlock();

    _orderFeedback.percent_complete = 5;
    _orderFeedback.message = "Transmitting heading order...";
    _orderActionServer.publishFeedback(_orderFeedback);

    _currentOperatorMessage_mutex.lock();
    _currentOperatorMessage->SetOrderSeqNumber(_oid);
    _currentOperatorMessage->SetKeepOrientationOrder(
        goal->keep_heading_degrees);
    _currentOperatorMessage->CancelLastOrderFlag(false);
    _currentOperatorMessage_updated = true;
    _currentOperatorMessage_mutex.unlock();
    _currentOperatorMessage_cond.notify_one();

    _orderFeedback.percent_complete = 15;
    _orderFeedback.message = "Waiting for the order acknowledgment...";
    _orderActionServer.publishFeedback(_orderFeedback);

    int nextOID = wireless_ardusub::HROVMessage::GetNextOrderSeqNumber(_oid);
    lock.lock();

    while (_requestedOID != nextOID) {
      _hrovState_updated_cond.wait_for(lock, std::chrono::seconds(1));
      if (CancelRequested()) {
        _orderFeedback.percent_complete = 50;
        _orderFeedback.message = "Cancelling last order...";
        _orderActionServer.publishFeedback(_orderFeedback);
        do {
          CancelLastOrder();
          _hrovState_updated_cond.wait_for(lock, std::chrono::seconds(1));
        } while (!_lastOrderCancelled);

        _currentOperatorMessage_mutex.lock();
        _currentOperatorMessage->CancelLastOrderFlag(false);
        _currentOperatorMessage_updated = true;
        _currentOperatorMessage_mutex.unlock();
        _currentOperatorMessage_cond.notify_one();

        _orderFeedback.percent_complete = 100;
        _orderFeedback.message = "Last order cancelled";
        _orderActionServer.publishFeedback(_orderFeedback);
        _transmittingOrder = false;
        lock.unlock();
        return;
      }
    }

    _currentOperatorMessage_mutex.lock();
    _currentOperatorMessage->SetNoOrder();
    _currentOperatorMessage_updated = true;
    _currentOperatorMessage_mutex.unlock();
    _currentOperatorMessage_cond.notify_one();

    _orderFeedback.percent_complete = 70;
    _orderFeedback.message = "ROV received the order.";
    _orderActionServer.publishFeedback(_orderFeedback);

    while (!_hrovReady) {
      _hrovState_updated_cond.wait_for(lock, std::chrono::seconds(1));
      if (CancelRequested()) {
        _orderFeedback.percent_complete = 50;
        _orderFeedback.message = "Cancelling last order...";
        _orderActionServer.publishFeedback(_orderFeedback);
        do {
          CancelLastOrder();
          _hrovState_updated_cond.wait_for(lock, std::chrono::seconds(1));
        } while (!_lastOrderCancelled);

        _currentOperatorMessage_mutex.lock();
        _currentOperatorMessage->CancelLastOrderFlag(false);
        _currentOperatorMessage_updated = true;
        _currentOperatorMessage_mutex.unlock();
        _currentOperatorMessage_cond.notify_one();

        _orderFeedback.percent_complete = 100;
        _orderFeedback.message = "Last order cancelled";
        _orderActionServer.publishFeedback(_orderFeedback);
        _transmittingOrder = false;
        _orderResult.success = true;
        _orderActionServer.setSucceeded(_orderResult);
        lock.unlock();
        return;
      }
    }

    _transmittingOrder = false;
    lock.unlock();

    _orderFeedback.percent_complete = 100;
    _orderFeedback.message = "Received last order completion confirmation.";
    _orderActionServer.publishFeedback(_orderFeedback);
    _orderResult.success = true;
    _orderActionServer.setSucceeded(_orderResult);
    break;
  }
  case 1: // HOLD TIME
  {
    Log->info("Hold image channel order received");
    break;
  }
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "operator_control_v3");
  Log = CreateLogger("operator_control_v3:main");
  Log->Info("Init");
  GetParams();

  Log->LogToConsole(params.log2Console);
  auto stream = CreateObject<dccomms_utils::S100Stream>(
      params.serialPort, SerialPortStream::BAUD_2400);
  stream->Open();

  ros::NodeHandle nh;
  OperatorController teleop(nh, stream);
  teleop.LogToConsole(params.log2Console);

  teleop.Spin();
  return 0;
}
