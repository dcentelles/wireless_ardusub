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
#include <dccomms_packets/VariableLengthPacket.h>
#include <dccomms_utils/S100Stream.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/Pose.h>
#include <image_utils_ros_msgs/EncodedImg.h>
#include <merbots_whrov_msgs/OrderAction.h>
#include <merbots_whrov_msgs/state.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <telerobotics/HROVMessageV2.h>
#include <telerobotics/Operator.h>
#include <telerobotics/OperatorMessageV2.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <vector>
#include <wireless_ardusub/Constants.h>
#include <wireless_ardusub/wireless_teleop_joyConfig.h>

using namespace cpplogging;
using namespace telerobotics;
using namespace telerobotics;

struct Params {
  std::string serialPort, masterUri, dccommsId;
  bool log2Console, log2File;
  int imageTrunkSize = DEFAULT_IMG_TRUNK_LENGTH;
};

static Params params;
static LoggerPtr Log;

int getParams() {
  ros::NodeHandle nh("~");
  std::string dccommsId;
  if (nh.getParam("dccommsId", dccommsId)) {
    Log->Info("dccommsId: {}", dccommsId);
    params.dccommsId = dccommsId;
  } else
    params.dccommsId = "operator";

  std::string serialPort;
  if (!nh.getParam("port", serialPort)) {
    Log->Info("Using comms service with dccomms Id '{}'", params.dccommsId);
  } else {
    if (serialPort == "service")
      Log->Info("Using comms service with dccomms Id '{}'", params.dccommsId);
    else
      Log->Info("Serial port: {}", serialPort);
  }
  params.serialPort = serialPort;

  char *cmasterUri = getenv("ROS_MASTER_URI");
  std::string masterUri = cmasterUri;
  Log->Info("ROS MASTER URI: {}", masterUri);

  params.masterUri = masterUri;

  bool log2Console;
  if (!nh.getParam("log2Console", log2Console)) {
    bool defaultValue = true;
    Log->Info("log2Console set to default => {}", defaultValue);
    params.log2Console = defaultValue;
  } else {
    Log->Info("log2Console: {}", log2Console);
    params.log2Console = log2Console;
  }

  bool log2File;
  if (!nh.getParam("log2File", log2File)) {
    bool defaultValue = false;
    Log->Info("log2File set to default => {}", defaultValue);
    params.log2File = defaultValue;
  } else {
    Log->Info("log2File: {}", log2File);
    params.log2File = log2File;
  }

  if (!nh.param("imageTrunkSize", params.imageTrunkSize,
                params.imageTrunkSize)) {
    Log->Info("imageTrunkSize set to default => {}", params.imageTrunkSize);
  } else {
    Log->Info("imageTrunkSize: {}", params.imageTrunkSize);
  }

  return 0;
}

class OperatorController : public Logger {
public:
  OperatorController(ros::NodeHandle &nh);
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
  std::mutex _transmittingOrder_mutex;
  std::condition_variable _transmittingOrder_cond;

  std::mutex _hrovState_mutex;
  std::condition_variable _hrovState_updated_cond;

  std::mutex _currentOperatorMessage_mutex;
  Ptr<OperatorMessageV2> _currentOperatorMessage;
  std::condition_variable _currentOperatorMessage_cond;
  bool _currentOperatorMessage_updated;

  std::mutex _currentHROVMessage_mutex;
  Ptr<HROVMessageV2> _currentHROVMessage;
  std::condition_variable _currentHROVMessage_cond;
  bool _currentHROVMessage_updated;

  TeleopOrderPtr _teleopOrder;
  std::mutex _teleopOrder_mutex;
  Ptr<HROVSettingsV2> _settings;
  Ptr<Operator> _node;
  ros::NodeHandle &_nh;
  ros::Publisher _currentHROVState_pub;
  ros::Publisher _encodedImage_pub;
  ros::Publisher _pose_pub;
  dynamic_reconfigure::Server<wireless_ardusub::wireless_teleop_joyConfig>
      _server;
  wireless_ardusub::wireless_teleop_joyConfig _config;
  ros::Subscriber _joy_sub;
  uint16_t _camera_tilt;
  bool _initLT;
  bool _initRT;
  int _lastSeq = -1;
  int _lastOtype = -1;
  std::vector<int> _previous_buttons;
  image_utils_ros_msgs::EncodedImg _encodedImgMsg;
  uint8_t _imgBuffer[telerobotics::MAX_IMG_SIZE];

  tf::StampedTransform _ned_tf;
  std::mutex _ned_tf_mutex;
  std::condition_variable _ned_tf_cond;
  tf::TransformBroadcaster _ned_broadcaster;
  tf::TransformListener _pose_listener;

  std::thread _hrovMsgParserWorker, _msgSenderWorker, _teleopOrderWorker,
      _posePublisher;

  bool _world_ned_set = false;

  // FUNCTIONS
  bool RisingEdge(const sensor_msgs::Joy::ConstPtr &joy, int index);
  void SetArming(bool armed);
  void CmdTakeoffLand(bool takeoff);
  int8_t ComputeAxisValue(const sensor_msgs::Joy::ConstPtr &joy, int index);
  uint16_t MapToPpm(double in);
  void ConfigCallback(wireless_ardusub::wireless_teleop_joyConfig &update,
                      uint32_t level);
  void JoyCallback(const sensor_msgs::Joy::ConstPtr &joy);
  void NotifyNewHROVState(bool _hrovReady, int _requestedOID, bool cancelled);
  void StartOrderActionServer() { _orderActionServer.start(); }
  bool CancelRequested(void);
  void CancelLastOrder(void);
  void ActionWorker(const merbots_whrov_msgs::OrderGoalConstPtr &goal);
  void StartWorkers();

  int GetImageSizeFromNumberOfPackets(int npackets);
};

void OperatorController::StartWorkers() {
  _hrovMsgParserWorker = std::thread([this]() {
    while (1) {
      std::unique_lock<std::mutex> lock(_currentHROVMessage_mutex);
      while (!_currentHROVMessage_updated) {
        _currentHROVMessage_cond.wait(lock);
      }
      merbots_whrov_msgs::state state;

      auto oid = _currentHROVMessage->GetExpectedOrderSeqNumber();
      auto cancelled = _currentHROVMessage->LastOrderCancelledFlag();
      auto ready = _currentHROVMessage->Ready();
      state.roll = _currentHROVMessage->GetRoll();
      state.pitch = _currentHROVMessage->GetPitch();
      state.x = _currentHROVMessage->GetX();
      state.y = _currentHROVMessage->GetY();
      state.altitude = _currentHROVMessage->GetZ();
      state.heading = _currentHROVMessage->GetHeading();
      state.keepingHeading = _currentHROVMessage->KeepingHeadingFlag();
      state.navMode = (int)_currentHROVMessage->GetNavMode();
      state.armed = _currentHROVMessage->Armed();
      _currentHROVMessage_updated = false;
      lock.unlock();

      _currentHROVState_pub.publish(state);

      tf::Vector3 position(state.x / 100., state.y / 100.,
                           state.altitude / 100.);

      // heading: 0-360

      double yaw = state.heading;
      yaw -= 360;
      yaw = yaw * M_PI / 180.;

      Info("CONFIRMED STATE - OID: {} ; CC: {} ; RDY: {} ; HD: {} ({}); NAV: "
           "{} ; "
           "ARMED: {} ; x:y:z: "
           "{} : {} : {}",
           oid, cancelled ? 1 : 0, ready ? 1 : 0, state.heading, yaw,
           state.navMode, state.armed ? 1 : 0, state.x, state.y,
           state.altitude);

      tf::Quaternion orientation = tf::createQuaternionFromYaw(yaw);
      _ned_tf_mutex.lock();
      _ned_tf.setOrigin(position);
      _ned_tf.setRotation(orientation);
      _ned_tf_cond.notify_one();
      _ned_tf_mutex.unlock();

      NotifyNewHROVState(ready, oid, cancelled);
    }
  });

  _msgSenderWorker = std::thread([this]() {
    while (1) {
      std::unique_lock<std::mutex> lock(_currentOperatorMessage_mutex);
      while (!_currentOperatorMessage_updated) {
        _currentOperatorMessage_cond.wait(lock);
      }
      auto seq = _currentOperatorMessage->GetOrderSeqNumber();
      if (_lastSeq != seq) {
        auto otype = _currentOperatorMessage->GetOrderType();
        Info("OWN STATE: SQ: {} ; OT: {}", seq, otype);
        _lastSeq = seq;
      }
      _node->SetTxState(_currentOperatorMessage->GetBuffer(),
                        _currentOperatorMessage->GetMsgSize());
      _currentOperatorMessage_updated = false;
    }
  });

  _teleopOrderWorker = std::thread([this]() {
    while (1) {
      std::this_thread::sleep_for(chrono::milliseconds(20));
      std::unique_lock<std::mutex> lock(_transmittingOrder_mutex);
      while (_transmittingOrder) {
        _transmittingOrder_cond.wait(lock);
      }
      _currentOperatorMessage_mutex.lock();
      _currentOperatorMessage->SetMoveOrder(_teleopOrder);
      _currentOperatorMessage_mutex.unlock();
      _currentOperatorMessage_updated = true;
      _currentOperatorMessage_cond.notify_one();
    }
  });

  _posePublisher = std::thread([this]() {
    tf::StampedTransform world_ned_tf;
    while (1) {
      if (_world_ned_set) {
        std::this_thread::sleep_for(chrono::milliseconds(20));
        std::unique_lock<std::mutex> lock(_ned_tf_mutex);
        _ned_tf_cond.wait(lock);

        tf::Transform wMv = world_ned_tf * _ned_tf;
        tf::Vector3 position = wMv.getOrigin();
        tf::Quaternion orientation = wMv.getRotation();
        geometry_msgs::Pose msg;
        msg.position.x = position.x();
        msg.position.y = position.y();
        msg.position.z = position.z();
        msg.orientation.x = orientation.x();
        msg.orientation.y = orientation.y();
        msg.orientation.z = orientation.z();
        msg.orientation.w = orientation.w();
        _pose_pub.publish(msg);
      } else {
        try {
          _pose_listener.waitForTransform("world", "local_origin_ned",
                                          ros::Time(0), ros::Duration(1));
          _pose_listener.lookupTransform("world", "local_origin_ned",
                                         ros::Time(0), world_ned_tf);
          _world_ned_set = true;
        } catch (tf::TransformException &e) {
          ROS_ERROR("Not able to lookup transform: %s", e.what());
          _world_ned_set = false;
        };
      }
    }
  });
}

OperatorController::OperatorController(ros::NodeHandle &nh)
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
  SetLogFormatter(std::make_shared<spdlog::pattern_formatter>("%D %T.%F %v"));
  SetAsyncMode(true);
  Log->info("Sender initialized");

  _encodedImgMsg.img.reserve(telerobotics::MAX_IMG_SIZE);

  _currentHROVState_pub =
      _nh.advertise<merbots_whrov_msgs::state>("current_hrov_state", 1);

  _encodedImage_pub =
      _nh.advertise<image_utils_ros_msgs::EncodedImg>("encoded_image", 1);

  _pose_pub = _nh.advertise<geometry_msgs::Pose>("/bluerov2/pose", 1);

  _currentOperatorMessage = OperatorMessageV2::Build();
  _currentHROVMessage = HROVMessageV2::BuildHROVMessageV2();

  _settings = HROVSettingsV2::Build();
  _teleopOrder = TeleopOrder::Build();
  SetArming(false);
  _node = CreateObject<Operator>();
  _node->SetLogLevel(LogLevel::info);
  _node->SetLogFormatter(
      std::make_shared<spdlog::pattern_formatter>("%D %T.%F %v"));

  if (params.log2File) {
    _node->LogToFile("op_v3_comms_node");
  }

  if (params.serialPort != "service") {
    Info("CommsDevice type: dccomms::SerialPortStream");
    dccomms::Ptr<SerialPortStream> stream =
        dccomms::CreateObject<dccomms::SerialPortStream>(
            params.serialPort, SerialPortStream::BAUD_9600);
    stream->SetHwFlowControl(true);
    stream->Open();
    _node->SetComms(stream);
  } else {
    Info("CommsDevice type: dccomms::CommsDeviceService");

    std::string dccommsId = params.dccommsId;
    Info("dccomms ID: {}", dccommsId);

    dccomms::Ptr<IPacketBuilder> pb =
        dccomms::CreateObject<VariableLengthPacketBuilder>();

    dccomms::Ptr<CommsDeviceService> commsService;
    commsService = dccomms::CreateObject<CommsDeviceService>(pb);
    commsService->SetCommsDeviceId(dccommsId);
    commsService->SetLogLevel(LogLevel::info);
    commsService->Start();
    commsService->SetLogFormatter(
        std::make_shared<spdlog::pattern_formatter>("%D %T.%F %v"));
    _node->SetComms(commsService);
  }

  _node->SetImageReceivedCallback([this](Operator &op) {
    int encodedImgSize;
    encodedImgSize = op.GetLastReceivedImage(_imgBuffer);
    Log->info("RX IMG {}", encodedImgSize);
    _encodedImgMsg.img.resize(encodedImgSize);
    memcpy(_encodedImgMsg.img.data(), _imgBuffer, encodedImgSize);
    _encodedImage_pub.publish(_encodedImgMsg);
  });

  _node->SetStateReceivedCallback([this](Operator &op) {
    Log->info("RX ST");
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

void OperatorController::NotifyNewHROVState(bool hrovReady, int requestedOID,
                                            bool cancelled) {
  _hrovState_mutex.lock();
  _hrovReady = hrovReady;
  _requestedOID = requestedOID;
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

  _teleopOrder_mutex.lock();

  _teleopOrder->SetX(x);
  _teleopOrder->SetY(-1 * y);
  _teleopOrder->SetZ(z);
  _teleopOrder->SetR(-1 * r);

  // mode switching
  if (RisingEdge(joy, _config.stabilize_button)) {
    _teleopOrder->SetFlyMode(ARDUSUB_NAV_MODE::NAV_STABILIZE);
  } else if (RisingEdge(joy, _config.alt_hold_button)) {
    _teleopOrder->SetFlyMode(ARDUSUB_NAV_MODE::NAV_DEPTH_HOLD);
  } else if (RisingEdge(joy, 0)) {
    _teleopOrder->SetFlyMode(ARDUSUB_NAV_MODE::NAV_MANUAL);
  } else if (RisingEdge(joy, 2)) {
    _teleopOrder->SetFlyMode(ARDUSUB_NAV_MODE::NAV_POS_HOLD);
  } else if (RisingEdge(joy, 5)) {
    _teleopOrder->SetFlyMode(ARDUSUB_NAV_MODE::NAV_GUIDED);
  }
  std::string modeName = "";
  switch (_teleopOrder->GetFlyMode()) {
  case ARDUSUB_NAV_MODE::NAV_DEPTH_HOLD:
    modeName = "DEPTH HOLD";
    break;
  case ARDUSUB_NAV_MODE::NAV_STABILIZE:
    modeName = "STABILIZE";
    break;
  case ARDUSUB_NAV_MODE::NAV_MANUAL:
    modeName = "MANUAL";
    break;
  case ARDUSUB_NAV_MODE::NAV_POS_HOLD:
    modeName = "POS HOLD";
    break;
  case ARDUSUB_NAV_MODE::NAV_GUIDED:
    modeName = "GUIDED";
    break;
  default:
    break;
  }

  Info("Send order: X: {} ; Y: {} ; Z: {} ; R: {} ; Arm: {} ; Mode: {}",
       _teleopOrder->GetX(), _teleopOrder->GetY(), _teleopOrder->GetZ(),
       _teleopOrder->GetR(), _teleopOrder->Arm() ? "true" : "false", modeName);
  _teleopOrder_mutex.unlock();

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
  int oid = _requestedOID;
  _currentOperatorMessage_mutex.lock();
  _currentOperatorMessage->CancelLastOrder(true);
  _currentOperatorMessage->SetOrderSeqNumber(oid);
  _currentOperatorMessage->SetNoOrder();
  _currentOperatorMessage_updated = true;
  _currentOperatorMessage_mutex.unlock();
  _currentOperatorMessage_cond.notify_one();
  // return HROVMessage::GetNextOrderSeqNumber(oid);
}

void OperatorController::ActionWorker(
    const merbots_whrov_msgs::OrderGoalConstPtr &goal) {

  // std::unique_lock<std::mutex>
  std::unique_lock<std::mutex> hrovStateLock(_hrovState_mutex);
  _orderFeedback.percent_complete = 5;
  _orderFeedback.message =
      "Waiting for the ROV to get ready to handle a new order...";
  _orderActionServer.publishFeedback(_orderFeedback);

  while (_transmittingOrder) {
    _hrovState_updated_cond.wait_for(hrovStateLock, std::chrono::seconds(1));
    if (CancelRequested()) {
      _orderFeedback.percent_complete = 100;
      _orderFeedback.message = "Last order cancelled";
      _orderActionServer.publishFeedback(_orderFeedback);
      hrovStateLock.unlock();
      return;
    }
  }

  std::unique_lock<std::mutex> transmittingOrderLock(_transmittingOrder_mutex);
  _transmittingOrder = true;
  // Si el robot recibe una nueva orden con el OID
  // que espera y está en curso alguna orden, cancelará la orden actual para
  // ejecutar la nueva.
  _oid = _requestedOID;
  hrovStateLock.unlock();

  _orderFeedback.percent_complete = 5;
  _orderFeedback.message = "Transmitting order...";
  _orderActionServer.publishFeedback(_orderFeedback);

  _currentOperatorMessage_mutex.lock();
  _currentOperatorMessage->SetOrderSeqNumber(_oid);
  _currentOperatorMessage_updated = true;

  switch (goal->type) {
  case 0: // HEADING
  {
    Log->info("Heading order received");
    if (goal->keep_heading_degrees > 360)
      _currentOperatorMessage->SetDisableKeepOrientationOrder();
    else
      _currentOperatorMessage->SetEnableKeepOrientationOrder(
          goal->keep_heading_degrees);
    break;
  }
  case 1: // HOLD TIME
  {
    Log->info("Hold image channel order received");
    _currentOperatorMessage->SetHoldChannelOrder(goal->hold_channel_duration +
                                                 5);
    break;
  }
  case 2: { // UPDATE IMAGE SETTINGS
    Log->info("Update image Settings order received");
    auto nbytes = GetImageSizeFromNumberOfPackets(goal->image_config.size);
    _settings->SetSettings(goal->image_config.roi_x0, goal->image_config.roi_y0,
                           goal->image_config.roi_x1, goal->image_config.roi_y1,
                           goal->image_config.roi_shift, nbytes);
    _settings->EncodeMonoVersion(goal->image_config.encode_mono);
    _currentOperatorMessage->SetUpdateImageSettingsOrder(_settings);
    break;
  }
  case 5: { // GOTO START
    double x = goal->x * 100, y = goal->y * 100, z = goal->depth * 100,
           heading = goal->keep_heading_degrees;
    double yaw = heading - 360;
    yaw = yaw * M_PI / 180.;

    Log->info("GoTo: {} ; {} ; {} ; {} ({})", goal->x, goal->y, goal->depth,
              heading, yaw);
    _currentHROVMessage->SetNavMode(ARDUSUB_NAV_MODE::NAV_GUIDED);
    _currentOperatorMessage->SetGoToOrder(x, y, z, heading);
    break;
  }
  }

  _currentOperatorMessage_mutex.unlock();
  _currentOperatorMessage_cond.notify_one();

  _orderFeedback.percent_complete = 15;
  _orderFeedback.message = "Waiting for the order acknowledgment...";
  _orderActionServer.publishFeedback(_orderFeedback);

  int nextOID = telerobotics::HROVMessageV2::GetNextOrderSeqNumber(_oid);
  hrovStateLock.lock();

  while (_requestedOID != nextOID) {
    _hrovState_updated_cond.wait_for(hrovStateLock, std::chrono::seconds(1));
    if (CancelRequested()) {
      _orderFeedback.percent_complete = 50;
      _orderFeedback.message = "Cancelling last order...";
      _orderActionServer.publishFeedback(_orderFeedback);
      do {
        CancelLastOrder();
        _hrovState_updated_cond.wait(hrovStateLock);
      } while (!_lastOrderCancelled && !_hrovReady);

      _currentOperatorMessage_mutex.lock();
      _currentOperatorMessage->CancelLastOrder(false);
      _currentOperatorMessage_updated = true;
      _currentOperatorMessage_mutex.unlock();
      _currentOperatorMessage_cond.notify_one();

      _orderFeedback.percent_complete = 100;
      _orderFeedback.message = "Last order cancelled";
      _orderActionServer.publishFeedback(_orderFeedback);
      _transmittingOrder = false;
      hrovStateLock.unlock();
      transmittingOrderLock.unlock();
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

  switch (goal->type) {
  case 1: // HOLD TIME
  {
    _node->DisableTransmission();
    hrovStateLock.unlock();
    this_thread::sleep_for(chrono::seconds(goal->hold_channel_duration + 5));
    hrovStateLock.lock();
    _node->EnableTransmission();
    break;
  }
  default:
    break;
  }

  while (!_hrovReady) {
    _hrovState_updated_cond.wait_for(hrovStateLock, std::chrono::seconds(1));
    if (CancelRequested()) {
      _orderFeedback.percent_complete = 50;
      _orderFeedback.message = "Cancelling last order...";
      _orderActionServer.publishFeedback(_orderFeedback);
      do {
        CancelLastOrder();
        _hrovState_updated_cond.wait(hrovStateLock);
      } while (!_lastOrderCancelled && !_hrovReady);

      _currentOperatorMessage_mutex.lock();
      _currentOperatorMessage->CancelLastOrder(false);
      _currentOperatorMessage_updated = true;
      _currentOperatorMessage_mutex.unlock();
      _currentOperatorMessage_cond.notify_one();

      _orderFeedback.percent_complete = 100;
      _orderFeedback.message = "Last order cancelled";
      _orderActionServer.publishFeedback(_orderFeedback);
      _transmittingOrder = false;
      _orderResult.success = true;
      _orderActionServer.setSucceeded(_orderResult);
      hrovStateLock.unlock();
      transmittingOrderLock.unlock();
      return;
    }
  }
  hrovStateLock.unlock();

  _orderFeedback.percent_complete = 100;
  _orderFeedback.message = "Received last order completion confirmation.";
  _orderActionServer.publishFeedback(_orderFeedback);
  _orderResult.success = true;
  _orderActionServer.setSucceeded(_orderResult);

  _transmittingOrder = false;
  transmittingOrderLock.unlock();
  _transmittingOrder_cond.notify_one();
}

int OperatorController::GetImageSizeFromNumberOfPackets(int npackets) {
  int fcsSize = 2;
  int res =
      params.imageTrunkSize * (npackets - 1) + params.imageTrunkSize - fcsSize;
  res = res >= 0 ? res : 0;
  return res;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "operator_control_v3");
  Log = CreateLogger("operator_control_v3:main");
  Log->SetLogLevel(LogLevel::info);
  Log->Info("Init");
  Log->SetAsyncMode();
  getParams();

  Log->LogToConsole(params.log2Console);
  ros::NodeHandle nh;
  OperatorController teleop(nh);
  teleop.LogToConsole(params.log2Console);
  teleop.SetLogLevel(LogLevel::info);
  teleop.SetAsyncMode();

  teleop.SetLogFormatter(
      std::make_shared<spdlog::pattern_formatter>("%D %T.%F %v"));
  Log->SetLogFormatter(
      std::make_shared<spdlog::pattern_formatter>("%D %T.%F %v"));
  if (params.log2File) {
    Log->LogToFile("op_v3_main");
    teleop.LogToFile("op_v3_control");
  }

  teleop.Spin();
  return 0;
}
