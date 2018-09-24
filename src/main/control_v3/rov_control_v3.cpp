#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/Geoid.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include <chrono>
#include <cpplogging/cpplogging.h>
#include <dccomms_utils/S100Stream.h>
#include <eigen_conversions/eigen_msg.h>
#include <image_utils_ros_msgs/EncodedImg.h>
#include <image_utils_ros_msgs/EncodingConfig.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_srvs/Empty.h> //For /cola2_control/disable_goto service
#include <telerobotics/HROVMessageV2.h>
#include <telerobotics/OperatorMessageV2.h>
#include <telerobotics/ROV.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h> //for position relative to ROV frame
#include <tf/transform_listener.h>
#include <wireless_ardusub/Constants.h>
#include <wireless_ardusub/utils.hpp>

#include <tf_conversions/tf_eigen.h>
#include <wireless_ardusub/pid.h>

// Merbots
#include <merbots_whrov_msgs/hrov_settings.h>
#include <merbots_whrov_msgs/movement.h>
#include <merbots_whrov_msgs/position.h>
// EndMerbots

// cola2
#include <cola2_msgs/Goto.h> //For /cola2_control/enable_goto service
// end cola2

// auv_msgs
#include <auv_msgs/NavSts.h>
// end auv_msgs

#include <nav_msgs/Odometry.h>

using namespace cpplogging;
using namespace std::chrono_literals;
using namespace telerobotics;
using namespace std;
using namespace telerobotics;
using namespace wireless_ardusub;

struct Params {
  std::string serialPort, masterUri, dccommsId;
  bool log2Console, log2File;
};

struct HROVPose {
  double yaw, pitch, roll, Z, X, Y, heading;
};

static ros::ServiceClient cola2Control, cola2ControlCancel;
static ros::Subscriber cola2Navigation_sub, sitlOdom_sub;
static auv_msgs::NavStsPtr navData;

static LoggerPtr Log;
static Params params;

static uint16_t localPort = 14550;

static dccomms::Ptr<OperatorMessageV2> currentOperatorMessage =
    dccomms::CreateObject<OperatorMessageV2>();
static bool currentOperatorMessage_updated;
static std::mutex currentOperatorMessage_mutex;
static std::condition_variable currentOperatorMessage_cond;
static TeleopOrderPtr lastOrder;
static int xVel;
static int yVel;
static int zVel;
static int rVel;

static dccomms::Ptr<HROVMessageV2> currentHROVMessageV2 =
    dccomms::CreateObject<HROVMessageV2>();
static bool currentHROVMessage_updated;
static std::mutex currentHROVMessage_mutex;
static std::condition_variable currentHROVMessage_cond;

static std::thread operatorMsgParserWorker;
static std::thread messageSenderWorker;
static std::thread keepOrientationWorker;
static std::thread holdChannelWorker;

static ros::Subscriber encodedImage_sub;
static ros::Publisher encodingConfig_pub;
static dccomms::Ptr<ROV> commsNode;

static image_utils_ros_msgs::EncodingConfig emsg;
static int lastImageSize = -1;

static uint16_t desiredOrientation;
static bool keepOrientation;
static std::mutex keepOrientation_mutex;
static std::condition_variable keepOrientation_cond;

static uint16_t holdChannelSeconds;
static bool holdChannelReceived;
static std::mutex holdChannel_mutex;
static std::condition_variable holdChannel_cond;

static bool cancelLastOrderFlag;

static bool executingOrder;
static std::mutex executingOrder_mutex;
static std::condition_variable executingOrder_cond;

static HROVPose currentHROVPose;
static std::mutex currentHROVPose_mutex;
static std::condition_variable currentHROVPose_cond;
static bool currentHROVPose_updated;
static bool home_set;
static bool armed;
static int32_t latitude, longitude;
static std::mutex gps_mutex, ned_mutex;
static std::condition_variable ned_cond;
static double ned_x, ned_y, ned_z;
static std::mutex attitude_mutex;
static tfScalar g_yaw, g_pitch, g_roll;

PID pid = PID(0.1, 180, -180, 0.1, 0.01, 0);

static ARDUSUB_NAV_MODE lastReceivedMode;

struct PoseRegister {
  tf::Vector3 position;
  tf::Quaternion orientation;
};

static bool communication_lost = false;
static std::list<PoseRegister> poseStack;
static int maxPoseStackSize = 4096;
static ros::Publisher pose_pub;

void notifyHROVMessageUpdated() {
  currentHROVMessage_updated = true;
  currentHROVMessage_cond.notify_one();
}
void notifyLastOrderCancellationAndRobotReady() {
  executingOrder_mutex.lock();
  executingOrder = false;
  executingOrder_cond.notify_one();
  executingOrder_mutex.unlock();

  currentHROVMessage_mutex.lock();
  currentHROVMessageV2->LastOrderCancelledFlag(true);
  currentHROVMessageV2->Ready(true);
  notifyHROVMessageUpdated();
  currentHROVMessage_mutex.unlock();
}

void notifyROVBusy() {
  executingOrder_mutex.lock();
  executingOrder = true;
  executingOrder_cond.notify_one();
  executingOrder_mutex.unlock();

  currentHROVMessage_mutex.lock();
  currentHROVMessageV2->Ready(false);
  notifyHROVMessageUpdated();
  currentHROVMessage_mutex.unlock();
}

void notifyROVReady() {
  executingOrder_mutex.lock();
  executingOrder = false;
  executingOrder_cond.notify_one();
  executingOrder_mutex.unlock();

  currentHROVMessage_mutex.lock();
  currentHROVMessageV2->Ready(true);
  notifyHROVMessageUpdated();
  currentHROVMessage_mutex.unlock();
}

double getJoyAxisNormalized(int x) { return 200. / 256 * x; }
double arduSubXYR(double per) { return per / 0.1; }
double arduSubZ(double per) { return (per + 100) / 0.2; }

void stopRobot() {
  // control->SetManualControl(x, y, z, r);
}

void moveYaw(double per) {
  if (lastOrder) {
    // rVel = ceil(arduSubXYR(per));
    // control->SetManualControl(xVel, yVel, zVel, rVel);
  }
}

void mockOrderWork() {
  notifyROVBusy();

  // work step 0
  this_thread::sleep_for(chrono::milliseconds(2000));
  if (cancelLastOrderFlag) {
    notifyLastOrderCancellationAndRobotReady();
    return;
  }

  // work step 1
  this_thread::sleep_for(chrono::milliseconds(2000));
  if (cancelLastOrderFlag) {
    notifyLastOrderCancellationAndRobotReady();
    return;
  }

  // work step 2
  this_thread::sleep_for(chrono::milliseconds(2000));
  if (cancelLastOrderFlag) {
    notifyLastOrderCancellationAndRobotReady();
    return;
  }

  // work step 3
  this_thread::sleep_for(chrono::milliseconds(2000));

  notifyROVReady();
}

void holdChannelWork() {
  notifyROVBusy();
  int seconds = holdChannelSeconds + 2;
  commsNode->HoldChannel(true);
  this_thread::sleep_for(chrono::seconds(seconds));
  commsNode->HoldChannel(false);
  notifyROVReady();
}

void holdChannelWorkLoop() {
  while (1) {
    Log->Debug("Waiting hold channel order");
    std::unique_lock<std::mutex> lock(holdChannel_mutex);
    while (!holdChannelReceived)
      holdChannel_cond.wait(lock);
    Log->Debug("Hold channel received");
    holdChannelReceived = false;
    holdChannelWork();
    Log->Debug("Hold channel finished");
  }
}

void updateImageSettings(const HROVSettingsV2Ptr &lastSettings) {
  notifyROVBusy();
  Log->Info("Current image settings:\n"
            "\t(x0,y0): ({},{})\n"
            "\t(x1,y1): ({},{})\n)"
            "\tsize: {} bytes"
            "\tROI shift: {}"
            "\tEncode mono version: {}",
            lastSettings->GetROIX0(), lastSettings->GetROIY0(),
            lastSettings->GetROIX1(), lastSettings->GetROIY1(),
            lastSettings->GetImgSize(), lastSettings->GetROIShift(),
            lastSettings->EncodeMonoVersion() ? "true" : "false");

  emsg.max_size = lastSettings->GetImgSize();
  emsg.shift = lastSettings->GetROIShift();
  emsg.x0 = lastSettings->GetROIX0();
  emsg.y0 = lastSettings->GetROIY0();
  emsg.x1 = lastSettings->GetROIX1();
  emsg.y1 = lastSettings->GetROIY1();
  emsg.encode8bversion = lastSettings->EncodeMonoVersion();

  encodingConfig_pub.publish(emsg);
  notifyROVReady();
}

void cancelLastOrder() {
  std::unique_lock<std::mutex> lock(executingOrder_mutex);
  while (executingOrder) {
    cancelLastOrderFlag = true;
    executingOrder_cond.wait(lock);
  }
  cancelLastOrderFlag = false;
  lock.unlock();
}

void cancelCurrentMoveOrder() {
  Log->Info("Cancel current order of movement");
  std_srvs::Empty srv;
  if (cola2ControlCancel.call(srv)) {
    Log->Info("Last order cancelled");
  } else {
    Log->Info("Failed to call service /cola2_control/disable_goto");
  }
}

void sendGoToLocalNED(double x, double y, double z, double yaw) {
  cancelCurrentMoveOrder();
  tf::Quaternion ned_orientation;
  ned_orientation.setRPY(0, 0, yaw);
  tf::Vector3 ned_position(x, y, z);
  tf::StampedTransform ned_tf;
  ned_tf.setOrigin(ned_position);
  ned_tf.setRotation(ned_orientation.normalize());

  cola2_msgs::Goto srv;

  srv.request.yaw = yaw;
  srv.request.position.x = x;
  srv.request.position.y = y;
  srv.request.position.z = z;

  srv.request.blocking = false;
  srv.request.keep_position = true;
  srv.request.position_tolerance.x = 0.4;
  srv.request.position_tolerance.y = 0.4;
  srv.request.position_tolerance.z = 0.4;
  srv.request.altitude_mode = false;
  srv.request.priority = 10;
  srv.request.reference = srv.request.REFERENCE_NED;

  if (cola2Control.call(srv)) {
    Log->Info("Service call success. Result: {}",
              srv.response.success ? "success" : "failed");
  } else {
    Log->Error("Service call failed");
  }
}

void handleNewOrder() {
  currentHROVMessage_mutex.lock();
  auto eSeq = currentHROVMessageV2->GetExpectedOrderSeqNumber();
  currentHROVMessage_mutex.unlock();

  auto orderType = currentOperatorMessage->GetOrderType();
  auto cSeq = currentOperatorMessage->GetOrderSeqNumber();
  auto _cancelLastOrder = currentOperatorMessage->CancelLastOrder();

  if (eSeq == cSeq) {
    if (_cancelLastOrder) {
      Log->Debug("Cancelation requested");
      cancelLastOrder();
      Log->Debug("Cancelled");
    } else if (orderType != OperatorMessageV2::OrderType::NoOrder) {
      Log->Debug("New order requested. Cancel current order (if any)");
      cancelLastOrder();
      Log->Debug("Cancelled");
      switch (orderType) {
      case OperatorMessageV2::OrderType::HoldChannel: {
        holdChannelSeconds = currentOperatorMessage->GetHoldChannelDuration();
        Log->Info("Received hold channel duration order: {} seconds",
                  holdChannelSeconds);
        holdChannelReceived = true;
        holdChannel_cond.notify_one();
        break;
      }
      case OperatorMessageV2::OrderType::KeepOrientation: {
        desiredOrientation = currentOperatorMessage->GetKeepOrientationValue();
        Log->Info("Received keep orientation order: {} degrees",
                  desiredOrientation);
        keepOrientation = true;
        keepOrientation_cond.notify_one();
        break;
      }
      case OperatorMessageV2::OrderType::DisableKeepOrientation: {
        Log->Info("Received disable keep orientation order");
        keepOrientation = false;
        keepOrientation_cond.notify_one();
        break;
      }
      case OperatorMessageV2::OrderType::UpdateImageSettings: {
        auto lastSettings = currentOperatorMessage->GetImageSettingsOrderCopy();
        updateImageSettings(lastSettings);
        break;
      }
      case OperatorMessageV2::OrderType::GoTo: {
        int x, y, z, heading;

        currentOperatorMessage->GetGoToOrder(x, y, z, heading);
        // We received NED coordinates.
        // see
        // https://github.com/mavlink/mavros/blob/de9f39a719b091b8448214a17d27b3b1c415d0dc/mavros/src/lib/uas_data.cpp#L55

        double x2 = x / 100.;
        double y2 = y / 100.;
        double z2 = z / 100.;

        // heading: 0-360
        double yaw = heading - 360;
        yaw = yaw * M_PI / 180.;

        Log->Info("GoTo: {} ; {} ; {} ; {} ({})", x2, y2, z2, heading, yaw);

        sendGoToLocalNED(x2, y2, z2, yaw);
        break;
      }
      }
      currentHROVMessage_mutex.lock();
      currentHROVMessageV2->IncExpectedOrderSeqNumber();
      currentHROVMessage_mutex.unlock();
      notifyHROVMessageUpdated();
    }
  }
}

void arm(bool v) {
  // We avoid set control->Arm(false) because there is a bug in the ArduSub
  // firmware
  // that adds an offset to the NED position when rearming.
  if (v) {
    // control->Arm(true);
  } else {
    // control->Arm(false);
    stopRobot();
    // control->SetFlyMode(FLY_MODE_R::MANUAL);
  }
  armed = v;
}
void messageSenderWork() {
  while (1) {
    std::unique_lock<std::mutex> lock(currentHROVMessage_mutex);
    while (!currentHROVMessage_updated) {
      currentHROVMessage_cond.wait(lock);
    }
    commsNode->SetCurrentTxState(currentHROVMessageV2->GetBuffer(),
                                 currentHROVMessageV2->GetMsgSize());

    auto oid = currentHROVMessageV2->GetExpectedOrderSeqNumber();
    auto cancelled = currentHROVMessageV2->LastOrderCancelledFlag();
    auto ready = currentHROVMessageV2->Ready();
    auto roll = currentHROVMessageV2->GetRoll();
    auto pitch = currentHROVMessageV2->GetPitch();
    auto x = currentHROVMessageV2->GetX();
    auto y = currentHROVMessageV2->GetY();
    auto altitude = currentHROVMessageV2->GetZ();
    auto heading = currentHROVMessageV2->GetHeading();
    auto navMode = (int)currentHROVMessageV2->GetNavMode();
    auto armed = currentHROVMessageV2->Armed();
    Log->Info("OWN STATE - OID: {} ; CC: {} ; RDY: {} ; HD: {} ; NAV: {} ;"
              "ARMED: {} ; x:y:z: "
              "{} : {} : {}",
              oid, cancelled ? 1 : 0, ready ? 1 : 0, heading, navMode,
              armed ? 1 : 0, x, y, altitude);
    currentHROVMessage_updated = false;
  }
}

void operatorMsgParserWork() {
  while (1) {
    std::unique_lock<std::mutex> lock(currentOperatorMessage_mutex);
    while (!currentOperatorMessage_updated) {
      currentOperatorMessage_cond.wait_for(lock, chrono::milliseconds(5000));
      if (!currentOperatorMessage_updated && !commsNode->HoldingChannel()) {
        if (!lastReceivedMode != ARDUSUB_NAV_MODE::NAV_GUIDED) {
          //          arm(false);
          //          Log->Warn("Heartbeat lost. Stopping robot to avoid
          //          thruster "
          //                    "interferences!");
          Log->Warn("Heartbeat lost! Considering communication lost and "
                    "launching pose recovery");
        }
        // control->SetFlyMode(FLY_MODE_R::GUIDED);
        communication_lost = true;
      }
    }
    communication_lost = false;
    currentOperatorMessage_updated = false;

    OperatorMessageV2::OrderType lastOrderType =
        currentOperatorMessage->GetOrderType();
    if (lastOrderType == OperatorMessageV2::OrderType::Move) {
      lastOrder = currentOperatorMessage->GetMoveOrderCopy();
      arm(lastOrder->Arm());
      lastReceivedMode = lastOrder->GetFlyMode();
      if (armed) {
        std::string modeName = "";
        switch (lastReceivedMode) {
        case ARDUSUB_NAV_MODE::NAV_DEPTH_HOLD:
          modeName = "DEPTH HOLD";
          // control->SetDepthHoldMode();
          break;
        case ARDUSUB_NAV_MODE::NAV_STABILIZE:
          modeName = "STABILIZE";
          // control->SetStabilizeMode();
          break;
        case ARDUSUB_NAV_MODE::NAV_MANUAL:
          modeName = "MANUAL";
          // control->SetManualMode();
          break;
        case ARDUSUB_NAV_MODE::NAV_POS_HOLD:
          modeName = "POS HOLD";
          // control->SetFlyMode(FLY_MODE_R::POS_HOLD);
          break;
        case ARDUSUB_NAV_MODE::NAV_GUIDED:
          modeName = "GUIDED";
          //  control->EnableManualControl(false);
          // control->SetFlyMode(FLY_MODE_R::GUIDED);
          break;
        default:
          break;
        }
        //        if (lastReceivedMode != ARDUSUB_NAV_MODE::NAV_GUIDED) {
        //          control->EnableManualControl(true);
        //        }
        Log->Info(
            "Send order: X: {} ; Y: {} ; Z: {} ; R: {} ; Arm: {} ; Mode: {}",
            lastOrder->GetX(), lastOrder->GetY(), lastOrder->GetZ(),
            lastOrder->GetR(), lastOrder->Arm() ? "true" : "false", modeName);
        int x = lastOrder->GetX();
        int y = lastOrder->GetY();
        int z = lastOrder->GetZ();
        int r = lastOrder->GetR();
        // order:  y = 200/256x
        // z control: y = 200/1000x - 100
        // x,y and r control: y = 200/2000x

        double xNorm = getJoyAxisNormalized(x);
        double yNorm = getJoyAxisNormalized(y);
        double zNorm = getJoyAxisNormalized(z);
        double rNorm = getJoyAxisNormalized(r);

        xVel = ceil(arduSubXYR(xNorm));
        yVel = ceil(arduSubXYR(yNorm));
        zVel = ceil(arduSubZ(zNorm));
        if (!keepOrientation)
          rVel = ceil(arduSubXYR(rNorm));

        //      Log->Info(
        //          "Manual control: X: {} ; Y: {} ; Z: {} ; R: {} ; Arm: {} ;
        //          Mode: {}",
        //          xVel, yVel, zVel, rVel, lastOrder->Arm() ? "true" : "false",
        //          modeName);

        // control->SetManualControl(xVel, yVel, zVel, rVel);
      }
    } else {
      handleNewOrder();
    }
  }
}

void startWorkers() {
  operatorMsgParserWorker = std::thread(operatorMsgParserWork);
  messageSenderWorker = std::thread(messageSenderWork);
  holdChannelWorker = std::thread(holdChannelWorkLoop);

  std::thread poseWorker([&]() {
    PoseRegister reg;
    double error = 0.5;
    bool recoveringLastPose = false;
    while (1) {
      std::unique_lock<std::mutex> lock(ned_mutex);
      ned_cond.wait(lock);
      if (!communication_lost) {
        recoveringLastPose = false;
        // save current position
        reg.position = tf::Vector3(ned_x, ned_y, ned_z);
        reg.orientation.setRPY(0, 0, g_yaw);
        poseStack.push_back(reg);
        if (poseStack.size() > maxPoseStackSize) {
          poseStack.pop_front();
        }
        lock.unlock();
        std::this_thread::sleep_for(chrono::seconds(2));
        Log->Info("Pose stack size: {}", poseStack.size());
      } else {
        // Pose recovery
        double x, y, z, yaw;
        x = ned_x;
        y = ned_y;
        z = ned_z;
        yaw = g_yaw;
        lock.unlock();
        if (poseStack.size() > 0) {
          tf::Vector3 position;
          double yaw, xerr, yerr, zerr;
          bool reached = false;
          do {
            position = poseStack.rbegin()->position;
            yaw = tf::getYaw(poseStack.rbegin()->orientation);
            xerr = abs(position.x() - x);
            yerr = abs(position.y() - y);
            zerr = abs(position.z() - z);
            reached = xerr <= error && yerr <= error && zerr <= error;
            if (reached)
              poseStack.pop_back();
          } while (reached && poseStack.size() > 0);
          Log->Warn("Recovering position: [ {} ; {} ; {} ]", position.x(),
                    position.y(), position.z());
          sendGoToLocalNED(position.x(), position.y(), position.z(), yaw);
          std::this_thread::sleep_for(chrono::seconds(2));
        }
      }
    }
  });
  poseWorker.detach();
}

void handleNewImage(image_utils_ros_msgs::EncodedImgConstPtr msg) {
  if (!commsNode->SendingCurrentImage()) {
    lastImageSize =
        emsg.max_size <= msg->img.size() ? emsg.max_size : msg->img.size();
    if (lastImageSize > 0) {
      Log->Info("TX IMG {}", lastImageSize);
      commsNode->SendImage((void *)msg->img.data(), lastImageSize);
    }
  } else {
    if (lastImageSize != emsg.max_size) {
      commsNode->CancelLastImage();
    }
  }
}

void handleNewNavigationData(const auv_msgs::NavSts::ConstPtr &msg) {
  //  ned_mutex.lock();
  //  // Convert from m to dm
  //  ned_z = msg->position.depth * 100;
  //  ned_x = msg->position.north * 100;
  //  ned_y = msg->position.east * 100;

  //  attitude_mutex.lock();
  //  g_yaw = msg->orientation.yaw;
  //  g_pitch = msg->orientation.pitch;
  //  g_roll = msg->orientation.roll;
  //  attitude_mutex.unlock();
  //  ned_mutex.unlock();
  //  ned_cond.notify_all();

  //  Log->Info("yaw: {} ; pitch: {} ; roll: {}", g_yaw, g_pitch, g_roll);
  //  int rx, ry;
  //  double heading;
  //  rx = telerobotics::utils::GetDiscreteYaw(g_roll);
  //  ry = telerobotics::utils::GetDiscreteYaw(g_pitch);
  //  heading = g_yaw;

  //  rx = rx > 180 ? -(360 - rx) : rx;
  //  ry = ry > 180 ? -(360 - ry) : ry;

  //  heading = heading * (180. / M_PI);
  //  if (heading < 0)
  //    heading = heading + 360;

  //  currentHROVMessage_mutex.lock();
  //  currentHROVMessageV2->SetRoll(rx);
  //  currentHROVMessageV2->SetPitch(ry);

  //  currentHROVMessageV2->SetHeading(static_cast<uint16_t>(std::round(heading)));
  //  currentHROVMessageV2->SetZ(ned_z);
  //  currentHROVMessageV2->SetX(ned_x);
  //  currentHROVMessageV2->SetY(ned_y);
  //  currentHROVMessage_updated = true;
  //  currentHROVMessage_mutex.unlock();
  //  currentHROVMessage_cond.notify_one();
}

void handleSitlOdom(const nav_msgs::Odometry::ConstPtr &msg) {

  ned_mutex.lock();
  // Convert from m to dm
  ned_z = msg->pose.pose.position.z * 100;
  ned_x = msg->pose.pose.position.x * 100;
  ned_y = msg->pose.pose.position.y * 100;

  tf::Quaternion rot(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                     msg->pose.pose.orientation.z,
                     msg->pose.pose.orientation.w);

  attitude_mutex.lock();

  tf::Matrix3x3(rot).getRPY(g_roll, g_pitch, g_yaw);
  attitude_mutex.unlock();
  ned_mutex.unlock();
  ned_cond.notify_all();

  Log->Info("yaw: {} ; pitch: {} ; roll: {}", g_yaw, g_pitch, g_roll);
  int rx, ry;
  double heading;
  rx = telerobotics::utils::GetDiscreteYaw(g_roll);
  ry = telerobotics::utils::GetDiscreteYaw(g_pitch);
  heading = g_yaw;

  rx = rx > 180 ? -(360 - rx) : rx;
  ry = ry > 180 ? -(360 - ry) : ry;

  heading = heading * (180. / M_PI);
  if (heading < 0)
    heading = heading + 360;

  currentHROVMessage_mutex.lock();
  currentHROVMessageV2->SetRoll(rx);
  currentHROVMessageV2->SetPitch(ry);

  currentHROVMessageV2->SetHeading(static_cast<uint16_t>(std::round(heading)));
  currentHROVMessageV2->SetZ(ned_z);
  currentHROVMessageV2->SetX(ned_x);
  currentHROVMessageV2->SetY(ned_y);
  currentHROVMessage_updated = true;
  currentHROVMessage_mutex.unlock();
  currentHROVMessage_cond.notify_one();
}

void initROSInterface(int argc, char **argv) {
  ros::NodeHandle nh;
  encodedImage_sub = nh.subscribe<image_utils_ros_msgs::EncodedImg>(
      "encoded_image", 1, boost::bind(handleNewImage, _1));
  encodingConfig_pub =
      nh.advertise<image_utils_ros_msgs::EncodingConfig>("encoding_config", 1);

  cola2ControlCancel =
      nh.serviceClient<std_srvs::Empty>("/cola2_control/disable_goto");
  cola2Control =
      nh.serviceClient<cola2_msgs::Goto>("/cola2_control/enable_goto");
  cola2Navigation_sub =
      nh.subscribe<auv_msgs::NavSts>("/cola2_navigation/nav_sts_hz", 1,
                                     boost::bind(handleNewNavigationData, _1));

  cola2Navigation_sub = nh.subscribe<nav_msgs::Odometry>(
      "/g500/ros_odom_to_pat", 1, boost::bind(handleSitlOdom, _1));

  pose_pub = nh.advertise<geometry_msgs::Pose>("/debug/pose", 1);
}

int getParams() {
  ros::NodeHandle nh("~");
  std::string dccommsId;
  if (nh.getParam("dccommsId", dccommsId)) {
    Log->Info("dccommsId: {}", dccommsId);
    params.dccommsId = dccommsId;
  } else
    params.dccommsId = "rov";

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

  return 0;
}
int main(int argc, char **argv) {
  Log = CreateLogger("ROVMain");
  Log->Info("Init");
  Log->SetAsyncMode(true);

  //// GET PARAMS
  ros::init(argc, argv, "rov_control");

  if (getParams())
    return 1;

  Log->SetLogLevel(cpplogging::LogLevel::info);
  // Log->FlushLogOn(cpplogging::LogLevel::info);
  Log->LogToConsole(params.log2Console);

  arm(false);

  initROSInterface(argc, argv);
  startWorkers();
  commsNode = dccomms::CreateObject<ROV>();

  commsNode->SetImageTrunkLength(DEFAULT_IMG_TRUNK_LENGTH);
  commsNode->SetAsyncMode();

  if (params.log2File) {
    Log->SetLogFormatter(
        std::make_shared<spdlog::pattern_formatter>("[%T.%F] %v"));
    Log->LogToFile("rov_v3_control");
    // control->LogToFile("rov_v3_gcs");
    commsNode->LogToFile("rov_v3_comms_node");
    commsNode->SetLogFormatter(
        std::make_shared<spdlog::pattern_formatter>("[%T.%F] %v"));
  }

  if (params.serialPort != "service") {
    Log->Info("CommsDevice type: dccomms::SerialPortStream");
    dccomms::Ptr<SerialPortStream> stream =
        dccomms::CreateObject<dccomms::SerialPortStream>(
            params.serialPort, SerialPortStream::BAUD_9600);
    stream->SetHwFlowControl(true);
    stream->Open();
    commsNode->SetComms(stream);
  } else {
    Log->Info("CommsDevice type: dccomms::CommsDeviceService");

    std::string dccommsId = params.dccommsId;
    Log->Info("dccomms ID: {}", dccommsId);

    dccomms::Ptr<IPacketBuilder> pb =
        dccomms::CreateObject<VariableLengthPacketBuilder>();

    dccomms::Ptr<CommsDeviceService> commsService;
    commsService = dccomms::CreateObject<CommsDeviceService>(pb);
    commsService->SetCommsDeviceId(dccommsId);
    commsService->SetLogLevel(LogLevel::info);
    commsService->Start();
    commsNode->SetComms(commsService);
  }

  commsNode->LogToConsole(params.log2Console);
  commsNode->SetLogLevel(LogLevel::info);

  commsNode->SetOrdersReceivedCallback([](ROV &receiver) {
    Log->Info("Orders received!");
    uint8_t state[300];
    receiver.GetCurrentRxState(state);

    currentOperatorMessage_mutex.lock();
    currentOperatorMessage->UpdateFromBuffer(state);
    currentOperatorMessage_mutex.unlock();
    currentOperatorMessage_updated = true;
    currentOperatorMessage_cond.notify_one();

    Log->Info("Orders updated");
  });
  commsNode->Start();

  currentHROVMessageV2->Ready(true);
  currentHROVMessageV2->SetNavMode(ARDUSUB_NAV_MODE::NAV_GUIDED);
  commsNode->SetCurrentTxState(currentHROVMessageV2->GetBuffer(),
                               currentHROVMessageV2->GetMsgSize());

  try {
    ros::Rate rate(30); // 30 hz
    while (ros::ok()) {
      ros::spinOnce();
      rate.sleep();
    }
  } catch (std::exception &e) {
    Log->Error("Exception: {}", e.what());
    exit(1);
  }
  return 0;
}
