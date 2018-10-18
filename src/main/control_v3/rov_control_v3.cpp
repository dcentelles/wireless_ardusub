#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/Geoid.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include <chrono>
#include <cpplogging/cpplogging.h>
#include <dccomms_utils/S100Stream.h>
#include <eigen_conversions/eigen_msg.h>
#include <image_utils_ros_msgs/EncodedImg.h>
#include <image_utils_ros_msgs/EncodingConfig.h>
#include <mavlink_cpp/GCSv1.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <telerobotics/HROVMessageV2.h>
#include <telerobotics/OperatorMessageV2.h>
#include <telerobotics/ROV.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <wireless_ardusub/Constants.h>
#include <wireless_ardusub/utils.hpp>

#include <mavros/frame_tf.h>
#include <tf_conversions/tf_eigen.h>
#include <wireless_ardusub/pid.h>

using namespace cpplogging;
using namespace std::chrono_literals;
using namespace telerobotics;
using namespace mavlink_cpp;
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

static LoggerPtr Log;
static Params params;

static uint16_t localPort = 14550;
static dccomms::Ptr<GCSv1> control = dccomms::CreateObject<GCSv1>(localPort);

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
static ros::Publisher setenupos_pub;
static dccomms::Ptr<ROV> commsNode;

static image_utils_ros_msgs::EncodingConfig emsg;
static int lastImageSize = -1;

static int desiredOrientation;
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

PID yawPID = PID(0.1, 100, -100, 1, 0.05, 0.5),
    xPID = PID(0.2, 100, -100, 0.1, 0.01, 0),
    yPID = PID(0.2, 100, -100, 0.1, 0.01, 0),
    zPID = PID(0.2, 100, -100, 0.1, 0.01, 0);

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

// from -127,127  to -100,100
double getJoyAxisNormalized(int x) { return 200. / 256 * x; }
// from -100,100 to -1000,1000
double arduSubXYR(double per) { return per * 10; }
double arduSubZ(double per) { return (per + 100) / 0.2; }

void stopRobot() {
  int x = ceil(arduSubXYR(0));
  int y = ceil(arduSubXYR(0));
  int z = ceil(arduSubZ(0));
  int r = ceil(arduSubXYR(0));
  control->SetManualControl(x, y, z, r);
}

void moveYaw(double per) {
  if (lastOrder) {
    rVel = ceil(arduSubXYR(per));
    control->SetManualControl(xVel, yVel, zVel, rVel);
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

double angleDistance(double src, double dst) {
  // returns values from -180 to 180
  while (src > 360)
    src -= 360;
  while (dst > 360)
    dst -= 360;
  double diff = dst - src;
  double adiff = std::abs(diff);

  // src: 0   --> dst: 190 --> distance: 170 --> I --> -170
  // src: 190 --> dst: 0   --> distance: 170 --> D --> 170
  // src: 350 --> dst: 10  --> distance: 20  --> D --> 20
  // src: 10 --> dst: 350  --> distance: 20 ---> I --> -20
  double distance;
  if (adiff > 180) {
    distance = adiff - 360;
  } else {
    distance = adiff;
  }
  if (diff < 0)
    distance *= -1;

  return distance;
}

void keepHeadingIteration(void) {
  double currentHeading = g_yaw * (180. / M_PI);
  if (currentHeading < 0)
    currentHeading = currentHeading + 360;

  // current heading between 0 and 360
  double diff = angleDistance(currentHeading, desiredOrientation);
  // diff in [-180,180]

  // per = 100/180 * d [-100,100]
  double per = 100. / 180 * diff;

  double vel = yawPID.calculate(0, -1 * per);

  Log->Info("****** GG: {} -- {} -- {} -- {} -- {}", diff, currentHeading, desiredOrientation, per, vel);

//  if(vel >0) vel = 50;
//  else vel = -50;
  moveYaw(vel);
}

void keepHeadingWorkLoop() {
  bool keepingHeading = false;
  while (1) {
    std::unique_lock<std::mutex> lock(keepOrientation_mutex);
    while (!keepOrientation) {
      currentHROVMessage_mutex.lock();
      keepingHeading = false;
      currentHROVMessageV2->KeepingHeadingFlag(false);
      currentHROVMessage_mutex.unlock();
      currentHROVMessage_cond.notify_one();
      Log->Debug("Keep orientation disabled");
      keepOrientation_cond.wait(lock);
      Log->Debug("Keep orientation received!");
    }
    if (!keepingHeading) {
      currentHROVMessage_mutex.lock();
      keepingHeading = true;
      currentHROVMessageV2->KeepingHeadingFlag(true);
      currentHROVMessage_mutex.unlock();
      currentHROVMessage_cond.notify_one();
    }

    keepHeadingIteration();
    this_thread::sleep_for(chrono::milliseconds(100));
  }
}

void setKeepOrientation(int v)
{
    desiredOrientation = v;
    keepOrientation = true;
    keepOrientation_cond.notify_all();
}


void enableKeepOrientation(bool e)
{
    keepOrientation = e;
    keepOrientation_cond.notify_all();
}


static bool guidedMode = false;
static bool goToMission = false;
static std::mutex goToMission_mutex;
static std::condition_variable goToMission_cond;

void guidedModeIteration(void) {}

void goToWorkLoop() {
  while (1) {
    std::unique_lock<std::mutex> lock(goToMission_mutex);
    while (!goToMission) {
      Log->Debug("no go to mission");
      goToMission_cond.wait(lock);
    }
    Log->Debug("go to mission received");
    guidedModeIteration();
    this_thread::sleep_for(chrono::milliseconds(100));
  }
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

void sendGoToLocalNED(double x, double y, double z, double yaw) {

  tf::Quaternion ned_orientation;
  ned_orientation.setRPY(0, 0, yaw);
  tf::Vector3 ned_position(x, y, z);
  tf::StampedTransform ned_tf;
  ned_tf.setOrigin(ned_position);
  ned_tf.setRotation(ned_orientation.normalize());

  // Send mavlink message

  /* Documentation start from bit 1 instead 0;
   * Ignore velocity and accel vectors, yaw rate.
   *
   * In past versions on PX4 there been bug described in #273.
   * If you got similar issue please try update firmware first.
   */
  const uint16_t ignore_all_except_xyz_y = (1 << 11) | (7 << 6) | (7 << 3);

  mavlink_set_position_target_local_ned_t sp;
  sp.time_boot_ms = ros::Time().toNSec() / 1e6;
  sp.coordinate_frame = MAV_FRAME_LOCAL_NED;
  sp.type_mask = ignore_all_except_xyz_y;
  sp.yaw = yaw;
  sp.yaw_rate = 0;
  sp.x = ned_position.x();
  sp.y = ned_position.y();
  sp.z = ned_position.z();
  sp.vx = 0;
  sp.vy = 0;
  sp.vz = 0;
  sp.afx = 0;
  sp.afy = 0;
  sp.afz = 0;

  control->SendSetPositionTargetLocalNED(sp);
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
        setKeepOrientation(heading);
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
    control->Arm(true);
  } else {
    // control->Arm(false);
    stopRobot();
    control->SetFlyMode(FLY_MODE_R::MANUAL);
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
    //    Log->Info("OWN STATE - OID: {} ; CC: {} ; RDY: {} ; HD: {} ; NAV: {}
    //    ;"
    //              "ARMED: {} ; x:y:z: "
    //              "{} : {} : {}",
    //              oid, cancelled ? 1 : 0, ready ? 1 : 0, heading, navMode,
    //              armed ? 1 : 0, x, y, altitude);
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
        control->SetFlyMode(FLY_MODE_R::GUIDED);
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
        case ARDUSUB_NAV_MODE::NAV_POS_HOLD:
          modeName = "POS HOLD";
          control->SetFlyMode(FLY_MODE_R::POS_HOLD);
          break;
        case ARDUSUB_NAV_MODE::NAV_GUIDED:
          modeName = "GUIDED";
          //  control->EnableManualControl(false);
          control->SetFlyMode(FLY_MODE_R::GUIDED);
          break;
        default:
          break;
        }
        //        if (lastReceivedMode != ARDUSUB_NAV_MODE::NAV_GUIDED) {
        //          control->EnableManualControl(true);
        //        }
        //        Log->Info(
        //            "Send order: X: {} ; Y: {} ; Z: {} ; R: {} ; Arm: {} ;
        //            Mode: {}",
        //            lastOrder->GetX(), lastOrder->GetY(), lastOrder->GetZ(),
        //            lastOrder->GetR(), lastOrder->Arm() ? "true" : "false",
        //            modeName);
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

        //        Log->Info("Manual control: X: {} ; Y: {} ; Z: {} ; R: {} ;
        //        Arm: {} ; "
        //                  "Mode  : {} ",
        //                  xVel, yVel, zVel, rVel, lastOrder->Arm() ? "true" :
        //                  "false",
        //                  modeName);

        control->SetManualControl(xVel, yVel, zVel, rVel);
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
  keepOrientationWorker = std::thread(keepHeadingWorkLoop);

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
      //Log->Info("TX IMG {}", lastImageSize);
      commsNode->SendImage((void *)msg->img.data(), lastImageSize);
    }
  } else {
    if (lastImageSize != emsg.max_size) {
      commsNode->CancelLastImage();
    }
  }
}

void handleNewHUDData(const mavlink_vfr_hud_t &msg) {
  currentHROVMessage_mutex.lock();
  currentHROVMessageV2->SetHeading(msg.heading);
  // currentHROVMessageV2->SetZ(-1 * msg.alt * 100);
  currentHROVMessage_updated = true;
  currentHROVMessage_mutex.unlock();
  currentHROVMessage_cond.notify_one();
}

void handleNewNavigationData(const mavlink_attitude_t &attitude) {
  attitude_mutex.lock();
  g_yaw = attitude.yaw;
  g_pitch = attitude.pitch;
  g_roll = attitude.roll;
  attitude_mutex.unlock();

  //Log->Info("yaw: {} ; pitch: {} ; roll: {}", g_yaw, g_pitch, g_roll);
  int rx, ry, rz;
  rx = telerobotics::utils::GetDiscreteYaw(g_roll);
  ry = telerobotics::utils::GetDiscreteYaw(g_pitch);
  rz = telerobotics::utils::GetDiscreteYaw(g_yaw);

  rx = rx > 180 ? -(360 - rx) : rx;
  ry = ry > 180 ? -(360 - ry) : ry;
  // rz = rz > 180 ? -(360-rz) : rz;

  currentHROVMessage_mutex.lock();
  currentHROVMessageV2->SetRoll(rx);
  currentHROVMessageV2->SetPitch(ry);
  currentHROVMessage_updated = true;
  currentHROVMessage_mutex.unlock();
  currentHROVMessage_cond.notify_one();
}

void handleNewArdusubState(const mavlink_heartbeat_t &msg) {
  ARDUSUB_NAV_MODE mode;
  if (armed) {
    switch (msg.custom_mode) {
    case FLY_MODE_R::MANUAL:
      mode = ARDUSUB_NAV_MODE::NAV_MANUAL;
      break;
    case FLY_MODE_R::STABILIZE:
      mode = ARDUSUB_NAV_MODE::NAV_STABILIZE;
      break;
    case FLY_MODE_R::DEPTH_HOLD:
      mode = ARDUSUB_NAV_MODE::NAV_DEPTH_HOLD;
      break;
    case FLY_MODE_R::POS_HOLD:
      mode = ARDUSUB_NAV_MODE::NAV_POS_HOLD;
      break;
    case FLY_MODE_R::GUIDED:
      mode = ARDUSUB_NAV_MODE::NAV_GUIDED;
      break;
    default:
      mode = ARDUSUB_NAV_MODE::NAV_UNKNOWN;
      break;
    }
  } else {
    mode = lastReceivedMode;
  }

  // bool armed = msg.base_mode & MAV_MODE_FLAG_SAFETY_ARMED;
  // armed = msg.base_mode & MAV_MODE_FLAG_SAFETY_ARMED;

  currentHROVMessage_mutex.lock();
  currentHROVMessageV2->SetNavMode(mode);
  currentHROVMessageV2->Armed(armed);
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

  setenupos_pub = nh.advertise<geometry_msgs::PoseStamped>(
      "/mavros/setpoint_position/local", 1);

  pose_pub = nh.advertise<geometry_msgs::Pose>("/debug/pose", 1);

  control->SetVfrHudCb(handleNewHUDData);
  control->SetAttitudeCb(handleNewNavigationData);
  control->SetHeartbeatCb(handleNewArdusubState);
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

  control->SetLogName("GCS");
  control->SetAsyncMode();
  control->SetLogLevel(info);
  arm(false);
  control->LogToConsole(params.log2Console);
  control->EnableGPSMock(false);

  initROSInterface(argc, argv);
  startWorkers();
  commsNode = dccomms::CreateObject<ROV>();

  commsNode->SetImageTrunkLength(DEFAULT_IMG_TRUNK_LENGTH);
  commsNode->SetAsyncMode();

  if (params.log2File) {
    Log->SetLogFormatter(
        std::make_shared<spdlog::pattern_formatter>("[%T.%F] %v"));
    Log->LogToFile("rov_v3_control");
    control->LogToFile("rov_v3_gcs");
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
    //Log->Info("Orders received!");
    uint8_t state[300];
    receiver.GetCurrentRxState(state);

    currentOperatorMessage_mutex.lock();
    currentOperatorMessage->UpdateFromBuffer(state);
    currentOperatorMessage_mutex.unlock();
    currentOperatorMessage_updated = true;
    currentOperatorMessage_cond.notify_one();

    //Log->Info("Orders updated");
  });
  commsNode->Start();

  currentHROVMessageV2->Ready(true);
  commsNode->SetCurrentTxState(currentHROVMessageV2->GetBuffer(),
                               currentHROVMessageV2->GetMsgSize());

  ///// GPS VISION
  tf::TransformListener listener;

  // Constructor for a ellipsoid

  std::shared_ptr<GeographicLib::Geoid> egm96_5;
  egm96_5 = std::make_shared<GeographicLib::Geoid>("egm96-5", "", true, true);

  GeographicLib::Geocentric earth(GeographicLib::Constants::WGS84_a(),
                                  GeographicLib::Constants::WGS84_f());

  double origin_lat = 39.993117, origin_lon = -0.068812, origin_alt = 0;
  // double origin_alt = GeographicLib::Geoid::GEOIDTOELLIPSOID *
  // (*egm96_5)(origin_lat, origin_alt);

  Eigen::Vector3d map_origin; //!< geodetic origin [lla]
  map_origin = {origin_lat, origin_lon, origin_alt};
  /**
   * @brief Conversion of the origin from geodetic coordinates (LLA)
   * to ECEF (Earth-Centered, Earth-Fixed)
   */

  GeographicLib::LocalCartesian localNED(map_origin.x(), map_origin.y(),
                                         map_origin.z(), earth);

  control->SetHomeUpdatedCb([&](const mavlink_home_position_t &msg) {

    map_origin = {msg.latitude / 1e7, msg.longitude / 1e7, msg.altitude / 1e3};

    double lat_error = map_origin.x() - origin_lat;
    double lon_error = map_origin.y() - origin_lon;
    double alt_error = map_origin.z() - origin_alt;
    double error = 1;

    home_set = lat_error < error && lon_error < error && alt_error < error;
  });

  control->SetLocalPositionNEDCb([&](const mavlink_local_position_ned_t &msg) {
    //    Log->Info("LOCAL_POSITION_NED:"
    //              "\ttime_boot_ms: {} ; "
    //              "x: {} ; "
    //              "y: {} ; "
    //              "z: {} ; "
    //              "vx: {} ; "
    //              "vy: {} ; "
    //              "vz: {} ; ",
    //              msg.time_boot_ms, msg.x, msg.y, msg.z, msg.vx, msg.vy,
    //              msg.vz);

    //    currentHROVMessage_mutex.lock();
    //    currentHROVMessageV2->SetZ(msg.z * 100);
    //    currentHROVMessage_updated = true;
    //    currentHROVMessage_mutex.unlock();
    //    currentHROVMessage_cond.notify_one();

    currentHROVMessage_mutex.lock();
    currentHROVMessageV2->SetX(msg.x * 100);
    currentHROVMessageV2->SetY(msg.y * 100);
    currentHROVMessageV2->SetZ(msg.z * 100);
    currentHROVMessage_updated = true;
    currentHROVMessage_mutex.unlock();
    currentHROVMessage_cond.notify_one();

    ned_mutex.lock();

    ned_x = msg.x;
    ned_y = msg.y;
    ned_z = msg.z;

    ned_cond.notify_all();
    ned_mutex.unlock();
  });

  control->SetGlobalPositionInt([&](const mavlink_global_position_int_t &msg) {
    gps_mutex.lock();
    latitude = msg.lat;
    longitude = msg.lon;
    gps_mutex.unlock();

    //    double x,y,z;
    //    localNED.Forward(msg.lat / 1e7, msg.lon / 1e7, (msg.alt /
    //    (double)1e3),
    //                     x, y, z);
    //    ned_mutex.lock();
    //    ned_x = y;
    //    ned_y = x;
    //    ned_z = -z;
    //    ned_mutex.unlock();

    //    currentHROVMessage_mutex.lock();
    //    currentHROVMessageV2->SetX(ned_x * 100);
    //    currentHROVMessageV2->SetY(ned_y * 100);
    //    currentHROVMessageV2->SetZ(ned_z * 100);
    //    currentHROVMessage_updated = true;
    //    currentHROVMessage_mutex.unlock();
    //    currentHROVMessage_cond.notify_one();

    //    Log->Info("NED:"
    //              "\ttime_boot_ms: {}\n"
    //              "\tned_x: {}\n"
    //              "\tned_y: {}\n"
    //              "\tted_z: {}\n",
    //              msg.time_boot_ms, ned_x, ned_y, ned_z);

  });

  std::thread setHomeWorker([&]() {
    while (true) {
      control->SetHome(origin_lat, origin_lon, origin_alt);
      std::this_thread::sleep_for(chrono::seconds(10));
    }
  });

  std::thread uwsimPublisher([&]() {
    bool world_ned_set = false;
    tf::StampedTransform world_ned_tf, ned_rov_tf;
    tf::TransformListener worldNEDListener;
    while (1) {
      if (world_ned_set) {
        std::this_thread::sleep_for(chrono::milliseconds(20));
        std::unique_lock<std::mutex> lock(ned_mutex);
        ned_cond.wait(lock);
        ned_rov_tf.setOrigin(tf::Vector3(ned_x, ned_y, ned_z));
        ned_rov_tf.setRotation(tf::createQuaternionFromYaw(g_yaw));

        tf::Transform wMv = world_ned_tf * ned_rov_tf;
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
        pose_pub.publish(msg);
      } else {
        try {
          worldNEDListener.waitForTransform("world", "local_origin_ned",
                                            ros::Time(0), ros::Duration(1));
          worldNEDListener.lookupTransform("world", "local_origin_ned",
                                           ros::Time(0), world_ned_tf);
          world_ned_set = true;
        } catch (tf::TransformException &e) {
          ROS_ERROR("Not able to lookup transform: %s", e.what());
          world_ned_set = false;
        };
      }
    }
  });
  uwsimPublisher.detach();

  std::thread GPSInput([&]() {
    while (1) {
      tf::StampedTransform ned_origin_rov_tf;
      mavlink_gps_input_t gpsinput;
      try {
        auto now = ros::Time::now();
        listener.waitForTransform("local_origin_ned", "rov", now,
                                  ros::Duration(1.0));
        listener.lookupTransform("local_origin_ned", "rov", ros::Time(0),
                                 ned_origin_rov_tf);

      } catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
        Log->Warn("GPS lost. Sending last filtered position...");
        ros::Duration(1.0).sleep();
        gps_mutex.lock();
        gpsinput.lat = latitude;  // [degrees * 1e7]
        gpsinput.lon = longitude; // [degrees * 1e7]
        gps_mutex.unlock();
      }

      tf::Vector3 position;
      position = ned_origin_rov_tf.getOrigin();
      tf::Quaternion rot = ned_origin_rov_tf.getRotation();
      double heading = tf::getYaw(rot);

      heading = heading * (180. / M_PI);
      if (heading < 0)
        heading = heading + 360;

      //      currentHROVMessage_mutex.lock();
      //      currentHROVMessageV2->SetX(position.x() * 100);
      //      currentHROVMessageV2->SetY(position.y() * 100);
      //      currentHROVMessageV2->SetHeading(heading);
      //      currentHROVMessage_updated = true;
      //      currentHROVMessage_mutex.unlock();
      //      currentHROVMessage_cond.notify_one();

      Eigen::Vector3d geodetic;
      Eigen::Vector3d current_ned(position.x(), position.y(), position.z());

      try {
        localNED.Reverse(current_ned.x(), current_ned.y(), current_ned.z(),
                         geodetic.x(), geodetic.y(), geodetic.z());
      } catch (const std::exception &e) {
        ROS_INFO_STREAM("FGPS: Caught exception: " << e.what() << std::endl);
        continue;
      }

      gpsinput.lat = geodetic.x() * 1e7; // [degrees * 1e7]
      gpsinput.lon = geodetic.y() * 1e7; // [degrees * 1e7]

      //      gpsinput.lat = 0; // geodetic.x() * 1e7; // [degrees * 1e7]
      //      gpsinput.lon = 0; // geodetic.y() * 1e7; // [degrees * 1e7]

      // BY GPS_INPUT
      //      uint32_t IGNORE_VELOCITIES_AND_ACCURACY =
      //          (GPS_INPUT_IGNORE_FLAG_VEL_HORIZ |
      //           GPS_INPUT_IGNORE_FLAG_VEL_VERT |
      //           GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY |
      //           GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY |
      //           GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY);
      uint32_t IGNORE_VELOCITIES_AND_ACCURACY =
          (GPS_INPUT_IGNORE_FLAG_ALT | GPS_INPUT_IGNORE_FLAG_VEL_HORIZ |
           GPS_INPUT_IGNORE_FLAG_VEL_VERT |
           GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY |
           GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY |
           GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY);

      gpsinput.gps_id = 0;
      gpsinput.fix_type = 3;
      gpsinput.hdop = 1;
      gpsinput.vdop = 1;
      gpsinput.satellites_visible = 10;
      gpsinput.ignore_flags = IGNORE_VELOCITIES_AND_ACCURACY;
      // control->SendGPSInput(gpsinput);

      //Log->Info("Lat: {} ; Lon: {}", gpsinput.lat, gpsinput.lon);
      std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }
  });

  tf::TransformBroadcaster ekfBroadcaster;
  tf::StampedTransform ned_origin_ekfrov_tf;

  control->Start();
  try {
    ros::Rate rate(30); // 30 hz

    while (ros::ok()) {
      attitude_mutex.lock();
      tfScalar last_roll = g_roll, last_pitch = g_pitch, last_yaw = g_yaw;
      attitude_mutex.unlock();

      ned_mutex.lock();
      double last_x = ned_x, last_y = ned_y, last_z = ned_z;
      ned_mutex.unlock();

      tf::Quaternion attitude =
          tf::createQuaternionFromRPY(0, 0, last_yaw).normalize();

      double heading = g_yaw * (180. / M_PI);
      if (heading < 0)
        heading = heading + 360;

      ned_origin_ekfrov_tf.setOrigin(tf::Vector3(last_x, last_y, last_z));
      ned_origin_ekfrov_tf.setRotation(attitude);

      //      ekfBroadcaster.sendTransform(
      //          tf::StampedTransform(ned_origin_ekfrov_tf, ros::Time::now(),
      //                               "local_origin_ned", "ekfrov"));

      ros::spinOnce();
      rate.sleep();
    }
  } catch (std::exception &e) {
    Log->Error("Exception: {}", e.what());
    exit(1);
  }
  return 0;
}
