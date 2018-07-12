#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/Geoid.hpp>
#include <chrono>
#include <cpplogging/cpplogging.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Pose.h>
#include <mavlink_cpp/GCSv1.h>
#include <ros/publisher.h>
#include <ros/publisher.h>
#include <ros/ros.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_listener.h>
#include <thread>

using namespace mavlink_cpp;
using namespace cpplogging;
using namespace std::chrono_literals;

int main(int argc, char **argv) {
  auto log = CreateLogger("GCS");
  log->SetLogLevel(debug);
  log->FlushLogOn(debug);
  uint16_t localPort = 14551;
  std::shared_ptr<GCSv1> control(new GCSv1(localPort));

  ros::init(argc, argv, "NEDPublisher");
  ros::NodeHandle nh;

  control->SetLogName("GCS");
  control->SetLogLevel(debug);
  control->FlushLogOn(debug);

  tf::Matrix3x3 rotMat;
  tfScalar yaw;
  tfScalar pitch;
  tfScalar roll;

  std::mutex rotlock;
  control->SetAttitudeCb([&](const mavlink_attitude_t &attitude) {
    yaw = attitude.yaw;
    pitch = attitude.pitch;
    roll = attitude.roll;
    rotlock.lock();
    rotMat.setRPY(roll, pitch, yaw);
    rotMat.inverse().getRPY(roll, pitch, yaw);
    rotlock.unlock();
    // log->Info("(inverse) R: {:9f} ; P: {:9f} ; Y: {:9f}", roll, pitch, yaw);
  });

  ros::Publisher bluerov2Pub;
  bluerov2Pub = nh.advertise<geometry_msgs::Pose>("/bluerov2/pose", 1);

  control->SetLocalPositionNEDCb([&](const mavlink_local_position_ned_t &msg) {
    log->Debug("LOCAL_POSITION_NED:"
               "\ttime_boot_ms: {}\n"
               "\tx: {}\n"
               "\ty: {}\n"
               "\tz: {}\n"
               "\tvx: {}\n"
               "\tvy: {}\n"
               "\tvz: {}\n",
               msg.time_boot_ms, msg.x, msg.y, msg.z, msg.vx, msg.vy, msg.vz);

    geometry_msgs::Pose bluerov2msg;
    bluerov2msg.position.x = msg.x;
    bluerov2msg.position.y = msg.x;
    bluerov2msg.position.z = msg.x;
    tf::Quaternion q;
    rotlock.lock();
    rotMat.getRotation(q);
    bluerov2msg.orientation.x = q.x();
    bluerov2msg.orientation.y = q.y();
    bluerov2msg.orientation.z = q.z();
    bluerov2msg.orientation.w = q.w();
    rotlock.unlock();
    bluerov2Pub.publish(bluerov2msg);

  });

  control->EnableGPSMock(true);
  control->EnableManualControl(false);
  control->Arm(false);

  control->Start();
  ros::Rate rate(30);

  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }
}
