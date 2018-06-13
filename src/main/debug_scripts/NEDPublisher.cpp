#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/Geoid.hpp>
#include <chrono>
#include <cpplogging/cpplogging.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Pose.h>
#include <mavlink_cpp/GCSv1.h>
#include <ros/publisher.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <thread>

using namespace mavlink_cpp;
using namespace cpplogging;
using namespace std::chrono_literals;

int main(int argc, char **argv) {
  auto log = CreateLogger("GCS");
  log->SetLogLevel(debug);
  log->FlushLogOn(debug);
  uint16_t localPort = 14550;
  std::shared_ptr<GCSv1> control(new GCSv1(localPort));

  ros::init(argc, argv, "NEDPublisher");
  ros::NodeHandle nh;

  control->SetLogName("GCS");
  control->SetLogLevel(debug);
  control->FlushLogOn(debug);

  control->SetAttitudeCb([&](const mavlink_attitude_t &attitude) {
    tf::Matrix3x3 rotMat;
    tfScalar yaw = attitude.yaw;
    tfScalar pitch = attitude.pitch;
    tfScalar roll = attitude.roll;
    rotMat.setRPY(roll, pitch, yaw);
    rotMat.inverse().getRPY(roll, pitch, yaw);
    log->Info("R: {:9f} ; P: {:9f} ; Y: {:9f}", roll, pitch, yaw);
  });

  control->EnableGPSMock(true);
  control->Arm(false);

  control->Start();
  ros::Rate rate(30);

  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }
}
