#include <chrono>
#include <cpplogging/cpplogging.h>
#include <geometry_msgs/Pose.h>
#include <mavlink_cpp/GCSv1.h>
#include <ros/publisher.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <thread>

using namespace mavlink_cpp;
using namespace cpplogging;
using namespace std::chrono_literals;

int main(int argc, char **argv) {
  auto log = CreateLogger("GCS");
  log->SetLogLevel(off);
  log->FlushLogOn(debug);
  uint16_t localPort = 14550;
  std::shared_ptr<GCSv1> control(new GCSv1(localPort));

  ros::init(argc, argv, "simple_bluerov2_publisher");
  ros::NodeHandle nh;
  ros::Publisher bluerov2Pub;
  bluerov2Pub = nh.advertise<geometry_msgs::Pose>("/bluerov2/pose", 1);

  control->SetLogName("GCS");
  control->SetLogLevel(debug);
  control->FlushLogOn(debug);

  control->EnableGPSMock(true);
  control->Arm(true);
  control->Start();
  ros::Rate rate(30);
  while (ros::ok()) {
    control->WaitForNEDUpdate();
    auto pos = control->GetNED();
    auto gps = control->GetGPS();
    auto imu = control->GetScaledIMU2();
    auto attitude = control->GetAttitude();

    tf::Matrix3x3 rotMat;
    tfScalar yaw = attitude.yaw;
    tfScalar pitch = attitude.pitch;
    tfScalar roll = attitude.roll;
    rotMat.setRPY(roll, pitch, yaw);
    tf::Quaternion quaternion;
    rotMat.getRotation(quaternion);

    geometry_msgs::Pose bluerov2msg;
    bluerov2msg.position.x = pos.x;
    bluerov2msg.position.y = pos.y;
    bluerov2msg.position.z = 4;
    bluerov2msg.orientation.x = quaternion.x();
    bluerov2msg.orientation.y = quaternion.y();
    bluerov2msg.orientation.z = quaternion.z();
    bluerov2msg.orientation.w = quaternion.w();
    bluerov2Pub.publish(bluerov2msg);
    ros::spinOnce();
    rate.sleep();
  }
}
