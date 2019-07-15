#include <chrono>
#include <cpplogging/cpplogging.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Pose.h>
#include <ros/publisher.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <thread>

using namespace cpplogging;
using namespace std::chrono_literals;

int main(int argc, char **argv) {
  auto log = CreateLogger("GCS");
  log->SetLogLevel(debug);
  log->FlushLogOn(debug);

  ros::init(argc, argv, "TfToPosePublisher");
  ros::NodeHandle nh;

  std::mutex rotlock;
  tf::TransformListener listener;
  tf2_ros::StaticTransformBroadcaster static_broadcaster;
  geometry_msgs::TransformStamped static_transformStamped;
  std::vector<geometry_msgs::TransformStamped> static_transforms;

  tf::StampedTransform cameraMrov, wMrov;
  ros::Rate rate(30);
  while (1) {
    try {
      listener.lookupTransform("bluerov2_camera", "hil", ros::Time(0),
                               cameraMrov);
      static_transformStamped.header.stamp = ros::Time::now();
      static_transformStamped.header.frame_id = "camera";
      static_transformStamped.child_frame_id = "erov";
      static_transformStamped.transform.translation.x =
          cameraMrov.getOrigin().getX();
      static_transformStamped.transform.translation.y =
          cameraMrov.getOrigin().getY();
      static_transformStamped.transform.translation.z =
          cameraMrov.getOrigin().getZ();
      static_transformStamped.transform.rotation.x =
          cameraMrov.getRotation().x();
      static_transformStamped.transform.rotation.y =
          cameraMrov.getRotation().y();
      static_transformStamped.transform.rotation.z =
          cameraMrov.getRotation().z();
      static_transformStamped.transform.rotation.w =
          cameraMrov.getRotation().w();
      static_transforms.push_back(static_transformStamped);
      break;
    } catch (tf::TransformException &ex) {
      log->Warn("TF: {}", ex.what());
      rate.sleep();
    }
  }
  static_broadcaster.sendTransform(static_transforms);
  ros::Publisher bluerov2Pub;
  bluerov2Pub = nh.advertise<geometry_msgs::Pose>("/bluerov2/pose", 1);

  while (ros::ok()) {
    try {
      listener.lookupTransform("world", "erov", ros::Time(0), wMrov);
    } catch (tf::TransformException &ex) {
      log->Warn("TF: {}", ex.what());
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      continue;
    }
    auto position = wMrov.getOrigin();
    auto rotation = wMrov.getRotation();

    geometry_msgs::Pose bluerov2msg;
    bluerov2msg.position.x = position.x();
    bluerov2msg.position.y = position.y();
    bluerov2msg.position.z = position.z();
    bluerov2msg.orientation.x = rotation.x();
    bluerov2msg.orientation.y = rotation.y();
    bluerov2msg.orientation.z = rotation.z();
    bluerov2msg.orientation.w = rotation.w();
    rotlock.unlock();
    bluerov2Pub.publish(bluerov2msg);

    ros::spinOnce();
    // rate.sleep();
  }
}
