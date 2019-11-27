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
  auto log = CreateLogger("TfToPosePublisher");
  log->SetLogLevel(debug);
  log->FlushLogOn(debug);

  ros::init(argc, argv, "TfToPosePublisher");
  ros::NodeHandle nh("~");

  std::mutex rotlock;
  tf::TransformListener listener;
  tf2_ros::StaticTransformBroadcaster static_broadcaster;
  geometry_msgs::TransformStamped static_transformStamped;
  std::vector<geometry_msgs::TransformStamped> static_transforms;

  std::string target_tf = "erov", target_topic = "/bluerov2/pose";
  tf::StampedTransform cameraMrov, wMrov;
  ros::Rate rate(30);

  if (!nh.getParam("target_tf", target_tf)) {
      log->Info("target_tf set to default => {}", target_tf);
  } else {
      log->Info("target_tf: {}", target_tf);
  }

  if (!nh.getParam("target_topic", target_topic)) {
      log->Info("target_topic set to default => {}", target_topic);
  } else {
      log->Info("target_topic: {}", target_topic);
  }

  ros::Publisher bluerov2Pub;
  bluerov2Pub = nh.advertise<geometry_msgs::Pose>(target_topic, 1);

  while (ros::ok()) {
    try {
      listener.lookupTransform("world", target_tf, ros::Time(0), wMrov);
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
