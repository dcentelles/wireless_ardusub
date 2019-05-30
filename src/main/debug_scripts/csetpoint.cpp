#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/Geoid.hpp>
#include <GeographicLib/LocalCartesian.hpp>
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

std::shared_ptr<GCSv1> control;
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

int main(int argc, char **argv) {
  auto log = CreateLogger("GCS");
  log->SetLogLevel(info);
  log->FlushLogOn(debug);
  uint16_t localPort = 14550;
  control = std::shared_ptr<GCSv1>(new GCSv1(localPort));

  ros::init(argc, argv, "custom_setpoint");
  ros::NodeHandle nh;
  ros::Publisher bluerov2Pub;

  control->SetLogName("GCS");
  control->SetLogLevel(info);
  control->FlushLogOn(debug);

  control->EnableGPSMock(false);
  control->SetManualControl(0, 0, 0, 0);
  control->EnableManualControl(true);
  control->SetDepthHoldMode();
  control->Arm(true);
  control->Start();

  tf::TransformListener listener;


  ros::Rate rate(10);

  tf::StampedTransform nedMrov;
  tf::Transform rovMned;
  while (ros::ok()) {
    try {
      listener.lookupTransform("local_origin_ned", "sitl", ros::Time(0),
                               nedMrov);
    } catch (tf::TransformException &ex) {
      log->Warn("TF: {}", ex.what());
      continue;
    }
    rovMned = nedMrov.inverse();
    tf::Vector3 rovTned = rovMned.getOrigin();
    tf::Quaternion rovRned = rovMned.getRotation();
    log->Info("[ {} , {} , {} ]", rovTned.getX(), rovTned.getY(), rovTned.getZ());
    auto x = ceil(arduSubXYR(20));
    auto y = ceil(arduSubXYR(0));
    auto z = ceil(arduSubZ(0));
    auto r = ceil(arduSubXYR(0));
    control->SetManualControl(x, y, z, r);
    ros::spinOnce();
    rate.sleep();
  }
}
