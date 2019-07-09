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
#include <tf2_ros/static_transform_broadcaster.h>
#include <thread>
#include <wireless_ardusub/pid.h>

using namespace mavlink_cpp;
using namespace cpplogging;
using namespace std::chrono_literals;

std::shared_ptr<GCSv1> control;

wireless_ardusub::PID yawPID =
                          wireless_ardusub::PID(0.1, 3.14, -3.14, 0.5, 1, 0.05),
                      vPID = wireless_ardusub::PID(0.1, 3, -3, 0.5, 1, 0.05);

// from -127,127  to -100,100
double getJoyAxisNormalized(int x) { return 200. / 256 * x; }
// from -100,100 to -1000,1000
double arduSubXYR(double per) { return per * 10; }
double arduSubZ(double per) { return (per + 100) / 0.2; }

double keepHeadingIteration(double diff) {
  double vel = yawPID.calculate(0, -1 * diff);

  return vel;
}

void stopRobot() {
  int x = ceil(arduSubXYR(0));
  int y = ceil(arduSubXYR(0));
  int z = ceil(arduSubZ(0));
  int r = ceil(arduSubXYR(0));
  control->SetManualControl(x, y, z, r);
}

void saturate(const double &max, const double &x, const double &y,
              const double &z, double &vx, double &vy, double &vz) {
  double rx, ry, rz;
  rx = x / max;
  ry = y / max;
  rz = y / max;
  if (rx > 1 || ry > 1 || rz > 1) {
    double alpha = rx;
    if (alpha < ry)
      alpha = ry;
    if (alpha < rz)
      alpha = rz;
    vx = x / alpha;
    vy = y / alpha;
    vz = z / alpha;
  } else {
    vx = x;
    vy = y;
    vz = z;
  }
}

void GetLinearVel(const double &diffx, const double &diffy, const double &diffz,
                  double &vx, double &vy, double &vz) {
  double mod = std::sqrt(diffx * diffx + diffy * diffy + diffz * diffz);
  double vel = vPID.calculate(0, mod);
  saturate(vel, diffx, diffy, diffz, vx, vy, vz);
  double m = 100 / 3.;
  vx = m * vx;
  vy = m * vy;
  vz = m * vz;
}

int main(int argc, char **argv) {
  auto log = CreateLogger("Main");
  log->SetLogLevel(info);
  log->FlushLogOn(debug);
  log->SetAsyncMode();
  uint16_t localPort = 14550;
  control = std::shared_ptr<GCSv1>(new GCSv1(localPort));

  ros::init(argc, argv, "custom_setpoint");
  ros::NodeHandle nh;
  ros::Publisher bluerov2Pub;

  control->SetLogName("GCS");
  control->SetLogLevel(info);
  control->SetAsyncMode();
  control->FlushLogOn(debug);

  control->EnableGPSMock(false);
  control->SetManualControl(0, 0, 0, 0);
  control->EnableManualControl(true);
  control->Start();
  control->Arm(true);

  tf::TransformListener listener;
  static tf2_ros::StaticTransformBroadcaster static_broadcaster;
  geometry_msgs::TransformStamped static_transformStamped;
  std::vector<geometry_msgs::TransformStamped> static_transforms;

  ros::Rate rate(10);

  tf::StampedTransform nedMrov, nedMtarget, rovMtarget, cameraMrov;
  tf::Transform rovMned;

  while (1) {
    try {
      listener.lookupTransform("bluerov2_camera", "sitl", ros::Time(0),
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
    }
  }
  static_broadcaster.sendTransform(static_transforms);
  while (ros::ok()) {
    try {
      listener.lookupTransform("local_origin_ned", "sitl", ros::Time(0),
                               nedMrov);
      listener.lookupTransform("local_origin_ned", "bluerov2_ghost",
                               ros::Time(0), nedMtarget);
      listener.lookupTransform("erov", "bluerov2_ghost", ros::Time(0),
                               rovMtarget);
    } catch (tf::TransformException &ex) {
      log->Warn("TF: {}", ex.what());
      continue;
    }
    control->Arm(true);
    //control->SetDepthHoldMode();

    rovMned = nedMrov.inverse();
    tf::Vector3 rovTned = rovMned.getOrigin();
    tf::Quaternion rovRned = rovMned.getRotation();

    tf::Vector3 rovTtarget = rovMtarget.getOrigin();
    tf::Quaternion rovRtarget = rovMtarget.getRotation();

    tf::Vector3 nedTrov = nedMrov.getOrigin();
    tf::Quaternion nedRrov = nedMrov.getRotation();
    tf::Vector3 nedTtarget = nedMtarget.getOrigin();
    tf::Quaternion nedRtarget = nedMtarget.getRotation();

    double vx, vy, vz;
    double vTlpX = rovTtarget.getX(), vTlpY = rovTtarget.getY(),
           vTlpZ = rovTtarget.getZ();
    GetLinearVel(vTlpX, vTlpY, -vTlpZ, vx, vy, vz);

    double rdiff = tf::getYaw(rovRtarget);
    double rv = keepHeadingIteration(rdiff);
    double mr = 100 / 3.14;
    rv = mr * rv;

    double baseZ = -40;
    double newZ = vz + baseZ;
    auto x = ceil(arduSubXYR(vx));
    auto y = ceil(arduSubXYR(vy));
    auto z = ceil(arduSubZ(newZ));
    auto r = ceil(arduSubXYR(rv));

    log->Info("[ {} , {} , {} ] ----> [ {} , {} , {} ] ----> [ {} ({} -- {}) , "
              "{} ({} -- {}) , {} ({} -- {})] ===== [ {} : {} : {} ]",
              nedTrov.getX(), nedTrov.getY(), nedTrov.getZ(), nedTtarget.getX(),
              nedTtarget.getY(), nedTtarget.getZ(), x, vTlpX, vx, y, vTlpY, vy,
              z, vTlpZ, newZ, rdiff, rv, r);

    control->SetManualControl(x, y, z, r);
    ros::spinOnce();
    rate.sleep();
  }
}
