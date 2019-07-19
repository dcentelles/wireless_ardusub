#include <chrono>
#include <cpplogging/cpplogging.h>
#include <dccomms/Utils.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/publisher.h>
#include <ros/ros.h>
#include <ros/subscriber.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <thread>
#include <vector>

using namespace cpplogging;
using namespace std::chrono_literals;

static ros::Publisher bluerov2Pub;

void ekfCb(
    const boost::shared_ptr<const geometry_msgs::PoseWithCovarianceStamped>
        msg) {
  //    geometry_msgs::Pose nmsg;
  //    nmsg.position = msg->pose.pose.position;
  //    nmsg.orientation = msg->pose.pose.orientation;

  //    bluerov2Pub.publish(nmsg);
}

int main(int argc, char **argv) {
  auto log = CreateLogger("GCS");
  log->SetLogLevel(debug);
  log->FlushLogOn(debug);
  log->LogToFile("/home/irs/naiffilter.log");

  ros::init(argc, argv, "tfToEkf");
  ros::NodeHandle nh;

  std::mutex rotlock;
  tf::TransformListener listener;
  tf::TransformBroadcaster tfBroadcaster;
  tf2_ros::StaticTransformBroadcaster static_broadcaster;
  geometry_msgs::TransformStamped static_transformStamped;
  std::vector<geometry_msgs::TransformStamped> static_transforms;

  tf::StampedTransform cameraMrov, wMrov;
  ros::Rate rate(30);
  while (ros::ok()) {
    try {
      listener.lookupTransform("bluerov2_camera", "hil", ros::Time(0),
                               cameraMrov);
      static_transformStamped.header.stamp = ros::Time::now();
      static_transformStamped.header.frame_id = "camera";
      static_transformStamped.child_frame_id = "aruco_rov";
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
  ros::Publisher ekfPub, naifFilterPub;
  ros::Subscriber ekfSub;
  bluerov2Pub = nh.advertise<geometry_msgs::Pose>("/bluerov2/pose", 1);
  ekfPub = nh.advertise<nav_msgs::Odometry>("/vo", 1);
  naifFilterPub = nh.advertise<nav_msgs::Odometry>("/naif_filter", 1);
  nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>(
      "/robot_pose_ekf/odom_combined", 1, ekfCb);

  geometry_msgs::Pose ekfInputPose, naifInputPose;
  nav_msgs::Odometry ekfInputMsg, naifInputMsg;

  double cov[36] = {
      0.1, 0, 0, 0,    0, 0, 0, 0.1, 0, 0, 0,    0, 0, 0, 0.1, 0, 0, 0,
      0,   0, 0, 0.05, 0, 0, 0, 0,   0, 0, 0.05, 0, 0, 0, 0,   0, 0, 0.05,
  };

  boost::array<double, 36> covmat;
  double *imax = cov + 36;
  int pos = 0;
  for (double *i = cov; i < imax; i++) {
    covmat[pos] = *i;
    pos++;
  }

  std::list<tf::StampedTransform> fwindow;

  tf::Vector3 wTrov, auxT, lastNaifT;
  tf::Quaternion wRrov;
  double lastNaifRoll, lastNaifPitch, lastNaifYaw, naifRoll, naifPitch, naifYaw,
      roll, pitch, yaw;

  dccomms::Timer pubTimer, logTimer;

  geometry_msgs::Pose nmsg;
  pubTimer.Reset();
  double pubRate = 3;
  double pubPeriod = 1 / pubRate * 1000; // ms
  logTimer.Reset();
  bool first = true;
  while (ros::ok()) {
    try {
      listener.lookupTransform("world", "aruco_rov", ros::Time(0), wMrov);
    } catch (tf::TransformException &ex) {
      log->Warn("TF: {}", ex.what());
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      continue;
    }
    if (pubTimer.Elapsed() < pubPeriod)
      continue;

    auxT = wMrov.getOrigin();
    wMrov.getBasis().getRPY(roll, pitch, yaw, 2);

    pubTimer.Reset();

    // Pub to ekf
    tf::poseTFToMsg(wMrov, ekfInputPose);
    ekfInputMsg.pose.pose = ekfInputPose;
    ekfInputMsg.pose.covariance = covmat;
    ekfInputMsg.header.stamp = ros::Time::now();
    ekfInputMsg.header.frame_id = "aruco_rov_ekf";
    ekfPub.publish(ekfInputMsg);

    if (first) {
      lastNaifT = auxT;
      lastNaifRoll = roll;
      lastNaifPitch = pitch;
      lastNaifYaw = yaw;
      first = false;
    }

    // Low pass filter
    double delta = 0.06449;
    double deltaB = 0.9355;
    wTrov[0] = deltaB * lastNaifT[0] + delta * auxT[0];
    wTrov[1] = deltaB * lastNaifT[1] + delta * auxT[1];
    wTrov[2] = deltaB * lastNaifT[2] + delta * auxT[2];

    naifRoll = deltaB * lastNaifRoll + delta * roll;
    naifPitch = deltaB * lastNaifPitch + delta * pitch;
    naifYaw = deltaB * lastNaifYaw + delta * yaw;

    log->Info("{} {} {} -- {} {} {} ", auxT[0], auxT[1], auxT[2], wTrov[0],
              wTrov[1], wTrov[2]);

    lastNaifT[0] = wTrov[0];
    lastNaifT[1] = wTrov[1];
    lastNaifT[2] = wTrov[2];
    lastNaifRoll = naifRoll;
    lastNaifPitch = naifPitch;
    lastNaifYaw = naifYaw;

    wMrov.setOrigin(wTrov);
    wRrov.setRPY(naifRoll, naifPitch, naifYaw);
    wMrov.setRotation(wRrov.normalize());

    tfBroadcaster.sendTransform(
        tf::StampedTransform(wMrov, ros::Time::now(), "world", "erov"));

    // Pub to naif
    tf::poseTFToMsg(wMrov, naifInputPose);
    naifInputMsg.pose.pose = naifInputPose;
    naifInputMsg.pose.covariance = covmat;
    naifInputMsg.header.stamp = ros::Time::now();
    naifInputMsg.header.frame_id = "aruco_rov_naif";
    naifFilterPub.publish(naifInputMsg);

    nmsg.position = naifInputPose.position;
    nmsg.orientation = naifInputPose.orientation;

    bluerov2Pub.publish(nmsg);

    ros::spinOnce();
  }
  return 0;
}

//    lastNaifR.setX(wRrov.getX());
//    lastNaifR.setY(wRrov.getY());
//    lastNaifR.setZ(wRrov.getZ());
//    lastNaifR.setW(wRrov.getW());

// Pub to ekf
//    tf::poseTFToMsg(wMrov, ekfInputPose);
//    ekfInputMsg.pose.pose = ekfInputPose;
//    ekfInputMsg.pose.covariance = covmat;
//    ekfInputMsg.header.stamp = ros::Time::now();
//    ekfInputMsg.header.frame_id = "aruco_rov_ekf";
//    ekfPub.publish(ekfInputMsg);

// Pub to naif
//    tf::poseTFToMsg(wMrov, naifInputPose);
//    naifInputMsg.pose.pose = naifInputPose;
//    naifInputMsg.pose.covariance = covmat;
//    naifInputMsg.header.stamp = ros::Time::now();
//    naifInputMsg.header.frame_id = "aruco_rov_naif";
//    naifFilterPub.publish(naifInputMsg);

//    nmsg.position = naifInputPose.position;
//    nmsg.orientation = naifInputPose.orientation;

//    bluerov2Pub.publish(nmsg);

//   double maxdiff = 0.35;
//   auxT = wMrov.getOrigin();
//   auxR = wMrov.getRotation();
//   double diffx, diffy, diffz;

//   diffx = std::abs(lastT[0] - auxT[0]);
//   diffy = std::abs(lastT[1] - auxT[1]);
//   diffz = std::abs(lastT[2] - auxT[2]);

//   lastT = auxT;
//   lastR = auxR;

//   double dist = std::sqrt(diffx * diffx + diffy * diffy + diffz * diffz);
//   if(dist > maxdiff)
//   {
//       continue;
//   }
//   //mean filter
//   fwindow.push_back(wMrov);
//   if(fwindow.size() > winSize)
//   {
//        fwindow.pop_front();
//   }
//   wTrov.setZero();
//   wRrov.setValue(0,0,0,0);

//   for(auto trans : fwindow)
//   {
//       auxT = wMrov.getOrigin();
//       auxR = wMrov.getRotation();

//       wTrov[0] += auxT[0];
//       wTrov[1] += auxT[1];
//       wTrov[2] += auxT[2];

//       wRrov.setX(wRrov.getX() + auxR.getX());
//       wRrov.setY(wRrov.getY() + auxR.getY());
//       wRrov.setZ(wRrov.getZ() + auxR.getZ());
//       wRrov.setW(wRrov.getW() + auxR.getW());
//   }
//   auto winsize = static_cast<double>(fwindow.size());
//   wRrov.setX(auxR.getX() / winsize);
//   wRrov.setY(auxR.getY() / winsize);
//   wRrov.setZ(auxR.getZ() / winsize);
//   wRrov.setW(auxR.getW() / winsize);

//   wTrov[0] = wTrov[0] / winsize;
//   wTrov[1] = wTrov[1] / winsize;
//   wTrov[2] = wTrov[2] / winsize;

//    wRrov.setX(deltaB * lastNaifR.getX() + delta * auxR.getX());
//    wRrov.setY(deltaB * lastNaifR.getY() + delta * auxR.getY());
//    wRrov.setZ(deltaB * lastNaifR.getZ() + delta * auxR.getZ());
//    wRrov.setW(deltaB * lastNaifR.getW() + delta * auxR.getW());
