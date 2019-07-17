#include <chrono>
#include <cpplogging/cpplogging.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Pose.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <thread>
#include <nav_msgs/Odometry.h>
#include <vector>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <dccomms/Utils.h>

using namespace cpplogging;
using namespace std::chrono_literals;

static ros::Publisher bluerov2Pub;

void ekfCb(const boost::shared_ptr<const geometry_msgs::PoseWithCovarianceStamped> msg)
{
    geometry_msgs::Pose nmsg;
    nmsg.position = msg->pose.pose.position;
    nmsg.orientation = msg->pose.pose.orientation;

    bluerov2Pub.publish(nmsg);
}

int main(int argc, char **argv) {
  auto log = CreateLogger("GCS");
  log->SetLogLevel(debug);
  log->FlushLogOn(debug);

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
  while (1) {
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
  ekfPub = nh.advertise<nav_msgs::Odometry>("/vo",1);
  naifFilterPub = nh.advertise<nav_msgs::Odometry>("/naif_filter",1);
  nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/robot_pose_ekf/odom_combined", 1, ekfCb);

  geometry_msgs::Pose ekfInputPose, naifInputPose;
  nav_msgs::Odometry ekfInputMsg, naifInputMsg;
  double cov[36] = {
      0.1, 0  , 0  , 0   , 0   , 0,
      0  , 0.1, 0  , 0   , 0   , 0,
      0  , 0  , 0.1, 0   , 0   , 0,
      0  , 0  , 0  , 0.05, 0   , 0,
      0  , 0  , 0  , 0   , 0.05, 0,
      0  , 0  , 0  , 0   , 0   , 0.05,
  };

  double covtwist[36] = {
      1, 0  , 0  , 0   , 0   , 0,
      0  , 1, 0  , 0   , 0   , 0,
      0  , 0  , 1, 0   , 0   , 0,
      0  , 0  , 0  , 1, 0   , 0,
      0  , 0  , 0  , 0   , 1, 0,
      0  , 0  , 0  , 0   , 0   , 1,
  };

//  double cov[36] = {
//      0.5, 0  , 0  , 0   , 0   , 0,
//      0  , 0.5, 0  , 0   , 0   , 0,
//      0  , 0  , 0.5, 0   , 0   , 0,
//      0  , 0  , 0  , 0.5, 0   , 0,
//      0  , 0  , 0  , 0   , 0.5, 0,
//      0  , 0  , 0  , 0   , 0   , 0.5,
//  };

  boost::array<double, 36> covmat;
  double * imax = cov + 36;
  int pos=0;
  for(double * i = cov; i < imax; i++)
  {
    covmat[pos] = *i;
    pos++;
  }

  std::list<tf::StampedTransform> fwindow;
  int winSize = 20;

  tf::Vector3 wTrov, auxT, lastT, lastMeanT;
  tf::Quaternion wRrov, auxR, lastR;

  dccomms::Timer pubTimer;

  pubTimer.Reset();
  double pubRate = 5;
  double pubPeriod = 1 / pubRate * 1000; //ms
  while (ros::ok()) {
    try {
      listener.lookupTransform("world", "aruco_rov", ros::Time(0), wMrov);
    } catch (tf::TransformException &ex) {
      log->Warn("TF: {}", ex.what());
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      continue;
    }
   if(pubTimer.Elapsed() < pubPeriod)
       continue;
   pubTimer.Reset();
   //Pub to ekf
    tf::poseTFToMsg(wMrov, ekfInputPose);
    ekfInputMsg.pose.pose = ekfInputPose;
    ekfInputMsg.pose.covariance = covmat;
    ekfInputMsg.header.stamp = ros::Time::now();
    ekfInputMsg.header.frame_id = "aruco_rov_ekf";
    ekfPub.publish(ekfInputMsg);

   double maxdiff = 0.35;

   if(fwindow.size() == 0)
   {
       lastR = wMrov.getRotation();
       lastT = wMrov.getOrigin();
   }

   auxT = wMrov.getOrigin();
   auxR = wMrov.getRotation();
   double diffx, diffy, diffz;

   diffx = std::abs(lastT[0] - auxT[0]);
   diffy = std::abs(lastT[1] - auxT[1]);
   diffz = std::abs(lastT[2] - auxT[2]);

   lastT = auxT;
   lastR = auxR;

   double dist = std::sqrt(diffx * diffx + diffy * diffy + diffz * diffz);
   if(dist > maxdiff)
   {
       continue;
   }
   //mean filter
   fwindow.push_back(wMrov);
   if(fwindow.size() > winSize)
   {
        fwindow.pop_front();
   }
   wTrov.setZero();
   wRrov.setValue(0,0,0,0);


   for(auto trans : fwindow)
   {
       auxT = wMrov.getOrigin();
       auxR = wMrov.getRotation();

       wTrov[0] += auxT[0];
       wTrov[1] += auxT[1];
       wTrov[2] += auxT[2];

       wRrov.setX(wRrov.getX() + auxR.getX());
       wRrov.setY(wRrov.getY() + auxR.getY());
       wRrov.setZ(wRrov.getZ() + auxR.getZ());
       wRrov.setW(wRrov.getW() + auxR.getW());
   }
   auto winsize = static_cast<double>(fwindow.size());
   wRrov.setX(auxR.getX() / winsize);
   wRrov.setY(auxR.getY() / winsize);
   wRrov.setZ(auxR.getZ() / winsize);
   wRrov.setW(auxR.getW() / winsize);

   wRrov.normalize();

   wTrov[0] = wTrov[0] / winsize;
   wTrov[1] = wTrov[1] / winsize;
   wTrov[2] = wTrov[2] / winsize;

   wMrov.setOrigin(wTrov);
   wMrov.setRotation(wRrov);

   tfBroadcaster.sendTransform(tf::StampedTransform(
                                wMrov,
                      ros::Time::now(),
                      "world", "mean"));

   log->Warn("Win. Size: {}", winsize);

   //Pub to naif
    tf::poseTFToMsg(wMrov, naifInputPose);
    naifInputMsg.pose.pose = naifInputPose;
    naifInputMsg.pose.covariance = covmat;
    naifInputMsg.header.stamp = ros::Time::now();
    naifInputMsg.header.frame_id = "aruco_rov_naif";
    naifFilterPub.publish(ekfInputMsg);





    ros::spinOnce();
  }
  return 0;
}
