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

int main(int argc, char **argv) {
  auto log = CreateLogger("GCS");
  log->SetLogLevel(info);
  log->FlushLogOn(debug);
  // ros::Publisher fake_gps_pub;
  uint16_t localPort = 14550;
  std::shared_ptr<GCSv1> control(new GCSv1(localPort));

  ros::init(argc, argv, "simple_bluerov2_publisher");
  ros::NodeHandle nh;
  ros::Publisher bluerov2Pub;
  bluerov2Pub = nh.advertise<geometry_msgs::Pose>("/bluerov2/pose", 1);

  control->SetLogName("GCS");
  control->SetLogLevel(off);
  control->FlushLogOn(debug);

  control->EnableGPSMock(false);
  // control->SetManualControl(0, 0, 0, 0);
  control->Arm(false);
  control->Start();

  tf::TransformListener listener;

  ///// GPS VISION
  double origin_lat = 0, origin_lon = 0, origin_alt = 0;
  Eigen::Vector3d map_origin;  //!< geodetic origin [lla]
  Eigen::Vector3d ecef_origin; //!< geocentric origin [m]
  map_origin = {origin_lat, origin_lon, origin_alt};
  /**
   * @brief Conversion of the origin from geodetic coordinates (LLA)
   * to ECEF (Earth-Centered, Earth-Fixed)
   */
  // Constructor for a ellipsoid
  GeographicLib::Geocentric earth(GeographicLib::Constants::WGS84_a(),
                                  GeographicLib::Constants::WGS84_f());
  earth.Forward(map_origin.x(), map_origin.y(), map_origin.z(), ecef_origin.x(),
                ecef_origin.y(), ecef_origin.z());

  GeographicLib::LocalCartesian localNED(map_origin.x(), map_origin.y(),
                                         map_origin.z(), earth);

  std::shared_ptr<GeographicLib::Geoid> egm96_5;
  egm96_5 = std::make_shared<GeographicLib::Geoid>("egm96-5", "", true, true);

  std::mutex ned_mutex;
  double ecef_x, ecef_y, ecef_z, ned_x, ned_y, ned_z;

  std::mutex attitude_mutex;
  tfScalar yaw;
  tfScalar pitch;
  tfScalar roll;

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

  });

  control->SetGlobalPositionInt([&](const mavlink_global_position_int_t &msg) {

    auto alt =
        (msg.alt / (double)1e3) -
        GeographicLib::Geoid::ELLIPSOIDTOGEOID * (*egm96_5)(msg.lat, msg.lon);

    earth.Forward(msg.lat / 1e7, msg.lon / 1e7, alt, ecef_x, ecef_y, ecef_z);

    ned_mutex.lock();
    localNED.Forward(msg.lat / 1e7, msg.lon / 1e7, (msg.alt / (double)1e3),
                     ned_x, ned_y, ned_z);

    log->Info("lat,lon,alt: {} : {} : {} ---- NED x,y,z {} : {} : {} ---- ECEF "
              "x,y,z {} : {} : {}",
              msg.lat, msg.lon, msg.alt, ned_x, ned_y, ned_z, ecef_x, ecef_y,
              ecef_z);
    ned_mutex.unlock();

  });

  control->SetAttitudeCb([&](const mavlink_attitude_t &attitude) {
    attitude_mutex.lock();
    yaw = attitude.yaw;
    pitch = attitude.pitch;
    roll = attitude.roll;
    attitude_mutex.unlock();

    log->Debug("R: {:9f} ; P: {:9f} ; Y: {:9f}", roll, pitch, yaw);
  });

  std::thread GPSInput([&]() {
    while (1) {
      tf::StampedTransform earth_rov_tf;
      try {
        auto now = ros::Time::now();
        listener.waitForTransform("earth", "rov", now, ros::Duration(3.0));
        listener.lookupTransform("earth", "rov", ros::Time(0), earth_rov_tf);

      } catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
        continue;
      }

      tf::Vector3 position;
      position = earth_rov_tf.getOrigin();

      //      geometry_msgs::Pose bluerov2msg;
      //      bluerov2msg.position.x = position.x();
      //      bluerov2msg.position.y = position.y();
      //      bluerov2msg.position.z = position.z();
      //      tf::quaternionTFToMsg(earth_rov_tf.getRotation(),
      //                            bluerov2msg.orientation);
      //      bluerov2Pub.publish(bluerov2msg);

      Eigen::Vector3d geodetic;
      Eigen::Vector3d current_ecef(ecef_origin.x() + position.x(),
                                   ecef_origin.y() + position.y(),
                                   ecef_origin.z() + position.z());

      try {
        earth.Reverse(current_ecef.x(), current_ecef.y(), current_ecef.z(),
                      geodetic.x(), geodetic.y(), geodetic.z());
      } catch (const std::exception &e) {
        ROS_INFO_STREAM("FGPS: Caught exception: " << e.what() << std::endl);
      }

      // BY GPS_INPUT
      uint32_t IGNORE_VELOCITIES_AND_ACCURACY =
          (GPS_INPUT_IGNORE_FLAG_ALT | GPS_INPUT_IGNORE_FLAG_VEL_HORIZ |
           GPS_INPUT_IGNORE_FLAG_VEL_VERT |
           GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY |
           GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY |
           GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY);

      mavlink_gps_input_t gpsinput;
      gpsinput.gps_id = 0;
      gpsinput.lat = geodetic.x() * 1e7; // [degrees * 1e7]
      gpsinput.lon = geodetic.y() * 1e7; // [degrees * 1e7]
      gpsinput.fix_type = 3;
      gpsinput.hdop = 1;
      gpsinput.vdop = 1;
      gpsinput.satellites_visible = 10;
      gpsinput.ignore_flags = IGNORE_VELOCITIES_AND_ACCURACY;
      control->SendGPSInput(gpsinput);

      // log->Info("Lat: {} ; Lon: {}", gpsinput.lat, gpsinput.lon);
      std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }
  });

  ros::Rate rate(10);
  tf::TransformBroadcaster rovVisionPositionBroadcaster;
  tf::TransformBroadcaster ekfBroadcaster;
  tf::StampedTransform earth_ekfrov_tf;
  tf::StampedTransform camera_rov_tf;

  while (ros::ok()) {
    tf::Quaternion q_rot, q_new;
    double r = -1.57079632648966, p = -1.57079632648966,
           y = 0; // Rotate the previous pose by 180* about X
    q_rot = tf::createQuaternionFromRPY(r, p, y);
    q_new = q_rot;
    camera_rov_tf.setOrigin(tf::Vector3(0, 0, 0));
    camera_rov_tf.setRotation(q_new);
    rovVisionPositionBroadcaster.sendTransform(
        tf::StampedTransform(camera_rov_tf, ros::Time::now(), "camera", "rov"));

    attitude_mutex.lock();
    tfScalar last_roll = roll, last_pitch = pitch, last_yaw = yaw;
    attitude_mutex.unlock();

    ned_mutex.lock();
    double last_x = ned_x, last_y = ned_y, last_z = ned_z;
    ned_mutex.unlock();

    tf::Quaternion attitude =
        tf::createQuaternionFromRPY(last_roll, last_pitch, last_yaw);

    earth_ekfrov_tf.setOrigin(tf::Vector3(last_x, last_y, last_z));
    earth_ekfrov_tf.setRotation(attitude);

    ekfBroadcaster.sendTransform(tf::StampedTransform(
        earth_ekfrov_tf, ros::Time::now(), "earth", "ekfrov"));

    ros::spinOnce();
    rate.sleep();
  }
}

// bool initPositionSet = false;
// tf::Vector3 initPosition;
// while (!initPositionSet) {
//  try {
//    ros::Time now = ros::Time::now();
//    listener.waitForTransform("world", "bluerov2/base_link", now,
//                              ros::Duration(3.0));
//    listener.lookupTransform("world", "bluerov2/base_link", ros::Time(0),
//                             transform);
//    initPosition = transform.getOrigin();
//    initPositionSet = true;
//  } catch (tf::TransformException ex) {
//    ROS_ERROR("%s", ex.what());
//    ros::Duration(1.0).sleep();
//  }
//}

// BY VISION_POSITION_ESTIMATE
//      mavlink_vision_position_estimate_t mavmsg;

//      mavmsg.x = position.x();
//      mavmsg.y = position.y();
//      mavmsg.z = position.z();
//      mavmsg.roll = roll;
//      mavmsg.pitch = pitch;
//      mavmsg.yaw = yaw;

//      control->SendVisionPositionEstimate(mavmsg);

// http://wiki.ros.org/tf2/Tutorials/Quaternions#Relative_rotations

//    tf::Quaternion q_orig, q_rot, q_new;
//    rotMat.getRotation(q_orig);
//    double r = 0, p = 0, y = 0; // Rotate the previous pose by 180* about
//    X
//    q_rot = tf::createQuaternionFromRPY(r, p, y);
//    q_new = q_rot * q_orig; // Calculate the new orientation
//    q_new.normalize();

//    geometry_msgs::Pose bluerov2msg;
//    bluerov2msg.position.x = initPosition.x() + pos.x;
//    bluerov2msg.position.y = initPosition.y() + pos.y;
//    bluerov2msg.position.z = initPosition.z() + pos.z;
//    bluerov2msg.orientation.x = q_new.x();
//    bluerov2msg.orientation.y = q_new.y();
//    bluerov2msg.orientation.z = q_new.z();
//    bluerov2msg.orientation.w = q_new.w();
//    bluerov2Pub.publish(bluerov2msg);
