#include <arpa/inet.h>
#include <chrono>
#include <cpplogging/cpplogging.h>
#include <dccomms/Utils.h>
#include <fcntl.h>
#include <geometry_msgs/Pose.h>
#include <mavlink_cpp/GCSv1.h>
#include <netinet/in.h>
#include <ros/publisher.h>
#include <ros/ros.h>
#include <sys/types.h>
#include <tf/transform_listener.h>
#include <thread>
#include <thread>
#include <unistd.h>

using namespace mavlink_cpp;
using namespace cpplogging;
using namespace std::chrono_literals;
using namespace dccomms;

struct FDMData {

  uint32_t version;
  uint32_t padding;
  double longitude;
  double latitude;
  double altitude;
  float agl;
  float phi;
  float theta;
  float psi;

  float phidot;		// roll rate (radians/sec)
  float thetadot;		// pitch rate (radians/sec)
  float psidot;		// yaw rate (radians/sec)
  float vcas;		        // calibrated airspeed
  float climb_rate;		// feet per second
  float v_north;              // north velocity in local/body frame, fps
  float v_east;               // east velocity in local/body frame, fps
  float v_down;               // down/vertical velocity in local/body frame, fps
  float v_wind_body_north;    // north velocity in local/body frame
                              // relative to local airmass, fps
  float v_wind_body_east;     // east velocity in local/body frame
                              // relative to local airmass, fps
  float v_wind_body_down;     // down/vertical velocity in local/body
                              // frame relative to local airmass, fps

  // Accelerations
  float A_X_pilot;		// X accel in body frame ft/sec^2
  float A_Y_pilot;		// Y accel in body frame ft/sec^2
  float A_Z_pilot;		// Z accel in body frame ft/sec^2
};

FDMData fdmData;
std::mutex fdmDataMutex;
std::condition_variable fdmDataCond;
bool fdmDataAvailable;

int main(int argc, char **argv) {
  auto log = CreateLogger("FDMReceiver");
  log->SetLogLevel(info);
  log->FlushLogOn(debug);

  ros::init(argc, argv, "simple_bluerov2_publisher");
  ros::NodeHandle nh;
  ros::Publisher bluerov2Pub;
  bluerov2Pub = nh.advertise<geometry_msgs::Pose>("/bluerov2/pose", 1);

  std::thread receiver([log]() {
    fdmDataAvailable = false;

    uint16_t localPort = 5503;
    int sockfd;
    struct sockaddr_in simAddr;
    struct sockaddr_in locAddr;
    sockfd = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
    memset(&locAddr, 0, sizeof(locAddr));
    locAddr.sin_family = AF_INET;
    locAddr.sin_addr.s_addr = INADDR_ANY;
    locAddr.sin_port = htons(localPort);
    if (fcntl(sockfd, F_SETFL, O_ASYNC) < 0) {
      close(sockfd);
      exit(EXIT_FAILURE);
    }
    if (-1 ==
        bind(sockfd, (struct sockaddr *)&locAddr, sizeof(struct sockaddr))) {
      perror("error bind failed");
      close(sockfd);
      exit(EXIT_FAILURE);
    }

    // ros::Rate rate(30);
    uint8_t rxbuff[GCS_BUFFER_LENGTH];

    uint32_t *version = (uint32_t *)rxbuff;
    uint32_t *padding = version + 1;
    double *longitude = (double *)(padding + 1);
    double *latitude = longitude + 1;
    double *altitude = latitude + 1;
    float *agl = (float *)(altitude + 1);
    float *phi = agl + 1;
    float *theta = phi + 1;
    float *psi = theta + 1;
    while (1) {
      memset(rxbuff, 0, GCS_BUFFER_LENGTH);
      socklen_t fromlen = sizeof(sockaddr);

      ssize_t recsize = recvfrom(sockfd, (void *)rxbuff, GCS_BUFFER_LENGTH, 0,
                                 (struct sockaddr *)&simAddr, &fromlen);
      if (recsize > 0) {
        fdmDataMutex.lock();
        if (true) {
          Utils::Switch4Bytes(&fdmData.version, version);
          Utils::Switch4Bytes(&fdmData.padding, padding);
          Utils::Switch8Bytes(&fdmData.longitude, longitude);
          Utils::Switch8Bytes(&fdmData.latitude, latitude);
          Utils::Switch8Bytes(&fdmData.altitude, altitude);
          Utils::Switch4Bytes(&fdmData.agl, agl);
          Utils::Switch4Bytes(&fdmData.phi, phi);
          Utils::Switch4Bytes(&fdmData.theta, theta);

        } else {
          fdmData.version = *version;
          fdmData.padding = *padding;
          fdmData.longitude = *longitude;
          fdmData.latitude = *latitude;
          fdmData.altitude = *altitude;
          fdmData.agl = *agl;
          fdmData.phi = *phi;
          fdmData.theta = *theta;
          fdmData.psi = *psi;
        }
        fdmDataAvailable = true;
        fdmDataCond.notify_one();
        log->Info("Received FDM data!!:\n"
                  "\tversion: {}\n"
                  "\tpadding: {}\n"
                  "\tlongitude: {:.9f}\n"
                  "\tlatitude: {:.9f}\n"
                  "\taltitude: {:.9f}\n"
                  "\tagl: {:.9f}\n"
                  "\tphi: {:.9f}\n"
                  "\ttheta: {:.9f}\n"
                  "\tpsi: {:.9f}",
                  fdmData.version, fdmData.padding, fdmData.longitude,
                  fdmData.latitude, fdmData.altitude, fdmData.agl, fdmData.phi,
                  fdmData.theta, fdmData.psi);
        fdmDataMutex.unlock();

      }
    }

  });
  ros::Rate rate(10);

  while (ros::ok()) {

    std::unique_lock<std::mutex> lock(fdmDataMutex);
    while (!fdmDataAvailable) {
      fdmDataCond.wait_for(lock, 2000ms);
      if (!fdmDataAvailable)
        log->Info("Waiting for FDM data...");
    }
    fdmDataAvailable = false;
//    log->Info("Received FDM data!!:\n"
//              "\tversion: {}\n"
//              "\tpadding: {}\n"
//              "\tlongitude: {:.9f}\n"
//              "\tlatitude: {:.9f}\n"
//              "\taltitude: {:.9f}\n"
//              "\tagl: {:.9f}\n"
//              "\tphi: {:.9f}\n"
//              "\ttheta: {:.9f}\n"
//              "\tpsi: {:.9f}",
//              fdmData.version, fdmData.padding, fdmData.longitude,
//              fdmData.latitude, fdmData.altitude, fdmData.agl, fdmData.phi,
//              fdmData.theta, fdmData.psi);
    //    control->WaitForNEDUpdate();
    //    auto pos = control->GetNED();
    //    auto gps = control->GetGPS();
    //    auto imu = control->GetScaledIMU2();
    //    auto attitude = control->GetAttitude();

    //    tf::Matrix3x3 rotMat;
    //    tfScalar yaw = attitude.yaw;
    //    tfScalar pitch = attitude.pitch;
    //    tfScalar roll = attitude.roll;
    //    rotMat.setRPY(roll, pitch, yaw);
    //    tf::Quaternion quaternion;
    //    rotMat.getRotation(quaternion);

    //    geometry_msgs::Pose bluerov2msg;
    //    bluerov2msg.position.x = pos.x;
    //    bluerov2msg.position.y = pos.y;
    //    bluerov2msg.position.z = -1;
    //    bluerov2msg.orientation.x = quaternion.x();
    //    bluerov2msg.orientation.y = quaternion.y();
    //    bluerov2msg.orientation.z = quaternion.z();
    //    bluerov2msg.orientation.w = quaternion.w();
    //    bluerov2Pub.publish(bluerov2msg);
    ros::spinOnce();
    rate.sleep();
  }
}
