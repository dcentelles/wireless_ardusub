/*
 * Adaptation of:
 *  File: bluerov_apps/src/teleop_joy.cpp
 *  Author: Josh Villbrandt <josh@javconcepts.com>
 *  Date: February 2016
 *  Description: Manual remote control of ROVs like the bluerov_apps.
 * By centelld@uji.es for the wireless bluerov project
 */

#include <chrono>
#include <cpplogging/cpplogging.h>
#include <mavlink_cpp/mavlink_cpp.h>
#include <wireless_ardusub/TeleopOrder.h>
#include <wireless_ardusub/nodes/ROV.h>

extern "C" {
// image compression
#include <image_utils/image_utils.h>
#include <image_utils/libdebt.h>
}

// visp_ros-grabbing
#include <visp_ros/vpROSGrabber.h>

// ViSP formats and conversion
#include <visp/vpImageConvert.h>
#include <visp/vpRGBa.h>
#include <visp3/core/vpImage.h>
// end ViSP

// OpenCV
#include <opencv2/imgproc/imgproc.hpp>
// end OpenCV

using namespace cpplogging;
using namespace std::chrono_literals;
using namespace wireless_ardusub;
using namespace mavlink_cpp;
using namespace std;

struct Params {
  std::string cameraTopic, masterUri;
  bool log2Console;
};

struct ProtocolConfig {
  unsigned int width;
  unsigned int height;
  unsigned int frameSize;
  ProtocolConfig() {
    width = 352;
    height = 288;
    frameSize = 100;
  }
};

static struct debtEncParam *eparam;
static struct debtDecParam dparam;
static LoggerPtr Log;
static Params params;
static ProtocolConfig pconfig;
static std::mutex config_mutex;
static cv::Mat cvImage;
static cv::Mat cvImageResized;
static cv::Mat cvImageYuv;

static void defaultParams(struct debtEncParam *e, struct debtDecParam *d) {
  debtEncParam_init(e);
  e->vector = 1;
  debtDecParam_init(d);
}

int GetParams(ros::NodeHandle &nh) {
  eparam = new debtEncParam();
  defaultParams(eparam, &dparam);

  std::string cameraTopic;
  if (!nh.getParam("image", cameraTopic)) {
    ROS_ERROR("Failed to get param image");
    return 1;
  } else {
    ROS_INFO("camera topic: %s", cameraTopic.c_str());
  }
  params.cameraTopic = cameraTopic;

  char *cmasterUri = getenv("ROS_MASTER_URI");
  std::string masterUri = cmasterUri;
  ROS_INFO("ROS MASTER URI: %s", masterUri.c_str());

  params.masterUri = masterUri;

  bool log2Console;
  if (!nh.getParam("log2Console", log2Console)) {
    ROS_ERROR("Failed to get param log2Console");
    return 1;
  } else {
    ROS_INFO("log2Console: %d", log2Console);
  }

  params.log2Console = log2Console;

  return 0;
}

void initROSInterface(ros::NodeHandle &nh, int argc, char **argv,
                      dccomms::Ptr<ROV> commsNode) {}

int main(int argc, char **argv) {
  Log = CreateLogger("TeleopROV");
  Log->Info("Init");

  //// GET PARAMS
  ros::init(argc, argv, "rov_control");
  ros::NodeHandle nh("~");

  if (GetParams(nh))
    return 1;

  //////////// GRABBER AND ENCODER SETUP
  vpImage<vpRGBa> Image;
  vpROSGrabber grabber;
  grabber.setMasterURI(params.masterUri);
  grabber.setImageTopic(params.cameraTopic);
  grabber.open(Image);

  struct sigaction sa;
  memset(&sa, 0, sizeof(sa));
  sigemptyset(&sa.sa_mask);
  sa.sa_sigaction = segfault_sigaction;
  sa.sa_flags = SA_SIGINFO;

  sigaction(SIGSEGV, &sa, NULL);

  struct imgBuffer *img = NULL;
  /* prepare an image buffer for the input image */
  if ((img = imgBuffer_aligned_new(1))) {
    if (!imgBuffer_reinit(img, pconfig.width, pconfig.height,
                          COLORFORMAT_FMT_420))
      return 1;
  } else
    return 1;

  /* setup the output buffer */
  unsigned char *buffer = NULL;
  int buflen = 0;
  int fixed = 0;
  if (eparam->maxsize) {
    buffer = (unsigned char *)malloc(buflen = eparam->maxsize);
    fixed = 1;
  }

  Log->SetLogLevel(cpplogging::LogLevel::debug);
  Log->FlushLogOn(cpplogging::LogLevel::info);

  cvImageResized.create(pconfig.height, pconfig.width, CV_8UC3);
  // http://stackoverflow.com/questions/14897525/create-yuv-mat-in-opencv-on-android
  cvImageYuv.create(pconfig.height + pconfig.height / 2, pconfig.width,
                    CV_8UC1);    // We allocate the buffer for a yuv420 size
  cvImageYuv.data = img->buffer; // imgBuffer will share the same buf

  TeleopOrderPtr order = TeleopOrder::Build();

  dccomms::Ptr<ROV> commsNode;
  uint16_t localPort = 14550;
  mavlink_cpp::Ptr<GCS> control = mavlink_cpp::CreateObject<GCS>(localPort);
  control->SetLogName("GCS");
  control->SetLogLevel(debug);
  control->Start();

  commsNode->SetOrdersReceivedCallback([order, control](ROV &receiver) {});

  Log->SetLogLevel(LogLevel::info);
  commsNode->SetLogLevel(LogLevel::info);
  commsNode->Start();

  try {

    ros::Rate rate(30); // 30 hz
    initROSInterface(nh, argc, argv, commsNode);

    while (ros::ok()) {
      if (!commsNode->SendingCurrentImage()) {
        /////////CAPTURE
        grabber.acquire(Image);

        /////////TRANSFORM
        // ViSP has not a method to convert RGB to YUV420, so we convert first
        // to an OpenCV type
        vpImageConvert::convert(Image, cvImage);
        cv::resize(cvImage, cvImageResized,
                   cv::Size(pconfig.width, pconfig.height));
        cv::cvtColor(cvImageResized, cvImageYuv, CV_RGB2YUV_I420);

        config_mutex.lock(); // At the moment, only the compression parameters
                             // will be dynamic

        /////////COMPRESS
        int res = encode(eparam, &buffer, buflen, fixed, img);

        /////////PREPARE TO SEND
        if (res) {
          Log->Info("Sending the new captured image...");
          commsNode->SendImage(buffer, pconfig.frameSize);
        }
        config_mutex.unlock();
      }
      ros::spinOnce();
      rate.sleep();
    }
  } catch (std::exception &e) {
    Log->Error("Exception: {}", e.what());
    exit(1);
  }
  return 0;
}
