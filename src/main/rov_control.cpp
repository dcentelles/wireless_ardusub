#include <iostream>

//ROS
#include <ros/ros.h>
#include <tf/transform_listener.h> //for position relative to ROV frame
//end ROS

#include <telerobotics/ROVCamera.h>

extern "C"
{
//image compression
#include <image_utils/image_utils.h>
#include <image_utils/libdebt.h>
}

//Sync
#include <mutex>
#include <condition_variable>

//Merbots
#include <wireless_ardusub/HROVMessage.h>
#include <wireless_ardusub/OperatorMessage.h>
#include <wireless_ardusub/utils.hpp>
#include <merbots_whrov_msgs/movement.h>
#include <merbots_whrov_msgs/position.h>
#include <merbots_whrov_msgs/hrov_settings.h>
//EndMerbots

#include <mavros_msgs/OverrideRCIn.h>
#include <sensor_msgs/Imu.h>
#include <mavros_msgs/VFR_HUD.h>

//Logging
#include <cpplogging/Logger.h>


//visp_ros-grabbing
#include <visp_ros/vpROSGrabber.h>

//ViSP formats and conversion
#include <visp3/core/vpImage.h>
#include <visp/vpRGBa.h>
#include <visp/vpImageConvert.h>
//end ViSP

//OpenCV
#include <opencv2/imgproc/imgproc.hpp>
//end OpenCV

using namespace std;
using namespace dcauv;

static LoggerPtr Log = cpplogging::CreateLogger("BlueROVControl");

static cv::Mat cvImage;
static cv::Mat cvImageResized;
static cv::Mat cvImageYuv;


struct Params
{
  std::string cameraTopic, masterUri;
  dcauv::LinkType linkType;
  int localAddr, remoteAddr;
  bool log2Console;
};

struct HROVPose
{
    double yaw, pitch, roll, Z, X, Y;
};

struct ProtocolConfig
{
        unsigned int frameSize;
        unsigned int maxPacketLength;
        unsigned int delayBetweenPackets;
        unsigned int width;
        unsigned int height;
        std::string camera, ltype;

	ProtocolConfig()
	{
	  delayBetweenPackets = 0;
	  maxPacketLength = 240;
	  frameSize = 400;
	  width = 352;
	  height = 288;
	  ltype = "fulld"; //link type
	}
};

static Params params;

static struct debtEncParam * eparam;
static struct debtDecParam dparam;

static ProtocolConfig pconfig;
static std::mutex config_mutex;

static std::thread controlWorker;

static wireless_ardusub::HROVMessagePtr currentHROVMessage;
static bool currentHROVMessage_updated;
static std::mutex currentHROVMessage_mutex;
static std::condition_variable currentHROVMessage_cond;

static wireless_ardusub::OperatorMessagePtr currentOperatorMessage;
static bool currentOperatorMessage_updated;
static std::mutex currentOperatorMessage_mutex;
static std::condition_variable currentOperatorMessage_cond;

static wireless_ardusub::HROVMoveOrderPtr newMoveOrder;
static bool newMoveOrderAvailable;
static std::mutex newMoveOrder_mutex;
static std::condition_variable newMoveOrder_cond;

static HROVPose currentHROVPose;
static std::mutex currentHROVPose_mutex;
static std::condition_variable currentHROVPose_cond;
static bool currentHROVPose_updated;

//operatorMsgParserWorker vars
static bool operatorMsgParserWorker_mustContinue_flag;
static std::thread operatorMsgParserWorker;
static std::thread headingWorker;

static bool controlWorker_mustContinue_flag;

static bool messageSenderWorker_mustContinue_flag;
static std::thread messageSenderWorker;

static ros::Publisher rcPublisher;
static ros::Subscriber ardusubNav_sub;
static ros::Subscriber ardusubHUD_sub;

static int rcDefVal = 1500;
static boost::array<int,8> rcDefault = { rcDefVal, rcDefVal, rcDefVal, rcDefVal, rcDefVal, rcDefVal, rcDefVal , rcDefVal};
static boost::array<int,8> rcIn = { rcDefVal, rcDefVal, rcDefVal, rcDefVal, rcDefVal, rcDefVal, rcDefVal , rcDefVal};
static bool keepHeading = false;
static uint16_t desiredHeading = 0;

void UpdateSettings(wireless_ardusub::HROVSettingsPtr settings, ROVCamera * rovCamera)
{
    merbots_whrov_msgs::hrov_settings::Ptr msg = settings->GetROSMsg();
    char roiparam[100];

    config_mutex.lock();
    debtEncParam_destroy(eparam);
    delete eparam;
    eparam = new debtEncParam();
    debtEncParam_init(eparam);
    eparam->vector = 1;
    eparam->nbands = 4;
    if(msg->image_config.roi_shift > 0)
    {
        sprintf(roiparam, "%d,%d,%d,%d",
                msg->image_config.roi_x0 & ~((1 << eparam->nbands) - 1), //nbands = 5 --> 0x1f ; nbands = 6 --> 0x3f ; etc
                msg->image_config.roi_y0 & ~((1 << eparam->nbands) - 1),
                msg->image_config.roi_x1 & ~((1 << eparam->nbands) - 1),
                msg->image_config.roi_y1 & ~((1 << eparam->nbands) - 1)
                );
        optGetRect(eparam, roiparam);
        eparam->uroi_shift = msg->image_config.roi_shift;
        Log->Debug("ROI conf.: {} -- shift: {}", roiparam, eparam->uroi_shift);
    }
    else
    {
        eparam->uroi_shift = 0;
    }

    int imgLength = (msg->image_config.size > 64 && msg->image_config.size <= 6000)
            ? msg->image_config.size : 64;
    pconfig.frameSize = imgLength;
    config_mutex.unlock();

    int requiredImageTrunkLength = (int)msg->protocol_config.max_packet_length - (int)wireless_ardusub::HROVMessage::MessageLength;
    int maxImageTrunkLength =
            ( requiredImageTrunkLength > 60 && requiredImageTrunkLength <= 6000)
            ? msg->protocol_config.max_packet_length : 60;
    rovCamera->SetMaxImageTrunkLength(maxImageTrunkLength);
}

void CancelCurrentMoveOrder()
{
    Log->Info("Cancel current order of movement");
    keepHeading = false;
    mavros_msgs::OverrideRCIn rcMsg;
    rcMsg.channels = rcDefault;
    rcPublisher.publish(rcMsg);
}

void operatorMsgParserWork(ROVCamera * rovCamera)
{
    while(1)
    {
        std::unique_lock<std::mutex> lock(currentOperatorMessage_mutex);
        while(!currentOperatorMessage_updated)
        {
            if(operatorMsgParserWorker_mustContinue_flag)
                currentOperatorMessage_cond.wait(lock);
            else
                return;
        }
        currentOperatorMessage_updated = false;

        auto settings = currentOperatorMessage->GetSettingsCopy();
        UpdateSettings(settings, rovCamera);

        wireless_ardusub::HROVMoveOrderPtr _moveOrder;
        currentHROVMessage_mutex.lock();
        if(currentHROVMessage->GetExpectedOrderSeqNumber() == currentOperatorMessage->GetOrderSeqNumber())
        {
            if(currentOperatorMessage->CancelLastOrderFlag())
            {
                CancelCurrentMoveOrder();
                currentHROVMessage->LastOrderCancelledFlag(true);
                currentHROVMessage->Ready(true);
                currentHROVMessage_updated = true;
            }
            else
            {
                switch(currentOperatorMessage->GetOrderType())
                {

                case wireless_ardusub::OperatorMessage::NoOrder:
                    Log->Info("Message received from operator does not contain an order");
                    break;
                case wireless_ardusub::OperatorMessage::Move:
                    CancelCurrentMoveOrder();
                    Log->Info("Received order of movement");
                    if(!currentHROVMessage->Ready())
                        CancelCurrentMoveOrder ();
                    currentHROVMessage->LastOrderCancelledFlag(false);
                    currentHROVMessage->Ready(false);
                    currentHROVMessage->IncExpectedOrderSeqNumber();
                    _moveOrder = currentOperatorMessage->GetMoveOrderCopy();

                    currentHROVMessage_updated = true;

                    newMoveOrder_mutex.lock();
                    newMoveOrder = _moveOrder;
                    newMoveOrderAvailable = true;
                    newMoveOrder_mutex.unlock();
                    newMoveOrder_cond.notify_one();

                    break;
                default:
                    Log->Critical("Order type is unknown");
                    break;
                }
            }
        }
        currentHROVMessage_mutex.unlock();
        if(currentHROVMessage_updated)
            currentHROVMessage_cond.notify_one();

    }
}

void startOperatorMsgParserWorker(ROVCamera * rovCamera)
{
    operatorMsgParserWorker_mustContinue_flag = true;
    operatorMsgParserWorker = std::thread(operatorMsgParserWork, rovCamera);
}

/**
   Length (angular) of a shortest way between two angles.
  It will be in range [0, 180].

 private int distance(int alpha, int beta) {
     int phi = Math.abs(beta - alpha) % 360;       // This is either the distance or 360 - distance
     int distance = phi > 180 ? 360 - phi : phi;
     return distance;
 }
*/

int angleDistance(int alpha, int beta)
{
  int phi = std::abs(beta - alpha) % 360;       // This is either the distance or 360 - distance
  int distance = phi > 180 ? 360 - phi : phi;
  return distance;
}

void headingWorker_work(void)
{
  auto inc = 55;
  while(1)
  {
    int currentHeading = std::round(currentHROVPose.yaw);
    int ahdiff = angleDistance(currentHeading, desiredHeading);

    bool right = true;
    if(ahdiff + currentHeading % 360 == desiredHeading)
      right = false;

    if(keepHeading && ahdiff > 0)
    {
       if(right > 0)
         rcIn[3] = rcDefVal+inc;
       else
         rcIn[3] = rcDefVal-inc;
    }
    else
    {
      rcIn[3] = rcDefVal;
    }
    mavros_msgs::OverrideRCIn rcMsg;
    rcMsg.channels = rcIn;
    rcPublisher.publish(rcMsg);

    this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

void startHeadingWorker(void)
{
    headingWorker = std::thread(headingWorker_work);
}

void terminateOperatorMsgParserWorker(void)
{
    operatorMsgParserWorker_mustContinue_flag = false;
    currentOperatorMessage_cond.notify_one();
}

void HandleOperatorMessageReceived(ROVCamera & rovCamera)
{
    Log->Info("Message received!");
    currentOperatorMessage_mutex.lock();
    rovCamera.GetCurrentRxState(currentOperatorMessage->GetBuffer());
    currentOperatorMessage_mutex.unlock();

    currentOperatorMessage_updated = true;

    /*FROM http://en.cppreference.com/w/cpp/thread/condition_variable/notify_one:
     * The notifying thread does not need to hold the lock on the same mutex as the one held by the waiting thread(s);
     * in fact doing so is a pessimization, since the notified thread would immediately block again,
     * waiting for the notifying thread to release the lock. However, some implementations (in particular
     * many implementations of pthreads) recognize this situation and avoid this "hurry up and wait" scenario
     * by transferring the waiting thread from the condition variable's queue directly to the queue of
     * the mutex within the notify call, without waking it up.
     * */
    currentOperatorMessage_cond.notify_one();
}

void controlWorker_work(void)
{
    while(1)
    {
        std::unique_lock<std::mutex> lock(newMoveOrder_mutex);
        while(!newMoveOrderAvailable)
        {
            if(controlWorker_mustContinue_flag)
                newMoveOrder_cond.wait(lock);
            else
                return;
        }

        //2: handle the new order of movement
        
        std::unique_lock<std::mutex> poseLock(currentHROVPose_mutex);
        while(!currentHROVPose_updated)
        {
            currentHROVPose_cond.wait(poseLock);
        }
        auto currentZ = currentHROVPose.Z;
        auto currentX = currentHROVPose.X;
        auto currentY = currentHROVPose.Y;
        auto currentYaw = currentHROVPose.yaw;
        auto currentPitch = currentHROVPose.pitch;
        auto currentRoll = currentHROVPose.roll;

        poseLock.unlock();

        //from dm to m
        auto requestZ = newMoveOrder->GetZ() / 10.;
        auto requestX = newMoveOrder->GetX() / 10.;
        auto requestY = newMoveOrder->GetY() / 10.;
        auto requestYaw = newMoveOrder->GetYaw();//wireless_ardusub::utils::GetContinuousYaw(newMoveOrder->GetYaw());

        bool validOrder = true;

        mavros_msgs::OverrideRCIn rcMsg;

        int inc = 500;
        int millis = 1000;
        if(newMoveOrder->Relative())
        {
            switch(newMoveOrder->GetFrame ())
            {
            case wireless_ardusub::HROVMoveOrder::Frame::ROV_FRAME:
            {
                if(requestZ != 0)
                   rcIn[2] = rcDefVal + (int) (requestZ*inc);

                if(requestX != 0)
                   rcIn[5] = rcDefVal + (int) (requestX*inc);

                if(requestY != 0)
                {
                  rcIn[0] = rcDefVal + (int) (requestY*inc);
                  rcIn[6] = rcDefVal + (int) (requestY*inc);
                }

                if(requestYaw != 0)
                {
                  desiredHeading = requestYaw;
                  keepHeading = true;
                }

                break;
            }
            case wireless_ardusub::HROVMoveOrder::Frame::WORLD_FRAME:

                break;

            default:
                Log->Critical("FRAME NOT SUPPORTED!");
                validOrder = false;
                break;
            }
        }

        if(validOrder)
        {
            rcMsg.channels = rcIn;
            rcPublisher.publish(rcMsg);
            this_thread::sleep_for(std::chrono::milliseconds(millis));
            rcIn[0] = rcDefVal;
            rcIn[2] = rcDefVal;
            rcIn[5] = rcDefVal;
            rcIn[6] = rcDefVal;
            rcMsg.channels = rcIn;
            rcPublisher.publish(rcMsg);
        }

        currentHROVMessage_mutex.lock();
        currentHROVMessage->Ready(true);
        currentHROVMessage_updated = true;
        currentHROVMessage_mutex.unlock();
        currentHROVMessage_cond.notify_one();

        /*NOTE: lock.unlock() is not needed here:
        when the lock object is out of scope
        its destructor (of the "unique_lock" type) is called,
        and it unlocks the mutex.
        */
        newMoveOrderAvailable = false;
    }
}

void startControlWorker(void)
{
    controlWorker_mustContinue_flag = true;
    controlWorker = std::thread(controlWorker_work);
}

void terminateControlWorker(void)
{
    controlWorker_mustContinue_flag = false;
    newMoveOrder_cond.notify_one();
}

void messageSenderWork(ROVCamera * rovCamera)
{
    while(1)
    {
        std::unique_lock<std::mutex> lock(currentHROVMessage_mutex);
        while(!currentHROVMessage_updated)
        {
            if(messageSenderWorker_mustContinue_flag)
                currentHROVMessage_cond.wait(lock);
            else
                return;
        }
        rovCamera->SetCurrentTxState(currentHROVMessage->GetBuffer());
        currentHROVMessage_updated = false;
    }
}

void startMessageSenderWorker(ROVCamera * rovCamera)
{
    messageSenderWorker_mustContinue_flag = true;
    messageSenderWorker = std::thread(messageSenderWork, rovCamera);
}

void terminateMessageSenderWorker(void)
{
    messageSenderWorker_mustContinue_flag = false;
    currentOperatorMessage_cond.notify_one();
}

void initMessages(void)
{
    currentHROVMessage = wireless_ardusub::HROVMessage::BuildHROVMessage();
    currentOperatorMessage = wireless_ardusub::OperatorMessage::BuildOperatorMessage();

    currentHROVMessage->Ready(true);
    currentOperatorMessage_updated = false;
    currentHROVMessage_updated = false;
    currentHROVPose_updated = false;
}

void HandleNewHUDData(const mavros_msgs::VFR_HUD::ConstPtr & msg, ROVCamera * rovCamera)
{
  currentHROVMessage_mutex.lock();
  currentHROVMessage->SetYaw (msg->heading);
  currentHROVMessage_updated = true;
  currentHROVMessage_mutex.unlock();
  currentHROVMessage_cond.notify_one();

  currentHROVPose_mutex.lock();
  currentHROVPose.yaw = msg->heading;
  currentHROVPose_updated = true;
  currentHROVPose_mutex.unlock();
  currentHROVPose_cond.notify_one();
}

void HandleNewNavigationData(const sensor_msgs::Imu::ConstPtr & msg, ROVCamera * rovCamera)
{
  //the following is not correct! x, y z is the components of a Quaternion, not the Euler angles.

  tf::Quaternion rotation(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w);

  tf::Matrix3x3 rotMat(rotation);

  tfScalar yaw, pitch, roll;
  rotMat.getEulerYPR (yaw,pitch,roll);

  int rx,ry,rz;
  rx = wireless_ardusub::utils::GetDiscreteYaw(roll);
  ry = wireless_ardusub::utils::GetDiscreteYaw(pitch);
  rz = wireless_ardusub::utils::GetDiscreteYaw(yaw);

  rx = rx > 180 ? -(360-rx) : rx;
  ry = ry > 180 ? -(360-ry) : ry;
  //rz = rz > 180 ? -(360-rz) : rz;

  currentHROVMessage_mutex.lock();
  //currentHROVMessage->SetYaw (rz);
  currentHROVMessage->SetX(rx);
  currentHROVMessage->SetY(ry);
  currentHROVMessage_updated = true;
  currentHROVMessage_mutex.unlock();
  currentHROVMessage_cond.notify_one();

  currentHROVPose_mutex.lock();
  //currentHROVPose.yaw = 0;
  currentHROVPose_updated = true;
  currentHROVPose_mutex.unlock();
  currentHROVPose_cond.notify_one();

}

void initROSInterface(ros::NodeHandle & nh, int argc, char** argv,  ROVCamera & rovCamera)
{
    initMessages();

    startMessageSenderWorker(&rovCamera);
    startOperatorMsgParserWorker(&rovCamera);
    startControlWorker();

    ardusubNav_sub = nh.subscribe<sensor_msgs::Imu>(
            "/mavros/imu/data",
            1,
            boost::bind(HandleNewNavigationData, _1, &rovCamera)
         );

    ardusubHUD_sub = nh.subscribe<mavros_msgs::VFR_HUD>(
            "/mavros/vfr_hud",
            1,
            boost::bind(HandleNewHUDData, _1, &rovCamera)
         );

    currentHROVMessage_mutex.lock();
    currentHROVMessage->SetYaw(0);
    currentHROVMessage->SetZ(0);
    currentHROVMessage->SetX(0);
    currentHROVMessage->SetY(0);
    currentHROVMessage_updated = true;
    currentHROVMessage_mutex.unlock();
    currentHROVMessage_cond.notify_one();

    rcPublisher = nh.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 1);


    startHeadingWorker();
}

static void
defaultParams(struct debtEncParam *e, struct debtDecParam *d)
{
    debtEncParam_init(e);
    e->vector = 1;
    debtDecParam_init(d);
}


int GetParams(ros::NodeHandle & nh)
{
  eparam = new debtEncParam();
  defaultParams(eparam, &dparam);

  std::string cameraTopic;
  if(!nh.getParam("image",cameraTopic))
  {
      ROS_ERROR("Failed to get param image");
      return 1;
  }
  else
  {
     ROS_INFO("camera topic: %s",cameraTopic.c_str());
  }
  params.cameraTopic = cameraTopic;

  char * cmasterUri = getenv ("ROS_MASTER_URI");
  std::string masterUri = cmasterUri;
  ROS_INFO("ROS MASTER URI: %s",masterUri.c_str());

  params.masterUri = masterUri;

  int remoteAddr;
  if(!nh.getParam("remoteAddr",remoteAddr))
  {
      ROS_ERROR("Failed to get param remoteAddr");
      return 1;
  }
  else
  {
     ROS_INFO("Remote addr: %d", remoteAddr);
  }

  params.remoteAddr = remoteAddr;

  int localAddr;
  if(!nh.getParam("localAddr",localAddr))
  {
      ROS_ERROR("Failed to get param localAddr");
      return 1;
  }
  else
  {
     ROS_INFO("local addr: %d", localAddr);
  }

  params.localAddr = localAddr;

  bool log2Console;
  if(!nh.getParam("log2Console",log2Console))
  {
      ROS_ERROR("Failed to get param log2Console");
      return 1;
  }
  else
  {
     ROS_INFO("log2Console: %d", log2Console);
  }

  params.log2Console = log2Console;

  LinkType linkType;
  std::string slinkType;
  if(!nh.getParam("linkType",slinkType))
  {
      ROS_ERROR("Failed to get param linkType");
      return 1;
  }
  else
  {
      if(slinkType == "fulld")
      {
          linkType = LinkType::fullDuplex;
      }
      else if (slinkType == "halfd")
      {
          linkType = LinkType::halfDuplex;
      }
      else
      {
          ROS_ERROR("wrong linkType value '%s'", slinkType.c_str ());
          return 1;
      }
      ROS_INFO("linkType: %s", slinkType.c_str ());
  }
  params.linkType = linkType;
  return 0;
}

int main(int argc, char** argv)
{
  std::cout << "ArduSub Wireless Control" << std::endl;

  //// GET PARAMS
  ros::init(argc, argv, "rov_control");
  ros::NodeHandle nh("~");

  if(GetParams(nh))
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
  sa.sa_flags   = SA_SIGINFO;

  sigaction(SIGSEGV, &sa, NULL);


  struct imgBuffer *img = NULL;
  /* prepare an image buffer for the input image */
  if ((img = imgBuffer_aligned_new(1))) {
          if (!imgBuffer_reinit(img, pconfig.width, pconfig.height, COLORFORMAT_FMT_420))
                  return 1;
  }
  else return 1;

  /* setup the output buffer */
  unsigned char *buffer = NULL;
  int buflen = 0;
  int fixed = 0;
  if (eparam->maxsize)
  {
    buffer = (unsigned char *) malloc(buflen = eparam->maxsize);
    fixed = 1;
  }

  ROVCamera rovCamera(params.linkType);
  rovCamera.SetLocalAddr(params.localAddr);
  rovCamera.SetRemoteAddr(params.remoteAddr);

  rovCamera.SetLogLevel(cpplogging::LogLevel::debug);
  rovCamera.FlushLogOn(cpplogging::LogLevel::info);
  rovCamera.LogToConsole(params.log2Console);
  //rovCamera.LogToFile("camera_comms_whrov_log");

  Log->SetLogLevel(cpplogging::LogLevel::debug);
  Log->FlushLogOn(cpplogging::LogLevel::info);

  cvImageResized.create(pconfig.height, pconfig.width, CV_8UC3);
  //http://stackoverflow.com/questions/14897525/create-yuv-mat-in-opencv-on-android
  cvImageYuv.create(pconfig.height + pconfig.height/2, pconfig.width, CV_8UC1); //We allocate the buffer for a yuv420 size
  cvImageYuv.data = img->buffer; //imgBuffer will share the same buf

  try
  {
      rovCamera.SetTxStateSize(wireless_ardusub::HROVMessage::MessageLength);
      rovCamera.SetRxStateSize(wireless_ardusub::OperatorMessage::MessageLength);
      rovCamera.SetMaxImageTrunkLength(pconfig.maxPacketLength-wireless_ardusub::HROVMessage::MessageLength);
      rovCamera.SetOrdersReceivedCallback(&HandleOperatorMessageReceived);

      ros::Rate rate(30); // 30 hz
      initROSInterface(nh, argc, argv, rovCamera);

      rovCamera.Start();
      while(ros::ok())
      {
          if(!rovCamera.SendingCurrentImage())
          {
              /////////CAPTURE
              grabber.acquire(Image);

	      /////////TRANSFORM
	      //ViSP has not a method to convert RGB to YUV420, so we convert first to an OpenCV type
	      vpImageConvert::convert(Image, cvImage);
	      cv::resize(cvImage, cvImageResized, cv::Size(pconfig.width, pconfig.height));
	      cv::cvtColor(cvImageResized, cvImageYuv, CV_RGB2YUV_I420);

	      config_mutex.lock(); //At the moment, only the compression parameters will be dynamic

              /////////COMPRESS
              int res = encode(eparam, &buffer, buflen, fixed, img);

	      /////////PREPARE TO SEND
	      if(res)
	      {
		      Log->Info("Sending the new captured image...");
		      rovCamera.SendImage(buffer, pconfig.frameSize);
	      }
	      config_mutex.unlock();
	  }
	  ros::spinOnce();
	  rate.sleep();
      }
  }
  catch(std::exception& e)
  {
          Log->Error("Exception: {}", e.what());
          exit(1);
  }
  return 0;
}
