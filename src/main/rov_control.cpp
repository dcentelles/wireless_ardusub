#include <iostream>

//ROS
#include <ros/ros.h>
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

//Logging
#include <Loggable.h>


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

static std::shared_ptr<spd::logger> Log = spd::stdout_color_mt("BlueROVControl");

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

static bool controlWorker_mustContinue_flag;

static bool messageSenderWorker_mustContinue_flag;
static std::thread messageSenderWorker;


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
        Log->debug("ROI conf.: {} -- shift: {}", roiparam, eparam->uroi_shift);
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
    Log->info("Cancel current order of movement");
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
                    Log->info("Message received from operator does not contain an order");
                    break;
                case wireless_ardusub::OperatorMessage::Move:
                    CancelCurrentMoveOrder();
                    Log->info("Received order of movement");
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
                    Log->critical("Order type is unknown");
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

void terminateOperatorMsgParserWorker(void)
{
    operatorMsgParserWorker_mustContinue_flag = false;
    currentOperatorMessage_cond.notify_one();
}

void HandleOperatorMessageReceived(ROVCamera & rovCamera)
{
    Log->info("Message received!");
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
        auto requestYaw = wireless_ardusub::utils::GetContinuousYaw(newMoveOrder->GetYaw());

        bool validOrder = true;
        if(newMoveOrder->Relative())
        {
            switch(newMoveOrder->GetFrame ())
            {
            case wireless_ardusub::HROVMoveOrder::Frame::ROV_FRAME:
            {

                break;
            }
            case wireless_ardusub::HROVMoveOrder::Frame::WORLD_FRAME:

                break;

            default:
                Log->critical("FRAME NOT SUPPORTED!");
                validOrder = false;
                break;
            }
        }


        if(validOrder)
        {

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

void initROSInterface(ros::NodeHandle & nh, int argc, char** argv,  ROVCamera & rovCamera)
{
    initMessages();

    startMessageSenderWorker(&rovCamera);
    startOperatorMsgParserWorker(&rovCamera);
    startControlWorker();
}

static void
defaultParams(struct debtEncParam *e, struct debtDecParam *d)
{
    debtEncParam_init(e);
    e->vector = 1;
    debtDecParam_init(d);
}


void GetParams(ros::NodeHandle & nh)
{
  eparam = new debtEncParam();
  defaultParams(eparam, &dparam);
}

int main(int argc, char** argv)
{
  std::cout << "ArduSub Wireless Control" << std::endl;

  //// GET PARAMS
  ros::init(argc, argv, "rov_control");
  ros::NodeHandle nh("~");

   GetParams(nh);

  return 0;
}
