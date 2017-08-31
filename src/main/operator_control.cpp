#include <iostream>
#include <thread>         // std::this_thread::sleep_for
#include <chrono>			// std::chrono::seconds
#include <stdint.h>
#include <telerobotics/ROVOperator.h>
#include <stdlib.h>
#include <cstdio>
#include <sys/stat.h>
#include <cstring>
#include <unistd.h>
#include <sys/time.h>
#include <pthread.h>
#include <mutex>
#include <condition_variable>

//ROS and image_transport
#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <merbots_whrov_msgs/movement.h>
#include <merbots_whrov_msgs/position.h>
#include <merbots_whrov_msgs/hrov_settings.h>
#include <merbots_whrov_msgs/MoveOrderAction.h>

#include "std_msgs/String.h"
#include <actionlib/server/simple_action_server.h>

//end ROS

//Merbots
#include <wireless_ardusub/HROVMessage.h>
#include <wireless_ardusub/OperatorMessage.h>
//EndMerbots

extern "C"
{
#include <image_utils/libdebt.h>
#include <image_utils/image_utils.h>
}


using namespace std;
using namespace dcauv;

static std::shared_ptr<spd::logger> Log = spd::stdout_color_mt("OperatorMain");
static struct debtEncParam e;
static struct debtDecParam d;
static uint8_t * imgBuffer;
static unsigned int g_iwidth;
static unsigned int g_iheight;
struct imgBuffer * img;
static cv::Mat myuv, mrgb;

//static image_transport::CameraPublisher camera_pub;
static image_transport::Publisher camera_pub;
static ros::Publisher  currentHROVState_pub;
static ros::Subscriber desiredHROVState_sub;
static sensor_msgs::ImagePtr camera_msg;

static uint8_t * y, *u, *v;
static wireless_ardusub::OperatorMessagePtr currentOperatorMessage;
static wireless_ardusub::HROVMessagePtr currentHROVMessage;
static std::mutex currentOperatorMessage_mutex;
static std::mutex currentHROVMessage_mutex;
static std::condition_variable currentOperatorMessage_cond;
static std::condition_variable currentHROVMessage_cond;
static bool currentOperatorMessage_updated;
static bool currentHROVMessage_updated;


//actionlib objects

class MoveOrderActionServer
{
public:
  void NotifyNewHROVState(bool _hrovReady, int _requestedOID, bool cancelled)
  {
    hrovState_mutex.lock();
    hrovReady = _hrovReady;
    requestedOID = _requestedOID;
    lastOrderCancelled = cancelled;
    hrovState_mutex.unlock();
    hrovState_updated_cond.notify_one();
  }

  //http://wiki.ros.org/actionlib_tutorials/Tutorials/SimpleActionServer%28ExecuteCallbackMethod%29
protected:
  std::string actionName;
  actionlib::SimpleActionServer<merbots_whrov_msgs::MoveOrderAction> actionServer;
  merbots_whrov_msgs::MoveOrderFeedback moveOrderFeedback;
  merbots_whrov_msgs::MoveOrderResult moveOrderResult;

  int oid, requestedOID;
  bool hrovReady, lastOrderCancelled;
  bool transmittingOrder;

  std::mutex hrovState_mutex;
  std::condition_variable hrovState_updated_cond;
public:

  MoveOrderActionServer(std::string name, ros::NodeHandle nh):
    actionServer(nh, name, boost::bind(&MoveOrderActionServer::actionWorker, this, _1), false ),
    actionName(name), oid(0), requestedOID(0), hrovReady(false), transmittingOrder(false)
  {}
  ~MoveOrderActionServer(void)
  {
  }
  void Start()
  {
    actionServer.start();
  }


  bool CancelRequested(void)
  {
    if(actionServer.isPreemptRequested() || !ros::ok())
    {
      Log->info("{}: Preempted", actionName);
      actionServer.setPreempted();
      return true;
    }
    return false;
  }

  void CancelLastOrder(void)
  {
    //Puede que la orden se haya enviado ya, por lo que hay que cancelarla
    currentOperatorMessage_mutex.lock();
    currentOperatorMessage->CancelLastOrderFlag(true);
    currentOperatorMessage->SetOrderSeqNumber(requestedOID);
    currentOperatorMessage->SetOrderType (wireless_ardusub::OperatorMessage::NoOrder);
    currentOperatorMessage_updated = true;
    currentOperatorMessage_mutex.unlock();
    currentOperatorMessage_cond.notify_one();
  }

  void actionWorker(const merbots_whrov_msgs::MoveOrderGoalConstPtr &goal)
  {
    auto moveOrder = wireless_ardusub::HROVMoveOrder::BuildHROVMoveOrder();

    moveOrder->SetYaw(goal->order.yaw);
    moveOrder->SetZ(goal->order.Z);
    moveOrder->SetX(goal->order.X);
    moveOrder->SetY(goal->order.Y);
    moveOrder->Relative(goal->order.relative);

    std::unique_lock<std::mutex> lock(hrovState_mutex);

    moveOrderFeedback.percent_complete = 5;
    moveOrderFeedback.message = "Waiting for the ROV to get ready to handle a new order...";
    actionServer.publishFeedback(moveOrderFeedback);

    while(transmittingOrder)
    {
      hrovState_updated_cond.wait_for(lock, std::chrono::seconds(1));
      if(CancelRequested())
      {
        moveOrderFeedback.percent_complete = 100;
        moveOrderFeedback.message = "Last order cancelled";
        actionServer.publishFeedback(moveOrderFeedback);
        lock.unlock();
        return;
      }
    }
    //Si el robot recibe una nueva orden con el OID
    //que espera y está en curso alguna orden, cancelará la orden actual para ejecutar la nueva.
    transmittingOrder = true;
    oid = requestedOID;
    lock.unlock();

    moveOrderFeedback.percent_complete = 10;
    moveOrderFeedback.message = "Sending the new position request...";
    actionServer.publishFeedback(moveOrderFeedback);

    currentOperatorMessage_mutex.lock();
    currentOperatorMessage->SetOrderSeqNumber(oid);
    currentOperatorMessage->SetOrderType(wireless_ardusub::OperatorMessage::Move);
    currentOperatorMessage->SetMoveOrder(moveOrder);
    currentOperatorMessage->CancelLastOrderFlag(false);
    currentOperatorMessage_updated = true;
    currentOperatorMessage_mutex.unlock();
    currentOperatorMessage_cond.notify_one();

    moveOrderFeedback.percent_complete = 15;
    moveOrderFeedback.message = "Waiting for the order acknowledgment...";
    actionServer.publishFeedback(moveOrderFeedback);

    int nextOID = wireless_ardusub::HROVMessage::GetNextOrderSeqNumber(oid);
    lock.lock();
    while(requestedOID != nextOID)
    {
      hrovState_updated_cond.wait_for(lock, std::chrono::seconds(1));
      if(CancelRequested())
      {
        moveOrderFeedback.percent_complete = 50;
        moveOrderFeedback.message = "Cancelling last order...";
        actionServer.publishFeedback(moveOrderFeedback);
        do
        {
          CancelLastOrder();
          //Hay que esperar la confirmación de que el ROV ha recibido la orden de cancelación de la requestedOID
          /*Si esto no se hace, y se da otra orden antes de que el rov haya recibido la orden anterior, la nuevo orden
                      puede llegar a enviarse con el mismo OID que la anterior. Que pasaría? Veamos un ejemplo:
                        1. Se envía una orden con OID 1.
                        2. Antes de que el ROV reciba la orden con OID 1, se cancela la orden para enviar otra.
                        3. Como el ROV todavía no ha recibido la orden anterior, su estado es de Ready, y su requested OID sigue siendo 1.
                        4. Al tener el estado de Ready, se envía la nueva orden con OID 1 y se espera la recepción de requested OID igual a 2
                        5. En ese momento llega la antigua orden al ROV (la que tiene OID 1, al igual que la nueva orden). El robot la ejecuta, ya que tiene la misma OID
                            que la que espera. Mientras, cambia la requested OID a la 2
                        6. Seguidamente llega al ROV la nueva orden, con OID 1. Sin embargo, como el ROV espera la OID = 2 la descarta.
                        7. Cuando el operador reciba un paquete con requested OID = 2, pensará que la nueva orden se ha enviado correctamente. Sin embargo,
                           lo que ha pasado es que el robot ha ejecutado la orden anterior.
                        */
          hrovState_updated_cond.wait_for(lock, std::chrono::seconds(1));
        }
        while(!lastOrderCancelled);

        currentOperatorMessage_mutex.lock();
        currentOperatorMessage->CancelLastOrderFlag(false);
        currentOperatorMessage_updated = true;
        currentOperatorMessage_mutex.unlock();
        currentOperatorMessage_cond.notify_one();

        moveOrderFeedback.percent_complete = 100;
        moveOrderFeedback.message = "Last order cancelled";
        actionServer.publishFeedback(moveOrderFeedback);
        transmittingOrder = false;
        lock.unlock();
        return;
      }
    }
    currentOperatorMessage_mutex.lock();
    currentOperatorMessage->SetOrderType(wireless_ardusub::OperatorMessage::NoOrder);
    currentOperatorMessage_updated = true;
    currentOperatorMessage_mutex.unlock();
    currentOperatorMessage_cond.notify_one();

    moveOrderFeedback.percent_complete = 60;
    moveOrderFeedback.message = "ROV received the order.";
    actionServer.publishFeedback(moveOrderFeedback);

    while(!hrovReady)
    {
      hrovState_updated_cond.wait_for(lock, std::chrono::seconds(1));
      if(CancelRequested())
      {
        moveOrderFeedback.percent_complete = 50;
        moveOrderFeedback.message = "Cancelling last order...";
        actionServer.publishFeedback(moveOrderFeedback);
        do
        {
          CancelLastOrder();
          hrovState_updated_cond.wait_for(lock, std::chrono::seconds(1));
        }
        while(!lastOrderCancelled);


        currentOperatorMessage_mutex.lock();
        currentOperatorMessage->CancelLastOrderFlag(false);
        currentOperatorMessage_updated = true;
        currentOperatorMessage_mutex.unlock();
        currentOperatorMessage_cond.notify_one();

        moveOrderFeedback.percent_complete = 100;
        moveOrderFeedback.message = "Last order cancelled";
        actionServer.publishFeedback(moveOrderFeedback);
        transmittingOrder = false;
        lock.unlock();
        return;
      }
    }
    transmittingOrder = false;
    lock.unlock();

    moveOrderFeedback.percent_complete = 100;
    moveOrderFeedback.message = "Received last order completion confirmation";
    actionServer.publishFeedback(moveOrderFeedback);
    moveOrderResult.success = true;
    actionServer.setSucceeded(moveOrderResult);
  }

};
static MoveOrderActionServer * orderActionServer;

//end actionlib objects

//messageSenderWorker vars
static bool messageSenderWorker_mustContinue_flag;
static std::thread messageSenderWorker;
void messageSenderWork(ROVOperator * rovOperator)
{
  while(1)
  {
    std::unique_lock<std::mutex> lock(currentOperatorMessage_mutex);
    while(!currentOperatorMessage_updated)
    {
      if(messageSenderWorker_mustContinue_flag)
        currentOperatorMessage_cond.wait(lock);
      else
        return;
    }
    rovOperator->SetDesiredState(currentOperatorMessage->GetBuffer());
    currentOperatorMessage_updated = false;
  }
}
//end messageSenderWorker vars

//rovMsgParserWorker vars
static bool hrovMsgParserWorker_mustContinue_flag;
static std::thread hrovMsgParserWorker;
void hrovMsgParserWork()
{
  while(1)
  {
    std::unique_lock<std::mutex> lock(currentHROVMessage_mutex);
    while(!currentHROVMessage_updated)
    {
      if(hrovMsgParserWorker_mustContinue_flag)
        currentHROVMessage_cond.wait(lock);
      else
        return;
    }
    auto oid = currentHROVMessage->GetExpectedOrderSeqNumber();
    auto cancelled = currentHROVMessage->LastOrderCancelledFlag ();
    auto ready = currentHROVMessage->Ready();
    auto X = currentHROVMessage->GetX();
    auto Y = currentHROVMessage->GetY();
    auto Z = currentHROVMessage->GetZ();
    auto yaw = currentHROVMessage->GetYaw();
    currentHROVMessage_updated = false;
    lock.unlock();

    merbots_whrov_msgs::position position;
    position.X = X;
    position.Y = Y;
    position.Z = Z;
    position.yaw = yaw;

    currentHROVState_pub.publish(position);
    orderActionServer->NotifyNewHROVState(ready, oid, cancelled);
  }
}

//end rovMsgParserWorker vars
void startMessageSenderWorker(ROVOperator * rovOperator)
{
  messageSenderWorker_mustContinue_flag = true;
  messageSenderWorker = std::thread(messageSenderWork, rovOperator);
}

void terminateMessageSenderWorker(void)
{
  messageSenderWorker_mustContinue_flag = false;
  currentOperatorMessage_cond.notify_one();
}

void startHROVMsgParserWorker()
{
  hrovMsgParserWorker_mustContinue_flag = true;
  hrovMsgParserWorker = std::thread(hrovMsgParserWork);
}

void terminateHROVMsgParserWorker(void)
{
  hrovMsgParserWorker_mustContinue_flag = false;
  currentHROVMessage_cond.notify_one();
}

struct ProtocolConfig
{
  unsigned int maxPacketLength;
  unsigned int txStateSize, rxStateSize;
  unsigned int width;
  unsigned int height;
  std::string ltype;

  ProtocolConfig()
  {
    txStateSize = 20;
    rxStateSize = 20;
    maxPacketLength = 200;
    width = 0;
    height = 0;
    ltype = "fulld"; //link type
  }

};

static void
usage(char *pgmname, struct debtEncParam *e, struct debtDecParam *d, ProtocolConfig & pconfig)
{
  Log->info(
        "usage: {} [options]\n"
        "Options:\n"
        "\t-v [0|1]         - Use vector instructions if possible <{}>\n"
        "\n"
        "Decoding settings: -d\n"
        "\t-b [bias]        - Bias used when decoding (default, zero, mid, exp, midexp) <{}>\n"
        "\t-a [bias weight] - Bias weight to use with bias <{}>\n"
        "\n"
        "Protocol settings:\n"
        "\t-L		    - Max. packet length <{}>\n"
        "\t-T (fulld|halfd)			- Protocol type <{}>\n",

        pgmname,
        e->vector,
        bias_name(d->bias), d->weight, pconfig.maxPacketLength, pconfig.ltype);
  exit(-1);
}

/* -d to decode or -e to encode */
/* returns -1 to graph, 0 to decode, 1 to encode */
static int
getOptions(int argc, char *argv[], struct debtEncParam *e, struct debtDecParam *d, 	ProtocolConfig & pconfig)
{
  int opt;
  int error = 0;
  int xi = 0;
  int dec = 0;
  int enc = 0;
  int graph = 0;

  while ((opt = getopt(argc, argv, "v:db:a:c:et:n:k:r:q:u:x:p:m:s:l:i:y:gf:h:I:L:T:")) != -1) {
    switch (opt) {
      case 'T':
        pconfig.ltype = std::string(optarg);
        if(pconfig.ltype != "fulld" && pconfig.ltype != "halfd")
          error += 1;
        break;
      case 'L':
        pconfig.maxPacketLength = atoi(optarg);
        break;
      case 'v':
        error += optGetIntError(optarg, &e->vector, 0);
        break;
      case 'h':
        error += optGetIntError(optarg, &e->benchmark, 0);
        break;
      case 'f':
        error += optGetIntError(optarg, &e->timestats, 0);
        break;
      case 'g':
        /* special hidden option to calculate rate x psnr curve */
        graph = 1;
        break;
      case 'd':
        ++dec;
        break;
      case 'b':
        if (!optarg) {
          d->bias = BIAS_DEF;
        } else {
          int b = bias_code(optarg);
          if (b != -1)
            d->bias = b;
          else
            ++error;
        }
        break;
      case 'a':
        error += optGetFloatError(optarg, &d->weight, 0.0);
        break;
      case 'c':
        if (optGetIntError(optarg, &xi, 0))
          ++error;
        else
          d->bitlen = xi << 3;
        break;
      case 'e':
        ++enc;
        break;
      case 't':
        if (!optarg) {
          e->transform = TRANSFORM_I_B_13x7T;
        } else {
          int t = transform_code(optarg);
          if (t != -1)
            e->transform = t;
          else
            ++error;
        }
        break;
      case 'n':
        error += optGetIntError(optarg, &e->nbands, 0);
        break;
      case 'k':
        error += optGetIntError(optarg, &e->maxk, 0);
        break;
      case 'r':
        error += optGetFloatError(optarg, &e->refmul, 0.0);
        break;
      case 'q':
        error += optGetFloatError(optarg, &e->quality, 0.0);
        break;
      case 'u':
        error += optGetIntError(optarg, &e->resolution, 0);
        break;
      case 'x':
        error += optGetIntError(optarg, &e->maxsize, 0);
        break;
      case 'p':
        error += optGetIntError(optarg, &e->pad, 0);
        break;
      case 'm':
        error += optGetIntError(optarg, &e->map, 0);
        break;
      case 's':
        error += optGetIntError(optarg, &e->esc, 0);
        break;
      case 'l':
        error += optGetIntError(optarg, &e->uroi_shift, 0);
        break;
      case 'i':
        error += optGetRect(e, optarg);
        break;
      case 'y':
        error += optGetIntError(optarg, &e->dynamic, 0);
        break;
      default: /* '?' */
        usage(argv[0], e, d, pconfig);
    }
  }

  if (error) {
    usage(pgmName(argv[0]), e, d, pconfig);
  }
  if (optind < argc) {
    Log->error("Unexpected argument(s) after options\n");
    usage(pgmName(argv[0]), e, d, pconfig);
  }
  if (dec && (enc || graph)) {
    Log->error("Decode and Encode flags cannot be used simultaneously\n");
    usage(pgmName(argv[0]), e, d, pconfig);
  }
  return graph ? -1 : !dec;
}

static void
defaultParams(struct debtEncParam *e, struct debtDecParam *d)
{
  debtEncParam_init(e);
  e->vector = 1;
  debtDecParam_init(d);
}

void HandleNewDesiredSettings(const merbots_whrov_msgs::hrov_settings::ConstPtr & msg)
{
  auto desiredSettings = wireless_ardusub::HROVSettings::BuildHROVSettings(msg);

  currentOperatorMessage_mutex.lock();
  currentOperatorMessage->SetSettings(desiredSettings);
  currentOperatorMessage_mutex.unlock();

  currentOperatorMessage_updated = true;
  currentOperatorMessage_cond.notify_one();

}

void HandleCurrentState(ROVOperator & rovOperator)
{
  currentHROVMessage_mutex.lock();
  rovOperator.GetLastConfirmedState(currentHROVMessage->GetBuffer());
  currentHROVMessage_mutex.unlock();

  currentHROVMessage_updated = true;
  currentHROVMessage_cond.notify_one();
}

void HandleNewImage(ROVOperator & rovOperator)
{
  Log->info("New image received! Decoding...");

  int fsize = rovOperator.GetLastReceivedImage(imgBuffer);
  d.bitlen = fsize << 3;
  decode(&d, imgBuffer, img);

  int ysize = img->width * img->height;
  int uvsize = img->uvwidth * img->uvheight;
  int yuvsize = ysize + (uvsize << 1);

  Log->debug("yb: {}",(unsigned long) y);
  Log->debug("Width: {}, Height: {}", img->width, img->height);
  Log->debug("y size: {}, U and V size: {}", ysize, uvsize);
  Log->debug("yuvsize: {}", yuvsize);


  if(img->width != g_iwidth || img->height != g_iheight)
  {
    g_iwidth = img->width;
    g_iheight = img->height;

    y = img->buffer;
    u = y + ysize;
    v = u + uvsize;

    /// USING OpenCV ///
    myuv.create(g_iheight + g_iheight/2, g_iwidth, CV_8UC1); //We allocate the buffer for a yuv420 size
    mrgb.create(g_iheight, g_iwidth, CV_8UC3);
    myuv.data = y; //imgBuffer will share the same buffer as myuv


  }
  /// USING OpenCV ///
  cv::cvtColor(myuv, mrgb, CV_YUV2BGR_I420);
  camera_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", mrgb).toImageMsg();
  camera_pub.publish(camera_msg);


}

void initMessages(void)
{
  currentHROVMessage = wireless_ardusub::HROVMessage::BuildHROVMessage();
  currentOperatorMessage = wireless_ardusub::OperatorMessage::BuildOperatorMessage();

  currentOperatorMessage_updated = false;
  currentHROVMessage_updated = false;
}

void initROSInterface(ros::NodeHandle & nh, int argc, char** argv,  ROVOperator & rovOperator)
{
  image_transport::ImageTransport it(nh);

  initMessages();

  orderActionServer = new MoveOrderActionServer("actions/move_order", nh);
  orderActionServer->Start();
  desiredHROVState_sub = nh.subscribe<merbots_whrov_msgs::hrov_settings>("desired_hrov_settings", 1, HandleNewDesiredSettings);//,
  // boost::bind(HandleNewDesiredSettings, _1, &rovOperator));

  camera_pub = it.advertise("camera", 1);

  currentHROVState_pub = nh.advertise<merbots_whrov_msgs::position>("current_hrov_position", 1);

  startMessageSenderWorker(&rovOperator);
  startHROVMsgParserWorker();
}

void startROSInterface()
{
  ros::spin();
}

int main(int argc, char ** argv) {

  ros::init(argc, argv, "operator_control");
  ros::NodeHandle nh("~");

  bool log2Console;
  if(!nh.getParam("log2Console",log2Console))
  {
    ROS_ERROR("Failed to get param log2Console");
    return 1;
  }
  else
  {
    ROS_INFO("Log2Console: %d", log2Console);
  }
  int remoteAddr;
  if(!nh.getParam("remoteAddr",remoteAddr))
  {
    ROS_ERROR("Failed to get param remoteAddr");
    return 1;
  }
  else
  {
    ROS_INFO("remoteAddr: %d", remoteAddr);
  }
  int localAddr;
  if(!nh.getParam("localAddr",localAddr))
  {
    ROS_ERROR("Failed to get param localAddr");
    return 1;
  }
  else
  {
    ROS_INFO("localAddr: %d", localAddr);
  }

  std::string log2File;
  bool logging2File = false;
  if(!nh.getParam("log2File",log2File))
  {
    ROS_ERROR("Failed to get param log2File");
    return 1;
  }
  else
  {
    ROS_INFO("log2File: %s", log2File.c_str());
    if(log2File == "")
    {
      ROS_INFO("No log to file");
    }
    else
    {
      ROS_INFO("Logging to file '%s'", log2File.c_str());
      logging2File = true;
    }
  }

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

  defaultParams(&e, &d);

  ProtocolConfig pconfig;
  //int act = getOptions(argc, argv, &e, &d, pconfig);

  img = imgBuffer_new();

  struct sigaction sa;
  memset(&sa, 0, sizeof(sa));
  sigemptyset(&sa.sa_mask);
  sa.sa_sigaction = segfault_sigaction;
  sa.sa_flags   = SA_SIGINFO;

  ROVOperator rovOperator(linkType);

  rovOperator.SetLogLevel(Loggable::debug);
  rovOperator.FlushLogOn(cpplogging::Loggable::info);
  rovOperator.LogToConsole(log2Console);
  if(logging2File)
  {
    rovOperator.LogToFile(log2File);
  }

  rovOperator.SetLocalAddr(localAddr);
  rovOperator.SetRemoteAddr(remoteAddr);

  Log->set_level(spdlog::level::info);
  Log->flush_on(spd::level::info);

  Log->info("remote addr: {}", remoteAddr);

  sigaction(SIGSEGV, &sa, NULL);
  imgBuffer = new uint8_t[1280*720*3*2+30]; //Unos 30 bytes mas para el header (que no sabemos, de momento, cuanto ocupa, y que depende de la resolucion de cada imagen recibida
  try
  {
    rovOperator.SetRxStateSize(wireless_ardusub::HROVMessage::MessageLength);
    rovOperator.SetTxStateSize(wireless_ardusub::OperatorMessage::MessageLength);
    rovOperator.SetImageReceivedCallback(&HandleNewImage);
    rovOperator.SetStateReceivedCallback(&HandleCurrentState);

    initROSInterface(nh, argc, argv, rovOperator);

    rovOperator.Start();

    startROSInterface();

  }catch(std::exception & e)
  {
    Log->error("Radio Exception: {}", e.what());
    exit(1);
  }

  debtDecParam_destroy(&d);
  debtEncParam_destroy(&e);
  imgBuffer_del(img);


  return 0;

}



