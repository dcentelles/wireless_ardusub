#include <condition_variable>
#include <cpplogging/cpplogging.h>
#include <cpputils/SignalManager.h>
#include <cstdio>
#include <geometry_msgs/Pose.h>
#include <merbots_whrov_msgs/OrderActionGoal.h>
#include <merbots_whrov_msgs/OrderGoal.h>
#include <mutex>
#include <ros/ros.h>
#include <ros/subscriber.h>
#include <stdio.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

using namespace cpplogging;
using namespace cpputils;

LoggerPtr Log;
tf::TransformListener *targetModemListener;

tf::StampedTransform rov_to_s100;
bool baseTfSet = false;

void handleNewOrder(merbots_whrov_msgs::OrderActionGoalConstPtr goal) {
  if (goal->goal.type == 5) {
    double x = goal->goal.x, y = goal->goal.y, z = goal->goal.depth;

    if (baseTfSet) // si tenim la tf entre base_link i s100
    {
      tf::Transform nedorig_to_target_rov;
      nedorig_to_target_rov.setOrigin(tf::Vector3(x, y, z));
      tf::Quaternion rot;
      rot.setRPY(0, 0, 0);
      nedorig_to_target_rov.setRotation(rot);

      tf::Transform nedorig_to_targetS100 = nedorig_to_target_rov * rov_to_s100;

      try {
        tf::StampedTransform nedorig_to_buoyS100;
        targetModemListener->waitForTransform("local_origin_ned", "buoy_s100",
                                              ros::Time(0), ros::Duration(2));
        targetModemListener->lookupTransform("local_origin_ned", "buoy_s100",
                                             ros::Time(0), nedorig_to_buoyS100);
        auto range = nedorig_to_buoyS100.getOrigin().distance(
            nedorig_to_targetS100.getOrigin());

        Log->Info("TM RNG: {}", range);

      } catch (tf::TransformException &e) {
        Log->Critical("Not able to lookup transform: %s", e.what());
      };
    }
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "goto_range");
  ros::NodeHandle nh("/merbots/wrov/operator/");
  targetModemListener = new tf::TransformListener();

  Log = CreateLogger("goto_range");
  Log->LogToConsole(true);
  Log->LogToFile("goto_range");
  Log->SetAsyncMode();
  Log->FlushLogOn(LogLevel::info);
  Log->SetLogFormatter(
      std::make_shared<spdlog::pattern_formatter>("[%T.%F] %v"));

  std::thread currentRangeWorker([&]() {
    tf::TransformListener currentRangeListener;
    tf::StampedTransform buoy_s100_M_bluerov2_s100;
    while (1) {
      try {
        currentRangeListener.waitForTransform("buoy_s100", "bluerov2_s100",
                                              ros::Time(0), ros::Duration(2));
        currentRangeListener.lookupTransform("buoy_s100", "bluerov2_s100",
                                             ros::Time(0),
                                             buoy_s100_M_bluerov2_s100);
        auto range = buoy_s100_M_bluerov2_s100.getOrigin().distance(
            tf::Vector3(0, 0, 0));
        Log->Info("CM RNG: {}", range);

      } catch (tf::TransformException &e) {
        Log->Critical("Not able to lookup transform: %s", e.what());
      };
      std::this_thread::sleep_for(std::chrono::milliseconds(150));
    }
  });

  std::thread targetRangeWorker([&]() {
    tf::TransformListener targetRangeListener;
    tf::StampedTransform buoy_s100_M_target_s100;
    while (1) {
      try {
        targetRangeListener.waitForTransform("buoy_s100", "target_s100",
                                             ros::Time(0), ros::Duration(2));
        targetRangeListener.lookupTransform(
            "buoy_s100", "target_s100", ros::Time(0), buoy_s100_M_target_s100);
        auto range =
            buoy_s100_M_target_s100.getOrigin().distance(tf::Vector3(0, 0, 0));
        Log->Info("TG RNG: {}", range);

      } catch (tf::TransformException &e) {
        Log->Critical("Not able to lookup transform: %s", e.what());
      };
      std::this_thread::sleep_for(std::chrono::milliseconds(150));
    }
  });

  std::thread baseToS100Getter([&]() {
    tf::TransformListener currentRangeListener;

    while (1) {
      try {
        currentRangeListener.waitForTransform("debug", "bluerov2_s100",
                                              ros::Time(0), ros::Duration(2));
        currentRangeListener.lookupTransform("debug", "bluerov2_s100",
                                             ros::Time(0), rov_to_s100);
        baseTfSet = true;
        return;

      } catch (tf::TransformException &e) {
        Log->Critical("Not able to lookup transform: %s", e.what());
      };
      std::this_thread::sleep_for(std::chrono::milliseconds(150));
    }
  });

  ros::Subscriber encodedImage_sub =
      nh.subscribe<merbots_whrov_msgs::OrderActionGoal>(
          "/merbots/wrov/operator/order/goal", 1,
          boost::bind(handleNewOrder, _1));

  SignalManager::SetLastCallback(SIGINT, [&](int sig) {
    printf("Received %d signal.\nFlushing log messages...", sig);
    fflush(stdout);
    Log->FlushLog();
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    printf("Log messages flushed.\n");
    exit(0);
  });
  ros::spin();

  return 0;
}
