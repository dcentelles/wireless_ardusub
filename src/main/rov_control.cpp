#include <iostream>

//ROS
#include <ros/ros.h>
//end ROS

#include <telerobotics/ROVCamera.h>

struct Params
{
  std::string cameraTopic, masterUri;
  dcauv::LinkType linkType;
  int localAddr, remoteAddr;
  bool log2Console;
};

static Params params;

void GetParams(ros::NodeHandle & nh)
{

}

int main(int argc, char** argv)
{
  std::cout << "Hello World!" << std::endl;

  //// GET PARAMS
  ros::init(argc, argv, "rov_control");
  ros::NodeHandle nh("~");

  return 0;
}
