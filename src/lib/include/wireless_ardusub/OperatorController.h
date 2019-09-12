#ifndef INCLUDE_WIRELESS_ARDUSUB_OPERATOR_CONTROLLER_H_
#define INCLUDE_WIRELESS_ARDUSUB_OPERATOR_CONTROLLER_H_

#include <control/pid.h>
#include <cpputils/Timer.h>
#include <mavlink_cpp/GCS.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

namespace wireless_ardusub {

using namespace cpputils;
using namespace mavlink_cpp;
using namespace control;

class OperatorController : public Logger {
public:
  struct Params {
    bool sitl;
  };
  struct ControlState {
    double x, y, z, r;
    FLY_MODE_R mode;
    bool arm;
  };

  OperatorController(const Params &params);
  void Start();
  ControlState ControlState;
  std::shared_ptr<GCS> Control;

private:
  tf::TransformListener listener;
  geometry_msgs::TransformStamped static_transformStamped;
  std::vector<geometry_msgs::TransformStamped> static_transforms;

  tf::StampedTransform nedMerov, nedMtarget, cameraMrov;
  tf::Transform rovMtarget;
  tf::Transform rovMned;
  ros::Publisher debugPublisher0;

  cpputils::Timer timer;

  // PID
  double vmax = 1000, vmin = -1000;
  PID yawPID, xPID, yPID, zPID;
  double yoffset, xoffset, roffset, zoffset, deadband, baseZ, zoffsetPos;
  // End PID

  Params _params;
  std::string _ref_tf, _robot_tf, _desired_robot_tf;

  // FUNCTIONS
  double keepHeadingIteration(const double &dt, double diff);
  double ArduSubXYR(double per);
  double ArduSubZ(double per);
  void Saturate(const double &max, const double &x, const double &y,
                const double &z, double &vx, double &vy, double &vz);
  void GetLinearXVel(const double &dt, const double &diff, double &v);
  void GetLinearYVel(const double &dt, const double &diff, double &v);
  void GetLinearZVel(const double &dt, const double &diff, double &v);
  void Loop();
  void ResetPID();
  void SetReferenceTfName(const std::string &ref);
  void SetRobotTfName(const std::string &ref);
  void SetDesiredPosTfName(const std::string &ref);

  std::thread _mainLoop;
};

} // namespace wireless_ardusub

#endif /* INCLUDE_WIRELESS_ARDUSUB_CONSTANTS_H_ */
