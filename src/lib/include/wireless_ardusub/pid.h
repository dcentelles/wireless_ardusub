#ifndef _PID_H_
#define _PID_H_

namespace wireless_ardusub {
class PIDImpl;
class PID {
public:
  // Kp -  proportional gain
  // Ki -  Integral gain
  // Kd -  derivative gain
  // max - maximum value of manipulated variable
  // min - minimum value of manipulated variable
  PID(double max, double min, double Kp, double Kd, double Ki);
  void Reset();

  // dt -  loop interval time
  // Returns the manipulated variable given a setpoint and current process value
  double calculate(const double & dt, const double & setpoint, const double & pv);
  ~PID();

private:
  PIDImpl *pimpl;
};
}
#endif
