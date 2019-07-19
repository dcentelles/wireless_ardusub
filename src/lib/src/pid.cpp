#ifndef _PID_SOURCE_
#define _PID_SOURCE_

#include <cmath>
#include <exception>
#include <iostream>
#include <wireless_ardusub/pid.h>

namespace wireless_ardusub {
using namespace std;

class PIDImpl {
public:
  PIDImpl(double dt, double max, double min, double Kp, double Kd, double Ki);
  ~PIDImpl();
  double calculate(double setpoint, double pv);
  void Reset();

private:
  double _dt;
  double _max;
  double _min;
  double _Kp;
  double _Kd;
  double _Ki;
  double _pre_error;
  double _integral;
};

PID::PID(double dt, double max, double min, double Kp, double Kd, double Ki) {
  pimpl = new PIDImpl(dt, max, min, Kp, Kd, Ki);
}
double PID::calculate(double setpoint, double pv) {
  return pimpl->calculate(setpoint, pv);
}

void PID::Reset() { pimpl->Reset(); }
void PIDImpl::Reset() {
  _integral = 0;
  _pre_error = 0;
}

PID::~PID() { delete pimpl; }

/**
 * Implementation
 */
PIDImpl::PIDImpl(double dt, double max, double min, double Kp, double Kd,
                 double Ki)
    : _dt(dt), _max(max), _min(min), _Kp(Kp), _Kd(Kd), _Ki(Ki), _pre_error(0),
      _integral(0) {}

double PIDImpl::calculate(double setpoint, double pv) {

  // Calculate error
  double error = setpoint - pv;

  // Proportional term
  double Pout = _Kp * error;

  // Integral term
  _integral += error * _dt;
  double Iout = _Ki * _integral;

  // Derivative term
  if (_dt == 0.0)
    std::cerr << "Impossible to create a PID regulator with a null loop "
                 "interval time."
              << std::endl;
  double derivative = (error - _pre_error) / _dt;
  double Dout = _Kd * derivative;

  // Calculate total output
  double output = Pout + Iout + Dout;

  // Restrict to max/min
  if (output > _max)
    output = _max;
  else if (output < _min)
    output = _min;

  // Save error to previous error
  _pre_error = error;
  std::cout << "Pout: " << Pout << "  Iout: " << Iout <<  "  Dout: " << Dout << " OUTPUT: " << output << std::endl;
  std::cout << std::flush;

  return output;
}

PIDImpl::~PIDImpl() {}
} // namespace wireless_ardusub
#endif
