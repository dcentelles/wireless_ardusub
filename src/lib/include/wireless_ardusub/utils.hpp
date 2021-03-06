#ifndef INCLUDE_WIRELESS_ARDUSUB_UTILS_HPP
#define INCLUDE_WIRELESS_ARDUSUB_UTILS_HPP
#include <math.h>

namespace telerobotics {
namespace utils {

static constexpr double range_yaw = M_PI * 2;

static int GetDiscreteYaw(double rad) {
  int degrees = std::round(rad * 180.0 / M_PI); // conversion to degrees
  if (degrees < 0)
    degrees += 360; // convert negative to positive angles

  return degrees;
}
static double GetContinuousYaw(int deg) { return deg / 180. * M_PI; }
}
}
#endif // UTILS_HPP
