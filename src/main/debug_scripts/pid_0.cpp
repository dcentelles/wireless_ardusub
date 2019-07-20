#include <stdio.h>
#include <wireless_ardusub/pid.h>

using namespace wireless_ardusub;

int main(int argc, char **argv) {
  PID pid = PID(180, -180, 0.1, 0.01, 0);

  double val = 180;
  for (int i = 0; i < 100; i++) {
    double inc = pid.calculate(0.1, 0, val);
    printf("val:% 7.3f inc:% 7.3f\n", val, inc);
    val += inc;
  }

  return 0;
}
