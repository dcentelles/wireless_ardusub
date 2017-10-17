#include <cstring>
#include <wireless_ardusub/TeleopOrder.h>

namespace wireless_ardusub {

TeleopOrder::TeleopOrder() { _Init(); }

TeleopOrder::TeleopOrder(void *buffer) {
  memcpy(_buffer, buffer, Size);
  _Init();
}

TeleopOrder::TeleopOrder(const TeleopOrder &order) {
  memcpy(_buffer, order._buffer, Size);
  _Init();
}

void TeleopOrder::_Init() {
  _x = (int8_t *)_buffer;
  _y = _x + 1;
  _z = _y + 1;
  _r = _z + 1;
  _flags0 = (uint8_t *)_r + 1;
}

int8_t TeleopOrder::GetX() { return *_x; }
int8_t TeleopOrder::GetY() { return *_y; }
int8_t TeleopOrder::GetZ() { return *_z; }
int8_t TeleopOrder::GetR() { return *_r; }

void TeleopOrder::SetX(int8_t x) { *_x = x; }
void TeleopOrder::SetY(int8_t y) { *_y = y; }
void TeleopOrder::SetZ(int8_t z) { *_z = z; }
void TeleopOrder::SetR(int8_t r) { *_r = r; }

void TeleopOrder::Arm(bool arm) {
  *_flags0 &= ~ARM_FLAG;
  *_flags0 |= arm ? ARM_FLAG : 0;
}
bool TeleopOrder::Arm() { return *_flags0 & ARM_FLAG; }

void TeleopOrder::DisArm(bool disarm) {
  *_flags0 &= ~DISARM_FLAG;
  *_flags0 |= disarm ? DISARM_FLAG : 0;
}
bool TeleopOrder::DisArm() { return *_flags0 & DISARM_FLAG; }

FLY_MODE TeleopOrder::GetFlyMode() {
  FLY_MODE mode = (FLY_MODE)(*_flags0 & FLY_MODE_MASK);
  return mode;
}

void TeleopOrder::SetFlyMode(FLY_MODE mode) {
  *_flags0 &= ~FLY_MODE_MASK;
  *_flags0 |= mode;
}

void TeleopOrder::BuildFromBuffer(void *buffer) {
  memcpy(_buffer, buffer, Size);
  _Init();
}
}
