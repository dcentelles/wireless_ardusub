#ifndef TELEOPORDER_H
#define TELEOPORDER_H

#include <cstdint>
#include <memory>

namespace wireless_ardusub {
class TeleopOrder;
typedef std::shared_ptr<TeleopOrder> TeleopOrderPtr;
enum FLY_MODE { STABILIZE = 1, DEPTH_HOLD = 2, MANUAL = 3 };
class TeleopOrder {
public:
  static const int Size = 5;
  static TeleopOrderPtr Build() { return TeleopOrderPtr(new TeleopOrder()); }

  static TeleopOrderPtr Build(void *buffer) {
    return TeleopOrderPtr(new TeleopOrder(buffer));
  }

  static TeleopOrderPtr Build(const TeleopOrder &order) {
    return TeleopOrderPtr(new TeleopOrder(order));
  }

  void BuildFromBuffer(void *);
  int8_t GetX();
  int8_t GetY();
  int8_t GetZ();
  int8_t GetR();

  void SetX(int8_t);
  void SetY(int8_t);
  void SetZ(int8_t);
  void SetR(int8_t);

  void Arm(bool);
  bool Arm();
  void DisArm(bool);
  bool DisArm();

  void *GetBuffer() { return _buffer; }

  FLY_MODE GetFlyMode();
  void SetFlyMode(FLY_MODE);

private:
  TeleopOrder();
  TeleopOrder(void *);
  TeleopOrder(const TeleopOrder &);

  uint8_t _buffer[Size];
  int8_t *_x, *_y, *_z, *_r;
  uint8_t *_flags0;

  const static int FLY_MODE_MASK = 0x3;
  const static uint8_t ARM_FLAG = 0x80, DISARM_FLAG = 0x40;
  void _Init();
};
}

#endif // TELEOPORDER_H
