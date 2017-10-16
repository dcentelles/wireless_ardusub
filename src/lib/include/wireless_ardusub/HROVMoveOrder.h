/*
 * HROVOrders.h
 *
 *  Created on: 30 nov. 2016
 *      Author: centelld
 */

#ifndef INCLUDE_WIRELESS_ARDUSUB_HROVMOVEORDER_H_
#define INCLUDE_WIRELESS_ARDUSUB_HROVMOVEORDER_H_

#include <merbots_whrov_msgs/movement.h>
#include <wireless_ardusub/Constants.h>

namespace wireless_ardusub {

class HROVMoveOrder;

typedef std::shared_ptr<HROVMoveOrder> HROVMoveOrderPtr;

class HROVMoveOrder {
public:
  const static uint8_t OrderSize = 7;
  HROVMoveOrder();
  virtual ~HROVMoveOrder();

  static HROVMoveOrderPtr BuildHROVMoveOrder() {
    return HROVMoveOrderPtr(new HROVMoveOrder());
  }

  void GetROSMsg(merbots_whrov_msgs::movement::Ptr &);

  void UpdateFromBuffer(uint8_t *);
  void GetBufferCopy(uint8_t *);

  uint8_t *GetBuffer() { return buffer; }

  void SetYaw(int);
  int16_t GetYaw();
  void SetZ(int);
  int16_t GetZ();
  void SetX(int);
  int16_t GetX();
  void SetY(int);
  int16_t GetY();

  void Relative(bool);
  void SetFrame(int);
  int GetFrame();
  bool Relative();

  enum Frame { ROV_FRAME = 0, WORLD_FRAME = 1 };

private:
  void _Init();
  uint8_t buffer[MAX_HROVSTATE_LENGHT];

  const static uint8_t
      RELATIVE_MOV = 0x80,
      FRAME = 0x40 // ROV_FRAME: ROV frame, WORLD_FRAME: World frame
      ;
  uint8_t *flags, *pose;

  bool bigEndian;
};

} /* namespace merbots */

#endif /* MERBOTS_LIB_INCLUDE_MERBOTS_HROVMOVEORDER_H_ */
