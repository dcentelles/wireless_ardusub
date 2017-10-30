/*
 * HROVOrders.cpp
 *
 *  Created on: 30 nov. 2016
 *      Author: centelld
 */

#include <dccomms/Utils.h>
#include <wireless_ardusub/HROVMessageV2.h>
#include <wireless_ardusub/utils.hpp>

namespace wireless_ardusub {

const uint8_t HROVMessageV2::MessageLength = 1 + // flags
                                             6   // pose size
    ;

HROVMessageV2::HROVMessageV2() { _Init(); }

HROVMessageV2::HROVMessageV2(uint8_t *_buffer) {
  _Init();
  UpdateFromBuffer(_buffer);
}

HROVMessageV2::~HROVMessageV2() {
  // TODO Auto-generated destructor stub
}

void HROVMessageV2::_Init() {
  bigEndian = dccomms::Utils::IsBigEndian();

  flags = buffer;
  pose = flags + 1;

  *flags = 0;
}

void HROVMessageV2::UpdateFromBuffer(uint8_t *_stateb) {
  memcpy(buffer, _stateb, MessageLength);
}

void HROVMessageV2::GetBufferCopy(uint8_t *_stateb) {
  memcpy(_stateb, buffer, MessageLength);
}

bool HROVMessageV2::LastOrderCancelledFlag() {
  return *flags & LAST_ORDER_CANCELLED_FLAG;
}

void HROVMessageV2::KeepingHeadingFlag(bool v) {
  *flags = v ? *flags | KEEPING_HEADING_FLAG : *flags & ~KEEPING_HEADING_FLAG;
}
bool HROVMessageV2::KeepingHeadingFlag() {
  return *flags & KEEPING_HEADING_FLAG;
}

void HROVMessageV2::LastOrderCancelledFlag(bool v) {
  *flags = v ? *flags | LAST_ORDER_CANCELLED_FLAG
             : *flags & ~LAST_ORDER_CANCELLED_FLAG;
}

bool HROVMessageV2::Ready() { return *flags & READY_FLAG; }

void HROVMessageV2::Ready(bool v) {
  *flags = v ? *flags | READY_FLAG : *flags & ~READY_FLAG;
}
uint8_t HROVMessageV2::GetExpectedOrderSeqNumber() {
  return *flags & NEXT_ORDER_SEQ_FLAG ? 1 : 0;
}

uint8_t HROVMessageV2::GetNextOrderSeqNumber(uint8_t seq) {
  return seq ? 0 : 1;
}

void HROVMessageV2::IncExpectedOrderSeqNumber() {
  *flags = *flags & NEXT_ORDER_SEQ_FLAG ? *flags & ~NEXT_ORDER_SEQ_FLAG
                                        : *flags | NEXT_ORDER_SEQ_FLAG;
}

void HROVMessageV2::SetHeading(uint16_t _yaw) {
  uint8_t *ptr = pose;

  // encoded with 9 bits (max value: 360)
  pose[0] = 0;
  pose[1] &= 0x7f;
  *ptr |= (_yaw & 0x01fe) >> 1;    // 8
  *(ptr + 1) |= (_yaw & 0x1) << 7; // +1
}

uint16_t HROVMessageV2::GetHeading() {
  uint8_t *ptr = pose;
  uint16_t res;

  res = *ptr << 1; // 8
  ptr++;
  res |= (*ptr & 0x80) >> 7;
  return res;
}

void HROVMessageV2::SetRoll(double _X) {
  // encoded with 13 bits (up to 819.2 m)
  uint8_t *ptr = pose + 1;
  int16_t X = (int16_t)std::round(_X);
  *ptr &= 0x80;
  *ptr |= (X & 0x1fc0) >> 6; // 7
  ptr++;
  *ptr &= 0x03;
  *ptr |= (X & 0x3f) << 2; // +6
}

int16_t HROVMessageV2::GetRoll() {
  uint8_t *ptr = pose + 1;
  int16_t res;
  res = (*ptr & 0x7f) << 6;
  ptr++;
  res |= *ptr >> 2;

  if (res & 0x1000) {
    res |= 0xfe00;
  }

  return res;
}

void HROVMessageV2::SetPitch(double _Y) {
  uint8_t *ptr = pose + 2;
  int16_t Y = (int16_t)std::round(_Y);
  *ptr &= 0xfc;
  *ptr |= (Y & 0x1800) >> 11; // 2
  ptr++;
  *ptr = (Y & 0x7f8) >> 3; //+8
  ptr++;
  *ptr &= 0x1f;
  *ptr |= (Y & 0x7) << 5; //+3
}

int16_t HROVMessageV2::GetPitch() {
  uint8_t *ptr = pose + 2;
  int16_t res;
  res = (*ptr & 0x03) << 11;
  ptr++;
  res |= *ptr << 3;
  ptr++;
  res |= *ptr >> 5;

  if (res & 0x1000) {
    res |= 0xfe00;
  }

  return res;
}

void HROVMessageV2::SetZ(double _Z) {
  uint8_t *ptr = pose + 4;
  int16_t Z = (int16_t)std::round(_Z);
  *ptr &= 0xe0;
  *ptr |= (Z & 0x1f00) >> 8; // 5
  ptr++;
  *ptr = (Z & 0xff); //+8
}

int16_t HROVMessageV2::GetAltitude() {
  uint8_t *ptr = pose + 4;
  int16_t res;
  res = (*ptr & 0x1f) << 8;
  ptr++;
  res |= *ptr;

  if (res & 0x1000) {
    res |= 0xfe00;
  }

  return res;
}

} /* namespace merbots */
