/*
 * HROVOrders.cpp
 *
 *  Created on: 30 nov. 2016
 *      Author: centelld
 */

#include <wireless_ardusub/OperatorMessageV2.h>

namespace wireless_ardusub {

const uint8_t OperatorMessageV2::MessageLength =
    TeleopOrder::Size + HROVSettingsV2::SettingsSize + 1 // flags
    ;

OperatorMessageV2::OperatorMessageV2() { _Init(); }

OperatorMessageV2::OperatorMessageV2(uint8_t *_buf) {
  _Init();
  UpdateFromBuffer(_buf);
}

void OperatorMessageV2::_Init() {
  messageInfo = buffer;
  settingsBuffer = messageInfo + 1;
  orderBuffer = settingsBuffer + HROVSettingsV2::SettingsSize;

  *messageInfo = 0;
}

OperatorMessageV2::~OperatorMessageV2() {
  // TODO Auto-generated destructor stub
}

uint8_t OperatorMessageV2::GetHoldChannelDuration() { return *orderBuffer; }

void OperatorMessageV2::SetHoldChannelDuration(uint8_t v) { *orderBuffer = v; }

HROVSettingsV2Ptr OperatorMessageV2::GetSettingsCopy() {
  auto settings = HROVSettingsV2::Build();
  settings->UpdateFromBuffer(settingsBuffer);
  return settings;
}

void OperatorMessageV2::SetSettings(HROVSettingsV2Ptr _settings) {
  _settings->GetBufferCopy(settingsBuffer);
}

TeleopOrderPtr OperatorMessageV2::GetMoveOrderCopy() {
  auto moveOrder = TeleopOrder::Build();
  moveOrder->BuildFromBuffer(orderBuffer);
  return moveOrder;
}

void OperatorMessageV2::SetMoveOrder(TeleopOrderPtr _moveOrder) {
  SetOrderType(OrderType::Move);
  memcpy(orderBuffer, _moveOrder->GetBuffer(), TeleopOrder::Size);
}

OperatorMessageV2::OrderType OperatorMessageV2::GetOrderType() {
  unsigned int type = *messageInfo & ORDER_TYPE_MASK;
  return type >= 0 && type < OtherNotImplemented ? (OrderType)type
                                                 : OtherNotImplemented;
}

void OperatorMessageV2::SetOrderType(OrderType orderType) {
  *messageInfo = (*messageInfo & ~ORDER_TYPE_MASK) | orderType;
}

uint8_t OperatorMessageV2::GetOrderSeqNumber() {
  return (*messageInfo & ORDER_SEQ_FLAG) ? 1 : 0;
}

void OperatorMessageV2::SetOrderSeqNumber(uint8_t seq) {
  *messageInfo =
      seq ? *messageInfo | ORDER_SEQ_FLAG : *messageInfo & ~ORDER_SEQ_FLAG;
}

void OperatorMessageV2::CancelLastOrderFlag(bool v) {
  *messageInfo = v ? *messageInfo | CANCEL_LAST_ORDER_FLAGH
                   : *messageInfo & ~CANCEL_LAST_ORDER_FLAGH;
}

bool OperatorMessageV2::CancelLastOrderFlag() {
  return *messageInfo & CANCEL_LAST_ORDER_FLAGH;
}

void OperatorMessageV2::UpdateFromBuffer(uint8_t *_stateb) {
  memcpy(buffer, _stateb, MessageLength);
}

void OperatorMessageV2::GetBufferCopy(uint8_t *_stateb) {
  memcpy(_stateb, buffer, MessageLength);
}

} /* namespace merbots */
