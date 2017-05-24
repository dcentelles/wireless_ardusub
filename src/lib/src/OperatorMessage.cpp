/*
 * HROVOrders.cpp
 *
 *  Created on: 30 nov. 2016
 *      Author: centelld
 */

#include <wireless_ardusub/OperatorMessage.h>

namespace wireless_ardusub {

const uint8_t OperatorMessage::MessageLength =
        HROVMoveOrder::OrderSize +
        HROVSettings::SettingsSize +
        1 //flags
        ;

OperatorMessage::OperatorMessage() {
    _Init();
}

OperatorMessage::OperatorMessage(uint8_t * _buf) {
    _Init();
    UpdateFromBuffer(_buf);
}

void OperatorMessage::_Init()
{
    messageInfo = buffer;
    settingsBuffer = messageInfo + 1;
    moveOrderBuffer = settingsBuffer + HROVSettings::SettingsSize;

    *messageInfo = 0;
}

OperatorMessage::~OperatorMessage() {
    // TODO Auto-generated destructor stub
}

HROVSettingsPtr OperatorMessage::GetSettingsCopy()
{
    auto settings = HROVSettings::BuildHROVSettings();
    settings->UpdateFromBuffer(settingsBuffer);
    return settings;
}

void OperatorMessage::SetSettings(HROVSettingsPtr _settings)
{
   _settings->GetBufferCopy(settingsBuffer);
}

HROVMoveOrderPtr OperatorMessage::GetMoveOrderCopy()
{
   auto moveOrder = HROVMoveOrder::BuildHROVMoveOrder();
   moveOrder->UpdateFromBuffer(moveOrderBuffer);
   return moveOrder;
}

void OperatorMessage::SetMoveOrder(HROVMoveOrderPtr _moveOrder)
{
   _moveOrder->GetBufferCopy(moveOrderBuffer);
}

OperatorMessage::OrderType OperatorMessage::GetOrderType()
{
    unsigned int type = *messageInfo & ORDER_TYPE_MASK;
    return type >= 0 && type < OtherNotImplemented ?
                (OrderType) type :
                OtherNotImplemented;
}

void OperatorMessage::SetOrderType(OrderType orderType)
{
    *messageInfo = (*messageInfo & ~ORDER_TYPE_MASK) | orderType;
}

uint8_t OperatorMessage::GetOrderSeqNumber()
{
    return (*messageInfo & ORDER_SEQ_FLAG) ? 1 : 0;
}

void OperatorMessage::SetOrderSeqNumber(uint8_t seq)
{
    *messageInfo = seq ? *messageInfo | ORDER_SEQ_FLAG :
                         *messageInfo & ~ORDER_SEQ_FLAG;
}

void OperatorMessage::CancelLastOrderFlag(bool v)
{
    *messageInfo = v ? *messageInfo | CANCEL_LAST_ORDER_FLAGH
                     : *messageInfo & ~CANCEL_LAST_ORDER_FLAGH;
}

bool OperatorMessage::CancelLastOrderFlag()
{
    return *messageInfo & CANCEL_LAST_ORDER_FLAGH;
}

void OperatorMessage::UpdateFromBuffer(uint8_t * _stateb)
{
    memcpy(buffer, _stateb, MessageLength);
}

void OperatorMessage::GetBufferCopy(uint8_t * _stateb)
{
    memcpy(_stateb, buffer, MessageLength);
}


} /* namespace merbots */
