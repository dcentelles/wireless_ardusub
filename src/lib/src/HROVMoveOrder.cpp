/*
 * HROVOrders.cpp
 *
 *  Created on: 30 nov. 2016
 *      Author: centelld
 */

#include <wireless_ardusub//HROVMoveOrder.h>
#include <wireless_ardusub/utils.hpp>
#include <dccomms/Utils.h>

namespace wireless_ardusub {

HROVMoveOrder::HROVMoveOrder() {
    _Init();
}

HROVMoveOrder::~HROVMoveOrder() {
    // TODO Auto-generated destructor stub
}

void HROVMoveOrder::_Init()
{
    bigEndian = dccomms::Utils::IsBigEndian();

    flags = buffer;
    pose = flags + 1;

    *flags = 0;

    SetFrame(ROV_FRAME);
}

void HROVMoveOrder::UpdateFromBuffer(uint8_t * _stateb)
{
    memcpy(buffer, _stateb, OrderSize);
}

void HROVMoveOrder::GetBufferCopy(uint8_t * _stateb)
{
    memcpy(_stateb, buffer, OrderSize);
}

void HROVMoveOrder::SetYaw(int _yaw)
{
    uint8_t * ptr = pose;

    //encoded with 9 bits (max value: 360)
    pose[0] = 0;
    pose[1] &= 0x7f;
    *ptr |= (_yaw & 0x01fe) >> 1; // 8
    *(ptr+1) |= (_yaw & 0x1) << 7; // +1
}

int16_t HROVMoveOrder::GetYaw()
{
    uint8_t * ptr = pose;
    int16_t res;

    res = *ptr << 1; // 8
    ptr++;
    res |= (*ptr & 0x80) >> 7;

    if(res & 0x0100)
    {
        res |= 0xfe00;
    }

    return res;
}
void HROVMoveOrder::SetX(int _X)
{
    //encoded with 13 bits (up to 819.2 m)
    uint8_t * ptr = pose + 1;
    int16_t X = (int16_t)std::round(_X);
    *ptr &= 0x80;
    *ptr |= (X & 0x1fc0) >> 6; // 7
    ptr++;
    *ptr &= 0x03;
    *ptr = (X & 0x3f) << 2; // +6
}


int16_t HROVMoveOrder::GetX()
{
    uint8_t * ptr = pose + 1;
    int16_t res;
    res = (*ptr & 0x7f) << 6;
    ptr++;
    res |= *ptr >> 2;

    if(res & 0x1000)
    {
        res |= 0xe000;
    }

    return res;
}

void HROVMoveOrder::SetY(int _Y)
{
    uint8_t * ptr = pose + 2;
    int16_t Y = (int16_t)std::round(_Y);
    *ptr &= 0xfc;
    *ptr |= (Y & 0x1800) >> 11; //2
    ptr++;
    *ptr = (Y & 0x7f8) >> 3; //+8
    ptr++;
    *ptr &= 0x1f;
    *ptr = (Y & 0x7) << 5; //+3
}


int16_t HROVMoveOrder::GetY()
{
    uint8_t * ptr = pose + 2;
    int16_t res;
    res = (*ptr & 0x03) << 11;
    ptr++;
    res |= *ptr << 3;
    ptr++;
    res |= *ptr >> 5;

    if(res & 0x1000)
    {
        res |= 0xe000;
    }

    return res;
}


void HROVMoveOrder::SetZ(int _Z)
{
    uint8_t * ptr = pose + 4;
    int16_t Z = (int16_t)std::round(_Z);
    *ptr &= 0xe0;
    *ptr |= (Z & 0x1f00) >> 8; //5
    ptr++;
    *ptr = (Z & 0xff); //+8
}

int16_t HROVMoveOrder::GetZ()
{
    uint8_t * ptr = pose + 4;
    int16_t res;
    res = (*ptr & 0x1f) << 8;
    ptr++;
    res |= *ptr ;

    if(res & 0x1000)
    {
        res |= 0xe000;
    }

    return res;
}

void HROVMoveOrder::Relative(bool v)
{
    *flags = v ? *flags | RELATIVE_MOV : *flags & ~RELATIVE_MOV;
}

bool HROVMoveOrder::Relative()
{
    return *flags & RELATIVE_MOV;
}

void HROVMoveOrder::SetFrame(int frameId)
{
    *flags = frameId ? *flags | FRAME : *flags & ~FRAME;
}

int HROVMoveOrder::GetFrame()
{
    auto frameId = *flags & FRAME ? 1 : 0;
    return frameId;
}


} /* namespace merbots */
