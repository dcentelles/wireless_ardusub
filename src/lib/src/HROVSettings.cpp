/*
 * HROVState.cpp
 *
 *  Created on: 30 nov. 2016
 *      Author: centelld
 */

#include <wireless_ardusub//HROVSettings.h>
#include <dccomms/Utils.h>

namespace merbots {

HROVSettings::HROVSettings() {
	_Init();
}

HROVSettings::HROVSettings(const merbots_whrov_msgs::hrov_settings::ConstPtr & cstate) {
	_Init();
    SetFromROSMsg(cstate);
}

HROVSettings::~HROVSettings() {
	// TODO Auto-generated destructor stub
}

void HROVSettings::_Init()
{
    bigEndian = dccomms::Utils::IsBigEndian();

	//IMAGE
    roi = (uint8_t*) buffer;

    roi_shift  = (uint8_t *) (roi + 5); //+5

    //roi shift ocupa 3 bits
    //la resolucion se codifica en 5 bits
    img_resolution = roi_shift;

    img_size = (uint16_t*) (img_resolution + 1);  //+2

	//PROTOCOL
    max_packet_length = img_size + 1;      //+2
	_SetROIConf(false);
}
void HROVSettings::UpdateFromBuffer(uint8_t * _stateb)
{
    memcpy(buffer, _stateb, SettingsSize);
}

void HROVSettings::GetBufferCopy(uint8_t * _stateb)
{
    memcpy(_stateb, buffer, SettingsSize);
}

void HROVSettings::SetFromROSMsg(const merbots_whrov_msgs::hrov_settings::ConstPtr & cstate)
{
    _SetROIConf(
			cstate->image_config.roi_x0,
			cstate->image_config.roi_y0,
			cstate->image_config.roi_x1,
			cstate->image_config.roi_y1,
			cstate->image_config.roi_shift
			);

	_SetImgSize(cstate->image_config.size);
	_SetImgResolution(cstate->image_config.resolution);
	_SetMaxPacketLength(cstate->protocol_config.max_packet_length);
}

merbots_whrov_msgs::hrov_settings::Ptr HROVSettings::GetROSMsg()
{
    merbots_whrov_msgs::hrov_settings::Ptr cstate(new merbots_whrov_msgs::hrov_settings());
    cstate.reset(new merbots_whrov_msgs::hrov_settings());
    cstate->image_config.size   = _GetImgSize();
	cstate->image_config.roi_x0 	= _GetROIX0();
	cstate->image_config.roi_y0		= _GetROIY0();
	cstate->image_config.roi_x1		= _GetROIX1();
	cstate->image_config.roi_y1		= _GetROIY1();
	cstate->image_config.roi_shift  = _GetROIShift();
	cstate->image_config.resolution = _GetImgResolution();

	cstate->protocol_config.max_packet_length = _GetMaxPacketLength();
    return cstate;
}

void HROVSettings::_SetROIConf(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint8_t shift)
{
    *roi_shift &= 0x1f;
    *roi_shift |= (shift & 0x7) << 5;
    roi[0] = 0;
    roi[1] = 0;
    roi[2] = 0;
    roi[3] = 0;
    roi[4] = 0;

    uint8_t * ptr = roi;

    //x0
    *ptr |= (x0 & 0x03fc) >> 2; // 8
    ptr++;
    *ptr |= (x0 & 0x3) << 6; //+2

    //y0
    *ptr |= (y0 & 0x03f0) >> 4; // 6
    ptr++;
    *ptr |= (y0 & 0xf) << 4; //+4


    //x1
    *ptr |= (x1 & 0x03c0) >> 6; // 4
    ptr++;
    *ptr |= (x1 & 0x3f) << 2; //+6

    //y1
    *ptr |= (y1 & 0x0300) >> 8; // 2
    ptr++;
    *ptr |= y1 & 0xff; //+8
}

void HROVSettings::_GetROIConf(int16_t & x0, int16_t & y0, int16_t & x1, int16_t & y1, uint8_t & shift)
{
    shift	= _GetROIShift ();
    x0      = _GetROIX0();
    y0      = _GetROIY0();
    x1      = _GetROIX1();
    y1      = _GetROIY1();

}

uint16_t HROVSettings::_GetROIX0()
{
    uint8_t * ptr = roi;
    uint16_t res;

    res = (*ptr & 0xff) << 2; //8
    res |= (*(ptr+1) & 0xc0) >> 6; //+2

    return res;
}

uint16_t HROVSettings::_GetROIY0()
{	
    uint8_t * ptr = roi+1;
    uint16_t res;

    res = (*ptr & 0x3f) << 4; //6
    res |= (*(ptr+1) & 0xf0) >> 4; //+4

    return res;
}

uint16_t HROVSettings::_GetROIX1()
{
    uint8_t * ptr = roi+2;
    uint16_t res;

    res = (*ptr & 0x0f) << 6; //4
    res |= (*(ptr+1) & 0xfc) >> 2; //+6

    return res;
}

uint16_t HROVSettings::_GetROIY1()
{
    uint8_t * ptr = roi+3;
    uint16_t res;

    res = (*ptr & 0x03) << 8; //2
    res |= *(ptr+1); //+8

    return res;
}

uint8_t HROVSettings::_GetROIShift()
{
    return *roi_shift >> 5;
}

void HROVSettings::_SetImgSize(uint16_t size)
{
    if(bigEndian)
    {
        *img_size = size;
    }
    else
    {
        dccomms::Utils::IntSwitchEndian(img_size, size);
    }
}

uint16_t HROVSettings::_GetImgSize()
{
    uint16_t res;
    if(bigEndian)
    {
        res = *img_size;
    }
    else
    {
        dccomms::Utils::IntSwitchEndian(&res, *img_size);
    }
    return res;
}

void HROVSettings::_SetImgResolution(uint16_t resolution)
{
    *img_resolution |= resolution & 0x1f;
}

uint8_t HROVSettings::_GetImgResolution()
{
    return *img_resolution & 0x1f;
}

void HROVSettings::_SetMaxPacketLength(uint16_t mpl)
{
    if(bigEndian)
    {
        *max_packet_length = mpl;
    }
    else
    {
        dccomms::Utils::IntSwitchEndian(max_packet_length, mpl);
    }
}

uint16_t HROVSettings::_GetMaxPacketLength()
{
    uint16_t res;
    if(bigEndian)
    {
        res = *max_packet_length;
    }
    else
    {
        dccomms::Utils::IntSwitchEndian(&res, *max_packet_length);
    }
    return res;
}

} /* namespace merbots */
