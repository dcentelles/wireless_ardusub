/*
 * HROVState.h
 *
 *  Created on: 30 nov. 2016
 *      Author: centelld
 */

#ifndef INCLUDE_WIRELESS_ARDUSUB_HROVSETTINGS_H_
#define INCLUDE_WIRELESS_ARDUSUB_HROVSETTINGS_H_

#include <merbots_whrov_msgs/hrov_settings.h>
#include <wireless_ardusub/Constants.h>

namespace wireless_ardusub {

class HROVSettings;

typedef std::shared_ptr<HROVSettings> HROVSettingsPtr;

class HROVSettings {
public:
  const static uint8_t SettingsSize = 10;
  HROVSettings();
  HROVSettings(const merbots_whrov_msgs::hrov_settings::ConstPtr &cstate);
  virtual ~HROVSettings();

  static HROVSettingsPtr BuildHROVSettings() {
    return HROVSettingsPtr(new HROVSettings());
  }
  static HROVSettingsPtr
  BuildHROVSettings(const merbots_whrov_msgs::hrov_settings::ConstPtr &cstate) {
    return HROVSettingsPtr(new HROVSettings(cstate));
  }
  void SetFromROSMsg(const merbots_whrov_msgs::hrov_settings::ConstPtr &);
  merbots_whrov_msgs::hrov_settings::Ptr GetROSMsg();

  void UpdateFromBuffer(uint8_t *);
  void GetBufferCopy(uint8_t *);

  uint8_t *GetBuffer() { return buffer; }

  inline uint16_t GetROIX0() { return _GetROIX0(); }
  inline uint16_t GetROIY0() { return _GetROIY0(); }
  inline uint16_t GetROIX1() { return _GetROIX1(); }
  inline uint16_t GetROIY1() { return _GetROIY1(); }
  inline uint8_t GetROIShift() { return _GetROIShift(); }
  inline uint16_t GetImgSize() { return _GetImgSize(); }

private:
  void _Init();

  void _SetROIConf(uint16_t x0 = 0, uint16_t y0 = 0, uint16_t x1 = 0,
                   uint16_t y1 = 0, uint8_t shift = 0);
  void _SetImgSize(uint16_t _size);
  void _SetImgResolution(uint16_t resolution);
  void _SetMaxPacketLength(uint16_t _maxPacketLength);

  void _GetROIConf(int16_t &x0, int16_t &y0, int16_t &x1, int16_t &y1,
                   uint8_t &shift);
  uint16_t _GetROIX0();
  uint16_t _GetROIY0();
  uint16_t _GetROIX1();
  uint16_t _GetROIY1();
  uint8_t _GetROIShift();

  uint16_t _GetImgSize();

  uint16_t _GetMaxPacketLength();
  uint8_t _GetImgResolution();

  uint8_t buffer[MAX_HROVSTATE_LENGHT];

  uint8_t *roi;
  uint8_t *roi_shift;
  uint16_t *img_size;      //, *img_width, *img_height;
  uint8_t *img_resolution; //, * roi_enabled

  uint16_t *max_packet_length;

  bool bigEndian;
};

} /* namespace merbots */

#endif /* MERBOTS_LIB_INCLUDE_MERBOTS_HROVSETTINGS_H_ */
