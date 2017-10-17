/*
 * ROVCamera.h
 *
 *  Created on: 27 oct. 2016
 *      Author: diego
 */

#ifndef WIRELESS_ARDUSUB_ROV_H_
#define WIRELESS_ARDUSUB_ROV_H_

#include <condition_variable>
#include <cpplogging/Loggable.h>
#include <dccomms/Utils.h>
#include <functional>
#include <iostream>
#include <mutex>
#include <wireless_ardusub/packets/SimplePacket.h>

namespace wireless_ardusub {

using namespace dccomms;
using namespace cpplogging;

class ROV : public Loggable {
public:
  static const int IMG_TRUNK_INFO_SIZE = 2, IMG_FIRST_TRUNK_FLAG = 0x8000,
                   IMG_LAST_TRUNK_FLAG = 0x4000, MAX_IMG_SIZE = 16383,
                   MAX_IMG_TRUNK_LENGTH = 100, MAX_IMG_STATE_LENGTH = 40,
                   IMG_CHKSUM_SIZE = 2, MAX_PACKET_LENGTH = 2048;

  ROV(Ptr<ICommsLink> _comms);
  virtual ~ROV();
  void SendImage(void *, unsigned int);

  // typedef std::function<void(void*, unsigned int)> f_data;
  typedef std::function<void(ROV &)> f_notification;

  void SetOrdersReceivedCallback(f_notification); // f_data);
  void SetLastImgSentCallback(f_notification);

  void GetCurrentRxState(void *dst);
  void SetCurrentTxState(void *src);

  bool SendingCurrentImage();

  void SetChecksumType(FCS fcs);
  void Start();

  virtual void SetLogLevel(cpplogging::LogLevel);
  virtual void SetLogName(string name);
  virtual void FlushLog();
  virtual void FlushLogOn(LogLevel);
  virtual void LogToConsole(bool);
  virtual void LogToFile(const string &filename);

  void SetMaxImageTrunkLength(int);
  void SetRxStateSize(int);
  void SetTxStateSize(int);

private:
  void _WaitForNewOrders();
  void _SendPacketWithCurrentStateAndImgTrunk();
  void _CheckIfEntireImgIsSent();

  void _UpdateCurrentRxStateFromRxState();
  void _UpdateTxStateFromCurrentTxState();
  void _SetEndianess();

  void _Work(); // for full duplex

  std::mutex _immutex, _rxstatemutex, _txstatemutex;
  condition_variable _imgInBufferCond;

  // f_data ordersReceivedCallback;
  f_notification _ordersReceivedCallback;
  f_notification _lastImageSentCallback;

  uint8_t *_buffer;
  uint8_t *_currentRxState, *_currentTxState;

  Ptr<ICommsLink> _comms;

  ServiceThread<ROV> _service;

  Ptr<SimplePacket> _txdlf;
  Ptr<SimplePacket> _rxdlf;
  //// TX /////
  uint8_t *_txbuffer;
  uint8_t *_txStatePtr;
  uint16_t *_imgTrunkInfoPtr;
  uint8_t *_imgTrunkPtr;

  uint8_t *_beginImgPtr;
  uint8_t *_currentImgPtr;
  uint16_t *_imgChksumPtr;
  uint8_t *_endImgPtr;

  //// RX /////
  uint8_t *_rxbuffer;
  uint8_t *_rxStatePtr;
  FCS _dlfcrctype;

  int _rxStateLength, _txStateLength;
  int _imgTrunkInfoLength;
  int _maxImgTrunkLength;
  int _maxPacketLength;

  bool _imgInBuffer;
  bool _bigEndian;
  Timer _rxtimer;

  bool _txStateSet;
};

} /* namespace dcauv */

#endif /* ROVCAMERA_H_ */
