/*
 * ROVOperator.h
 *
 *  Created on: 27 oct. 2016
 *      Author: diego
 */

#ifndef WIRELESS_ARDUSUB_OPERATOR_H_
#define WIRELESS_ARDUSUB_OPERATOR_H_

#include <cpplogging/Loggable.h>
#include <dccomms/Utils.h>
#include <functional>
#include <iostream>
#include <mutex>
#include <wireless_ardusub/nodes/Constants.h>
#include <wireless_ardusub/packets/SimplePacket.h>

namespace wireless_ardusub {

using namespace dccomms;
using namespace cpplogging;
using namespace std;
using namespace teleop_v3;

class Operator : public Loggable {
public:
  Operator(Ptr<CommsDevice> comms);
  virtual ~Operator();
  void SetDesiredState(const void *data);

  // http://stackoverflow.com/questions/2298242/callback-functions-in-c
  /*
      //mode 1:
      typedef void (*f_data)(void*,unsigned int);
      void SetDataReceivedCallback(f_data);
      */
  // mode 2:
  typedef std::function<void(Operator &)> f_notification;

  void SetImageReceivedCallback(f_notification);
  void SetStateReceivedCallback(f_notification);

  int GetLastReceivedImage(void *);
  void GetLastConfirmedState(void *);

  void Start();

  virtual void SetLogLevel(cpplogging::LogLevel);
  virtual void SetLogName(string name);
  virtual void FlushLog();
  virtual void FlushLogOn(LogLevel);
  virtual void LogToConsole(bool);
  virtual void LogToFile(const string &filename);

  int GetImageSizeFromNumberOfPackets(int npackets);
  void SetMaxImageTrunkLength(int);
  void SetRxStateSize(int);
  void SetTxStateSize(int);
  void DisableTransmission();
  void EnableTransmission();

private:
  void _UpdateRxStateSize(int);
  void _UpdateTxStateSize(int);

  bool _canTransmit;
  f_notification imageReceivedCallback;
  f_notification stateReceivedCallback;

  std::mutex immutex, txstatemutex, rxstatemutex;
  uint8_t *buffer;
  uint8_t *currentRxState,
      *desiredState, // == currentTxState
      *beginImgPtr, *beginLastImgPtr;
  uint16_t lastImgSize;

  Ptr<CommsDevice> _comms;
  Ptr<SimplePacket> txdlf;
  Ptr<SimplePacket> rxdlf;

  ServiceThread<Operator> txservice;
  ServiceThread<Operator> rxservice;

  FCS dlfcrctype;

  ///// TX ////
  uint8_t *txStatePtr, *txbuffer;

  ///// RX ////
  uint8_t *rxStatePtr, *imgTrunkPtr, *rxbuffer, *currentImgPtr;

  uint16_t *imgTrunkInfoPtr;

  int rxStateLength, txStateLength;
  int imgTrunkInfoLength;
  int maxImgTrunkLength;
  int maxPacketLength;
  // int minPacketLength;

  bool imgInBuffer;
  bool bigEndian;
  Timer rxtimer;

  void _WaitForCurrentStateAndNextImageTrunk(int millis_timeout);
  void _UpdateImgBufferFromLastMsg();
  void _SendPacketWithDesiredState();
  void _UpdateLastConfirmedStateFromLastMsg();

  void _Work(); // for half duplex

  void _RxWork(); // for full duplex
  void _TxWork(); // for full duplex

  uint16_t _GetTrunkInfo();
  uint16_t _GetTrunkSize(uint16_t rawInfo);
  void _LastTrunkReceived(uint16_t trunkSize);

  bool desiredStateSet;
};

} /* namespace dcauv */

#endif /* ROVOPERATOR_H_ */
