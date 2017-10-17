/*
 * ROVCamera.cpp
 *
 *  Created on: 27 oct. 2016
 *      Author: diego
 */

#include <dccomms/Checksum.h>
#include <wireless_ardusub/nodes/ROV.h>

namespace wireless_ardusub {

using namespace dccomms;
void defaultLastImageSentCallback(ROV &rovcamera) {
  // Nothing to do
}

void defaultOrdersReceivedCallback(ROV &rovcamera) {
  // Nothing to do
}

ROV::ROV(Ptr<ICommsLink> comms) : _service(this) {
  // TODO Auto-generated constructor stub
  _comms = comms;
  _SetEndianess();
  _rxStateLength = MAX_NODE_STATE_LENGTH;
  _txStateLength = MAX_NODE_STATE_LENGTH;
  _imgTrunkInfoLength = IMG_TRUNK_INFO_SIZE;
  _maxImgTrunkLength = MAX_IMG_TRUNK_LENGTH;
  _maxPacketLength = MAX_PACKET_LENGTH;

  _dlfcrctype = CRC16;
  _buffer = new uint8_t[_maxPacketLength + MAX_NODE_STATE_LENGTH * 2 +
                        MAX_IMG_SIZE]; // buffer max size is orientative...
  _currentRxState = _buffer;
  _currentTxState = _currentRxState + _rxStateLength;
  _beginImgPtr = _currentTxState + _txStateLength;
  _imgInBuffer = false;
  _lastImageSentCallback = &defaultLastImageSentCallback;
  _ordersReceivedCallback = &defaultOrdersReceivedCallback;

  _service.SetWork(&ROV::_Work);
  SetLogName("ROV");
  _txStateSet = false;
}

ROV::~ROV() {
  // TODO Auto-generated destructor stub
  if (_service.IsRunning())
    _service.Stop();
  delete _buffer;
}

void ROV::SetMaxImageTrunkLength(int _len) {
  _len = _len <= MAX_IMG_TRUNK_LENGTH ? _len : MAX_IMG_TRUNK_LENGTH;
  _maxImgTrunkLength = _len;
  Log->debug("Set a new maximum image trunk length: {} bytes", _len);
}

void ROV::SetTxStateSize(int _len) {
  _len = _len <= MAX_NODE_STATE_LENGTH ? _len : MAX_NODE_STATE_LENGTH;
  _txStateLength = _len;
  _beginImgPtr = _currentTxState + _txStateLength;
  Log->debug("Set a new Tx-State length: {} bytes", _len);
}
void ROV::SetRxStateSize(int _len) {
  _len = _len <= MAX_NODE_STATE_LENGTH ? _len : MAX_NODE_STATE_LENGTH;
  _rxStateLength = _len;
  _currentTxState = _currentRxState + _rxStateLength;
  SetTxStateSize(_txStateLength);
  Log->debug("Set a new Rx-State length: {} bytes", _len);
}

void ROV::SetLogLevel(cpplogging::LogLevel _level) {
  Loggable::SetLogLevel(_level);
  _comms->SetLogLevel(_level);
}

void ROV::SetLogName(string name) {
  Loggable::SetLogName(name);
  _comms->SetLogName(name + ":Comms");
}

void ROV::LogToConsole(bool c) {
  Loggable::LogToConsole(c);
  _comms->LogToConsole(c);
}

void ROV::LogToFile(const string &filename) {
  Loggable::LogToFile(filename);
  _comms->LogToFile(filename + "_service");
}

void ROV::FlushLog() {
  Loggable::FlushLog();
  _comms->FlushLog();
}

void ROV::FlushLogOn(LogLevel level) {
  Loggable::FlushLogOn(level);
  _comms->FlushLogOn(level);
}

void ROV::SetChecksumType(FCS fcs) { _dlfcrctype = fcs; }

void ROV::SendImage(void *_buf, unsigned int _length) {
  // TODO: aply a hash to de img in order to check errors when reassembling
  // trunks
  std::unique_lock<std::mutex> lock(_immutex);
  while (_imgInBuffer) {
    _imgInBufferCond.wait(lock);
  }
  memcpy(_beginImgPtr, _buf, _length);
  _imgChksumPtr = (uint16_t *)(_beginImgPtr + _length);
  _endImgPtr = ((uint8_t *)_imgChksumPtr) + IMG_CHKSUM_SIZE;
  _currentImgPtr = _beginImgPtr;

  uint16_t imgChksum = Checksum::crc16(_beginImgPtr, _length);
  if (_bigEndian) {
    *_imgChksumPtr = imgChksum;
  } else {
    Utils::IntSwitchEndian(_imgChksumPtr, imgChksum);
  }

  // TEMPORAL CHECK
  uint32_t crc = Checksum::crc16(_beginImgPtr, _length + IMG_CHKSUM_SIZE);
  if (crc != 0) {
    Log->critical("data link frame with errors before transmission");
  }

  _imgInBuffer = true;
  Log->debug("New image available to transmit ({} bytes).", _length);

  // mutex is unlocked automatically when calling the unique_lock destructor:
  // http://www.cplusplus.com/reference/mutex/unique_lock/
}

void ROV::SetLastImgSentCallback(f_notification _callback) {
  _immutex.lock();
  _lastImageSentCallback = _callback;
  _immutex.unlock();
}

void ROV::SetOrdersReceivedCallback(f_notification _callback) {
  _rxstatemutex.lock();
  _ordersReceivedCallback = _callback;
  _rxstatemutex.unlock();
}

bool ROV::SendingCurrentImage() { return _imgInBuffer; }

void ROV::Start() {
  _txdlf = CreateObject<SimplePacket>(
      _txStateLength + _imgTrunkInfoLength + _maxImgTrunkLength, _dlfcrctype);
  _rxdlf = CreateObject<SimplePacket>(_rxStateLength, _dlfcrctype);

  _txbuffer = _txdlf->GetPayloadBuffer();
  _rxbuffer = _rxdlf->GetPayloadBuffer();

  _txStatePtr = _txbuffer;
  _imgTrunkInfoPtr = (uint16_t *)(_txStatePtr + _txStateLength);
  _imgTrunkPtr = ((uint8_t *)_imgTrunkInfoPtr) + IMG_TRUNK_INFO_SIZE;

  _currentImgPtr = _beginImgPtr; // No image in buffer
  _endImgPtr = _currentImgPtr;   //

  _rxStatePtr = _rxbuffer;

  _service.Start();
}

void ROV::_WaitForNewOrders() {
  *_comms >> _rxdlf;
  if (_rxdlf->PacketIsOk()) {
    Log->info("Packet received ({} bytes)", _rxdlf->GetPacketSize());
    _UpdateCurrentRxStateFromRxState();
    _ordersReceivedCallback(*this);
  } else {
    Log->warn("Packet received with errors ({} bytes)",
              _rxdlf->GetPacketSize());
  }
}
void ROV::_Work() {
  _WaitForNewOrders();
  _immutex.lock();
  _SendPacketWithCurrentStateAndImgTrunk();
  _CheckIfEntireImgIsSent();
  _immutex.unlock();
}

void ROV::_UpdateCurrentRxStateFromRxState() {
  _rxstatemutex.lock();
  memcpy(_currentRxState, _rxStatePtr, _rxStateLength);
  _rxstatemutex.unlock();
}
void ROV::_UpdateTxStateFromCurrentTxState() {
  _txstatemutex.lock();
  memcpy(_txStatePtr, _currentTxState, _txStateLength);
  _txstatemutex.unlock();
}

void ROV::GetCurrentRxState(void *dst) {
  _rxstatemutex.lock();
  memcpy(dst, _currentRxState, _rxStateLength);
  _rxstatemutex.unlock();
}

void ROV::SetCurrentTxState(void *src) {
  _txstatemutex.lock();
  memcpy(_currentTxState, src, _txStateLength);
  _txstatemutex.unlock();
  _txStateSet = true;
}

void ROV::_SendPacketWithCurrentStateAndImgTrunk() {
  // TODO: Prepare the next packet with the next image's trunk and send it
  // unsigned long a1 = (unsigned long) endImgPtr;
  // unsigned long a0 = (unsigned long) currentImgPtr;
  if (_comms->BusyTransmitting()) {
    Log->critical("TX: possible bug: device busy transmitting at init..");
    return;
  }
  if (_txStateSet) {
    int bytesLeft = _endImgPtr - _currentImgPtr;
    int nextTrunkLength;
    uint16_t trunkInfo = 0;

    _UpdateTxStateFromCurrentTxState();

    if (bytesLeft > 0) // == (ImgInBuffer == True)
    {
      if (bytesLeft > _maxImgTrunkLength) {
        nextTrunkLength = _maxImgTrunkLength;
      } else {
        trunkInfo |= IMG_LAST_TRUNK_FLAG;
        nextTrunkLength = bytesLeft;
      }
      if (_beginImgPtr == _currentImgPtr)
        trunkInfo |= IMG_FIRST_TRUNK_FLAG;

      trunkInfo |= nextTrunkLength;

      if (_bigEndian)
        *_imgTrunkInfoPtr = trunkInfo;
      else {
        Utils::IntSwitchEndian(_imgTrunkInfoPtr, trunkInfo);
      }

      memcpy(_imgTrunkPtr, _currentImgPtr, nextTrunkLength);
      _currentImgPtr += nextTrunkLength;

      _txdlf->UpdateFCS();
    } else {
      *_imgTrunkInfoPtr = 0;
      _txdlf->UpdateFCS();
    }
    Log->info("Sending packet ({} bytes)", _txdlf->GetPacketSize());
    *_comms << _txdlf;
  } else {
    Log->warn("TX: current state is not set yet");
    Utils::Sleep(1000);
  }
}

void ROV::_CheckIfEntireImgIsSent() {
  // Check If it has been sent the last image trunk and call the callback
  if (_imgInBuffer && _currentImgPtr == _endImgPtr) {
    _imgInBuffer = false;
    _currentImgPtr = _beginImgPtr;
    _endImgPtr = _currentImgPtr;
    _imgInBufferCond.notify_one();
    _lastImageSentCallback(*this);
    Log->debug("TX: image transmission completed");
  }
}

void ROV::_SetEndianess() { _bigEndian = DataLinkFrame::IsBigEndian(); }

} /* namespace dcauv */
