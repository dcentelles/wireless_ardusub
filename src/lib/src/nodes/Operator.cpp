/*
 * ROVOperator.cpp
 *
 *  Created on: 27 oct. 2016
 *      Author: diego
 */

#include <dccomms/Checksum.h>
#include <dccomms/DataLinkFrame.h>
#include <wireless_ardusub/nodes/Operator.h>

namespace wireless_ardusub {

using namespace dccomms;

void defaultImageReceivedCallback(Operator &rovOperator) {}

void defaultStateReceivedCallback(Operator &rovOperator) {}

Operator::Operator() : txservice(this), rxservice(this) {
  rxbuffer = 0;
  imgTrunkPtr = 0;
  txbuffer = 0;
  imgTrunkInfoPtr = 0;
  currentImgPtr = 0;
  rxStatePtr = 0;
  txStatePtr = 0;

  bigEndian = DataLinkFrame::IsBigEndian();
  imgTrunkInfoLength = IMG_TRUNK_INFO_SIZE;
  maxImgTrunkLength = MAX_IMG_TRUNK_LENGTH;
  maxPacketLength = MAX_PACKET_LENGTH;

  dlfcrctype = CRC16;
  buffer = new uint8_t[maxPacketLength + MAX_IMG_SIZE + IMG_CHKSUM_SIZE +
                       MAX_IMG_SIZE];

  currentRxState = buffer;
  _UpdateRxStateSize(MAX_NODE_STATE_LENGTH);
  _UpdateTxStateSize(MAX_NODE_STATE_LENGTH);

  lastImgSize = 0;
  imgInBuffer = false;
  txservice.SetWork(&Operator::_TxWork);
  rxservice.SetWork(&Operator::_RxWork);

  imageReceivedCallback = &defaultImageReceivedCallback;
  stateReceivedCallback = &defaultStateReceivedCallback;

  desiredStateSet = false;
  _canTransmit = true;
}

Operator::~Operator() {
  if (txservice.IsRunning())
    txservice.Stop();
  if (rxservice.IsRunning())
    rxservice.Stop();
  delete buffer;
}

void Operator::SetComms(Ptr<CommsDevice> comms) { _comms = comms; }

int Operator::GetImageSizeFromNumberOfPackets(int npackets) {
  int fcsSize = 2;
  int res = maxImgTrunkLength * (npackets - 1) + maxImgTrunkLength - fcsSize;
  res = res >= 0 ? res : 0;
  return res;
}

void Operator::SetMaxImageTrunkLength(int _len) {
  _len = _len <= MAX_IMG_TRUNK_LENGTH ? _len : MAX_IMG_TRUNK_LENGTH;
  maxImgTrunkLength = _len;
}

void Operator::SetRxStateSize(int _len) {
  _len = _len <= MAX_NODE_STATE_LENGTH ? _len : MAX_NODE_STATE_LENGTH;
  _UpdateRxStateSize(_len);
  Log->debug("Set a new Rx-State length: {} bytes", _len);
}

void Operator::SetTxStateSize(int _len) {
  _len = _len <= MAX_NODE_STATE_LENGTH ? _len : MAX_NODE_STATE_LENGTH;
  _UpdateTxStateSize(_len);
  Log->debug("Set a new Tx-State length: {} bytes", _len);
}

void Operator::_UpdateRxStateSize(int _len) {
  rxStateLength = _len;
  desiredState = currentRxState + rxStateLength;
  beginImgPtr = desiredState + txStateLength;
  beginLastImgPtr = beginImgPtr + MAX_IMG_SIZE;
}

void Operator::_UpdateTxStateSize(int _len) {
  txStateLength = _len;
  beginImgPtr = desiredState + txStateLength;
  beginLastImgPtr = beginImgPtr + MAX_IMG_SIZE;
}

int Operator::GetLastReceivedImage(void *data) {
  int imgSize;
  immutex.lock();
  memcpy(data, beginLastImgPtr, lastImgSize);
  imgSize = lastImgSize;
  immutex.unlock();

  return imgSize;
}

void Operator::GetLastConfirmedState(void *data) {
  rxstatemutex.lock();
  memcpy(data, currentRxState, rxStateLength);
  rxstatemutex.unlock();
}

void Operator::SetDesiredState(const void *_data) {
  txstatemutex.lock();
  memcpy(desiredState, _data, txStateLength);
  txstatemutex.unlock();
  desiredStateSet = true;
}

void Operator::SetImageReceivedCallback(f_notification _callback) {
  imageReceivedCallback = _callback;
}

void Operator::SetStateReceivedCallback(f_notification _callback) {
  stateReceivedCallback = _callback;
}

uint32_t Operator::GetTxPacketSize() { return txStateLength; }

uint32_t Operator::GetRxPacketSize() {
  return rxStateLength + imgTrunkInfoLength + maxImgTrunkLength;
}

void Operator::Start() {
  txdlf = CreateObject<SimplePacket>(GetTxPacketSize(), dlfcrctype);
  rxdlf = CreateObject<SimplePacket>(GetRxPacketSize(), dlfcrctype);

  auto txdlbuffer = txdlf->GetPayloadBuffer();
  auto rxdlbuffer = rxdlf->GetPayloadBuffer();

  txbuffer = txdlf->GetPayloadBuffer();
  rxbuffer = rxdlf->GetPayloadBuffer();

  txStatePtr = txbuffer;
  rxStatePtr = rxbuffer;

  imgTrunkInfoPtr = (uint16_t *)(rxStatePtr + rxStateLength);
  imgTrunkPtr = ((uint8_t *)imgTrunkInfoPtr) + IMG_TRUNK_INFO_SIZE;

  currentImgPtr = beginImgPtr;

  if (!_comms) {
    Log->info("CommsDevice type: dccomms::CommsDeviceService");
    auto rxPacketSize = GetRxPacketSize();
    auto txPacketSize = GetTxPacketSize();
    Log->info("Transmitted packet size: {} bytes.\n"
              "Received packet size: {} bytes.",
              txPacketSize, rxPacketSize);

    std::string dccommsId = "operator";
    Log->info("dccomms ID: {}", dccommsId);

    dccomms::Ptr<IPacketBuilder> pb =
        dccomms::CreateObject<SimplePacketBuilder>(rxPacketSize);

    dccomms::Ptr<CommsDeviceService> commsService;
    commsService = dccomms::CreateObject<CommsDeviceService>(pb);
    commsService->SetCommsDeviceId(dccommsId);
    commsService->SetLogLevel(LogLevel::info);
    commsService->Start();
    _comms = commsService;
  }
  txservice.Start();
  rxservice.Start();
}

void Operator::DisableTransmission() { _canTransmit = false; }
void Operator::EnableTransmission() { _canTransmit = true; }
void Operator::_Work() {}

void Operator::_TxWork() {
  while (!desiredStateSet) {
    std::this_thread::sleep_for(chrono::milliseconds(750));
  }
  if (_canTransmit) {
    _SendPacketWithDesiredState();
    std::this_thread::sleep_for(chrono::milliseconds(1000));
  } else
    std::this_thread::sleep_for(chrono::milliseconds(50));
}

void Operator::_RxWork() {
  Log->debug("RX: waiting for new state from ROV...");
  _WaitForCurrentStateAndNextImageTrunk(0);
}

void Operator::_WaitForCurrentStateAndNextImageTrunk(int timeout) {
  // Wait for the next packet and call the callback
  Log->debug("RX: waiting for frames...");
  *_comms >> rxdlf;
  if (rxdlf->PacketIsOk()) {
    Log->info("Packet received ({} bytes)", rxdlf->GetPacketSize());
    _UpdateLastConfirmedStateFromLastMsg();
    _UpdateImgBufferFromLastMsg();
    stateReceivedCallback(*this);
  } else {
    Log->warn("Packet received with errors ({} bytes)", rxdlf->GetPacketSize());
  }
}

void Operator::_UpdateImgBufferFromLastMsg() {
  uint16_t trunkInfo = _GetTrunkInfo();
  uint16_t trunkSize = _GetTrunkSize(trunkInfo);

  if (trunkInfo != 0) {
    if (trunkInfo & IMG_FIRST_TRUNK_FLAG) // the received trunk is the first
                                          // trunk of an image
    {
      Log->debug("RX: the received trunk is the first trunk of an image");
      memcpy(beginImgPtr, imgTrunkPtr, trunkSize);
      currentImgPtr = beginImgPtr + trunkSize;
      if (trunkInfo & IMG_LAST_TRUNK_FLAG) // the received trunk is also the
                                           // last of an image (the image only
                                           // has 1 trunk)
      {
        Log->debug("RX: the received trunk is also the last of an image (the "
                   "image only has 1 trunk)");
        _LastTrunkReceived(trunkSize);
      }
    } else // the received trunk is not the first of an image
    {
      if (currentImgPtr != beginImgPtr) // first trunk has already been received
      {
        memcpy(currentImgPtr, imgTrunkPtr, trunkSize);
        currentImgPtr += trunkSize;
        if (trunkInfo &
            IMG_LAST_TRUNK_FLAG) // the received trunk is the last of an image
        {
          Log->debug("RX: the received trunk is the last of an image");
          _LastTrunkReceived(trunkSize);
        }
      } else {
        // else, we are waiting for the first trunk of an image
        Log->debug("RX: waiting for the first trunk of an image");
      }
    }
  } else {
    // else, packet without an image trunk
    Log->warn("RX: packet received without an image trunk");
  }
}
void Operator::_LastTrunkReceived(uint16_t trunkSize) {
  int blockSize = currentImgPtr - beginImgPtr;

  currentImgPtr = beginImgPtr;
  uint16_t crc = Checksum::crc16(beginImgPtr, blockSize);
  if (crc == 0) {
    immutex.lock();
    lastImgSize = blockSize - IMG_CHKSUM_SIZE;
    memcpy(beginLastImgPtr, beginImgPtr, lastImgSize);
    immutex.unlock();

    imageReceivedCallback(*this);
  } else {
    Log->warn("RX: image received with errors... (some packets were lost)");
  }
}

uint16_t Operator::_GetTrunkInfo() {
  uint16_t result;
  if (bigEndian) {
    result = *imgTrunkInfoPtr;
  } else {
    Utils::IntSwitchEndian(&result, *imgTrunkInfoPtr);
  }
  return result;
}

uint16_t Operator::_GetTrunkSize(uint16_t rawInfo) { return rawInfo & 0x3fff; }

void Operator::_SendPacketWithDesiredState() {
  if (desiredStateSet) {
    if (!_comms->BusyTransmitting()) {
      txstatemutex.lock();
      memcpy(txStatePtr, desiredState, txStateLength);
      txstatemutex.unlock();

      txdlf->UpdateFCS();
      Log->info("Sending packet ({} bytes)", txdlf->GetPacketSize());
      *_comms << txdlf;
      while (_comms->BusyTransmitting())
        ;
    }
  } else {
    Log->warn("TX: desired state is not set yet");
    Utils::Sleep(1000);
  }
}

void Operator::_UpdateLastConfirmedStateFromLastMsg() {
  rxstatemutex.lock();
  memcpy(currentRxState, rxStatePtr, rxStateLength);
  rxstatemutex.unlock();
}

} /* namespace dcauv */
