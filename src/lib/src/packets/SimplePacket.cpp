#include <wireless_ardusub/packets/SimplePacket.h>

namespace wireless_ardusub {
SimplePacket::SimplePacket(int payloadSize, FCS fcs) {
  FCS_SIZE = fcs == CRC16 ? 2 : 0;
  PAYLOAD_SIZE = payloadSize;
  _packetSize = PRE_SIZE + PAYLOAD_SIZE + FCS_SIZE;
  _AllocBuffer(_packetSize);
  _Init();
}

void SimplePacket::_Init() {
  _pre = GetBuffer();
  *_pre = 0x55;
  _payload = _pre + PRE_SIZE;
  _fcs = _payload + PAYLOAD_SIZE;
}

void SimplePacket::CopyFromRawBuffer(void *buffer) {
  _SetBuffer(buffer);
  _Init();
}

inline uint8_t *SimplePacket::GetPayloadBuffer() { return _payload; }

inline uint32_t SimplePacket::GetPayloadSize() { return PAYLOAD_SIZE; }

inline int SimplePacket::GetPacketSize() { return _packetSize; }

void SimplePacket::Read(Stream *stream) {
  stream->WaitFor(_pre, PRE_SIZE);
  stream->Read(_payload, PAYLOAD_SIZE + FCS_SIZE);
}

void SimplePacket::GetPayload(void *copy, int size) {
  auto copySize = PAYLOAD_SIZE < size ? PAYLOAD_SIZE : size;
  memcpy(copy, _payload, copySize);
}

void SimplePacket::SetPayload(const void *data, int size) {
  auto copySize = PAYLOAD_SIZE < size ? PAYLOAD_SIZE : size;
  memcpy(_payload, data, copySize);
}

void SimplePacket::UpdateFCS() {
  switch (FCS_SIZE) {
  case 2: {
    uint16_t crc = Checksum::crc16(_payload, PAYLOAD_SIZE);
    *_fcs = (uint8_t)(crc >> 8);
    *(_fcs + 1) = (uint8_t)(crc & 0xff);
  } break;
  default:
    break;
  }
}

bool SimplePacket::_CheckFCS() {
  switch (FCS_SIZE) {
  case 2: {
    uint16_t crc = Checksum::crc16(_payload, PAYLOAD_SIZE + FCS_SIZE);
    return crc == 0;
  }
  default:
    return true;
  }
}
bool SimplePacket::PacketIsOk() { return _CheckFCS(); }
}
