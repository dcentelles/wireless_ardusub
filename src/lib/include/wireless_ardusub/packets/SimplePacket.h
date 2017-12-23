#ifndef WIRELESS_ARDUSUB_PACKETS_SIMPLEPACKET_H_
#define WIRELESS_ARDUSUB_PACKETS_SIMPLEPACKET_H_

#include <dccomms/dccomms.h>
#include <wireless_ardusub/packets/types.h>

using namespace dccomms;
namespace wireless_ardusub {

class SimplePacket : public Packet {
public:
  SimplePacket(int payloadSize, FCS fcs = CRC16);
  void CopyFromRawBuffer(void *buffer);
  uint8_t *GetPayloadBuffer();
  uint32_t GetPayloadSize();
  int GetPacketSize();
  void Read(Stream *comms);
  void PayloadUpdated(uint32_t payloadSize);

  bool PacketIsOk();

  void GetPayload(void *copy, int size);
  void SetPayload(const void *data, int size);


  void UpdateFCS();

private:
  int PAYLOAD_SIZE, FCS_SIZE;
  static const int PRE_SIZE = 1;

  uint8_t *_pre;
  uint8_t *_payload;
  uint8_t *_fcs;
  int _packetSize;
  void _Init();
  bool _CheckFCS();
};

class SimplePacketBuilder : public IPacketBuilder {
public:
  SimplePacketBuilder(int payloadSize, FCS fcs = CRC16) {}
  PacketPtr CreateFromBuffer(void *buffer) {
    auto pkt = CreateObject<SimplePacket>(_payloadSize, _fcs);
    pkt->CopyFromRawBuffer(buffer);
    return pkt;
  }
  PacketPtr Create() { return CreateObject<SimplePacket>(_payloadSize, _fcs); }

private:
  int _payloadSize;
  FCS _fcs;
};
}
#endif
