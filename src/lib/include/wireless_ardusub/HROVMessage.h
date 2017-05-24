/*
 * HROVOrders.h
 *
 *  Created on: 30 nov. 2016
 *      Author: centelld
 */

#ifndef MERBOTS_LIB_INCLUDE_MERBOTS_HROVMESSAGE_H_
#define MERBOTS_LIB_INCLUDE_MERBOTS_HROVMESSAGE_H_

#include <merbots_whrov_msgs/movement.h>
#include <wireless_ardusub/Constants.h>


namespace wireless_ardusub {

class HROVMessage;

typedef std::shared_ptr<HROVMessage> HROVMessagePtr;

class HROVMessage {
public:
        const static uint8_t MessageLength;
        HROVMessage();
        HROVMessage(uint8_t *);
        virtual ~HROVMessage();

        static HROVMessagePtr BuildHROVMessage()
        {
            return HROVMessagePtr(new HROVMessage());
        }
        static HROVMessagePtr BuildHROVMessage(uint8_t * _buffer)
        {
            return HROVMessagePtr(new HROVMessage(_buffer));
        }

        void UpdateFromBuffer(uint8_t *);
        void GetBufferCopy(uint8_t *);

        uint8_t * GetBuffer(){return buffer;}

        static uint8_t GetNextOrderSeqNumber(uint8_t sq);

        bool Ready();
        void Ready(bool);
        uint8_t GetExpectedOrderSeqNumber(); //returns 1 or 0

        //void SetExpectedOrderSeqNumber(uint8_t);
        void IncExpectedOrderSeqNumber();
        void SetYaw(uint16_t);
        uint16_t GetYaw();
        int16_t GetZ();
        void SetZ(double);
        int16_t GetX();
        void SetX(double);
        int16_t GetY();
        void SetY(double);

        void LastOrderCancelledFlag(bool);
        bool LastOrderCancelledFlag();

private:
        void _Init();
        uint8_t buffer[MAX_HROVSTATE_LENGHT];

        const static uint8_t
            READY_FLAG = 0x80,
            NEXT_ORDER_SEQ_FLAG = 0x40,
            LAST_ORDER_CANCELLED_FLAG = 0x20
            ;
        uint8_t *flags, *pose;

        bool bigEndian;


};

} /* namespace merbots */

#endif /* MERBOTS_LIB_INCLUDE_MERBOTS_HROVMESSAGE_H_ */
