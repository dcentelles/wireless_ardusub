#ifndef WIRELESS_ARDUSUB_NODES_CONSTANTS_H_
#define WIRELESS_ARDUSUB_NODES_CONSTANTS_H_

namespace wireless_ardusub {

namespace teleop_v3 {
static const int IMG_TRUNK_INFO_SIZE = 2;
static const int IMG_FIRST_TRUNK_FLAG = 0x8000;
static const int IMG_LAST_TRUNK_FLAG = 0x4000;
static const int MAX_IMG_SIZE = 20000;
static const int MAX_IMG_TRUNK_LENGTH = 100;
static const int MAX_NODE_STATE_LENGTH = 40;
static const int IMG_CHKSUM_SIZE = 2;
static const int MAX_PACKET_LENGTH = 2048;
}
}

#endif
