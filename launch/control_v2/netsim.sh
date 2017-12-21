#!/bin/bash

rosservice call /dccomms_netsim/add_custom_channel "id: 0
minPrTime: 2.0
prTimeIncPerMeter: 0.0"

rosservice call /dccomms_netsim/add_custom_net_device "{dccommsId: 'rov', mac: 1, frameId: '', bitrate: 500.0, bitrateSd: 0.0, maxDistance: 9999999,
  minDistance: 0, minPktErrorRate: 0.0, pktErrorRateIncPerMeter: 0.0}" 

rosservice call /dccomms_netsim/add_custom_net_device "{dccommsId: 'operator', mac: 2, frameId: '', bitrate: 500.0, bitrateSd: 0.0, maxDistance: 9999999,
  minDistance: 0, minPktErrorRate: 0.0, pktErrorRateIncPerMeter: 0.0}" 

rosservice call /dccomms_netsim/link_dev_to_channel "{dccommsId: 'rov', channelId: 0}" 
rosservice call /dccomms_netsim/link_dev_to_channel "{dccommsId: 'operator', channelId: 0}" 

rosservice call /dccomms_netsim/start_simulation 
