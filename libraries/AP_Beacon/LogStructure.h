#pragma once

#include <AP_Logger/LogStructure.h>
#include "AP_Beacon_config.h"

#define LOG_IDS_FROM_BEACON \
    LOG_BEACON_MSG

// @LoggerMessage: BCN
// @Description: Beacon information
// @Field: TimeUS: Time since system startup
// @Field: Health: True if beacon sensor is healthy
// @Field: Cnt: Number of beacons being used
// @Field: D0: Distance to first beacon
// @Field: D1: Distance to second beacon
// @Field: D2: Distance to third beacon
// @Field: D3: Distance to fourth beacon
// @Field: PosX: Calculated beacon position, x-axis
// @Field: PosY: Calculated beacon position, y-axis
// @Field: PosZ: Calculated beacon position, z-axis
// @Field: R0: RX RSSI of first beacon
// @Field: R1: RX RSSI of second beacon
// @Field: R2: RX RSSI of third beacon
// @Field: R3: RX RSSI of fourth beacon
// @Field: F0: FP RSSI of first beacon
// @Field: F1: FP RSSI of second beacon
// @Field: F2: FP RSSI of third beacon
// @Field: F3: FP RSSI of fourth beacon

struct PACKED log_Beacon {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t health;
    uint8_t count;
    float dist0;
    float dist1;
    float dist2;
    float dist3;
    float posx;
    float posy;
    float posz;
    float rx0;
    float rx1;
    float rx2;
    float rx3;
    float fp0;
    float fp1;
    float fp2;
    float fp3;
};

#if AP_BEACON_ENABLED
#define LOG_STRUCTURE_FROM_BEACON \
    { LOG_BEACON_MSG, sizeof(log_Beacon), \
        "BCN", "QBBfffffffffffffff",  "TimeUS,Health,Cnt,D0,D1,D2,D3,PosX,PosY,PosZ,R0,R1,R2,R3,F0,F1,F2,F3", "s--mmmmmmmdBBBdBBB", "F--000000000000000", true },
#else
#define LOG_STRUCTURE_FROM_BEACON
#endif
