#pragma once

#include "prio_hub.hpp"
#include "prio_device.hpp"

#define PRIO_BASE_STATION_NUM_CHILDREN 1
#define PRIO_DEFAULT_WIRELESS_COMMAND_RETRIES 5

class PrioBaseStation : public PrioDevice
{
public:
    PrioBaseStation(const char* port);
    PrioBaseStation(const PrioBaseStation& other);
    ~PrioBaseStation();
    void operator =(const PrioBaseStation& other);

    //Port control functions
    PRIO_ERROR openPort(const char* port);
    PRIO_ERROR closePort();

    //Header control functions
    PRIO_ERROR enableTimestampsWireless();
    PRIO_ERROR disableTimestampsWireless();

    //Wireless sensor functions
    PRIO_ERROR getPairedHub(shared_ptr<PrioHub>& sensor);

    //Streaming Commands
    // 85(0x55)
    PRIO_ERROR startStreaming(U32 data_flags, U32 interval, U32 duration, bool reg=true);
    // 86(0x56)
    PRIO_ERROR stopStreaming(bool dereg=true);

    //Recording functions
    PRIO_ERROR startRecording(U32 data_flags, U32 interval, U32 duration);
    // 86(0x56)
    PRIO_ERROR stopRecording();

    //Call and response functions
    // 192(0xC0)
    PRIO_ERROR getWirelessPanID(U16* panid);
    // 193(0xC1)
    PRIO_ERROR setWirelessPanID(U16 panid);
    // 194(0xC2)
    PRIO_ERROR getWirelessChannel(U8* channel);
    // 195(0xc3)
    PRIO_ERROR setWirelessChannel(U8 channel);
    // 210(0xd2)
    PRIO_ERROR getWirelessChannelStrengths(U8* channel_strengths16);
    // 214(0xd6)
    PRIO_ERROR getSignalStrength(U8* signal_strength);

    PRIO_ERROR getSerialNumberAtLogicalID(U8 logical_id, U32* serial_number);

    PRIO_ERROR getFirmwareVersionAtLogicalID(U8 logical_id, const char* firmware_version);
    PRIO_ERROR getHardwareVersionAtLogicalID(U8 logical_id, const char* hardware_version);

    PRIO_ERROR commitSettings();

    // 186(0xBa)
    PRIO_ERROR autoPairBaseStationWithHub();

    PRIO_ERROR reenumerateDevice();

    //PRIO_ERROR _setWirelessResponseHeader(U32 flags);
    U16 _detectStreaming(U32 interval_us);

    vector<shared_ptr<PrioHub>> _children; //always has TSS_DONGLE_NUM_CHILDREN elements, unused elements are null
    U32 _responseHeaderWirelessFlagsPreStream;
    U32 _maxStreamInterval;
    U8 _childCount;
};