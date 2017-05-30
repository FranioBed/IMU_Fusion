#pragma once

#include "prio_device.hpp"

class PrioBaseStation;

#define PRIO_NUM_STREAMING_SLOTS 8

class PrioHub : public PrioDevice
{
public:
    PrioHub();
    PrioHub(const char* port);
    PrioHub(const PrioHub& other);
    ~PrioHub();

    PRIO_ERROR _init();

    void operator =(const PrioHub& other);

    //Port control functions
    PRIO_ERROR openPort(const char* port);
    PRIO_ERROR closePort();

    PRIO_ERROR reenumerateDevice();
    //Header control functions
    PRIO_ERROR enableTimestampsWired();
    PRIO_ERROR disableTimestampsWired();

    //Streaming functions
    PRIO_ERROR enableStreamingWireless(U32 data_flags, U32 interval, U32 duration, U32 delay = 0);
    PRIO_ERROR disableStreamingWireless();
    PRIO_ERROR startStreamingWired(prio_stream_commands data_flags, U32 interval, U32 duration, U32 delay = 0, bool reg = true);
    PRIO_ERROR stopStreamingWired(bool dereg = true);
    PRIO_ERROR getFirstStreamingPacket(PrioStreamPacket* packet);
    PRIO_ERROR getLastStreamingPacket(PrioStreamPacket* packet);
    U32 getFullLengthOfStreamData();

    U32 getStreamingPacketsInWaiting();
    bool didStreamingOverflow();

    float getStreamUpdateRate();

    //Recording functions
    PRIO_ERROR startRecordingWired(prio_stream_commands data_flags, U32 interval, U32 duration, U32 delay = 0, bool reg = true);
    PRIO_ERROR stopRecordingWired(bool dereg = true);

    PRIO_ERROR disableRecordingWireless();

    void setupRecordingOptions(U32 buffer_size, bool is_wrapping = false);
    U32 getMaxRecordedSamples();
    bool isBufferWrapping();
    U32 getLengthOfRecordedSamples();
    PRIO_ERROR getRecordedSamples(PrioHeader *header_data, U8* packet_data, U32 packet_count);
    PRIO_ERROR getRecordedSampleAtIndex(PrioHeader *header_data, U8* packet_data, U32 index);
    PRIO_ERROR popFrontRecordedSample(PrioHeader *header_data, U8* packet_data);
    void clearRecordedSamples();

    std::vector<U8> getActiveSensors();

    //Call and response functions
    //Commands
    // 10(0x0A)
    PRIO_ERROR getControllerData(U8 index, U8* logical_id, U8* data);
    // 80(0x50)
    PRIO_ERROR setStreamingSlots(prio_stream_commands stream_slots);
    // 81(0x51)
    PRIO_ERROR getStreamingSlots(prio_stream_commands* stream_slots);
    // 82(0x52)
    PRIO_ERROR setStreamingInterval(U32 interval);
    // 83(0x53)
    PRIO_ERROR getStreamingInterval(U32* interval);
    // 176(0xB0)
    PRIO_ERROR enumerateHub();
    // 177(0xB1)
    PRIO_ERROR getHubEnumerationValue(U64* enumeration_value);
    // 178(0xB2)
    PRIO_ERROR getNumberOfEnumeratedSensors(U8* num_sensors);
    // 179(0xB3)
    PRIO_ERROR resetAllSensors();
    // 180(0xb4)
    PRIO_ERROR powerDownAllSensors();
    // 181(0xb5)
    PRIO_ERROR powerUpAllSensors();
    // 183(0xB7)
    PRIO_ERROR setAutoEnumerationMode(bool enable);
    // 184(0xB8)
    PRIO_ERROR getAutoEnumerationMode(bool* enabled);
    // 192(0xC0)
    PRIO_ERROR getWirelessPanID(U16* panid);
    // 193(0xC1)
    PRIO_ERROR setWirelessPanID(U16 panid);
    // 194(0xC2)
    PRIO_ERROR getWirelessChannel(U8* channel);
    // 195(0xc3)
    PRIO_ERROR setWirelessChannel(U8 channel);
    // 198(0xc6)
    PRIO_ERROR getWirelessAddress(U16* address);
    // 202(0xCa)
    PRIO_ERROR getBatteryPercentRemaining(U8* battery_percent);
    // 203(0xCb)
    PRIO_ERROR getBatteryStatus(U8* status);
    // 227(0xE3)
    PRIO_ERROR setPerformanceMode(bool enable);
    // 228(0xE4)
    PRIO_ERROR getPerformanceMode(bool* enabled);
    // 250(0xFA)
    PRIO_ERROR getButtonState(U8* button_state);
    // 255(0xE1)
    PRIO_ERROR commitSettings();

    PRIO_ERROR getSerialNumberAtLogicalID(U8 logical_id, U32* serial_number);
    PRIO_ERROR getFirmwareVersionAtLogicalID(U8 logical_id, const char* firmware_version);
    PRIO_ERROR getHardwareVersionAtLogicalID(U8 logical_id, const char* hardware_version);

    // Sensor Functions
    void setCommandRetries(U8 retries);
    // 0(0x00)
    PRIO_ERROR getTaredOrientationAsQuaternion(U8 logical_id, float* quat, U8 num_floats);
    // 6(0x06)
    PRIO_ERROR getUntaredOrientationAsQuaternion(U8 logical_id, float* quat, U8 num_floats);
    // 37(0x25)
    PRIO_ERROR getCorrectedSensorData(U8 logical_id, float* gyroscope, float* accelrometer, float* magnetometer);
    // 38(0x26)
    PRIO_ERROR getCorrectedGyroscope(U8 logical_id, float* gyroscope);
    // 39(0x27)
    PRIO_ERROR getCorrectedAccelerometer(U8 logical_id, float* accelerometer);
    // 40(0x28)
    PRIO_ERROR getCorrectedMagnetometer(U8 logical_id, float* magnetometer);
    // 64(0x40)
    PRIO_ERROR getRawSensorData(U8 logical_id, float* gyroscope, float* accelrometer, float* magnetometer);
    // 65(0x41)
    PRIO_ERROR getRawGyroscope(U8 logical_id, float* gyroscope);
    // 66(0x42)
    PRIO_ERROR getRawAccelerometer(U8 logical_id, float* accelerometer);
    // 67(0x43)
    PRIO_ERROR getRawMagnetometer(U8 logical_id, float* magnetometer);
    // 96(0x60)
    PRIO_ERROR tareWithCurrentOrientation(U8 logical_id);
    // 160(0xA0)
    PRIO_ERROR setMagnetometerCalibParams(U8 logical_id, const float* scale9, const float* bias3);
    // 161(0xA1)
    PRIO_ERROR setAccelerometerCalibParams(U8 logical_id, const float* scale9, const float* bias3);
    // 162(0xA2)
    PRIO_ERROR getMagnetometerCalibParams(U8 logical_id, float* scale9, float* bias3);
    // 163(0XA3)
    PRIO_ERROR getAccelerometerCalibParams(U8 logical_id, float* scale9, float* bias3);
    // 164(0xA4)
    PRIO_ERROR getGyroCalibParams(U8 logical_id, float* scale9, float* bias3);
    // 166(0xA6)
    PRIO_ERROR setGyroCalibParams(U8 logical_id, const float* scale9, const float* bias3);
    // 167(0xA7)
    PRIO_ERROR setBetaCalcParams(U8 logical_id, float settle_base, float settle_div, float base, float mul);
    // 168(0xA8)
    PRIO_ERROR getBetaCalcParams(U8 logical_id, float* settle_base, float* settle_div, float* base, float* mul);
    // 169(0xA9)
    PRIO_ERROR setAutoCalibEnabled(U8 logical_id, U8 enabled);
    // 170(0xAA)
    PRIO_ERROR getAutoCalibEnabled(U8 logical_id, U8* enabled);
    // 225(0xE1)
    PRIO_ERROR commitSensorSettings(U8 logical_id);

    PRIO_ERROR getAllControllerData(U8* logical_id_1, U8* logical_id_2, U8* data_1, U8* data_2, U8* button_state);

    //not private, but not meant to be called by end-users
    void _parseStreamingPacketItem(U8* dest, U8* src, U16& src_index, U32 nbytes);
    void _parseStreamingPacketItemFloats(float* dest, U8* src, U16& src_index, U16 nfloats);
    PRIO_ERROR _parseStreamingPacket(PrioStreamPacket* packet);
    void _addToStreamingSlots(U8 command, U8 nbytes, U8* command_buff, U8& curr_slot);
    void _setupEnumeratorBitfield();
    PRIO_ERROR _prepareStreaming(prio_stream_commands data_flags, U32 interval, U32 duration, U32 delay = 0);
    PRIO_ERROR _triggerStreaming();

    std::vector<U8> _active_nodes;
    U64 _enumeration_bitfield;
    U8 _joystick_1_id;
    U8 _joystick_2_id;
    bool swapStreamJoystick;
    U8 _streamingSlots[8];
    U8 _streamingSlotIndex[8];
    U8 _streamDataSize;
    deque<PrioStreamPacket> _streamBuffer;
    U32 _streamInterval;
    U32 _recordDataSize;
    clock_t last_time;
    float average_time;
    deque<float> packet_times;

    std::thread _parser_thread;
    void _parseStreamData();
    bool _breakParseThread;
    std::mutex _parser_mutex;
    vector<U8> _unparsed_stream_data;

    std::mutex _readerThreadMutex;
    PrioBaseStation* _ownerDongle;
};