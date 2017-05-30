#include "prio_api_export.h"
#include "prio_api.hpp"
#include <iostream>
#include <fstream>

vector<shared_ptr<PrioHub>> stored_hubs;
vector<shared_ptr<PrioBaseStation>> stored_bs;
vector<prio_ComPort> stored_prio_ports;
U8 prio_stored_port_index = 0;

#define HUB_RANGE_CHECK() if (hub_id >= stored_hubs.size() || !stored_hubs[hub_id]){return PRIO_INVALID_DEVICE_ID;}
#define BS_RANGE_CHECK() if (bs_id >= stored_bs.size() || !stored_bs[bs_id]){return PRIO_INVALID_DEVICE_ID;}

PRIO_EXPORT PRIO_ERROR prio_initAPI()
{
    gPrioAPI.init();

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_deinitAPI()
{
    gPrioAPI.deinit();

    stored_hubs.clear();
    stored_bs.clear();
    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_createHub(const char* port_name, prio_device_id* out_id)
{
    PrioHub* hub = new PrioHub(port_name);
    if (!hub->isConnected())
    {
        delete hub;
        return PRIO_ERROR_COMMAND_FAIL;
    }

    *out_id = stored_hubs.size();
    stored_hubs.push_back(shared_ptr<PrioHub>(hub));

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_removeHub(prio_device_id hub_id)
{
    HUB_RANGE_CHECK();

    stored_hubs[hub_id].reset();

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_hub_isConnected(prio_device_id hub_id)
{
    HUB_RANGE_CHECK();

    if (!stored_hubs[hub_id]->isConnected())
    {
        return PRIO_ERROR_COMMAND_FAIL;
    }

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_hub_getLastHubTimestamp(prio_device_id hub_id, U32* timestamp)
{
    HUB_RANGE_CHECK();

    *timestamp = stored_hubs[hub_id]->getLastHeader()->SensorTimestamp;

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_hub_getLastSystemTimestamp(prio_device_id hub_id, float* timestamp)
{
    HUB_RANGE_CHECK();

    *timestamp = stored_hubs[hub_id]->getLastHeader()->SystemTimestamp;

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_hub_getSerialNumber(prio_device_id hub_id, U32* serial_number)
{
    HUB_RANGE_CHECK();

    *serial_number = stored_hubs[hub_id]->getSerialNumber();

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_hub_getHardwareVersion(prio_device_id hub_id, const char* hw_version)
{
    HUB_RANGE_CHECK();

    PRIO_ERROR_CHECK(stored_hubs[hub_id]->getHardwareVersion(hw_version));

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_hub_getFirmwareVersion(prio_device_id hub_id, const char* fw_version)
{
    HUB_RANGE_CHECK();

    PRIO_ERROR_CHECK(stored_hubs[hub_id]->getFirmwareVersion(fw_version));

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_bs_getWirelessChannelStrengths(prio_device_id bs_id, U8* channel_strengths16)
{
    BS_RANGE_CHECK();

    PRIO_ERROR_CHECK(stored_bs[bs_id]->getWirelessChannelStrengths(channel_strengths16));

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_bs_getSignalStrength(prio_device_id bs_id, U8* signal_strength)
{
    BS_RANGE_CHECK();

    PRIO_ERROR_CHECK(stored_bs[bs_id]->getSignalStrength(signal_strength));

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_hub_getSensorType(prio_device_id hub_id, U8* sensor_type)
{
    HUB_RANGE_CHECK();

    *sensor_type = stored_hubs[hub_id]->getSensorType();

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_hub_openPort(prio_device_id hub_id, const char* port_name)
{
    HUB_RANGE_CHECK();

    PRIO_ERROR_CHECK(stored_hubs[hub_id]->openPort(port_name));

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_hub_closePort(prio_device_id hub_id)
{
    HUB_RANGE_CHECK();

    PRIO_ERROR_CHECK(stored_hubs[hub_id]->closePort());

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_hub_enableTimestampsWired(prio_device_id hub_id)
{
    HUB_RANGE_CHECK();

    PRIO_ERROR_CHECK(stored_hubs[hub_id]->enableTimestampsWired());

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_hub_disableTimestampsWired(prio_device_id hub_id)
{
    HUB_RANGE_CHECK();

    PRIO_ERROR_CHECK(stored_hubs[hub_id]->disableTimestampsWired());

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_hub_enableStreamingWireless(prio_device_id hub_id, U32 data_flags, U32 interval, U32 duration, U32 delay)
{
    HUB_RANGE_CHECK();

    PRIO_ERROR_CHECK(stored_hubs[hub_id]->enableStreamingWireless(data_flags, interval, duration, delay));

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_hub_disableStreamingWireless(prio_device_id hub_id)
{
    HUB_RANGE_CHECK();

    PRIO_ERROR_CHECK(stored_hubs[hub_id]->disableStreamingWireless());

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_hub_startStreamingWired(prio_device_id hub_id, prio_stream_commands data_flags, U32 interval, U32 duration, U32 delay)
{
    HUB_RANGE_CHECK();

    return stored_hubs[hub_id]->startStreamingWired(data_flags, interval, duration, delay);
}

PRIO_EXPORT PRIO_ERROR prio_hub_stopStreamingWired(prio_device_id hub_id)
{
    HUB_RANGE_CHECK();

    return stored_hubs[hub_id]->stopStreamingWired();
}

PRIO_EXPORT PRIO_ERROR prio_hub_getFirstStreamingPacket(prio_device_id hub_id, prio_StreamHeaderData *header_data, U8* packet_data)
{
    HUB_RANGE_CHECK();

    PrioStreamPacket packet;
    PRIO_ERROR_CHECK(stored_hubs[hub_id]->getFirstStreamingPacket(&packet));
    prio_StreamHeaderData tmp;
    tmp.battery_level = packet.header.BatteryLevel;
    tmp.battery_status = packet.header.BatteryStatus;
    tmp.hub_button = packet.header.HubButton;
    tmp.joystick1.x_Axis = packet.header.Joystick1.x_Axis;
    tmp.joystick1.y_Axis = packet.header.Joystick1.y_Axis;
    tmp.joystick1.ButtonState = packet.header.Joystick1.ButtonState;
    tmp.joystick1.Trigger = packet.header.Joystick1.Trigger;
    tmp.joystick2.x_Axis = packet.header.Joystick2.x_Axis;
    tmp.joystick2.y_Axis = packet.header.Joystick2.y_Axis;
    tmp.joystick2.ButtonState = packet.header.Joystick2.ButtonState;
    tmp.joystick2.Trigger = packet.header.Joystick2.Trigger;

    *header_data = tmp;

    memcpy(packet_data, packet.rawData, packet.rawDataSize);

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_hub_getLastStreamingPacket(prio_device_id hub_id, prio_StreamHeaderData *header_data, U8* packet_data)
{
    HUB_RANGE_CHECK();

    PrioStreamPacket packet;
    PRIO_ERROR_CHECK(stored_hubs[hub_id]->getLastStreamingPacket(&packet));

    prio_StreamHeaderData tmp;
    tmp.battery_level = packet.header.BatteryLevel;
    tmp.battery_status = packet.header.BatteryStatus;
    tmp.hub_button = packet.header.HubButton;
    tmp.joystick1.x_Axis = packet.header.Joystick1.x_Axis;
    tmp.joystick1.y_Axis = packet.header.Joystick1.y_Axis;
    tmp.joystick1.ButtonState = packet.header.Joystick1.ButtonState;
    tmp.joystick1.Trigger = packet.header.Joystick1.Trigger;
    tmp.joystick2.x_Axis = packet.header.Joystick2.x_Axis;
    tmp.joystick2.y_Axis = packet.header.Joystick2.y_Axis;
    tmp.joystick2.ButtonState = packet.header.Joystick2.ButtonState;
    tmp.joystick2.Trigger = packet.header.Joystick2.Trigger;

    *header_data = tmp;

    memcpy(packet_data, packet.rawData, packet.rawDataSize);

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_hub_getStreamingPacketsInWaiting(prio_device_id hub_id, U32* in_waiting)
{
    HUB_RANGE_CHECK();

    *in_waiting = stored_hubs[hub_id]->getStreamingPacketsInWaiting();

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_hub_didStreamingOverflow(prio_device_id hub_id, U8* overflow)
{
    HUB_RANGE_CHECK();

    *overflow = stored_hubs[hub_id]->didStreamingOverflow();

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_hub_getStreamUpdateRate(prio_device_id hub_id, float* update_rate)
{
    HUB_RANGE_CHECK();

    *update_rate = stored_hubs[hub_id]->getStreamUpdateRate();

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_hub_startRecordingWired(prio_device_id hub_id, prio_stream_commands data_flags, U32 interval, U32 duration, U32 delay)
{
    HUB_RANGE_CHECK();

    PRIO_ERROR_CHECK(stored_hubs[hub_id]->startRecordingWired(data_flags, interval, duration, delay));

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_hub_stopRecordingWired(prio_device_id hub_id)
{
    HUB_RANGE_CHECK();

    PRIO_ERROR_CHECK(stored_hubs[hub_id]->stopRecordingWired());

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_hub_disableRecordingWireless(prio_device_id hub_id)
{
    HUB_RANGE_CHECK();

    PRIO_ERROR_CHECK(stored_hubs[hub_id]->disableRecordingWireless());

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_hub_setupRecordingOptions(prio_device_id hub_id, U32 buffer_size, bool is_wrapping)
{
    HUB_RANGE_CHECK();

    stored_hubs[hub_id]->setupRecordingOptions(buffer_size, is_wrapping);

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_hub_getMaxRecordedSamples(prio_device_id hub_id, U32 *max_sample_count)
{
    HUB_RANGE_CHECK();

    *max_sample_count = stored_hubs[hub_id]->getMaxRecordedSamples();

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_hub_getRecordingWrappingMode(prio_device_id hub_id, bool* wrapping_enabled)
{
    HUB_RANGE_CHECK();

    *wrapping_enabled = stored_hubs[hub_id]->isBufferWrapping();

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_hub_getLengthOfRecordedSamples(prio_device_id hub_id, U32* sample_count)
{
    HUB_RANGE_CHECK();

    *sample_count = stored_hubs[hub_id]->getLengthOfRecordedSamples();
    
    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_hub_getRecordedSamples(prio_device_id hub_id, PrioHeader *header_data, U8* packet_data, U32 packet_count)
{
    HUB_RANGE_CHECK();

    PRIO_ERROR_CHECK(stored_hubs[hub_id]->getRecordedSamples(header_data, packet_data, packet_count));

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_hub_getRecordedSampleAtIndex(prio_device_id hub_id, PrioHeader *header_data, U8* packet_data, U32 index)
{
    HUB_RANGE_CHECK();

    PRIO_ERROR_CHECK(stored_hubs[hub_id]->getRecordedSampleAtIndex(header_data, packet_data, index));

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_hub_popFrontRecordedSample(prio_device_id hub_id, PrioHeader *header_data, U8* packet_data)
{
    HUB_RANGE_CHECK();

    PRIO_ERROR_CHECK(stored_hubs[hub_id]->popFrontRecordedSample(header_data, packet_data));

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_hub_clearRecordedSamples(prio_device_id hub_id)
{
    HUB_RANGE_CHECK();

    stored_hubs[hub_id]->clearRecordedSamples();

    return PRIO_NO_ERROR;
}


PRIO_EXPORT PRIO_ERROR prio_hub_enumerateHub(prio_device_id hub_id)
{
    HUB_RANGE_CHECK();

    PRIO_ERROR_CHECK(stored_hubs[hub_id]->enumerateHub());

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_hub_getNumEnumeratedSensors(prio_device_id hub_id, U8* num_sensors)
{
    HUB_RANGE_CHECK();

    PRIO_ERROR_CHECK(stored_hubs[hub_id]->getNumberOfEnumeratedSensors(num_sensors));

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_hub_getActiveSensors(prio_device_id hub_id, U8* active_sensors, U8 active_sensors_len, U8* active_len)
{
    HUB_RANGE_CHECK();

    std::vector<U8> active_list = stored_hubs[hub_id]->getActiveSensors();
    for (U8 i = 0; i < active_sensors_len; i++)
    {
        if (i >= active_list.size())
        {
            break;
        }
        active_sensors[i] = active_list[i];
    }

    if (active_len != nullptr)
    {
        *active_len = (uint8_t)active_list.size();
    }

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_hub_getFullLengthOfStreamingData(prio_device_id hub_id, U32* stream_data_len)
{
    HUB_RANGE_CHECK();
    std::vector<U8> active_list = stored_hubs[hub_id]->getActiveSensors();
    *stream_data_len = stored_hubs[hub_id]->getFullLengthOfStreamData();
    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_hub_setCommandRetries(prio_device_id hub_id, U8 retries)
{
    HUB_RANGE_CHECK();

    stored_hubs[hub_id]->setCommandRetries(retries);

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_hub_setStreamingInterval(prio_device_id hub_id, U32 interval)
{
    HUB_RANGE_CHECK();

    PRIO_ERROR_CHECK(stored_hubs[hub_id]->setStreamingInterval(interval));

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_hub_getStreamingInterval(prio_device_id hub_id, U32* interval)
{
    HUB_RANGE_CHECK();

    PRIO_ERROR_CHECK(stored_hubs[hub_id]->getStreamingInterval(interval));

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_hub_resetAllSensors(prio_device_id hub_id)
{
    HUB_RANGE_CHECK();

    PRIO_ERROR_CHECK(stored_hubs[hub_id]->resetAllSensors());

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_hub_powerDownAllSensors(prio_device_id hub_id)
{
    HUB_RANGE_CHECK();

    PRIO_ERROR_CHECK(stored_hubs[hub_id]->powerDownAllSensors());

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_hub_powerUpAllSensors(prio_device_id hub_id)
{
    HUB_RANGE_CHECK();

    PRIO_ERROR_CHECK(stored_hubs[hub_id]->powerUpAllSensors());

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_hub_setAutoEnumerationMode(prio_device_id hub_id, bool enable)
{
    HUB_RANGE_CHECK();

    PRIO_ERROR_CHECK(stored_hubs[hub_id]->setAutoEnumerationMode(enable));

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_hub_getAutoEnumerationMode(prio_device_id hub_id, bool* enabled)
{
    HUB_RANGE_CHECK();

    PRIO_ERROR_CHECK(stored_hubs[hub_id]->getAutoEnumerationMode(enabled));

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_hub_getWirelessAddress(prio_device_id hub_id, U16* address)
{
    HUB_RANGE_CHECK();

    PRIO_ERROR_CHECK(stored_hubs[hub_id]->getWirelessAddress(address));

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_hub_getBatteryStatus(prio_device_id hub_id, U8* status)
{
    HUB_RANGE_CHECK();

    PRIO_ERROR_CHECK(stored_hubs[hub_id]->getBatteryStatus(status));

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_hub_setPerformanceMode(prio_device_id hub_id, bool enable)
{
    HUB_RANGE_CHECK();

    PRIO_ERROR_CHECK(stored_hubs[hub_id]->setPerformanceMode(enable));

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_hub_getPerformanceMode(prio_device_id hub_id, bool* enabled)
{
    HUB_RANGE_CHECK();

    PRIO_ERROR_CHECK(stored_hubs[hub_id]->getPerformanceMode(enabled));

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_hub_getTaredOrientationAsQuaternion(prio_device_id hub_id, U8 logical_id, U8 num_floats, float* quat)
{
    HUB_RANGE_CHECK();

    PRIO_ERROR_CHECK(stored_hubs[hub_id]->getTaredOrientationAsQuaternion(logical_id, quat, num_floats));

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_hub_getUntaredOrientationAsQuaternion(prio_device_id hub_id, U8 logical_id, U8 num_floats, float* quat)
{
    HUB_RANGE_CHECK();

    PRIO_ERROR_CHECK(stored_hubs[hub_id]->getUntaredOrientationAsQuaternion(logical_id, quat, num_floats));

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_hub_getControllerData(prio_device_id hub_id, U8 index, U8* logical_id, U8* data)
{
    HUB_RANGE_CHECK();

    PRIO_ERROR_CHECK(stored_hubs[hub_id]->getControllerData(index, logical_id, data));

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_hub_getAllControllerData(prio_device_id hub_id, U8* logical_id_1, U8* logical_id_2, U8* data_1, U8* data_2, U8* button_state)
{
    HUB_RANGE_CHECK();

    PRIO_ERROR_CHECK(stored_hubs[hub_id]->getAllControllerData(logical_id_1, logical_id_2, data_1, data_2, button_state));

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_hub_getCorrectedSensorData(prio_device_id hub_id, U8 logical_id, float* gyroscope, float* accelerometer, float* magnetometer)
{
    HUB_RANGE_CHECK();

    PRIO_ERROR_CHECK(stored_hubs[hub_id]->getCorrectedSensorData(logical_id, gyroscope, accelerometer, magnetometer));

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_hub_getCorrectedGyroscope(prio_device_id hub_id, U8 logical_id, float* gyroscope)
{
    HUB_RANGE_CHECK();

    PRIO_ERROR_CHECK(stored_hubs[hub_id]->getCorrectedGyroscope(logical_id, gyroscope));

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_hub_getCorrectedAccelerometer(prio_device_id hub_id, U8 logical_id, float* accelerometer)
{
    HUB_RANGE_CHECK();

    PRIO_ERROR_CHECK(stored_hubs[hub_id]->getCorrectedAccelerometer(logical_id, accelerometer));

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_hub_getCorrectedMagnetometer(prio_device_id hub_id, U8 logical_id, float* magnetometer)
{
    HUB_RANGE_CHECK();

    PRIO_ERROR_CHECK(stored_hubs[hub_id]->getCorrectedMagnetometer(logical_id, magnetometer));

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_hub_getRawSensorData(prio_device_id hub_id, U8 logical_id, float* gyroscope, float* accelerometer, float* magnetometer)
{
    HUB_RANGE_CHECK();

    PRIO_ERROR_CHECK(stored_hubs[hub_id]->getRawSensorData(logical_id, gyroscope, accelerometer, magnetometer));

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_hub_getRawGyroscope(prio_device_id hub_id, U8 logical_id, float* gyroscope)
{
    HUB_RANGE_CHECK();

    PRIO_ERROR_CHECK(stored_hubs[hub_id]->getRawGyroscope(logical_id, gyroscope));

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_hub_getRawAccelerometer(prio_device_id hub_id, U8 logical_id, float* accelerometer)
{
    HUB_RANGE_CHECK();

    PRIO_ERROR_CHECK(stored_hubs[hub_id]->getRawAccelerometer(logical_id, accelerometer));

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_hub_getRawMagnetometer(prio_device_id hub_id, U8 logical_id, float* magnetometer)
{
    HUB_RANGE_CHECK();

    PRIO_ERROR_CHECK(stored_hubs[hub_id]->getRawMagnetometer(logical_id, magnetometer));

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_hub_setMagnetometerCalibParams(prio_device_id hub_id, U8 logical_id, const float* scale9, const float* bias3)
{
    HUB_RANGE_CHECK();

    PRIO_ERROR_CHECK(stored_hubs[hub_id]->setMagnetometerCalibParams(logical_id, scale9, bias3));

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_hub_setAccelerometerCalibParams(prio_device_id hub_id, U8 logical_id, const float* scale9, const float* bias3)
{
    HUB_RANGE_CHECK();

    PRIO_ERROR_CHECK(stored_hubs[hub_id]->setAccelerometerCalibParams(logical_id, scale9, bias3));

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_hub_setGyroCalibParams(prio_device_id hub_id, U8 logical_id, const float* scale9, const float* bias3)
{
    HUB_RANGE_CHECK();

    PRIO_ERROR_CHECK(stored_hubs[hub_id]->setGyroCalibParams(logical_id, scale9, bias3));

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_hub_getMagnetometerCalibParams(prio_device_id hub_id, U8 logical_id, float* scale9, float* bias3)
{
    HUB_RANGE_CHECK();

    PRIO_ERROR_CHECK(stored_hubs[hub_id]->getMagnetometerCalibParams(logical_id, scale9, bias3));

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_hub_getAccelerometerCalibParams(prio_device_id hub_id, U8 logical_id, float* scale9, float* bias3)
{
    HUB_RANGE_CHECK();

    PRIO_ERROR_CHECK(stored_hubs[hub_id]->getAccelerometerCalibParams(logical_id, scale9, bias3));

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_hub_getGyroCalibParams(prio_device_id hub_id, U8 logical_id, float* scale9, float* bias3)
{
    HUB_RANGE_CHECK();

    PRIO_ERROR_CHECK(stored_hubs[hub_id]->getGyroCalibParams(logical_id, scale9, bias3));

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_hub_setBetaCalcParams(prio_device_id hub_id, U8 logical_id, float settle_base, float settle_div, float base, float mul)
{
    HUB_RANGE_CHECK();

    PRIO_ERROR_CHECK(stored_hubs[hub_id]->setBetaCalcParams(logical_id, settle_base, settle_div, base, mul));

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_hub_getBetaCalcParams(prio_device_id hub_id, U8 logical_id, float* settle_base, float* settle_div, float* base, float* mul)
{
    HUB_RANGE_CHECK();

    PRIO_ERROR_CHECK(stored_hubs[hub_id]->getBetaCalcParams(logical_id, settle_base, settle_div, base, mul));

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_hub_setAutoCalibEnabled(prio_device_id hub_id, U8 logical_id, U8 enabled)
{
    HUB_RANGE_CHECK();

    PRIO_ERROR_CHECK(stored_hubs[hub_id]->setAutoCalibEnabled(logical_id, enabled));

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_hub_getAutoCalibEnabled(prio_device_id hub_id, U8 logical_id, U8* enabled)
{
    HUB_RANGE_CHECK();

    PRIO_ERROR_CHECK(stored_hubs[hub_id]->getAutoCalibEnabled(logical_id, enabled));

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_hub_commitSensorSettings(prio_device_id hub_id, U8 logical_id)
{
    HUB_RANGE_CHECK();

    PRIO_ERROR_CHECK(stored_hubs[hub_id]->commitSensorSettings(logical_id));

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_hub_tareWithCurrentOrientation(prio_device_id hub_id, U8 logical_id)
{
    HUB_RANGE_CHECK();

    PRIO_ERROR_CHECK(stored_hubs[hub_id]->tareWithCurrentOrientation(logical_id));

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_hub_getWirelessChannel(prio_device_id hub_id, U8* channel)
{
    HUB_RANGE_CHECK();

    PRIO_ERROR_CHECK(stored_hubs[hub_id]->getWirelessChannel(channel));

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_hub_setWirelessChannel(prio_device_id hub_id, U8 channel)
{
    HUB_RANGE_CHECK();

    PRIO_ERROR_CHECK(stored_hubs[hub_id]->setWirelessChannel(channel));

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_hub_getWirelessPanID(prio_device_id hub_id, U16* pan_id)
{
    HUB_RANGE_CHECK();

    PRIO_ERROR_CHECK(stored_hubs[hub_id]->getWirelessPanID(pan_id));

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_hub_setWirelessPanID(prio_device_id hub_id, U16 pan_id)
{
    HUB_RANGE_CHECK();

    PRIO_ERROR_CHECK(stored_hubs[hub_id]->setWirelessPanID(pan_id));

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_hub_getHubBatteryLevel(prio_device_id hub_id, U8* battery_level)
{
    HUB_RANGE_CHECK();

    PRIO_ERROR_CHECK(stored_hubs[hub_id]->getBatteryPercentRemaining(battery_level));

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_hub_getHubButtonState(prio_device_id hub_id, U8* button_state)
{
    HUB_RANGE_CHECK();

    PRIO_ERROR_CHECK(stored_hubs[hub_id]->getButtonState(button_state));

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_hub_commitSettings(prio_device_id hub_id)
{
    HUB_RANGE_CHECK();

    PRIO_ERROR_CHECK(stored_hubs[hub_id]->commitSettings());

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_hub_getSerialNumberAtLogicalID(prio_device_id hub_id, U8 logical_id, U32* serial_number)
{
    HUB_RANGE_CHECK();

    PRIO_ERROR_CHECK(stored_hubs[hub_id]->getSerialNumberAtLogicalID(logical_id, serial_number));
    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_hub_getFirmwareVersionAtLogicalID(prio_device_id hub_id, U8 logical_id, const char* firmware_version)
{
    HUB_RANGE_CHECK();

    PRIO_ERROR_CHECK(stored_hubs[hub_id]->getFirmwareVersionAtLogicalID(logical_id, firmware_version));

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_hub_getHardwareVersionAtLogicalID(prio_device_id hub_id, U8 logical_id, const char* hardware_version)
{
    HUB_RANGE_CHECK();

    PRIO_ERROR_CHECK(stored_hubs[hub_id]->getHardwareVersionAtLogicalID(logical_id, hardware_version));

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_createBaseStation(const char* port_name, prio_device_id* out_id)
{
    PrioBaseStation* dongle = new PrioBaseStation(port_name);

    if (!dongle->isConnected())
    {
        delete dongle;
        return PRIO_ERROR_COMMAND_FAIL;
    }

    *out_id = stored_bs.size();
    stored_bs.push_back(unique_ptr<PrioBaseStation>(dongle));

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_removeBaseStation(prio_device_id bs_id)
{
    BS_RANGE_CHECK();

    stored_bs[bs_id].reset();

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_bs_isConnected(prio_device_id bs_id)
{
    BS_RANGE_CHECK();

    if (!stored_bs[bs_id]->isConnected())
    {
        return PRIO_ERROR_COMMAND_FAIL;
    }

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_bs_getLastBasestationTimestamp(prio_device_id bs_id, U32* timestamp)
{
    BS_RANGE_CHECK();

    *timestamp = stored_bs[bs_id]->getLastHeader()->SensorTimestamp;

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_bs_getLastSystemTimestamp(prio_device_id bs_id, float* timestamp)
{
    BS_RANGE_CHECK();

    *timestamp = stored_bs[bs_id]->getLastHeader()->SystemTimestamp;

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_bs_getSerialNumber(prio_device_id bs_id, U32* serial_number)
{
    BS_RANGE_CHECK();

    *serial_number = stored_bs[bs_id]->getSerialNumber();

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_bs_openPort(prio_device_id bs_id, const char* port_name)
{
    BS_RANGE_CHECK();

    PRIO_ERROR_CHECK(stored_bs[bs_id]->openPort(port_name));

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_bs_closePort(prio_device_id bs_id)
{
    BS_RANGE_CHECK();

    PRIO_ERROR_CHECK(stored_bs[bs_id]->closePort());

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_bs_enableTimestampsWireless(prio_device_id bs_id)
{
    BS_RANGE_CHECK();

    PRIO_ERROR_CHECK(stored_bs[bs_id]->enableTimestampsWireless());

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_bs_disableTimestampsWireless(prio_device_id bs_id)
{
    BS_RANGE_CHECK();

    PRIO_ERROR_CHECK(stored_bs[bs_id]->disableTimestampsWireless());

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_bs_getWirelessHub(prio_device_id bs_id, prio_device_id* out_id)
{
    BS_RANGE_CHECK();

    shared_ptr<PrioHub> sensor;

    if (stored_bs[bs_id]->_childCount != 0)
    {
        sensor = stored_bs[bs_id]->_children[0];

        *out_id = std::find(stored_hubs.begin(), stored_hubs.end(), sensor) - stored_hubs.begin();

        return PRIO_NO_ERROR;
    }

    PRIO_ERROR_CHECK(stored_bs[bs_id]->getPairedHub(sensor));

    if (!sensor->isConnected())
    {
        return PRIO_ERROR_COMMAND_FAIL;
    }

    *out_id = stored_hubs.size();
    stored_hubs.push_back(shared_ptr<PrioHub>(sensor));

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_bs_autoPairWithHub(prio_device_id bs_id)
{
    BS_RANGE_CHECK();

    shared_ptr<PrioHub> sensor;
    prio_device_id hub_id;

    if (stored_bs[bs_id]->_childCount != 0)
    {
        sensor = stored_bs[bs_id]->_children[0];

        hub_id = std::find(stored_hubs.begin(), stored_hubs.end(), sensor) - stored_hubs.begin();
    }

    PRIO_ERROR_CHECK(stored_bs[bs_id]->autoPairBaseStationWithHub());
    yost::sleep_ms(2500); //Delay to allow the device time to complete pairing handshake
    PRIO_ERROR_CHECK(stored_bs[bs_id]->getPairedHub(sensor))

    stored_hubs[hub_id] = sensor;

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_bs_startStreaming(prio_device_id bs_id, U32 data_flags, U32 interval, U32 duration)
{
    BS_RANGE_CHECK();

    if(stored_bs[bs_id]->_isStreaming)
    {
        return PRIO_ERROR_COMMAND_FAIL;
    }
    PRIO_ERROR_CHECK(stored_bs[bs_id]->startStreaming(data_flags, interval, duration));

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_bs_stopStreaming(prio_device_id bs_id)
{
    BS_RANGE_CHECK();
    
    PRIO_ERROR_CHECK(stored_bs[bs_id]->stopStreaming());

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_bs_startRecording(prio_device_id bs_id, U32 data_flags, U32 interval, U32 duration)
{
    BS_RANGE_CHECK();
    if (stored_bs[bs_id]->_isStreaming || stored_bs[bs_id]->_isRecording)
    {
        return PRIO_ERROR_COMMAND_FAIL;
    }
    PRIO_ERROR_CHECK(stored_bs[bs_id]->startRecording(data_flags, interval, duration));

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_bs_stopRecording(prio_device_id bs_id)
{
    BS_RANGE_CHECK();

    PRIO_ERROR_CHECK(stored_bs[bs_id]->stopRecording());

    return PRIO_NO_ERROR;
}


PRIO_EXPORT PRIO_ERROR prio_bs_getWirelessChannel(prio_device_id bs_id, U8* channel)
{
    BS_RANGE_CHECK();

    PRIO_ERROR_CHECK(stored_bs[bs_id]->getWirelessChannel(channel));

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_bs_setWirelessChannel(prio_device_id bs_id, U8 channel)
{
    BS_RANGE_CHECK();

    PRIO_ERROR_CHECK(stored_bs[bs_id]->setWirelessChannel(channel));

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_bs_getWirelessPanID(prio_device_id bs_id, U16* pan_id)
{
    BS_RANGE_CHECK();

    PRIO_ERROR_CHECK(stored_bs[bs_id]->getWirelessPanID(pan_id));

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_bs_setWirelessPanID(prio_device_id bs_id, U16 pan_id)
{
    BS_RANGE_CHECK();

    PRIO_ERROR_CHECK(stored_bs[bs_id]->setWirelessPanID(pan_id));

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_bs_getSerialNumberAtLogicalID(prio_device_id bs_id, U8 logical_id, U32* serial_number)
{
    BS_RANGE_CHECK();

    PRIO_ERROR_CHECK(stored_bs[bs_id]->getSerialNumberAtLogicalID(logical_id, serial_number));

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_bs_getFirmwareVersionAtLogicalID(prio_device_id bs_id, U8 logical_id, const char* firmware_version)
{
    BS_RANGE_CHECK();

    PRIO_ERROR_CHECK(stored_bs[bs_id]->getFirmwareVersionAtLogicalID(logical_id, firmware_version));

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_bs_getHardwareVersionAtLogicalID(prio_device_id bs_id, U8 logical_id, const char* hardware_version)
{
    BS_RANGE_CHECK();

    PRIO_ERROR_CHECK(stored_bs[bs_id]->getHardwareVersionAtLogicalID(logical_id, hardware_version));

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_bs_commitSettings(prio_device_id bs_id)
{
    BS_RANGE_CHECK();

    PRIO_ERROR_CHECK(stored_bs[bs_id]->commitSettings());

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_bs_getHardwareVersion(prio_device_id bs_id, const char* hardware_version)
{
    BS_RANGE_CHECK();

    PRIO_ERROR_CHECK(stored_bs[bs_id]->getHardwareVersion(hardware_version));

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_bs_getFirmwareVersion(prio_device_id bs_id, const char* firmware_version)
{
    BS_RANGE_CHECK();

    PRIO_ERROR_CHECK(stored_bs[bs_id]->getFirmwareVersion(firmware_version));

    return PRIO_NO_ERROR;
}

PRIO_EXPORT void prio_findPorts(U32 find_flags)
{
    stored_prio_ports = prioAPIFindPorts(find_flags);

    prio_stored_port_index = 0;
}

PRIO_EXPORT PRIO_ERROR prio_getNextPort(char* port_name, U8* sensor_type)
{
    if (prio_stored_port_index >= stored_prio_ports.size())
        return PRIO_ERROR_MEMORY;

    memcpy(port_name, stored_prio_ports[prio_stored_port_index].port_name, 64 );

    U8 type = stored_prio_ports[prio_stored_port_index].device_type; 
    *sensor_type = type;

    prio_stored_port_index++;

    return PRIO_NO_ERROR;
}

PRIO_EXPORT PRIO_ERROR prio_getPort(char* port_name, U32 index, U8* sensor_type)
{
    if (index >= stored_prio_ports.size())
        return PRIO_ERROR_MEMORY;
    memcpy(port_name, stored_prio_ports[index].port_name, 64);

    U8 type = stored_prio_ports[index].device_type;
    *sensor_type = type;

    return PRIO_NO_ERROR;
}