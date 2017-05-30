#include "threespace_api_export.h"
#include "threespace_api.hpp"

vector<shared_ptr<TssSensor>> stored_sensors;
vector<shared_ptr<TssDongle>> stored_dongles;
vector<TssComPort> stored_tss_ports;
U8 stored_port_index = 0;

#define SENSOR_RANGE_CHECK() if (sensor_id >= stored_sensors.size() || !stored_sensors[sensor_id]){return TSS_ERROR_INVALID_ID;}
#define DONGLE_RANGE_CHECK() if (dongle_id >= stored_dongles.size() || !stored_dongles[dongle_id]){return TSS_ERROR_INVALID_ID;}

TSS_EXPORT TSS_RESULT tss_initAPI()
{
    gAPI.init();

    return TSS_SUCCESS;
}

TSS_EXPORT TSS_RESULT tss_deinitAPI()
{
    gAPI.deinit();

    return TSS_SUCCESS;
}

TSS_EXPORT TSS_RESULT tss_createSensor(const char* port_name, tss_device_id* out_id)
{
    TssSensor* sensor = new TssSensor(port_name);

    if (!sensor->isConnected())
    {
        delete sensor;
        return TSS_ERROR_CANT_OPEN_PORT;
    }

    *out_id = stored_sensors.size();
    stored_sensors.push_back(shared_ptr<TssSensor>(sensor));

    return TSS_SUCCESS;
}

TSS_EXPORT TSS_RESULT tss_removeSensor(tss_device_id sensor_id)
{
    SENSOR_RANGE_CHECK();

    stored_sensors[sensor_id].reset();

    return TSS_SUCCESS;
}

TSS_EXPORT TSS_RESULT tss_sensor_isConnected(tss_device_id sensor_id)
{
    SENSOR_RANGE_CHECK();

    if (!stored_sensors[sensor_id]->isConnected())
    {
        return TSS_ERROR_NOT_CONNECTED;
    }

    return TSS_SUCCESS;
}

TSS_EXPORT TSS_RESULT tss_sensor_getLastSensorTimestamp(tss_device_id sensor_id, U32* timestamp)
{
    SENSOR_RANGE_CHECK();

    *timestamp = stored_sensors[sensor_id]->getLastHeader()->SensorTimestamp;

    return TSS_SUCCESS;
}

TSS_EXPORT TSS_RESULT tss_sensor_getLastSystemTimestamp(tss_device_id sensor_id, float* timestamp)
{
    SENSOR_RANGE_CHECK();

    *timestamp = stored_sensors[sensor_id]->getLastHeader()->SystemTimestamp;

    return TSS_SUCCESS;
}

TSS_EXPORT TSS_RESULT tss_sensor_getSerialNumber(tss_device_id sensor_id, U32* serial_number)
{
    SENSOR_RANGE_CHECK();

    *serial_number = stored_sensors[sensor_id]->getSerialNumber();

    return TSS_SUCCESS;
}

TSS_EXPORT TSS_RESULT tss_sensor_getSensorType(tss_device_id sensor_id, U8* sensor_type)
{
    SENSOR_RANGE_CHECK();

    *sensor_type = stored_sensors[sensor_id]->getSensorType();

    return TSS_SUCCESS;
}

TSS_EXPORT TSS_RESULT tss_sensor_openPort(tss_device_id sensor_id, const char* port_name)
{
    SENSOR_RANGE_CHECK();

    TSS_ERROR_CHECK(stored_sensors[sensor_id]->openPort(port_name));

    return TSS_SUCCESS;
}

TSS_EXPORT TSS_RESULT tss_sensor_closePort(tss_device_id sensor_id)
{
    SENSOR_RANGE_CHECK();

    TSS_ERROR_CHECK(stored_sensors[sensor_id]->closePort());

    return TSS_SUCCESS;
}

TSS_EXPORT TSS_RESULT tss_sensor_enableTimestampsWired(tss_device_id sensor_id)
{
    SENSOR_RANGE_CHECK();

    TSS_ERROR_CHECK(stored_sensors[sensor_id]->enableTimestampsWired());

    return TSS_SUCCESS;
}

TSS_EXPORT TSS_RESULT tss_sensor_disableTimestampsWired(tss_device_id sensor_id)
{
    SENSOR_RANGE_CHECK();

    TSS_ERROR_CHECK(stored_sensors[sensor_id]->disableTimestampsWired());

    return TSS_SUCCESS;
}

TSS_EXPORT TSS_RESULT tss_sensor_enableStreamingWireless(tss_device_id sensor_id, U32 data_flags, U32 interval, U32 duration, U32 delay)
{
    SENSOR_RANGE_CHECK();

    TSS_ERROR_CHECK(stored_sensors[sensor_id]->enableStreamingWireless(data_flags, interval, duration, delay));

    return TSS_SUCCESS;
}

TSS_EXPORT TSS_RESULT tss_sensor_disableStreamingWireless(tss_device_id sensor_id)
{
    SENSOR_RANGE_CHECK();

    TSS_ERROR_CHECK(stored_sensors[sensor_id]->disableStreamingWireless());

    return TSS_SUCCESS;
}

TSS_EXPORT TSS_RESULT tss_sensor_startStreamingWired(tss_device_id sensor_id, U32 data_flags, U32 interval, U32 duration, U32 delay)
{
    SENSOR_RANGE_CHECK();

    return stored_sensors[sensor_id]->startStreamingWired(data_flags, interval, duration, delay);
}

TSS_EXPORT TSS_RESULT tss_sensor_getFirstStreamingPacket(tss_device_id sensor_id, U8* packet_data)
{
    SENSOR_RANGE_CHECK();

    TssStreamPacket packet;
    TSS_ERROR_CHECK(stored_sensors[sensor_id]->getFirstStreamingPacket(&packet));

    memcpy(packet_data, packet.rawData, packet.rawDataSize);

    return TSS_SUCCESS;
}

TSS_EXPORT TSS_RESULT tss_sensor_getLastStreamingPacket(tss_device_id sensor_id, U8* packet_data)
{
    SENSOR_RANGE_CHECK();

    TssStreamPacket packet;
    TSS_ERROR_CHECK(stored_sensors[sensor_id]->getLastStreamingPacket(&packet));

    memcpy(packet_data, packet.rawData, packet.rawDataSize);

    return TSS_SUCCESS;
}

TSS_EXPORT TSS_RESULT tss_sensor_stopStreamingWired(tss_device_id sensor_id)
{
    SENSOR_RANGE_CHECK();

    return stored_sensors[sensor_id]->stopStreamingWired();
}

TSS_EXPORT TSS_RESULT tss_sensor_getStreamingPacketsInWaiting(tss_device_id sensor_id, U32* in_waiting)
{
    SENSOR_RANGE_CHECK();

    *in_waiting = stored_sensors[sensor_id]->getStreamingPacketsInWaiting();

    return TSS_SUCCESS;
}

TSS_EXPORT TSS_RESULT tss_sensor_didStreamingOverflow(tss_device_id sensor_id, U8* overflow)
{
    SENSOR_RANGE_CHECK();

    *overflow = stored_sensors[sensor_id]->didStreamingOverflow();

    return TSS_SUCCESS;
}

TSS_EXPORT TSS_RESULT tss_sensor_setCommandRetries(tss_device_id sensor_id, U8 retries)
{
    SENSOR_RANGE_CHECK();

    stored_sensors[sensor_id]->setCommandRetries(retries);

    return TSS_SUCCESS;
}

TSS_EXPORT TSS_RESULT tss_sensor_getTaredOrientationAsQuaternion(tss_device_id sensor_id, float* quat)
{
    SENSOR_RANGE_CHECK();

    Orient vquat;
    TSS_ERROR_CHECK(stored_sensors[sensor_id]->getTaredOrientationAsQuaternion(&vquat));

    memcpy(quat, vquat.data, sizeof(float) * 4);

    return TSS_SUCCESS;
}

TSS_EXPORT TSS_RESULT tss_sensor_getTaredOrientationAsEulerAngles(tss_device_id sensor_id, float* euler)
{
    SENSOR_RANGE_CHECK();

    TSS_ERROR_CHECK(stored_sensors[sensor_id]->getTaredOrientationAsEulerAngles(euler));

    return TSS_SUCCESS;
}

TSS_EXPORT TSS_RESULT tss_sensor_getTaredOrientationAsRotationMatrix(tss_device_id sensor_id, float* mat)
{
    SENSOR_RANGE_CHECK();

    Matrix3x3 vmat;
    TSS_ERROR_CHECK(stored_sensors[sensor_id]->getTaredOrientationAsRotationMatrix(&vmat));

    memcpy(mat, vmat.data, sizeof(float) * 9);

    return TSS_SUCCESS;
}

TSS_EXPORT TSS_RESULT tss_sensor_getTaredOrientationAsAxisAngle(tss_device_id sensor_id, float* vec, float* angle)
{
    SENSOR_RANGE_CHECK();

    Vector3 vvec;
    TSS_ERROR_CHECK(stored_sensors[sensor_id]->getTaredOrientationAsAxisAngle(&vvec, angle));

    memcpy(vec, vvec.data, sizeof(float) * 3);

    return TSS_SUCCESS;
}

TSS_EXPORT TSS_RESULT tss_sensor_getTaredOrientationAsTwoVector(tss_device_id sensor_id, float* forward, float* down)
{
    SENSOR_RANGE_CHECK();

    Vector3 vforward;
    Vector3 vdown;
    TSS_ERROR_CHECK(stored_sensors[sensor_id]->getTaredOrientationAsTwoVector(&vforward, &vdown));

    memcpy(forward, vforward.data, sizeof(float) * 3);
    memcpy(down, vdown.data, sizeof(float) * 3);

    return TSS_SUCCESS;
}

TSS_EXPORT TSS_RESULT tss_sensor_getUntaredOrientationAsQuaternion(tss_device_id sensor_id, float* orient_quat)
{
    SENSOR_RANGE_CHECK();

    Orient vquat;
    TSS_ERROR_CHECK(stored_sensors[sensor_id]->getUntaredOrientationAsQuaternion(&vquat));

    memcpy(orient_quat, vquat.data, sizeof(float) * 4);

    return TSS_SUCCESS;
}

TSS_EXPORT TSS_RESULT tss_sensor_getUntaredOrientationAsEulerAngles(tss_device_id sensor_id, float* euler)
{
    SENSOR_RANGE_CHECK();

    TSS_ERROR_CHECK(stored_sensors[sensor_id]->getUntaredOrientationAsEulerAngles(euler));

    return TSS_SUCCESS;
}

TSS_EXPORT TSS_RESULT tss_sensor_getUntaredOrientationAsRotationMatrix(tss_device_id sensor_id, float* mat)
{
    SENSOR_RANGE_CHECK();

    Matrix3x3 vmat;
    TSS_ERROR_CHECK(stored_sensors[sensor_id]->getUntaredOrientationAsRotationMatrix(&vmat));

    memcpy(mat, vmat.data, sizeof(float) * 9);

    return TSS_SUCCESS;
}

TSS_EXPORT TSS_RESULT tss_sensor_getUntaredOrientationAsAxisAngle(tss_device_id sensor_id, float* vec, float* angle)
{
    SENSOR_RANGE_CHECK();

    Vector3 vvec;
    TSS_ERROR_CHECK(stored_sensors[sensor_id]->getUntaredOrientationAsAxisAngle(&vvec, angle));

    memcpy(vec, vvec.data, sizeof(float) * 3);

    return TSS_SUCCESS;
}

TSS_EXPORT TSS_RESULT tss_sensor_getUntaredOrientationAsTwoVector(tss_device_id sensor_id, float* north, float* gravity)
{
    SENSOR_RANGE_CHECK();

    Vector3 vnorth;
    Vector3 vgravity;
    TSS_ERROR_CHECK(stored_sensors[sensor_id]->getUntaredOrientationAsTwoVector(&vnorth, &vgravity));

    memcpy(north, vnorth.data, sizeof(float) * 3);
    memcpy(gravity, vgravity.data, sizeof(float) * 3);

    return TSS_SUCCESS;
}

TSS_EXPORT TSS_RESULT tss_sensor_tareWithCurrentOrientation(tss_device_id sensor_id)
{
    SENSOR_RANGE_CHECK();

    TSS_ERROR_CHECK(stored_sensors[sensor_id]->tareWithCurrentOrientation());

    return TSS_SUCCESS;
}

TSS_EXPORT TSS_RESULT tss_sensor_getWirelessChannel(tss_device_id sensor_id, U8* channel)
{
    SENSOR_RANGE_CHECK();

    TSS_ERROR_CHECK(stored_sensors[sensor_id]->getWirelessChannel(channel));

    return TSS_SUCCESS;
}

TSS_EXPORT TSS_RESULT tss_sensor_setWirelessChannel(tss_device_id sensor_id, U8 channel)
{
    SENSOR_RANGE_CHECK();

    TSS_ERROR_CHECK(stored_sensors[sensor_id]->setWirelessChannel(channel));

    return TSS_SUCCESS;
}

TSS_EXPORT TSS_RESULT tss_sensor_getWirelessPanID(tss_device_id sensor_id, U16* pan_id)
{
    SENSOR_RANGE_CHECK();

    TSS_ERROR_CHECK(stored_sensors[sensor_id]->getWirelessPanID(pan_id));

    return TSS_SUCCESS;
}

TSS_EXPORT TSS_RESULT tss_sensor_setWirelessPanID(tss_device_id sensor_id, U16 pan_id)
{
    SENSOR_RANGE_CHECK();

    TSS_ERROR_CHECK(stored_sensors[sensor_id]->setWirelessPanID(pan_id));

    return TSS_SUCCESS;
}

TSS_EXPORT TSS_RESULT tss_sensor_commitSettings(tss_device_id sensor_id)
{
    SENSOR_RANGE_CHECK();

    TSS_ERROR_CHECK(stored_sensors[sensor_id]->commitSettings());

    return TSS_SUCCESS;
}

TSS_EXPORT TSS_RESULT tss_sensor_commitWirelessSettings(tss_device_id sensor_id)
{
    SENSOR_RANGE_CHECK();

    TSS_ERROR_CHECK(stored_sensors[sensor_id]->commitWirelessSettings());

    return TSS_SUCCESS;
}

TSS_EXPORT TSS_RESULT tss_createDongle(const char* port_name, tss_device_id* out_id)
{
    TssDongle* dongle = new TssDongle(port_name);

    if (!dongle->isConnected())
    {
        delete dongle;
        return TSS_ERROR_CANT_OPEN_PORT;
    }

    *out_id = stored_dongles.size();
    stored_dongles.push_back(unique_ptr<TssDongle>(dongle));

    return TSS_SUCCESS;
}

TSS_EXPORT TSS_RESULT tss_removeDongle(tss_device_id dongle_id)
{
    DONGLE_RANGE_CHECK();

    stored_dongles[dongle_id].reset();

    return TSS_SUCCESS;
}

TSS_EXPORT TSS_RESULT tss_dongle_isConnected(tss_device_id dongle_id)
{
    DONGLE_RANGE_CHECK();

    if (!stored_dongles[dongle_id]->isConnected())
    {
        return TSS_ERROR_NOT_CONNECTED;
    }

    return TSS_SUCCESS;
}

TSS_EXPORT TSS_RESULT tss_dongle_getLastdongleTimestamp(tss_device_id dongle_id, U32* timestamp)
{
    DONGLE_RANGE_CHECK();

    *timestamp = stored_dongles[dongle_id]->getLastHeader()->SensorTimestamp;

    return TSS_SUCCESS;
}

TSS_EXPORT TSS_RESULT tss_dongle_getLastSystemTimestamp(tss_device_id dongle_id, float* timestamp)
{
    DONGLE_RANGE_CHECK();

    *timestamp = stored_dongles[dongle_id]->getLastHeader()->SystemTimestamp;

    return TSS_SUCCESS;
}

TSS_EXPORT TSS_RESULT tss_dongle_getSerialNumber(tss_device_id dongle_id, U32* serial_number)
{
    DONGLE_RANGE_CHECK();

    *serial_number = stored_dongles[dongle_id]->getSerialNumber();

    return TSS_SUCCESS;
}

TSS_EXPORT TSS_RESULT tss_dongle_openPort(tss_device_id dongle_id, const char* port_name)
{
    DONGLE_RANGE_CHECK();

    TSS_ERROR_CHECK(stored_dongles[dongle_id]->openPort(port_name));

    return TSS_SUCCESS;
}

TSS_EXPORT TSS_RESULT tss_dongle_closePort(tss_device_id dongle_id)
{
    DONGLE_RANGE_CHECK();

    TSS_ERROR_CHECK(stored_dongles[dongle_id]->closePort());

    return TSS_SUCCESS;
}

TSS_EXPORT TSS_RESULT tss_dongle_enableTimestampsWireless(tss_device_id dongle_id)
{
    DONGLE_RANGE_CHECK();

    TSS_ERROR_CHECK(stored_dongles[dongle_id]->enableTimestampsWireless());

    return TSS_SUCCESS;
}

TSS_EXPORT TSS_RESULT tss_dongle_disableTimestampsWireless(tss_device_id dongle_id)
{
    DONGLE_RANGE_CHECK();

    TSS_ERROR_CHECK(stored_dongles[dongle_id]->disableTimestampsWireless());

    return TSS_SUCCESS;
}

TSS_EXPORT TSS_RESULT tss_dongle_getWirelessSensor(tss_device_id dongle_id, U8 logical_id, tss_device_id* out_id)
{
    DONGLE_RANGE_CHECK();

    shared_ptr<TssSensor> sensor;

    TSS_ERROR_CHECK(stored_dongles[dongle_id]->getWirelessSensor(logical_id, sensor));

    if (!sensor->isConnected())
    {
        return TSS_ERROR_NOT_CONNECTED;
    }

    *out_id = stored_sensors.size();
    stored_sensors.push_back(shared_ptr<TssSensor>(sensor));

    return TSS_SUCCESS;
}

TSS_EXPORT TSS_RESULT tss_dongle_removeWirelessSensor(tss_device_id dongle_id, U8 logical_id)
{
    DONGLE_RANGE_CHECK();

    TSS_ERROR_CHECK(stored_dongles[dongle_id]->removeWirelessSensor(logical_id));

    return TSS_SUCCESS;
}

TSS_EXPORT TSS_RESULT tss_dongle_enableAllSensorsAndStartStreaming(tss_device_id dongle_id, U32 data_flags, U32 interval, U32 duration)
{
    DONGLE_RANGE_CHECK();

    TSS_ERROR_CHECK(stored_dongles[dongle_id]->enableAllSensorsAndStartStreaming(data_flags, interval, duration));

    return TSS_SUCCESS;
}

TSS_EXPORT TSS_RESULT tss_dongle_startStreaming(tss_device_id dongle_id)
{
    DONGLE_RANGE_CHECK();

    TSS_ERROR_CHECK(stored_dongles[dongle_id]->startStreaming());

    return TSS_SUCCESS;
}

TSS_EXPORT TSS_RESULT tss_dongle_stopStreaming(tss_device_id dongle_id)
{
    DONGLE_RANGE_CHECK();

    TSS_ERROR_CHECK(stored_dongles[dongle_id]->stopStreaming());

    return TSS_SUCCESS;
}

TSS_EXPORT TSS_RESULT tss_dongle_getWirelessChannel(tss_device_id dongle_id, U8* channel)
{
    DONGLE_RANGE_CHECK();

    TSS_ERROR_CHECK(stored_dongles[dongle_id]->getWirelessChannel(channel));

    return TSS_SUCCESS;
}

TSS_EXPORT TSS_RESULT tss_dongle_setWirelessChannel(tss_device_id dongle_id, U8 channel)
{
    DONGLE_RANGE_CHECK();

    TSS_ERROR_CHECK(stored_dongles[dongle_id]->setWirelessChannel(channel));

    return TSS_SUCCESS;
}

TSS_EXPORT TSS_RESULT tss_dongle_getWirelessPanID(tss_device_id dongle_id, U16* pan_id)
{
    DONGLE_RANGE_CHECK();

    TSS_ERROR_CHECK(stored_dongles[dongle_id]->getWirelessPanID(pan_id));

    return TSS_SUCCESS;
}

TSS_EXPORT TSS_RESULT tss_dongle_setWirelessPanID(tss_device_id dongle_id, U16 pan_id)
{
    DONGLE_RANGE_CHECK();

    TSS_ERROR_CHECK(stored_dongles[dongle_id]->setWirelessPanID(pan_id));

    return TSS_SUCCESS;
}

TSS_EXPORT TSS_RESULT tss_dongle_getSerialNumberAtLogicalID(tss_device_id dongle_id, U8 logical_id, U32* serial_number)
{
    DONGLE_RANGE_CHECK();

    TSS_ERROR_CHECK(stored_dongles[dongle_id]->getSerialNumberAtLogicalID(logical_id, serial_number));

    return TSS_SUCCESS;
}

TSS_EXPORT TSS_RESULT tss_dongle_setSerialNumberAtLogicalID(tss_device_id dongle_id, U8 logical_id, U32 serial_number)
{
    DONGLE_RANGE_CHECK();

    TSS_ERROR_CHECK(stored_dongles[dongle_id]->setSerialNumberAtLogicalID(logical_id, serial_number));

    return TSS_SUCCESS;
}

TSS_EXPORT TSS_RESULT tss_dongle_commitWirelessSettings(tss_device_id dongle_id)
{
    DONGLE_RANGE_CHECK();

    TSS_ERROR_CHECK(stored_dongles[dongle_id]->commitWirelessSettings());

    return TSS_SUCCESS;
}

void tss_findSensorPorts(U32 find_flags)
{
    stored_tss_ports = tssFindSensorPorts(find_flags);

    stored_port_index = 0;
}

U32 tss_getNextSensorPort(char* port_name, U8* sensor_type)
{
    if (stored_port_index >= stored_tss_ports.size())
        return 0;

    memcpy(port_name, stored_tss_ports[stored_port_index].port_name.c_str(), stored_tss_ports[stored_port_index].port_name.size());
    *sensor_type = stored_tss_ports[stored_port_index].device_type;

    stored_port_index++;

    return 1;
}