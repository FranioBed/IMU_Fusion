#include "threespace_sensor.hpp"
#include "threespace_api.hpp"

TssSensor::TssSensor()
{
    _type = TSS_DEVICE_TYPE_SENSOR;

    _ownerDongle = NULL;
    _streamInterval = 0;
    _streamDataSize = 0;
}

TssSensor::TssSensor(std::string port)
{
    _type = TSS_DEVICE_TYPE_SENSOR;

    _ownerDongle = NULL;
    _streamInterval = 0;
    _streamDataSize = 0;

    result = _openPort(port);

    if (result != TSS_SUCCESS)
    {
        closePort();
        return;
    }

    _sendCommand(86); //stop streaming

    result = _readSerialNumber();

    if (result != TSS_SUCCESS)
    {
        closePort();
        return;
    }

    result = _readVersionString();

    if (result != TSS_SUCCESS)
    {
        closePort();
        return;
    }

    if (_sensorType == TSS_DNG)
    {
        closePort();
        return;
    }
}

TssSensor::TssSensor(const TssSensor& other)
{
    throw runtime_error("Deep copies of sensor objects not allowed.");
}

void TssSensor::operator =(const TssSensor& other)
{
    throw runtime_error("Deep copies of sensor objects not allowed.");
}

TSS_RESULT TssSensor::enableStreamingWireless(U32 data_flags, U32 interval, U32 duration, U32 delay)
{
    if (!_isWireless)
    {
        return TSS_ERROR_WIRELESS_ONLY;
    }

    BASIC_CALL_CHECK_TSS();

    return _prepareStreaming(data_flags, interval, duration, delay);
}

TSS_RESULT TssSensor::disableStreamingWireless()
{
    if (!_isWireless)
    {
        return TSS_ERROR_WIRELESS_ONLY;
    }

    BASIC_CALL_CHECK_TSS();

    _streamDataSize = 0; //the only variable the dongle checks to see if a sensor wants to stream, set to 0 to exclude it

    return TSS_SUCCESS;
}

void TssSensor::_addToStreamingSlots(U8 command, U8 nbytes, U8* command_buff, U8& curr_slot)
{
    if (curr_slot < TSS_NUM_STREAMING_SLOTS) //only add command to a slot if slots remain
    {
        command_buff[curr_slot + 1] = command;
        _streamingSlots[curr_slot] = command;
        _streamDataSize += nbytes;
        curr_slot++;
    }
}

TSS_RESULT TssSensor::_prepareStreaming(U32 data_flags, U32 interval, U32 duration, U32 delay)
{
    if (_isStreaming)
    {
        return TSS_ERROR_ALREADY_STREAMING;
    }


    U8 curr_slot = 0;
    U8 buff[256];


    //initialize streaming slots based on data flags
    //also initialize the streaming data size so mass reads can be performed
    _streamDataSize = 0;
    buff[0] = 80; //set streaming slots command

    //go through the flags one by one in order and put them in slots
    //we rely on the numerical values of the flags to determine what order
    //they are assigned to slots in
    if (data_flags & TSS_STREAM_TARED_ORIENTATION_AS_QUATERNION)
    {
        _addToStreamingSlots(0x00, 16, buff, curr_slot);
    }
    if (data_flags & TSS_STREAM_TARED_ORIENTATION_AS_EULER_ANGLES)
    {
        _addToStreamingSlots(0x01, 12, buff, curr_slot);
    }
    if (data_flags & TSS_STREAM_TARED_ORIENTATION_AS_ROTATION_MATRIX)
    {
        _addToStreamingSlots(0x02, 36, buff, curr_slot);
    }
    if (data_flags & TSS_STREAM_TARED_ORIENTATION_AS_AXIS_ANGLE)
    {
        _addToStreamingSlots(0x03, 16, buff, curr_slot);
    }
    if (data_flags & TSS_STREAM_TARED_ORIENTATION_AS_TWO_VECTOR)
    {
        _addToStreamingSlots(0x04, 24, buff, curr_slot);
    }
    if (data_flags & TSS_STREAM_UNTARED_ORIENTATION_AS_QUATERNION)
    {
        _addToStreamingSlots(0x06, 16, buff, curr_slot);
    }
    if (data_flags & TSS_STREAM_UNTARED_ORIENTATION_AS_EULER_ANGLES)
    {
        _addToStreamingSlots(0x07, 12, buff, curr_slot);
    }
    if (data_flags & TSS_STREAM_UNTARED_ORIENTATION_AS_ROTATION_MATRIX)
    {
        _addToStreamingSlots(0x08, 36, buff, curr_slot);
    }
    if (data_flags & TSS_STREAM_UNTARED_ORIENTATION_AS_AXIS_ANGLE)
    {
        _addToStreamingSlots(0x09, 16, buff, curr_slot);
    }
    if (data_flags & TSS_STREAM_UNTARED_ORIENTATION_AS_TWO_VECTOR)
    {
        _addToStreamingSlots(0x0A, 24, buff, curr_slot);
    }

    //fill out any unused slots with blank commands
    while (curr_slot < 8)
    {
        buff[curr_slot + 1] = 0xff;
        _streamingSlots[curr_slot] = 0xff;
        curr_slot++;
    }
    TSS_RETRY(_sendCommandBytes(buff, 9), TSS_START_STREAM_RETRIES);

    if (result != TSS_SUCCESS)
        return result;

    //record the streaming interval, or a guess at the interval
    if (interval == 0)
        _streamInterval = 5000;
    else
        if (interval < 1000)
            _streamInterval = 1000;
        else
            _streamInterval = interval;


    //send stream timing command to sensor
    buff[0] = 82;
    memcpy(buff + 1, &interval, 4);
    memcpy(buff + 5, &duration, 4);
    memcpy(buff + 9, &delay, 4);

    yost::swap32(buff + 1);
    yost::swap32(buff + 5);
    yost::swap32(buff + 9);

    TSS_RETRY(_sendCommandBytes(buff, 13), TSS_START_STREAM_RETRIES);

    if (result != TSS_SUCCESS)
        return result;

    return TSS_SUCCESS;
}

TSS_RESULT TssSensor::_triggerStreaming()
{
    _streamBuffer.clear();

    //start streaming
    TSS_RETRY(_sendCommand(85), TSS_START_STREAM_RETRIES);

    if (result != TSS_SUCCESS)
        return result;

    _isStreaming = true;

    return TSS_SUCCESS;
}

TSS_RESULT TssSensor::openPort(std::string port)
{
    if (_port)
    {
        return TSS_ERROR_ALREADY_CONNECTED;
    }

    _openPort(port);

    return TSS_SUCCESS;
}

TSS_RESULT TssSensor::closePort()
{
    if (_isWireless)
    {
        return TSS_ERROR_CANT_CLOSE_WIRELESS_PORT;
    }

    BASIC_CALL_CHECK_TSS();

    _port->close();
    _port.reset();


    return TSS_SUCCESS;
}

TSS_RESULT TssSensor::enableTimestampsWired()
{
    if (_isWireless)
    {
        return TSS_ERROR_NOT_AVAILABLE_WIRELESS;
    }

    BASIC_CALL_CHECK_TSS();

    _setResponseHeader(_responseHeaderFlags | TSS_RESPONSE_HEADER_TIMESTAMP);

    return TSS_SUCCESS;
}

TSS_RESULT TssSensor::disableTimestampsWired()
{
    if (_isWireless)
    {
        return TSS_ERROR_NOT_AVAILABLE_WIRELESS;
    }

    BASIC_CALL_CHECK_TSS();

    _setResponseHeader(_responseHeaderFlags & ~TSS_RESPONSE_HEADER_TIMESTAMP);

    return TSS_SUCCESS;
}

TSS_RESULT TssSensor::startStreamingWired(U32 data_flags, U32 interval, U32 duration, U32 delay)
{
    if (_isWireless)
    {
        return TSS_ERROR_NO_WIRELESS_STREAMING_FROM_SENSOR;
    }

    TSS_ERROR_CHECK(_prepareStreaming(data_flags, interval, duration, delay));

    TSS_ERROR_CHECK(_triggerStreaming());

    //register this device with the overall module so it can assign threading to it
    gAPI.registerStreamingDevice(this);

    return TSS_SUCCESS;
}

void TssSensor::_parseStreamingPacketItem(U8* dest, U8* src, U16& src_index, U8 nbytes)
{
    memcpy(dest, src + src_index, nbytes);
    src_index += nbytes;
}

void TssSensor::_parseStreamingPacketItemFloats(float* dest, U8* src, U16& src_index, U8 nfloats)
{
    _parseStreamingPacketItem((U8*)dest, src, src_index, nfloats * 4);

    _parseFloats(dest, nfloats);
}

TSS_RESULT TssSensor::_parseStreamingPacket(TssStreamPacket* packet)
{
    U16 raw_data_index = 0;

    U8 i;
    for (i = 0; i < 8; i++)
    {
        if (_streamingSlots[i] == 0xff)
            break;

        //check through the slots one by one and pull out the raw data that belongs to that slot
        if (_streamingSlots[i] == 0x00)
        {
            _parseStreamingPacketItemFloats(packet->taredOrientQuat.data, packet->rawData, raw_data_index, 4);
        }
        else if (_streamingSlots[i] == 0x01)
        {
            _parseStreamingPacketItemFloats(packet->taredOrientEuler, packet->rawData, raw_data_index, 3);
        }
        else if (_streamingSlots[i] == 0x02)
        {
            _parseStreamingPacketItemFloats(packet->taredOrientMatrix.data, packet->rawData, raw_data_index, 9);
        }
        else if (_streamingSlots[i] == 0x03)
        {
            float data[4];
            _parseStreamingPacketItemFloats(data, packet->rawData, raw_data_index, 4);
            memcpy(packet->taredOrientAxis.data, data, sizeof(float) * 3);
            packet->taredOrientAngle = data[3];
        }
        else if (_streamingSlots[i] == 0x04)
        {
            float data[6];
            _parseStreamingPacketItemFloats(data, packet->rawData, raw_data_index, 6);
            memcpy(packet->taredOrientForward.data, data, sizeof(float) * 3);
            memcpy(packet->taredOrientDown.data, data + 3, sizeof(float) * 3);
        }
        else if (_streamingSlots[i] == 0x06)
        {
            _parseStreamingPacketItemFloats(packet->untaredOrientQuat.data, packet->rawData, raw_data_index, 4);
        }
        else
            if (_streamingSlots[i] == 0x07)
            {
                _parseStreamingPacketItemFloats(packet->untaredOrientEuler, packet->rawData, raw_data_index, 3);
            }
            else if (_streamingSlots[i] == 0x08)
            {
                _parseStreamingPacketItemFloats(packet->untaredOrientMatrix.data, packet->rawData, raw_data_index, 9);
            }
            else if (_streamingSlots[i] == 0x09)
            {
                float data[4];
                _parseStreamingPacketItemFloats(data, packet->rawData, raw_data_index, 4);
                memcpy(packet->untaredOrientAxis.data, data, sizeof(float) * 3);
                packet->untaredOrientAngle = data[3];
            }
            else if (_streamingSlots[i] == 0x0A)
            {
                float data[6];
                _parseStreamingPacketItemFloats(data, packet->rawData, raw_data_index, 6);
                memcpy(packet->untaredOrientNorth.data, data, sizeof(float) * 3);
                memcpy(packet->untaredOrientGravity.data, data + 3, sizeof(float) * 3);
            }
    }

    return TSS_SUCCESS;
}

TSS_RESULT TssSensor::getFirstStreamingPacket(TssStreamPacket* packet)
{
    gAPI._readerThreadMutex.lock(); //lock the reading thread so it won't mess with our buffer
    if (_streamBuffer.len == 0)
    {
        gAPI._readerThreadMutex.unlock();
        return TSS_ERROR_NOT_ENOUGH_DATA;
    }

    //get index of last packet
    U32 packet_index = _streamBuffer.start;
    _streamBuffer.getPacket(packet, packet_index);

    //parse the streaming packet out into member variables
    result = _parseStreamingPacket(packet);
    if (result != TSS_SUCCESS)
    {
        gAPI._readerThreadMutex.unlock();
        return result;
    }

    //remove the parsed packet from the buffer
    _streamBuffer.removeFirstPacket();

    gAPI._readerThreadMutex.unlock();

    return TSS_SUCCESS;
}

TSS_RESULT TssSensor::getLastStreamingPacket(TssStreamPacket* packet)
{
    gAPI._readerThreadMutex.lock(); //lock the reading thread so it won't mess with our buffer

    if (_streamBuffer.len == 0)
    {
        gAPI._readerThreadMutex.unlock();
        return TSS_ERROR_NOT_ENOUGH_DATA;
    }

    //get index of last packet
    U32 packet_index = _streamBuffer.end;
    if (packet_index == 0)
        packet_index = TSS_STREAM_PACKET_CIRCULAR_BUFFER_SIZE - 1;
    else
        packet_index--;
    _streamBuffer.getPacket(packet, packet_index);

    //parse the streaming packet out into member variables
    result = _parseStreamingPacket(packet);
    if (result != TSS_SUCCESS)
    {
        gAPI._readerThreadMutex.unlock();
        return result;
    }

    //dump all packets in the buffer
    _streamBuffer.clear();

    gAPI._readerThreadMutex.unlock();

    return TSS_SUCCESS;
}

TSS_RESULT TssSensor::stopStreamingWired()
{
    if (!_port)
    {
        return TSS_ERROR_NOT_CONNECTED;
    }

    if (_isWireless)
    {
        return TSS_ERROR_NOT_AVAILABLE_WIRELESS;
    }

    if (!_isStreaming)
    {
        return TSS_ERROR_NOT_STREAMING;
    }

    gAPI.unregisterStreamingDevice(this);

    TSS_ERROR_CHECK(_sendCommand(86)); //stop streaming command

    _isStreaming = false;

    return result;
}

U32 TssSensor::getStreamingPacketsInWaiting()
{
    return _streamBuffer.len;
}

bool TssSensor::didStreamingOverflow()
{
    return _streamBuffer.len == TSS_STREAM_PACKET_CIRCULAR_BUFFER_SIZE;
}

void TssSensor::setCommandRetries(U8 retries)
{
    _commandRetries = retries;
}

TSS_RESULT TssSensor::getTaredOrientationAsQuaternion(Orient* quat)
{
    TSS_ERROR_CHECK(_checkedCommandWriteReadFloats(0x00, quat->data, 4));

    return TSS_SUCCESS;
}

TSS_RESULT TssSensor::getTaredOrientationAsEulerAngles(float* euler)
{
    TSS_ERROR_CHECK(_checkedCommandWriteReadFloats(0x01, euler, 3));

    return TSS_SUCCESS;
}

TSS_RESULT TssSensor::getTaredOrientationAsRotationMatrix(Matrix3x3* mat)
{
    TSS_ERROR_CHECK(_checkedCommandWriteReadFloats(0x02, mat->data, 9));

    return TSS_SUCCESS;
}

TSS_RESULT TssSensor::getTaredOrientationAsAxisAngle(Vector3* vec, float* angle)
{
    float data[4];

    TSS_ERROR_CHECK(_checkedCommandWriteReadFloats(0x03, data, 4));

    memcpy(vec->data, data, sizeof(float) * 3);
    *angle = data[3];

    return TSS_SUCCESS;
}

TSS_RESULT TssSensor::getTaredOrientationAsTwoVector(Vector3* forward, Vector3* down)
{
    float data[6];

    TSS_ERROR_CHECK(_checkedCommandWriteReadFloats(0x04, data, 6));

    memcpy(forward->data, data, sizeof(float) * 3);
    memcpy(down->data, data, sizeof(float) * 3);

    return TSS_SUCCESS;
}

TSS_RESULT TssSensor::getUntaredOrientationAsQuaternion(Orient* quat)
{
    TSS_ERROR_CHECK(_checkedCommandWriteReadFloats(0x06, quat->data, 4));

    return TSS_SUCCESS;
}

TSS_RESULT TssSensor::getUntaredOrientationAsEulerAngles(float* euler)
{
    TSS_ERROR_CHECK(_checkedCommandWriteReadFloats(0x07, euler, 3));

    return TSS_SUCCESS;
}

TSS_RESULT TssSensor::getUntaredOrientationAsRotationMatrix(Matrix3x3* mat)
{
    TSS_ERROR_CHECK(_checkedCommandWriteReadFloats(0x08, mat->data, 9));

    return TSS_SUCCESS;
}

TSS_RESULT TssSensor::getUntaredOrientationAsAxisAngle(Vector3* vec, float* angle)
{
    float data[4];

    TSS_ERROR_CHECK(_checkedCommandWriteReadFloats(0x09, data, 4));

    memcpy(vec->data, data, sizeof(float) * 3);
    *angle = data[3];

    return TSS_SUCCESS;
}

TSS_RESULT TssSensor::getUntaredOrientationAsTwoVector(Vector3* forward, Vector3* down)
{
    float data[6];

    TSS_ERROR_CHECK(_checkedCommandWriteReadFloats(0x0A, data, 6));

    memcpy(forward->data, data, sizeof(float) * 3);
    memcpy(down->data, data, sizeof(float) * 3);

    return TSS_SUCCESS;
}

TSS_RESULT TssSensor::tareWithCurrentOrientation()
{
    BASIC_CALL_CHECK_TSS();

    TSS_ERROR_CHECK(_sendCommand(0x60));

    return TSS_SUCCESS;
}

TSS_RESULT TssSensor::getWirelessChannel(U8* channel)
{
    if (_sensorType != TSS_WL)
    {
        return TSS_ERROR_SENSOR_TYPE_MISMATCH;
    }

    TSS_ERROR_CHECK(_checkedCommandWriteRead(0xC2, channel, 1));

    return TSS_SUCCESS;
}

TSS_RESULT TssSensor::setWirelessChannel(U8 channel)
{
    if (_sensorType != TSS_WL)
    {
        return TSS_ERROR_SENSOR_TYPE_MISMATCH;
    }

    BASIC_CALL_CHECK_TSS();

    U8 buff[2];
    buff[0] = 0xC3;
    buff[1] = channel;

    TSS_ERROR_CHECK(_sendCommandBytes(buff, 2));

    return TSS_SUCCESS;
}

TSS_RESULT TssSensor::getWirelessPanID(U16* panid)
{
    if (_sensorType != TSS_WL)
    {
        return TSS_ERROR_SENSOR_TYPE_MISMATCH;
    }

    TSS_ERROR_CHECK(_checkedCommandWriteRead(0xC0, (U8*)panid, 2));

    yost::swap16((U8*)panid);

    return TSS_SUCCESS;
}

TSS_RESULT TssSensor::setWirelessPanID(U16 panid)
{
    if (_sensorType != TSS_WL)
    {
        return TSS_ERROR_SENSOR_TYPE_MISMATCH;
    }

    BASIC_CALL_CHECK_TSS();

    U8 buff[3];
    buff[0] = 0xC1;

    memcpy(buff + 1, &panid, 2);
    yost::swap16((U8*)buff + 1);

    TSS_ERROR_CHECK(_sendCommandBytes(buff, 3));

    return TSS_SUCCESS;
}

TSS_RESULT TssSensor::commitSettings()
{
    BASIC_CALL_CHECK_TSS();

    TSS_ERROR_CHECK(_sendCommand(0xE1));

    return TSS_SUCCESS;
}

TSS_RESULT TssSensor::commitWirelessSettings()
{
    BASIC_CALL_CHECK_TSS();

    TSS_ERROR_CHECK(_sendCommand(0xc5));

    return TSS_SUCCESS;
}