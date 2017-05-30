#include "threespace_dongle.hpp"
#include "threespace_api.hpp"

TssDongle::TssDongle(std::string port)
{
    _type = TSS_DEVICE_TYPE_DONGLE;

    result = _openPort(port);

    if (result != TSS_SUCCESS)
    {
        closePort();
        return;
    }

    _sendCommand(86); //pause streaming output

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

    if (_sensorType != TSS_DNG)
    {
        closePort();
        return;
    }

    _childCount = 0;

    _responseHeaderWirelessFlags = 0;

    _setWirelessResponseHeader(TSS_RESPONSE_HEADER_SUCCESS | TSS_RESPONSE_HEADER_LOGICAL_ID | TSS_RESPONSE_HEADER_DATA_LENGTH);

    _maxStreamInterval = 0;

    _children.resize(TSS_DONGLE_NUM_CHILDREN, NULL);
}

TssDongle::TssDongle(const TssDongle& other)
{
    throw runtime_error("Deep copies of dongle objects not allowed.");
}

void TssDongle::operator =(const TssDongle& other)
{
    throw runtime_error("Deep copies of dongle objects not allowed.");
}

TSS_RESULT TssDongle::openPort(std::string port)
{
    if (_port)
    {
        return TSS_ERROR_ALREADY_CONNECTED;
    }

    _openPort(port);

    return TSS_SUCCESS;
}

TSS_RESULT TssDongle::closePort()
{
    BASIC_CALL_CHECK_TSS();

    _port->close();

    for (U8 i = 0; i < _children.size(); i++)
    {
        if (_children[i] != nullptr)
        {
            _children[i]->_ownerDongle = NULL;
            _children[i]->_port = NULL;
            _children[i].reset();
        }
    }

    _port.reset();

    return TSS_SUCCESS;
}

TSS_RESULT TssDongle::enableTimestampsWireless()
{
    BASIC_CALL_CHECK_TSS();

    _setWirelessResponseHeader(_responseHeaderWirelessFlags | TSS_RESPONSE_HEADER_TIMESTAMP);

    return TSS_SUCCESS;
}

TSS_RESULT TssDongle::disableTimestampsWireless()
{
    BASIC_CALL_CHECK_TSS();

    _setWirelessResponseHeader(_responseHeaderWirelessFlags & ~TSS_RESPONSE_HEADER_TIMESTAMP);

    return TSS_SUCCESS;
}

TSS_RESULT TssDongle::getWirelessSensor(U8 logical_id, shared_ptr<TssSensor>& sensor)
{
    BASIC_CALL_CHECK_TSS();

    if (logical_id >= TSS_DONGLE_NUM_CHILDREN)
    {
        return TSS_ERROR_LOGICAL_ID_OUT_OF_RANGE;
    }

    if (_children[logical_id])
    {
        return TSS_ERROR_CHILD_EXISTS;
    }

    sensor = shared_ptr<TssSensor>(new TssSensor());

    sensor->_port = _port;
    sensor->_ownerDongle = this;
    sensor->_isWireless = true;
    sensor->_logicalId = logical_id;
    sensor->_commandRetries = TSS_DEFAULT_WIRELESS_COMMAND_RETRIES;

    sensor->_setResponseHeader(_responseHeaderWirelessFlags);

    //in an ideal case, it would be better to be able to tell when it had stopped streaming
    //and stop sending the commands at that point, but attempts to use the streaming bitfield
    //here yielded poor results.  we should revisit this and find a better way to do this
    //when we get the chance.
    for (U8 i = 0; i < TSS_STOP_STREAM_RETRIES; i++)
    {
        sensor->_sendCommand(86, true); //stop streaming command, don't wait for results
    }

    //sleep for a long time to make sure the dongle has processed all the commands and responded to each
    yost::sleep_ms(500);

    //dump data from all the stop streaming commands
    _purgeInput();

#define TSS_WIRELESS_SENSOR_SERIAL_RETRIES 50

    bool success = false;
    for (U32 i = 0; i < TSS_WIRELESS_SENSOR_SERIAL_RETRIES; i++)
    {
        result = sensor->_readSerialNumber();

        if (result == TSS_SUCCESS)
        {
            success = true;
            break;
        }
    }

    if (!success)
    {
        sensor.reset();
        return TSS_ERROR_CANT_READ_INIT_DATA;
    }



    success = false;
    for (U32 i = 0; i < TSS_WIRELESS_SENSOR_SERIAL_RETRIES; i++)
    {
        result = sensor->_readVersionString();

        if (result == TSS_SUCCESS)
        {
            success = true;
            break;
        }
    }

    if (!success)
    {
        sensor.reset();
        return TSS_ERROR_CANT_READ_INIT_DATA;
    }


    _children[logical_id] = sensor;
    _childCount++;

    return TSS_SUCCESS;
}

TSS_RESULT TssDongle::removeWirelessSensor(U8 logical_id)
{
    BASIC_CALL_CHECK_TSS();

    if (logical_id >= TSS_DONGLE_NUM_CHILDREN)
    {
        return TSS_ERROR_LOGICAL_ID_OUT_OF_RANGE;
    }

    if (!_children[logical_id])
    {
        return TSS_ERROR_CHILD_DOESNT_EXIST;
    }

    _childCount--;
    _children[logical_id].reset();

    return TSS_SUCCESS;
}

TSS_RESULT TssDongle::_setWirelessResponseHeader(U32 flags)
{
    U8 buff[256];
    buff[0] = 219;
    memcpy(buff + 1, &flags, 4);
    yost::swap32(buff + 1);
    _sendCommandBytes(buff, 5);

    _sendCommand(220); //set wireless response header command
    _readBytes(buff, 4);

    for (vector<shared_ptr<TssSensor>>::iterator it = _children.begin(); it != _children.end(); ++it)
    {
        if ((*it))
            (*it)->_setResponseHeader(flags);
    }

    _responseHeaderWirelessFlags = flags;
    _responseHeaderWirelessSize = 0;

    if (_responseHeaderWirelessFlags & TSS_RESPONSE_HEADER_SUCCESS)
    {
        _responseHeaderWirelessSize += 1;
    }

    if (_responseHeaderWirelessFlags & TSS_RESPONSE_HEADER_TIMESTAMP)
    {
        _responseHeaderWirelessSize += 4;
    }

    if (_responseHeaderWirelessFlags & TSS_RESPONSE_HEADER_COMMAND_ECHO)
    {
        _responseHeaderWirelessSize += 1;
    }

    if (_responseHeaderWirelessFlags & TSS_RESPONSE_HEADER_CHECKSUM)
    {
        _responseHeaderWirelessSize += 1;
    }

    if (_responseHeaderWirelessFlags & TSS_RESPONSE_HEADER_LOGICAL_ID)
    {
        _responseHeaderWirelessSize += 1;
    }

    if (_responseHeaderWirelessFlags & TSS_RESPONSE_HEADER_SERIAL_NUMBER)
    {
        _responseHeaderWirelessSize += 4;
    }

    if (_responseHeaderWirelessFlags & TSS_RESPONSE_HEADER_DATA_LENGTH)
    {
        _responseHeaderWirelessSize += 1;
    }

    return TSS_SUCCESS;
}

TSS_RESULT TssDongle::enableAllSensorsAndStartStreaming(U32 data_flags, U32 interval, U32 duration)
{
    BASIC_CALL_CHECK_TSS();

    U8 count = 0;
    for (auto& child : _children)
    {
        if (!child)
        {
            continue;
        }

        //this automatically handles the delays for all children while starting to stream
        TSS_RESULT result = child->enableStreamingWireless(data_flags, interval, TSS_STREAM_DURATION_INFINITE, 1000000 /** (_childCount - count)*/);
        //yost::sleep_ms(20);

        if (result != TSS_SUCCESS)
        {
            return result;
        }
        count++;
    }

    return startStreaming();
}

TSS_RESULT TssDongle::startStreaming()
{
    BASIC_CALL_CHECK_TSS();

    U8 failures;

    _maxStreamInterval = 0;

    for (int i = 0; i < TSS_START_STREAM_RETRIES; i++)
    {
        failures = 0;

        for (vector<shared_ptr<TssSensor>>::iterator it = _children.begin(); it != _children.end(); ++it)
        {
            if ((*it) && (*it)->_streamDataSize && !(*it)->_isStreaming)
            {
                (*it)->_streamBuffer.clear();

                if ((*it)->_streamInterval > _maxStreamInterval)
                {
                    _maxStreamInterval = (*it)->_streamInterval;
                }
                yost::sleep_ms(20);
                //call start streaming command
                if ((*it)->_sendCommand(85) == TSS_SUCCESS)
                {
                    (*it)->_isStreaming = true;
                }
                else
                {
                    failures++;
                }
            }
        }

        if (failures == 0)
            break;
    }


    if (failures > 0)
        return TSS_ERROR_NOT_STREAMING;

    TSS_ERROR_CHECK(_sendCommand(85)); //open the floodgates, here it comes

    _isStreaming = true;

    _responseHeaderWirelessFlagsPreStream = _responseHeaderWirelessFlags;

    //always use the logical id header flag so we know what data belongs to
    _setWirelessResponseHeader(_responseHeaderWirelessFlags | TSS_RESPONSE_HEADER_LOGICAL_ID);

    //register this device, since there is now something to stream
    gAPI.registerStreamingDevice(this);

    return TSS_SUCCESS;
}

U16 TssDongle::_detectStreaming(U32 interval_us)
{
    U16 response;

    _sendCommand(183);

    result = _readBytes((U8*)&response, 2);


    //the 15 is a fudge factor meant to guarantee any streaming data will be in during the waiting period
    yost::sleep_ms(interval_us / 1000 * 15);


    _sendCommand(183);

    result = _readBytes((U8*)&response, 2);

    if (result == TSS_SUCCESS)
    {
        return response;
    }

    return 0;
}

TSS_RESULT TssDongle::stopStreaming()
{
    //in this function, we specifically don't make sure the dongle thinks it is streaming before
    //telling sensors to stop, as we want this to be able to catch any stray streaming sensors

    if (!isConnected())
    {
        return TSS_ERROR_NOT_CONNECTED;
    }

    if (_isStreaming)
    {
        _isStreaming = false;

        gAPI.unregisterStreamingDevice(this);

        _sendCommand(86); //pause streaming output

        _setWirelessResponseHeader(_responseHeaderWirelessFlagsPreStream);
    }

    //dump any extra streaming data
    _purgeInput();

    for (int i = 0; i < TSS_STOP_STREAM_RETRIES; i++)
    {
        for (vector<shared_ptr<TssSensor>>::iterator it = _children.begin(); it != _children.end(); ++it)
        {
            if ((*it))
            {
                (*it)->_sendCommand(86); //stop streaming command
                yost::sleep_ms(20);
            }
        }

        if (_detectStreaming(_maxStreamInterval) == 0x0000)
            break;
    }

    for (vector<shared_ptr<TssSensor>>::iterator it = _children.begin(); it != _children.end(); ++it)
    {
        if ((*it))
        {
            (*it)->_isStreaming = false;
        }
    }


    if (_detectStreaming(_maxStreamInterval) != 0x0000)
    {
        _maxStreamInterval = 0;
        return TSS_ERROR_CANT_STOP_STREAMING;
    }

    _maxStreamInterval = 0;

    return TSS_SUCCESS;
}

TSS_RESULT TssDongle::getWirelessChannel(U8* channel)
{
    TSS_ERROR_CHECK(_checkedCommandWriteRead(0xC2, channel, 1));

    return TSS_SUCCESS;
}

TSS_RESULT TssDongle::setWirelessChannel(U8 channel)
{
    BASIC_CALL_CHECK_TSS();

    U8 buff[2];
    buff[0] = 0xC3;
    buff[1] = channel;

    TSS_ERROR_CHECK(_sendCommandBytes(buff, 2));

    return TSS_SUCCESS;
}

TSS_RESULT TssDongle::getWirelessPanID(U16* panid)
{
    TSS_ERROR_CHECK(_checkedCommandWriteRead(0xC0, (U8*)panid, 2));

    yost::swap16((U8*)panid);

    return TSS_SUCCESS;
}

TSS_RESULT TssDongle::setWirelessPanID(U16 panid)
{
    BASIC_CALL_CHECK_TSS();

    U8 buff[3];
    buff[0] = 0xC1;

    memcpy(buff + 1, &panid, 2);
    yost::swap16((U8*)buff + 1);

    TSS_ERROR_CHECK(_sendCommandBytes(buff, 3));

    return TSS_SUCCESS;
}

TSS_RESULT TssDongle::getSerialNumberAtLogicalID(U8 logical_id, U32* serial_number)
{
    BASIC_CALL_CHECK_TSS();

    U8 buff[2];
    buff[0] = 0xD0;
    buff[1] = logical_id;

    TSS_ERROR_CHECK(_sendCommandBytes(buff, 2));

    TSS_ERROR_CHECK(_readBytes((U8*)serial_number, 4));

    yost::swap32((U8*)serial_number);

    return TSS_SUCCESS;
}

TSS_RESULT TssDongle::setSerialNumberAtLogicalID(U8 logical_id, U32 serial_number)
{
    BASIC_CALL_CHECK_TSS();

    U8 buff[6];
    buff[0] = 0xD1;
    buff[1] = logical_id;

    memcpy(buff + 2, &serial_number, 4);
    yost::swap32(buff + 2);

    TSS_ERROR_CHECK(_sendCommandBytes(buff, 6));

    return TSS_SUCCESS;
}

TSS_RESULT TssDongle::commitWirelessSettings()
{
    BASIC_CALL_CHECK_TSS();

    TSS_ERROR_CHECK(_sendCommand(0xc5));

    return TSS_SUCCESS;
}