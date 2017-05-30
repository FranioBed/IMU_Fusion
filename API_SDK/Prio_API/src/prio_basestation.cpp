#include "prio_basestation.hpp"
#include "prio_api.hpp"

PrioBaseStation::PrioBaseStation(const char* port)
{
    _type = PRIO_DEVICE_TYPE_BS;
    _portName = new char[64];
    _children.resize(0, NULL);
    _isWireless = false;
    _logicalId = 176;

    prio_error = openPort(port);

    if (prio_error != PRIO_NO_ERROR)
    {
        closePort();
        return;
    }

    gPrioAPI._commandThreadMutex.lock();
    _sendCommand(86); //pause streaming output
    gPrioAPI._commandThreadMutex.unlock();
    
    disableTimestampsWireless();

    prio_error = _readSerialNumber();

    if (prio_error != PRIO_NO_ERROR)
    {
        closePort();
        return;
    }

    prio_error = _readVersionString();

    if (prio_error != PRIO_NO_ERROR)
    {
        closePort();
        return;
    }

    if (_sensorType != PRIO_BS)
    {
        closePort();
        return;
    }

    _childCount = 0;

    _responseHeaderWirelessFlags = 0;
    
    _setResponseHeader(0);

    _setWirelessResponseHeader(PRIO_RESPONSE_HEADER_SUCCESS | PRIO_RESPONSE_HEADER_COMMAND_ECHO | PRIO_RESPONSE_HEADER_LOGICAL_ID);

    _responseHeaderWirelessFlagsPreStream = _responseHeaderWirelessFlags;

    _maxStreamInterval = 0;
    _commandRetries = 5;

    _children.resize(PRIO_BASE_STATION_NUM_CHILDREN, NULL);
}

PrioBaseStation::PrioBaseStation(const PrioBaseStation& other)
{
    throw runtime_error("Deep copies of dongle objects not allowed.");
}

PrioBaseStation::~PrioBaseStation()
{
    stopStreaming();

    if (_port && _isportOwner)
    {
        _breakPortThread = true;
        _portThread.join();
        _portThreadMutex.lock();
        _portThreadMutex.unlock();
        _port = NULL;
    }
}

void PrioBaseStation::operator =(const PrioBaseStation& other)
{
    throw runtime_error("Deep copies of dongle objects not allowed.");
}

PRIO_ERROR PrioBaseStation::openPort(const char* port)
{
    if (_port)
    {
        return PRIO_ERROR_COMMAND_FAIL;
    }

    prio_error = _openPort(port);

    if (prio_error == PRIO_NO_ERROR)
    {
        for (U8 i = 0; i < _children.size(); i++)
        {
            _children[i]->_ownerDongle = this;
            _children[i]->_port = _port;
        }
    }

    return prio_error;
}

PRIO_ERROR PrioBaseStation::closePort()
{
    BASIC_CALL_CHECK();

    if (_portThread.joinable())
    {
        _breakPortThread = true;
        _portThread.join();
        _portThreadMutex.lock();
        _portThreadMutex.unlock();
    }

    _port = NULL;
    _port.reset();

    for (U8 i = 0; i < _children.size(); i++)
    {
        _children[i]->_ownerDongle = NULL;
        _children[i]->_port = NULL;
        _children[i]->_port.reset();
    }

    return PRIO_NO_ERROR;
}

PRIO_ERROR PrioBaseStation::enableTimestampsWireless()
{
    BASIC_CALL_CHECK();

    for (auto hub : _children)
    {
        hub->enableTimestampsWired();
    }

    _setWirelessResponseHeader(_responseHeaderWirelessFlags | PRIO_RESPONSE_HEADER_TIMESTAMP);

    return PRIO_NO_ERROR;
}

PRIO_ERROR PrioBaseStation::disableTimestampsWireless()
{
    BASIC_CALL_CHECK();

    for (auto hub : _children)
    {
        hub->disableTimestampsWired();
    }

    _setWirelessResponseHeader(_responseHeaderWirelessFlags & ~PRIO_RESPONSE_HEADER_TIMESTAMP);

    return PRIO_NO_ERROR;
}

PRIO_ERROR PrioBaseStation::getPairedHub(shared_ptr<PrioHub>& hub)
{
    hub = shared_ptr<PrioHub>(new PrioHub());

    hub->_port = _port;
    hub->_ownerDongle = this;
    hub->_isWireless = true;
    hub->_logicalId = 240;
    hub->_commandRetries = PRIO_DEFAULT_WIRELESS_COMMAND_RETRIES;

    hub->_setResponseHeader(_responseHeaderWirelessFlags);
    hub->_setWirelessResponseHeader(_responseHeaderWirelessFlags);
    hub->disableTimestampsWired();

    //in an ideal case, it would be better to be able to tell when it had stopped streaming
    //and stop sending the commands at that point, but attempts to use the streaming bitfield
    //here yielded poor results.  we should revisit this and find a better way to do this
    //when we get the chance.
    gPrioAPI._commandThreadMutex.lock();
    for (U8 i = 0; i < PRIO_STOP_STREAM_RETRIES; i++)
    {
        hub->_sendCommand(86, true); //stop streaming command, don't wait for results
    }

    //sleep for a long time to make sure the dongle has processed all the commands and responded to each
    yost::sleep_ms(500);

    //dump data from all the stop streaming commands
    _purgeInput();

    gPrioAPI._commandThreadMutex.unlock();

    hub->_setResponseHeader(_responseHeaderWirelessFlags);
    hub->_setWirelessResponseHeader(_responseHeaderWirelessFlags);


    #define PRIO_WIRELESS_SENSOR_SERIAL_RETRIES 50

    bool success = false;
    for (U32 i = 0; i < PRIO_WIRELESS_SENSOR_SERIAL_RETRIES; i++)
    {
        prio_error = hub->_readSerialNumber();

        if (prio_error == PRIO_NO_ERROR)
        {
            success = true;
            break;
        }
    }

    if (!success)
    {
        hub.reset();
        return PRIO_ERROR_READ;
    }

    success = false;
    for (U32 i = 0; i < PRIO_WIRELESS_SENSOR_SERIAL_RETRIES; i++)
    {
        prio_error = hub->_readVersionString();

        if (prio_error == PRIO_NO_ERROR)
        {
            success = true;
            break;
        }
    }

    if (!success)
    {
        hub.reset();
        return PRIO_ERROR_READ;
    }

    hub->_setupEnumeratorBitfield();

    _children[0] = hub;
    _childCount++;

    return PRIO_NO_ERROR;
}

PRIO_ERROR PrioBaseStation::startStreaming(U32 data_flags, U32 interval, U32 duration, bool reg)
{
    BASIC_CALL_CHECK();

    U8 count = 0;
    for (auto& child : _children)
    {
        if (!child)
        {
            continue;
        }

        //this automatically handles the delays for all children while starting to stream
        PRIO_ERROR error = child->enableStreamingWireless(data_flags, interval, PRIO_STREAM_DURATION_INFINITE, 25000 * (_childCount - count));
        if (error != PRIO_NO_ERROR)
        {
            return error;
        }
        count++;
    }

    U8 failures;

    _maxStreamInterval = 0;

    for (int i = 0; i < PRIO_START_STREAM_RETRIES; i++)
    {
        failures = 0;

        for (vector<shared_ptr<PrioHub>>::iterator it = _children.begin(); it != _children.end(); ++it)
        {
            if ((*it) && (*it)->_streamDataSize && !(*it)->_isStreaming)
            {
                (*it)->_streamBuffer.clear();

                if ((*it)->_streamInterval > _maxStreamInterval)
                {
                    _maxStreamInterval = (*it)->_streamInterval;
                }

                gPrioAPI._commandThreadMutex.lock();
                //call start streaming command
                if ((*it)->_sendCommand(85) == PRIO_NO_ERROR)
                {
                    (*it)->_isStreaming = true;
                    (*it)->_breakParseThread = false;
                    (*it)->_unparsed_stream_data.empty();
                    (*it)->_parser_thread = std::thread(&PrioHub::_parseStreamData, (*it));
                    (*it)->_setResponseHeader(0);
                    (*it)->_setWirelessResponseHeader(0);
                }
                else
                {
                    failures++;
                }
                gPrioAPI._commandThreadMutex.unlock();
            }
        }

        if (failures == 0)
        {
            break;
        }
    }


    if (failures > 0)
    {
        return PRIO_ERROR_COMMAND_FAIL;
    }

    _responseHeaderWirelessFlagsPreStream = _responseHeaderWirelessFlags;

    _setWirelessResponseHeader(0);

    gPrioAPI._commandThreadMutex.lock();
    PRIO_COMMAND_CHECK(_sendCommand(85)); //open the floodgates, here it comes
    gPrioAPI._commandThreadMutex.unlock();
    _isStreaming = true;

    //register this device, since there is now something to stream
    if (reg)
    {
        gPrioAPI.registerStreamingDevice(this);
    }
    else
    {
        gPrioAPI.unpauseStreamingDevice(this);
    }


    return PRIO_NO_ERROR;
}

U16 PrioBaseStation::_detectStreaming(U32 interval_us)
{
    U16 response;

    gPrioAPI._commandThreadMutex.lock();
    _sendCommand(183);

    prio_error = _readBytes((U8*)&response, 2);

    //the 45 is a fudge factor meant to guarantee any streaming data will be in during the waiting period
    yost::sleep_ms(interval_us / 1000 * 45);

    _sendCommand(183);

    prio_error = _readBytes((U8*)&response, 2);
    gPrioAPI._commandThreadMutex.unlock();

    if (prio_error == PRIO_NO_ERROR)
    {
        return response;
    }

    return 0;
}

PRIO_ERROR PrioBaseStation::stopStreaming(bool dereg)
{
    //in this function, we specifically don't make sure the basestatipm thinks it is streaming before
    //telling sensors to stop, as we want this to be able to catch any stray streaming sensors

    if (!isConnected())
    {
        return PRIO_ERROR_COMMAND_FAIL;
    }

    if (_isStreaming)
    {
        _isStreaming = false;

        if (dereg)
        {
            gPrioAPI.unregisterStreamingDevice(this);
        }
        else
        {
            gPrioAPI.pauseStreamingDevice(this);
        }

        gPrioAPI._commandThreadMutex.lock();
        _sendCommand(86); //pause streaming output
        gPrioAPI._commandThreadMutex.unlock();

        _setWirelessResponseHeader(_responseHeaderWirelessFlagsPreStream);
    }

    //dump any extra streaming data
    _purgeInput();

    for (int i = 0; i < PRIO_STOP_STREAM_RETRIES; i++)
    {
        for (vector<shared_ptr<PrioHub>>::iterator it = _children.begin(); it != _children.end(); ++it)
        {
            if ((*it))
            {
                gPrioAPI._commandThreadMutex.lock();
                (*it)->_sendCommand(86); //stop streaming command
                (*it)->_breakParseThread = true;
                if ((*it)->_parser_thread.joinable())
                {
                    (*it)->_parser_thread.join();
                }
                (*it)->_unparsed_stream_data.empty();
                gPrioAPI._commandThreadMutex.unlock();
            }
        }

        if (_detectStreaming(_maxStreamInterval) == 0x0000)
            break;
    }

    for (vector<shared_ptr<PrioHub>>::iterator it = _children.begin(); it != _children.end(); ++it)
    {
        if ((*it))
        {
            (*it)->_isStreaming = false;
            (*it)->_setResponseHeader(_responseHeaderWirelessFlagsPreStream);
            (*it)->_setWirelessResponseHeader(_responseHeaderWirelessFlagsPreStream);
        }
    }

    if (_detectStreaming(_maxStreamInterval) != 0x0000)
    {
        _maxStreamInterval = 0;
        return PRIO_ERROR_COMMAND_FAIL;
    }
    _maxStreamInterval = 0;
    return PRIO_NO_ERROR;
}

PRIO_ERROR PrioBaseStation::startRecording(U32 data_flags, U32 interval, U32 duration)
{
    BASIC_CALL_CHECK();

    U8 count = 0;
    for (auto& child : _children)
    {
        if (!child)
        {
            continue;
        }

        //this automatically handles the delays for all children while starting to stream
        prio_error = child->enableStreamingWireless(data_flags, interval, duration, 25000 * (_childCount - count));
        if (prio_error != PRIO_NO_ERROR)
        {
            return prio_error;
        }
        count++;
    }

    U8 failures;

    _maxStreamInterval = 0;

    for (int i = 0; i < PRIO_START_STREAM_RETRIES; i++)
    {
        failures = 0;

        for (vector<shared_ptr<PrioHub>>::iterator it = _children.begin(); it != _children.end(); ++it)
        {
            if ((*it) && (*it)->_streamDataSize && !(*it)->_isStreaming)
            {
                (*it)->_streamBuffer.clear();

                if ((*it)->_streamInterval > _maxStreamInterval)
                {
                    _maxStreamInterval = (*it)->_streamInterval;
                }

                prio_error = (*it)->enableTimestampsWired(); //If this is called pre command 85 is doesn't give data
                (*it)->_breakParseThread = false;
                (*it)->_unparsed_stream_data.empty();
                (*it)->_parser_thread = std::thread(&PrioHub::_parseStreamData, (*it));
                //call start streaming command
                gPrioAPI._commandThreadMutex.lock();
                if ((*it)->_sendCommand(85) == PRIO_NO_ERROR)
                {
                    (*it)->_isStreaming = true;
                    (*it)->_isRecording = true;
                }
                else
                {
                    failures++;
                }
                gPrioAPI._commandThreadMutex.unlock();
            }
        }

        if (failures == 0)
        {
            break;
        }
    }


    if (failures > 0)
    {
        return PRIO_ERROR_COMMAND_FAIL;
    }

    _responseHeaderWirelessFlagsPreStream = _responseHeaderWirelessFlags;


    //always use the logical id header flag so we know what data belongs to
    _setWirelessResponseHeader(_responseHeaderWirelessFlags | PRIO_RESPONSE_HEADER_LOGICAL_ID | PRIO_RESPONSE_HEADER_TIMESTAMP);

    gPrioAPI._commandThreadMutex.lock();
    PRIO_COMMAND_CHECK(_sendCommand(85)); //open the floodgates, here it comes
    gPrioAPI._commandThreadMutex.unlock();

    _isStreaming = true;
    _isRecording = true;

    //register this device, since there is now something to stream
    gPrioAPI.registerStreamingDevice(this);


    return PRIO_NO_ERROR;
}

PRIO_ERROR PrioBaseStation::stopRecording()
{
    //in this function, we specifically don't make sure the dongle thinks it is streaming before
    //telling sensors to stop, as we want this to be able to catch any stray streaming sensors
    disableTimestampsWireless();

    if (!isConnected())
    {
        return PRIO_ERROR_COMMAND_FAIL;
    }
    
    if (_isStreaming && _isRecording)
    {
        _isStreaming = false;
        _isRecording = false;

        gPrioAPI.unregisterStreamingDevice(this);

        gPrioAPI._commandThreadMutex.lock();
        _sendCommand(86); //pause streaming output
        gPrioAPI._commandThreadMutex.unlock();

        _setWirelessResponseHeader(_responseHeaderWirelessFlagsPreStream);
    }

    //dump any extra streaming data
    _purgeInput();

    for (int i = 0; i < PRIO_STOP_STREAM_RETRIES; i++)
    {
        for (vector<shared_ptr<PrioHub>>::iterator it = _children.begin(); it != _children.end(); ++it)
        {
            if ((*it))
            {
                gPrioAPI._commandThreadMutex.lock();
                (*it)->_sendCommand(86); //stop streaming command
                (*it)->_breakParseThread = true;
                if ((*it)->_parser_thread.joinable())
                {
                    (*it)->_parser_thread.join();
                }
                (*it)->_unparsed_stream_data.empty();
                gPrioAPI._commandThreadMutex.unlock();
            }
        }

        if (_detectStreaming(_maxStreamInterval) == 0x0000)
            break;
    }

    for (vector<shared_ptr<PrioHub>>::iterator it = _children.begin(); it != _children.end(); ++it)
    {
        if ((*it))
        {
            (*it)->_isStreaming = false;
            (*it)->_isRecording = false;
            (*it)->_setResponseHeader(_responseHeaderWirelessFlagsPreStream);
        }
    }

    if (_detectStreaming(_maxStreamInterval) != 0x0000)
    {
        _maxStreamInterval = 0;
        return PRIO_ERROR_COMMAND_FAIL;
    }
    _maxStreamInterval = 0;
    return PRIO_NO_ERROR;
}

PRIO_ERROR PrioBaseStation::getWirelessChannel(U8* channel)
{
    gPrioAPI._commandThreadMutex.lock();
    PRIO_COMMAND_CHECK(_checkedCommandWriteRead(0xC2, channel, 1));
    gPrioAPI._commandThreadMutex.unlock();

    return PRIO_NO_ERROR;
}

PRIO_ERROR PrioBaseStation::setWirelessChannel(U8 channel)
{
    BASIC_CALL_CHECK();

    U8 buff[2];
    buff[0] = 0xC3;
    buff[1] = channel;

    gPrioAPI._commandThreadMutex.lock();
    PRIO_COMMAND_CHECK(_sendCommandBytes(buff, 2));
    gPrioAPI._commandThreadMutex.unlock();

    return PRIO_NO_ERROR;
}

PRIO_ERROR PrioBaseStation::getWirelessPanID(U16* panid)
{
    gPrioAPI._commandThreadMutex.lock();
    PRIO_COMMAND_CHECK(_checkedCommandWriteRead(0xC0, (U8*)panid, 2));
    gPrioAPI._commandThreadMutex.unlock();

    yost::swap16((U8*)panid);

    return PRIO_NO_ERROR;
}

PRIO_ERROR PrioBaseStation::setWirelessPanID(U16 panid)
{
    BASIC_CALL_CHECK();

    U8 buff[3];
    buff[0] = 0xC1;

    memcpy(buff + 1, &panid, 2);
    yost::swap16(buff + 1);

    gPrioAPI._commandThreadMutex.lock();
    PRIO_COMMAND_CHECK(_sendCommandBytes(buff, 3));
    gPrioAPI._commandThreadMutex.unlock();

    return PRIO_NO_ERROR;
}

PRIO_ERROR PrioBaseStation::getSerialNumberAtLogicalID(U8 logical_id, U32* serial_number)
{
    BASIC_CALL_CHECK();

    gPrioAPI._commandThreadMutex.lock();
    PRIO_COMMAND_CHECK(_sendCommand(237, logical_id));

    PRIO_COMMAND_CHECK(_readBytes((U8*)serial_number, 4));
    gPrioAPI._commandThreadMutex.unlock();

    yost::swap32((U8*)serial_number);

    return PRIO_NO_ERROR;
}

PRIO_ERROR PrioBaseStation::getFirmwareVersionAtLogicalID(U8 logical_id, const char* firmware_version)
{
    BASIC_CALL_CHECK();

    gPrioAPI._commandThreadMutex.lock();
    PRIO_COMMAND_CHECK(_sendCommand(223, logical_id));

    U8 buff[13];
    PRIO_COMMAND_CHECK(_readBytes(buff, 12));

    gPrioAPI._commandThreadMutex.unlock();
    buff[12] = '\0';

    memcpy((char*)firmware_version, buff, 13);

    return PRIO_NO_ERROR;
}

PRIO_ERROR PrioBaseStation::getHardwareVersionAtLogicalID(U8 logical_id, const char* hardware_version)
{
    gPrioAPI._commandThreadMutex.lock();
    PRIO_COMMAND_CHECK(_sendCommand(230,logical_id));

    U8 buff[33];
    PRIO_COMMAND_CHECK(_readBytes(buff, 32));
    gPrioAPI._commandThreadMutex.unlock();
    buff[32] = '\0';

    memcpy((char*)hardware_version, buff, 33);

    return PRIO_NO_ERROR;
}


PRIO_ERROR PrioBaseStation::autoPairBaseStationWithHub()
{
    BASIC_CALL_CHECK();

    gPrioAPI._commandThreadMutex.lock();
    PRIO_COMMAND_CHECK(_sendCommand(0xba));
    gPrioAPI._commandThreadMutex.unlock();

    return PRIO_NO_ERROR;
}

PRIO_ERROR PrioBaseStation::reenumerateDevice()
{
    bool restartStreaming = _isStreaming;
    if (_isStreaming)
    {
        stopStreaming(false);
    }

    for (vector<shared_ptr<PrioHub>>::iterator it = _children.begin(); it != _children.end(); ++it)
    {
        if ((*it))
        {
            (*it)->_setupEnumeratorBitfield();
        }
    }

    if (restartStreaming)
    {
        for (int i = 0; i < PRIO_STOP_STREAM_RETRIES; i++)
        {
            prio_error = startStreaming(PRIO_STREAM_UNTARED_ORIENTATION_AS_QUATERNION, 0, PRIO_STREAM_DURATION_INFINITE, false);
            if (prio_error != PRIO_NO_ERROR)
            {
                break;
            }
        }
    }

    return PRIO_NO_ERROR;
}

// 210(0xd2)
PRIO_ERROR PrioBaseStation::getWirelessChannelStrengths(U8* channel_strengths16)
{
    BASIC_CALL_CHECK();

    gPrioAPI._commandThreadMutex.lock();
    PRIO_COMMAND_CHECK(_checkedCommandWriteRead(0xD2, (U8*)channel_strengths16, 16));
    gPrioAPI._commandThreadMutex.unlock();

    return PRIO_NO_ERROR;
}

PRIO_ERROR PrioBaseStation::getSignalStrength(U8* signal_strength)
{
    BASIC_CALL_CHECK();

    gPrioAPI._commandThreadMutex.lock();
    PRIO_COMMAND_CHECK(_checkedCommandWriteRead(0xD6, (U8*)signal_strength, 1));
    gPrioAPI._commandThreadMutex.unlock();

    return PRIO_NO_ERROR;
}

PRIO_ERROR PrioBaseStation::commitSettings()
{
    BASIC_CALL_CHECK();

    gPrioAPI._commandThreadMutex.lock();
    PRIO_COMMAND_CHECK(_sendCommand(0xE1));
    gPrioAPI._commandThreadMutex.unlock();

    return PRIO_NO_ERROR;
}