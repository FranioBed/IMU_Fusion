#include "prio_hub.hpp"
#include "prio_api.hpp"
#include "yost_math.hpp"
#include <iostream>
#include <fstream>

void PrioHub::_parseStreamData()
{
    while (1)
    {
        if (_breakParseThread)
            break;
        yost::sleep_ms(1);
        _parser_mutex.lock();

        bool foundAPacket = false;
        int packet_start = 0;
        U32 stream_data_size = 0;
        PrioStreamPacket packet;
        PrioHeader header;
        if (_isRecording)
        {
            while (1) //We break out in the case where we have found a full packet to consume
            {
                // Checking to see if the buffer can still have a packet header in it
                if (_unparsed_stream_data.size() < packet_start + _responseHeaderSize + 3)
                {
                    _parser_mutex.unlock();
                    break;
                }

                U8 currentPattern[3];
                currentPattern[0] = _unparsed_stream_data[packet_start];
                currentPattern[1] = _unparsed_stream_data[packet_start + 1];
                currentPattern[2] = _unparsed_stream_data[packet_start + 2];

                if (currentPattern[0] != 0 || currentPattern[1] != 255 || currentPattern[2] != 255)
                {
                    packet_start++;
                    continue;
                }
                packet_start += 3;

                vector<U8>::const_iterator first = _unparsed_stream_data.begin() + packet_start;
                vector<U8>::const_iterator last = _unparsed_stream_data.begin() + packet_start + _responseHeaderSize;
                vector<U8> buff(first, last);

                if (_responseHeaderFlags & PRIO_RESPONSE_HEADER_TIMESTAMP)
                {
                    U8 checksum = 0;
                    for (int i = 3; i < buff.size() - 1; i++)
                    {
                        checksum += buff[i];
                    }

                    //Check the checksum to see if this is indeed a packet header
                    if (!checksum == buff[buff.size() - 1])
                    {
                        packet_start++;
                        continue;
                    }
                }
                else
                {
                    U8 checksum = 0;
                    for (int i = 0; i < buff.size() - 1; i++)
                    {
                        checksum += buff[i];
                    }

                    //Check the checksum to see if this is indeed a packet header
                    if (!(checksum == buff[buff.size() - 1]))
                    {
                        packet_start++;
                        continue;
                    }
                }

                _prioParseStreamData(buff, (U8)_responseHeaderFlags, &header);

                //Parse active sensors to figure out our header size
                if (header.SensorBitfield != _enumeration_bitfield)
                {
                    _active_nodes.clear();
                    uint64_t bit;
                    uint8_t idx = 0;
                    for (bit = 1; bit < (1LL << 43); bit <<= 1)
                    {
                        if (header.SensorBitfield & bit)
                        {
                            _active_nodes.push_back(idx);
                        }
                        idx++;
                    }
                    U8 joystick1ID = (header.SensorBitfield & 71776119061217280) >> 48;
                    memcpy(&_joystick_1_id, &joystick1ID, 1);

                    U8 joystick2ID = (header.SensorBitfield & 18374686479671623680) >> 56;
                    memcpy(&_joystick_2_id, &joystick2ID, 1);

                    if ((6 < joystick1ID && joystick1ID < 14) || (20 < joystick1ID && joystick1ID < 28) || (34 < joystick1ID && joystick1ID < 42))
                    {
                        swapStreamJoystick = false;
                    }
                    else if ((6 < joystick2ID && joystick2ID < 14) || (20 < joystick2ID && joystick2ID < 28) || (34 < joystick2ID && joystick2ID < 42))
                    {
                        swapStreamJoystick = true;
                    }
                    else if ((0 < joystick1ID && joystick1ID < 7) || (13 < joystick1ID && joystick1ID < 19) || (20 < joystick1ID && joystick1ID < 28))
                    {
                        swapStreamJoystick = true;
                    }
                    else
                    {
                        swapStreamJoystick = false;
                    }

                    _enumeration_bitfield = header.SensorBitfield;
                }

                if (swapStreamJoystick)
                {
                    PrioJoystick tmp;
                    memcpy(&tmp, &header.Joystick1, sizeof(PrioJoystick));

                    memcpy(&header.Joystick1, &header.Joystick2, sizeof(PrioJoystick));
                    memcpy(&header.Joystick2, &tmp, sizeof(PrioJoystick));
                }

                stream_data_size = _streamDataSize*getActiveSensors().size();

                //If not enough data return
                if (_unparsed_stream_data.size() < packet_start + stream_data_size + _responseHeaderSize)
                {
                    _parser_mutex.unlock();
                    break;
                }

                first = _unparsed_stream_data.begin() + packet_start + _responseHeaderSize;
                last = _unparsed_stream_data.begin() + packet_start + _responseHeaderSize + stream_data_size;
                vector<U8> packet_buff(first, last);
                _unparsed_stream_data.erase(_unparsed_stream_data.begin(), last);

                memcpy(&packet.header, &header, sizeof(PrioHeader));
				memcpy(packet.rawData, &packet_buff[0], packet_buff.size());
                packet.rawDataSize = stream_data_size;

                packet_start = 0;

                _readerThreadMutex.lock();
                _streamBuffer.push_back(packet);
                clock_t time = clock();
                if (last_time != 0)
                {
                    float deltaTime = (time - last_time) / (CLOCKS_PER_SEC / 1000.0f);
                    packet_times.push_back(deltaTime);
                    if (packet_times.size() > 10)
                    {
                        packet_times.pop_front();
                    }
                    average_time = 0;
                    for (int i = 0; i < packet_times.size(); i++)
                    {
                        average_time += packet_times[i];
                    }
                    average_time = average_time / packet_times.size();
                }
                last_time = time;
                _readerThreadMutex.unlock();
                _parser_mutex.unlock();
                break;
            }
        }
        else
        {
            while (1)
            {
                // Checking to see if the buffer can still have a packet header in it
                if (_unparsed_stream_data.size() < packet_start + _responseHeaderSize + 3)
                {
                    if (foundAPacket)
                    {
                        _readerThreadMutex.lock();
                        _streamBuffer.push_back(packet);
                        clock_t time = clock();
                        if (last_time != 0)
                        {
                            float deltaTime = (time - last_time) / (CLOCKS_PER_SEC / 1000.0f);
                            packet_times.push_back(deltaTime);
                            if (packet_times.size() > 10)
                            {
                                packet_times.pop_front();
                            }
                            average_time = 0;
                            for (int i = 0; i < packet_times.size(); i++)
                            {
                                average_time += packet_times[i];
                            }
                            average_time = average_time / packet_times.size();
                        }
                        last_time = time;
                        _readerThreadMutex.unlock();
                    }
                    _parser_mutex.unlock();
                    break;
                }

                U8 currentPattern[3];
                currentPattern[0] = _unparsed_stream_data[packet_start];
                currentPattern[1] = _unparsed_stream_data[packet_start + 1];
                currentPattern[2] = _unparsed_stream_data[packet_start + 2];

                if (currentPattern[0] != 0 || currentPattern[1] != 255 || currentPattern[2] != 255)
                {
                    packet_start++;
                    continue;
                }
                packet_start += 3;

                vector<U8>::const_iterator first = _unparsed_stream_data.begin() + packet_start;
                vector<U8>::const_iterator last = _unparsed_stream_data.begin() + packet_start + _responseHeaderSize;
                vector<U8> buff(first, last);

                if (_responseHeaderFlags & PRIO_RESPONSE_HEADER_TIMESTAMP)
                {
                    U8 checksum = 0;
                    for (int i = 3; i < buff.size() - 1; i++)
                    {
                        checksum += buff[i];
                    }

                    //Check the checksum to see if this is indeed a packet header
                    if (!checksum == buff[buff.size() - 1])
                    {
                        packet_start++;
                        continue;
                    }
                }
                else
                {
                    U8 checksum = 0;
                    for (int i = 0; i < buff.size() - 1; i++)
                    {
                        checksum += buff[i];
                    }

                    //Check the checksum to see if this is indeed a packet header
                    if (!(checksum == buff[buff.size() - 1]))
                    {
                        packet_start++;
                        continue;
                    }
                }

                _prioParseStreamData(buff, (U8)_responseHeaderFlags, &header);

                //Parse active sensors to figure out our header size
                if (header.SensorBitfield != _enumeration_bitfield)
                {
                    _active_nodes.clear();
                    uint64_t bit;
                    uint8_t idx = 0;
                    for (bit = 1; bit < (1LL << 43); bit <<= 1)
                    {
                        if (header.SensorBitfield & bit)
                        {
                            _active_nodes.push_back(idx);
                        }
                        idx++;
                    }
                    U8 joystick1ID = (header.SensorBitfield & 71776119061217280) >> 48;
                    memcpy(&_joystick_1_id, &joystick1ID, 1);

                    U8 joystick2ID = (header.SensorBitfield & 18374686479671623680) >> 56;
                    memcpy(&_joystick_2_id, &joystick2ID, 1);

                    if ((6 < joystick1ID && joystick1ID < 14) || (20 < joystick1ID && joystick1ID < 28) || (34 < joystick1ID && joystick1ID < 42))
                    {
                        swapStreamJoystick = false;
                    }
                    else if ((6 < joystick2ID && joystick2ID < 14) || (20 < joystick2ID && joystick2ID < 28) || (34 < joystick2ID && joystick2ID < 42))
                    {
                        swapStreamJoystick = true;
                    }
                    else if ((0 < joystick1ID && joystick1ID < 7) || (13 < joystick1ID && joystick1ID < 19) || (20 < joystick1ID && joystick1ID < 28))
                    {
                        swapStreamJoystick = true;
                    }
                    else
                    {
                        swapStreamJoystick = false;
                    }

                    _enumeration_bitfield = header.SensorBitfield;
                }

                if (swapStreamJoystick)
                {
                    PrioJoystick tmp;
                    memcpy(&tmp, &header.Joystick1, sizeof(PrioJoystick));

                    memcpy(&header.Joystick1, &header.Joystick2, sizeof(PrioJoystick));
                    memcpy(&header.Joystick2, &tmp, sizeof(PrioJoystick));
                }

                stream_data_size = _streamDataSize*getActiveSensors().size();
                //If not enough data return
                if (_unparsed_stream_data.size() < packet_start + stream_data_size + _responseHeaderSize)
                {
                    if (foundAPacket)
                    {
                        _readerThreadMutex.lock();
                        _streamBuffer.push_back(packet);
                        clock_t time = clock();
                        if (last_time != 0)
                        {
                            float deltaTime = (time - last_time) / (CLOCKS_PER_SEC / 1000.0f);
                            packet_times.push_back(deltaTime);
                            if (packet_times.size() > 10)
                            {
                                packet_times.pop_front();
                            }
                            average_time = 0;
                            for (int i = 0; i < packet_times.size(); i++)
                            {
                                average_time += packet_times[i];
                            }
                            average_time = average_time / packet_times.size();
                        }
                        last_time = time;
                        _readerThreadMutex.unlock();
                    }
                    _parser_mutex.unlock();
                    break;
                }

                foundAPacket = true;
                first = _unparsed_stream_data.begin() + packet_start + _responseHeaderSize;
                last = _unparsed_stream_data.begin() + packet_start + _responseHeaderSize + stream_data_size;
                vector<U8> packet_buff(first, last);
                _unparsed_stream_data.erase(_unparsed_stream_data.begin(), last);

                memcpy(&packet.header, &header, sizeof(PrioHeader));
                memcpy(packet.rawData, &packet_buff[0], packet_buff.size());

                packet.rawDataSize = stream_data_size;

                packet_start = 0;
            }
        }
    }
}

float ReverseFloat(const float inFloat)
{
    float retVal;
    char *floatToConvert = (char*)& inFloat;
    char *returnFloat = (char*)& retVal;

    // swap the bytes into a temporary buffer
    returnFloat[0] = floatToConvert[3];
    returnFloat[1] = floatToConvert[2];
    returnFloat[2] = floatToConvert[1];
    returnFloat[3] = floatToConvert[0];

    return retVal;
}

PrioHub::PrioHub()
{
    _type = PRIO_DEVICE_TYPE_HUB;

    _ownerDongle = NULL;
    _streamInterval = 0;
    _streamDataSize = 0;
    _portName = new char[64];
    _breakParseThread = false;
    _unparsed_stream_data.empty();
    last_time = 0;
}

PrioHub::PrioHub(const char* port)
{
    _type = PRIO_DEVICE_TYPE_HUB;

    _ownerDongle = NULL;
    _streamInterval = 0;
    _streamDataSize = 0;
    _portName = new char[64];
    _breakParseThread = false;
    _unparsed_stream_data.empty();
    prio_error = openPort(port);

    if (prio_error != PRIO_NO_ERROR)
    {
        closePort();
        return;
    }

    gPrioAPI._commandThreadMutex.lock();
    _sendCommand(86); //stop streaming
    gPrioAPI._commandThreadMutex.unlock();

    disableTimestampsWired();

    _setResponseHeader(0);

    _setWirelessResponseHeader(PRIO_RESPONSE_HEADER_COMMAND_ECHO | PRIO_RESPONSE_HEADER_LOGICAL_ID | PRIO_RESPONSE_HEADER_SUCCESS);

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
    _setupEnumeratorBitfield();

    if (_sensorType == PRIO_BS)
    {
        closePort();
        return;
    }
}

PrioHub::PrioHub(const PrioHub& other)
{
    throw runtime_error("Deep copies of sensor objects not allowed.");
}

PrioHub::~PrioHub()
{
    _breakParseThread = true;
    stopStreamingWired();

    if (_port && _isportOwner)
    {
        _breakPortThread = true;
        if (_portThread.joinable())
        {
            _portThread.join();
        }
        _port = NULL;
    }
}

void PrioHub::operator =(const PrioHub& other)
{
    throw runtime_error("Deep copies of sensor objects not allowed.");
}

PRIO_ERROR PrioHub::enableStreamingWireless(U32 data_flags, U32 interval, U32 duration, U32 delay)
{
    BASIC_CALL_CHECK();
    
    return _prepareStreaming(data_flags, interval, duration, delay);
}

PRIO_ERROR PrioHub::disableStreamingWireless()
{
    BASIC_CALL_CHECK();

    _streamDataSize = 0; //the only variable the dongle checks to see if a sensor wants to stream, set to 0 to exclude it

    return PRIO_NO_ERROR;
}

void PrioHub::_addToStreamingSlots(U8 command, U8 nbytes, U8* command_buff, U8& curr_slot)
{
    if (curr_slot < PRIO_NUM_STREAMING_SLOTS) //only add command to a slot if slots remain
    {
        command_buff[curr_slot + 1] = command;
        _streamingSlots[curr_slot] = command;
        _streamDataSize += nbytes;
        curr_slot++;
    }
}

void PrioHub::_setupEnumeratorBitfield()
{
    // Reset active nodes list
    _active_nodes.clear();

    U64 enum_bitfield = 0;

    for (int i = 0; i < PRIO_STOP_STREAM_RETRIES; i++)
    {
        PRIO_ERROR error = getHubEnumerationValue(&enum_bitfield);
        U64 mask = 255;
        U64 pass = enum_bitfield & mask;

        U64 tmp = enum_bitfield & 255;
        tmp = tmp | (enum_bitfield >> 1 & 32640);
        tmp = tmp | (enum_bitfield >> 2 & 4177920);
        tmp = tmp | (enum_bitfield >> 3 & 534773760);
        tmp = tmp | (enum_bitfield >> 4 & 68451041280);
        tmp = tmp | (enum_bitfield >> 5 & 8761733283840);

        //Get the joystick indexs
        tmp = tmp | (enum_bitfield & 71776119061217280);
        tmp = tmp | (enum_bitfield & 18374686479671623680);

        enum_bitfield = tmp;

        if (error == PRIO_NO_ERROR)
        {
            uint64_t bit;
            uint8_t idx = 0; 
            for (bit = 1; bit < (1LL << 43); bit <<= 1)
            {
                if (enum_bitfield & bit)
                {
                    _active_nodes.push_back(idx);
                }
                idx++;
            }
            _enumeration_bitfield = enum_bitfield;
            prio_stream_commands stream_command = PRIO_STREAM_UNTARED_ORIENTATION_AS_QUATERNION;
            setStreamingSlots(stream_command);
            break;
        }
    }
}

PRIO_ERROR PrioHub::_prepareStreaming(prio_stream_commands data_flags, U32 interval, U32 duration, U32 delay)
{
    if (_isStreaming)
    {
        return PRIO_ERROR_COMMAND_FAIL;
    }

    U8 curr_slot = 0;
    U8 buff[256];

    //initialize streaming slots based on data flags
    //also initialize the streaming data size so mass reads can be performed
    _streamBuffer.clear();
    _streamDataSize = 0;
    buff[0] = 80; //set streaming slots command

    // Find the order of joystick data in the stream header
    U8 button_state = 0;
    U8 data[4];
    U8 data2[4];
    getAllControllerData(&_joystick_1_id, &_joystick_2_id, data, data2, &button_state);

    if ((6 < _joystick_1_id && _joystick_1_id < 14) || (20 < _joystick_1_id && _joystick_1_id < 28) || (34 < _joystick_1_id && _joystick_1_id < 42))
    {
        swapStreamJoystick = false;
    }
    else if ((6 < _joystick_2_id && _joystick_2_id < 14) || (20 < _joystick_2_id && _joystick_2_id < 28) || (34 < _joystick_2_id && _joystick_2_id < 42))
    {
        swapStreamJoystick = true;
    }
    else if ((0 < _joystick_1_id && _joystick_1_id < 7) || (13 < _joystick_1_id && _joystick_1_id < 19) || (20 < _joystick_1_id && _joystick_1_id < 28))
    {
        swapStreamJoystick = true;
    }
    else
    {
        swapStreamJoystick = false;
    }

    //go through the flags one by one in order and put them in slots
    //we rely on the numerical values of the flags to determine what order
    //they are assigned to slots in

    if (data_flags & PRIO_STREAM_TARED_ORIENTATION_AS_QUATERNION)
    {
        _addToStreamingSlots(0x00, 16, buff, curr_slot);
    }
    if (data_flags & PRIO_STREAM_UNTARED_ORIENTATION_AS_QUATERNION)
    {
        _addToStreamingSlots(0x06, 16, buff, curr_slot);
    }
    if (data_flags & PRIO_STREAM_ALL_CORRECTED_COMPONENT_SENSOR_DATA)
    {
        _addToStreamingSlots(0x25, 36, buff, curr_slot);
    }
    if (data_flags & PRIO_STREAM_CORRECTED_GYRO_RATE)
    {
        _addToStreamingSlots(0x26, 12, buff, curr_slot);
    }
    if (data_flags & PRIO_STREAM_CORRECTED_ACCELEROMETER_VECTOR)
    {
        _addToStreamingSlots(0x27, 12, buff, curr_slot);
    }
    if (data_flags & PRIO_STREAM_CORRECTED_MAGNETOMETER_VECTOR)
    {
        _addToStreamingSlots(0x28, 12, buff, curr_slot);
    }
    if (data_flags & PRIO_STREAM_ALL_RAW_COMPONENT_SENSOR_DATA)
    {
        _addToStreamingSlots(0x40, 36, buff, curr_slot);
    }
    if (data_flags & PRIO_STREAM_RAW_GYROSCOPE_RATE)
    {
        _addToStreamingSlots(0x41, 12, buff, curr_slot);
    }
    if (data_flags & PRIO_STREAM_RAW_ACCELEROMETER_DATA)
    {
        _addToStreamingSlots(0x42, 12, buff, curr_slot);
    }
    if (data_flags & PRIO_STREAM_RAW_MAGNETOMETER_DATA)
    {
        _addToStreamingSlots(0x43, 12, buff, curr_slot);
    }

    //fill out any unused slots with blank commands
    while (curr_slot < 8)
    {
        buff[curr_slot + 1] = 0xff;
        _streamingSlots[curr_slot] = 0xff;
        curr_slot++;
    }
    gPrioAPI._commandThreadMutex.lock();
    PRIO_RETRY(_sendCommandBytes(buff, 9), PRIO_START_STREAM_RETRIES);

    if (prio_error != PRIO_NO_ERROR)
        return prio_error;

    //record the streaming interval, or a guess at the interval
    if (interval == 0)
        _streamInterval = 10000;
    else
        if (interval < 10000)
            _streamInterval = 10000;
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

    PRIO_RETRY(_sendCommandBytes(buff, 5), PRIO_START_STREAM_RETRIES);
    gPrioAPI._commandThreadMutex.unlock();

    if (prio_error != PRIO_NO_ERROR)
        return prio_error;

    return PRIO_NO_ERROR;
}

PRIO_ERROR PrioHub::_triggerStreaming()
{
    _streamBuffer.clear();

    //start streaming
    gPrioAPI._commandThreadMutex.lock();
    PRIO_RETRY(_sendCommand(85,true), PRIO_START_STREAM_RETRIES);
    gPrioAPI._commandThreadMutex.unlock();

    if (prio_error != PRIO_NO_ERROR)
        return prio_error;

    _isStreaming = true;

    return PRIO_NO_ERROR;
}

PRIO_ERROR PrioHub::openPort(const char* port)
{
    if (_port)
    {
        return PRIO_ERROR_COMMAND_FAIL;
    }

    return _openPort(port);
}

PRIO_ERROR PrioHub::closePort()
{
    if (_isWireless)
    {
        return PRIO_ERROR_COMMAND_FAIL;
    }

    BASIC_CALL_CHECK();

    _breakPortThread = true;
    if (_portThread.joinable())
    {
        _portThread.join();
    }
    _port = NULL;

    return PRIO_NO_ERROR;
}

PRIO_ERROR PrioHub::enableTimestampsWired()
{
    BASIC_CALL_CHECK();

    U8 buff[2];
    buff[0] = 0xDB;
    buff[1] = 1;

    gPrioAPI._commandThreadMutex.lock();
    PRIO_COMMAND_CHECK(_sendCommandBytes(buff, 2, true));

    _purgeInput();
    gPrioAPI._commandThreadMutex.unlock();

    _setResponseHeader(_responseHeaderFlags | PRIO_RESPONSE_HEADER_TIMESTAMP);

    return PRIO_NO_ERROR;
}

PRIO_ERROR PrioHub::disableTimestampsWired()
{
    BASIC_CALL_CHECK();

    U8 buff[2];
    buff[0] = 0xDB;
    buff[1] = 0;

    gPrioAPI._commandThreadMutex.lock();
    PRIO_COMMAND_CHECK(_sendCommandBytes(buff, 2, true));

    _purgeInput();
    gPrioAPI._commandThreadMutex.unlock();

    _setResponseHeader(_responseHeaderFlags & ~PRIO_RESPONSE_HEADER_TIMESTAMP);

    return PRIO_NO_ERROR;
}

PRIO_ERROR PrioHub::startStreamingWired(prio_stream_commands data_flags, U32 interval, U32 duration, U32 delay, bool reg)
{
    if (_isWireless)
    {
        return PRIO_ERROR_COMMAND_FAIL;
    }

    PRIO_ERROR_CHECK(_prepareStreaming(data_flags, interval, duration, delay));

    PRIO_ERROR_CHECK(_triggerStreaming());

    _setResponseHeader(0);

    _breakParseThread = false;
    _unparsed_stream_data.empty();
    _parser_thread = std::thread(&PrioHub::_parseStreamData, this);


    //register this device with the overall module so it can assign threading to it
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

PRIO_ERROR PrioHub::stopStreamingWired(bool dereg)
{
    if (!_port)
    {
        return PRIO_ERROR_COMMAND_FAIL;
    }

    if (_isWireless)
    {
        return PRIO_INVALID_COMMAND;
    }

    if (!_isStreaming)
    {
        return PRIO_ERROR_COMMAND_FAIL_COMMUNICATION;
    }

    if (dereg)
    {
        gPrioAPI.unregisterStreamingDevice(this);
    }
    else
    {
        gPrioAPI.pauseStreamingDevice(this);
    }

    gPrioAPI._commandThreadMutex.lock();
    prio_error = _sendCommand(86); //stop streaming command
    gPrioAPI._commandThreadMutex.unlock();

    _breakParseThread = true;
    if (_parser_thread.joinable())
    {
        _parser_thread.join();
    }
    _unparsed_stream_data.empty();
    _isStreaming = false;

    _setResponseHeader(0);

    yost::sleep_ms(100); //Sleep to finish up streaming


    return prio_error;
}

void PrioHub::_parseStreamingPacketItem(U8* dest, U8* src, U16& src_index, U32 nbytes)
{
    memcpy(dest, src + src_index, nbytes);
    src_index += nbytes;
}

void PrioHub::_parseStreamingPacketItemFloats(float* dest, U8* src, U16& src_index, U16 nfloats)
{
    _parseFloats((float*) (src+src_index), nfloats);
    _parseStreamingPacketItem((U8*)dest, src, src_index, nfloats * 4);
}

PRIO_ERROR PrioHub::_parseStreamingPacket(PrioStreamPacket* packet)
{
    U16 raw_data_index = 0;
    uint32_t data_size = 0;

    U8 i;
    for (i = 0; i < 8; i++)
    {
        if (_streamingSlots[i] == 0xff)
            break;

        //check through the slots one by one and pull out the raw data that belongs to that slot
        if (_streamingSlots[i] == 0x00)
        {
            data_size = 4 * _active_nodes.size();
            float* data = new float[data_size];
            _parseStreamingPacketItemFloats(data, packet->rawData, raw_data_index, data_size);
        }
        else if (_streamingSlots[i] == 0x06)
        {
            data_size = 4 * _active_nodes.size();
            float* data = new float[data_size];

            _parseStreamingPacketItemFloats(data, packet->rawData, raw_data_index, data_size);
        }
        else if (_streamingSlots[i] == 0x25)
        {
            data_size = 9 * _active_nodes.size();
            float* data = new float[data_size];
            _parseStreamingPacketItemFloats(data, packet->rawData, raw_data_index, data_size);
        }
        else if (_streamingSlots[i] == 0x26)
        {
            data_size = 3 * _active_nodes.size();
            float* data = new float[data_size];
            _parseStreamingPacketItemFloats(data, packet->rawData, raw_data_index, data_size);
        }
        else if (_streamingSlots[i] == 0x27)
        {
            data_size = 3 * _active_nodes.size();
            float* data = new float[data_size];
            _parseStreamingPacketItemFloats(data, packet->rawData, raw_data_index, data_size);
        }
        else if ( _streamingSlots[i] == 0x28)
        {
            data_size = 3 * _active_nodes.size();
            float* data = new float[data_size];
            _parseStreamingPacketItemFloats(data, packet->rawData, raw_data_index, data_size);
        }
        else if (_streamingSlots[i] == 0x40)
        {
            data_size = 9 * _active_nodes.size();
            float* data = new float[data_size];
            _parseStreamingPacketItemFloats(data, packet->rawData, raw_data_index, data_size);
        }
        else if (_streamingSlots[i] == 0x41)
        {
            data_size = 3 * _active_nodes.size();
            float* data = new float[data_size];
            _parseStreamingPacketItemFloats(data, packet->rawData, raw_data_index, data_size);
        }
        else if (_streamingSlots[i] == 0x42)
        {
            data_size = 3 * _active_nodes.size();
            float* data = new float[data_size];
            _parseStreamingPacketItemFloats(data, packet->rawData, raw_data_index, data_size);
        }
        else if(_streamingSlots[i] == 0x43)
        {
            data_size = 3 * _active_nodes.size();
            float* data = new float[data_size];
            _parseStreamingPacketItemFloats(data, packet->rawData, raw_data_index, data_size);
        }
    }

    return PRIO_NO_ERROR;
}

PRIO_ERROR PrioHub::getFirstStreamingPacket(PrioStreamPacket* packet)
{

    _readerThreadMutex.lock(); //lock the reading thread so it won't mess with our buffer

    if (_streamBuffer.size() == 0)
    {
        _readerThreadMutex.unlock();
        return PRIO_ERROR_READ;
    }

    // Get the first packet
    *packet = _streamBuffer.front();
    // Remove the first packet
    _streamBuffer.pop_front();

    _readerThreadMutex.unlock();

    //parse the streaming packet out into member variables
    prio_error = _parseStreamingPacket(packet);

    return prio_error;
}

PRIO_ERROR PrioHub::getLastStreamingPacket(PrioStreamPacket* packet)
{
    _readerThreadMutex.lock(); //lock the reading thread so it won't mess with our buffer

    if (_streamBuffer.size() == 0)
    {
        _readerThreadMutex.unlock();
        return PRIO_ERROR_READ;
    }

    // Get the last packet
    *packet = _streamBuffer.back();
    // Clear all the packets
    _streamBuffer.clear();

    _readerThreadMutex.unlock();

    //parse the streaming packet out into member variables
    prio_error = _parseStreamingPacket(packet);

    return PRIO_NO_ERROR;
}

U32 PrioHub::getFullLengthOfStreamData()
{
    return (U32)_streamDataSize*(getActiveSensors().size());
}

U32 PrioHub::getStreamingPacketsInWaiting()
{
    return _streamBuffer.size();
}

bool PrioHub::didStreamingOverflow()
{
    return _streamBuffer.size() == _streamBuffer.max_size();
}

float PrioHub::getStreamUpdateRate()
{
    return average_time;
}

PRIO_ERROR PrioHub::startRecordingWired(prio_stream_commands data_flags, U32 interval, U32 duration, U32 delay, bool reg)
{
    PRIO_ERROR_CHECK(startStreamingWired(data_flags,interval,duration,delay, reg));
    PRIO_ERROR_CHECK(enableTimestampsWired());
    _isRecording = true;

    return PRIO_NO_ERROR;
}

PRIO_ERROR PrioHub::stopRecordingWired(bool dereg)
{
    PRIO_ERROR_CHECK(stopStreamingWired(dereg));
    PRIO_ERROR_CHECK(disableTimestampsWired());
    _isRecording = false;

    return PRIO_NO_ERROR;
}

PRIO_ERROR PrioHub::disableRecordingWireless()
{
    PRIO_ERROR_CHECK(disableStreamingWireless());
    _isRecording = false;

    return PRIO_NO_ERROR;
}

void PrioHub::setupRecordingOptions(U32 buffer_size, bool is_wrapping)
{
    if (buffer_size < _streamBuffer.max_size())
    {
        _maxRecordingCount = buffer_size;
    }
    else
    {
        _maxRecordingCount = _streamBuffer.max_size();
    }

    _wrappingMode = is_wrapping;
}

U32 PrioHub::getMaxRecordedSamples()
{
    return _maxRecordingCount;
}

bool PrioHub::isBufferWrapping()
{
    return _wrappingMode;
}

U32 PrioHub::getLengthOfRecordedSamples()
{
    return _streamBuffer.size();
}

PRIO_ERROR PrioHub::getRecordedSamples(PrioHeader *header_data, U8* packet_data, U32 packet_count)
{
    if (_streamBuffer.size() < packet_count)
    {
        return PRIO_ERROR_PARAMETER;
    }

    _readerThreadMutex.lock();
    for (U32 i = 0; i < packet_count; i++)
    {
        PrioStreamPacket tmp = _streamBuffer[i];

        prio_error = _parseStreamingPacket(&tmp);

        memcpy(header_data+i, &tmp.header, sizeof(PrioHeader));
        memcpy(packet_data + i*(_streamDataSize*_active_nodes.size()), &tmp.rawData, (_streamDataSize*_active_nodes.size()));
    }
    _readerThreadMutex.unlock();

    return PRIO_NO_ERROR;
}

PRIO_ERROR PrioHub::getRecordedSampleAtIndex(PrioHeader *header_data, U8* packet_data, U32 index)
{
    if (_streamBuffer.size() < index)
    {
        return PRIO_ERROR_PARAMETER;
    }

    _readerThreadMutex.lock();
    PrioStreamPacket tmp = _streamBuffer[index];
    _readerThreadMutex.unlock();

    prio_error = _parseStreamingPacket(&tmp);

    memcpy(header_data, &tmp.header, sizeof(PrioHeader));
    memcpy(packet_data, &tmp.rawData, (_streamDataSize*_active_nodes.size()));
    return PRIO_NO_ERROR;
}

PRIO_ERROR PrioHub::popFrontRecordedSample(PrioHeader *header_data, U8* packet_data)
{
    if (_streamBuffer.size() < 1)
    {
        return PRIO_ERROR_PARAMETER;
    }

    _readerThreadMutex.lock();
    PrioStreamPacket tmp = _streamBuffer.front();
    _streamBuffer.pop_front();
    _readerThreadMutex.unlock();
    prio_error = _parseStreamingPacket(&tmp);

    memcpy(header_data, &tmp.header, sizeof(PrioHeader));
    memcpy(packet_data, &tmp.rawData, (_streamDataSize*_active_nodes.size()));

    return PRIO_NO_ERROR;
}


void PrioHub::clearRecordedSamples()
{
    _streamBuffer.clear();
}

std::vector<U8> PrioHub::getActiveSensors()
{
    return _active_nodes;
}

void PrioHub::setCommandRetries(U8 retries)
{
    _commandRetries = retries;
}

PRIO_ERROR PrioHub::setStreamingSlots(prio_stream_commands stream_slots)
{
    BASIC_CALL_CHECK();

    _prepareStreaming(stream_slots, 0, 0);
    return PRIO_NO_ERROR;
}

PRIO_ERROR PrioHub::getStreamingSlots(prio_stream_commands* stream_slots)
{
    BASIC_CALL_CHECK();

    gPrioAPI._commandThreadMutex.lock();
    PRIO_COMMAND_CHECK(_checkedCommandWriteRead(81, (U8*)stream_slots, 4));
    gPrioAPI._commandThreadMutex.unlock();

    return PRIO_NO_ERROR;
}

PRIO_ERROR PrioHub::enumerateHub()
{
    BASIC_CALL_CHECK();

    //Command always returns failure over wireless.
    gPrioAPI._commandThreadMutex.lock();
    _sendCommand(0xB0,_logicalId);
    gPrioAPI._commandThreadMutex.unlock();

    yost::sleep_ms(2000);
    _setupEnumeratorBitfield();

    return PRIO_NO_ERROR;
}

PRIO_ERROR PrioHub::getHubEnumerationValue(U64* enumeration_value)
{
    BASIC_CALL_CHECK();

    gPrioAPI._commandThreadMutex.lock();
    PRIO_COMMAND_CHECK(_checkedCommandWriteRead(0xB1, (U8*)enumeration_value, 8));
    gPrioAPI._commandThreadMutex.unlock();

    yost::swap64((U8*)enumeration_value);

    return PRIO_NO_ERROR;
}

PRIO_ERROR PrioHub::getNumberOfEnumeratedSensors(U8* num_sensors)
{
    gPrioAPI._commandThreadMutex.lock();
    PRIO_COMMAND_CHECK(_checkedCommandWriteRead(0xB2, (U8*)num_sensors, 1));
    gPrioAPI._commandThreadMutex.unlock();

    return PRIO_NO_ERROR;
}

PRIO_ERROR PrioHub::getTaredOrientationAsQuaternion(U8 logical_id, float* quat, U8 num_floats)
{
    gPrioAPI._commandThreadMutex.lock();
    PRIO_COMMAND_CHECK(_checkedCommandWriteReadFloats(0x00, logical_id, quat, num_floats));
    gPrioAPI._commandThreadMutex.unlock();

    return PRIO_NO_ERROR;
}

PRIO_ERROR PrioHub::getUntaredOrientationAsQuaternion(U8 logical_id, float* quat, U8 num_floats )
{
    gPrioAPI._commandThreadMutex.lock();
    PRIO_COMMAND_CHECK(_checkedCommandWriteReadFloats(0x06, logical_id, quat, num_floats));
    gPrioAPI._commandThreadMutex.unlock();

    return PRIO_NO_ERROR;
}

PRIO_ERROR PrioHub::getControllerData(U8 index, U8* logical_id, U8* data)
{
    U8 buff[5];
    buff[0] = 0x0A;
    buff[1] = index;

    gPrioAPI._commandThreadMutex.lock();
    PRIO_COMMAND_CHECK(_sendCommandBytes(buff, 2));
    PRIO_COMMAND_CHECK(_readBytes(buff, 5));
    gPrioAPI._commandThreadMutex.unlock();

    *logical_id = buff[0];
    memcpy(data, buff+1, 4);

    return PRIO_NO_ERROR;
}

PRIO_ERROR PrioHub::getAllControllerData(U8* logical_id_1, U8* logical_id_2, U8* data_1, U8* data_2, U8* button_state)
{
    U8 buff[5];
    buff[0] = 0x0A;
    buff[1] = 0;
    PRIO_ERROR joystick1_error;
    PRIO_ERROR joystick2_error;
    PRIO_ERROR hub_error;

    gPrioAPI._commandThreadMutex.lock();

    joystick1_error = _sendCommandBytes(buff, 2);
    if (joystick1_error == PRIO_NO_ERROR)
    {
        joystick1_error = _readBytes((U8*)buff, 5);
        if (joystick1_error == PRIO_NO_ERROR)
        {
            *logical_id_1 = buff[0];
            _joystick_1_id = *logical_id_1;
            memcpy(data_1, buff + 1, 4);
        }
    }

    buff[0] = 0x0A;
    buff[1] = 1;
    joystick2_error = _sendCommandBytes(buff, 2);
    if (joystick2_error == PRIO_NO_ERROR)
    {
        joystick2_error = _readBytes((U8*)buff, 5);
        if (joystick2_error == PRIO_NO_ERROR)
        {
            *logical_id_2 = buff[0];
            _joystick_2_id = *logical_id_2;
            memcpy(data_2, buff + 1, 4);
        }
    }

    U8 button = 0;
    hub_error = _checkedCommandWriteRead(0xFA, &button, 1);
    if (hub_error == PRIO_NO_ERROR)
    {
        *button_state = button;
    }

    gPrioAPI._commandThreadMutex.unlock();

    //If all three failed return failure otherwise return a sucess
    if (hub_error && joystick1_error && joystick2_error)
    {
        return PRIO_ERROR_COMMAND_FAIL;
    }
    return PRIO_NO_ERROR;
}

PRIO_ERROR PrioHub::tareWithCurrentOrientation(U8 logical_id)
{
    BASIC_CALL_CHECK();

    gPrioAPI._commandThreadMutex.lock();
    PRIO_COMMAND_CHECK(_sendCommand(0x60, logical_id));
    gPrioAPI._commandThreadMutex.unlock();

    return PRIO_NO_ERROR;
}

PRIO_ERROR PrioHub::getWirelessChannel(U8* channel)
{
    if (_sensorType != PRIO_HUB)
    {
        return PRIO_INVALID_COMMAND;
    }

    gPrioAPI._commandThreadMutex.lock();
    PRIO_COMMAND_CHECK(_checkedCommandWriteRead(0xC2, channel, 1));
    gPrioAPI._commandThreadMutex.unlock();

    return PRIO_NO_ERROR;
}

PRIO_ERROR PrioHub::setWirelessChannel(U8 channel)
{
    if (_sensorType != PRIO_HUB)
    {
        return PRIO_INVALID_COMMAND;
    }

    BASIC_CALL_CHECK();

    U8 buff[2];
    buff[0] = 0xC3;
    buff[1] = channel;

    gPrioAPI._commandThreadMutex.lock();
    PRIO_COMMAND_CHECK(_sendCommandBytes(buff, 2));
    gPrioAPI._commandThreadMutex.unlock();

    return PRIO_NO_ERROR;
}

PRIO_ERROR PrioHub::getWirelessPanID(U16* panid)
{
    if (_sensorType != PRIO_HUB)
    {
        return PRIO_INVALID_COMMAND;
    }

    gPrioAPI._commandThreadMutex.lock();
    PRIO_COMMAND_CHECK(_checkedCommandWriteRead(0xC0, (U8*)panid, 2));
    gPrioAPI._commandThreadMutex.unlock();

    yost::swap16((U8*)panid);

    return PRIO_NO_ERROR;
}

PRIO_ERROR PrioHub::setWirelessPanID(U16 panid)
{
    if (_sensorType != PRIO_HUB)
    {
        return PRIO_INVALID_COMMAND;
    }

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

PRIO_ERROR PrioHub::getBatteryPercentRemaining(U8* battery_percent)
{
    BASIC_CALL_CHECK();

    if (_sensorType != PRIO_HUB)
    {
        return PRIO_INVALID_COMMAND;
    }

    gPrioAPI._commandThreadMutex.lock();
    PRIO_COMMAND_CHECK(_checkedCommandWriteRead(0xCA, battery_percent, 1));
    gPrioAPI._commandThreadMutex.unlock();

    return PRIO_NO_ERROR;
}

PRIO_ERROR PrioHub::getButtonState(U8* button_state)
{
    BASIC_CALL_CHECK();

    if (_sensorType != PRIO_HUB)
    {
        return PRIO_INVALID_COMMAND;
    }

    gPrioAPI._commandThreadMutex.lock();
    PRIO_COMMAND_CHECK(_checkedCommandWriteRead(0xFA, button_state, 1));
    gPrioAPI._commandThreadMutex.unlock();

    return PRIO_NO_ERROR;
}

PRIO_ERROR PrioHub::commitSettings()
{
    BASIC_CALL_CHECK();

    gPrioAPI._commandThreadMutex.lock();
    PRIO_COMMAND_CHECK(_sendCommand(0xE1));
    gPrioAPI._commandThreadMutex.unlock();

    return PRIO_NO_ERROR;
}

PRIO_ERROR PrioHub::getSerialNumberAtLogicalID(U8 logical_id, U32* serial_number)
{
    BASIC_CALL_CHECK();

    gPrioAPI._commandThreadMutex.lock();

    PRIO_COMMAND_CHECK(_sendCommand(237, logical_id));

    PRIO_COMMAND_CHECK(_readBytes((U8*)serial_number, 4));
    gPrioAPI._commandThreadMutex.unlock();

    yost::swap32((U8*)serial_number);

    return PRIO_NO_ERROR;
}

PRIO_ERROR PrioHub::getFirmwareVersionAtLogicalID(U8 logical_id, const char* firmware_version)
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

PRIO_ERROR PrioHub::getHardwareVersionAtLogicalID(U8 logical_id, const char* hardware_version)
{
    gPrioAPI._commandThreadMutex.lock();
    PRIO_COMMAND_CHECK(_sendCommand(230, logical_id));

    U8 buff[33];
    PRIO_COMMAND_CHECK(_readBytes(buff, 32));
    gPrioAPI._commandThreadMutex.unlock();
    buff[32] = '\0';

    memcpy((char*)hardware_version, buff, 33);

    return PRIO_NO_ERROR;
}

PRIO_ERROR PrioHub::setStreamingInterval(U32 interval)
{
    BASIC_CALL_CHECK();

    if (_sensorType != PRIO_HUB)
    {
        return PRIO_INVALID_COMMAND;
    }

    U8 buff[5];
    buff[0] = 0x52;

    memcpy(buff + 1, &interval, 4);
    yost::swap32(buff + 1);

    gPrioAPI._commandThreadMutex.lock();
    PRIO_COMMAND_CHECK(_sendCommandBytes(buff, 5));
    gPrioAPI._commandThreadMutex.unlock();

    return PRIO_NO_ERROR;
}

PRIO_ERROR PrioHub::getStreamingInterval(U32* interval)
{
    BASIC_CALL_CHECK();

    if (_sensorType != PRIO_HUB)
    {
        return PRIO_INVALID_COMMAND;
    }

    gPrioAPI._commandThreadMutex.lock();
    PRIO_COMMAND_CHECK(_checkedCommandWriteRead(0x53, (U8*)interval, 4));
    gPrioAPI._commandThreadMutex.unlock();

    yost::swap32((U8*)interval);
    return PRIO_NO_ERROR;
}

PRIO_ERROR PrioHub::resetAllSensors()
{
    BASIC_CALL_CHECK();

    if (_sensorType != PRIO_HUB)
    {
        return PRIO_INVALID_COMMAND;
    }

    gPrioAPI._commandThreadMutex.lock();
    PRIO_COMMAND_CHECK(_sendCommand(0xB3));
    gPrioAPI._commandThreadMutex.unlock();

    return PRIO_NO_ERROR;
}

PRIO_ERROR PrioHub::powerDownAllSensors()
{
    BASIC_CALL_CHECK();

    if (_sensorType != PRIO_HUB)
    {
        return PRIO_INVALID_COMMAND;
    }

    gPrioAPI._commandThreadMutex.lock();
    PRIO_COMMAND_CHECK(_sendCommand(0xB4));
    gPrioAPI._commandThreadMutex.unlock();

    return PRIO_NO_ERROR;
}

PRIO_ERROR PrioHub::powerUpAllSensors()
{
    BASIC_CALL_CHECK();

    if (_sensorType != PRIO_HUB)
    {
        return PRIO_INVALID_COMMAND;
    }

    gPrioAPI._commandThreadMutex.lock();
    PRIO_COMMAND_CHECK(_sendCommand(0xB5));
    gPrioAPI._commandThreadMutex.unlock();

    return PRIO_NO_ERROR;
}

PRIO_ERROR PrioHub::setAutoEnumerationMode(bool enable)
{
    BASIC_CALL_CHECK();

    if (_sensorType != PRIO_HUB)
    {
        return PRIO_INVALID_COMMAND;
    }

    U8 buff[2];
    buff[0] = 0xB7;
    buff[1] = enable;

    gPrioAPI._commandThreadMutex.lock();
    PRIO_COMMAND_CHECK(_sendCommandBytes(buff, 2));
    gPrioAPI._commandThreadMutex.unlock();

    return PRIO_NO_ERROR;
}

PRIO_ERROR PrioHub::getAutoEnumerationMode(bool* enabled)
{
    BASIC_CALL_CHECK();

    if (_sensorType != PRIO_HUB)
    {
        return PRIO_INVALID_COMMAND;
    }
    
    gPrioAPI._commandThreadMutex.lock();
    PRIO_COMMAND_CHECK(_checkedCommandWriteRead(0xB8, (U8*)enabled, 1));
    gPrioAPI._commandThreadMutex.unlock();

    return PRIO_NO_ERROR;
}

PRIO_ERROR PrioHub::getWirelessAddress(U16* address)
{
    BASIC_CALL_CHECK();

    if (_sensorType != PRIO_HUB)
    {
        return PRIO_INVALID_COMMAND;
    }

    gPrioAPI._commandThreadMutex.lock();
    PRIO_COMMAND_CHECK(_checkedCommandWriteRead(0xC6, (U8*)address, 2));
    gPrioAPI._commandThreadMutex.unlock();

    yost::swap16((U8*)address);

    return PRIO_NO_ERROR;
}

PRIO_ERROR PrioHub::getBatteryStatus(U8* status)
{
    BASIC_CALL_CHECK();

    if (_sensorType != PRIO_HUB)
    {
        return PRIO_INVALID_COMMAND;
    }

    gPrioAPI._commandThreadMutex.lock();
    PRIO_COMMAND_CHECK(_checkedCommandWriteRead(0xCB, (U8*)status, 1));
    gPrioAPI._commandThreadMutex.unlock();

    return PRIO_NO_ERROR;
}

PRIO_ERROR PrioHub::setPerformanceMode(bool enable)
{
    if (_sensorType != PRIO_HUB)
    {
        return PRIO_INVALID_COMMAND;
    }

    BASIC_CALL_CHECK();

    U8 buff[2];
    buff[0] = 0xE3;

    memcpy(buff + 1, &enable, 1);

    gPrioAPI._commandThreadMutex.lock();
    PRIO_COMMAND_CHECK(_sendCommandBytes(buff, 2));
    gPrioAPI._commandThreadMutex.unlock();

    return PRIO_NO_ERROR;
}

PRIO_ERROR PrioHub::getPerformanceMode(bool* enabled)
{
    BASIC_CALL_CHECK();

    if (_sensorType != PRIO_HUB)
    {
        return PRIO_INVALID_COMMAND;
    }

    gPrioAPI._commandThreadMutex.lock();
    PRIO_COMMAND_CHECK(_checkedCommandWriteRead(0xE4, (U8*)enabled, 1));
    gPrioAPI._commandThreadMutex.unlock();

    return PRIO_NO_ERROR;
}

PRIO_ERROR PrioHub::getCorrectedSensorData(U8 logical_id, float* gyroscope, float* accelrometer, float* magnetometer)
{
    BASIC_CALL_CHECK();

    float data[9];

    gPrioAPI._commandThreadMutex.lock();
    PRIO_COMMAND_CHECK(_checkedCommandWriteReadFloats(0x25, logical_id, &data[0], 9));
    gPrioAPI._commandThreadMutex.unlock();

    memcpy(gyroscope, &data[0], 3 * sizeof(float));
    memcpy(accelrometer, &data[3], 3 * sizeof(float));
    memcpy(magnetometer, &data[6], 3 * sizeof(float));

    return PRIO_NO_ERROR;
}

PRIO_ERROR PrioHub::getCorrectedGyroscope(U8 logical_id, float* gyroscope)
{
    BASIC_CALL_CHECK();

    float data[3];
    gPrioAPI._commandThreadMutex.lock();
    PRIO_COMMAND_CHECK(_checkedCommandWriteReadFloats(0x26, logical_id, &data[0], 3));
    gPrioAPI._commandThreadMutex.unlock();

    memcpy(gyroscope, data, 3 * sizeof(float));

    return PRIO_NO_ERROR;
}

PRIO_ERROR PrioHub::getCorrectedAccelerometer(U8 logical_id, float* accelerometer)
{
    BASIC_CALL_CHECK();

    float data[3];
    gPrioAPI._commandThreadMutex.lock();
    PRIO_COMMAND_CHECK(_checkedCommandWriteReadFloats(0x27, logical_id, &data[0], 3));
    gPrioAPI._commandThreadMutex.unlock();

    memcpy(accelerometer, data, 3 * sizeof(float));

    return PRIO_NO_ERROR;
}

PRIO_ERROR PrioHub::getCorrectedMagnetometer(U8 logical_id, float* magnetometer)
{
    BASIC_CALL_CHECK();

    float data[3];
    gPrioAPI._commandThreadMutex.lock();
    PRIO_COMMAND_CHECK(_checkedCommandWriteReadFloats(0x28, logical_id, &data[0], 3));
    gPrioAPI._commandThreadMutex.unlock();

    memcpy(magnetometer, data, 3 * sizeof(float));

    return PRIO_NO_ERROR;
}

PRIO_ERROR PrioHub::getRawSensorData(U8 logical_id, float* gyroscope, float* accelrometer, float* magnetometer)
{
    BASIC_CALL_CHECK();

    float data[9];
    gPrioAPI._commandThreadMutex.lock();
    PRIO_COMMAND_CHECK(_checkedCommandWriteReadFloats(0x40, logical_id, &data[0], 9));
    gPrioAPI._commandThreadMutex.unlock();

    memcpy(gyroscope, &data[0], 3 * sizeof(float));
    memcpy(accelrometer, &data[3], 3 * sizeof(float));
    memcpy(magnetometer, &data[6], 3 * sizeof(float));

    return PRIO_NO_ERROR;
}

PRIO_ERROR PrioHub::getRawGyroscope(U8 logical_id, float* gyroscope)
{
    BASIC_CALL_CHECK();

    float data[3];
    gPrioAPI._commandThreadMutex.lock();
    PRIO_COMMAND_CHECK(_checkedCommandWriteReadFloats(0x41, logical_id, &data[0], 3));
    gPrioAPI._commandThreadMutex.unlock();

    memcpy(gyroscope, data, 3 * sizeof(float));

    return PRIO_NO_ERROR;
}

PRIO_ERROR PrioHub::getRawAccelerometer(U8 logical_id, float* accelerometer)
{
    BASIC_CALL_CHECK();

    float data[3];
    gPrioAPI._commandThreadMutex.lock();
    PRIO_COMMAND_CHECK(_checkedCommandWriteReadFloats(0x42, logical_id, &data[0], 3));
    gPrioAPI._commandThreadMutex.unlock();

    memcpy(accelerometer, data, 3 * sizeof(float));

    return PRIO_NO_ERROR;
}

PRIO_ERROR PrioHub::getRawMagnetometer(U8 logical_id, float* magnetometer)
{
    BASIC_CALL_CHECK();

    float data[3];
    gPrioAPI._commandThreadMutex.lock();
    PRIO_COMMAND_CHECK(_checkedCommandWriteReadFloats(0x43, logical_id, &data[0], 3));
    gPrioAPI._commandThreadMutex.unlock();

    memcpy(magnetometer, data, 3 * sizeof(float));

    return PRIO_NO_ERROR;
}

PRIO_ERROR PrioHub::setMagnetometerCalibParams(U8 logical_id, const float* scale9, const float* bias3)
{
    BASIC_CALL_CHECK();

    U8 buff[49];
    buff[0] = 0xA0;

    memcpy(buff + 1, scale9, 36);
    yost::swap32(buff + 1);
    yost::swap32(buff + 5);
    yost::swap32(buff + 9);
    yost::swap32(buff + 13);
    yost::swap32(buff + 17);
    yost::swap32(buff + 21);
    yost::swap32(buff + 25);
    yost::swap32(buff + 29);
    yost::swap32(buff + 33);

    memcpy(buff + 37, bias3, 12);
    yost::swap32(buff + 37);
    yost::swap32(buff + 41);
    yost::swap32(buff + 45);

    gPrioAPI._commandThreadMutex.lock();
    PRIO_COMMAND_CHECK(_sendCommandBytes(buff, 49, logical_id));
    gPrioAPI._commandThreadMutex.unlock();

    return PRIO_NO_ERROR;
}

PRIO_ERROR PrioHub::setAccelerometerCalibParams(U8 logical_id, const float* scale9, const float* bias3)
{
    BASIC_CALL_CHECK();

    U8 buff[49];
    buff[0] = 0xA1;

    memcpy(buff + 1, scale9, 36);
    yost::swap32(buff + 1);
    yost::swap32(buff + 5);
    yost::swap32(buff + 9);
    yost::swap32(buff + 13);
    yost::swap32(buff + 17);
    yost::swap32(buff + 21);
    yost::swap32(buff + 25);
    yost::swap32(buff + 29);
    yost::swap32(buff + 33);

    memcpy(buff + 37, bias3, 12);
    yost::swap32(buff + 37);
    yost::swap32(buff + 41);
    yost::swap32(buff + 45);

    gPrioAPI._commandThreadMutex.lock();
    PRIO_COMMAND_CHECK(_sendCommandBytes(buff, 49, logical_id));
    gPrioAPI._commandThreadMutex.unlock();

    return PRIO_NO_ERROR;
}

PRIO_ERROR PrioHub::getMagnetometerCalibParams(U8 logical_id, float* scale9, float* bias3)
{
    BASIC_CALL_CHECK();

    float data[12];
    gPrioAPI._commandThreadMutex.lock();
    PRIO_COMMAND_CHECK(_checkedCommandWriteReadFloats(0xA2, logical_id, &data[0], 12));
    gPrioAPI._commandThreadMutex.unlock();

    memcpy(scale9, data, 36);
    memcpy(bias3, data + 9, 12);

    return PRIO_NO_ERROR;
}

PRIO_ERROR PrioHub::getAccelerometerCalibParams(U8 logical_id, float* scale9, float* bias3)
{
    BASIC_CALL_CHECK();

    float data[12];
    gPrioAPI._commandThreadMutex.lock();
    PRIO_COMMAND_CHECK(_checkedCommandWriteReadFloats(0xA3, logical_id, &data[0], 12));
    gPrioAPI._commandThreadMutex.unlock();

    memcpy(scale9, data, 36);
    memcpy(bias3, data + 9, 12);
    return PRIO_NO_ERROR;
}

PRIO_ERROR PrioHub::getGyroCalibParams(U8 logical_id, float* scale9, float* bias3)
{
    BASIC_CALL_CHECK();

    float data[12];
    gPrioAPI._commandThreadMutex.lock();
    PRIO_COMMAND_CHECK(_checkedCommandWriteReadFloats(0xA4, logical_id, &data[0], 12));
    gPrioAPI._commandThreadMutex.unlock();

    memcpy(scale9, data, 36);
    memcpy(bias3, data + 9, 12);

    return PRIO_NO_ERROR;
}

PRIO_ERROR PrioHub::setGyroCalibParams(U8 logical_id, const float* scale9, const float* bias3)
{
    BASIC_CALL_CHECK();

    U8 buff[49];
    buff[0] = 0xA6;

    memcpy(buff + 1, scale9, 36);
    yost::swap32(buff + 1);
    yost::swap32(buff + 5);
    yost::swap32(buff + 9);
    yost::swap32(buff + 13);
    yost::swap32(buff + 17);
    yost::swap32(buff + 21);
    yost::swap32(buff + 25);
    yost::swap32(buff + 29);
    yost::swap32(buff + 33);

    memcpy(buff + 37, bias3, 12);
    yost::swap32(buff + 37);
    yost::swap32(buff + 41);
    yost::swap32(buff + 45);

    gPrioAPI._commandThreadMutex.lock();
    PRIO_COMMAND_CHECK(_sendCommandBytes(buff, 49, logical_id));
    gPrioAPI._commandThreadMutex.unlock();

    return PRIO_NO_ERROR;
}

PRIO_ERROR PrioHub::setBetaCalcParams(U8 logical_id, float settle_base, float settle_div, float base, float mul)
{
    BASIC_CALL_CHECK();

    U8 buff[17];
    buff[0] = 0xA7;

    memcpy(buff + 1, &settle_base, 4);
    yost::swap32(buff + 1);
    memcpy(buff + 5, &settle_div, 4);
    yost::swap32(buff + 5);
    memcpy(buff + 9, &base, 4);
    yost::swap32(buff + 9);
    memcpy(buff + 13, &mul , 4);
    yost::swap32(buff + 13);

    gPrioAPI._commandThreadMutex.lock();
    PRIO_COMMAND_CHECK(_sendCommandBytes(buff, 17, logical_id));
    gPrioAPI._commandThreadMutex.unlock();

    return PRIO_NO_ERROR;
}

PRIO_ERROR PrioHub::getBetaCalcParams(U8 logical_id, float* settle_base, float* settle_div, float* base, float* mul)
{
    BASIC_CALL_CHECK();

    float data[4];
    gPrioAPI._commandThreadMutex.lock();
    PRIO_COMMAND_CHECK(_checkedCommandWriteReadFloats(0xA8, logical_id, &data[0], 4));
    gPrioAPI._commandThreadMutex.unlock();

    *settle_base = data[0];
    *settle_div = data[1];
    *base = data[2];
    *mul = data[3];

    return PRIO_NO_ERROR;
}

PRIO_ERROR PrioHub::setAutoCalibEnabled(U8 logical_id, U8 enabled)
{
    BASIC_CALL_CHECK();

    U8 buff[2];
    buff[0] = 0xA9;

    memcpy(buff + 1, &enabled, 1);

    gPrioAPI._commandThreadMutex.lock();
    PRIO_COMMAND_CHECK(_sendCommandBytes(buff, 2, logical_id));
    gPrioAPI._commandThreadMutex.unlock();

    return PRIO_NO_ERROR;
}

PRIO_ERROR PrioHub::getAutoCalibEnabled(U8 logical_id, U8* enabled)
{
    BASIC_CALL_CHECK();

    gPrioAPI._commandThreadMutex.lock();
    PRIO_COMMAND_CHECK(_checkedCommandWriteRead(0xAA, logical_id, (U8*)enabled, 1));
    gPrioAPI._commandThreadMutex.unlock();

    return PRIO_NO_ERROR;
}

PRIO_ERROR PrioHub::commitSensorSettings(U8 logical_id)
{
    BASIC_CALL_CHECK();

    gPrioAPI._commandThreadMutex.lock();
    PRIO_COMMAND_CHECK(_sendCommand(0xE1, logical_id));
    gPrioAPI._commandThreadMutex.unlock();

    return PRIO_NO_ERROR;
}

PRIO_ERROR PrioHub::reenumerateDevice()
{
    bool restartStreaming = _isStreaming;
    if (_isStreaming)
    {
        stopStreamingWired();
    }

    _setupEnumeratorBitfield();

    if (restartStreaming)
    {
        startStreamingWired(PRIO_STREAM_UNTARED_ORIENTATION_AS_QUATERNION, 0, PRIO_STREAM_DURATION_INFINITE);
    }
    return PRIO_NO_ERROR;
}