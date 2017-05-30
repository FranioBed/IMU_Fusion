#include "threespace_device.hpp"
#include <iostream>
#include <thread>
#include <algorithm>
#include <time.h>

TSS_RESULT result;
U32 ntries;

void TssStreamPacketCircularBuffer::clear()
{
    start = 0;
    end = 0;
    len = 0;
}

void TssStreamPacketCircularBuffer::removeFirstPacket()
{
    if (len == 0)
        return;

    //remove the start element from the buffer, wrapping if needed
    start++;
    if (start == TSS_STREAM_PACKET_CIRCULAR_BUFFER_SIZE)
        start = 0;
    len--;
}

void TssStreamPacketCircularBuffer::addPacket(TssHeader* header, U8* data, U32 nbytes)
{
    //put the element at the end of the buffer, updating the appropriate indexing variables
    memcpy(&buff[end].header, header, sizeof(TssHeader));
    memcpy(buff[end].rawData, data, nbytes);
    buff[end].rawDataSize = nbytes;

    len++;
    end++;
    if (end == TSS_STREAM_PACKET_CIRCULAR_BUFFER_SIZE)
        end = 0;

    //if we have too many elements in the list, delete the first one
    if (len > TSS_STREAM_PACKET_CIRCULAR_BUFFER_SIZE)
    {
        start++;
        if (start == TSS_STREAM_PACKET_CIRCULAR_BUFFER_SIZE)
            start = 0;
        len = TSS_STREAM_PACKET_CIRCULAR_BUFFER_SIZE;
    }
}

void TssStreamPacketCircularBuffer::getPacket(TssStreamPacket* packet, U32 index)
{
    memcpy(&packet->header, &buff[index].header, sizeof(TssHeader));
    memcpy(packet->rawData, buff[index].rawData, buff[index].rawDataSize);
    packet->rawDataSize = buff[index].rawDataSize;
}

void _tssParseResponseHeader(U8* data, U8 flags, TssHeader* header)
{
    memset(header, 0xff, sizeof(TssHeader));

    if (flags == 0)
        return;

    U8 index = 0;

    if (flags & TSS_RESPONSE_HEADER_SUCCESS)
    {
        memcpy(&header->Success, data + index, 1);
        header->Success = !header->Success; //the header data has true as failure when it comes in, flip it here
        index += 1;
    }

    if (flags & TSS_RESPONSE_HEADER_TIMESTAMP)
    {
        memcpy(&header->SensorTimestamp, data + index, 4);
        yost::swap32((U8*)&header->SensorTimestamp);
        index += 4;
    }

    if (flags & TSS_RESPONSE_HEADER_COMMAND_ECHO)
    {
        memcpy(&header->CommandEcho, data + index, 1);
        index += 1;
    }

    if (flags & TSS_RESPONSE_HEADER_CHECKSUM)
    {
        memcpy(&header->Checksum, data + index, 1);
        index += 1;
    }

    if (flags & TSS_RESPONSE_HEADER_LOGICAL_ID)
    {
        memcpy(&header->LogicalId, data + index, 1);
        index += 1;
    }

    if (flags & TSS_RESPONSE_HEADER_SERIAL_NUMBER)
    {
        memcpy(&header->SerialNumber, data + index, 4);
        yost::swap32((U8*)&header->SerialNumber);
        index += 4;
    }

    if (flags & TSS_RESPONSE_HEADER_DATA_LENGTH)
    {
        memcpy(&header->DataLength, data + index, 1);
        index += 1;
    }

    header->SystemTimestamp = yost::get_time();
}

TssDevice::TssDevice()
{
    _isWireless = false;
    _logicalId = 0xff;
    _isportOwner = false;
    _responseHeaderFlags = 0;
    _responseHeaderSize = 0;
    _commandRetries = 1;
    _isStreaming = false;
}

TssDevice::~TssDevice()
{
    if (_port && _isportOwner)
    {
        _port->close();
        _port.reset();
    }
}

bool TssDevice::isConnected()
{
    return _port && _port->isOpen();
}

TssHeader* TssDevice::getLastHeader()
{
    return &_lastHeader;
}

U32 TssDevice::getSerialNumber()
{
    return _serialNumber;
}

TSS_TYPE TssDevice::getSensorType()
{
    return _sensorType;
}

TSS_RESULT TssDevice::_setResponseHeader(U32 flags)
{
    //send the set response header command, pick wired or wireless based on wireless flag
    if (!_isWireless)
    {
        U8 buff[256];
        buff[0] = 221;
        memcpy(buff + 1, &flags, 4);
        yost::swap32(buff + 1);
        _sendCommandBytes(buff, 5);
    }


    _responseHeaderFlags = flags;
    _responseHeaderSize = 0;

    if (_responseHeaderFlags & TSS_RESPONSE_HEADER_SUCCESS)
    {
        _responseHeaderSize += 1;
    }

    if (_responseHeaderFlags & TSS_RESPONSE_HEADER_TIMESTAMP)
    {
        _responseHeaderSize += 4;
    }

    if (_responseHeaderFlags & TSS_RESPONSE_HEADER_COMMAND_ECHO)
    {
        _responseHeaderSize += 1;
    }

    if (_responseHeaderFlags & TSS_RESPONSE_HEADER_CHECKSUM)
    {
        _responseHeaderSize += 1;
    }

    if (_responseHeaderFlags & TSS_RESPONSE_HEADER_LOGICAL_ID)
    {
        _responseHeaderSize += 1;
    }

    if (_responseHeaderFlags & TSS_RESPONSE_HEADER_SERIAL_NUMBER)
    {
        _responseHeaderSize += 4;
    }

    if (_responseHeaderFlags & TSS_RESPONSE_HEADER_DATA_LENGTH)
    {
        _responseHeaderSize += 1;
    }

    return TSS_SUCCESS;
}

TSS_RESULT TssDevice::_openPort(std::string name)
{
    _portName = name;
    Timeout t;

    t.inter_byte_timeout = 100;
    t.read_timeout_multiplier = 100;
    t.read_timeout_constant = 100;
    t.write_timeout_multiplier = 3;
    t.write_timeout_constant = 2;

    try
    {
        _port = shared_ptr<Serial>(new Serial(name, 115200, t));
    }
    catch (IOException&)
    {
        _port.reset();
        return TSS_ERROR_CANT_OPEN_PORT;
    }

    if (!_port->isOpen())
    {
        _port.reset();
        return TSS_ERROR_CANT_OPEN_PORT;
    }

    _isportOwner = true;

    return TSS_SUCCESS;
}

TSS_RESULT TssDevice::_writeBytes(U8* data, U8 n_bytes)
{
    unsigned long written;
    written = _port->write(data, n_bytes);

    if (written != n_bytes)
        return TSS_ERROR_NOT_ENOUGH_DATA;

    return TSS_SUCCESS;
}

TSS_RESULT TssDevice::_readBytes(U8* data, U8 n_bytes)
{
    if (n_bytes == 0)
        return TSS_SUCCESS;

    unsigned long read;
    read = _port->read(data, n_bytes);

    if (read != n_bytes)
        return TSS_ERROR_NOT_ENOUGH_DATA;

    return TSS_SUCCESS;
}

TSS_RESULT TssDevice::_readResponseHeader()
{
    U8 buff[256];
    TSS_ERROR_CHECK(_readBytes(buff, _responseHeaderSize));
    _tssParseResponseHeader(buff, _responseHeaderFlags, &_lastHeader);
    return TSS_SUCCESS;
}

TSS_RESULT TssDevice::_sendCommandBytes(const U8* data, U8 n_bytes, bool ignore_response)
{
    U8 buff[256];

    U8 index;
    U8 checksum;
    TSS_RESULT result;

    //dump any pre-existing data
    _port->flushInput();

    //select the corrent command type byte based on wirelessness and header flags
    if (_isWireless)
    {
        checksum = _logicalId;

        if (_responseHeaderFlags == 0)
        {
            buff[0] = 0xf8;
        }
        else
        {
            buff[0] = 0xfa;
        }

        buff[1] = _logicalId;
        index = 2;
    }
    else
    {
        checksum = 0;

        if (_responseHeaderFlags == 0)
        {
            buff[0] = 0xf7;
        }
        else
        {
            buff[0] = 0xf9;
        }

        index = 1;
    }

    //move command data over into the final call buffer and calculate the checksum
    U8 i;
    for (i = 0; i < n_bytes; i++)
    {
        buff[index] = data[i];
        checksum += data[i];
        index++;
    }

    buff[index] = checksum;
    index++;

    TSS_ERROR_CHECK(_writeBytes(buff, index));

    if (ignore_response)
    {
        //do nothing here, caller is expected to purge data once it is done
    }
    else
    {
        //read out header and determine success if possible, but do not read the command data yet
        TSS_ERROR_CHECK(_readResponseHeader());

        if (_responseHeaderFlags & TSS_RESPONSE_HEADER_SUCCESS)
        {
            if (!_lastHeader.Success)
            {
                return TSS_ERROR_COMMAND_FAILURE;
            }
        }
    }


    return TSS_SUCCESS;
}

TSS_RESULT TssDevice::_sendCommand(U8 command, bool ignore_response)
{
    U8 buff[1];
    buff[0] = command;

    TSS_ERROR_CHECK(_sendCommandBytes(buff, 1, ignore_response));

    return TSS_SUCCESS;
}

TSS_RESULT TssDevice::_readFloats(float* data, U8 n_floats)
{
    TSS_ERROR_CHECK(_readBytes((U8*)data, n_floats * 4));

    _parseFloats(data, n_floats);

    return TSS_SUCCESS;
}

TSS_RESULT TssDevice::_parseFloats(float* data, U8 n_floats)
{
    U8 i;
    for (i = 0; i < n_floats; i++)
    {
        yost::swap32((U8*)&(data[i]));
    }

    return TSS_SUCCESS;
}

TSS_RESULT TssDevice::_checkedCommandWriteRead(U8 command, U8* data, U8 n_bytes)
{
    BASIC_CALL_CHECK_TSS();

    for (U8 i = 0; i < _commandRetries; i++)
    {
        result = _sendCommand(command);

        if (result != TSS_SUCCESS)
            continue;

        result = _readBytes(data, n_bytes);

        if (result != TSS_SUCCESS)
            continue;

        break;
    }

    return result;
}

TSS_RESULT TssDevice::_checkedCommandWriteReadFloats(U8 command, float* data, U8 n_floats)
{
    BASIC_CALL_CHECK_TSS();

    for (U8 i = 0; i < _commandRetries; i++)
    {
        result = _sendCommand(command);

        if (result != TSS_SUCCESS)
            continue;

        result = _readFloats(data, n_floats);

        if (result != TSS_SUCCESS)
            continue;

        break;
    }

    return result;
}

void TssDevice::_purgeInput()
{
    yost::sleep_ms(TSS_MAX_COMMAND_RETURN_TIME_MS);

    _port->flushInput();
}

TSS_RESULT TssDevice::_readSerialNumber()
{
    TSS_ERROR_CHECK(_sendCommand(237));

    TSS_ERROR_CHECK(_readBytes((U8*)&_serialNumber, 4));

    yost::swap32((U8*)&_serialNumber);

    return TSS_SUCCESS;
}

//bool startsWith(string starts, string with)
//{
//	return starts.substr(0, with.size()) == with;
//}

TSS_RESULT TssDevice::_readVersionString()
{
    TSS_ERROR_CHECK(_sendCommand(230));

    U8 buff[33];
    TSS_ERROR_CHECK(_readBytes(buff, 32));
    buff[32] = '\0';

    _versionString = (char*)buff;

    string version_sub = _versionString.substr(4);
    if (yost::startsWith(version_sub, "USB"))
    {
        _sensorType = TSS_USB;
    }
    else if (yost::startsWith(version_sub, "EM"))
    {
        _sensorType = TSS_EM;
    }
    else if (yost::startsWith(version_sub, "WL"))
    {
        _sensorType = TSS_WL;
    }
    else if (yost::startsWith(version_sub, "DNG"))
    {
        _sensorType = TSS_DNG;
    }
    else if (yost::startsWith(version_sub, "DL"))
    {
        _sensorType = TSS_DL;
    }
    else if (yost::startsWith(version_sub, "BT"))
    {
        _sensorType = TSS_BT;
    }


    return TSS_SUCCESS;
}