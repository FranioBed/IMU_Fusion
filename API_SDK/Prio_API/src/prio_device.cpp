#include "prio_device.hpp"
#include "prio_api.hpp"
#include <iostream>
#include <thread>
#include <algorithm>
#include <time.h>
#include <fstream>

PRIO_ERROR prio_error;
U32 prio_ntries;

void PrioDevice::_comPortMainThread(const char* name)
{
    _portThreadMutex.lock();

    _threadRunning = true;
    _breakPortThread = false;
    shared_ptr<Serial> port;
    
    Timeout t;
    t.inter_byte_timeout = 100;
    t.read_timeout_multiplier = 100;
    t.read_timeout_constant = 100;
    t.write_timeout_multiplier = 100;
    t.write_timeout_constant = 100;

    try
    {
        port = shared_ptr<Serial>(new Serial(name, 115200, t));
        port->flushInput();
    }
    catch (IOException&)
    {
        port.reset();
        _breakPortThread = false;
        _threadRunning = false;
        _portThreadMutex.unlock();
        return;
    }
    catch (...)
    {
        port.reset();
        _breakPortThread = false;
        _threadRunning = false;
        _portThreadMutex.unlock();
        return;
    }

    if (!port->isOpen())
    {
        port.reset();
        _breakPortThread = false;
        _threadRunning = false;
        _portThreadMutex.unlock();
        return;
    }

    _port = port;

    _portThreadMutex.unlock();
    while (1)
    {
        _portThreadMutex.lock();
        if (_breakPortThread == true)
        {
            _threadRunning = false;
            port->close();
            port.reset();
            _port.reset();
            _portThreadMutex.unlock();
            break;
        }
        _portThreadMutex.unlock();
        yost::sleep_ms(1);
    }
}

void _prioParseStreamData(std::vector<U8> data, U8 nodes, PrioHeader* header )
{
    U8 index = 0;

    if (nodes & PRIO_RESPONSE_HEADER_TIMESTAMP)
    {
        memcpy(&header->SensorTimestamp, &data[index], 4);
        yost::swap32((U8*)&header->SensorTimestamp);
        //header->SensorTimestamp = header->SensorTimestamp * 1e-3;
        index += 4;
    }

    if (data.size() != index+20)
    {
        return;
    }


    // Parse stream header
    header->BatteryLevel = data[index];
    index++;
    header->BatteryStatus = data[index];
    index++;
    header->HubButton = data[index];
    index++;
    memcpy(&header->Joystick1.x_Axis, &data[index], 1);
    index++;
    memcpy(&header->Joystick1.y_Axis, &data[index], 1);
    index++;
    header->Joystick1.Trigger = data[index];
    index++;
    header->Joystick1.ButtonState = data[index];
    index++;
    memcpy(&header->Joystick2.x_Axis, &data[index], 1);
    index++;
    memcpy(&header->Joystick2.y_Axis, &data[index], 1);
    index++;
    header->Joystick2.Trigger = data[index];
    index++;
    header->Joystick2.ButtonState = data[index];
    index++;
    header->SystemTimestamp = yost::get_time();

    memcpy(&header->SensorBitfield, &data[index], 8);
    yost::swap64((U8*)&header->SensorBitfield);

    U64 tmp = header->SensorBitfield & 255;
    tmp = tmp | (header->SensorBitfield >> 1 & 32640);
    tmp = tmp | (header->SensorBitfield >> 2 & 4177920);
    tmp = tmp | (header->SensorBitfield >> 3 & 534773760);
    tmp = tmp | (header->SensorBitfield >> 4 & 68451041280);
    tmp = tmp | (header->SensorBitfield >> 5 & 8761733283840);

    //Get the joystick indexs
    tmp = tmp | (header->SensorBitfield & 71776119061217280);
    tmp = tmp | (header->SensorBitfield & 18374686479671623680);

    header->SensorBitfield = tmp;
    index += 8;
    header->Checksum = data[index];
}

void _prioParseResponseHeader(U8* data, U8 flags, PrioHeader* header)
{
    memset(header, 0xff, sizeof(PrioHeader));
    header->Success = 0;

    U8 index = 0;
    if (flags & PRIO_RESPONSE_HEADER_SUCCESS)
    {
        memcpy(&header->Success, data + index, 1);
        index += 1;
    }

    if (flags & PRIO_RESPONSE_HEADER_TIMESTAMP)
    {
        memcpy(&header->SensorTimestamp, data + index, 4);
        yost::swap32((U8*)&header->SensorTimestamp);
        index += 4;
    }

    if (flags & PRIO_RESPONSE_HEADER_COMMAND_ECHO)
    {
        memcpy(&header->CommandEcho, data + index, 1);
        index += 1;
    }

    if (flags & PRIO_RESPONSE_HEADER_LOGICAL_ID)
    {
        memcpy(&header->LogicalId, data + index, 1);
        index += 1;
    }
    header->SystemTimestamp = yost::get_time();
}

PrioDevice::PrioDevice()
{
    _isWireless = false;
    _logicalId = 0xff;
    _isportOwner = false; 
    _responseHeaderFlags = 0;
    _responseHeaderSize = 0;
    _commandRetries = 1;
    _isStreaming = false;
}

PrioDevice::~PrioDevice()
{
    _breakPortThread = true;
    if (_portThread.joinable())
    {
        _portThread.join();
        _portThreadMutex.lock();
        _portThreadMutex.unlock();
    }
    _port = NULL;
}

bool PrioDevice::isConnected()
{
    return _port && _port->isOpen();
}

PrioHeader* PrioDevice::getLastHeader()
{
    return &_lastHeader;
}

U32 PrioDevice::getSerialNumber()
{
    return _serialNumber;
}

PRIO_TYPE PrioDevice::getSensorType()
{
    return _sensorType;
}

PRIO_ERROR PrioDevice::_setResponseHeader(U32 flags)
{
    //send the set response header command, pick wired or wireless based on wireless flag, does this even work anymore??????
    if (!_isWireless)
    {
        U8 buff[256];
        buff[0] = 221;
        memcpy(buff + 1, &flags, 4);
        yost::swap32(buff + 1);
        gPrioAPI._commandThreadMutex.lock();
        _sendCommandBytes(buff, 5);
        gPrioAPI._commandThreadMutex.unlock();
    }

    _responseHeaderFlags = flags;
    _responseHeaderSize = 0;

    if (_responseHeaderFlags & PRIO_RESPONSE_HEADER_SUCCESS)
    {
        _responseHeaderSize += 1;
    }

    if (_responseHeaderFlags & PRIO_RESPONSE_HEADER_TIMESTAMP)
    {
        _responseHeaderSize += 4;
    }

    if (_responseHeaderFlags & PRIO_RESPONSE_HEADER_COMMAND_ECHO)
    {
        _responseHeaderSize += 1;
    }

    if (_responseHeaderFlags & PRIO_RESPONSE_HEADER_LOGICAL_ID)
    {
        _responseHeaderSize += 1;
    }

    if (_responseHeaderFlags & PRIO_RESPONSE_HEADER_DATA_LENGTH)
    {
        _responseHeaderSize += 1;
    }
    if (_isStreaming)
    {
        _responseHeaderSize += 20;
    }
    return PRIO_NO_ERROR;
}

PRIO_ERROR PrioDevice::_setWirelessResponseHeader(U32 flags)
{
    //send the set response header command, pick wired or wireless based on wireless flag, does this even work anymore??????
    if (!_isWireless)
    {
        U8 buff[256];
        buff[0] = 221;
        memcpy(buff + 1, &flags, 4);
        yost::swap32(buff + 1);
        gPrioAPI._commandThreadMutex.lock();
        _sendCommandBytes(buff, 5);
        gPrioAPI._commandThreadMutex.unlock();
    }

    _responseHeaderWirelessFlags = flags;
    _responseHeaderWirelessSize = 0;

    if (_responseHeaderWirelessFlags & PRIO_RESPONSE_HEADER_SUCCESS)
    {
        _responseHeaderWirelessSize += 1;
    }

    if (_responseHeaderWirelessFlags & PRIO_RESPONSE_HEADER_TIMESTAMP)
    {
        _responseHeaderWirelessSize += 4;
    }

    if (_responseHeaderWirelessFlags & PRIO_RESPONSE_HEADER_COMMAND_ECHO)
    {
        _responseHeaderWirelessSize += 1;
    }

    if (_responseHeaderWirelessFlags & PRIO_RESPONSE_HEADER_LOGICAL_ID)
    {
        _responseHeaderWirelessSize += 1;
    }

    if (_responseHeaderWirelessFlags & PRIO_RESPONSE_HEADER_DATA_LENGTH)
    {
        _responseHeaderWirelessSize += 1;
    }
    if (_isStreaming)
    {
        _responseHeaderWirelessSize += 20;
    }
    return PRIO_NO_ERROR;
}

PRIO_ERROR PrioDevice::_openPort(const char* name)
{
    _threadRunning = false;
    try
    {
        _portThread = std::thread(&PrioDevice::_comPortMainThread, this, name);
    }
    catch (std::exception& ex)
    {
        printf("Error in thread create: %s",ex.what());
    }

    yost::sleep_ms(1); // Sleep for a non-zero amount 

    _portThreadMutex.lock();
    if (!_threadRunning)
    {
        _portThreadMutex.unlock();
        return PRIO_ERROR_COMMAND_FAIL;
    }
    _isportOwner = true;
    memcpy((char*)_portName, name, sizeof(char) * 64);
    _portThreadMutex.unlock();
    return PRIO_NO_ERROR;
}

PRIO_ERROR PrioDevice::_writeBytes(U8* data, U16 n_bytes)
{
    unsigned long written = 10000;

    _portThreadMutex.lock();
    try
    {
        written = _port->write(data, n_bytes);
    }
    catch(...)
    {
        return PRIO_ERROR_WRITE;
    }
    _portThreadMutex.unlock();

    if (written != n_bytes)
    {
        return PRIO_ERROR_WRITE;
    }

    return PRIO_NO_ERROR;
}

PRIO_ERROR PrioDevice::_readBytes(U8* data, U16 n_bytes)
{
    if (n_bytes == 0)
        return PRIO_NO_ERROR;

    unsigned long read;
    _portThreadMutex.lock();
    try
    {
        read = _port->read(data, n_bytes);
    }
    catch (...)
    {
        return PRIO_ERROR_WRITE;
    }
    _portThreadMutex.unlock();

    if (read != n_bytes)
    {
        return PRIO_ERROR_READ;
    }

    return PRIO_NO_ERROR;
}

PRIO_ERROR PrioDevice::_readResponseHeader()
{
    U8 buff[256];
    PRIO_ERROR_CHECK(_readBytes(buff, _responseHeaderSize));
    _prioParseResponseHeader(buff, _responseHeaderFlags, &_lastHeader);
    return PRIO_NO_ERROR;
}

PRIO_ERROR PrioDevice::_readWirelessResponseHeader()
{
    U8 buff[256];
    PRIO_ERROR_CHECK(_readBytes(buff, _responseHeaderWirelessSize));
    _prioParseResponseHeader(buff, _responseHeaderWirelessFlags, &_lastHeader);
    return PRIO_NO_ERROR;
}

PRIO_ERROR PrioDevice::_sendCommandBytes(const U8* data, U16 n_bytes, bool ignore_response)
{
    return _sendCommandBytes(data, n_bytes, _logicalId, ignore_response);
}

PRIO_ERROR PrioDevice::_sendCommandBytes(const U8* data, U16 n_bytes, U8 logical_id, bool ignore_response)
{
    U8 buff[256];

    U8 index=0;
	U8 checksum = 0;

    //dump any pre-existing data
    _port->flushInput();

    //select the corrent command type byte based on wirelessness and header flags
    if (_isWireless || logical_id != _logicalId)
    {
        checksum = logical_id;

        buff[0] = 0xf8;

        buff[1] = logical_id;
        index = 2;
    }
    else
    {
        checksum = 0;

        buff[0] = 0xf7;

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

    PRIO_ERROR_CHECK(_writeBytes(buff, index));

    if (ignore_response)
    {
        //do nothing here, caller is expected to purge data once it is done
    }
    else
    {
        if (_isWireless || logical_id != _logicalId)
        {
            //read out header and determine success if possible, but do not read the command data yet
            PRIO_ERROR_CHECK(_readWirelessResponseHeader());
        }
        else
        {
            //read out header and determine success if possible, but do not read the command data yet
            PRIO_ERROR_CHECK(_readResponseHeader());
        }

        if (_lastHeader.Success == 4)
        {
            return PRIO_ERROR_COMMAND_FAIL_COMMUNICATION;
        }
        else if (_lastHeader.Success == 6)
        {
            return PRIO_ERROR_COMMAND_FAIL_ENUMERATION_BIT_UNREAD;
        }
        else if (_lastHeader.Success != 0)
        {
            return PRIO_ERROR_COMMAND_FAIL;
        }
    }
    return PRIO_NO_ERROR;
}

PRIO_ERROR PrioDevice::_sendCommand(U8 command, bool ignore_response)
{
    return _sendCommand(command, _logicalId, ignore_response);
}

PRIO_ERROR PrioDevice::_sendCommand(U8 command, U8 logical_id, bool ignore_response)
{
    U8 buff[1];
    buff[0] = command;

    PRIO_ERROR_CHECK(_sendCommandBytes(buff, 1, logical_id, ignore_response));

    return PRIO_NO_ERROR;
}

PRIO_ERROR PrioDevice::_readFloats(float* data, U16 n_floats)
{
    PRIO_ERROR_CHECK(_readBytes((U8*)data, n_floats * 4));

    _parseFloats(data, n_floats);

    return PRIO_NO_ERROR;
}

PRIO_ERROR PrioDevice::_parseFloats(float* data, U16 n_floats)
{
    U16 i;
    for (i = 0; i < n_floats; i++)
    {
        yost::swap32((U8*)&(data[i]));
    }

    return PRIO_NO_ERROR;
}

PRIO_ERROR PrioDevice::_checkedCommandWriteRead(U8 command, U8* data, U16 n_bytes)
{
    return _checkedCommandWriteRead(command, _logicalId, data, n_bytes);
}

PRIO_ERROR PrioDevice::_checkedCommandWriteRead(U8 command, U8 logical_id, U8* data, U16 n_bytes)
{
    BASIC_CALL_CHECK();

    for (U8 i = 0; i < _commandRetries; i++)
    {
        prio_error = _sendCommand(command, logical_id);

        if (prio_error != PRIO_NO_ERROR)
            continue;

        prio_error = _readBytes(data, n_bytes);

        if (prio_error != PRIO_NO_ERROR)
            continue;

        break;
    }

    return prio_error;
}

PRIO_ERROR PrioDevice::_checkedCommandWriteReadFloats(U8 command, float* data, U16 n_floats)
{
    return _checkedCommandWriteReadFloats(command, _logicalId, data, n_floats);
}

PRIO_ERROR PrioDevice::_checkedCommandWriteReadFloats(U8 command, U8 logical_id, float* data, U16 n_floats)
{
    BASIC_CALL_CHECK();

    for (U8 i = 0; i < _commandRetries; i++)
    {
        U8 buff[1];
        buff[0] = command;
        prio_error = _sendCommand(command, logical_id);

        if (prio_error != PRIO_NO_ERROR)
            continue;

        prio_error = _readFloats(data, n_floats);

        if (prio_error != PRIO_NO_ERROR)
            continue;

        break;
    }

    return prio_error;
}

void PrioDevice::_purgeInput()
{
    yost::sleep_ms(PRIO_MAX_COMMAND_RETURN_TIME_MS);

    _port->flushInput();
} 

PRIO_ERROR PrioDevice::_readSerialNumber()
{
    gPrioAPI._commandThreadMutex.lock();
    PRIO_COMMAND_CHECK(_sendCommand(237));

    PRIO_COMMAND_CHECK(_readBytes((U8*)&_serialNumber, 4));
    gPrioAPI._commandThreadMutex.unlock();

    yost::swap32((U8*)&_serialNumber);

    return PRIO_NO_ERROR;
}

PRIO_ERROR PrioDevice::_readVersionString()
{
    gPrioAPI._commandThreadMutex.lock();
    PRIO_COMMAND_CHECK(_sendCommand(230));
    U8 buff[33] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    PRIO_COMMAND_CHECK(_readBytes(buff, 32));
    gPrioAPI._commandThreadMutex.unlock();
    buff[32] = '\0';

    _versionString = (char*)buff;

    string version_sub = _versionString.substr(4);
    if (startsWith(version_sub, "BS"))
    {
        _sensorType = PRIO_BS;
    }
    else if (startsWith(version_sub, "HUB"))
    {
        _sensorType = PRIO_HUB;
    }


    return PRIO_NO_ERROR;
}

PRIO_ERROR PrioDevice::getFirmwareVersion(const char* firmware_version)
{
    gPrioAPI._commandThreadMutex.lock();
    PRIO_COMMAND_CHECK(_sendCommand(223));

    U8 buff[13];
    PRIO_COMMAND_CHECK(_readBytes(buff, 12));

    gPrioAPI._commandThreadMutex.unlock();
    buff[12] = '\0';
    
    memcpy((char*)firmware_version, buff, 13);

    return PRIO_NO_ERROR;
}

PRIO_ERROR PrioDevice::restoreFactorySettings()
{
    gPrioAPI._commandThreadMutex.lock();
    PRIO_COMMAND_CHECK(_sendCommand(224));
    gPrioAPI._commandThreadMutex.unlock();

    return PRIO_NO_ERROR;
}

PRIO_ERROR PrioDevice::commitSettings()
{
    gPrioAPI._commandThreadMutex.lock();
    PRIO_COMMAND_CHECK(_sendCommand(225));
    gPrioAPI._commandThreadMutex.unlock();

    return PRIO_NO_ERROR;
}

PRIO_ERROR PrioDevice::resetDevice()
{
    gPrioAPI._commandThreadMutex.lock();
    PRIO_COMMAND_CHECK(_sendCommand(226));
    gPrioAPI._commandThreadMutex.unlock();

    return PRIO_NO_ERROR;
}

PRIO_ERROR PrioDevice::enterBootloaderMode()
{
    gPrioAPI._commandThreadMutex.lock();
    PRIO_COMMAND_CHECK(_sendCommand(229));
    gPrioAPI._commandThreadMutex.unlock();

    return PRIO_NO_ERROR;
}

PRIO_ERROR PrioDevice::getHardwareVersion(const char* hardware_version)
{
    gPrioAPI._commandThreadMutex.lock();
    PRIO_COMMAND_CHECK(_sendCommand(230));

    U8 buff[33];
    PRIO_COMMAND_CHECK(_readBytes(buff, 32));
    gPrioAPI._commandThreadMutex.unlock();
    buff[32] = '\0';

    memcpy((char*)hardware_version, buff, 33);

    return PRIO_NO_ERROR;
}

// 235(0xEB)
PRIO_ERROR PrioDevice::getUpdateRate(U32* rate)
{
    gPrioAPI._commandThreadMutex.lock();
    PRIO_COMMAND_CHECK(_sendCommand(237));

    PRIO_COMMAND_CHECK(_readBytes((U8*)rate, 4));
    gPrioAPI._commandThreadMutex.unlock();

    yost::swap32((U8*)rate);

    return PRIO_NO_ERROR;
}

// 238(0xEE)
PRIO_ERROR PrioDevice::setLEDColor(float* new_color)
{
    gPrioAPI._commandThreadMutex.lock();
    PRIO_COMMAND_CHECK(_checkedCommandWriteReadFloats(0x01, new_color, 3));
    gPrioAPI._commandThreadMutex.unlock();

    return PRIO_NO_ERROR;
}