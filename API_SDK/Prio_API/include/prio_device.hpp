#pragma once

#include <memory>
#include <string>
#include <vector>
#include <deque>

#include <mutex>
#include <thread>

#include "yost_math.hpp"
#include "serial.h"
#include "prio_api_export.h"
#include "yost_core_api.hpp"

using namespace std;
using namespace serial;

extern PRIO_ERROR prio_error; // Make prio_prio_error Nick 7/21/2016
extern U32 prio_ntries;

#define PRIO_ERROR_CHECK(x) prio_error=x;if(prio_error != PRIO_NO_ERROR)return prio_error;
#define PRIO_COMMAND_CHECK(x) prio_error=x;if(prio_error != PRIO_NO_ERROR){gPrioAPI._commandThreadMutex.unlock();if(prio_error == PRIO_ERROR_COMMAND_FAIL_ENUMERATION_BIT_UNREAD){reenumerateDevice();gPrioAPI._commandThreadMutex.lock();prio_error=x;gPrioAPI._commandThreadMutex.unlock();if(prio_error == PRIO_NO_ERROR)return prio_error;}return prio_error;}
#define PRIO_RETRY(x, tries) prio_ntries=tries;while(prio_ntries>0){prio_error=x;if(prio_error == PRIO_NO_ERROR)break;prio_ntries--;}

//#define _swap(a,b,c) c=a;a=b;b=c

#define PRIO_START_STREAM_RETRIES 50
#define PRIO_STOP_STREAM_RETRIES 30

#define PRIO_MAX_COMMAND_RETURN_TIME_MS 500

#define PRIO_MAX_STREAM_PACKET_RAW_DATA_SIZE 512

//note about this struct: there was a design tradeoff here between memory and speed.
//this is the speed form of this struct, as all data that was in a streaming packet
//is present and ready for use, though memory is sacrificed for this.  The memory
//form would involve only having the rawData array here, and a set of functions to
//pull out desired pieces of data.  I can't say at this point which would be better
//in the long run, so we may want to try it the other way at some point.
struct PrioStreamPacket
{
    //Header Info
    PrioHeader header;

    //Stream Data
    Orient taredOrientQuat;
    Orient untaredOrientQuat;
    Vector3 corectedGyro;
    Vector3 corectedAccelerometer;
    Vector3 corectedMagnetometer;
    Vector3 rawGyro;
    Vector3 rawAccelerometer;
    Vector3 rawMagnetometer;

    U8 rawData[PRIO_MAX_STREAM_PACKET_RAW_DATA_SIZE];
    U32 rawDataSize;
};

#define PRIO_STREAM_PACKET_CIRCULAR_BUFFER_SIZE 1024

#define PRIO_DEVICE_TYPE_HUB 0
#define PRIO_DEVICE_TYPE_BS 1

void _prioParseStreamData(std::vector<U8> data, U8 nodes, PrioHeader* header);
void _prioParseResponseHeader(U8* data, U8 flags, PrioHeader* header);

class PrioDevice
{
public:
    PrioDevice();
    virtual ~PrioDevice() = 0;

    bool isConnected();
    PrioHeader* getLastHeader();
    U32 getSerialNumber();
    PRIO_TYPE getSensorType();

    //Port functions
    //The reason for this is because there is a windows bug where Windows won't realase the com port until the process shuts down
    void _comPortMainThread(const char* name);

    PRIO_ERROR _openPort(const char* name);
    virtual PRIO_ERROR closePort() = 0;
    PRIO_ERROR _writeBytes(U8* data, U16 n_bytes);
    PRIO_ERROR _readBytes(U8* data, U16 n_bytes);
    void _purgeInput();

    //Response header functions
    PRIO_ERROR _readResponseHeader();
    PRIO_ERROR _setResponseHeader(U32 flags);

    PRIO_ERROR _readWirelessResponseHeader();
    PRIO_ERROR _setWirelessResponseHeader(U32 flags);

    //Command sending support functions
    PRIO_ERROR _sendCommandBytes(const U8* data, U16 n_bytes, bool ignore_response = false);
    PRIO_ERROR _sendCommandBytes(const U8* data, U16 n_bytes, U8 logical_id, bool ignore_response = false);
    PRIO_ERROR _sendCommand(U8 command, bool ignore_response = false);
    PRIO_ERROR _sendCommand(U8 command, U8 logical_id, bool ignore_response = false);
    PRIO_ERROR _readFloats(float* data, U16 n_floats);
    PRIO_ERROR _parseFloats(float* data, U16 n_floats);
    PRIO_ERROR _checkedCommandWriteRead(U8 command, U8* data, U16 n_bytes);
    PRIO_ERROR _checkedCommandWriteRead(U8 command, U8 logical_id, U8* data, U16 n_bytes);
    PRIO_ERROR _checkedCommandWriteReadFloats(U8 command, float* data, U16 n_floats);
    PRIO_ERROR _checkedCommandWriteReadFloats(U8 command, U8 logical_id, float* data, U16 n_floats);

    //Lower level command functions
    PRIO_ERROR _readSerialNumber();
    PRIO_ERROR _readVersionString();

    //Device Functions
    virtual PRIO_ERROR reenumerateDevice() = 0;

    //Command Response Functions
    // 223(0xDF)
    PRIO_ERROR getFirmwareVersion(const char* firmware_version);
    // 224(0xE0)
    PRIO_ERROR restoreFactorySettings();
    // 225(0xE1)
    PRIO_ERROR commitSettings();
    // 226(0xE2)
    PRIO_ERROR resetDevice();
    // 229(0xE5)
    PRIO_ERROR enterBootloaderMode();
    // 230(0xE6)
    PRIO_ERROR getHardwareVersion(const char* hardware_version);
    // 235(0xEB)
    PRIO_ERROR getUpdateRate(U32* rate);
    // 238(0xEE)
    PRIO_ERROR setLEDColor(float* new_color);

    //Port thread
    //std::mutex _commandThreadMutex;
    std::mutex _portThreadMutex;
    std::thread _portThread;
    bool _breakPortThread;
    bool _threadRunning;

    std::string _hardware_version;
    std::string _firmware_version;

    U8 _type;
    bool _isportOwner;
    const char* _portName;
    bool _isWireless;
    U32 _responseHeaderFlags;
    U8 _responseHeaderSize;
    U32 _responseHeaderWirelessFlags;
    U8 _responseHeaderWirelessSize;
    shared_ptr<Serial> _port;
    U8 _logicalId;
    PrioHeader _lastHeader;
    U32 _serialNumber;
    std::string _versionString;
    PRIO_TYPE _sensorType;
    U8 _commandRetries;
    bool _isStreaming;
    U16 _streamDataSize;
    bool _isRecording;
    U32 _maxRecordingCount;
    bool _wrappingMode;
};

#define BASIC_CALL_CHECK() if (!isConnected()){return PRIO_ERROR_FIRMWARE_INCOMPATIBLE;}if (_isStreaming){return PRIO_ERROR_COMMAND_FAIL;}