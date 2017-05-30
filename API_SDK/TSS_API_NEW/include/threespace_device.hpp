#pragma once

#include <memory>
#include <string>
#include <vector>
#include "yost_math.hpp"
#include "serial/serial.h"
#include "threespace_api_export.h"
#include "yost_core_api.hpp"

using namespace std;
using namespace serial;

extern TSS_RESULT result;
extern U32 ntries;
#define TSS_ERROR_CHECK(x) result=x;if(result != TSS_SUCCESS)return result
#define TSS_RETRY(x, tries) ntries=tries;while(ntries>0){result=x;if(result==TSS_SUCCESS)break;ntries--;}

//#define _swap(a,b,c) c=a;a=b;b=c

#define TSS_START_STREAM_RETRIES 200
#define TSS_STOP_STREAM_RETRIES 100

#define TSS_MAX_COMMAND_RETURN_TIME_MS 500

struct TssHeader
{
    bool Success;
    U32 SensorTimestamp;
    float SystemTimestamp;
    U8 CommandEcho;
    U8 Checksum;
    U8 LogicalId;
    U32 SerialNumber;
    U8 DataLength;
};

#define TSS_MAX_STREAM_PACKET_RAW_DATA_SIZE 128

//note about this struct: there was a design tradeoff here between memory and speed.
//this is the speed form of this struct, as all data that was in a streaming packet
//is present and ready for use, though memory is sacrificed for this.  The memory
//form would involve only having the rawData array here, and a set of functions to
//pull out desired pieces of data.  I can't say at this point which would be better
//in the long run, so we may want to try it the other way at some point.
struct TssStreamPacket
{
    TssHeader header;

    Orient taredOrientQuat;
    float taredOrientEuler[3];
    Matrix3x3 taredOrientMatrix;
    Vector3 taredOrientAxis;
    float taredOrientAngle;
    Vector3 taredOrientForward;
    Vector3 taredOrientDown;
    Orient untaredOrientQuat;
    float untaredOrientEuler[3];
    Matrix3x3 untaredOrientMatrix;
    Vector3 untaredOrientAxis;
    float untaredOrientAngle;
    Vector3 untaredOrientNorth;
    Vector3 untaredOrientGravity;

    U8 rawData[TSS_MAX_STREAM_PACKET_RAW_DATA_SIZE];
    U32 rawDataSize;
};

//void swap16(U8* data);
//void swap32(U8* data);
//float get_time();
//void sleep_ms(U32 time_ms);
#define TSS_STREAM_PACKET_CIRCULAR_BUFFER_SIZE 1024

struct TssStreamPacketCircularBuffer
{
    TssStreamPacket buff[TSS_STREAM_PACKET_CIRCULAR_BUFFER_SIZE];
    U32 start;
    U32 end;
    U32 len;

    void clear();
    void removeFirstPacket();
    void addPacket(TssHeader* header, U8* data, U32 nbytes);
    void getPacket(TssStreamPacket* packet, U32 index);
};

#define TSS_DEVICE_TYPE_SENSOR 0
#define TSS_DEVICE_TYPE_DONGLE 1

void _tssParseResponseHeader(U8* data, U8 flags, TssHeader* header);

class TssDevice
{
public:
    TssDevice();
    virtual ~TssDevice();

    bool isConnected();
    TssHeader* getLastHeader();
    U32 getSerialNumber();
    TSS_TYPE getSensorType();

    //Port functions
    TSS_RESULT _openPort(std::string name);
    TSS_RESULT _writeBytes(U8* data, U8 n_bytes);
    TSS_RESULT _readBytes(U8* data, U8 n_bytes);
    void _purgeInput();

    //Response header functions
    TSS_RESULT _readResponseHeader();
    TSS_RESULT _setResponseHeader(U32 flags);

    //Command sending support functions
    TSS_RESULT _sendCommandBytes(const U8* data, U8 n_bytes, bool ignore_response = false);
    TSS_RESULT _sendCommand(U8 command, bool ignore_response = false);
    TSS_RESULT _readFloats(float* data, U8 n_floats);
    TSS_RESULT _parseFloats(float* data, U8 n_floats);
    TSS_RESULT _checkedCommandWriteRead(U8 command, U8* data, U8 n_bytes);
    TSS_RESULT _checkedCommandWriteReadFloats(U8 command, float* data, U8 n_floats);

    //Lower level command functions
    TSS_RESULT _readSerialNumber();
    TSS_RESULT _readVersionString();

    U8 _type;
    bool _isportOwner;
    bool _isWireless;
    U32 _responseHeaderFlags;
    U8 _responseHeaderSize;
    string _portName;
    shared_ptr<Serial> _port;
    U8 _logicalId;
    TssHeader _lastHeader;
    U32 _serialNumber;
    std::string _versionString;
    TSS_TYPE _sensorType;
    U8 _commandRetries;
    bool _isStreaming;

};


#define BASIC_CALL_CHECK_TSS() if (!isConnected()){return TSS_ERROR_NOT_CONNECTED;}if (_isStreaming){return TSS_ERROR_COMMAND_CALLED_DURING_STREAMING;}
