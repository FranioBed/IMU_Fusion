#pragma once

#include "threespace_sensor.hpp"

#define TSS_DONGLE_NUM_CHILDREN 15
#define TSS_DEFAULT_WIRELESS_COMMAND_RETRIES 5

class TssDongle : public TssDevice
{
public:
	TssDongle(string port);
	TssDongle(const TssDongle& other);

	void operator =(const TssDongle& other);

	//Port control functions
	TSS_RESULT openPort(std::string port);
	TSS_RESULT closePort();

	//Header control functions
	TSS_RESULT enableTimestampsWireless();
	TSS_RESULT disableTimestampsWireless();

	//Wireless sensor functions
	TSS_RESULT getWirelessSensor(U8 logical_id, shared_ptr<TssSensor>& sensor);
	TSS_RESULT removeWirelessSensor(U8 logical_id);

	//Streaming functions
	TSS_RESULT enableAllSensorsAndStartStreaming(U32 data_flags, U32 interval, U32 duration);
	TSS_RESULT startStreaming();
	TSS_RESULT stopStreaming();

	//Call and response functions
	TSS_RESULT getWirelessChannel(U8* channel);
	TSS_RESULT setWirelessChannel(U8 channel);
	TSS_RESULT getWirelessPanID(U16* panid);
	TSS_RESULT setWirelessPanID(U16 panid);
	TSS_RESULT getSerialNumberAtLogicalID(U8 logical_id, U32* serial_number);
	TSS_RESULT setSerialNumberAtLogicalID(U8 logical_id, U32 serial_number);
	TSS_RESULT commitWirelessSettings();

	TSS_RESULT _setWirelessResponseHeader(U32 flags);
	U16 _detectStreaming(U32 interval_us);

	vector<shared_ptr<TssSensor>> _children; //always has TSS_DONGLE_NUM_CHILDREN elements, unused elements are null
	U32 _responseHeaderWirelessFlags;
	U8 _responseHeaderWirelessSize;
	U32 _responseHeaderWirelessFlagsPreStream;
	U32 _maxStreamInterval;
	U8 _childCount;
};