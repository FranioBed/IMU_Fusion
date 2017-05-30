#pragma once

#include "threespace_device.hpp"

class TssDongle;

#define TSS_NUM_STREAMING_SLOTS 8

class TssSensor : public TssDevice
{
public:
	TssSensor();
	TssSensor(std::string port);
	TssSensor(const TssSensor& other);

	void operator =(const TssSensor& other);

	//Port control functions
	TSS_RESULT openPort(std::string port);
	TSS_RESULT closePort();

	//Header control functions
	TSS_RESULT enableTimestampsWired();
	TSS_RESULT disableTimestampsWired();

	//Streaming functions
	TSS_RESULT enableStreamingWireless(U32 data_flags, U32 interval, U32 duration, U32 delay = 0);
	TSS_RESULT disableStreamingWireless();
	TSS_RESULT startStreamingWired(U32 data_flags, U32 interval, U32 duration, U32 delay = 0);
	TSS_RESULT getFirstStreamingPacket(TssStreamPacket* packet);
	TSS_RESULT getLastStreamingPacket(TssStreamPacket* packet);
	TSS_RESULT stopStreamingWired();
	U32 getStreamingPacketsInWaiting();
	bool didStreamingOverflow();

	//Call and response functions
	void setCommandRetries(U8 retries);
	TSS_RESULT getTaredOrientationAsQuaternion(Orient* quat);
	TSS_RESULT getTaredOrientationAsEulerAngles(float* euler);
	TSS_RESULT getTaredOrientationAsRotationMatrix(Matrix3x3* mat);
	TSS_RESULT getTaredOrientationAsAxisAngle(Vector3* vec, float* angle);
	TSS_RESULT getTaredOrientationAsTwoVector(Vector3* forward, Vector3* down);
    TSS_RESULT getUntaredOrientationAsQuaternion(Orient* quat);
	TSS_RESULT getUntaredOrientationAsEulerAngles(float* euler);
	TSS_RESULT getUntaredOrientationAsRotationMatrix(Matrix3x3* mat);
	TSS_RESULT getUntaredOrientationAsAxisAngle(Vector3* vec, float* angle);
	TSS_RESULT getUntaredOrientationAsTwoVector(Vector3* north, Vector3* gravity);
	TSS_RESULT tareWithCurrentOrientation();
	TSS_RESULT getWirelessChannel(U8* channel);
	TSS_RESULT setWirelessChannel(U8 channel);
	TSS_RESULT getWirelessPanID(U16* panid);
	TSS_RESULT setWirelessPanID(U16 panid);
	TSS_RESULT commitSettings();
	TSS_RESULT commitWirelessSettings();


	//not private, but not meant to be called by end-users
	void _parseStreamingPacketItem(U8* dest, U8* src, U16& src_index, U8 nbytes);
	void _parseStreamingPacketItemFloats(float* dest, U8* src, U16& src_index, U8 nfloats);
	TSS_RESULT _parseStreamingPacket(TssStreamPacket* packet);
	void _addToStreamingSlots(U8 command, U8 nbytes, U8* command_buff, U8& curr_slot);
	TSS_RESULT _prepareStreaming(U32 data_flags, U32 interval, U32 duration, U32 delay = 0);
	TSS_RESULT _triggerStreaming();
	TSS_RESULT _stopStreamingNoPurge();

	U8 _streamingSlots[8];
	U8 _streamingSlotIndex[8];
	U8 _streamDataSize;
	TssStreamPacketCircularBuffer _streamBuffer;
	U32 _streamInterval;
	TssDongle* _ownerDongle;
};