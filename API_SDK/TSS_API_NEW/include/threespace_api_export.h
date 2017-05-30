/********************************************//**
* \ingroup threespace_api
* \file threespace_api_export.h
* \section Details
* \author Chris George
* \author Steve Landers
* \author Travis Lynn
* \version YOST 3-Space API 1.0.0
* \copyright Copyright 1998-2014, Yost Labs Corporation
*
* \brief This file is a collection of convenience functions for using the Yost 3-Space devices
* for use in a program written in C/C++ or any language that can import a compiled library (.dll, .so, etc).
*
* The Yost 3-Space API is released under the Yost 3-Space Open Source License, which allows for both
* non-commercial use and commercial use with certain restrictions.
*
* For Non-Commercial Use, your use of Covered Works is governed by the GNU GPL v.3, subject to the Yost 3-Space Open
* Source Licensing Overview and Definitions.
*
* For Commercial Use, a Yost Commercial/Redistribution License is required, pursuant to the Yost 3-Space Open Source
* Licensing Overview and Definitions. Commercial Use, for the purposes of this License, means the use, reproduction
* and/or Distribution, either directly or indirectly, of the Covered Works or any portion thereof, or a Compilation,
* Improvement, or Modification, for Pecuniary Gain. A Yost Commercial/Redistribution License may or may not require
* payment, depending upon the intended use.
*
* Full details of the Yost 3-Space Open Source License can be found in license.txt
* License also available online at http://www.yeitechnology.com/yei-3-space-open-source-license
***********************************************/
#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

	//useful types
	typedef uint8_t U8;
	typedef uint16_t U16;
	typedef uint32_t U32;

	typedef U32 tss_device_id;

	enum TSS_RESULT
	{
		TSS_SUCCESS = 0,
		TSS_ERROR_NOT_ENOUGH_DATA = 1,
		TSS_ERROR_CANT_OPEN_PORT = 2,
		TSS_ERROR_NOT_AVAILABLE_WIRELESS = 3,
		TSS_ERROR_ALREADY_STREAMING = 4,
		TSS_ERROR_NOT_STREAMING = 5,
		TSS_ERROR_LOGICAL_ID_OUT_OF_RANGE = 6,
		TSS_ERROR_CHILD_EXISTS = 7,
		TSS_ERROR_COMMAND_FAILURE = 8,
		TSS_ERROR_CANT_STOP_STREAMING = 9,
		TSS_ERROR_NO_WIRELESS_STREAMING_FROM_SENSOR = 10,
		TSS_ERROR_COMMAND_CALLED_DURING_STREAMING = 11,
		TSS_ERROR_NOT_CONNECTED = 12,
		TSS_ERROR_CANT_CLOSE_WIRELESS_PORT = 13,
		TSS_ERROR_ALREADY_CONNECTED = 14,
		TSS_ERROR_INVALID_ID = 15,
		TSS_ERROR_CANT_READ_INIT_DATA = 16,
		TSS_ERROR_WIRELESS_ONLY = 17,
		TSS_ERROR_SENSOR_TYPE_MISMATCH = 18,
		TSS_ERROR_CHILD_DOESNT_EXIST = 19
	};

	static const char* tss_result_string[] = {
		"TSS_SUCCESS",
		"TSS_ERROR_NOT_ENOUGH_DATA",
		"TSS_ERROR_CANT_OPEN_PORT",
		"TSS_ERROR_NOT_AVAILABLE_WIRELESS",
		"TSS_ERROR_ALREADY_STREAMING",
		"TSS_ERROR_NOT_STREAMING",
		"TSS_ERROR_LOGICAL_ID_OUT_OF_RANGE",
		"TSS_ERROR_CHILD_EXISTS",
		"TSS_ERROR_COMMAND_FAILURE",
		"TSS_ERROR_CANT_STOP_STREAMING",
		"TSS_ERROR_NO_WIRELESS_STREAMING_FROM_SENSOR",
		"TSS_ERROR_COMMAND_CALLED_DURING_STREAMING",
		"TSS_ERROR_NOT_CONNECTED",
		"TSS_ERROR_CANT_CLOSE_WIRELESS_PORT",
		"TSS_ERROR_ALREADY_CONNECTED",
		"TSS_ERROR_INVALID_ID",
		"TSS_ERROR_CANT_READ_INIT_DATA",
		"TSS_ERROR_WIRELESS_ONLY",
		"TSS_ERROR_SENSOR_TYPE_MISMATCH",
		"TSS_ERROR_CHILD_DOESNT_EXIST"
	};

#define TSS_STREAMING_PACKET_SIZE 128
#define TSS_PORT_NAME_SIZE 128
#define TSS_STREAM_DURATION_INFINITE 0xffffffff

	enum TSS_STREAM
	{
		TSS_STREAM_TARED_ORIENTATION_AS_QUATERNION = 1,             /**< Stream command to get the tared orientation as a quaternion */
        TSS_STREAM_TARED_ORIENTATION_AS_EULER_ANGLES = 2,           /**< Stream command to get the tared orientation as a euler angle */
        TSS_STREAM_TARED_ORIENTATION_AS_ROTATION_MATRIX = 4,        /**< Stream command to get the tared orientation as a rotation matrix */
        TSS_STREAM_TARED_ORIENTATION_AS_AXIS_ANGLE = 8,             /**< Stream command to get the tared orientation as a axis angle */
        TSS_STREAM_TARED_ORIENTATION_AS_TWO_VECTOR = 16,            /**< Stream command to get the tared orientation as two vectors */
        TSS_STREAM_UNTARED_ORIENTATION_AS_QUATERNION = 32,          /**< Stream command to get the untared orientation as a quaternion */
        TSS_STREAM_UNTARED_ORIENTATION_AS_EULER_ANGLES = 64,        /**< Stream command to get the untared orientation as a euler angle */
        TSS_STREAM_UNTARED_ORIENTATION_AS_ROTATION_MATRIX = 128,    /**< Stream command to get the untared orientation as a rotation matrix */
        TSS_STREAM_UNTARED_ORIENTATION_AS_AXIS_ANGLE = 256,         /**< Stream command to get the untared orientation as a axis angle */
        TSS_STREAM_UNTARED_ORIENTATION_AS_TWO_VECTOR = 512          /**< Stream command to get the untared orientation as two vectors */
    };

	enum TSS_RESPONSE_HEADER
	{
		TSS_RESPONSE_HEADER_SUCCESS = 1,
		TSS_RESPONSE_HEADER_TIMESTAMP = 2,
		TSS_RESPONSE_HEADER_COMMAND_ECHO = 4,
		TSS_RESPONSE_HEADER_CHECKSUM = 8,
		TSS_RESPONSE_HEADER_LOGICAL_ID = 16,
		TSS_RESPONSE_HEADER_SERIAL_NUMBER = 32,
		TSS_RESPONSE_HEADER_DATA_LENGTH = 64
	};

	enum TSS_TYPE
	{
		TSS_UNKNOWN = 1,    /**< Device type was not able to be determined */
        TSS_BTL = 2,        /**< 3-Space Bootloader */
		TSS_USB = 4,        /**< 3-Space USB */
		TSS_EM = 8,         /**< 3-Space Embedded */
		TSS_WL = 16,        /**< 3-Space Wireless */
		TSS_DNG = 32,       /**< 3-Space Dongle */
		TSS_DL = 64,        /**< 3-Space Data Logger */
		TSS_BT = 128,       /**< 3-Space Bluetooth */
        TSS_FIND_ALL_KNOWN = 2147483647
	};

#define TSS_WIRED (TSS_USB | TSS_EM | TSS_WL | TSS_DL | TSS_BT)

#if defined(_MSC_VER)
#define TSS_EXPORT __declspec(dllexport)
#elif defined(__GNUC__) && !defined(__APPLE__)
#define TSS_EXPORT __attribute__ ((dllexport))
#elif defined(__GNUC__) && defined(__APPLE__)
#define TSS_EXPORT
#endif

    /********************************************//**
    * \ingroup tss_api_methods
    * \brief Initalize the 3-Space API.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
	TSS_EXPORT TSS_RESULT tss_initAPI();
    /********************************************//**
    * \ingroup tss_api_methods
    * \brief Deconstruct the 3-Space API.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
	TSS_EXPORT TSS_RESULT tss_deinitAPI();

	////Sensor functions
    /********************************************//**
    * \ingroup tss_sensor_methods
    * \brief Creates a 3-Space Sensor ID.
    *
    * This creates and performs initial setup on the connected 3-Space Sensor.
    * The returned 3-Space Sensor ID is used to interact with the 3-Space Sensor.
    * When a 3-Space Sensor ID is created, other programs cannot use that serial port until the port is closed.
    * \param[in] port_name The serial port string for the serial port to be queried.
    * \param[out] out_id A tss_device_id will be returned to represent the 3-Space Sensor.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
	TSS_EXPORT TSS_RESULT tss_createSensor(const char* port_name, tss_device_id* out_id);
    /********************************************//**
    * \ingroup tss_sensor_methods
    * \brief This removes the connected 3-Space Sensor.
    * \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_RESULT tss_removeSensor(tss_device_id sensor_id);

	//Standard device functions
    /********************************************//**
    * \ingroup tss_sensor_methods
    * \brief Checks to see if the specified 3-Space Sensor. is connected.
    * \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
	TSS_EXPORT TSS_RESULT tss_sensor_isConnected(tss_device_id sensor_id);
    /********************************************//**
    * \ingroup tss_sensor_methods
    * \brief Retrieves the last timestamp logged by the 3-Space Sensor.
    * \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
    * \param[out] timestamp the returned timestamp in milliseconds.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
	TSS_EXPORT TSS_RESULT tss_sensor_getLastSensorTimestamp(tss_device_id sensor_id, U32* timestamp);
    /********************************************//**
    * \ingroup tss_sensor_methods
    * \brief Retrieves the last timestamp logged by the System.
    * \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
    * \param[out] timestamp the returned timestamp in milliseconds.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_RESULT tss_sensor_getLastSystemTimestamp(tss_device_id sensor_id, float* timestamp);
    /********************************************//**
    * \ingroup tss_sensor_methods
    * \brief Return the serial number of the 3-Space Sensor.
    * \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
    * \param[out] serial_number The serial number.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_RESULT tss_sensor_getSerialNumber(tss_device_id sensor_id, U32* serial_number);
    /********************************************//**
    * \ingroup tss_sensor_methods
    * \brief Retrieves the last timestamp logged by the 3-Space Sensor.
    * \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
    * \param[out] sensor_type The type of the 3-Space Sensor.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_RESULT tss_sensor_getSensorType(tss_device_id sensor_id, U8* sensor_type);
	
	//Port control functions
    /********************************************//**
    * \ingroup tss_sensor_methods
    * \brief Open a port to connect to 3-Space Sensor.
    * \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
    * \param[in] port_name The name of the communication port
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
	TSS_EXPORT TSS_RESULT tss_sensor_openPort(tss_device_id sensor_id, const char* port_name);
    /********************************************//**
    * \ingroup tss_sensor_methods
    * \brief Close the connected port
    * \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_RESULT tss_sensor_closePort(tss_device_id sensor_id);

	//Header control functions
    /********************************************//**
    * \ingroup tss_sensor_methods
    * \brief Enables timestamps for wired connection
    * \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_RESULT tss_sensor_enableTimestampsWired(tss_device_id sensor_id);
    /********************************************//**
    * \ingroup tss_sensor_methods
    * \brief Disables timestamps for wired connection
    * \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_RESULT tss_sensor_disableTimestampsWired(tss_device_id sensor_id);

	//Streaming functions
    /********************************************//**
    * \ingroup tss_sensor_methods
    * \brief Enable wireless streaming with the given flags.
    * \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
    * \param[in] data_flags The data flags reprsenting the data to stream. The flags are TSS_STREAM_TARED_ORIENTATION_AS_QUATERNION, TSS_STREAM_TARED_ORIENTATION_AS_EULER_ANGLES,
    * TSS_STREAM_TARED_ORIENTATION_AS_ROTATION_MATRIX, TSS_STREAM_TARED_ORIENTATION_AS_AXIS_ANGLE, TSS_STREAM_TARED_ORIENTATION_AS_TWO_VECTOR, TSS_STREAM_UNTARED_ORIENTATION_AS_QUATERNION,
	* TSS_STREAM_UNTARED_ORIENTATION_AS_EULER_ANGLES, TSS_STREAM_UNTARED_ORIENTATION_AS_ROTATION_MATRIX, TSS_STREAM_UNTARED_ORIENTATION_AS_AXIS_ANGLE, TSS_STREAM_UNTARED_ORIENTATION_AS_TWO_VECTOR.
    * \param[in] interval The interval to stream data.
    * \param[in] duration The duration of the stream.
    * \param[in] delay A delay before streaming.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_RESULT tss_sensor_enableStreamingWireless(tss_device_id sensor_id, U32 data_flags, U32 interval, U32 duration, U32 delay);
    /********************************************//**
    * \ingroup tss_sensor_methods
    * \brief Disables wireless streaming with the given flags.
    * \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_RESULT tss_sensor_disableStreamingWireless(tss_device_id sensor_id);
    /********************************************//**
    * \ingroup tss_sensor_methods
    * \brief Enables wired streaming with the given flags.
    * \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
    * \param[in] data_flags The data flags reprsenting the data to stream. The flags are TSS_STREAM_TARED_ORIENTATION_AS_QUATERNION, TSS_STREAM_TARED_ORIENTATION_AS_EULER_ANGLES,
    * TSS_STREAM_TARED_ORIENTATION_AS_ROTATION_MATRIX, TSS_STREAM_TARED_ORIENTATION_AS_AXIS_ANGLE, TSS_STREAM_TARED_ORIENTATION_AS_TWO_VECTOR, TSS_STREAM_UNTARED_ORIENTATION_AS_QUATERNION,
    * TSS_STREAM_UNTARED_ORIENTATION_AS_EULER_ANGLES, TSS_STREAM_UNTARED_ORIENTATION_AS_ROTATION_MATRIX, TSS_STREAM_UNTARED_ORIENTATION_AS_AXIS_ANGLE, TSS_STREAM_UNTARED_ORIENTATION_AS_TWO_VECTOR.
    * \param[in] interval The interval to stream data.
    * \param[in] duration The duration of the stream.
    * \param[in] delay A delay before streaming.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_RESULT tss_sensor_startStreamingWired(tss_device_id sensor_id, U32 data_flags, U32 interval, U32 duration, U32 delay);
    /********************************************//**
    * \ingroup tss_sensor_methods
    * \brief Return the oldest received packet.
    * \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
    * \param[out] packet_data The first packet in the buffer.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    //The given array of bytes must be TSS_MAX_STREAM_PACKET_RAW_DATA_SIZE long
    TSS_EXPORT TSS_RESULT tss_sensor_getFirstStreamingPacket(tss_device_id sensor_id, U8* packet_data);
    /********************************************//**
    * \ingroup tss_sensor_methods
    * \brief Return the newest received packet.
    * \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
    * \param[out] packet_data The first packet in the buffer.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_RESULT tss_sensor_getLastStreamingPacket(tss_device_id sensor_id, U8* packet_data);
    /********************************************//**
    * \ingroup tss_sensor_methods
    * \brief Disables wired streaming with the given flags.
    * \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_RESULT tss_sensor_stopStreamingWired(tss_device_id sensor_id);
    /********************************************//**
    * \ingroup tss_sensor_methods
    * \brief Get the number of unread stream packets.
    * \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
    * \param[out] in_waiting The number of unread packets.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_RESULT tss_sensor_getStreamingPacketsInWaiting(tss_device_id sensor_id, U32* in_waiting);
    /********************************************//**
    * \ingroup tss_sensor_methods
    * \brief Whether or not the 3-Space Sensor as overloaded and wrapped the buffer.
    * \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
    * \param[out] overflow The U8 representing whether of not the buffer has overloaded.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_RESULT tss_sensor_didStreamingOverflow(tss_device_id sensor_id, U8* overflow);

	//Call and response functions
    /********************************************//**
    * \ingroup tss_sensor_methods
    * \brief Sets the total number of times to retry a command before failing.
    * \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
    * \param[in] retries The number of times to retry a command before failing.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
	TSS_EXPORT TSS_RESULT tss_sensor_setCommandRetries(tss_device_id sensor_id, U8 retries);
    /********************************************//**
    * \ingroup tss_sensor_methods
    * \brief Get the tared orientation of a sensor as a Quaternion, it will be returned in a float array in the order (x,y,z,w).
    * \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
    * \param[out] orient_quat The oreintation of the given sensor as a quaterion.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_RESULT tss_sensor_getTaredOrientationAsQuaternion(tss_device_id sensor_id, float* orient_quat);
    /********************************************//**
    * \ingroup tss_sensor_methods
    * \brief Get the tared orientation of a sensor as a euler angle, it will be returned in a float array in the order (x,y,z).
    * \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
    * \param[out] euler The oreintation of the given sensor as a euler angle.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_RESULT tss_sensor_getTaredOrientationAsEulerAngles(tss_device_id sensor_id, float* euler);
    /********************************************//**
    * \ingroup tss_sensor_methods
    * \brief Get the tared orientation of a sensor as a Rotation Matrix, it will be returned in a float array in the order ([0][0],[0][1],[0][2],[1][0],[1][1],[1][2],[2][0],[2][1],[2][2]).
    * \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
    * \param[out] mat The oreintation of the given sensor as a rotation matrix.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_RESULT tss_sensor_getTaredOrientationAsRotationMatrix(tss_device_id sensor_id, float* mat);
    /********************************************//**
    * \ingroup tss_sensor_methods
    * \brief Get the tared orientation of a sensor as an Axis Angle , it will be returned in a float array in the order (x,y,z) and the angle.
    * \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
    * \param[out] vec The axis of the angle.
    * \param[out] angle The angle of the oreintation.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_RESULT tss_sensor_getTaredOrientationAsAxisAngle(tss_device_id sensor_id, float* vec, float* angle);
    /********************************************//**
    * \ingroup tss_sensor_methods
    * \brief Get the tared orientation of a sensor as a forward vector and a down vector, they will be returned as two float arrays in the order (x,y,z).
    * \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
    * \param[out] forward The forward vector.
    * \param[out] down The down vector.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_RESULT tss_sensor_getTaredOrientationAsTwoVector(tss_device_id sensor_id, float* forward, float* down);
    /********************************************//**
    * \ingroup tss_sensor_methods
    * \brief Get the untared orientation of a sensor as a Quaternion, it will be returned in a float array in the order (x,y,z,w).
    * \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
    * \param[out] orient_quat The oreintation of the given sensor as a quaterion.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_RESULT tss_sensor_getUntaredOrientationAsQuaternion(tss_device_id sensor_id, float* orient_quat);
    /********************************************//**
    * \ingroup tss_sensor_methods
    * \brief Get the untared orientation of a sensor, it will be returned in a float array in the order (x,y,z).
    * \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
    * \param[out] euler The oreintation of the given sensor as a euler angle.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_RESULT tss_sensor_getUntaredOrientationAsEulerAngles(tss_device_id sensor_id, float* euler);
    /********************************************//**
    * \ingroup tss_sensor_methods
    * \brief Get the untared orientation of a sensor as a Rotation Matrix, it will be returned in a float array in the order ([0][0],[0][1],[0][2],[1][0],[1][1],[1][2],[2][0],[2][1],[2][2]).
    * \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
    * \param[out] mat The oreintation of the given sensor as a rotation matrix.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_RESULT tss_sensor_getUntaredOrientationAsRotationMatrix(tss_device_id sensor_id, float* mat);
    /********************************************//**
    * \ingroup tss_sensor_methods
    * \brief Get the untared orientation of a sensor as an Axis Angle , it will be returned in a float array in the order (x,y,z) and the angle.
    * \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
    * \param[out] vec The axis of the angle.
    * \param[out] angle The angle of the oreintation.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_RESULT tss_sensor_getUntaredOrientationAsAxisAngle(tss_device_id sensor_id, float* vec, float* angle);
    /********************************************//**
    * \ingroup tss_sensor_methods
    * \brief Get the untared orientation of a sensor as a north vector and a gravity vector, it will be returned in as two float arrays in the order (x,y,z).
    * \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
    * \param[out] north The forward vector.
    * \param[out] gravity The down vector.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_RESULT tss_sensor_getUntaredOrientationAsTwoVector(tss_device_id sensor_id, float* north, float* gravity);
    /********************************************//**
    * \ingroup tss_sensor_methods
    * \brief Enumerate the 3-Space Sensor
    * \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_RESULT tss_sensor_tareWithCurrentOrientation(tss_device_id sensor_id);
    /********************************************//**
    * \ingroup tss_sensor_methods
    * \brief Return the current wireless channel of the 3-Space Sensor. In order for a Dongle to communicate with a 3-Space Sensor, they must both share the same channel.
    * \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
    * \param[out] channel The wireless channel ID.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_RESULT tss_sensor_getWirelessChannel(tss_device_id sensor_id, U8* channel);
    /********************************************//**
    * \ingroup tss_sensor_methods
    * \brief Set the wireless channel of the 3-Space Sensor. In order for a Dongle to communicate with a 3-Space Sensor, they must both share the same channel. Note that channel must be in the range 11 � 26 inclusive.
    * \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
    * \param[in] channel The wireless channel ID.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_RESULT tss_sensor_setWirelessChannel(tss_device_id sensor_id, U8 channel);
    /********************************************//**
    * \ingroup tss_sensor_methods
    * \brief Return the current wireless panID of the 3-Space Sensor. In order for a Dongle to communicate with a 3-Space Sensor, they must both share the same panID.
    * \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
    * \param[out] pan_id The wireless pan ID.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_RESULT tss_sensor_getWirelessPanID(tss_device_id sensor_id, U16* pan_id);
    /********************************************//**
    * \ingroup tss_sensor_methods
    * \brief Set the wireless panID of the 3-Space Sensor. In order for a Dongle to communicate with a 3-Space Sensor, they must both share the same panID. Note that panIDs must be in the range 1 � 65534 inclusive.
    * \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
    * \param[in] pan_id The wireless pan ID.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_RESULT tss_sensor_setWirelessPanID(tss_device_id sensor_id, U16 pan_id);
    /********************************************//**
    * \ingroup tss_sensor_methods
    * \brief Commits all committable settings to non-volatile memory so that they will persist beyond a powercycle. For more information on Commitable Settings, please refer to the PrioVR User's Manual.
    * \param[in] sensor_id The tss_device_id that represents the 3-Space Sensor.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_RESULT tss_sensor_commitSettings(tss_device_id sensor_id);
    /********************************************//**
    * \ingroup tss_dongle_methods
    * \brief Commits all committable wireless settings to non-volatile memory so that they will persist beyond a powercycle. For more information on Commitable Settings, please refer to the PrioVR User's Manual.
    * \param[in] sensor_id The tss_device_id that represents the 3-Space Dongle.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_RESULT tss_sensor_commitWirelessSettings(tss_device_id sensor_id);

	////Dongle functions
    /********************************************//**
    * \ingroup tss_dongle_methods
    * This creates and performs initial setup on the connected 3-Space Dongle.
    * The returned Basestation ID is used to interact with the 3-Space Dongle.
    * When a HUB ID is created, other programs cannot use that serial port until the port is closed.
    * \param[in] port_name The serial port string for the serial port to be queried.
    * \param[out] out_id A tss_device_id will be returned to represent the Dongle.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
	TSS_EXPORT TSS_RESULT tss_createDongle(const char* port_name, tss_device_id* out_id);
    /********************************************//**
    * \ingroup tss_dongle_methods
    * \brief This removes the connected 3-Space Dongle.
    * \param[in] dongle_id The tss_device_id that represents the Prio Basestation.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_RESULT tss_removeDongle(tss_device_id dongle_id);

	//Standard device functions
    /********************************************//**
    * \ingroup tss_dongle_methods
    * \brief Checks to see if the specified 3-Space Dongle is connected.
    * \param[in] dongle_id The tss_device_id that represents the 3-Space Dongle.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_RESULT tss_dongle_isConnected(tss_device_id dongle_id);
    /********************************************//**
    * \ingroup tss_dongle_methods
    * \brief Retrieves the last timestamp logged by the System.
    * \param[in] dongle_id The tss_device_id that represents the 3-Space Dongle.
    * \param[out] timestamp The returned timestamp in milliseconds.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_RESULT tss_dongle_getLastSensorTimestamp(tss_device_id dongle_id, U32* timestamp);
    /********************************************//**
    * \ingroup tss_dongle_methods
    * \brief Retrieves the last timestamp logged by the Dongle.
    * \param[in] dongle_id The tss_device_id that represents the 3-Space Dongle.
    * \param[out] timestamp The returned timestamp in milliseconds.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_RESULT tss_dongle_getLastSystemTimestamp(tss_device_id dongle_id, float* timestamp);
    /********************************************//**
    * \ingroup tss_dongle_methods
    * \brief Retrieves the last timestamp logged by the Dongle.
    * \param[in] dongle_id The tss_device_id that represents the 3-Space Dongle.
    * \param[out] timestamp The returned timestamp in milliseconds.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_RESULT tss_dongle_getSerialNumber(tss_device_id dongle_id, U32* timestamp);

	//Port control functions
    /********************************************//**
    * \ingroup tss_dongle_methods
    * \brief Open a port to connect to 3-Space Dongle.
    * \param[in] dongle_id The tss_device_id that represents the 3-Space Dongle.
    * \param[in] port_name The name of the communication port
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_RESULT tss_dongle_openPort(tss_device_id dongle_id, const char* port_name);
    /********************************************//**
    * \ingroup tss_dongle_methods
    * \brief Close the connected port
    * \param[in] dongle_id The tss_device_id that represents the 3-Space Dongle.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_RESULT tss_dongle_closePort(tss_device_id dongle_id);

	//Header control functions
    /********************************************//**
    * \ingroup tss_dongle_methods
    * \brief Enables timestamps over wireless.
    * \param[in] dongle_id The tss_device_id that represents the 3-Space Dongle.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_RESULT tss_dongle_enableTimestampsWireless(tss_device_id dongle_id);
    /********************************************//**
    * \ingroup tss_dongle_methods
    * \brief Disables timestamps over wireless.
    * \param[in] dongle_id The tss_device_id that represents the 3-Space Dongle.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_RESULT tss_dongle_disableTimestampsWireless(tss_device_id dongle_id);

	//Wireless sensor functions
    /********************************************//**
    * \ingroup tss_dongle_methods
    * \brief Get the paried 3-Space Sensor.
    * \param[in] dongle_id The tss_device_id that represents the 3-Space Dongle.
    * \param[in] logical_id The logical ID that represents the 3-Space Sensor.
    * \param[in] out_id The tss_device_id that represents the connected 3-Space Sensor.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_RESULT tss_dongle_getWirelessSensor(tss_device_id dongle_id, U8 logical_id, tss_device_id* out_id);
    /********************************************//**
    * \ingroup tss_dongle_methods
    * \brief Remove the paried 3-Space Sensor.
    * \param[in] dongle_id The tss_device_id that represents the 3-Space Dongle.
    * \param[in] logical_id The logical ID that represents the 3-Space Sensor.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_RESULT tss_dongle_removeWirelessSensor(tss_device_id dongle_id, U8 logical_id);

	//Streaming functions
    /********************************************//**
    * \ingroup tss_sensor_methods
    * \brief Enables wireless streaming with the given flags.
    * \param[in] dongle_id The tss_device_id that represents the 3-Space Sensor.
    * \param[in] data_flags The data flags reprsenting the data to stream. The flags are TSS_STREAM_TARED_ORIENTATION_AS_QUATERNION, TSS_STREAM_TARED_ORIENTATION_AS_EULER_ANGLES,
    * TSS_STREAM_TARED_ORIENTATION_AS_ROTATION_MATRIX, TSS_STREAM_TARED_ORIENTATION_AS_AXIS_ANGLE, TSS_STREAM_TARED_ORIENTATION_AS_TWO_VECTOR, TSS_STREAM_UNTARED_ORIENTATION_AS_QUATERNION,
    * TSS_STREAM_UNTARED_ORIENTATION_AS_EULER_ANGLES, TSS_STREAM_UNTARED_ORIENTATION_AS_ROTATION_MATRIX, TSS_STREAM_UNTARED_ORIENTATION_AS_AXIS_ANGLE, TSS_STREAM_UNTARED_ORIENTATION_AS_TWO_VECTOR.
    * \param[in] interval The interval to stream data.
    * \param[in] duration The duration of the stream.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_RESULT tss_dongle_enableAllSensorsAndStartStreaming(tss_device_id dongle_id, U32 data_flags, U32 interval, U32 duration);
    /********************************************//**
    * \ingroup tss_dongle_methods
    * \brief Start the streaming with the given flags.
    * \param[in] dongle_id The tss_device_id that represents the 3-Space Dongle.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_RESULT tss_dongle_startStreaming(tss_device_id dongle_id);
    /********************************************//**
    * \ingroup tss_dongle_methods
    * \brief Stops the streaming
    * \param[in] dongle_id The tss_device_id that represents the 3-Space Dongle.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_RESULT tss_dongle_stopStreaming(tss_device_id dongle_id);

	//Call and response functions
    /********************************************//**
    * \ingroup tss_dongle_methods
    * \brief Return the current wireless channel of the  3-Space Dongle. In order for a Dongle to communicate with a 3-Space Sensor, they must both share the same channel.
    * \param[in] dongle_id The tss_device_id that represents the 3-Space Dongle.
    * \param[out] channel The wireless channel ID.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_RESULT tss_dongle_getWirelessChannel(tss_device_id dongle_id, U8* channel);
    /********************************************//**
    * \ingroup tss_dongle_methods
    * \brief Set the wireless channel of the 3-Space Dongle. In order for a Dongle to communicate with a 3-Space Sensor, they must both share the same channel. Note that channel must be in the range 11 � 26 inclusive.
    * \param[in] dongle_id The tss_device_id that represents the 3-Space Dongle.
    * \param[in] channel The wireless channel ID.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_RESULT tss_dongle_setWirelessChannel(tss_device_id dongle_id, U8 channel);
    /********************************************//**
    * \ingroup tss_dongle_methods
    * \brief Return the current wireless panID of the 3-Space Dongle. In order for a Dongle to communicate with a 3-Space Sensor, they must both share the same panID.
    * \param[in] dongle_id The tss_device_id that represents the 3-Space Dongle.
    * \param[out] pan_id The wireless pan ID.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_RESULT tss_dongle_getWirelessPanID(tss_device_id dongle_id, U16* pan_id);
    /********************************************//**
    * \ingroup tss_dongle_methods
    * \brief Set the wireless panID of the 3-Space Dongle. In order for a Dongle to communicate with a 3-Space Sensor, they must both share the same panID. Note that panIDs must be in the range 1 � 65534 inclusive.
    * \param[in] dongle_id The tss_device_id that represents the 3-Space Dongle.
    * \param[in] pan_id The wireless pan ID.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_RESULT tss_dongle_setWirelessPanID(tss_device_id dongle_id, U16 pan_id);
    /********************************************//**
    * \ingroup tss_dongle_methods
    * \brief Return the serial number of the PrioVR Device.
    * \param[in] dongle_id The tss_device_id that represents the 3-Space Dongle.
    * \param[in] logical_id The tss_device_id that represents the 3-Space Device.
    * \param[out] serial_number The serial number.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_RESULT tss_dongle_getSerialNumberAtLogicalID(tss_device_id dongle_id, U8 logical_id, U32* serial_number);
    /********************************************//**
    * \ingroup tss_dongle_methods
    * \brief Set the serial number of the PrioVR Device.
    * \param[in] dongle_id The tss_device_id that represents the 3-Space Dongle.
    * \param[in] logical_id The tss_device_id that represents the 3-Space Device.
    * \param[in] serial_number The serial number.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_RESULT tss_dongle_setSerialNumberAtLogicalID(tss_device_id dongle_id, U8 logical_id, U32 serial_number);
    /********************************************//**
    * \ingroup tss_dongle_methods
    * \brief Commits all committable wireless settings to non-volatile memory so that they will persist beyond a powercycle. For more information on Commitable Settings, please refer to the PrioVR User's Manual.
    * \param[in] dongle_id The tss_device_id that represents the 3-Space Dongle.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    TSS_EXPORT TSS_RESULT tss_dongle_commitWirelessSettings(tss_device_id dongle_id);

    //Port functions
    /********************************************//**
    * \ingroup tss_port_methods
    * \brief Find Prio Devices connected to the computer.
    * \param[in] find_flags The flags for what devices to look for. 1-TSS_UNKNOWN, 2-TSS_BTL, 4-TSS_USB, 8-TSS_EM, 16-TSS_WL, 32-TSS_DNG, 64-TSS_DL, 128-TSS_BT, 2147483647-TSS_FIND_ALL_KNOWN.
    ***********************************************/
	TSS_EXPORT void tss_findSensorPorts(U32 find_flags);
    /********************************************//**
    * \ingroup tss_port_methods
    * \brief Return the COM Port, and attached 3-Space Device type on the next unused COM Port.
    * \param[out] port_name The COM Port.
    * \param[out] sensor_type The type of 3-Space Device.
    * \return U32 if successful 1, if error 0.
    ***********************************************/
	//port_name must be TSS_PORT_NAME_SIZE characters long
	TSS_EXPORT U32 tss_getNextSensorPort(char* port_name, U8* sensor_type);
}