/********************************************//**
* \ingroup prio_api
* \file prio_api_export.h
* \section Details
* \author Chris George
* \author Steve Landers
* \author Travis Lynn
* \author Nick Leyder
* \version YOST Prio API 1.0.0
* \copyright Copyright 1998-2016, Yost Labs Corporation
*
* \brief This file is a collection of convenience functions for using the Yost PrioVR devices
* for use in a program written in C/C++ or any language that can import a compiled library (.dll, .so, etc).
*
* The Yost Prio API is released under the Yost 3-Space Open Source License, which allows for both
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
#include <string>

#ifdef __cplusplus
extern "C"
{
#endif

#if defined(_MSC_VER)
#define PRIO_EXPORT __declspec(dllexport)
#elif defined(__GNUC__) && !defined(__APPLE__)
#define PRIO_EXPORT __attribute__ ((dllexport))
#elif defined(__GNUC__) && defined(__APPLE__)
#define PRIO_EXPORT
#endif

    //useful types
    typedef uint8_t U8;
    typedef uint16_t U16;
    typedef uint32_t U32;
    typedef uint64_t U64;

    /********************************************//**
     * \ingroup prio_api
     * The default number of soft retries on wireless.
     * If prio_resetPrioApi is called the retries will be set back to this value.
     ***********************************************/
    #define PRIO_DEFAULT_WIRLESSS_RETRIES 1

    /********************************************//**
     * \ingroup prio_api
     * The logical id for the PrioVR Basestation device.
     ***********************************************/
    #define PRIO_BASE_STATION_LOGICAL_ID 0xfe

    /********************************************//**
     * \ingroup prio_api
     * The logical id for the PrioVR Hub device.
     ***********************************************/
    #define PRIO_HUB_LOGICAL_ID 0xf0

    /********************************************//**
     * \ingroup prio_api
     * The logical id for broadcasting the command.
     ***********************************************/
    #define PRIO_BROADCAST_LOGICAL_ID 0xf1

#define PRIO_STREAMING_PACKET_SIZE 128
#define PRIO_PORT_NAME_SIZE 128
#define PRIO_STREAM_DURATION_INFINITE 0xffffffff

    /********************************************//**
     * \ingroup prio_api
     * An enum expressing masks used to quickly determine the type of a device.
     ***********************************************/
    typedef enum PRIO_DEVICE_ID_MASK
    {
        PRIO_NO_DEVICE_ID        = 0x0100,  /**< Invalid Device ID */
        PRIO_BASE_STATION_ID     = 0x0200,  /**< Basestation ID */
        PRIO_HUB_ID              = 0x0400   /**< Hub ID */
    }PRIO_DEVICE_ID_MASK;

    /********************************************//**
     * \ingroup prio_api
     * Yost Prio API device identifier, a common parameter needed for all Prio API calls.
     ***********************************************/
    typedef U32 prio_device_id;

    /********************************************//**
     * \ingroup prio_api
     * An enum expressing the compatibility level of the PrioVR device.
     ***********************************************/
    typedef enum PRIO_FIRMWARE_COMPATIBILITY
    {
        PRIO_FW_NOT_COMPATIBLE,     /**< Firmware is not compatible with the API and should be updated */
        PRIO_FW_010_COMPATIBLE,      /**< PrioVR device compatible with 0.1.0 commands and up */
        PRIO_FW_015_COMPATIBLE
    }PRIO_FIRMWARE_COMPATIBILITY;

    /********************************************//**
     * \ingroup prio_api
     * A c_string array to help express the compatibility level of the PrioVR device.
     ***********************************************/
    static const char* const prio_firmware_version_string[] =
    {
        "00Jan2014",
        "23Jun2014",
        "15Apr2015"
    };

    /********************************************//**
     * \ingroup prio_api
     * An enum expressing the different types of errors a Prio API call can return.
     ***********************************************/
    typedef enum PRIO_ERROR
    {
        PRIO_NO_ERROR,                      /**< The API call successfully executed */
        PRIO_ERROR_COMMAND_FAIL,            /**< The command returned a failed response */
        PRIO_ERROR_COMMAND_FAIL_COMMUNICATION,
        PRIO_ERROR_COMMAND_FAIL_ENUMERATION_BIT_UNREAD,
        PRIO_ERROR_COMMAND_FAIL_INVALID_ARGS,
        PRIO_INVALID_COMMAND,               /**< The API call was made on a device type that does not support the attempted command */
        PRIO_INVALID_LOGICAL_ID,            /**< The logical id parameter passed in to an API call is not allowed */
        PRIO_INVALID_DEVICE_ID,             /**< The prio_device_id parameter passed in to an API call is not associated with a connected PrioVR device */
        PRIO_ERROR_PARAMETER,               /**< General parameter fail */
        PRIO_ERROR_TIMEOUT,                 /**< The command's timeout has been reached */
        PRIO_ERROR_WRITE,                   /**< The API call executed failed to write all the data necessary to execute the command to the intended device */
        PRIO_ERROR_READ,                    /**< The API call executed failed to read all the data necessary to execute the command to the intended device */
        PRIO_ERROR_STREAM_SLOTS_FULL,       /**< The device's stream slots are full */
        PRIO_ERROR_STREAM_CONFIG,           /**< The device's stream configuration is corrupted */
        PRIO_ERROR_MEMORY,                  /**< A memory error occurred in the API */
        PRIO_ERROR_FIRMWARE_INCOMPATIBLE,   /**< Device firmware does not support that command, firmware update highly recommended */
        PRIO_ERROR_RESET_API                /**< The API failed to reset itself */
    }PRIO_ERROR;

    /********************************************//**
     * \ingroup prio_api
     * A c_string array to help express the different types of errors a Prio API call can return.
     ***********************************************/
    static const char* const prio_error_string[] = {
        "PRIO_NO_ERROR",
        "PRIO_ERROR_COMMAND_FAIL",
        "PRIO_ERROR_COMMAND_FAIL_COMMUNICATION",
        "PRIO_ERROR_COMMAND_FAIL_ENUMERATION_BIT_UNREAD",
        "PRIO_ERROR_COMMAND_FAIL_INVALID_ARGS",
        "PRIO_INVALID_COMMAND",
        "PRIO_INVALID_LOGICAL_ID",
        "PRIO_INVALID_DEVICE_ID",
        "PRIO_ERROR_PARAMETER",
        "PRIO_ERROR_TIMEOUT",
        "PRIO_ERROR_WRITE",
        "PRIO_ERROR_READ",
        "PRIO_ERROR_STREAM_SLOTS_FULL",
        "PRIO_ERROR_STREAM_CONFIG",
        "PRIO_ERROR_MEMORY",
        "PRIO_ERROR_FIRMWARE_INCOMPATIBLE",
        "PRIO_ERROR_RESET_API"
    };

    /********************************************//**
     * \ingroup prio_api
     * An enum expressing the possible Prio API Timestamp Modes.
     ***********************************************/
    typedef enum PRIO_TIMESTAMP_MODE
    {
        PRIO_TIMESTAMP_NONE,    /**< Disables timestamp, timestamp will return 0 */
        PRIO_TIMESTAMP_SENSOR,  /**< PrioVR device's timestamp, this can be set with prio_setTimestamp */
        PRIO_TIMESTAMP_SYSTEM   /**< The data is timestamped on arrival to the system using the high-resolution clock from the chrono library*/
    }PRIO_TIMESTAMP_MODE;

    /********************************************//**
     * \ingroup prio_api
     * An enum expressing the command list of Streamable Commands.
     ***********************************************/
    typedef enum PRIO_STREAM_COMMAND
    {
        PRIO_STREAM_TARED_ORIENTATION_AS_QUATERNION        = 1,         /**< Stream command to get the tared orientation as a quaternion */
        PRIO_STREAM_ALL_CORRECTED_COMPONENT_SENSOR_DATA    = 2,         /**< Stream command to get all corrected component sensor data */
        PRIO_STREAM_CORRECTED_GYRO_RATE                    = 4,         /**< Stream command to get the corrected gyro rate */
        PRIO_STREAM_CORRECTED_ACCELEROMETER_VECTOR         = 8,         /**< Stream command to get the corrected accelerometer vector */
        PRIO_STREAM_CORRECTED_MAGNETOMETER_VECTOR          = 16,        /**< Stream command to get the corrected magnetometer vector */
        PRIO_STREAM_UNTARED_ORIENTATION_AS_QUATERNION      = 32,        /**< Stream command to get the untared orientation as quaternion */
        PRIO_STREAM_ALL_RAW_COMPONENT_SENSOR_DATA          = 64,        /**< Stream command to get all raw component sensor data */
        PRIO_STREAM_RAW_GYROSCOPE_RATE                     = 128,       /**< Stream command to get the raw gyro rate */
        PRIO_STREAM_RAW_ACCELEROMETER_DATA                 = 256,       /**< Stream command to get the raw accelerometer vector */
        PRIO_STREAM_RAW_MAGNETOMETER_DATA                  = 512,       /**< Stream command to get the raw magnetometer vector */
        PRIO_NULL                                          = 0          /**< Null entry to return no data */
    }PRIO_STREAM_COMMAND;

     /********************************************//**
     * \ingroup prio_api
     * Yost Prio API streamable command.
     ***********************************************/
    typedef U32 prio_stream_commands;

    /********************************************//**
     * \ingroup prio_api
     * \brief A structure that contains information about the joystick.
     ***********************************************/
    typedef struct PrioJoystick
    {
        U8 x_Axis;                 /**< The horizontal value of the analog stick range -1 to 1 */
        U8 y_Axis;                 /**< The vertical value of the analog stick range -1 to 1 */
        U8 Trigger;                 /**< The value of the analog button range 0 to 1 */
        U8 ButtonState;            /**< Bit field of all buttons. */
    }PrioJoystick;

    /********************************************//**
     * \ingroup prio_api
     * \brief A structure that contains information about the stream header.
     ***********************************************/
    typedef struct prio_StreamHeaderData
    {
        uint8_t battery_level;          /**< Battery level of the PrioVR Hub. */
        U8 battery_status;               /**< Battery status of the PrioVR Hub. \li 0: Battery Status Normal\li 1: Battery Status Charging\li 2: Battery Status Charged\li 4: Battery Status Low */
        uint8_t hub_button;             /**< State of the Prio Hub buttons. \li 0: No Buttons\li 1: Button 1\li 2: Button 2\li 3: Both Buttons */
        PrioJoystick joystick1;        /**< The \ref prio_Joystick data of the first controller. */
        PrioJoystick joystick2;        /**< The \ref prio_Joystick data of the second controller. */
    }prio_StreamHeaderData;

    /********************************************//**
    * \ingroup prio_api
    * \brief A structure that contains the entire packet header data. NOTE: Some options may not be initialized. 
    ***********************************************/
    typedef struct PrioHeader
    {
        U8 Success;
        U32 SensorTimestamp;
        float SystemTimestamp;
        U8 CommandEcho;
        U8 Checksum;
        U8 LogicalId;
        U32 SerialNumber;
        U8 DataLength;
        U64 SensorBitfield;

        //PrioVR Data
        U8 BatteryLevel;                /**< Battery level of the PrioVR Hub.*/
        U8 BatteryStatus;               /**< Battery status of the PrioVR Hub. \li 0: Battery Status Normal\li 1: Battery Status Charging\li 2: Battery Status Charged\li 4: Battery Status Low */
        U8 HubButton;                   /**< State of the PrioVR Hub buttons. \li 0: No Buttons\li 1: Button 1\li 2: Button 2\li 3: Both Buttons */
        PrioJoystick Joystick1;         /**< The \ref prio_Joystick data of the first controller. */
        PrioJoystick Joystick2;         /**< The \ref prio_Joystick data of the second controller. */

    }PrioControllerData;


    /********************************************//**
     * \ingroup prio_api
     * An enum expressing the find flags for the prio_getComPorts filter parameter.
     ***********************************************/
    typedef enum PRIO_FIND
    {
        PRIO_FIND_BS         = 0x01,    /**< Find PrioVR Basestation */
        PRIO_FIND_HUB        = 0x02,    /**< Find PrioVR Hub */
        PRIO_FIND_ALL_KNOWN  = 0x03,     /**< Find all known PrioVR devices */
        PRIO_FIND_UNKNOWN    = 0x04,    /**< Find serial ports that may have PrioVR devices connected to them */
        PRIO_FIND_ALL        = 0x0f     /**< Find all serial ports including "unknown" serial ports that may have PrioVR devices connected */
    }PRIO_FIND;

    enum PRIO_RESPONSE_HEADER
    {
        PRIO_RESPONSE_HEADER_SUCCESS = 1,
        PRIO_RESPONSE_HEADER_TIMESTAMP = 2,
        PRIO_RESPONSE_HEADER_COMMAND_ECHO = 4,
        PRIO_RESPONSE_HEADER_LOGICAL_ID = 16,
        PRIO_RESPONSE_HEADER_DATA_LENGTH = 64
    };

    /********************************************//**
     * \ingroup prio_api
     * An enum expressing the types of PrioVR devices.
     ***********************************************/
    typedef enum PRIO_TYPE
    {
        PRIO_UNKNOWN,   /**< Device type was not able to be determined */
        PRIO_BS,        /**< PrioVR Basestation */
        PRIO_HUB,        /**< PrioVR Hub */
        PRIO_ALL
    }PRIO_TYPE;

    /********************************************//**
     * \ingroup prio_api
     * A c_string array to help express the types of PrioVR devices.
     ***********************************************/
    static const char* const prio_type_string[] =
    {
        "PRIO_UNKNOWN",
        "PRIO_BS",
        "PRIO_HUB"
    };

    /********************************************//**
     * \ingroup prio_api
     * An enum expressing the states of PrioVR devices.
     ***********************************************/
    typedef enum PRIO_DEVICE_STATE
    {
        PRIO_STATE_UNKNOWN,         /**< Device state is unknown */
        PRIO_STATE_DISCONNECTED,    /**< The device is disconnected */
        PRIO_STATE_IDLE,            /**< The device is connected and waiting for a command */
        PRIO_STATE_STREAMING,       /**< The device is streaming data */
        PRIO_STATE_RECORDING        /**< The device is recording data */
    }PRIO_DEVICE_STATE;

    /********************************************//**
     * \ingroup prio_api
     * A c_string array to help express the states of PrioVR devices.
     ***********************************************/
    static const char* const prio_state_string[] =
    {
        "PRIO_STATE_UNKNOWN",
        "PRIO_STATE_DISCONNECTED",
        "PRIO_STATE_IDLE",
        "PRIO_STATE_STREAMING",
        "PRIO_STATE_RECORDING"
    };

    /********************************************//**
     * \ingroup prio_api
     * \brief A structure that contains basic information about a serial port.
     ***********************************************/
    typedef struct prio_ComPort
    {
        char* port_name;      /**< The serial port string. */
        U8 device_type;       /**< The type of PrioVR device connected through the serial port. */
    }prio_ComPort;

    /********************************************//**
     * \ingroup prio_api
     * \brief A structure that contains information about the connected PrioVR device.
     ***********************************************/
    typedef struct prio_DeviceInfo
    {
        U8 device_type;                               /**< The type of PrioVR device connected through the serial port. */
        uint32_t serial_number;                       /**< The serial number for the PrioVR device connected through the serial port. */
        char* firmware_version;                       /**< The version of the firmware installed on the connected PrioVR device.\bug Some device firmware versions are truncated. */
        char* hardware_version;                       /**< The hardware revision and type of the connected PrioVR device. */
        PRIO_FIRMWARE_COMPATIBILITY fw_compatibility; /**< Firmware compatibility level. \note Level may be lower than current if no functional changes were made. */
    }prio_DeviceInfo;

    
    /********************************************//**
    * \ingroup prio_api_methods
    * \brief Initalize the Prio API.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_initAPI();
    /********************************************//**
    * \ingroup prio_api_methods
    * \brief Deconstruct the Prio API.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_deinitAPI();

    //General functions
    /********************************************//**
    * \ingroup prio_hub_methods
    * \brief Creates a Hub ID.
    *
    * This creates and performs initial setup on the connected PrioVR Hub.
    * The returned Hub ID is used to interact with the PrioVR Hub.
    * When a Hub ID is created, other programs cannot use that serial port until the port is closed.
    * \param[in] port_name The serial port string for the serial port to be queried.
    * \param[out] out_id A prio_device_id will be returned to represent the Hub.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_createHub(const char* port_name, prio_device_id* out_id);
    /********************************************//**
    * \ingroup prio_hub_methods
    * \brief This removes the connected PrioVR Hub.
    * \param[in] hub_id The prio_device_id that represents the PrioVR Hub.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_removeHub(prio_device_id hub_id);
    //Standard device functions
    /********************************************//**
    * \ingroup prio_hub_methods
    * \brief Checks to see if the specified PrioVR Hub is connected.
    * \param[in] hub_id The prio_device_id that represents the PrioVR Hub.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_hub_isConnected(prio_device_id hub_id);
    /********************************************//**
    * \ingroup prio_hub_methods
    * \brief Retrieves the last timestamp logged by the Hub.
    * \param[in] hub_id The prio_device_id that represents the PrioVR Hub.
    * \param[out] timestamp the returned timestamp in milliseconds.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_hub_getLastHubTimestamp(prio_device_id hub_id, U32* timestamp);
    /********************************************//**
    * \ingroup prio_hub_methods
    * \brief Retrieves the last timestamp logged by the System.
    * \param[in] hub_id The prio_device_id that represents the PrioVR Hub.
    * \param[out] timestamp the returned timestamp in milliseconds.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_hub_getLastSystemTimestamp(prio_device_id hub_id, float* timestamp);
    /********************************************//**
    * \ingroup prio_hub_methods
    * \brief Return the serial number of the PrioVR Hub.
    * \param[in] hub_id The prio_device_id that represents the PrioVR Hub.
    * \param[out] serial_number The serial number.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_hub_getSerialNumber(prio_device_id hub_id, U32* serial_number);
    /********************************************//**
    * \ingroup prio_basestation_methods
    * \brief Return the hardware version of the PrioVR Hub.
    * \param[in] hub_id The prio_device_id that represents the PrioVR Hub.
    * \param[out] hw_version The hardware version.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_hub_getHardwareVersion(prio_device_id hub_id, const char* hw_version);
    /********************************************//**
    * \ingroup prio_basestation_methods
    * \brief Return the firmware version of the PrioVR Hub.
    * \param[in] hub_id The prio_device_id that represents the PrioVR HUb.
    * \param[out] fw_version The firmware version.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_hub_getFirmwareVersion(prio_device_id hub_id, const char* fw_version);
    /********************************************//**
    * \ingroup prio_hub_methods
    * \brief Retrieves the sensor type of the PrioVR Hub.
    * \param[in] hub_id The prio_device_id that represents the PrioVR Hub.
    * \param[out] sensor_type The type of the PrioVR Device.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_hub_getSensorType(prio_device_id hub_id, U8* sensor_type);
    //Port control functions
    /********************************************//**
    * \ingroup prio_hub_methods
    * \brief Open a port to connect to the PrioVR Suit.
    * \param[in] hub_id The prio_device_id that represents the PrioVR Hub.
    * \param[in] port_name The name of the communication port
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_hub_openPort(prio_device_id hub_id, const char* port_name);
    /********************************************//**
    * \ingroup prio_hub_methods
    * \brief Close the connected port.
    * \param[in] hub_id The prio_device_id that represents the PrioVR Hub.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_hub_closePort(prio_device_id hub_id);

    //Header control functions
    /********************************************//**
    * \ingroup prio_hub_methods
    * \brief Enables timestamps for wired connection
    * \param[in] hub_id The prio_device_id that represents the PrioVR Hub.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_hub_enableTimestampsWired(prio_device_id hub_id);
    /********************************************//**
    * \ingroup prio_hub_methods
    * \brief Disables timestamps for wired connection
    * \param[in] hub_id The prio_device_id that represents the PrioVR Hub.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_hub_disableTimestampsWired(prio_device_id hub_id);

    //Streaming functions
    /********************************************//**
    * \ingroup prio_hub_methods
    * \brief Enable wireless streaming with the given flags.
    * \param[in] hub_id The prio_device_id that represents the PrioVR Hub.
    * \param[in] data_flags The data flags reprsenting the data to stream. The flags are PRIO_STREAM_TARED_ORIENTATION_AS_QUATERNION, PRIO_STREAM_UNTARED_ORIENTATION_AS_QUATERNION
    * PRIO_STREAM_ALL_CORRECTED_COMPONENT_SENSOR_DATA, PRIO_STREAM_CORRECTED_GYRO_RATE, PRIO_STREAM_CORRECTED_ACCELEROMETER_VECTOR), PRIO_STREAM_CORRECTED_MAGNETOMETER_VECTOR,
    * PRIO_STREAM_ALL_RAW_COMPONENT_SENSOR_DATA, PRIO_STREAM_RAW_GYROSCOPE_RATE, PRIO_STREAM_RAW_ACCELEROMETER_DATA, PRIO_STREAM_RAW_MAGNETOMETER_DATA.
    * \param[in] interval The interval to stream data.
    * \param[in] duration The duration of the stream.
    * \param[in] delay A delay before streaming.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_hub_enableStreamingWireless(prio_device_id hub_id, U32 data_flags, U32 interval, U32 duration, U32 delay=0);
    /********************************************//**
    * \ingroup prio_hub_methods
    * \brief Disables wireless streaming.
    * \param[in] hub_id The prio_device_id that represents the PrioVR Hub.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_hub_disableStreamingWireless(prio_device_id hub_id);
    /********************************************//**
    * \ingroup prio_hub_methods
    * \brief Enables wired streaming with the given flags.
    * \param[in] hub_id The prio_device_id that represents the PrioVR Hub.
    * \param[in] data_flags The data flags reprsenting the data to stream. The flags are PRIO_STREAM_TARED_ORIENTATION_AS_QUATERNION, PRIO_STREAM_UNTARED_ORIENTATION_AS_QUATERNION
    * PRIO_STREAM_ALL_CORRECTED_COMPONENT_SENSOR_DATA, PRIO_STREAM_CORRECTED_GYRO_RATE, PRIO_STREAM_CORRECTED_ACCELEROMETER_VECTOR), PRIO_STREAM_CORRECTED_MAGNETOMETER_VECTOR,
    * PRIO_STREAM_ALL_RAW_COMPONENT_SENSOR_DATA, PRIO_STREAM_RAW_GYROSCOPE_RATE, PRIO_STREAM_RAW_ACCELEROMETER_DATA, PRIO_STREAM_RAW_MAGNETOMETER_DATA.
    * \param[in] interval The interval to stream data.
    * \param[in] duration The duration of the stream.
    * \param[in] delay A delay before streaming.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_hub_startStreamingWired(prio_device_id hub_id, prio_stream_commands data_flags, U32 interval, U32 duration, U32 delay=0);
    /********************************************//**
    * \ingroup prio_hub_methods
    * \brief Disables wired streaming.
    * \param[in] hub_id The prio_device_id that represents the PrioVR Hub.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_hub_stopStreamingWired(prio_device_id hub_id);
    /********************************************//**
    * \ingroup prio_hub_methods
    * \brief Return the oldest received packet.
    * \param[in] hub_id The prio_device_id that represents the PrioVR Hub.
    * \param[in] header_data The header for the first packet in the buffer.
    * \param[out] packet_data The first packet in the buffer.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    //The given array of bytes must be prio_MAX_STREAM_PACKET_RAW_DATA_SIZE long
    PRIO_EXPORT PRIO_ERROR prio_hub_getFirstStreamingPacket(prio_device_id hub_id, prio_StreamHeaderData *header_data, U8* packet_data);
    /********************************************//**
    * \ingroup prio_hub_methods
    * \brief Return the newest received packet.
    * \param[in] hub_id The prio_device_id that represents the PrioVR Hub.
    * \param[in] header_data The header for the last packet in the buffer.
    * \param[out] packet_data The last packet in the buffer.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_hub_getLastStreamingPacket(prio_device_id hub_id, prio_StreamHeaderData *header_data, U8* packet_data);
    /********************************************//**
    * \ingroup prio_hub_methods
    * \brief Get the number of unread stream packets.
    * \param[in] hub_id The prio_device_id that represents the PrioVR Hub.
    * \param[out] in_waiting The number of unread packets.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_hub_getStreamingPacketsInWaiting(prio_device_id hub_id, U32* in_waiting);
    /********************************************//**
    * \ingroup prio_hub_methods
    * \brief Returns whether or not the Hub has overloaded and wrapped the buffer.
    * \param[in] hub_id The prio_device_id that represents the PrioVR Hub.
    * \param[out] overflow The U8 representing whether of not the buffer has overloaded.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_hub_didStreamingOverflow(prio_device_id hub_id, U8* overflow);
    /********************************************//**
    * \ingroup prio_hub_methods
    * \brief Get the current update rate of the PrioVR Hub in miliseconds.
    * \param[in] hub_id The prio_device_id that represents the PrioVR Hub.
    * \param[out] update_rate A time in miliseconds of the current update rate, as an average over the last ten packets.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_hub_getStreamUpdateRate(prio_device_id hub_id, float* update_rate);

    //Recording functions
    /********************************************//**
    * \ingroup prio_hub_methods
    * \brief Enables wired recording with the given flags.
    * \param[in] hub_id The prio_device_id that represents the PrioVR Hub.
    * \param[in] data_flags The data flags reprsenting the data to stream. The flags are PRIO_STREAM_TARED_ORIENTATION_AS_QUATERNION, PRIO_STREAM_UNTARED_ORIENTATION_AS_QUATERNION
    * PRIO_STREAM_ALL_CORRECTED_COMPONENT_SENSOR_DATA, PRIO_STREAM_CORRECTED_GYRO_RATE, PRIO_STREAM_CORRECTED_ACCELEROMETER_VECTOR), PRIO_STREAM_CORRECTED_MAGNETOMETER_VECTOR,
    * PRIO_STREAM_ALL_RAW_COMPONENT_SENSOR_DATA, PRIO_STREAM_RAW_GYROSCOPE_RATE, PRIO_STREAM_RAW_ACCELEROMETER_DATA, PRIO_STREAM_RAW_MAGNETOMETER_DATA.
    * \param[in] interval The interval to stream data.
    * \param[in] duration The duration of the stream.
    * \param[in] delay A delay before streaming.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_hub_startRecordingWired(prio_device_id hub_id, prio_stream_commands data_flags, U32 interval, U32 duration, U32 delay = 0);
    /********************************************//**
    * \ingroup prio_hub_methods
    * \brief Disables wired recording.
    * \param[in] hub_id The prio_device_id that represents the PrioVR Hub.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_hub_stopRecordingWired(prio_device_id hub_id);
    /********************************************//**
    * \ingroup prio_hub_methods
    * \brief Disables wireless recording.
    * \param[in] hub_id The prio_device_id that represents the PrioVR Hub.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_hub_disableRecordingWireless(prio_device_id hub_id);
    /********************************************//**
    * \ingroup prio_hub_methods
    * \brief Setup recording 
    * \param[in] hub_id The prio_device_id that represents the PrioVR Hub.
    * \param[in] buffer_size The maxium number of recorded samples.
    * \param[in] is_wrapping Whether or not the buffer will wrap upon becoming full.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_hub_setupRecordingOptions(prio_device_id hub_id, U32 buffer_size, bool is_wrapping = false);
    /********************************************//**
    * \ingroup prio_hub_methods
    * \brief Get the maxium number of recorded samples.
    * \param[in] hub_id The prio_device_id that represents the PrioVR Hub.
    * \param[out] max_sample_count The maxium number of recorded samples.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_hub_getMaxRecordedSamples(prio_device_id hub_id, U32* max_sample_count);
    /********************************************//**
    * \ingroup prio_hub_methods
    * \brief Get whether or not the buffer will wrap upon becoming full.
    * \param[in] hub_id The prio_device_id that represents the PrioVR Hub.
    * \param[out] wrapping_enabled Whether or not the buffer will wrap upon becoming full.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_hub_getRecordingWrappingMode(prio_device_id hub_id, bool* wrapping_enabled);
    /********************************************//**
    * \ingroup prio_hub_methods
    * \brief Get the number of recorded packets.
    * \param[in] hub_id The prio_device_id that represents the PrioVR Hub.
    * \param[out] sample_count The number of recorded packets.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_hub_getLengthOfRecordedSamples(prio_device_id hub_id, U32* sample_count);
    /********************************************//**
    * \ingroup prio_hub_methods
    * \brief Get a number of recorded samples.
    * \param[in] hub_id The prio_device_id that represents the PrioVR Hub.
    * \param[out] header_data An array of the sample headers.
    * \param[out] packet_data An array of the sample sensor data.
    * \param[in] packet_count The number of samples to return;
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_hub_getRecordedSamples(prio_device_id hub_id, PrioHeader *header_data, U8* packet_data, U32 packet_count);
    /********************************************//**
    * \ingroup prio_hub_methods
    * \brief Get the recorded sample at the given index.
    * \param[in] hub_id The prio_device_id that represents the PrioVR Hub.
    * \param[out] header_data An array of the sample headers.
    * \param[out] packet_data An array of the sample sensor data.
    * \param[in] index The index of the sample to return;
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_hub_getRecordedSampleAtIndex(prio_device_id hub_id, PrioHeader *header_data, U8* packet_data, U32 index);
    /********************************************//**
    * \ingroup prio_hub_methods
    * \brief Get the recorded sample at the front of the buffer.
    * \param[in] hub_id The prio_device_id that represents the PrioVR Hub.
    * \param[out] header_data An array of the sample headers.
    * \param[out] packet_data An array of the sample sensor data.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_hub_popFrontRecordedSample(prio_device_id hub_id, PrioHeader *header_data, U8* packet_data);
    /********************************************//**
    * \ingroup prio_hub_methods
    * \brief Clear the recorded samples.
    * \param[in] hub_id The prio_device_id that represents the PrioVR Hub.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_hub_clearRecordedSamples(prio_device_id hub_id);

    //Call and response functions
    /********************************************//**
    * \ingroup prio_hub_methods
    * \brief Disables wireless streaming.
    * \param[in] hub_id The prio_device_id that represents the PrioVR Hub.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_hub_enumerateHub(prio_device_id hub_id);
    /********************************************//**
    * \ingroup prio_hub_methods
    * \brief Get the total number of enumerated sensors connected to the PrioVR Hub.
    * \param[in] hub_id The prio_device_id that represents the PrioVR Hub.
    * \param[out] num_sensors The number of enumerated sensors
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_hub_getNumEnumeratedSensors(prio_device_id hub_id, U8* num_sensors);
    /********************************************//**
    * \ingroup prio_hub_methods
    * \brief Retrieves a list of active sensor nodes at the logical IDs on the PrioVR device.
    * \param[in] hub_id The device ID for the PrioVR Hub.
    * \param[out] active_sensors A list of logical IDs to return the results.
    * \param[in] active_sensors_len The size of the list entered.
    * \param[out] active_len Writes the actual number of active sensors. Is optional.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_hub_getActiveSensors(prio_device_id hub_id, U8* active_sensors, U8 active_sensors_len, U8* active_len);
    /********************************************//**
    * \ingroup prio_hub_methods
    * \brief Retrieves the full length of the stream data.
    * \param[in] hub_id The device ID for the PrioVR Hub.
    * \param[out] stream_data_len The length of the stream data.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_hub_getFullLengthOfStreamingData(prio_device_id hub_id, U32* stream_data_len);
    /********************************************//**
    * \ingroup prio_hub_methods
    * \brief Sets the total number of times to retry a command before failing.
    * \param[in] hub_id The prio_device_id that represents the PrioVR Hub.
    * \param[in] retries The number of times to retry a command before failing.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_hub_setCommandRetries(prio_device_id hub_id, U8 retries);
    /********************************************//**
    * \ingroup prio_hub_methods
    * \brief Get the current wireless channel of the PrioVR Hub. In order for a Basestation to communicate with a Hub, they must both share the same channel.
    * \param[in] hub_id The prio_device_id that represents the PrioVR Hub.
    * \param[out] channel The wireless channel ID.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_hub_getWirelessChannel(prio_device_id hub_id, U8* channel);
    /********************************************//**
    * \ingroup prio_hub_methods
    * \brief Set the wireless channel of the PrioVR Hub. In order for a Basestation to communicate with a Hub, they must both share the same channel. Note that channel must be in the range 11-26 inclusive.
    * \param[in] hub_id The prio_device_id that represents the PrioVR Hub.
    * \param[in] channel The wireless channel ID.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_hub_setWirelessChannel(prio_device_id hub_id, U8 channel);
    /********************************************//**
    * \ingroup prio_hub_methods
    * \brief Return the current wireless panID of the PrioVR Hub. In order for a Basestation to communicate with a Hub, they must both share the same panID.
    * \param[in] hub_id The prio_device_id that represents the PrioVR Hub.
    * \param[out] pan_id The wireless pan ID.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_hub_getWirelessPanID(prio_device_id hub_id, U16* pan_id);
    /********************************************//**
    * \ingroup prio_hub_methods
    * \brief Set the wireless panID of the PrioVR Hub. In order for a PrioVR Basestation to communicate with a PrioVR Hub, they must both share the same panID. Note that panIDs must be in the range 1-65534 inclusive.
    * \param[in] hub_id The prio_device_id that represents the PrioVR Hub.
    * \param[in] pan_id The wireless pan ID.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_hub_setWirelessPanID(prio_device_id hub_id, U16 pan_id);
    /********************************************//**
    * \ingroup prio_basestation_methods
    * \brief Return the serial number of the PrioVR Device at the given logical id.
    * \param[in] hub_id The prio_device_id that represents the PrioVR Hub.
    * \param[in] logical_id The logical ID of the PrioVR Device to get the serial number of.
    * \param[out] serial_number The serial number.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_hub_getSerialNumberAtLogicalID(prio_device_id hub_id, U8 logical_id, U32* serial_number);
    /********************************************//**
    * \ingroup prio_basestation_methods
    * \brief Return the firmware version of the PrioVR Device at the given logical id.
    * \param[in] hub_id The prio_device_id that represents the PrioVR Hub.
    * \param[in] logical_id The logical ID of the PrioVR Device to get the firmware version of.
    * \param[out] firmware_version The firmware version.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_hub_getFirmwareVersionAtLogicalID(prio_device_id hub_id, U8 logical_id, const char* firmware_version);
    /********************************************//**
    * \ingroup prio_basestation_methods
    * \brief Return the hardware version of the PrioVR Device at the given logical id.
    * \param[in] hub_id The prio_device_id that represents the PrioVR Hub.
    * \param[in] logical_id The logical ID of the PrioVR Device to get the hardware version of.
    * \param[out] hardware_version The hardware version.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_hub_getHardwareVersionAtLogicalID(prio_device_id hub_id, U8 logical_id, const char* hardware_version);
    /********************************************//**
    * \ingroup prio_hub_methods
    * \brief Retrieves the battery level of the PrioVR Hub.
    * \param[in] hub_id The prio_device_id that represents the PrioVR Hub.
    * \param[out] battery_level The current battery level.
    * \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_hub_getHubBatteryLevel(prio_device_id hub_id, U8* battery_level);
    /********************************************//**
    * \ingroup prio_hub_methods
    * \brief Get the current state of the PrioVR Hub Buttons [0-Neither Pressed, 1-Left Pressed, 2-Right Pressed, 3-Both Pressed].
    * \param[in] hub_id The prio_device_id that represents the PrioVR Hub.
    * \param[out] button_state The PrioVR Hub button state.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_hub_getHubButtonState(prio_device_id hub_id, U8* button_state);
    /********************************************//**
    * \ingroup prio_hub_methods
    * \brief Commits all committable settings to non-volatile memory so that they will persist beyond a powercycle. For more information on Commitable Settings, please refer to the PrioVR User's Manual.
    * \param[in] hub_id The prio_device_id that represents the PrioVR Hub.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_hub_commitSettings(prio_device_id hub_id);
    /********************************************//**
    * \ingroup prio_hub_methods
    * \brief Set the streaming interval of the given PrioVR Hub.
    * \param[in] hub_id The prio_device_id that represents the PrioVR Hub.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_hub_setStreamingInterval(prio_device_id hub_id, U32 interval);
    /********************************************//**
    * \ingroup prio_hub_methods
    * \brief Get the streaming interval of the given PrioVR Hub.
    * \param[in] hub_id The prio_device_id that represents the PrioVR Hub.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_hub_getStreamingInterval(prio_device_id hub_id, U32* interval);
    /********************************************//**
    * \ingroup prio_hub_methods
    * \brief Resets all of the connected PrioVR Sensors of the given PrioVR Hub.
    * \param[in] hub_id The prio_device_id that represents the PrioVR Hub.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_hub_resetAllSensors(prio_device_id hub_id);
    /********************************************//**
    * \ingroup prio_hub_methods
    * \brief Power down all of the connected PrioVR Sensors of the given PrioVR Hub.
    * \param[in] hub_id The prio_device_id that represents the PrioVR Hub.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_hub_powerDownAllSensors(prio_device_id hub_id);
    /********************************************//**
    * \ingroup prio_hub_methods
    * \brief Power up all of the connected PrioVR Sensors of the given PrioVR Hub.
    * \param[in] hub_id The prio_device_id that represents the PrioVR Hub.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_hub_powerUpAllSensors(prio_device_id hub_id);
    /********************************************//**
    * \ingroup prio_hub_methods
    * \brief Set the Auto Enumeration of the given PrioVR Hub.
    * \param[in] hub_id The prio_device_id that represents the PrioVR Hub.
    * \param[in] enable Whether or not the PrioVR Hub is in auto enumeration mode.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_hub_setAutoEnumerationMode(prio_device_id hub_id, bool enable);
    /********************************************//**
    * \ingroup prio_hub_methods
    * \brief Get the Auto Enumeration of the given PrioVR Hub.
    * \param[in] hub_id The prio_device_id that represents the PrioVR Hub.
    * \param[out] enable Whether or not the PrioVR Hub is in auto enumeration mode.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_hub_getAutoEnumerationMode(prio_device_id hub_id, bool* enabled);
    /********************************************//**
    * \ingroup prio_hub_methods
    * \brief Get the current wireless of the given PrioVR Hub.
    * \param[in] hub_id The prio_device_id that represents the PrioVR Hub.
    * \param[out] address The wireless address of the PrioVR Hub.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_hub_getWirelessAddress(prio_device_id hub_id, U16* address);
    /********************************************//**
    * \ingroup prio_hub_methods
    * \brief Get the current battery status of the given PrioVR Hub. The possible battery status are: 0-BATTERY_STATUS_NORMAL, 1-BATTERY_STATUS_CHARGING, 2-BATTERY_STATUS_CHARGED, 3-BATTERY_STATUS_LOW.
    * \param[in] hub_id The prio_device_id that represents the PrioVR Hub.
    * \param[out] status The status of the PrioVR Hub battery.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_hub_getBatteryStatus(prio_device_id hub_id, U8* status);
    /********************************************//**
    * \ingroup prio_hub_methods
    * \brief Set the performance mode of the given PrioVR Hub. The modes are: False-PEFORMANCE_MODE_STANDARD, True-PERFORMANCE_MODE_POWER_SAVING.
    * \param[in] hub_id The prio_device_id that represents the PrioVR Hub.
    * \param[in] enable Whether or not the PrioVR Hub is in power saving mode.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_hub_setPerformanceMode(prio_device_id hub_id, bool enable);
    /********************************************//**
    * \ingroup prio_hub_methods
    * \brief Get the performance mode of the given PrioVR Hub. The modes are: False-PEFORMANCE_MODE_STANDARD, True-PERFORMANCE_MODE_POWER_SAVING.
    * \param[in] hub_id The prio_device_id that represents the PrioVR Hub.
    * \param[out] enable Whether or not the PrioVR Hub is in power saving mode.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_hub_getPerformanceMode(prio_device_id hub_id, bool* enabled);
    /********************************************//**
    * \ingroup prio_hub_methods
    * \brief Get the orientation of a sensor, it will be returned in a float array in the order (x,y,z,w).
    * \param[in] hub_id The prio_device_id that represents the PrioVR Hub.
    * \param[in] logical_id The logical ID of the sensor to get the orientation of.
    * \param[out] orient_quat The oreintation of the given sensor.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_hub_getTaredOrientationAsQuaternion(prio_device_id hub_id, U8 logical_id, U8 num_floats, float* orient_quat);
    /********************************************//**
    * \ingroup prio_hub_methods
    * \brief Get the untared orientation of a sensor, it will be returned in a float array in the order (x,y,z,w).
    * \param[in] hub_id The prio_device_id that represents the PrioVR Hub.
    * \param[in] logical_id The logical ID of the sensor to get the orientation of.
    * \param[in] num_floats The number of floats to be returned.
    * \param[out] quat The oreintation of the given sensor.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_hub_getUntaredOrientationAsQuaternion(prio_device_id hub_id, U8 logical_id, U8 num_floats, float* quat);
    /********************************************//**
    * \ingroup prio_hub_methods
    * \brief Get the controller data of the controller at the given index, in the order (X-axis,Y-axis,Trigger,Button state)
    * \param[in] hub_id The prio_device_id that represents the PrioVR Hub.
    * \param[in] index The index of the controller to get the controller data from. Note: There can only be two controllers plugged in and communicating.
    * \param[in] logical_id The logical id of the joystick's sensor.
    * \param[out] data The controller data of given controller.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_hub_getControllerData(prio_device_id hub_id, U8 index, U8* logical_id, U8* data);
    /********************************************//**
    * \ingroup prio_hub_methods
    * \brief Get all of the connected controller data for the attached controllers, it will be returned in two U8 arrays in the order (x-axis,y-axis,trigger,button state).
    * \param[in] hub_id The prio_device_id that represents the PrioVR Hub.
    * \param[out] data_1 The controller data of the first controller.
    * \param[out] data_2 The controller data of the second controller.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_hub_getAllControllerData(prio_device_id hub_id, U8* logical_id_1, U8* logical_id_2, U8* data_1, U8* data_2, U8* button_state);
    /********************************************//**
    * \ingroup prio_hub_methods
    * \brief Get the corrected sensor data, for each component the data will be returned in the order (x,y,z). 
    * \param[in] hub_id The prio_device_id that represents the PrioVR Hub.
    * \param[in] logical_id The logical ID of the sensor to get all the corrected data from.
    * \param[out] gyroscope The corrected gyroscope data.
    * \param[out] accelerometer The corrected accelerometer data.
    * \param[out] magnetometer The corrected magnetometer data.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_hub_getCorrectedSensorData(prio_device_id hub_id, U8 logical_id, float* gyroscope, float* accelerometer, float* magnetometer);
    /********************************************//**
    * \ingroup prio_hub_methods
    * \brief Get the corrected gryoscope of a sensor, it will be returned in the order (x,y,z).
    * \param[in] hub_id The prio_device_id that represents the PrioVR Hub.
    * \param[in] logical_id The logical ID of the sensor to get the corrected gryospcope data from.
    * \param[out] gyroscope The corrected gyroscope data.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_hub_getCorrectedGyroscope(prio_device_id hub_id, U8 logical_id, float* gyroscope);
    /********************************************//**
    * \ingroup prio_hub_methods
    * \brief Get the corrected accelerometer of a sensor, it will be returned in the order (x,y,z).
    * \param[in] hub_id The prio_device_id that represents the PrioVR Hub.
    * \param[in] logical_id The logical ID of the sensor to get the corrected accelerometer data from.
    * \param[out] accelerometer The corrected accelerometer data.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_hub_getCorrectedAccelerometer(prio_device_id hub_id, U8 logical_id, float* accelerometer);
    /********************************************//**
    * \ingroup prio_hub_methods
    * \brief Get the corrected magnetometer of a sensor, it will be returned in the order (x,y,z).
    * \param[in] hub_id The prio_device_id that represents the PrioVR Hub.
    * \param[in] logical_id The logical ID of the sensor to get the corrected magnetometer data from.
    * \param[out] magnetometer The corrected magnetometer data.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_hub_getCorrectedMagnetometer(prio_device_id hub_id, U8 logical_id, float* magnetometer);
    /********************************************//**
    * \ingroup prio_hub_methods
    * \brief Get the raw sensor data, for each component the data will be returned in the order (x,y,z).
    * \param[in] hub_id The prio_device_id that represents the PrioVR Hub.
    * \param[in] logical_id The logical ID of the sensor to get all the raw data from.
    * \param[out] gyroscope The raw gyroscope data.
    * \param[out] accelerometer The raw accelerometer data.
    * \param[out] magnetometer The raw magnetometer data.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_hub_getRawSensorData(prio_device_id hub_id, U8 logical_id, float* gyroscope, float* accelerometer, float* magnetometer);
    /********************************************//**
    * \ingroup prio_hub_methods
    * \brief Get the raw gryoscope of a sensor, it will be returned in the order (x,y,z).
    * \param[in] hub_id The prio_device_id that represents the PrioVR Hub.
    * \param[in] logical_id The logical ID of the sensor to get the raw gryoscope data from.
    * \param[out] gyroscope The raw gyroscope data.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_hub_getRawGyroscope(prio_device_id hub_id, U8 logical_id, float* gyroscope);
    /********************************************//**
    * \ingroup prio_hub_methods
    * \brief Get the raw gryoscope of a sensor, it will be returned in the order (x,y,z).
    * \param[in] hub_id The prio_device_id that represents the PrioVR Hub.
    * \param[in] logical_id The logical ID of the sensor to get the raw accelerometer data from.
    * \param[out] accelerometer The raw accelerometer data.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_hub_getRawAccelerometer(prio_device_id hub_id, U8 logical_id, float* accelerometer);
    /********************************************//**
    * \ingroup prio_hub_methods
    * \brief Get the raw gryoscope of a sensor, it will be returned in the order (x,y,z).
    * \param[in] hub_id The prio_device_id that represents the PrioVR Hub.
    * \param[in] logical_id The logical ID of the sensor to get the raw magnetometer data from.
    * \param[out] magnetometer The raw magnetometer data.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_hub_getRawMagnetometer(prio_device_id hub_id, U8 logical_id, float* magnetometer);
    /********************************************//**
    * \ingroup prio_hub_methods
    * \brief Set the magnetometer calibration paramaters of a sensor.
    * \param[in] hub_id The prio_device_id that represents the PrioVR Hub.
    * \param[in] logical_id The logical ID of the sensor to set the magnetometer calibration paramaters of.
    * \param[in] scale9 The 3x3 martix representing the scale.
    * \param[in] bias3 The bias of the calibration.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_hub_setMagnetometerCalibParams(prio_device_id hub_id, U8 logical_id, const float* scale9, const float* bias3);
    /********************************************//**
    * \ingroup prio_hub_methods
    * \brief Set the accelerometer calibration paramaters of a sensor.
    * \param[in] hub_id The prio_device_id that represents the PrioVR Hub.
    * \param[in] logical_id The logical ID of the sensor to set the accelerometer calibration paramaters of.
    * \param[in] scale9 The 3x3 martix representing the scale.
    * \param[in] bias3 The bias of the calibration.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_hub_setAccelerometerCalibParams(prio_device_id hub_id, U8 logical_id, const float* scale9, const float* bias3);
    /********************************************//**
    * \ingroup prio_hub_methods
    * \brief Set the gyroscope calibration paramaters of a sensor.
    * \param[in] hub_id The prio_device_id that represents the PrioVR Hub.
    * \param[in] logical_id The logical ID of the sensor to set the gyroscope calibration paramaters of.
    * \param[in] scale9 The 3x3 martix representing the scale.
    * \param[in] bias3 The bias of the calibration.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_hub_setGyroCalibParams(prio_device_id hub_id, U8 logical_id, const float* scale9, const float* bias3);
    /********************************************//**
    * \ingroup prio_hub_methods
    * \brief Get the magnetometer calibration paramaters of a sensor.
    * \param[in] hub_id The prio_device_id that represents the PrioVR Hub.
    * \param[in] logical_id The logical ID of the sensor to get the magnetometer calibration paramaters of a sensor from .
    * \param[out] scale9 The 3x3 martix representing the scale.
    * \param[out] bias3 The bias of the calibration.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_hub_getMagnetometerCalibParams(prio_device_id hub_id, U8 logical_id, float* scale9, float* bias3);
    /********************************************//**
    * \ingroup prio_hub_methods
    * \brief Get the accelerometer calibration paramaters of a sensor.
    * \param[in] hub_id The prio_device_id that represents the PrioVR Hub.
    * \param[in] logical_id The logical ID of the sensor to get the accelerometer calibration paramaters of a sensor from.
    * \param[out] scale9 The 3x3 martix representing the scale.
    * \param[out] bias3 The bias of the calibration.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_hub_getAccelerometerCalibParams(prio_device_id hub_id, U8 logical_id, float* scale9, float* bias3);
    /********************************************//**
    * \ingroup prio_hub_methods
    * \brief Get the gyroscope calibration paramaters of a sensor.
    * \param[in] hub_id The prio_device_id that represents the PrioVR Hub.
    * \param[in] logical_id The logical ID of the sensor to get the gyroscope calibration paramaters of a sensor from.
    * \param[out] scale9 The 3x3 martix representing the scale.
    * \param[out] bias3 The bias of the calibration.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_hub_getGyroCalibParams(prio_device_id hub_id, U8 logical_id, float* scale9, float* bias3);
    /********************************************//**
    * \ingroup prio_hub_methods
    * \brief Set the beta calibration paramaters of a sensor.
    * \param[in] hub_id The prio_device_id that represents the PrioVR Hub.
    * \param[in] logical_id The logical ID of the sensor to set the beta calibration paramaters of.
    * \param[in] settle_base
    * \param[in] settle_div
    * \param[in] base
    * \param[in] mul
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_hub_setBetaCalcParams(prio_device_id hub_id, U8 logical_id, float settle_base, float settle_div, float base, float mul);
    /********************************************//**
    * \ingroup prio_hub_methods
    * \brief Get the beta calibration paramaters of a sensor.
    * \param[in] hub_id The prio_device_id that represents the PrioVR Hub.
    * \param[in] logical_id The logical ID of the sensor to get the beta calibration paramaters from.
    * \param[out] settle_base
    * \param[out] settle_div
    * \param[out] base
    * \param[out] mul
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_hub_getBetaCalcParams(prio_device_id hub_id, U8 logical_id, float* settle_base, float* settle_div, float* base, float* mul);
    /********************************************//**
    * \ingroup prio_hub_methods
    * \brief Set whether or not the given sensor has auto calibration enabled.
    * \param[in] hub_id The prio_device_id that represents the PrioVR Hub.
    * \param[in] logical_id The logical id that represents the sensor.
    * \param[out] enabled The bool that represents the auto calibration state.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_hub_setAutoCalibEnabled(prio_device_id hub_id, U8 logical_id, U8 enabled);
    /********************************************//**
    * \ingroup prio_hub_methods
    * \brief Get whether or not the given sensor has auto calibration enabled.
    * \param[in] hub_id The prio_device_id that represents the PrioVR Hub.
    * \param[in] logical_id The logical id that represents the sensor.
    * \param[out] enabled The bool that represents the auto calibration state.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_hub_getAutoCalibEnabled(prio_device_id hub_id, U8 logical_id, U8* enabled);
    /********************************************//**
    * \ingroup prio_hub_methods
    * \brief Commits all committable settings to non-volatile memory so that they will persist beyond a powercycle for the given sensor. For more information on Commitable Settings, please refer to the PrioVR User's Manual.
    * \param[in] hub_id The prio_device_id that represents the PrioVR Hub.
    * \param[in] logical_id The logical id that represents the sensor.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_hub_commitSensorSettings(prio_device_id hub_id, U8 logical_id);
    /********************************************//**
    * \ingroup prio_hub_methods
    * \brief Tare the sensor with the current orientation.
    * \param[in] hub_id The prio_device_id that represents the PrioVR Hub.
    * \param[in] logical_id The logical id that represents the sensor.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_hub_tareWithCurrentOrientation(prio_device_id hub_id, U8 logical_id);

    ////BaseStation functions
    /********************************************//**
    * \ingroup prio_basestation_methods
    * This creates and performs initial setup on the connected PrioVR Basestation.
    * The returned PrioVR Basestation ID is used to interact with the PrioVR Basestation.
    * When a Hub ID is created, other programs cannot use that serial port until the port is closed.
    * \param[in] port_name The serial port string for the serial port to be queried.
    * \param[out] out_id A prio_device_id will be returned to represent the PrioVR Basestation.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_createBaseStation(const char* port_name, prio_device_id* out_id);
    /********************************************//**
    * \ingroup prio_basestation_methods
    * \brief This removes the connected PrioVR Basestation.
    * \param[in] bs_id The prio_device_id that represents the PrioVR Basestation.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_removeBaseStation(prio_device_id bs_id);
    //Standard device functions
    /********************************************//**
    * \ingroup prio_basestation_methods
    * \brief Checks to see if the specified PrioVR Basestation is connected.
    * \param[in] bs_id The prio_device_id that represents the PrioVR Basestation.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_bs_isConnected(prio_device_id bs_id);
    /********************************************//**
    * \ingroup prio_basestation_methods
    * \brief Retrieves the last timestamp logged by the System.
    * \param[in] bs_id The prio_device_id that represents the PrioVR Basestation.
    * \param[out] timestamp The returned timestamp in milliseconds.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_bs_getLastSystemTimestamp(prio_device_id bs_id, float* timestamp);
    /********************************************//**
    * \ingroup prio_basestation_methods
    * \brief Retrieves the last timestamp logged by the PrioVR Basestation.
    * \param[in] bs_id The prio_device_id that represents the PrioVR Basestation.
    * \param[out] timestamp The returned timestamp in milliseconds.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_bs_getLastBasestationTimestamp(prio_device_id bs_id, U32* timestamp);
    /********************************************//**
    * \ingroup prio_basestation_methods
    * \brief Retrieves the serial number of the given PrioVR Basestation.
    * \param[in] bs_id The prio_device_id that represents the PrioVR Basestation.
    * \param[out] serial_number The returned serial_number.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_bs_getSerialNumber(prio_device_id bs_id, U32* serial_number);
    /********************************************//**
    * \ingroup prio_basestation_methods
    * \brief Retrieves the strength of the wireless channels for the PrioVR Basestation.
    * \param[in] bs_id The prio_device_id that represents the PrioVR Basestation.
    * \param[out] channel_strengths16 The returned channel strengths as a 16 byte array.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_bs_getWirelessChannelStrengths(prio_device_id bs_id, U8* channel_strengths16);
    /********************************************//**
    * \ingroup prio_basestation_methods
    * \brief Retrieves the last signal strength seen by the PrioVR Basestation.
    * \param[in] bs_id The prio_device_id that represents the PrioVR Basestation.
    * \param[out] signal_strength The strenth of the signal.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_bs_getSignalStrength(prio_device_id bs_id, U8* signal_strength);

    //Port control functions
    /********************************************//**
    * \ingroup prio_basestation_methods
    * \brief Open a port to connect to PrioVR Basestation.
    * \param[in] bs_id The prio_device_id that represents the PrioVR Basestation.
    * \param[in] port_name The name of the communication port
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_bs_openPort(prio_device_id bs_id, const char* port_name);
    /********************************************//**
    * \ingroup prio_basestation_methods
    * \brief Close the connected port
    * \param[in] bs_id The prio_device_id that represents the PrioVR Basestation.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_bs_closePort(prio_device_id bs_id);

    //Header control functions
    /********************************************//**
    * \ingroup prio_basestation_methods
    * \brief Enables timestamps over wireless.
    * \param[in] bs_id The prio_device_id that represents the PrioVR Basestation.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_bs_enableTimestampsWireless(prio_device_id bs_id);
    /********************************************//**
    * \ingroup prio_basestation_methods
    * \brief Disables timestamps over wireless.
    * \param[in] bs_id The prio_device_id that represents the PrioVR Basestation.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_bs_disableTimestampsWireless(prio_device_id bs_id);

    //Wireless hub functions
    /********************************************//**
    * \ingroup prio_basestation_methods
    * \brief Get the paried PrioVR Hub.
    * \param[in] bs_id The prio_device_id that represents the PrioVR Basestation.
    * \param[in] out_id The prio_device_id that represents the connected PrioVR Hub.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_bs_getWirelessHub(prio_device_id bs_id, prio_device_id* out_id);
    /********************************************//**
    * \ingroup prio_basestation_methods
    * \brief Pair the PrioVR Basestation with a PrioVR Hub.
    * \param[in] bs_id The prio_device_id that represents the PrioVR Basestation.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_bs_autoPairWithHub(prio_device_id bs_id);

    //Streaming functions
    /********************************************//**
    * \ingroup prio_basestation_methods
    * \brief Start the streaming with the given flags.
    * \param[in] bs_id The prio_device_id that represents the PrioVR Basestation.
    * \param[in] data_flags The data flags reprsenting the data to stream. The flags are PRIO_STREAM_TARED_ORIENTATION_AS_QUATERNION, PRIO_STREAM_UNTARED_ORIENTATION_AS_QUATERNION
    * PRIO_STREAM_ALL_CORRECTED_COMPONENT_SENSOR_DATA, PRIO_STREAM_CORRECTED_GYRO_RATE, PRIO_STREAM_CORRECTED_ACCELEROMETER_VECTOR, PRIO_STREAM_CORRECTED_MAGNETOMETER_VECTOR, 
    * PRIO_STREAM_ALL_RAW_COMPONENT_SENSOR_DATA, PRIO_STREAM_RAW_GYROSCOPE_RATE, PRIO_STREAM_RAW_ACCELEROMETER_DATA, PRIO_STREAM_RAW_MAGNETOMETER_DATA.
    * \param[in] interval The interval to stream data.
    * \param[in] duration The duration of the stream.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_bs_startStreaming(prio_device_id bs_id, U32 data_flags, U32 interval, U32 duration);
    /********************************************//**
    * \ingroup prio_basestation_methods
    * \brief Stops the streaming
    * \param[in] bs_id The prio_device_id that represents the PrioVR Basestation.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_bs_stopStreaming(prio_device_id bs_id);

    //Recording functions
    /********************************************//**
    * \ingroup prio_basestation_methods
    * \brief Start the recording with the given flags.
    * \param[in] bs_id The prio_device_id that represents the PrioVR Basestation.
    * \param[in] data_flags The data flags reprsenting the data to record. The flags are PRIO_STREAM_TARED_ORIENTATION_AS_QUATERNION, PRIO_STREAM_UNTARED_ORIENTATION_AS_QUATERNION
    * PRIO_STREAM_ALL_CORRECTED_COMPONENT_SENSOR_DATA, PRIO_STREAM_CORRECTED_GYRO_RATE, PRIO_STREAM_CORRECTED_ACCELEROMETER_VECTOR, PRIO_STREAM_CORRECTED_MAGNETOMETER_VECTOR,
    * PRIO_STREAM_ALL_RAW_COMPONENT_SENSOR_DATA, PRIO_STREAM_RAW_GYROSCOPE_RATE, PRIO_STREAM_RAW_ACCELEROMETER_DATA, PRIO_STREAM_RAW_MAGNETOMETER_DATA.
    * \param[in] interval The interval to record data.
    * \param[in] duration The duration of the recording.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_bs_startRecording(prio_device_id bs_id, U32 data_flags, U32 interval, U32 duration);
    /********************************************//**
    * \ingroup prio_basestation_methods
    * \brief Stops the recording
    * \param[in] bs_id The prio_device_id that represents the PrioVR Basestation.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_bs_stopRecording(prio_device_id bs_id);

    //Call and response functions
    /********************************************//**
    * \ingroup prio_basestation_methods
    * \brief Return the current wireless channel of the  PrioVR Basestation. In order for a PrioVR Basestation to communicate with a PrioVR Hub, they must both share the same channel.
    * \param[in] bs_id The prio_device_id that represents the PrioVR Basestation.
    * \param[out] channel The wireless channel ID.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_bs_getWirelessChannel(prio_device_id bs_id, U8* channel);
    /********************************************//**
    * \ingroup prio_basestation_methods
    * \brief Set the wireless channel of the PrioVR Basestation. In order for a PrioVR Basestation to communicate with a PrioVR Hub, they must both share the same channel. Note that channel must be in the range 11 � 26 inclusive.
    * \param[in] bs_id The prio_device_id that represents the PrioVR Basestation.
    * \param[in] channel The wireless channel ID.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_bs_setWirelessChannel(prio_device_id bs_id, U8 channel);
    /********************************************//**
    * \ingroup prio_basestation_methods
    * \brief Return the current wireless panID of the PrioVR Basestation. In order for a PrioVR Basestation to communicate with a PrioVR Hub, they must both share the same panID.
    * \param[in] bs_id The prio_device_id that represents the PrioVR Basestation.
    * \param[out] pan_id The wireless pan ID.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_bs_getWirelessPanID(prio_device_id bs_id, U16* pan_id);
    /********************************************//**
    * \ingroup prio_basestation_methods
    * \brief Set the wireless panID of the PrioVR Basestation. In order for a PrioVR Basestation to communicate with a PrioVR Hub, they must both share the same panID. Note that panIDs must be in the range 1 � 65534 inclusive.
    * \param[in] bs_id The prio_device_id that represents the PrioVR Basestation.
    * \param[in] pan_id The wireless pan ID.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_bs_setWirelessPanID(prio_device_id bs_id, U16 pan_id);
    /********************************************//**
    * \ingroup prio_basestation_methods
    * \brief Return the serial number of the PrioVR Device.
    * \param[in] bs_id The prio_device_id that represents the PrioVR Basestation.
    * \param[in] logical_id The logical ID of the PrioVR Device to get the firmware version of.
    * \param[out] serial_number The serial number.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_bs_getSerialNumberAtLogicalID(prio_device_id bs_id, U8 logical_id, U32* serial_number);
    /********************************************//**
    * \ingroup prio_basestation_methods
    * \brief Return the firmware version of the PrioVR Device at the given logical id.
    * \param[in] bs_id The prio_device_id that represents the PrioVR Basestation.
    * \param[in] logical_id The logical ID of the PrioVR Device to get the firmware version of.
    * \param[out] firmware_version The firmware version.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_bs_getFirmwareVersionAtLogicalID(prio_device_id bs_id, U8 logical_id, const char* firmware_version);
    /********************************************//**
    * \ingroup prio_basestation_methods
    * \brief Return the hardware version of the PrioVR Device at the given logical id.
    * \param[in] bs_id The prio_device_id that represents the PrioVR Basestation.
    * \param[in] logical_id The logical ID of the PrioVR Device to get the hardware version of.
    * \param[out] hardware_version The hardware version.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_bs_getHardwareVersionAtLogicalID(prio_device_id bs_id, U8 logical_id, const char* hardware_version);
    /********************************************//**
    * \ingroup prio_basestation_methods
    * \brief Commits all committable settings to non-volatile memory so that they will persist beyond a powercycle. For more information on Commitable Settings, please refer to the PrioVR User's Manual.
    * \param[in] bs_id The prio_device_id that represents the PrioVR Basestation.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_bs_commitSettings(prio_device_id bs_id);
    /********************************************//**
    * \ingroup prio_basestation_methods
    * \brief Return the hardware version of the PrioVR Basestation.
    * \param[in] bs_id The prio_device_id that represents the PrioVR Basestation.
    * \param[out] hw_version The hardware version.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_bs_getHardwareVersion(prio_device_id bs_id, const char* hardware_version);
    /********************************************//**
    * \ingroup prio_basestation_methods
    * \brief Return the firmware version of the PrioVR Basestation.
    * \param[in] bs_id The prio_device_id that represents the PrioVR Basestation.
    * \param[out] fw_version The firmware version.
    * \return PRIO_NO_ERROR(0) if successful, non zero if an error occurred.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_bs_getFirmwareVersion(prio_device_id bs_id, const char* firmware_version);

    //Port functions
    /********************************************//**
    * \ingroup prio_port_methods
    * \brief Find PrioVR Devices connected to the computer.
    * \param[in] find_flags The flags for what devices to look for. 1-PRIO_BS, 2-PRIO_HUB, 3- PRIO_FIND_ALL_KNOWN, 4- PRIO_FIND_UNKNOWN, 15-PRIO_FIND_ALL.
    ***********************************************/
    PRIO_EXPORT void prio_findPorts(U32 find_flags);
    //port_name must be prio_PORT_NAME_SIZE characters long
    /********************************************//**
    * \ingroup prio_port_methods
    * \brief Return the COM Port, and attached PrioVR Device type on the next unused COM Port.
    * \param[out] port_name The COM Port.
    * \param[out] sensor_type The type of PrioVR Device.
    * \return U32 if successful 1, if error 0.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_getNextPort(char* port_name, U8* sensor_type);
    /********************************************//**
    * \ingroup prio_port_methods
    * \brief Return the COM Port, and attached PrioVR Device type on the COM Port at the given index.
    * \param[out] port_name The COM Port.
    * \param[in] index The index of the COM Port.
    * \param[out] sensor_type The type of PrioVR Device.
    * \return U32 if successful 1, if error 0.
    ***********************************************/
    PRIO_EXPORT PRIO_ERROR prio_getPort(char* port_name, U32 index, U8* sensor_type);
}