/********************************************//**
 * Copyright 1998-2014, YOST Labs Corporation
 * This Source Code Form is subject to the terms of the YOST 3-Space Open
 * Source License available online at:
 * http://www.yeitechnology.com/yei-3-space-open-source-license
 ***********************************************/
#ifndef _YOST_CONNECTION_PROCESSOR_H_
#define _YOST_CONNECTION_PROCESSOR_H_
class PrioAPI;
#include "yost_skeleton_processor.hpp"
#include "prio_api_export.h"
#include "prio_api.hpp"


namespace yost
{
    // Base class for the connection device of the Skeletal API
    class YostConnection : public SkeletonProcessor
    {
    public:
        YostConnection();
        virtual ~YostConnection();

        virtual bool isConnected();
        virtual U32 getDeviceId();
        virtual void resetSkeletonMap();

        virtual void runProcess() = 0;

        // Device functions
        void reconnectDevice();
        virtual void disconnectDevice() = 0;

        // Command functions
        std::unique_ptr<float[]> getLedColor();
        void setLedColor(const float* rgb_color3);

        // Streaming functions
        void setupStreamingData();
        //void setupStreamingDataWithAcceleration();
        virtual void startStreaming() = 0;
        virtual void stopStreaming() = 0;

        // Skeleton calls
        virtual bool calibrate(float wait_time=0.0f) = 0;
        virtual void setupSkeletonMap() = 0;

    protected:
        bool _is_connected;
        U32 _device;
        bool _is_mapped;
        bool _is_calibrated;
        bool _is_streaming;
        bool _is_recording;
        U32 _stream_slots[8];
        std::unique_ptr<float[]> _stream_data;
        U32 _stream_data_size;
        U32 _sensor_stream_data_size;
        U32 _stream_timestamp;
        U32 _max_recording_size;
        std::map<U32, U32> _skeleton_id_order;
        std::map<U32, std::string> _skeleton_id_map;
        std::map<std::string, U32> _reverse_skeleton_id_map;
    };

    // Class for Prio device connections
    class PrioConnection : public YostConnection
    {
    public:
        PrioConnection(U8 device_type, U8 index = 0);
        PrioConnection(prio_device_id device_id, PRIO_TYPE device_type);
        ~PrioConnection();

        void runProcess();

        // Device functions
        void disconnectDevice();
        PRIO_TYPE getDeviceType();

        // Command functions
        std::unique_ptr<float[]> getLedColor();
        void getHubBatteryLevel(U8* battery_level);
        void getHubBatteryStatus(U8* battery_status);
        void getHubButtonState(U8* button);
        void getHubButton(U8 buttonIndex, U8* button);
        void getJoystickState(PrioJoystick *left_joystick, PrioJoystick* right_joystick);
        void getJoystickButtonState(U8 joystick_index, U8* button_state, bool* is_valid);
        void getJoystickAxis(U8 axis_index, U8* axis_stat, bool* is_valide);
        void getActiveJoystickByIndex(U8 joystick_index, bool* is_active);
        void getActiveJoysticks(U8* active_joysticks);

        // Streaming functions
        void setupStreamingData();
        void setupStreamingDataWithAcceleration();
        void startStreaming();
        void stopStreaming();
        prio_StreamHeaderData getStreamingHeader();

        void clearRecordedSamples();

        // Skeleton calls
        bool calibrate(float wait_time=0.0f);
        void setBoneUpdateMode(bool fused);
        bool getBoneUpdateMode();
        void setupSkeletonMap();

        //Testing things
        bool isValidQuat(Orient input);
        std::shared_ptr<Bone> findValidParent(std::shared_ptr<Bone> bone);

        // Controller Data
        // Set the Controller Update Rate when not streaming
        void setControllerUpdateRate(U32 update_rate);
        // Get the Joystick Update Rate when not streaming
        void getControllerUpdateRate(U32* update_rate);

        void startControllerUpdating();
        void stopControllerUpdating();

        void activeSensorsHaveChanged(bool* changed);

    private:
        PRIO_TYPE _device_type;
        U32 _hub_device_ID;
        std::unique_ptr<prio_StreamHeaderData> _header_data;
        U8 _active_sensors_len;
        std::unique_ptr<U8[]> _active_sensors;
        bool _active_sensors_have_changed;

        PrioJoystick _joystick1_state;
        PrioJoystick _joystick2_state;
        U8 _hub_button_state;

        bool _is_joystick_polling;
        bool _bone_fused_mode;
        U8 _controller_thread_index; // Will be phased out
        bool _breakControllerThread;
        U32 _controller_update_rate; // How long we sleep before polling the joystick data
        std::thread _controllerThread;

        void _prioControllerThread();
        void _updateControllerData(U8* joystick1, U8* joystick2, U8* hub_button);
    };
};

#endif //_YOST_CONNECTION_PROCESSOR_H_
