/********************************************//**
 * Copyright 1998-2014, YEI Corporation
 * This Source Code Form is subject to the terms of the YEI 3-Space Open
 * Source License available online at:
 * http://www.yeitechnology.com/yei-3-space-open-source-license
 ***********************************************/
#include "yost_connection_processor.hpp"
#include "yost_skeleton_core_api.hpp"
#include <iostream>
#include <fstream>

namespace yost
{
    YostConnection::YostConnection()
    {
        _is_connected = false;
        _is_mapped = false;
        _is_streaming = false;
        _is_recording = false;
        _stream_data_size = 0;
    }

    YostConnection::~YostConnection()
    {
    }

    bool YostConnection::isConnected()
    {
        return _is_connected;
    }

    U32 YostConnection::getDeviceId()
    {
        return _device;
    }

    void YostConnection::resetSkeletonMap()
    {
        _skeleton_id_map.clear();
        _is_mapped = false;
    }

    void PrioConnection::_prioControllerThread()
    {
        bool swapStreamJoystick = false;
        U8 joystick_1_data[4] = { 128, 128, 0, 0 };
        U8 joystick_2_data[4] = { 128, 128, 0, 0 };
        U8 joystick_1_id = 255;
        U8 joystick_2_id = 255;

        U8 hub_button_state = 0;
        clock_t time;
        U32 dTime;
        U32 remaining_wait_time;
        while (1)
        {
            if (_breakControllerThread)
                break;

            time = clock();

            PRIO_ERROR error = prio_hub_getAllControllerData(_hub_device_ID, &joystick_1_id, &joystick_2_id, joystick_1_data, joystick_2_data, &hub_button_state);

            if ((6 < joystick_1_id && joystick_1_id < 14) || (20 < joystick_1_id && joystick_1_id < 28) || (34 < joystick_1_id && joystick_1_id < 42))
            {
                swapStreamJoystick = false;
            }
            else if ((6 < joystick_2_id && joystick_2_id < 14) || (20 < joystick_2_id && joystick_2_id < 28) || (34 < joystick_2_id && joystick_2_id < 42))
            {
                swapStreamJoystick = true;
            }
            else if ((0 < joystick_1_id && joystick_1_id < 7) || (13 < joystick_1_id && joystick_1_id < 19) || (20 < joystick_1_id && joystick_1_id < 28))
            {
                swapStreamJoystick = true;
            }
            else
            {
                swapStreamJoystick = false;
            }

            
            if (!swapStreamJoystick)
            {
                _updateControllerData(joystick_1_data, joystick_2_data, &hub_button_state);
            }
            else
            {
                _updateControllerData(joystick_2_data, joystick_1_data, &hub_button_state);
            }

            dTime = (clock() - time) / (CLOCKS_PER_SEC / 1000);
            remaining_wait_time = _controller_update_rate - dTime;

            if ((int)remaining_wait_time > 0)
            {
                yost::sleep_ms(_controller_update_rate - dTime);
            }
        }
    }

    #pragma optimize( "", off )
    PrioConnection::PrioConnection(U8 device_type, U8 index)
    {
        _type = PROCESSOR_TYPE_PRIO;
        _device = PRIO_NO_DEVICE_ID;
        _active_sensors_len = 0;
        _controller_update_rate = 25;
        _is_joystick_polling = false;
        _bone_fused_mode = false;
        prio_error = PRIO_NO_ERROR;
        prio_ComPort port;

        port.port_name = new char[64];
        port.device_type = (U8) 0;

        if (device_type == PRIO_TYPE::PRIO_ALL)
        {
            prio_findPorts(PRIO_ALL);
            
            if (prio_getPort(port.port_name, index, &port.device_type) == PRIO_NO_ERROR)
            {
                if (port.device_type == PRIO_BS)
                {
                    prio_error = prio_createBaseStation(port.port_name, &_device);

                    if (prio_error)
                    {
                        return;
                    }

                    prio_error = prio_bs_getWirelessHub(_device, &_hub_device_ID);

                    if (prio_error)
                    {
                        return;
                    }

                    _is_connected = true;
                    _device_type = PRIO_TYPE::PRIO_BS;
                }
                else if (port.device_type == PRIO_HUB)
                {
                    prio_findPorts(PRIO_HUB);

                    if (prio_getPort(port.port_name, index, &port.device_type) == PRIO_NO_ERROR)
                    {
                        prio_error = prio_createHub(port.port_name, &_hub_device_ID);

                        if (prio_error)
                        {
                            return;
                        }
                        _is_connected = true;
                        _device_type = PRIO_TYPE::PRIO_HUB;
                    }
                    else
                    {
                        return;
                    }
                }
            }
        }
        else if (device_type == PRIO_TYPE::PRIO_BS)
        {
            prio_findPorts(PRIO_BS);

            if (prio_getPort(port.port_name, index, &port.device_type) == PRIO_NO_ERROR)
            {
                prio_error = prio_createBaseStation(port.port_name, &_device);

                if (prio_error)
                {
                    return;
                }

                prio_error = prio_bs_getWirelessHub(_device, &_hub_device_ID);

                if (prio_error)
                {
                    return;
                }

                _is_connected = true;
                _device_type = PRIO_TYPE::PRIO_BS;
            }
            else
            {
                return;
            }
        }
        else
        {
            prio_findPorts(PRIO_HUB);

            if (prio_getPort(port.port_name, index, &port.device_type) == PRIO_NO_ERROR)
            {
                prio_error = prio_createHub(port.port_name, &_hub_device_ID);

                if (prio_error)
                {
                    return;
                }
                _is_connected = true;
                _device_type = PRIO_TYPE::PRIO_HUB;
            }
            else
            {
                return;
            }
        }

        prio_error = prio_hub_enumerateHub(_hub_device_ID);

        if (prio_error)
        {
            _is_connected = false;
        }

        prio_hub_getActiveSensors(_hub_device_ID, nullptr, 0, &_active_sensors_len);

        _active_sensors.reset(new U8[_active_sensors_len]);

        prio_hub_getActiveSensors(_hub_device_ID, _active_sensors.get(), _active_sensors_len, nullptr);

        prio_hub_setAutoEnumerationMode(_hub_device_ID, true);

        if (_active_sensors_len <= 0)
        {
            _is_connected = false;
        }
        else
        {
            _is_connected = true;
        }
    }

    PrioConnection::PrioConnection(prio_device_id device_id, PRIO_TYPE device_type)
    {
        _type = PROCESSOR_TYPE_PRIO;
        _device = PRIO_NO_DEVICE_ID;
        _active_sensors_len = 0;
        _controller_update_rate = 25;
        _is_joystick_polling = false;
        _breakControllerThread = false;
        _bone_fused_mode = false;
        prio_error = PRIO_NO_ERROR;
        _device_type = device_type;

        if (device_type == PRIO_TYPE::PRIO_BS)
        {
            _device = device_id;
            prio_error = prio_bs_getWirelessHub(_device, &_hub_device_ID);

            if (prio_error)
            {
                return;
            }
        }
        else
        {
            _hub_device_ID = device_id;
        }

        _is_connected = true;

        prio_error = prio_hub_enumerateHub(_hub_device_ID);

        if (prio_error)
        {
            _is_connected = false;
        }

        prio_hub_getActiveSensors(_hub_device_ID, nullptr, 0, &_active_sensors_len);

        _active_sensors.reset(new U8[_active_sensors_len]);

        prio_hub_getActiveSensors(_hub_device_ID, _active_sensors.get(), _active_sensors_len, nullptr);

        prio_hub_setAutoEnumerationMode(_hub_device_ID, true);

        if (_active_sensors_len <= 0)
        {
            _is_connected = false;
        }
        else
        {
            _is_connected = true;
        }
    }

    PrioConnection::~PrioConnection()
    {
        disconnectDevice();
        if (_is_connected)
        {
            PRIO_ERROR error = prio_removeHub(_hub_device_ID);
            error = prio_removeBaseStation(_device);
        }
    }

    void PrioConnection::runProcess()
    {
        if (!_is_connected || !_is_mapped || !_is_streaming)
        {
            return;
        }

        prio_StreamHeaderData *tmp = new prio_StreamHeaderData;
        prio_error = prio_hub_getLastStreamingPacket(_hub_device_ID, tmp, (U8*)_stream_data.get());

        if (prio_error == PRIO_NO_ERROR)
        {
            // Check the suit active sensors againist our old result if differnet report it and try to update like normal
            // Grab the old active sensors for comprasion 
            std::unique_ptr<U8[]> old_active_sensors(new U8[_active_sensors_len]);
            U8 old_active_sensors_len = _active_sensors_len;

            memcpy((U8*)old_active_sensors.get(), (U8*)_active_sensors.get(), _active_sensors_len);

            // Gather the most recent packets active sensors
            prio_hub_getActiveSensors(_hub_device_ID, nullptr, 0, &_active_sensors_len);
            _active_sensors.reset(new U8[_active_sensors_len]);
            prio_hub_getActiveSensors(_hub_device_ID, _active_sensors.get(), _active_sensors_len, nullptr);

            // Sort the arrays so we can compare them
            sort((U8*)_active_sensors.get(), (U8*)_active_sensors.get() + _active_sensors_len);
            sort((U8*)old_active_sensors.get(), (U8*)old_active_sensors.get() + old_active_sensors_len);

            // Compare the arrays
            if (_active_sensors_len == old_active_sensors_len)
            {
                for (int i = 0; i < _active_sensors_len; i++)
                {
                    if (old_active_sensors.get()[i] != _active_sensors.get()[i])
                    {
                        _active_sensors_have_changed = true;
                        break;
                    }
                }
            }
            else
            {
                _active_sensors_have_changed = true;
            }

            // Update
            _header_data = std::unique_ptr<prio_StreamHeaderData>(tmp);
            Vector3 new_vec;
            Orient new_orient, tmp_orient, finalOrient;
            uint8_t i, j;
            uint32_t stream_idx = 0;
            std::map<std::string, Orient> pose = _skeleton->getSkeletonPose();

            std::vector<std::string> boneNames = _skeleton->getBoneNameList();
            std::shared_ptr<Bone> bone;

            for (std::map<U32, std::string>::iterator iter = _skeleton_id_map.begin(), iter_end = _skeleton_id_map.end(); iter != iter_end; ++iter)
            {
                bone = _skeleton->getBone(iter->second);
                bool found = false;
                
                for (i = 0; i < _active_sensors_len; i++)
                {
                    if (_active_sensors[i] == iter->first)
                    {
                        stream_idx = i * _sensor_stream_data_size;
                        found = true;
                    }
                }

                if (found)
                {
                    if (bone == nullptr)
                    {
                        continue;
                    }
                    for (j = 0; j < 8; j++)
                    {
                        switch (_stream_slots[j])
                        {
                        case PRIO_NULL:
                            break;
                        case PRIO_STREAM_ALL_CORRECTED_COMPONENT_SENSOR_DATA:
                            stream_idx += 9;
                            break;
                        case PRIO_STREAM_ALL_RAW_COMPONENT_SENSOR_DATA:
                            stream_idx += 9;
                            break;
                        case PRIO_STREAM_CORRECTED_GYRO_RATE:
                            stream_idx += 3;
                            break;
                        case PRIO_STREAM_CORRECTED_MAGNETOMETER_VECTOR:
                            stream_idx += 3;
                            break;
                        case PRIO_STREAM_RAW_GYROSCOPE_RATE:
                            stream_idx += 3;
                            break;
                        case PRIO_STREAM_RAW_MAGNETOMETER_DATA:
                            stream_idx += 3;
                            break;
                        case PRIO_STREAM_CORRECTED_ACCELEROMETER_VECTOR:
                            stream_idx += 3;
                            break;
                        case PRIO_STREAM_RAW_ACCELEROMETER_DATA:
                            memcpy(new_vec.data, _stream_data.get() + stream_idx, 12);
                            bone->setAcceleration(new_vec);
                            stream_idx += 3;
                            break;
                        case PRIO_STREAM_TARED_ORIENTATION_AS_QUATERNION:
                            memcpy(new_orient.data, _stream_data.get() + stream_idx, 16);
                            stream_idx += 4;
                            break;
                        case PRIO_STREAM_UNTARED_ORIENTATION_AS_QUATERNION:
                            memcpy(new_orient.data, _stream_data.get() + stream_idx, 16);
                            stream_idx += 4;
                            break;
                        }
                    }
                    Orient calibrate_offset = bone->getCalibrationOffset();
                    orientMul(&new_orient, &calibrate_offset, &tmp_orient);
                    Orient calibrate_tare = bone->getCalibrationTare();
                    orientMul(&calibrate_tare, &tmp_orient, &new_orient);

                    Orient poseOffset = pose[bone->getName()];
                    orientInverse(&poseOffset);

                    orientMul(&new_orient, &poseOffset, &finalOrient);

                    bone->setRawOrientation(finalOrient);
                }
            }

            _skeleton->getRootBone()->update();
        }
    }

    //Add some function to find a parent that is in the active sensor
    std::shared_ptr<Bone> PrioConnection::findValidParent(std::shared_ptr<Bone> bone)
    {
        std::shared_ptr<Bone> parent;
        parent = bone->getParent();
        if (parent == nullptr)
        {
            return nullptr;
        }

        U32 parent_id = parent->getDeviceId();
        U32 parent_offset = 0;
        bool found = false;
        for (int k = 0; k < _active_sensors_len; k++)
        {
            if (_active_sensors[k] == parent_id)
            {
                parent_offset = k * _sensor_stream_data_size;
                found = true;
                break;
            }
        }

        if (found)
        {
            return parent;
        }
        return findValidParent(parent);
    }

    bool PrioConnection::isValidQuat(Orient input)
    {
        float value = input.data[0] * input.data[0] + input.data[1] * input.data[1] + input.data[2] * input.data[2] + input.data[3] * input.data[3];
        if (value > .9 && value < 1.1)
        {
            return true;
        }
        return false;
    }

    PRIO_TYPE PrioConnection::getDeviceType()
    {
        return _device_type;
    }

    void PrioConnection::getHubBatteryLevel(U8* battery_level)
    {
        if (_is_streaming)
        {
            *battery_level = getStreamingHeader().battery_level;
            return;
        }
        else
        {
            prio_hub_getHubBatteryLevel(_hub_device_ID, battery_level);
            return;
        }
    }

    void PrioConnection::getHubBatteryStatus(U8* battery_status)
    {
        if (_is_streaming)
        {
            *battery_status = getStreamingHeader().battery_status;
            return;
        }
        else
        {
            prio_hub_getBatteryStatus(_hub_device_ID, battery_status);
            return;
        }
    }

    void PrioConnection::getHubButtonState(U8* button)
    {
        if (_is_streaming)
        {
            *button = getStreamingHeader().hub_button;
        }
        else
        {
            *button = _hub_button_state;
        }
    }
    
    void PrioConnection::getHubButton(U8 buttonIndex, U8* button)
    {
        uint8_t buttonState = 0;
        *button = 0;
        if (_is_streaming)
        {
            buttonState = getStreamingHeader().hub_button;
        }
        else
        {
            buttonState = _hub_button_state;
        }

        if ((buttonState & (1 << ((buttonIndex + 1) - 1))) != 0)
        {
            *button = 1;
        }
    }

    void PrioConnection::getJoystickState(PrioJoystick *left_joystick, PrioJoystick *right_joystick)
    {
        if (_is_streaming)
        {
            *left_joystick = getStreamingHeader().joystick1;
            *right_joystick = getStreamingHeader().joystick2;
        }
        else
        {
            *left_joystick = _joystick1_state;
            *right_joystick = _joystick2_state;
        }

        // Joystick Dead Zones
        if (left_joystick->x_Axis < 150 && left_joystick->x_Axis > 106)
        {
            left_joystick->x_Axis = 128;
        }

        if (left_joystick->y_Axis < 150 && left_joystick->y_Axis > 106)
        {
            left_joystick->y_Axis = 128;
        }

        if (right_joystick->x_Axis < 150 && right_joystick->x_Axis > 106)
        {
            right_joystick->x_Axis = 128;
        }

        if (right_joystick->y_Axis < 150 && right_joystick->y_Axis > 106)
        {
            right_joystick->y_Axis = 128;
        }
    }

    void PrioConnection::getJoystickButtonState(U8 button_index, U8* button_state, bool* is_valid)
    {
        *is_valid = true;
        PrioJoystick joystick1;
        PrioJoystick joystick2;
        getJoystickState(&joystick1, &joystick2);

        if (button_index == 6)
        {
            *button_state = joystick1.Trigger;
            if (joystick1.ButtonState == 0)
            {
                *is_valid = false;
            }
            return;
        }
        
        if (button_index == 13)
        {
            *button_state = joystick2.Trigger;
            if (joystick2.ButtonState == 0)
            {
                *is_valid = false;
            }
            return;
        }


        uint8_t button;
        if (button_index < 6)
        {
            button = joystick1.ButtonState;
            if (joystick1.ButtonState == 0)
            {
                *is_valid = false;
            }
            return;
        }
        else
        {
            button = joystick2.ButtonState;
            if (joystick2.ButtonState == 0)
            {
                *is_valid = false;
            }
            return;
            button_index -= 7;
        }

        switch (button_index)
        {
            case 0:
                if ((button & 1))
                {
                    *button_state = 1;
                    return;
                }
                else
                {
                    *button_state = 0;
                    return;
                }
                break;
            case 1:
                if ((button & 2))
                {
                    *button_state = 1;
                    return;
                }
                else
                {
                    *button_state = 0;
                    return;
                }
                break;
            case 2:
                if ((button & 4))
                {
                    *button_state = 1;
                    return;
                }
                else
                {
                    *button_state = 0;
                    return;
                }
                break;
            case 3:
                    if ((button & 8))
                    {
                        *button_state = 1;
                        return;
                    }
                    else
                    {
                        *button_state = 0;
                        return;
                    }
                    break;
            case 4:
                if ((button & 16))
                {
                    *button_state = 1;
                    return;
                }
                else
                {
                    *button_state = 0;
                    return;
                }
                break;
            case 5:
                if ((button & 32))
                {
                    *button_state = 1;
                    return;
                }
                else
                {
                    *button_state = 0;
                    return;
                }
                break;
        }
    }

    void PrioConnection::getJoystickAxis(U8 axis_index, U8* axis_state, bool* is_valid)
    {
        *is_valid = true;
        PrioJoystick joystick1;
        PrioJoystick joystick2;
        getJoystickState(&joystick1, &joystick2);

        switch (axis_index)
        {
            case 0:
                *axis_state = joystick1.x_Axis;
                if (joystick1.ButtonState == 0)
                {
                    *is_valid = false;
                }
                break;
            case 1:
                *axis_state = joystick1.y_Axis;
                if (joystick1.ButtonState == 0)
                {
                    *is_valid = false;
                }
                break;
            case 2:
                *axis_state = joystick2.x_Axis;
                if (joystick2.ButtonState == 0)
                {
                    *is_valid = false;
                }
                break;
            case 3:
                *axis_state = joystick2.y_Axis;
                if (joystick2.ButtonState == 0)
                {
                    *is_valid = false;
                }
                break;
        }
    }

    void PrioConnection::getActiveJoystickByIndex(U8 joystick_index, bool* is_active)
    {
        PrioJoystick joystick1;
        PrioJoystick joystick2;
        getJoystickState(&joystick1, &joystick2);

        switch (joystick_index)
        {
            case 0:
                if (joystick1.ButtonState & 128)
                {
                    *is_active = true;
                }
                else
                {
                    *is_active = false;
                }
                break;
            case 1:
                if (joystick2.ButtonState & 128)
                {
                    *is_active = true;
                }
                else
                {
                    *is_active = false;
                }
                break;
        }
    }

    void PrioConnection::getActiveJoysticks(U8* active_joysticks)
    {
        PrioJoystick joystick1;
        PrioJoystick joystick2;
        getJoystickState(&joystick1, &joystick2);
        *active_joysticks = 0;

        if (joystick1.ButtonState & 128)
        {
            *active_joysticks += 1;
        }

        if (joystick2.ButtonState & 128)
        {
            *active_joysticks += 2;
        }
    }

    void PrioConnection::setControllerUpdateRate(U32 update_rate)
    {
        _controller_update_rate = update_rate;
    }

    void PrioConnection::getControllerUpdateRate(U32* update_rate)
    {
        *update_rate = _controller_update_rate;
    }

    void PrioConnection::startControllerUpdating()
    {
        if (!_is_joystick_polling)
        {
            _breakControllerThread = false;
            _controllerThread = std::thread(&PrioConnection::_prioControllerThread, this);
            _is_joystick_polling = true;
        }
    }

    void PrioConnection::stopControllerUpdating()
    {
        if (_is_joystick_polling)
        {
            _breakControllerThread = true;
            if (_controllerThread.joinable())
            {
                _controllerThread.join();
            }
            _is_joystick_polling = false;
        }
    }

    void PrioConnection::_updateControllerData(U8* left_joystick_state, U8* right_joystick_state, U8* hub_button)
    {
        _joystick1_state.x_Axis = left_joystick_state[0];
        _joystick1_state.y_Axis = left_joystick_state[1];
        _joystick1_state.Trigger = left_joystick_state[2];
        _joystick1_state.ButtonState = left_joystick_state[3];

        _joystick2_state.x_Axis = right_joystick_state[0];
        _joystick2_state.y_Axis = right_joystick_state[1];
        _joystick2_state.Trigger = right_joystick_state[2];
        _joystick2_state.ButtonState = right_joystick_state[3];

        _hub_button_state = *hub_button;
    }

    // This needs to be called before start streaming
    void PrioConnection::setupStreamingData()
    {
        memset(_stream_slots, PRIO_NULL, sizeof(_stream_slots));
        _stream_slots[0] = PRIO_STREAM_UNTARED_ORIENTATION_AS_QUATERNION;
        _stream_data_size = 0;
        _sensor_stream_data_size = 4;
    }

    // This needs to be called before start streaming
    void PrioConnection::setupStreamingDataWithAcceleration()
    {
        memset(_stream_slots, PRIO_NULL, sizeof(_stream_slots));
        _stream_slots[0] = PRIO_STREAM_UNTARED_ORIENTATION_AS_QUATERNION;
        _stream_slots[1] = PRIO_STREAM_CORRECTED_ACCELEROMETER_VECTOR;
        _stream_data_size = 0;
        _sensor_stream_data_size = 7;
    }

    // This needs to be called before runProcess
    void PrioConnection::startStreaming()
    {
        U32 data_flags = 0;
        for (int i = 0; i < 8; i++)
        {
            data_flags = data_flags + _stream_slots[i];
        }

        if (_device_type == PRIO_TYPE::PRIO_BS)
        {
            prio_error = prio_bs_startStreaming(_device, data_flags, 0, PRIO_STREAM_DURATION_INFINITE);
        }
        else
        {
            prio_error = prio_hub_startStreamingWired(_hub_device_ID, data_flags, 0, PRIO_STREAM_DURATION_INFINITE);
        }

        if (prio_error == PRIO_NO_ERROR)
        {
            prio_error = prio_hub_getFullLengthOfStreamingData(_hub_device_ID, &_stream_data_size);
            if (prio_error == PRIO_NO_ERROR)
            {
                _header_data = std::unique_ptr<prio_StreamHeaderData>(new prio_StreamHeaderData);

                // Sending a blank value to have in case we don't have a first packet to get
                _header_data->battery_level = 0;
                _header_data->hub_button = 0;
                _header_data->joystick1.x_Axis = 128;
                _header_data->joystick1.y_Axis = 128;
                _header_data->joystick1.Trigger = 0;
                _header_data->joystick1.ButtonState = 0;
                _header_data->joystick2.x_Axis = 128;
                _header_data->joystick2.y_Axis = 128;
                _header_data->joystick2.Trigger = 0;
                _header_data->joystick2.ButtonState = 0;

                _stream_data.reset(new float[_stream_data_size]);
                _is_streaming = true;
            }
        }
    }

    void PrioConnection::stopStreaming()
    {
        if (_device_type == PRIO_TYPE::PRIO_BS)
        {
            prio_error = prio_bs_stopStreaming(_device);
        }
        else
        {
            prio_error = prio_hub_stopStreamingWired(_hub_device_ID);
        }

        if (prio_error == PRIO_NO_ERROR)
        {
            _header_data.release();
            _stream_data.release();
            _is_streaming = false;
        }
    }

    void PrioConnection::disconnectDevice()
    {
        return;
    }

    prio_StreamHeaderData PrioConnection::getStreamingHeader()
    {
        return *_header_data;
    }

    // The calibration is calculated using the Hub sensor and applied to the other sensors
    // This needs to be called before runProcess
    bool PrioConnection::calibrate(float wait_time)
    {
        stopStreaming();
        _active_sensors_have_changed = false;
        _is_calibrated = false;

        float* untared_quats = new float[_active_sensors_len * 4];
        std::map<int32_t, Orient> avg_orients;
        PRIO_ERROR error = PRIO_NO_ERROR;
        uint8_t i, j;

        setupSkeletonMap();

        if (!_is_mapped)
        {
            return _is_calibrated;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds((uint32_t)(wait_time * 1000)));

        // Get an average sample of all the sensors' orientations
        std::map<int32_t, Orient>::iterator avg_it;
        std::map<int32_t, Orient>::iterator end_avg_it = avg_orients.end();

        error = prio_hub_getUntaredOrientationAsQuaternion(_hub_device_ID, PRIO_BROADCAST_LOGICAL_ID, _active_sensors_len * 4, untared_quats);
        for (i = 0; i < 10; i++)
        {
            error = prio_hub_getUntaredOrientationAsQuaternion(_hub_device_ID, PRIO_BROADCAST_LOGICAL_ID, _active_sensors_len * 4, untared_quats);
            if (error == PRIO_NO_ERROR)
            {
                for (j = 0; j < _active_sensors_len; j++)
                {
                    if (avg_orients.find(_active_sensors[j]) == end_avg_it)
                    {
                        avg_orients[_active_sensors[j]].data[0] = untared_quats[j * 4];
                        avg_orients[_active_sensors[j]].data[1] = untared_quats[(j * 4) + 1];
                        avg_orients[_active_sensors[j]].data[2] = untared_quats[(j * 4) + 2];
                        avg_orients[_active_sensors[j]].data[3] = untared_quats[(j * 4) + 3];
                    }
                    else
                    {
                        avg_orients[_active_sensors[j]].data[0] += untared_quats[j * 4];
                        avg_orients[_active_sensors[j]].data[1] += untared_quats[(j * 4) + 1];
                        avg_orients[_active_sensors[j]].data[2] += untared_quats[(j * 4) + 2];
                        avg_orients[_active_sensors[j]].data[3] += untared_quats[(j * 4) + 3];
                    }
                }
            }
        }

        for (avg_it = avg_orients.begin(); avg_it != end_avg_it; avg_it++)
        {
            orientNormalize(&((*avg_it).second));
        }

        Orient anchor_placement_orient(0, 0.7071f, -0.7071f, 0);
        orientInverse(&anchor_placement_orient);

        // Check if the Hub sensor is in the list of orientations
        int8_t anchor_sensor_idx = 0;
        if (avg_orients.find(anchor_sensor_idx) == end_avg_it)
        {
            _skeleton->setCalibration(_is_calibrated);
            return _is_calibrated;
        }
        Orient anchor_sensor_orient = avg_orients[anchor_sensor_idx];
        
        // Need to offest the Hub sensor orientation to match the space of the API (Left-handed: X-right, Y-up, Z-forward)
        Orient anchor_reference_orient;
        orientMul(&anchor_sensor_orient, &anchor_placement_orient, &anchor_reference_orient);

        // Calculate the offset for the Z-axis of rotation
        Vector3 z_axis(0, 0, 1);
        orientRotate(&anchor_reference_orient, &z_axis);

        // Calculate the offset for the X-axis of rotation
        Vector3 x_axis;
        Vector3 vector_y = Vector3(0, 1, 0);
        vectorCross(&vector_y, &z_axis, &x_axis);

        Vector3 vector_x = Vector3(1.0f, 0.0f, 0.0f);
        anchor_reference_orient = getOrientFromVectors(&vector_x, &x_axis);

        // Apply offest to all sensors
        std::map<std::string, Orient> pose = _skeleton->getSkeletonPose();
        std::map<std::string, Orient>::iterator pose_it;
        std::map<std::string, Orient>::iterator end_pose_it = pose.end();
        Orient base_pose_orient;
        Orient pose_orient;
        
        std::shared_ptr<Bone> bone;
        uint32_t bone_device_id;
        Orient curr_bone_orient;
        Orient curr_bone_offset;
        Orient curr_bone_tare;

        for (pose_it = pose.begin(); pose_it != end_pose_it; pose_it++)
        {
            bone = _skeleton->getBone((*pose_it).first);
            if (bone == nullptr)
            {
                continue;
            }
            base_pose_orient = (*pose_it).second;
            orientMul(&anchor_reference_orient, &base_pose_orient, &pose_orient);

            bone_device_id = bone->getDeviceId();

            if (avg_orients.find(bone_device_id) != end_avg_it)
            {
                curr_bone_orient = avg_orients[bone_device_id];
                orientInverse(&curr_bone_orient);

                orientMul(&curr_bone_orient, &pose_orient, &curr_bone_offset);

                bone->setCalibrationOffset(curr_bone_offset);

                curr_bone_tare = anchor_reference_orient;
                orientInverse(&curr_bone_tare);

                bone->setCalibrationTare(curr_bone_tare);
            }
        }

        // Update the skeleton before continuing on so positions will be valid
        bone = _skeleton->getRootBone();
        if (bone != nullptr)
        {
            bone->update();
        }

        _is_calibrated = true;
        _skeleton->setCalibration(_is_calibrated);

        return _is_calibrated;
    }

    void PrioConnection::setBoneUpdateMode(bool fused)
    {
        _bone_fused_mode = fused;
    }

    bool PrioConnection::getBoneUpdateMode()
    {
        return _bone_fused_mode;
    }

    // This needs to be called before runProcess
    void PrioConnection::setupSkeletonMap()
    {
        std::shared_ptr<Bone> bone;
        for (int i = 0; i < _active_sensors_len; i++ )
        {
            bone = _skeleton->getBone(_skeleton_id_map[_active_sensors[i]]);
            if (bone == nullptr)
            {
                break;
            }
            bone->setDeviceId(-1);
            bone->setUpdateType(YOST_SKELETON_BONE_UPDATE_NULL);
        }

        resetSkeletonMap();

        std::string xml_data = _skeleton->utilityGetDeviceXmlMap();

        if (xml_data.size() == 0)
        {
            _is_mapped = false;
        }
        else
        {
            rapidxml::xml_document<> xml_doc;
            rapidxml::xml_node<>* root_node;
            rapidxml::xml_node<>* suit_node;

            // Parse the string using the rapidxml file parsing library into xml_doc
            xml_doc.parse<0>(&xml_data[0]);

            // Find our root node
            root_node = xml_doc.first_node();

            // Get our suit layout
            suit_node = root_node->first_node();

            // Interate over the sensor assignments
            for (rapidxml::xml_node<>* sensor_node = suit_node->first_node(); sensor_node != nullptr; sensor_node = sensor_node->next_sibling())
            {
                uint32_t serial = std::stoul(sensor_node->first_attribute("SERIAL")->value(), nullptr, 16);
                std::string bone_name = sensor_node->first_attribute("BONE")->value();
                _skeleton_id_map[serial] = bone_name;
                bone = _skeleton->getBone(bone_name);

                if (bone != nullptr)
                {
                    bone->setDeviceId(serial);
                    bone->setUpdateType(YOST_SKELETON_BONE_UPDATE_DEVICE);
                    bone->setAllChildrenUpdateType(YOST_SKELETON_BONE_UPDATE_FUSED);
                }
                else
                {
                    _is_mapped = false;
                    return;
                }
            }
            _is_mapped = true;
        }
    }

    void PrioConnection::activeSensorsHaveChanged(bool* changed)
    {
        *changed = _active_sensors_have_changed;
    }
}