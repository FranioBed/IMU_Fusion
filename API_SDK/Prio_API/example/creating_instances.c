/********************************************//**
 * This example shows how to connect to and find a Prio device.
 * When an instance of a Prio device is created in the API a prio_device_id is returned.
 * This device ID is needed to call many functions in the API.
 *
 * Copyright 1998-2014, YEI Corporation
 * This Source Code Form is subject to the terms of the YEI 3-Space Open
 * Source License available online at:
 * http://www.yeitechnology.com/yei-3-space-open-source-license
 ***********************************************/

#include <stdio.h>
#include <stdint.h>
#include "../../prio_api.h"

int main()
{
    prio_device_id prio_device;
    PRIO_ERROR error;

    // If the serial port is already known for the Prio device, we can just create an instance without doing a search
    const char* com_port_str = "COM7";    // <-- Edit this to your device's serial port
    printf("====Creating Prio Device on %s====\n", com_port_str);
    prio_device = prio_createDeviceStr(com_port_str, PRIO_TIMESTAMP_SYSTEM);
    if (prio_device == PRIO_NO_DEVICE_ID)
    {
        printf("Failed to create a device on %s\n", com_port_str);
    }
    else
    {
        printf("Created a Prio device on %s\n", com_port_str);
        prio_destroyDevice(prio_device);
    }
    printf("================================\n");

    // Finding a single serial port for a Prio device to create an instance for
    printf("\n====Creating a Prio Device from Search====\n");
    prio_ComPort com_port;
    int offset = 0;
    while (prio_getComPorts(&com_port, 1, offset, PRIO_FIND_ALL_KNOWN))
    {
        prio_device = prio_createDeviceStr(com_port.com_port, PRIO_TIMESTAMP_SYSTEM);
        if (prio_device == PRIO_NO_DEVICE_ID)
        {
            printf("Failed to create a device on %s\n", com_port.com_port);
        }
        else
        {
            printf("Created a Prio device on %s\n", com_port.com_port);
            prio_destroyDevice(prio_device);
        }
        offset++;
    }
    printf("================================\n");

    // Finding all known serial ports for a Prio device to create an instance for
    printf("\n====Creating Prio Devices from Search====\n");
    prio_ComPort com_ports[20];
    int device_count = prio_getComPorts(com_ports, 20, 0, PRIO_FIND_ALL_KNOWN);
    printf("========Found %d Device(s)========\n", device_count);
    for (int i = 0; i < device_count; ++i)
    {
        prio_device = prio_createDeviceStr(com_ports[i].com_port, PRIO_TIMESTAMP_SYSTEM);
        if (prio_device == PRIO_NO_DEVICE_ID)
        {
            printf("Failed to create a device on %s\n", com_ports[i].com_port);
        }
        else
        {
            printf("Created a Prio device on %s\n", com_ports[i].com_port);
            prio_destroyDevice(prio_device);
        }
    }
    printf("================================\n");

    // Finding all unknown serial port types and checking if they are a Prio device
    printf("\n====Checking Unknown Ports for Prio Devices====\n");
    device_count = prio_getComPorts(com_ports, 20, 0, PRIO_FIND_UNKNOWN);
    printf("========Found %d Device(s)========\n", device_count);
    prio_DeviceInfo device_info;
    for (int i = 0; i < device_count; ++i)
    {
        prio_device = prio_createDeviceStr(com_ports[i].com_port, PRIO_TIMESTAMP_SYSTEM);
        if (prio_device == PRIO_NO_DEVICE_ID)
        {
            printf("Failed to create a device on %s\n", com_ports[i].com_port);
        }
        else
        {
            printf("Created a Prio device on %s\n", com_ports[i].com_port);
            error = prio_getDeviceInfo(prio_device, &device_info);
            if (error == PRIO_NO_ERROR)
            {
                printf("============(%s)=============\n", com_ports[i].com_port);
                printf("DeviceType:%s\n", prio_type_string[device_info.device_type]);
                printf("Serial:%08X\n", device_info.serial_number);
                printf("HardwareVersion:%s\n", device_info.hardware_version);
                printf("FirmwareVersion:%s\n", device_info.firmware_version);
                printf("Compatibility:%d\n", device_info.fw_compatibility);
            }
            else
            {
                printf("ERROR: %s\n", prio_error_string[error]);
            }
            prio_destroyDevice(prio_device);
        }
    }
    printf("================================\n");

    // Call reset to clean up the Prio API
    printf("====Resetting API====\n");
    error = prio_resetPrioApi();
    if (error != PRIO_NO_ERROR)
    {
        printf("ERROR: %s\n", prio_error_string[error]);
    }

    printf("\nFinished press Enter to continue");
    getchar();
    return 0;
}
