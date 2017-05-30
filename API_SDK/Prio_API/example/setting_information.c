/********************************************//**
 * This example demonstrates setting options on Prio devices.
 *
 * Copyright 1998-2014, YEI Corporation
 * This Source Code Form is subject to the terms of the YEI 3-Space Open
 * Source License available online at:
 * http://www.yeitechnology.com/yei-3-space-open-source-license
 ***********************************************/

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "../../prio_api.h"

// OS specific sleep
#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif // _WIN32

void mySleep(uint32_t milliseconds)
{
#ifdef _WIN32
    Sleep(milliseconds);
#else
    usleep(milliseconds);
#endif // _WIN32
}

int main()
{
    prio_device_id  prio_device;
    prio_ComPort com_port;
    PRIO_ERROR error;

    // In this example we are looking for a Prio Base Station (PRIO_FIND_BS) to communicate wirelessly
    // to a Prio Hub, setting the LED color for both the Hub and Base Station
    // We are going to connect to the first one found
    printf("====Creating a Prio Device from Search====\n");
    if (prio_getComPorts(&com_port, 1, 0, PRIO_FIND_BS))
    {
        prio_device = prio_createDeviceStr(com_port.com_port, PRIO_TIMESTAMP_SYSTEM);
        if (prio_device == PRIO_NO_DEVICE_ID)
        {
            printf("Failed to create a device on %s\n", com_port.com_port);
            return 1;
        }
        printf("Created a Prio device on %s\n", com_port.com_port);
    }
    else
    {
        printf("No Prio Base Station devices found\n");
        return 1;
    }
    printf("================================\n");

    printf("\n====Setting the LED Color of the Prio Hub to Pulse BLUE====\n");
    uint8_t start_rgb[3] = { 0, 0, 255 };
    uint8_t end_rgb[3] = { 0, 0, 0 };
    error = prio_setLedColorSuit(prio_device, PRIO_HUB_LOGICAL_ID, start_rgb, end_rgb, NULL);
    // Checking the error returned from the function is typically a good idea
    // In most cases functions succeed as long as the parameters are valid, though calls sent wirelessly should always be checked
    if (error == PRIO_NO_ERROR)
    {
        printf("LED should be plusing BLUE\n");
        printf("================================\n");

        mySleep(2000);

        printf("\n====Setting the LED Color of the Prio Hub to Pulse RED====\n");
        start_rgb[0] = 255;
        start_rgb[2] = 0;
        error = prio_setLedColorSuit(prio_device, PRIO_HUB_LOGICAL_ID, start_rgb, end_rgb, NULL);
        if (error == PRIO_NO_ERROR)
        {
            printf("LED should be plusing RED\n");
        }
        else
        {
            printf("ERROR: %s\n", prio_error_string[error]);
        }
    }
    else
    {
        printf("ERROR: %s\n", prio_error_string[error]);
    }
    printf("================================\n");

    printf("\n====Setting the LED Color of the Prio Base Station to RED====\n");
    float red[3] = { 1.0f, 0.0f, 0.0f };
    error = prio_setLedColor(prio_device, red, NULL);
    if (error == PRIO_NO_ERROR)
    {
        printf("LED should be be RED\n");
    }
    else
    {
        printf("ERROR: %s\n", prio_error_string[error]);
    }
    printf("================================\n");

    mySleep(2000);

    printf("\n====Setting the LED Color of the Prio Base Station to RED====\n");
    float blue[3] = { 0.0f, 0.0f, 1.0f };
    error = prio_setLedColor(prio_device, blue, NULL);
    if (error == PRIO_NO_ERROR)
    {
        printf("LED should be be BLUE\n");
    }
    else
    {
        printf("ERROR: %s\n", prio_error_string[error]);
    }
    printf("================================\n");

    // Destory the instance of the Prio Base Station
    prio_destroyDevice(prio_device);

    // Call reset to clean up the PRIO API
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
