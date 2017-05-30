/********************************************//**
 * This example demonstrates getting basic data wirelessly with the Prio devices.
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

int main()
{
    prio_device_id prio_device;
    prio_ComPort com_port;
    PRIO_ERROR error;

    // In this example we are looking for a Prio Base Station (PRIO_FIND_BS) to communicate wirelessly
    // to a Prio Hub, getting information from the sensor nodes connected
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

    // Searching for all present sensor nodes
    printf("\n====Finding All Present Sensor Nodes====\n");
    uint8_t present_nodes[42] = { 0 };
    uint8_t i;
    for (i = 0; i < sizeof(present_nodes) / sizeof(uint8_t); i++)
    {
        uint8_t present;
        error = prio_isSensorPresent(prio_device, i, &present);
        present_nodes[i] = present;
        if (present)
        {
            printf("Sensor Node %d is present\n", i);
        }
    }
    printf("================================\n");

    // Enumerate the Prio Hub to find new sensor nodes
    printf("\n====Enumerating the Prio Hub====\n");
    error = prio_enumerateHub(prio_device, NULL);
    printf("================================\n");

    // Check to see which sensor nodes are present after enumeration
    printf("\n====Finding All Present Sensor Nodes After Enumeration====\n");
    uint8_t present;
    for (i = 0; i < sizeof(present_nodes) / sizeof(uint8_t); i++)
    {
        error = prio_isSensorPresent(prio_device, i, &present);
        present_nodes[i] = present;
        if (present)
        {
            printf("Sensor Node %d is present\n", i);
        }
    }
    printf("================================\n");

    // Get the Tared Orientation of each present sensor node.
    printf("\n====Getting Tared Orientation of Each Present Sensor Node====\n");
    float quat[4] = { 0 };
    uint32_t timestamp = 0;
    for (i = 0; i < sizeof(present_nodes) / sizeof(uint8_t); i++)
    {
        if (present_nodes[i])
        {
            // Checking the error returned from the function is typically a good idea
            // In most cases functions succeed as long as the parameters are valid, though calls sent wirelessly should always be checked
            error = prio_getTaredOrientationAsQuaternion(prio_device, i, quat, &timestamp);
            if (error == PRIO_NO_ERROR)
            {
                printf("====Sensor Node %d====\n", i);
                printf("Quat: %f,%f,%f,%f  --  Timestamp: %u\n", quat[0], quat[1], quat[2], quat[3], timestamp);
            }
            else
            {
                printf("ERROR: %s\n", prio_error_string[error]);
            }
            memset(quat, 0, sizeof(quat));
            timestamp = 0;
        }
    }
    printf("================================\n");

    // Destory the instance of the Prio Base Station
    prio_destroyDevice(prio_device);

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
