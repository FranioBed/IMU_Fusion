/********************************************//**
 * This example demonstrates setting up and getting stream data from a Prio device.
 *
 * Copyright 1998-2014, YEI Corporation
 * This Source Code Form is subject to the terms of the YEI 3-Space Open
 * Source License available online at:
 * http://www.yeitechnology.com/yei-3-space-open-source-license
 ***********************************************/
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <stdint.h>
#include "../../prio_api.h"

int main()
{
    prio_device_id prio_device;
    prio_ComPort com_port;
    PRIO_ERROR error;
    const float RUN_TIME = 10.0f;    // in seconds
    clock_t start_time;

    // In this example we are looking for a Prio Base Station (PRIO_FIND_BS) to communicate wirelessly
    // to a Prio Hub, setting up streaming and reading the stream data
    // We are going to connect to the first one found
    printf("====Creating a Prio Device from Search====\n");
    if (prio_getComPorts(&com_port, 1, 0, PRIO_FIND_BS))
    {
        prio_device = prio_createDeviceStr(com_port.com_port, PRIO_TIMESTAMP_SYSTEM);
        if (prio_device == PRIO_NO_DEVICE_ID)
        {
            printf("Failed to create a sensor on %s\n", com_port.com_port);
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

    // Getting the number of active sensor nodes will help in reading the stream data
    printf("\n====Get the Length of Active Sensor Nodes of the Prio Hub====\n");
    uint8_t* active_nodes;
    uint8_t active_nodes_len = 0;
    error = prio_getActiveSensors(prio_device, NULL, 0, &active_nodes_len);
    active_nodes = (uint8_t*)malloc(active_nodes_len * sizeof(uint8_t));
    error = prio_getActiveSensors(prio_device, active_nodes, active_nodes_len, NULL);
    printf("================================\n");

    // We must first setup the streaming slots of the Prio device before streaming
    printf("\n====Set the Stream Slots of the Prio Device====\n");
    prio_stream_commands slots8[8] = { PRIO_GET_UNTARED_ORIENTATION_AS_QUATERNION, PRIO_NULL, PRIO_NULL, PRIO_NULL, PRIO_NULL, PRIO_NULL, PRIO_NULL, PRIO_NULL };
    error = prio_setStreamingSlots(prio_device, slots8, NULL);
    printf("================================\n");

    // Next we must setup the streaming parameters of the Prio device based on the streaming slots
    printf("\n====Set the Stream Parameters of the Prio Device====\n");
    prio_StreamHeaderData header_data;
    float* stream_data;
    uint32_t stream_timestamp;
    uint32_t stream_data_len = 0;
    error = prio_getFullLengthOfStreamData(prio_device, &stream_data_len);
    stream_data = (float*)malloc(active_nodes_len * stream_data_len);
    printf("================================\n");

    // Now you may start streaming
    // There are two ways of getting the stream data (Blocked and Non-Blocked)
    printf("\n====Start Streaming====\n");
    error = prio_startStreaming(prio_device, 1, NULL);
    if (error == PRIO_NO_ERROR)
    {
        printf("Streaming has started!\n");
        printf("================================\n");

        start_time = clock();
        uint8_t i;
        uint32_t stream_idx;

        // The Blocked method is great for getting unique data as it will wait the given time till new data has been read
        printf("\n====Get Data Using Blocked Method====\n");
        while ((((float)(clock() - start_time)) / CLOCKS_PER_SEC < RUN_TIME))
        {
            error = prio_getLatestStreamData(prio_device, &header_data, stream_data, active_nodes_len * stream_data_len, 16, &stream_timestamp);
            if (error == PRIO_NO_ERROR)
            {
                for (i = 0; i < active_nodes_len; i++)
                {
                    stream_idx = i * (stream_data_len / sizeof(float));
                    printf("====Sensor Node %d====\n", active_nodes[i]);
                    printf("Quat: %f, %f, %f, %f -- Timestamp: %u\n", stream_data[stream_idx], stream_data[stream_idx + 1], stream_data[stream_idx + 2], stream_data[stream_idx + 3], stream_timestamp);
                }
            }
            else
            {
                printf("ERROR: %s\n", prio_error_string[error]);
            }
        }
        printf("================================\n");

        start_time = clock();

        // The Non-Blocked method is great for getting data fast but it is not guaranteed  that new data has been read
        printf("\n====Get Data Using Non-Blocked Method====\n");
        while ((((float)(clock() - start_time)) / CLOCKS_PER_SEC < RUN_TIME))
        {
            error = prio_getLastStreamData(prio_device, &header_data, stream_data, active_nodes_len * stream_data_len, &stream_timestamp);
            if (error == PRIO_NO_ERROR)
            {
                for (i = 0; i < active_nodes_len; i++)
                {
                    stream_idx = i * (stream_data_len / sizeof(float));
                    printf("====Sensor Node %d====\n", active_nodes[i]);
                    printf("Quat: %f, %f, %f, %f -- Timestamp: %u\n", stream_data[stream_idx], stream_data[stream_idx + 1], stream_data[stream_idx + 2], stream_data[stream_idx + 3], stream_timestamp);
                }
            }
            else
            {
                printf("ERROR: %s\n", prio_error_string[error]);
            }
        }

        // It is best to stop the streaming once it is no longer needed as it can interfere with other command calls
        printf("\n====Stop Streaming====\n");
        error = prio_stopStreaming(prio_device, NULL);
        if (error == PRIO_NO_ERROR)
        {
            printf("Streaming has stopped!\n");
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

    // Clean up allocated memory
    free(active_nodes);
    free(stream_data);

    // Destory the instance of the Prio Base Station
    prio_destroyDevice(prio_device);

    // Call reset to clean up the PRIO API
    printf("====Resetting API====\n");
    error = prio_resetPrioApi();
    if (error != PRIO_NO_ERROR)
    {
        printf("PRIO ERROR: %s\n", prio_error_string[error]);
    }

    printf("\nFinished press Enter to continue");
    getchar();
    return 0;
}
