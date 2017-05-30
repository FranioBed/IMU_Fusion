/********************************************//**
* This file should not be included for compiling the Prio API.
* This file is mainly a testing ground for various functionality of the API and Prio devices.
*
* Copyright 1998-2014, YEI Corporation
* This Source Code Form is subject to the terms of the YEI 3-Space Open
* Source License available online at:
* http://www.yosttechnology.com/yost-3-space-open-source-license
***********************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <memory>
#include <windows.h>
#include <chrono>
#include <thread>
#include <vector>
#include <string>
#include "prio_api_export.h"
#include "yost_skeleton_api.h"
#include <iostream>
#include <fstream>
#include <time.h>
using namespace std;

void prio_pro_skeleton_test()
{
    yost_skeleton_id skel_id;
    YOST_SKELETON_ERROR error;

    // Create a skeleton using the user's sex and age
    uint8_t is_male = 1;    // <-- Edit this to your user's sex
    uint32_t age = 24;      // <-- Edit this to your user's age

    printf("====Creating Skeleton for a %s, %d Years Old====\n", is_male ? "Male" : "Female", age);
    skel_id = yostskel_createStandardSkeletonWithHeight(is_male, 1.83f);
    if (skel_id == YOST_SKELETON_INVALID_ID)
    {
        printf("Failed to create a skeleton for a %s, %d years old\n", is_male ? "Male" : "Female", age);
    }
    else
    {
        printf("Created a skeleton for a %s, %d years old\n", is_male ? "Male" : "Female", age);
    }
    printf("================================\n");

	yostskel_setSkeletonUnit(skel_id, YOST_SKELETON_UNIT_METERS);

    // Check Version String
    char versionString[64];
    error = yostskel_getVersionString(versionString, 64);
    printf("====Version String====\n");
    if (error != YOST_SKELETON_NO_ERROR)
    {
        printf("ERROR: %s\n", yost_skeleton_error_string[error]);
    }
    else
    {
        printf("%s\n", versionString);
    }

    // Sets the skeleton to the standard T Pose
    printf("====Setting a Pose for Skeleton====\n");
    error = yostskel_setSkeletonToStandardTPose(skel_id);
    if (error != YOST_SKELETON_NO_ERROR)
    {
        printf("ERROR: %s\n", yost_skeleton_error_string[error]);
    }
    else
    {
        printf("Skeleton is now in a \"T\" pose\n");
    }
    printf("================================\n");

    error = yostskel_setStandardDeviceXmlMapPrioProLayout(skel_id);


    // Create a Prio Processor
    printf("====Creating a Prio Processor====\n");
    yost_skeleton_id prio_id = yostskel_createPrioProcessorWithComOffset(PRIO_TYPE::PRIO_ALL,0);
    if (prio_id == YOST_SKELETON_INVALID_ID)
    {
        printf("Failed to create Prio Processor\n");
        getchar();
        yostskel_destroyProcessor(prio_id);
        yostskel_destroySkeleton(skel_id);
        // Reset API to clean up allocated memory
        printf("====Resetting API====\n");
        error = yostskel_resetSkeletalApi();
        if (error != YOST_SKELETON_NO_ERROR)
        {
            printf("ERROR: %s\n", yost_skeleton_error_string[error]);
        }

        printf("\nFinished press Enter to continue");
        getchar();
        return;
    }
    else
    {
        printf("Created a Prio Processor %d\n", prio_id);
        U8 deviceType = 225;
        error = yostskel_getPrioProcessorDeviceType(prio_id, &deviceType);
        printf("Processor type: %d\n", deviceType);
        //Get the processor id
        UINT32 procIndex;
        error = yostskel_getPrioProcessorDeviceIndex(prio_id, &procIndex);
        printf("Processor id: %d\n", procIndex);
    }
    printf("================================\n");

    // Add the Prio Proccesor to the internal skeleton
    // Adding a Processor to the skeleton allows the Processor to manipulate the bones of the skeleton
    printf("====Adding the Prio Processor to the Skeleton====\n");
    error = yostskel_addProcessorToSkeleton(skel_id, YOST_SKELETON_PROCESSOR_LIST_END, prio_id);
    if (error != YOST_SKELETON_NO_ERROR)
    {
        printf("ERROR: %s\n", yost_skeleton_error_string[error]);
    }
    else
    {
        printf("Added Prio Processor(%d) to Skeleton(%d)\n", prio_id, skel_id);
    }
    printf("================================\n");

    //Get processor list
    yost_skeleton_id processors[2];
    error = yostskel_getProcessorListFromSkeleton(skel_id, processors, 2);

    //Processor name lis
    char processNames[1][20];

    error = yostskel_getProcessorNameListFromSkeleton(skel_id, (char**)processNames, 1, 20);
    printf("Processors(%d) named: (%s)\n", processors[0], processNames[0]);

    if (error != YOST_SKELETON_NO_ERROR)
    {
        printf("ERROR: %s\n", yost_skeleton_error_string[error]);
    }
    else
    {
        printf("Added Prio Processor(%d) to Skeleton(%d)\n", prio_id, skel_id);
    }
    printf("================================\n");


    // Finding the root bone
    printf("====Setting up skeleton bones====\n");
    char name_buff[256];
    error = yostskel_getRootBoneName(skel_id, name_buff, 256);
    if (error != YOST_SKELETON_NO_ERROR)
    {
        printf("ERROR: %s\n", yost_skeleton_error_string[error]);
    }
    else
    {
        printf("Root Bone: %s\n", name_buff);
    }

    // Get the standard bone names and put them in a array
    // Doing this makes it easy to loop through all the bones needed
    char names[5][32];
    // The following should return YEI_SKELETON_NO_ERROR as long as the buffer size is large enough
    error = yostskel_getStandardBoneName(YOST_SKELETON_BONE_SPINE, names[0], 32);
    error = yostskel_getStandardBoneName(YOST_SKELETON_BONE_HEAD, names[1], 32);
    error = yostskel_getStandardBoneName(YOST_SKELETON_BONE_LEFT_FOOT, names[2], 32);
    error = yostskel_getStandardBoneName(YOST_SKELETON_BONE_RIGHT_FOOT, names[3], 32);
    error = yostskel_getStandardBoneName(YOST_SKELETON_BONE_HIPS, names[4], 32);

    printf("================================\n");

    // Calibrating the Prio suit while establishing the Skeleton Bone Map
    // Pause to follow in exe
    printf("\nPress Enter to Start Calibration\n");
    getchar();
    yostskel_stopPrioProcessor(prio_id);
    printf("====Calibrating the Prio Suit====\n");
    error = yostskel_calibratePrioProcessor(prio_id,0);
    if (error != YOST_SKELETON_NO_ERROR)
    {
        printf("ERROR: %s\n", yost_skeleton_error_string[error]);
        getchar();
        return;
    }
    else
    {
        printf("Prio calibration complete\n");
        bool cal;
        error = yostskel_isCalibrated(skel_id, &cal);
        if (cal)
        {
            printf("Skeleton is calibrated\n");
        }
    }
    printf("================================\n");

    // Sets up the standard streaming of data for the Prio suit
    // Runs the Prio Processor
    printf("====Start Streaming====\n");
    error = yostskel_setupStandardStreamPrioProcessor(prio_id);
    //error = yostskel_setupStandardStreamWithAccelerationPrioProcessor(prio_id);
    error = yostskel_startPrioProcessor(prio_id);

    if (error == YOST_SKELETON_NO_ERROR)
    {
        printf("Streaming has started!\n");
        printf("================================\n");

        // Loop through all bone names and return the orientation
        // Only bones active in the Tss layout will be returned
        float quat[16][4];
        float offset[16][4];
        float pos[16][3];
        float vel[16][3];
        float accel[16][3];
        float translation[3];
        uint8_t i;

        printf("\n====Getting Data from Skeleton====\n");
        int timer = 0;
        clock_t time;
        double deltaTime = 0;
        while (timer < 10)
        {
            time = clock();
            error = yostskel_update(skel_id);
            if (error == YOST_SKELETON_NO_ERROR)
            {
                for (i = 0; i < 2; i++)
                {
                    printf("====Bone: %s====\n", names[i]);
                    //Gets the orientation of a given bone
                    error = yostskel_getBoneOrientation(skel_id, names[i], quat[i]);
                    error = yostskel_getBoneOrientationOffset(skel_id, names[i], offset[i]);
                    error = yostskel_getBonePosition(skel_id, names[i], pos[i]);
                    error = yostskel_getBoneVelocity(skel_id, names[i], vel[i]);
                    error = yostskel_getBoneAcceleration(skel_id, names[i], accel[i]);
                    if (error == YOST_SKELETON_NO_ERROR)
                    {
                        printf("Quat: %f, %f, %f, %f\n", quat[i][0], quat[i][1], quat[i][2], quat[i][3]);
                        printf("Offset: %f, %f, %f, %f\n", offset[i][0], offset[i][1], offset[i][2], offset[i][3]);
                        printf("Position: %f, %f, %f\n", pos[i][0], pos[i][1], pos[i][2]);
                        printf("Velocity: %f, %f, %f\n", vel[i][0], vel[i][1], vel[i][2]);
                        printf("Accelerations: %f, %f, %f\n", accel[i][0], accel[i][1], accel[i][2]);
                    }
                    else
                    {
                        printf("ERROR: %s on names[%d] %s\n", yost_skeleton_error_string[error], i, names[i]);
                    }
                }
            }
            else
            {
                printf("ERROR: %s\n", yost_skeleton_error_string[error]);
            }

            bool hasUpdated = false;
            error = yostskel_hasPrioProcessorUpdatedState(prio_id, &hasUpdated);
            if (error == YOST_SKELETON_NO_ERROR)
            {
                printf("State changed: %d", hasUpdated);

                if (hasUpdated)
                {
                    //error = yostskel_stopPrioProcessor(prio_id);
                    error = yostskel_setSkeletonToStandardTPose(skel_id);
                    error = yostskel_calibratePrioProcessor(prio_id, 0);
                    error = yostskel_startPrioProcessor(prio_id);
                }
            }
            else
            {
                printf("ERROR in state has changed: %s\n", yost_skeleton_error_string[error]);
            }

            getchar();
            timer += 1;
            deltaTime += (clock() - time) / (CLOCKS_PER_SEC / 1000);
            printf("============DeltaTime: %f\n", (clock() - time) );
        }
        deltaTime = deltaTime/100;
        printf("=============Average time of 100000: %f=============\n", deltaTime);
        printf("================================\n");
        U8 battery_level = 0;
        error = yostskel_getHubBatteryLevel(prio_id, &battery_level);
        if (error == YOST_SKELETON_NO_ERROR)
        {
            printf("Battery Level: %d\n", battery_level);
        }
        else
        {
            printf("ERROR: %s\n", yost_skeleton_error_string[error]);
        }
        printf("====Stop Streaming====\n");
        error = yostskel_stopPrioProcessor(prio_id);
        if (error == YOST_SKELETON_NO_ERROR)
        {
            printf("Streaming has stopped!\n");
        }
        else
        {
            printf("ERROR: %s\n", yost_skeleton_error_string[error]);
        }

        battery_level = 255;
        error = yostskel_getHubBatteryLevel(prio_id, &battery_level);
        if (error == YOST_SKELETON_NO_ERROR)
        {
            printf("Battery Level: %d\n", battery_level);
        }
        else
        {
            printf("ERROR: %s\n", yost_skeleton_error_string[error]);
        }

        memset(quat, 0, sizeof(quat));
    }
    else
    {
        printf("ERROR: %s\n", yost_skeleton_error_string[error]);
    }
    printf("================================\n");
    memset(names, 0, sizeof(names));

    yostskel_startPrioProcessor(prio_id);

    uint8_t leftJoy[4];
    uint8_t rightJoy[4];

    for (int i = 0; i < 5; i++)
    {
        yostskel_update(skel_id);
        error = yostskel_getJoyStickStatePrioProcessor(prio_id, &leftJoy[0], &rightJoy[0]);
        if (error != YOST_SKELETON_NO_ERROR)
        {
            printf("ERROR: %s\n", yost_skeleton_error_string[error]);
            return;
        }
        else
        {
            printf("Left Joystick x-axis: %d\n", leftJoy[0]);
            printf("Left Joystick y-axis: %d\n", leftJoy[1]);
            printf("Left Joystick trigger: %d\n", leftJoy[2]);
            printf("Left Joystick button state: %d\n", leftJoy[3]);

            printf("Right Joystick x-axis: %d\n", rightJoy[0]);
            printf("Right Joystick y-axis: %d\n", rightJoy[1]);
            printf("Right Joystick trigger: %d\n", rightJoy[2]);
            printf("Right Joystick button state: %d\n", rightJoy[3]);
        }
        getchar();
    }

    yostskel_stopPrioProcessor(prio_id);

    yostskel_destroySkeleton(skel_id);

    // Reset API to clean up allocated memory
    printf("====Resetting API====\n");
    error = yostskel_resetSkeletalApi();
    if (error != YOST_SKELETON_NO_ERROR)
    {
        printf("ERROR: %s\n", yost_skeleton_error_string[error]);
    }

    return;
}

void prio_pro_basestation_test()
{
    yost_skeleton_id skel_id;
    YOST_SKELETON_ERROR error;

    // Create a skeleton using the user's sex and age
    uint8_t is_male = 1;    // <-- Edit this to your user's sex
    uint32_t age = 23;      // <-- Edit this to your user's age

    printf("====Creating Skeleton for a %s, %d Years Old====\n", is_male ? "Male" : "Female", age);
    skel_id = yostskel_createStandardSkeletonWithHeight(is_male, 1.83f);

    if (skel_id == YOST_SKELETON_INVALID_ID)
    {
        printf("Failed to create a skeleton for a %s, %d years old\n", is_male ? "Male" : "Female", age);
    }
    else
    {
        printf("Created a skeleton for a %s, %d years old\n", is_male ? "Male" : "Female", age);
    }
    printf("================================\n");

    // Check Version String
    char versionString[64];
    error = yostskel_getVersionString(versionString, 64);
    printf("====Version String====\n");
    if (error != YOST_SKELETON_NO_ERROR)
    {
        printf("ERROR: %s\n", yost_skeleton_error_string[error]);
    }
    else
    {
        printf("%s\n", versionString);
    }

    // Sets the skeleton to the standard T Pose
    printf("====Setting a Pose for Skeleton====\n");
    error = yostskel_setSkeletonToStandardTPose(skel_id);
    if (error != YOST_SKELETON_NO_ERROR)
    {
        printf("ERROR: %s\n", yost_skeleton_error_string[error]);
    }
    else
    {
        printf("Skeleton is now in a \"T\" pose\n");
    }
    printf("================================\n");

    error = yostskel_setStandardDeviceXmlMapPrioProLayout(skel_id);


    // Create a Prio Processor
    printf("====Creating a Prio Device from Search====\n");
    prio_device_id bs_device;
    prio_device_id hub_device;
    prio_ComPort port;
    port.port_name = new char[64];
    PRIO_ERROR prio_error;

    prio_findPorts(PRIO_BS);

    if (prio_getPort(port.port_name, 0, &port.device_type) == PRIO_NO_ERROR)
    {
        prio_error = prio_createBaseStation(port.port_name, &bs_device);

        if (error)
        {
            printf("Failed to create PrioVR on %s!\n", port.port_name);
            return;
        }

        prio_error = prio_bs_getWirelessHub(bs_device, &hub_device);

        if (error)
        {
            printf("Failed to create PrioVR on %s!\n", port.port_name);
            return;
        }
    }
    else
    {
        printf("Failed to get the port!\n");
        return;
    }
    printf("====Created a BaseStation====\n");

    printf("====Creating a Prio Processor====\n");
    yost_skeleton_id prio_id = yostskel_createPrioProcessorWithPrioVRBasestation(bs_device);
    if (prio_id == YOST_SKELETON_INVALID_ID)
    {
        printf("Failed to create Prio Processor\n");
        getchar();
        yostskel_destroyProcessor(prio_id);
        yostskel_destroySkeleton(skel_id);
        // Reset API to clean up allocated memory
        printf("====Resetting API====\n");
        error = yostskel_resetSkeletalApi();
        if (error != YOST_SKELETON_NO_ERROR)
        {
            printf("ERROR: %s\n", yost_skeleton_error_string[error]);
        }

        printf("\nFinished press Enter to continue");
        getchar();
        return;
    }
    else
    {
        printf("Created a Prio Processor %d\n", prio_id);
        //Get the processor id
        UINT32 procIndex;
        error = yostskel_getPrioProcessorDeviceIndex(prio_id, &procIndex);
        printf("Processor id: %d", procIndex);
    }
    printf("================================\n");


    // Add the Prio Proccesor to the internal skeleton
    // Adding a Processor to the skeleton allows the Processor to manipulate the bones of the skeleton
    printf("====Adding the Prio Processor to the Skeleton====\n");
    error = yostskel_addProcessorToSkeleton(skel_id, YOST_SKELETON_PROCESSOR_LIST_END, prio_id);
    if (error != YOST_SKELETON_NO_ERROR)
    {
        printf("ERROR: %s\n", yost_skeleton_error_string[error]);
    }
    else
    {
        printf("Added Prio Processor(%d) to Skeleton(%d)\n", prio_id, skel_id);
    }
    printf("================================\n");

    //Get processor list
    yost_skeleton_id processors[2];
    error = yostskel_getProcessorListFromSkeleton(skel_id, processors, 2);

    //Processor name list
    char processNames[1][20];

    error = yostskel_getProcessorNameListFromSkeleton(skel_id, (char**)processNames, 1, 20);
    printf("Processors(%d) named: (%s)\n", processors[0], processNames[0]);

    //error = yostskel_removeProcessorFromSkeleton(skel_id, 0);

    if (error != YOST_SKELETON_NO_ERROR)
    {
        printf("ERROR: %s\n", yost_skeleton_error_string[error]);
    }
    else
    {
        printf("Added Prio Processor(%d) to Skeleton(%d)\n", prio_id, skel_id);
    }
    printf("================================\n");

    U8 battery_level = 255;
    error = yostskel_getHubBatteryLevel(prio_id, &battery_level);

    if (error != YOST_SKELETON_NO_ERROR)
    {
        printf("ERROR: %s\n", yost_skeleton_error_string[error]);
    }
    else
    {
        printf("Battery Level: %d\n", battery_level);
    }

    // Finding the root bone
    printf("====Setting up skeleton bones====\n");
    char name_buff[256];
    error = yostskel_getRootBoneName(skel_id, name_buff, 256);
    if (error != YOST_SKELETON_NO_ERROR)
    {
        printf("ERROR: %s\n", yost_skeleton_error_string[error]);
    }
    else
    {
        printf("Root Bone: %s\n", name_buff);
    }

    // Get the standard bone names and put them in a array
    // Doing this makes it easy to loop through all the bones needed
    char names[2][64];
    // The following should return YEI_SKELETON_NO_ERROR as long as the buffer size is large enough
    error = yostskel_getStandardBoneName(YOST_SKELETON_BONE_SPINE, names[0], 32);
    error = yostskel_getStandardBoneName(YOST_SKELETON_BONE_HEAD, names[1], 32);


    //Testing polling utility
    error = yostskel_utilityRegisterBone(skel_id, names[0]);
    error = yostskel_utilityRegisterBone(skel_id, names[1]);

    error = yostskel_utilityDeregisterBone(skel_id, names[1]);

    printf("================================\n");

    // Calibrating the Prio suit while establishing the Skeleton Bone Map
    // Pause to follow in exe
    printf("\nPress Enter to Start Calibration\n");
    getchar();
    yostskel_stopPrioProcessor(prio_id);
    printf("====Calibrating the Prio Suit====\n");
    error = yostskel_calibratePrioProcessor(prio_id, 0);
    if (error != YOST_SKELETON_NO_ERROR)
    {
        printf("ERROR: %s\n", yost_skeleton_error_string[error]);
        getchar();
        return;
    }
    else
    {
        printf("Prio calibration complete\n");
        bool cal;
        error = yostskel_isCalibrated(skel_id, &cal);
        if (cal)
        {
            printf("Skeleton is calibrated\n");
        }
    }
    printf("================================\n");

    // Sets up the standard streaming of data for the Prio suit
    // Runs the Prio Processor
    printf("====Start Streaming====\n");
    error = yostskel_setupStandardStreamPrioProcessor(prio_id);
    error = yostskel_startPrioProcessor(prio_id);

    if (error == YOST_SKELETON_NO_ERROR)
    {
        printf("Streaming has started!\n");
        printf("================================\n");

        // Loop through all bone names and return the orientation
        // Only bones active in the Tss layout will be returned
        float quat[16][4];
        float offset[16][4];
        float pos[16][3];
        float vel[16][3];
        float accel[16][3];
        uint8_t i;

        printf("\n====Getting Data from Skeleton====\n");
        int timer = 0;
        clock_t time;
        double deltaTime = 0;
        while (timer < 10)
        {
            time = clock();
            error = yostskel_update(skel_id);
            if (error == YOST_SKELETON_NO_ERROR)
            {
                for (i = 0; i < 2; i++)
                {
                    printf("====Bone: %s====\n", names[i]);
                    //Gets the orientation of a given bone
                    error = yostskel_getBoneOrientation(skel_id, names[i], quat[i]);
                    error = yostskel_getBoneOrientationOffset(skel_id, names[i], offset[i]);
                    error = yostskel_getBonePosition(skel_id, names[i], pos[i]);
                    error = yostskel_getBoneVelocity(skel_id, names[i], vel[i]);
                    error = yostskel_getBoneAcceleration(skel_id, names[i], accel[i]);
                    if (error == YOST_SKELETON_NO_ERROR)
                    {
                        printf("Quat: %f, %f, %f, %f\n", quat[i][0], quat[i][1], quat[i][2], quat[i][3]);
                        printf("Offset: %f, %f, %f, %f\n", offset[i][0], offset[i][1], offset[i][2], offset[i][3]);
                        printf("Position: %f, %f, %f\n", pos[i][0], pos[i][1], pos[i][2]);
                        printf("Velocity: %f, %f, %f\n", vel[i][0], vel[i][1], vel[i][2]);
                        printf("Accelerations: %f, %f, %f\n", accel[i][0], accel[i][1], accel[i][2]);
                    }
                    else
                    {
                        printf("ERROR: %s on names[%d] %s\n", yost_skeleton_error_string[error], i, names[i]);
                    }
                }
            }
            else
            {
                printf("ERROR: %s\n", yost_skeleton_error_string[error]);
            }

            bool hasUpdated = false;
            error = yostskel_hasPrioProcessorUpdatedState(prio_id, &hasUpdated);
            if (error == YOST_SKELETON_NO_ERROR)
            {
                printf("State changed: %d", hasUpdated);

                if (hasUpdated)
                {
                    error = yostskel_setSkeletonToStandardTPose(skel_id);
                    error = yostskel_calibratePrioProcessor(prio_id, 0);
                    error = yostskel_startPrioProcessor(prio_id);
                }
            }
            else
            {
                printf("ERROR in state has changed: %s\n", yost_skeleton_error_string[error]);
            }

            getchar();
            timer += 1;
            deltaTime += (clock() - time) / (CLOCKS_PER_SEC / 1000);
            printf("============DeltaTime: %f\n", (clock() - time));
        }
        deltaTime = deltaTime / 100;
        printf("=============Average time of 100000: %f=============\n", deltaTime);
        printf("================================\n");
        U8 battery_level = 0;
        error = yostskel_getHubBatteryLevel(prio_id, &battery_level);
        if (error == YOST_SKELETON_NO_ERROR)
        {
            printf("Battery Level: %d\n", battery_level);
        }
        else
        {
            printf("ERROR: %s\n", yost_skeleton_error_string[error]);
        }
        printf("====Stop Streaming====\n");
        error = yostskel_stopPrioProcessor(prio_id);
        if (error == YOST_SKELETON_NO_ERROR)
        {
            printf("Streaming has stopped!\n");
        }
        else
        {
            printf("ERROR: %s\n", yost_skeleton_error_string[error]);
        }

        battery_level = 255;
        error = yostskel_getHubBatteryLevel(prio_id, &battery_level);
        if (error == YOST_SKELETON_NO_ERROR)
        {
            printf("Battery Level: %d\n", battery_level);
        }
        else
        {
            printf("ERROR: %s\n", yost_skeleton_error_string[error]);
        }

        memset(quat, 0, sizeof(quat));
    }
    else
    {
        printf("ERROR: %s\n", yost_skeleton_error_string[error]);
    }
    printf("================================\n");
    memset(names, 0, sizeof(names));

    yostskel_startPrioProcessor(prio_id);

    yostskel_stopPrioProcessor(prio_id);

    yostskel_destroySkeleton(skel_id);

    // Reset API to clean up allocated memory
    printf("====Resetting API====\n");
    error = yostskel_resetSkeletalApi();
    if (error != YOST_SKELETON_NO_ERROR)
    {
        printf("ERROR: %s\n", yost_skeleton_error_string[error]);
    }

    return;
}

void prio_pro_hub_test()
{
    yost_skeleton_id skel_id;
    YOST_SKELETON_ERROR error;

    // Create a skeleton using the user's sex and age
    uint8_t is_male = 1;    // <-- Edit this to your user's sex
    uint32_t age = 23;      // <-- Edit this to your user's age

    printf("====Creating Skeleton for a %s, %d Years Old====\n", is_male ? "Male" : "Female", age);
    skel_id = yostskel_createStandardSkeletonWithAge(is_male, age);
    yostskel_loadBoneXMLHierarchy(skel_id, "BONETEST.xml");
    //skel_id = yostskel_createSkeletonFromFile("BONETEST.xml");
    if (skel_id == YOST_SKELETON_INVALID_ID)
    {
        printf("Failed to create a skeleton for a %s, %d years old\n", is_male ? "Male" : "Female", age);
    }
    else
    {
        printf("Created a skeleton for a %s, %d years old\n", is_male ? "Male" : "Female", age);
    }
    printf("================================\n");

    // Check Version String
    char versionString[64];
    error = yostskel_getVersionString(versionString, 64);
    printf("====Version String====\n");
    if (error != YOST_SKELETON_NO_ERROR)
    {
        printf("ERROR: %s\n", yost_skeleton_error_string[error]);
    }
    else
    {
        printf("%s\n", versionString);
    }

    // Sets the skeleton to the standard T Pose
    printf("====Setting a Pose for Skeleton====\n");
    error = yostskel_setSkeletonToStandardTPose(skel_id);
    if (error != YOST_SKELETON_NO_ERROR)
    {
        printf("ERROR: %s\n", yost_skeleton_error_string[error]);
    }
    else
    {
        printf("Skeleton is now in a \"T\" pose\n");
    }
    printf("================================\n");

    error = yostskel_setStandardDeviceXmlMapPrioProLayout(skel_id);


    // Create a Prio Processor
    printf("====Creating a Prio Device from Search====\n");
    prio_device_id hub_device;
    prio_ComPort port;
    port.port_name = new char[64];
    PRIO_ERROR prio_error;

    prio_findPorts(PRIO_HUB);

    if (prio_getPort(port.port_name, 0, &port.device_type) == PRIO_NO_ERROR)
    {
        prio_error = prio_createHub(port.port_name, &hub_device);

        if (error)
        {
            printf("Failed to create PrioVR on %s!\n", port.port_name);
            return;
        }
    }
    else
    {
        printf("Failed to get the port!\n");
        return;
    }
    printf("====Created a Hub====\n");

    printf("====Creating a Prio Processor====\n");
    yost_skeleton_id prio_id = yostskel_createPrioProcessorWithPrioVRHub(hub_device);
    if (prio_id == YOST_SKELETON_INVALID_ID)
    {
        printf("Failed to create Prio Processor\n");
        getchar();
        yostskel_destroyProcessor(prio_id);
        yostskel_destroySkeleton(skel_id);
        // Reset API to clean up allocated memory
        printf("====Resetting API====\n");
        error = yostskel_resetSkeletalApi();
        if (error != YOST_SKELETON_NO_ERROR)
        {
            printf("ERROR: %s\n", yost_skeleton_error_string[error]);
        }

        printf("\nFinished press Enter to continue");
        getchar();
        return;
    }
    else
    {
        printf("Created a Prio Processor %d\n", prio_id);
        //Get the processor id
        UINT32 procIndex;
        error = yostskel_getPrioProcessorDeviceIndex(prio_id, &procIndex);
        printf("Processor id: %d", procIndex);
    }
    printf("================================\n");

    // Add the Prio Proccesor to the internal skeleton
    // Adding a Processor to the skeleton allows the Processor to manipulate the bones of the skeleton
    printf("====Adding the Prio Processor to the Skeleton====\n");
    error = yostskel_addProcessorToSkeleton(skel_id, YOST_SKELETON_PROCESSOR_LIST_END, prio_id);
    if (error != YOST_SKELETON_NO_ERROR)
    {
        printf("ERROR: %s\n", yost_skeleton_error_string[error]);
    }
    else
    {
        printf("Added Prio Processor(%d) to Skeleton(%d)\n", prio_id, skel_id);
    }
    printf("================================\n");

    //Get processor list
    yost_skeleton_id processors[2];
    error = yostskel_getProcessorListFromSkeleton(skel_id, processors, 2);

    //Processor name lis
    char processNames[1][20];

    error = yostskel_getProcessorNameListFromSkeleton(skel_id, (char**)processNames, 1, 20);
    printf("Processors(%d) named: (%s)\n", processors[0], processNames[0]);

    //error = yostskel_removeProcessorFromSkeleton(skel_id, 0);

    if (error != YOST_SKELETON_NO_ERROR)
    {
        printf("ERROR: %s\n", yost_skeleton_error_string[error]);
    }
    else
    {
        printf("Added Prio Processor(%d) to Skeleton(%d)\n", prio_id, skel_id);
    }
    printf("================================\n");


    U8 battery_level = 255;
    error = yostskel_getHubBatteryLevel(prio_id, &battery_level);

    if (error != YOST_SKELETON_NO_ERROR)
    {
        printf("ERROR: %s\n", yost_skeleton_error_string[error]);
    }
    else
    {
        printf("Battery Level: %d\n", battery_level);
    }

    // Finding the root bone
    printf("====Setting up skeleton bones====\n");
    char name_buff[256];
    error = yostskel_getRootBoneName(skel_id, name_buff, 256);
    if (error != YOST_SKELETON_NO_ERROR)
    {
        printf("ERROR: %s\n", yost_skeleton_error_string[error]);
    }
    else
    {
        printf("Root Bone: %s\n", name_buff);
    }

    // Get the standard bone names and put them in a array
    // Doing this makes it easy to loop through all the bones needed
    char names[2][64];
    // The following should return YEI_SKELETON_NO_ERROR as long as the buffer size is large enough
    error = yostskel_getStandardBoneName(YOST_SKELETON_BONE_SPINE, names[0], 32);
    error = yostskel_getStandardBoneName(YOST_SKELETON_BONE_HEAD, names[1], 32);


    //Testing polling utility
    error = yostskel_utilityRegisterBone(skel_id, names[0]);
    error = yostskel_utilityRegisterBone(skel_id, names[1]);

    error = yostskel_utilityDeregisterBone(skel_id, names[1]);

    printf("================================\n");

    // Calibrating the Prio suit while establishing the Skeleton Bone Map
    // Pause to follow in exe
    printf("\nPress Enter to Start Calibration\n");
    getchar();
    yostskel_stopPrioProcessor(prio_id);
    printf("====Calibrating the Prio Suit====\n");
    error = yostskel_calibratePrioProcessor(prio_id, 0);
    if (error != YOST_SKELETON_NO_ERROR)
    {
        printf("ERROR: %s\n", yost_skeleton_error_string[error]);
        getchar();
        return;
    }
    else
    {
        printf("Prio calibration complete\n");
        bool cal;
        error = yostskel_isCalibrated(skel_id, &cal);
        if (cal)
        {
            printf("Skeleton is calibrated\n");
        }
    }
    printf("================================\n");

    // Sets up the standard streaming of data for the Prio suit
    // Runs the Prio Processor
    printf("====Start Streaming====\n");
    error = yostskel_setupStandardStreamPrioProcessor(prio_id);
    //error = yostskel_setupStandardStreamWithAccelerationPrioProcessor(prio_id);
    error = yostskel_startPrioProcessor(prio_id);

    if (error == YOST_SKELETON_NO_ERROR)
    {
        printf("Streaming has started!\n");
        printf("================================\n");

        // Loop through all bone names and return the orientation
        // Only bones active in the Tss layout will be returned
        float quat[16][4];
        float offset[16][4];
        float pos[16][3];
        float vel[16][3];
        float accel[16][3];
        uint8_t i;

        printf("\n====Getting Data from Skeleton====\n");
        int timer = 0;
        clock_t time;
        double deltaTime = 0;
        while (timer < 10)
        {
            time = clock();
            error = yostskel_update(skel_id);
            if (error == YOST_SKELETON_NO_ERROR)
            {
                for (i = 0; i < 2; i++)
                {
                    printf("====Bone: %s====\n", names[i]);
                    //Gets the orientation of a given bone
                    error = yostskel_getBoneOrientation(skel_id, names[i], quat[i]);
                    error = yostskel_getBoneOrientationOffset(skel_id, names[i], offset[i]);
                    error = yostskel_getBonePosition(skel_id, names[i], pos[i]);
                    error = yostskel_getBoneVelocity(skel_id, names[i], vel[i]);
                    error = yostskel_getBoneAcceleration(skel_id, names[i], accel[i]);
                    if (error == YOST_SKELETON_NO_ERROR)
                    {
                        printf("Quat: %f, %f, %f, %f\n", quat[i][0], quat[i][1], quat[i][2], quat[i][3]);
                        printf("Offset: %f, %f, %f, %f\n", offset[i][0], offset[i][1], offset[i][2], offset[i][3]);
                        printf("Position: %f, %f, %f\n", pos[i][0], pos[i][1], pos[i][2]);
                        printf("Velocity: %f, %f, %f\n", vel[i][0], vel[i][1], vel[i][2]);
                        printf("Accelerations: %f, %f, %f\n", accel[i][0], accel[i][1], accel[i][2]);
                    }
                    else
                    {
                        printf("ERROR: %s on names[%d] %s\n", yost_skeleton_error_string[error], i, names[i]);
                    }
                }
            }
            else
            {
                printf("ERROR: %s\n", yost_skeleton_error_string[error]);
            }

            bool hasUpdated = false;
            error = yostskel_hasPrioProcessorUpdatedState(prio_id, &hasUpdated);
            if (error == YOST_SKELETON_NO_ERROR)
            {
                printf("State changed: %d", hasUpdated);

                if (hasUpdated)
                {
                    error = yostskel_setSkeletonToStandardTPose(skel_id);
                    error = yostskel_calibratePrioProcessor(prio_id, 0);
                    error = yostskel_startPrioProcessor(prio_id);
                }
            }
            else
            {
                printf("ERROR in state has changed: %s\n", yost_skeleton_error_string[error]);
            }

            getchar();
            timer += 1;
            deltaTime += (clock() - time) / (CLOCKS_PER_SEC / 1000);
            printf("============DeltaTime: %f\n", (clock() - time));
        }
        deltaTime = deltaTime / 100;
        printf("=============Average time of 100000: %f=============\n", deltaTime);
        printf("================================\n");
        U8 battery_level = 0;
        error = yostskel_getHubBatteryLevel(prio_id, &battery_level);
        if (error == YOST_SKELETON_NO_ERROR)
        {
            printf("Battery Level: %d\n", battery_level);
        }
        else
        {
            printf("ERROR: %s\n", yost_skeleton_error_string[error]);
        }
        printf("====Stop Streaming====\n");
        error = yostskel_stopPrioProcessor(prio_id);
        if (error == YOST_SKELETON_NO_ERROR)
        {
            printf("Streaming has stopped!\n");
        }
        else
        {
            printf("ERROR: %s\n", yost_skeleton_error_string[error]);
        }

        battery_level = 255;
        error = yostskel_getHubBatteryLevel(prio_id, &battery_level);
        if (error == YOST_SKELETON_NO_ERROR)
        {
            printf("Battery Level: %d\n", battery_level);
        }
        else
        {
            printf("ERROR: %s\n", yost_skeleton_error_string[error]);
        }

        memset(quat, 0, sizeof(quat));
    }
    else
    {
        printf("ERROR: %s\n", yost_skeleton_error_string[error]);
    }
    printf("================================\n");
    memset(names, 0, sizeof(names));

    yostskel_startPrioProcessor(prio_id);

    yostskel_stopPrioProcessor(prio_id);

    yostskel_destroySkeleton(skel_id);

    // Reset API to clean up allocated memory
    printf("====Resetting API====\n");
    error = yostskel_resetSkeletalApi();
    if (error != YOST_SKELETON_NO_ERROR)
    {
        printf("ERROR: %s\n", yost_skeleton_error_string[error]);
    }

    return;
}

//RUN MAIN LOOP
int main()
{
    prio_pro_skeleton_test();
    //prio_pro_basestation_test();
    //prio_pro_hub_test();

    system("pause");
    return 0;
}
