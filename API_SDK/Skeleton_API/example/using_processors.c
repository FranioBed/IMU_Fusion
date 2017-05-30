/********************************************//**
 * This file should not be included for compiling the Skeleton API.
 * This file is mainly an example for various functionality of the API.
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
#include "yei_skeleton_api.h"

int main()
{
    yei_skeleton_id skel_id;

    // Create a skeleton using the user's sex and age
    uint8_t is_male = 0;    // <-- Edit this to your user's sex
    uint32_t age = 34;      // <-- Edit this to your user's age
    printf("====Creating Skeleton for a %s, %d Years Old====\n", is_male ? "Male" : "Female", age);
    skel_id = yeiskel_createStandardSkeletonWithAge(is_male, age);
    if (skel_id == YEI_SKELETON_INVALID_ID)
    {
        printf("Failed to create a skeleton for a %s, %d years old\n", is_male ? "Male" : "Female", age);
    }
    else
    {
        printf("Created a skeleton for a %s, %d years old\n", is_male ? "Male" : "Female", age);
    }
    printf("================================\n");

    // Sets the skeleton to the standard T Pose
    printf("====Setting a Pose for Skeleton====\n");
    YEI_SKELETON_ERROR error = yeiskel_setSkeletonToStandardTPose(skel_id);
    if (error != YEI_SKELETON_NO_ERROR)
    {
        printf("ERROR: %s\n", yei_skeleton_error_string[error]);
    }
    else
    {
        printf("Skeleton is now in a \"T\" pose\n");
    }
    printf("================================\n");

    // Create a Prio Processor
    printf("====Creating a Prio Processor====\n");
    yei_skeleton_id prio_id = yeiskel_createPrioProcessor(0);
    if (prio_id == YEI_SKELETON_INVALID_ID)
    {
        printf("Failed to create Prio Processor\n");
    }
    else
    {
        printf("Created a Prio Processor\n");
    }
    printf("================================\n");
    
    // Add the Prio Proccesor to the internal skeleton
    // Adding a Processor to the skeleton allows the Processor to manipulate the bones of the skeleton
    printf("====Adding the Prio Processor to the Skeleton====\n");
    error = yeiskel_addProcessorToSkeleton(skel_id, YEI_SKELETON_PROCESSOR_LIST_END, prio_id);
    if (error != YEI_SKELETON_NO_ERROR)
    {
        printf("ERROR: %s\n", yei_skeleton_error_string[error]);
    }
    else
    {
        printf("Added Prio Processor(%d) to Skeleton(%d)\n", prio_id, skel_id);
    }
    printf("================================\n");

    // The standard Prio Layout uses the YEI Skeleton standard bone names
    printf("====Mapping the Skeleton to Match the Prio Lite Suit====\n");
    error = yeiskel_setStandardDeviceXmlMapPrioLiteLayout(skel_id);
    if (error != YEI_SKELETON_NO_ERROR)
    {
        printf("ERROR: %s\n", yei_skeleton_error_string[error]);
    }
    else
    {
        printf("Skeleton set up for Prio Lite suit\n");
    }
    printf("================================\n");

    // Finding the root bone
    printf("====Setting up skeleton bones====\n");
    char name_buff[256];
    error = yeiskel_getRootBoneName(skel_id, name_buff, 256);
    if (error != YEI_SKELETON_NO_ERROR)
    {
        printf("ERROR: %s\n", yei_skeleton_error_string[error]);
    }
    else
    {
        printf("Root Bone: %s\n", name_buff);
    }

    // Get the standard bone names and put them in a array
    // Doing this makes it easy to loop through all the bones needed
    printf("Getting standard bone names for Prio Lite suit\n");
    char names[9][32];
    // The following should return YEI_SKELETON_NO_ERROR as long as the buffer size is large enough
    error = yeiskel_getStandardBoneName(YEI_SKELETON_STANDARD_BONE_HIPS, names[0], 32);
    error = yeiskel_getStandardBoneName(YEI_SKELETON_STANDARD_BONE_SPINE, names[1], 32);
    error = yeiskel_getStandardBoneName(YEI_SKELETON_STANDARD_BONE_HEAD, names[2], 32);
    error = yeiskel_getStandardBoneName(YEI_SKELETON_STANDARD_BONE_LEFT_UPPER_ARM, names[3], 32);
    error = yeiskel_getStandardBoneName(YEI_SKELETON_STANDARD_BONE_LEFT_LOWER_ARM, names[4], 32);
    error = yeiskel_getStandardBoneName(YEI_SKELETON_STANDARD_BONE_LEFT_HAND, names[5], 32);
    error = yeiskel_getStandardBoneName(YEI_SKELETON_STANDARD_BONE_RIGHT_UPPER_ARM, names[6], 32);
    error = yeiskel_getStandardBoneName(YEI_SKELETON_STANDARD_BONE_RIGHT_LOWER_ARM, names[7], 32);
    error = yeiskel_getStandardBoneName(YEI_SKELETON_STANDARD_BONE_RIGHT_HAND, names[8], 32);
    printf("================================\n");

    // Calibrating the Prio suit while establishing the Skeleton Bone Map
    // Pause to follow in exe
    printf("\nPress Enter to Start Calibration\n");
    getchar();
    printf("====Calibrating the Prio Suit====\n");
    error = yeiskel_calibratePrioProcessor(prio_id, 0.0f);
    if (error != YEI_SKELETON_NO_ERROR)
    {
        printf("ERROR: %s\n", yei_skeleton_error_string[error]);
    }
    else
    {
        printf("Prio calibration complete\n");
    }
    printf("================================\n");

    // Sets up the standard streaming of data for the Prio suit
    printf("====Setting up the Standard Prio Streaming====\n");
    error = yeiskel_setStandardStreamPrioProcessor(prio_id);
    if (error != YEI_SKELETON_NO_ERROR)
    {
        printf("ERROR: %s\n", yei_skeleton_error_string[error]);
    }
    else
    {
        printf("Streaming is ready\n");
    }
    printf("================================\n");
    // Pause to follow in exe
    printf("\nPress Enter to Start\n");
    getchar();


    // Runs the Prio Processor
    // This connects to the Prio base station and starts streaming data
    printf("====Start Streaming====\n");
    error = yeiskel_startPrioProcessor(prio_id);
    if (error == YEI_SKELETON_NO_ERROR)
    {
        printf("Streaming has started!\n");
        printf("================================\n");

        // Loop through all bone names and return the orientation
        // Only bones active in the Prio layout will be returned
        float quat[9][4];
        float vect3[9][3];
        const float RUN_TIME = 10.0f;    // in seconds
        uint8_t i;
        clock_t start_time = clock();
        printf("\n====Getting Data from Skeleton====\n");
        while ((((float)(clock() - start_time)) / CLOCKS_PER_SEC < RUN_TIME))
        {
            error = yeiskel_update(skel_id);
            if (error == YEI_SKELETON_NO_ERROR)
            {
                for (i = 0; i < 9; i++)
                {
                    printf("====Bone: %s====\n", names[i]);
                    //Gets the orientation of a given bone
                    error = yeiskel_getBoneOrientation(skel_id, names[i], quat[i]);
                    if (error == YEI_SKELETON_NO_ERROR)
                    {
                        printf("Quat: %f, %f, %f, %f\n", quat[i][0], quat[i][1], quat[i][2], quat[i][3]);
                    }
                    else
                    {
                        printf("ERROR: %s\n", yei_skeleton_error_string[error]);
                    }
                    error = yeiskel_getBoneVelocity(skel_id, names[i], vect3[i]);
                    if (error == YEI_SKELETON_NO_ERROR)
                    {
                        printf("Pos: %f, %f, %f\n", vect3[i][0], vect3[i][1], vect3[i][2]);
                    }
                    else
                    {
                        printf("ERROR: %s\n", yei_skeleton_error_string[error]);
                    }
                }
            }
            else
            {
                printf("ERROR: %s\n", yei_skeleton_error_string[error]);
            }
        }
        printf("================================\n");

        // Tells the Prio Processor to stop, stopping the stream data
        printf("====Stop Streaming====\n");
        error = yeiskel_stopPrioProcessor(prio_id);
        if (error == YEI_SKELETON_NO_ERROR)
        {
            printf("Streaming has stopped!\n");
        }
        else
        {
            printf("ERROR: %s\n", yei_skeleton_error_string[error]);
        }
    }
    else
    {
        printf("ERROR: %s\n", yei_skeleton_error_string[error]);
    }
    printf("================================\n");

    // Destroy the Skeleton to clean up any allocated memory and clean up any attached processors
    yeiskel_destroySkeleton(skel_id);

    // Reset API to clean up allocated memory
    printf("====Resetting API====\n");
    error = yeiskel_resetSkeletalApi();
    if (error != YEI_SKELETON_NO_ERROR)
    {
        printf("ERROR: %s\n", yei_skeleton_error_string[error]);
    }

    printf("\nFinished press Enter to continue");
    getchar();
    return 0;
}
