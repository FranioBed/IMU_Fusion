/********************************************//**
 * This example shows how to create skeleton and processor instances.
 * When an instance of a skeleton or processor is created in the API a yei_skeleton_id is returned.
 * This yei_skeleton ID is needed to call many functions in the API.
 *
 * Copyright 1998-2014, YEI Corporation
 * This Source Code Form is subject to the terms of the YEI 3-Space Open
 * Source License available online at:
 * http://www.yeitechnology.com/yei-3-space-open-source-license
 ***********************************************/
#include <stdio.h>
#include <stdlib.h>
#include "yei_skeleton_api.h"

int main()
{
    yei_skeleton_id skel_id;

    // Create a skeleton using the user's sex and age
    uint8_t is_male = 1;    // <-- Edit this to your user's sex
    uint32_t age = 26;      // <-- Edit this to your user's age
    printf("====Creating Skeleton for a %s, %d Years Old====\n", is_male ? "Male" : "Female", age);
    skel_id = yeiskel_createStandardSkeletonWithAge(is_male, age);
    if (skel_id == YEI_SKELETON_INVALID_ID)
    {
        printf("Failed to create a skeleton for a %s, %d years old\n", is_male ? "Male" : "Female", age);
    }
    else
    {
        printf("Created a skeleton for a %s, %d years old\n", is_male ? "Male" : "Female", age);
        yeiskel_destroySkeleton(skel_id);
    }
    printf("================================\n");

    // Create a skeleton using the user's sex and height (in meters)
    float height = 1.8f;    // <-- Edit this to your user's height
    printf("====Creating Skeleton for a %s, %f Meters Tall====\n", is_male ? "Male" : "Female", height);
    skel_id = yeiskel_createStandardSkeletonWithHeight(is_male, height);
    if (skel_id == YEI_SKELETON_INVALID_ID)
    {
        printf("Failed to create a skeleton for a %s, %f meters tall\n", is_male ? "Male" : "Female", height);
    }
    else
    {
        printf("Created a skeleton for a %s, %f meters tall\n", is_male ? "Male" : "Female", height);
        yeiskel_destroySkeleton(skel_id);
    }
    printf("================================\n");

    // Create a skeleton using previously created skeleton hierarchy (.xml) file
    const char* xml_file = "./YEI_Skeleton_Hierarchy.xml";  // <-- Edit this to your user's profile file
    printf("====Creating Skeleton from File====\n");
    skel_id = yeiskel_createSkeletonFromFile(xml_file);
    if (skel_id == YEI_SKELETON_INVALID_ID)
    {
        printf("Failed to create a skeleton from file %s\n", xml_file);
    }
    else
    {
        printf("Created a skeleton from file %s\n", xml_file);
        yeiskel_destroySkeleton(skel_id);
    }
    printf("================================\n");

    // Call reset to clean up the Prio API.
    YEI_SKELETON_ERROR error = yeiskel_resetSkeletalApi();
    if (error != YEI_SKELETON_NO_ERROR)
    {
        printf("ERROR: %s\n", yei_skeleton_error_string[error]);
    }

    printf("\nFinished press Enter to continue");
    getchar();
    return 0;
}
