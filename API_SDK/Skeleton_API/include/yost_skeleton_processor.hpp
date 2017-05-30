/********************************************//**
 * Copyright 1998-2014, Yost Labs Corporation
 * This Source Code Form is subject to the terms of the YOST 3-Space Open
 * Source License available online at:
 * http://www.yeitechnology.com/yei-3-space-open-source-license
 ***********************************************/
#ifndef _YOST_SKELETON_PROCESSOR_H_
#define _YOST_SKELETON_PROCESSOR_H_
#include <stdint.h>
#include <map>
#include <vector>
#include <memory>
#include <string>
#include <thread>
#include <chrono>

#include "yost_math.hpp"
#include "rapidxml/rapidxml.hpp"

namespace yost
{
    // This is here to help with circular dependency
    class Skeleton;
    class Bone;

    typedef enum PROCESSOR_TYPE
    {
        PROCESSOR_TYPE_PRIO         = 0,
        PROCESSOR_TYPE_PED_TRACK    = 1,
        PROCESSOR_TYPE_SMOOTHING    = 2,
        PROCESSOR_TYPE_TSS          = 3
    }PROCESSOR_TYPE;

    static const char* const processor_type_string[] =
    {
        "PROCESSOR_TYPE_PRIO",
        "PROCESSOR_TYPE_PED_TRACK",
        "PROCESSOR_TYPE_SMOOTHING",
        "PROCESSOR_TYPE_TSS_MOCAP"
    };

    // This is the base class for any objects that add extra functionality to a Skeleton.
    class SkeletonProcessor
    {
    public:
        SkeletonProcessor();
        virtual ~SkeletonProcessor();

        // This is where this processor does all its work.
        virtual void runProcess() = 0;

        void setSkeleton(Skeleton* skeleton);
        Skeleton* getSkeleton();
        PROCESSOR_TYPE getProcessorType();
        std::string getProcessorTypeString();

    protected:
        Skeleton* _skeleton;
        PROCESSOR_TYPE _type;
    };
};

#endif //_YOST_SKELETON_PROCESSOR_H_
