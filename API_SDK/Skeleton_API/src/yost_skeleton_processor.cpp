/********************************************//**
 * Copyright 1998-2014, Yost Labs Corporation
 * This Source Code Form is subject to the terms of the Yost 3-Space Open
 * Source License available online at:
 * http://www.yeitechnology.com/yei-3-space-open-source-license
 ***********************************************/
#include "yost_skeleton_processor.hpp"

namespace yost
{
    SkeletonProcessor::SkeletonProcessor()
    {
    }

    SkeletonProcessor::~SkeletonProcessor()
    {
    }

    void SkeletonProcessor::setSkeleton(Skeleton* skeleton)
    {
        _skeleton = skeleton;
    }

    Skeleton* SkeletonProcessor::getSkeleton()
    {
        return _skeleton;
    }

    PROCESSOR_TYPE SkeletonProcessor::getProcessorType()
    {
        return _type;
    }

    std::string SkeletonProcessor::getProcessorTypeString()
    {
        return processor_type_string[_type];
    }
};
