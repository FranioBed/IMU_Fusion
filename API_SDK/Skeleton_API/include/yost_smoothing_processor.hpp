/********************************************//**
 * Copyright 1998-2014, YOST Labs Corporation
 * This Source Code Form is subject to the terms of the YEI 3-Space Open
 * Source License available online at:
 * http://www.yeitechnology.com/yei-3-space-open-source-license
 ***********************************************/
#ifndef _YOST_SMOOTHING_PROCESSOR_H_
#define _YOST_SMOOTHING_PROCESSOR_H_
#include "yost_skeleton_processor.hpp"
#include "yost_skeleton_api.h"
#include <cfloat>
#include <deque>
#include <math.h>

namespace yost
{
    // This class adds dynamic smoothing to the Skeleton
    // It tracks the variance of the bone's rotations and applies levels of smoothing according to the variance
    class Smoothing : public SkeletonProcessor
    {
    public:
        Smoothing();
        ~Smoothing();
        void runProcess();
        void startSmoothingBone(std::string bone_name);
        void stopSmoothingBone(std::string bone_name);

        void setAllMinSmoothingFactor(float min_smoothing_factor);
        void setAllMaxSmoothingFactor(float max_smoothing_factor);
        void setAllLowerVarianceBound(float lower_variance_bound);
        void setAllUpperVarianceBound(float upper_variance_bound);
        void setAllVarianceMultiplyFactor(float multiply_factor);
        void setAllVarianceDataLength(int variance_length);

        YOST_SKELETON_ERROR setBoneMinSmoothingFactor(std::string bone_name, float min_smoothing_factor);
        YOST_SKELETON_ERROR getBoneMinSmoothingFactor(std::string bone_name, float* min_smoothing_factor);
        YOST_SKELETON_ERROR setBoneMaxSmoothingFactor(std::string bone_name, float max_smoothing_factor);
        YOST_SKELETON_ERROR getBoneMaxSmoothingFactor(std::string bone_name, float* max_smoothing_factor);
        YOST_SKELETON_ERROR setBoneLowerVarianceBound(std::string bone_name, float lower_variance_bound);
        YOST_SKELETON_ERROR getBoneLowerVarianceBound(std::string bone_name, float* lower_variance_bound);
        YOST_SKELETON_ERROR setBoneUpperVarianceBound(std::string bone_name, float upper_variance_bound);
        YOST_SKELETON_ERROR getBoneUpperVarianceBound(std::string bone_name, float* upper_variance_bound);
        YOST_SKELETON_ERROR setBoneVarianceMultiplyFactor(std::string bone_name, float multiply_factor);
        YOST_SKELETON_ERROR getBoneVarianceMultiplyFactor(std::string bone_name, float* multiply_factor);
        YOST_SKELETON_ERROR setBoneVarianceDataLength(std::string bone_name, int variance_length);
        YOST_SKELETON_ERROR getBoneVarianceDataLength(std::string bone_name, int* variance_length);

    private:
        struct smooth_bone
        {
            int max_variance_data_length;
            float min_smoothing_factor;
            float max_smoothing_factor;
            float lower_variance_bound;
            float upper_variance_bound;
            float variance_multiply_factor;
        };

        void _runSmoothing();
        float _calculateVariance(std::deque<float> arr);

        Orient _slerp(Orient qa, Orient qb, double t);
        std::map<std::string, struct variances*> _bone_variance_deques;
        std::map<std::string, Orient> _old_quaternions;
        std::map<std::string, bool> _ped_bones;
        std::map<std::string, smooth_bone> _bone_options;
    };
};

#endif //_YOST_SMOOTHING_PROCESSOR_H_