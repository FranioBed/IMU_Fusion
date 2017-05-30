/********************************************//**
* Copyright 1998-2014, YOST Labs Corporation
* This Source Code Form is subject to the terms of the YEI 3-Space Open
* Source License available online at:
* http://www.yeitechnology.com/yei-3-space-open-source-license
***********************************************/
#ifndef _YOST_PEDESTRIAN_TRACKING_PROCESSOR_H_
#define _YOST_PEDESTRIAN_TRACKING_PROCESSOR_H_
#include "yost_skeleton_processor.hpp"
#include <cfloat>
#include <deque>
#include <math.h>

namespace yost
{
    // This class adds pedestrian tracking capability to the Skeleton
    // It tracks the user's center of mass and determines which foot to pin down
    class PedestrianTracking : public SkeletonProcessor
    {
    public:
        PedestrianTracking();
        ~PedestrianTracking();
        void runProcess();

        void setGroundHeight(float height);
        void startTrackingBone(std::string bone_name);
        void stopTrackingBone(std::string bone_name);
        void setMaxCertainty(float certainty);
        float getMaxCertainty();
        void setVarianceMultiplyFactor(float multiply_factor);
        float getVarianceMultiplyFactor();
        void setVarianceDataLength(int variance_length);
        int getVarianceDataLength();
        void setRootBone(std::string bone_name);

        void resetPosition();

        Vector3 getSkeletonTranslation();
        Vector3 getSkeletonTranslationAsPrecentOfHeight();

        struct pinnedBoneData
        {
            Vector3 anchor_point;
            std::string pinned_bone_name;
        };

        pinnedBoneData getPinnedBoneData();

    private:
        float _calculateVariance(std::deque<float> arr);

        std::map<std::string, struct variances* > _bone_variance_deques;
        std::map<std::string, bool > _ped_bones;
        Vector3 _root_bone_init;
        Vector3 _anchor_point;
        Vector3 _translation;
        Vector3 _translation_by_height_precentage;
        std::string _last_lowest_part;
        float _ground_height;
        int _max_variance_data_length;
        float _max_certainty;
        float _variance_multiply_factor;
        float _last_lowest_part_variance;
        std::string _root_bone;
    };
};
#endif //_YOST_PEDESTRIAN_TRACKING_PROCESSOR_H_