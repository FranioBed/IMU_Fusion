/********************************************//**
* Copyright 1998-2014, Yost Labs Corporation
* This Source Code Form is subject to the terms of the YOST 3-Space Open
* Source License available online at:
* http://www.yeitechnology.com/yei-3-space-open-source-license
***********************************************/
#include "yost_pedestrian_tracking_processor.hpp"
#include "yost_skeleton_core_api.hpp"
#include <mutex>
#include <thread>
#include <fstream>

namespace yost
{
    struct variances
    {
        std::deque<float>* x;
        std::deque<float>* y;
        std::deque<float>* z;
        std::deque<float>* w;
    };

    static std::mutex io_mutex;

    PedestrianTracking::PedestrianTracking()
    {
        _type = PROCESSOR_TYPE_PED_TRACK;

        _anchor_point.data[0] = INFINITY;
        _anchor_point.data[1] = INFINITY;
        _anchor_point.data[2] = INFINITY;

        _max_variance_data_length = 50;
        _max_certainty = 1;
        _variance_multiply_factor = 4;
        _last_lowest_part = "";
        _last_lowest_part_variance = 0;

        _ped_bones["RightFoot"] = true;
        _ped_bones["LeftFoot"] = true;
        _ped_bones["LeftLowerLeg"] = true;
        _ped_bones["RightLowerLeg"] = true;
        _ped_bones["LeftUpperLeg"] = true;
        _ped_bones["RightUpperLeg"] = true;
        _ped_bones["Hips"] = true;
        _ped_bones["Spine2"] = true;
        _ped_bones["RightShoulder"] = true;
        _ped_bones["LeftShoulder"] = true;
        _ped_bones["RightUpperArm"] = true;
        _ped_bones["LeftUpperArm"] = true;
        _ped_bones["RightLowerArm"] = true;
        _ped_bones["LeftLowerArm"] = true;
        _ped_bones["RightHand"] = true;
        _ped_bones["LeftHand"] = true;

        _translation = Vector3(0, 0, 0);

        _root_bone = "Hips";
    }

    PedestrianTracking::~PedestrianTracking()
    {
    }


    void PedestrianTracking::setMaxCertainty(float certainty)
    {
        _max_certainty = certainty;
    }

    float PedestrianTracking::getMaxCertainty()
    {
        return _max_certainty;
    }

    void PedestrianTracking::setVarianceMultiplyFactor(float multiply_factor)
    {
        _variance_multiply_factor = multiply_factor;
    }

    float PedestrianTracking::getVarianceMultiplyFactor()
    {
        return _variance_multiply_factor;
    }

    void PedestrianTracking::setVarianceDataLength(int variance_length)
    {
        _max_variance_data_length = variance_length;
    }

    int PedestrianTracking::getVarianceDataLength()
    {
        return _max_variance_data_length;
    }

    void PedestrianTracking::startTrackingBone(std::string bone_name)
    {
        _ped_bones[bone_name] = true;
    }

    void PedestrianTracking::stopTrackingBone(std::string bone_name)
    {
        _ped_bones.erase(bone_name);
    }

    void PedestrianTracking::setGroundHeight(float height)
    {
        _ground_height = height;
    }

    void PedestrianTracking::setRootBone(std::string bone_name)
    {
        _root_bone = bone_name;
    }

    void PedestrianTracking::resetPosition()
    {
        _skeleton->getRootBone()->setPosition(_root_bone_init);
        _last_lowest_part = "";
        _anchor_point = Vector3(0, INFINITY , 0);
    }

    Vector3 PedestrianTracking::getSkeletonTranslation()
    {
        Vector3 tmp = _translation;
        _translation = Vector3(0, 0, 0);
        return tmp;
    }

    Vector3 PedestrianTracking::getSkeletonTranslationAsPrecentOfHeight()
    {
        Vector3 tmp = _translation_by_height_precentage;
        _translation_by_height_precentage = Vector3(0, 0, 0);
        return tmp;
    }

    PedestrianTracking::pinnedBoneData PedestrianTracking::getPinnedBoneData()
    {
        pinnedBoneData pinned_bone_data = pinnedBoneData{ _anchor_point, _last_lowest_part };
        return pinned_bone_data;
    }

    void PedestrianTracking::runProcess()
    {
        bool islocked = io_mutex.try_lock();
        if (islocked)
        {
            int root_bone_index = -1;
            std::shared_ptr<Bone> root_bone = _skeleton->getRootBone();
            if (root_bone == nullptr)
            {
                return;
            }

            YOST_SKELETON_UNIT units = _skeleton->getSkeletonUnit();
            float toCentimeters = 1;
            if (units == YOST_SKELETON_UNIT_METERS)
            {
                toCentimeters = 100;
            }
            else if (units == YOST_SKELETON_UNIT_INCHES)
            {
                toCentimeters = 2.54f;
            }
            else if (units == YOST_SKELETON_UNIT_FEET)
            {
                toCentimeters = 30.48f;
            }

            std::vector<std::shared_ptr<Bone>> bones = _skeleton->getBoneList();
            Vector3 pos;
            float min_y = FLT_MAX;
            U32 lowest_bone = 0;

            float lowestPointBuffer = _max_certainty - (_last_lowest_part_variance*_variance_multiply_factor);

            for (int i = 0; i < bones.size(); i++)
            {
                std::string boneName = bones[i]->getName();
                if (_ped_bones.find(boneName) != _ped_bones.end())
                {
                    if (strcmp(boneName.c_str(), _root_bone.c_str()) == 0)
                    {
                        root_bone_index = i;
                    }
                    if (_bone_variance_deques.find(boneName) == _bone_variance_deques.end())//first run for this bone
                    {
                        _bone_variance_deques[boneName] = new struct variances;
                        _bone_variance_deques[boneName]->x = new std::deque<float>;
                        _bone_variance_deques[boneName]->y = new std::deque<float>;
                        _bone_variance_deques[boneName]->z = new std::deque<float>;
                        _bone_variance_deques[boneName]->w = new std::deque<float>;
                    }

                    Orient tmpOrient = bones[i]->getOrientation();
                    (*_bone_variance_deques[boneName]->x).push_back(tmpOrient.data[QUAT_X]);
                    (*_bone_variance_deques[boneName]->y).push_back(tmpOrient.data[QUAT_Y]);
                    (*_bone_variance_deques[boneName]->z).push_back(tmpOrient.data[QUAT_Z]);
                    (*_bone_variance_deques[boneName]->w).push_back(tmpOrient.data[QUAT_W]);

                    while ((*_bone_variance_deques[boneName]->x).size() > _max_variance_data_length || (*_bone_variance_deques[boneName]->y).size() > _max_variance_data_length || (*_bone_variance_deques[boneName]->z).size() > _max_variance_data_length || (*_bone_variance_deques[boneName]->w).size() > _max_variance_data_length)
                    {
                        (*_bone_variance_deques[boneName]->x).pop_front();
                        (*_bone_variance_deques[boneName]->y).pop_front();
                        (*_bone_variance_deques[boneName]->z).pop_front();
                        (*_bone_variance_deques[boneName]->w).pop_front();
                    }

                    float varianceX = _calculateVariance(*_bone_variance_deques[boneName]->x);
                    float varianceY = _calculateVariance(*_bone_variance_deques[boneName]->y);
                    float varianceZ = _calculateVariance(*_bone_variance_deques[boneName]->z);
                    float varianceW = _calculateVariance(*_bone_variance_deques[boneName]->w);

                    float totalVariance = varianceX + varianceY + varianceZ + varianceW;
                    pos = bones[i]->getPosition();

                    pos.data[0] *= toCentimeters;
                    pos.data[1] *= toCentimeters;
                    pos.data[2] *= toCentimeters;

                    if (pos.data[1] < min_y - lowestPointBuffer - totalVariance * _variance_multiply_factor / 4 * _max_certainty)
                    {
                        lowest_bone = i;
                        min_y = pos.data[1];
                        _last_lowest_part_variance = totalVariance;
                    }
                }
            }

            if (strcmp(_last_lowest_part.c_str(), bones[lowest_bone]->getName().c_str()) != 0)
            {
                Vector3 tmpVect = bones[lowest_bone]->getPosition();
                tmpVect.data[0] *= toCentimeters;
                tmpVect.data[1] *= toCentimeters;
                tmpVect.data[2] *= toCentimeters;

                _anchor_point.data[0] = tmpVect.data[0];
                _anchor_point.data[1] = tmpVect.data[1];
                _anchor_point.data[2] = tmpVect.data[2];

                _last_lowest_part_variance = 0;

                auto bvdIt = _bone_variance_deques.begin();
                while (bvdIt != _bone_variance_deques.end())
                {
                    bvdIt->second->x->clear();
                    bvdIt->second->y->clear();
                    bvdIt->second->z->clear();
                    bvdIt->second->w->clear();
                    bvdIt++;
                }
                _last_lowest_part = bones[lowest_bone]->getName();
            }

            if (_anchor_point.data[1] == INFINITY)
            {
                _root_bone_init = _skeleton->getRootBone()->getPosition();
                Vector3 tmpVect = bones[lowest_bone]->getPosition();

                tmpVect.data[0] *= toCentimeters;
                tmpVect.data[1] *= toCentimeters;
                tmpVect.data[2] *= toCentimeters;

                _anchor_point.data[0] = tmpVect.data[0];
                _anchor_point.data[1] = tmpVect.data[1];
                _anchor_point.data[2] = tmpVect.data[2];
            }
            Vector3 tmpVect = bones[lowest_bone]->getPosition();

            tmpVect.data[0] *= toCentimeters;
            tmpVect.data[1] *= toCentimeters;
            tmpVect.data[2] *= toCentimeters;

            Vector3 moveDirection;
            moveDirection.data[0] = _anchor_point.data[0] - tmpVect.data[0];
            moveDirection.data[1] = 0;
            moveDirection.data[2] = _anchor_point.data[2] - tmpVect.data[2];
            std::shared_ptr<Bone> rootBone;
            if (root_bone_index < 0)
            {
                rootBone = _skeleton->getRootBone();
            }
            else
            {
                rootBone = bones[root_bone_index];
            }
            Vector3 rootVect = rootBone->getPosition();

            Vector3 finalTranslation;
            finalTranslation.data[0] = moveDirection.data[0];
            finalTranslation.data[1] = -min_y;
            finalTranslation.data[2] = moveDirection.data[2];

            finalTranslation.data[0] /= toCentimeters;
            finalTranslation.data[1] /= toCentimeters;
            finalTranslation.data[2] /= toCentimeters;

            _skeleton->utilityTranslate(finalTranslation);

            _translation = Vector3(_translation.data[0] + finalTranslation.data[0], _translation.data[1] + finalTranslation.data[1], _translation.data[2] + finalTranslation.data[2]);
            _translation_by_height_precentage = Vector3((_translation.data[0] + finalTranslation.data[0]) / _skeleton->getHeight(), (_translation.data[1] + finalTranslation.data[1]) / _skeleton->getHeight(), (_translation.data[2] + finalTranslation.data[2]) / _skeleton->getHeight());

            io_mutex.unlock();
        }

    }

    float PedestrianTracking::_calculateVariance(std::deque<float> arr)
    {

        if (arr.size() == 1){
            return 0;
        }
        std::vector<float> tmpArr;
        for (int i = 0; i < arr.size(); i++)
        {
            tmpArr.push_back(arr[i]);
        }

        float mean = 0;
        for (int i = 0; i < tmpArr.size(); i++)
        {
            mean += tmpArr[i];
        }
        mean /= tmpArr.size();

        float squareSum = 0;
        for (int i = 0; i < tmpArr.size(); i++)
        {
            tmpArr[i] -= mean;
            tmpArr[i] *= tmpArr[i];
            squareSum += tmpArr[i];
        }

        float variance = 0;
        if (squareSum > 0)
        {
            variance = std::sqrtf(squareSum / (tmpArr.size() - 1));
        }
        return variance;
    }

};