/********************************************//**
 * Copyright 1998-2014, Yost Labs Corporation
 * This Source Code Form is subject to the terms of the YOST 3-Space Open
 * Source License available online at:
 * http://www.yeitechnology.com/yei-3-space-open-source-license
 ***********************************************/
#include "yost_skeleton_api.h"
#include "yost_skeleton_core_api.hpp"
#include "yost_connection_processor.hpp"
#include "yost_pedestrian_tracking_processor.hpp"
#include "yost_smoothing_processor.hpp"
#include "yost_tss_connection_processor.hpp"
#include "prio_api_export.h"
#include "threespace_api_export.h"

extern "C"
{
    // API Specific Methods
    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_getVersionString(char* buffer, uint32_t buffer_len)
    {
        std::string version = yost::getVersion();
        if (version.size() > buffer_len)
        {
            return YOST_SKELETON_ERROR_BUFFER_TOO_SMALL;
        }

        memcpy(buffer, version.c_str(), version.size());

        return YOST_SKELETON_NO_ERROR;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_resetSkeletalApi()
    {
        PRIO_ERROR error = prio_deinitAPI();

        if (error != PRIO_NO_ERROR)
        {
            return YOST_SKELETON_ERROR_RESET_API;
        }

        result = tss_deinitAPI();

        if (result)
        {
            return YOST_SKELETON_ERROR_RESET_API;
        }
        yost::stored_processors.clear();

        yost::stored_skeletons.clear();

        return YOST_SKELETON_NO_ERROR;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_setAxisDirections(int8_t* order, int8_t* negate)
    {
        yost::setAxisDirections(order, negate);
        return YOST_SKELETON_NO_ERROR;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_getStandardBoneName(YOST_SKELETON_BONE bone, char* bone_name, uint32_t name_len)
    {
        std::string name = yost::getStandardBoneName(bone);
        if (name_len >= name.size())
        {
            memcpy(bone_name, name.c_str(), name_len);
            return YOST_SKELETON_NO_ERROR;
        }
        return YOST_SKELETON_ERROR_BUFFER_TOO_SMALL;
    }

    // Skeleton Methods
    SKEL_EXPORT yost_skeleton_id yostskel_createStandardSkeletonWithAge(uint8_t male, uint32_t age)
    {
        if (age == 0)
        {
            return (yost_skeleton_id)YOST_SKELETON_INVALID_ID;
        }
        std::unique_ptr<yost::Skeleton> new_skel(new yost::Skeleton());
        new_skel->createStandardSkeletonHierarchyWithAge(male ? true : false, age);
        yost::stored_skeletons.emplace(yost::stored_skeletons.end(), new_skel.release());
        return (yost_skeleton_id)(YOST_SKELETON_ID + yost::stored_skeletons.size() - 1);
    }

    SKEL_EXPORT yost_skeleton_id yostskel_createStandardSkeletonWithHeight(uint8_t male, float height)
    {
        if (height <= 0)
        {
            return (yost_skeleton_id)YOST_SKELETON_INVALID_ID;
        }

        std::unique_ptr<yost::Skeleton> new_skel(new yost::Skeleton());
        new_skel->createStandardSkeletonHierarchy(male ? true : false, height);
        yost::stored_skeletons.emplace(yost::stored_skeletons.end(), new_skel.release());
        return (yost_skeleton_id)(YOST_SKELETON_ID + yost::stored_skeletons.size() - 1);
    }

    SKEL_EXPORT yost_skeleton_id yostskel_createSkeletonFromFile(const char* hierarchy_file)
    {
        std::unique_ptr<yost::Skeleton> new_skel(new yost::Skeleton());
        YOST_SKELETON_ERROR error = new_skel->loadSkeletonHierarchy(hierarchy_file);
        if (error != YOST_SKELETON_NO_ERROR)
        {
            return YOST_SKELETON_INVALID_ID;
        }
        yost::stored_skeletons.emplace(yost::stored_skeletons.end(), new_skel.release());
        return (yost_skeleton_id)(YOST_SKELETON_ID + yost::stored_skeletons.size() - 1);
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_destroySkeleton(yost_skeleton_id skel_id)
    {
        if (skel_id >= YOST_SKELETON_ID && skel_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID))
        {
            uint32_t skel_idx = ~(~skel_id | YOST_SKELETON_ID);
            if (skel_idx < yost::stored_skeletons.size())
            {
                yost::stored_skeletons[skel_idx].reset();
                return YOST_SKELETON_NO_ERROR;
            }
        }
        return YOST_SKELETON_ERROR_SKELETON_NOT_FOUND;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_setSkeletonToStandardTPose(yost_skeleton_id skel_id)
    {
        if (skel_id >= YOST_SKELETON_ID && skel_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID))
        {
            uint32_t skel_idx = ~(~skel_id | YOST_SKELETON_ID);
            if (skel_idx < yost::stored_skeletons.size())
            {
                yost::stored_skeletons[skel_idx]->setToStandardTPose();
                return YOST_SKELETON_NO_ERROR;
            }
        }
        return YOST_SKELETON_ERROR_SKELETON_NOT_FOUND;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_setSkeletonToStandardClaspedPose(yost_skeleton_id skel_id)
    {
        if (skel_id >= YOST_SKELETON_ID && skel_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID))
        {
            uint32_t skel_idx = ~(~skel_id | YOST_SKELETON_ID);
            if (skel_idx < yost::stored_skeletons.size())
            {
                yost::stored_skeletons[skel_idx]->setToStandardClaspedPose();
                return YOST_SKELETON_NO_ERROR;
            }
        }
        return YOST_SKELETON_ERROR_SKELETON_NOT_FOUND;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_setSkeletonToStandardNeutralPose(yost_skeleton_id skel_id)
    {
        if (skel_id >= YOST_SKELETON_ID && skel_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID))
        {
            uint32_t skel_idx = ~(~skel_id | YOST_SKELETON_ID);
            if (skel_idx < yost::stored_skeletons.size())
            {
                yost::stored_skeletons[skel_idx]->setToStandardNeutralPose();
                return YOST_SKELETON_NO_ERROR;
            }
        }
        return YOST_SKELETON_ERROR_SKELETON_NOT_FOUND;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_addBoneAlias(yost_skeleton_id skel_id, const char* your_bone_name, const char* bone_name)
    {
        if (skel_id >= YOST_SKELETON_ID && skel_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID))
        {
            uint32_t skel_idx = ~(~skel_id | YOST_SKELETON_ID);
            if (skel_idx < yost::stored_skeletons.size())
            {
                return yost::stored_skeletons[skel_idx]->addBoneAlias(your_bone_name, bone_name);
            }
        }
        return YOST_SKELETON_ERROR_SKELETON_NOT_FOUND;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_addProcessorToSkeleton(yost_skeleton_id skel_id, uint32_t index, yost_skeleton_id proc_id)
    {
        if (skel_id >= YOST_SKELETON_ID && skel_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID))
        {
            uint32_t skel_idx = ~(~skel_id | YOST_SKELETON_ID);
            if (skel_idx < yost::stored_skeletons.size())
            {
                if (proc_id >= YOST_SKELETON_PROCESSOR_ID && skel_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID | YOST_SKELETON_PROCESSOR_ID))
                {
                    uint32_t proc_idx = ~(~proc_id | YOST_SKELETON_PROCESSOR_ID);
                    if (proc_idx < yost::stored_processors.size())
                    {
                        return yost::stored_skeletons[skel_idx]->addProcessor(index, proc_id);
                    }
                }
            }
            return YOST_SKELETON_ERROR_PROCESSOR_NOT_FOUND;
        }
        return YOST_SKELETON_ERROR_SKELETON_NOT_FOUND;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_removeProcessorFromSkeleton(yost_skeleton_id skel_id, uint32_t index)
    {
        if (skel_id >= YOST_SKELETON_ID && skel_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID))
        {
            uint32_t skel_idx = ~(~skel_id | YOST_SKELETON_ID);
            if (skel_idx < yost::stored_skeletons.size())
            {
                return yost::stored_skeletons[skel_idx]->removeProcessor(index);
            }
        }
        return YOST_SKELETON_ERROR_SKELETON_NOT_FOUND;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_getProcessorListFromSkeleton(yost_skeleton_id skel_id, yost_skeleton_id* processor_list, uint32_t list_len)
    {
        if (skel_id >= YOST_SKELETON_ID && skel_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID))
        {
            uint32_t skel_idx = ~(~skel_id | YOST_SKELETON_ID);
            if (skel_idx < yost::stored_skeletons.size())
            {
                std::deque<yost_skeleton_id> proc_list = yost::stored_skeletons[skel_idx]->getProcessorList();
                if (list_len >= proc_list.size())
                {
                    uint32_t i = 0;
                    for (auto proc : proc_list)
                    {
                        processor_list[i] = proc;
                        i++;
                    }
                    return YOST_SKELETON_NO_ERROR;
                }
                return YOST_SKELETON_ERROR_BUFFER_TOO_SMALL;
            }
        }
        return YOST_SKELETON_ERROR_SKELETON_NOT_FOUND;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_getProcessorNameListFromSkeleton(yost_skeleton_id skel_id, char** processor_list, uint32_t list_len, uint32_t name_len)
    {
        if (skel_id >= YOST_SKELETON_ID && skel_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID))
        {
            uint32_t skel_idx = ~(~skel_id | YOST_SKELETON_ID);
            if (skel_idx < yost::stored_skeletons.size())
            {
                std::vector<std::string> proc_list = yost::stored_skeletons[skel_idx]->getProcessorNameList();
                if (list_len >= proc_list.size())
                {
                    uint32_t i = 0;
                    for (auto proc : proc_list)
                    {
                        if (name_len < proc.size())
                        {
                            return YOST_SKELETON_ERROR_BUFFER_TOO_SMALL;
                        }
                        char** tmp = &processor_list[i];
                        tmp = (char**)((char*)tmp + (name_len - 8)*i);
                        memcpy(tmp, proc.c_str(), proc.size());
                        i++;
                    }
                    return YOST_SKELETON_NO_ERROR;
                }
                return YOST_SKELETON_ERROR_BUFFER_TOO_SMALL;
            }
        }
        return YOST_SKELETON_ERROR_SKELETON_NOT_FOUND;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_update(yost_skeleton_id skel_id)
    {
        if (skel_id >= YOST_SKELETON_ID && skel_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID))
        {
            uint32_t skel_idx = ~(~skel_id | YOST_SKELETON_ID);
            if (skel_idx < yost::stored_skeletons.size())
            {
                return yost::stored_skeletons[skel_idx]->update();
            }
        }
        return YOST_SKELETON_ERROR_SKELETON_NOT_FOUND;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_hasUpdated(yost_skeleton_id skel_id, bool* has_changed)
    {
        if (skel_id >= YOST_SKELETON_ID && skel_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID))
        {
            uint32_t skel_idx = ~(~skel_id | YOST_SKELETON_ID);
            if (skel_idx < yost::stored_skeletons.size())
            {
                *has_changed = yost::stored_skeletons[skel_idx]->hasUpdated();
                return YOST_SKELETON_NO_ERROR;
            }
        }
        return YOST_SKELETON_ERROR_SKELETON_NOT_FOUND;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_getRootBoneName(yost_skeleton_id skel_id, char* bone_name, uint32_t name_len)
    {
        if (skel_id >= YOST_SKELETON_ID && skel_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID))
        {
            uint32_t skel_idx = ~(~skel_id | YOST_SKELETON_ID);
            if (skel_idx < yost::stored_skeletons.size())
            {
                std::string root_name = yost::stored_skeletons[skel_idx]->getRootBoneName();
                if (name_len >= root_name.size())
                {
                    memcpy(bone_name, root_name.c_str(), root_name.size());
                    return YOST_SKELETON_NO_ERROR;
                }
                return YOST_SKELETON_ERROR_BUFFER_TOO_SMALL;
            }
        }
        return YOST_SKELETON_ERROR_SKELETON_NOT_FOUND;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_getBoneNameList(yost_skeleton_id skel_id, char** bone_list, uint32_t list_len, uint32_t name_len)
    {
        if (skel_id >= YOST_SKELETON_ID && skel_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID))
        {
            uint32_t skel_idx = ~(~skel_id | YOST_SKELETON_ID);
            if (skel_idx < yost::stored_skeletons.size())
            {
                std::vector<std::string> name_list = yost::stored_skeletons[skel_idx]->getBoneNameList();
                if (list_len >= name_list.size())
                {
                    uint32_t i = 0;
                    for (auto name : name_list)
                    {
                        if (name_len < name.size())
                        {
                            return YOST_SKELETON_ERROR_BUFFER_TOO_SMALL;
                        }

                        char* tmp2 = bone_list[i];
                        memcpy(tmp2, name.c_str(), name.size());
                        i++;
                    }
                    return YOST_SKELETON_NO_ERROR;
                }
                return YOST_SKELETON_ERROR_BUFFER_TOO_SMALL;
            }
        }
        return YOST_SKELETON_ERROR_SKELETON_NOT_FOUND;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_setBoneOrientationOffset(yost_skeleton_id skel_id, const char* bone_name, float* quat4)
    {
        if (skel_id >= YOST_SKELETON_ID && skel_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID))
        {
            uint32_t skel_idx = ~(~skel_id | YOST_SKELETON_ID);
            if (skel_idx < yost::stored_skeletons.size())
            {
                Orient orientation;
                yost::_floatArrayToOrient(quat4, &orientation);
                
                YOST_SKELETON_ERROR error = yost::stored_skeletons[skel_idx]->setBoneOrientationOffset(bone_name, orientation);
                return error;
            }
        }
        return YOST_SKELETON_ERROR_SKELETON_NOT_FOUND;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_getBoneOrientationOffset(yost_skeleton_id skel_id, const char* bone_name, float* quat4)
    {
        if (skel_id >= YOST_SKELETON_ID && skel_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID))
        {
            uint32_t skel_idx = ~(~skel_id | YOST_SKELETON_ID);
            if (skel_idx < yost::stored_skeletons.size())
            {
                Orient orientation;
                YOST_SKELETON_ERROR error = yost::stored_skeletons[skel_idx]->getBoneOrientationOffset(bone_name, &orientation);
                if (error == YOST_SKELETON_NO_ERROR)
                {
                    yost::_orientToFloatArray(orientation, quat4);
                }
                return error;
            }
        }
        return YOST_SKELETON_ERROR_SKELETON_NOT_FOUND;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_getBoneOrientation(yost_skeleton_id skel_id, const char* bone_name, float* quat4)
    {
        if (skel_id >= YOST_SKELETON_ID && skel_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID))
        {
            uint32_t skel_idx = ~(~skel_id | YOST_SKELETON_ID);
            if (skel_idx < yost::stored_skeletons.size())
            {
                Orient orientation;
                YOST_SKELETON_ERROR error = yost::stored_skeletons[skel_idx]->getBoneOrientation(bone_name, &orientation);
                
                if (error == YOST_SKELETON_NO_ERROR)
                {
                    yost::_orientToFloatArray(orientation, quat4);
                }

                return error;
            }
        }
        return YOST_SKELETON_ERROR_SKELETON_NOT_FOUND;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_getBonePosition(yost_skeleton_id skel_id, const char* bone_name, float* pos3)
    {
        if (skel_id >= YOST_SKELETON_ID && skel_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID))
        {
            uint32_t skel_idx = ~(~skel_id | YOST_SKELETON_ID);
            if (skel_idx < yost::stored_skeletons.size())
            {
                Vector3 vec;
                YOST_SKELETON_ERROR error = yost::stored_skeletons[skel_idx]->getBonePosition(bone_name, &vec);
                if (error == YOST_SKELETON_NO_ERROR)
                {
                    yost::_vector3ToFloatArray(vec, pos3);
                }
                return error;
            }
        }
        return YOST_SKELETON_ERROR_SKELETON_NOT_FOUND;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_getBoneVelocity(yost_skeleton_id skel_id, const char* bone_name, float* vel3)
    {
        if (skel_id >= YOST_SKELETON_ID && skel_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID))
        {
            uint32_t skel_idx = ~(~skel_id | YOST_SKELETON_ID);
            if (skel_idx < yost::stored_skeletons.size())
            {
                Vector3 vel;
                YOST_SKELETON_ERROR error = yost::stored_skeletons[skel_idx]->getBoneVelocity(bone_name, &vel);
                if (error == YOST_SKELETON_NO_ERROR)
                {
                    yost::_vector3ToFloatArray(vel, vel3);
                }
                return error;
            }
        }
        return YOST_SKELETON_ERROR_SKELETON_NOT_FOUND;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_getBoneAcceleration(yost_skeleton_id skel_id, const char* bone_name, float* accel3)
    {
        if (skel_id >= YOST_SKELETON_ID && skel_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID))
        {
            uint32_t skel_idx = ~(~skel_id | YOST_SKELETON_ID);
            if (skel_idx < yost::stored_skeletons.size())
            {
                Vector3 accel;
                YOST_SKELETON_ERROR error = yost::stored_skeletons[skel_idx]->getBoneAcceleration(bone_name, &accel);
                if (error == YOST_SKELETON_NO_ERROR)
                {
                    yost::_vector3ToFloatArray(accel, accel3);
                }
                return error;
            }
        }
        return YOST_SKELETON_ERROR_SKELETON_NOT_FOUND;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_getAllBoneOrientationOffset(yost_skeleton_id skel_id, float* quat4)
    {
        if (skel_id >= YOST_SKELETON_ID && skel_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID))
        {
            uint32_t skel_idx = ~(~skel_id | YOST_SKELETON_ID);
            if (skel_idx < yost::stored_skeletons.size())
            {
                return yost::stored_skeletons[skel_idx]->getAllBoneOrientationOffsets(quat4);
            }
        }
        return YOST_SKELETON_ERROR_SKELETON_NOT_FOUND;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_getAllBoneOrientations(yost_skeleton_id skel_id, float* quat4)
    {
        if (skel_id >= YOST_SKELETON_ID && skel_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID))
        {
            uint32_t skel_idx = ~(~skel_id | YOST_SKELETON_ID);
            if (skel_idx < yost::stored_skeletons.size())
            {
                return yost::stored_skeletons[skel_idx]->getAllBoneOrientations(quat4);
            }
        }
        return YOST_SKELETON_ERROR_SKELETON_NOT_FOUND;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_getAllBonePositions(yost_skeleton_id skel_id, float* pos3)
    {
        if (skel_id >= YOST_SKELETON_ID && skel_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID))
        {
            uint32_t skel_idx = ~(~skel_id | YOST_SKELETON_ID);
            if (skel_idx < yost::stored_skeletons.size())
            {
                return yost::stored_skeletons[skel_idx]->getAllBonePositions(pos3);
            }
        }
        return YOST_SKELETON_ERROR_SKELETON_NOT_FOUND;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_getAllBoneVelocities(yost_skeleton_id skel_id, float* vel3)
    {
        if (skel_id >= YOST_SKELETON_ID && skel_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID))
        {
            uint32_t skel_idx = ~(~skel_id | YOST_SKELETON_ID);
            if (skel_idx < yost::stored_skeletons.size())
            {
                return yost::stored_skeletons[skel_idx]->getAllBoneVelocitites(vel3);
            }
        }
        return YOST_SKELETON_ERROR_SKELETON_NOT_FOUND;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_getAllBoneAccelerations(yost_skeleton_id skel_id, float* accel3)
    {
        if (skel_id >= YOST_SKELETON_ID && skel_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID))
        {
            uint32_t skel_idx = ~(~skel_id | YOST_SKELETON_ID);
            if (skel_idx < yost::stored_skeletons.size())
            {
                return yost::stored_skeletons[skel_idx]->getAllBoneAccelerations(accel3);
            }
        }
        return YOST_SKELETON_ERROR_SKELETON_NOT_FOUND;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_setBonePositionOffset(yost_skeleton_id skel_id, const char* bone_name, float* pos_offset3)
    {
        if (skel_id >= YOST_SKELETON_ID && skel_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID))
        {
            uint32_t skel_idx = ~(~skel_id | YOST_SKELETON_ID);
            if (skel_idx < yost::stored_skeletons.size())
            {
                Vector3 pos_offset;
                yost::_floatArrayToVector3(pos_offset3, &pos_offset);
                YOST_SKELETON_ERROR error = yost::stored_skeletons[skel_idx]->setBonePositionOffset(bone_name, pos_offset);

                return error;
            }
        }
        return YOST_SKELETON_ERROR_SKELETON_NOT_FOUND;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_getBonePositionOffset(yost_skeleton_id skel_id, const char* bone_name, float* pos_offset3)
    {
        if (skel_id >= YOST_SKELETON_ID && skel_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID))
        {
            uint32_t skel_idx = ~(~skel_id | YOST_SKELETON_ID);
            if (skel_idx < yost::stored_skeletons.size())
            {
                Vector3 pos_offset;
                YOST_SKELETON_ERROR error = yost::stored_skeletons[skel_idx]->getBonePositionOffset(bone_name, &pos_offset);
                if (error == YOST_SKELETON_NO_ERROR)
                {
                    yost::_vector3ToFloatArray(pos_offset, pos_offset3);
                }
                return error;
            }
        }
        return YOST_SKELETON_ERROR_SKELETON_NOT_FOUND;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_extractSkeletonHierarchy(yost_skeleton_id skel_id, char* buffer, uint32_t buffer_len)
    {
        if (skel_id >= YOST_SKELETON_ID && skel_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID))
        {
            uint32_t skel_idx = ~(~skel_id | YOST_SKELETON_ID);
            if (skel_idx < yost::stored_skeletons.size())
            {
                std::string skel_hierarchy = yost::stored_skeletons[skel_idx]->extractHierarchy();
                if (buffer_len >= skel_hierarchy.size())
                {
                    memcpy(buffer, skel_hierarchy.c_str(), buffer_len);
                    return YOST_SKELETON_NO_ERROR;
                }
                return YOST_SKELETON_ERROR_BUFFER_TOO_SMALL;
            }
        }
        return YOST_SKELETON_ERROR_SKELETON_NOT_FOUND;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_isCalibrated(yost_skeleton_id skel_id, bool* calibrated)
    {
        if (skel_id >= YOST_SKELETON_ID && skel_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID))
        {
            uint32_t skel_idx = ~(~skel_id | YOST_SKELETON_ID);
            if (skel_idx < yost::stored_skeletons.size())
            {
                *calibrated = yost::stored_skeletons[skel_idx]->isCalibrated();
                return YOST_SKELETON_NO_ERROR;
            }
        }
        return YOST_SKELETON_ERROR_SKELETON_NOT_FOUND;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_getSkeletonName(yost_skeleton_id skel_id, char* skel_name, uint32_t name_len)
    {
        if (skel_id >= YOST_SKELETON_ID && skel_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID))
        {
            uint32_t skel_idx = ~(~skel_id | YOST_SKELETON_ID);
            if (skel_idx < yost::stored_skeletons.size())
            {
                std::string name = yost::stored_skeletons[skel_idx]->getName();
                if (name_len >= name.size())
                {
                    memcpy(skel_name, name.c_str(), name_len);
                    return YOST_SKELETON_NO_ERROR;
                }
                return YOST_SKELETON_ERROR_BUFFER_TOO_SMALL;
            }
        }
        return YOST_SKELETON_ERROR_SKELETON_NOT_FOUND;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_setSkeletonName(yost_skeleton_id skel_id, const char* skel_name)
    {
        if (skel_id >= YOST_SKELETON_ID && skel_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID))
        {
            uint32_t skel_idx = ~(~skel_id | YOST_SKELETON_ID);
            if (skel_idx < yost::stored_skeletons.size())
            {
                if (strlen(skel_name) == 0)
                {
                    return YOST_SKELETON_ERROR_EMPTY_NAME;
                }
                yost::stored_skeletons[skel_idx]->setName(skel_name);
                return YOST_SKELETON_NO_ERROR;
            }
        }
        return YOST_SKELETON_ERROR_SKELETON_NOT_FOUND;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_getSkeletonUnit(yost_skeleton_id skel_id, YOST_SKELETON_UNIT* skel_unit)
    {
        if (skel_id >= YOST_SKELETON_ID && skel_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID))
        {
            uint32_t skel_idx = ~(~skel_id | YOST_SKELETON_ID);
            if (skel_idx < yost::stored_skeletons.size())
            {
                *skel_unit = yost::stored_skeletons[skel_idx]->getSkeletonUnit();
                return YOST_SKELETON_NO_ERROR;
            }
        }
        return YOST_SKELETON_ERROR_SKELETON_NOT_FOUND;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_setSkeletonUnit(yost_skeleton_id skel_id, YOST_SKELETON_UNIT skel_unit)
    {
        if (skel_id >= YOST_SKELETON_ID && skel_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID))
        {
            uint32_t skel_idx = ~(~skel_id | YOST_SKELETON_ID);
            if (skel_idx < yost::stored_skeletons.size())
            {
                yost::stored_skeletons[skel_idx]->setSkeletonUnit(skel_unit);
                return YOST_SKELETON_NO_ERROR;
            }
        }
        return YOST_SKELETON_ERROR_SKELETON_NOT_FOUND;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_addBoneToSkeleton(yost_skeleton_id skel_id, const char* bone_name, YOST_SKELETON_BONE_UPDATE_TYPE update_type, float* offset3, const char* parent_name)
    {
        if (skel_id >= YOST_SKELETON_ID && skel_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID))
        {
            uint32_t skel_idx = ~(~skel_id | YOST_SKELETON_ID);
            if (skel_idx < yost::stored_skeletons.size())
            {
                Vector3 offset;
                yost::_floatArrayToVector3(offset3, &offset);
                return yost::stored_skeletons[skel_idx]->utilityAddBone(bone_name, update_type, offset, parent_name);
            }
        }
        return YOST_SKELETON_ERROR_SKELETON_NOT_FOUND;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_removeBoneFromSkeleton(yost_skeleton_id skel_id, const char* bone_name)
    {
        if (skel_id >= YOST_SKELETON_ID && skel_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID))
        {
            uint32_t skel_idx = ~(~skel_id | YOST_SKELETON_ID);
            if (skel_idx < yost::stored_skeletons.size())
            {
                return yost::stored_skeletons[skel_idx]->utilityRemoveBone(bone_name);
            }
        }
        return YOST_SKELETON_ERROR_SKELETON_NOT_FOUND;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_utilityRegisterBone(yost_skeleton_id skel_id, const char* bone_name)
    {
        if (skel_id >= YOST_SKELETON_ID && skel_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID))
        {
            uint32_t skel_idx = ~(~skel_id | YOST_SKELETON_ID);
            if (skel_idx < yost::stored_skeletons.size())
            {
                return yost::stored_skeletons[skel_idx]->utilityRegisterBone(bone_name);
            }
        }
        return YOST_SKELETON_ERROR_SKELETON_NOT_FOUND;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_utilityDeregisterBone(yost_skeleton_id skel_id, const char* bone_name)
    {
        if (skel_id >= YOST_SKELETON_ID && skel_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID))
        {
            uint32_t skel_idx = ~(~skel_id | YOST_SKELETON_ID);
            if (skel_idx < yost::stored_skeletons.size())
            {
                return yost::stored_skeletons[skel_idx]->utilityDeregisterBone(bone_name);
            }
        }
        return YOST_SKELETON_ERROR_SKELETON_NOT_FOUND;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_utilityRegisteredBoneList(yost_skeleton_id skel_id, char** bone_list, uint32_t list_len, uint32_t name_len)
    {
        if (skel_id >= YOST_SKELETON_ID && skel_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID))
        {
            uint32_t skel_idx = ~(~skel_id | YOST_SKELETON_ID);
            if (skel_idx < yost::stored_skeletons.size())
            {
                std::vector<std::string> name_list = yost::stored_skeletons[skel_idx]->utilityRegisteredBoneList();
                if (list_len >= name_list.size())
                {
                    uint32_t i = 0;
                    for (auto name : name_list)
                    {
                        if (name_len < name.size())
                        {
                            return YOST_SKELETON_ERROR_BUFFER_TOO_SMALL;
                        }

                        char* tmp2 = bone_list[i];
                        memcpy(tmp2, name.c_str(), name.size());
                        i++;
                    }
                    return YOST_SKELETON_NO_ERROR;
                }
                return YOST_SKELETON_ERROR_BUFFER_TOO_SMALL;
            }
        }
        return YOST_SKELETON_ERROR_SKELETON_NOT_FOUND;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_utilityRegisteredBoneCount(yost_skeleton_id skel_id, uint32_t* bone_count)
    {
        if (skel_id >= YOST_SKELETON_ID && skel_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID))
        {
            uint32_t skel_idx = ~(~skel_id | YOST_SKELETON_ID);
            if (skel_idx < yost::stored_skeletons.size())
            {
                *bone_count =  yost::stored_skeletons[skel_idx]->utilityRegisteredBoneCount();
                return YOST_SKELETON_NO_ERROR;
            }
        }
        return YOST_SKELETON_ERROR_SKELETON_NOT_FOUND;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_setBonePoseOrientation(yost_skeleton_id skel_id, const char* bone_name, float* quat4)
    {
        if (skel_id >= YOST_SKELETON_ID && skel_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID))
        {
            uint32_t skel_idx = ~(~skel_id | YOST_SKELETON_ID);
            if (skel_idx < yost::stored_skeletons.size())
            {
                Orient pose;
                yost::_floatArrayToOrient(quat4, &pose);
                return yost::stored_skeletons[skel_idx]->utilitySetBonePoseOrientation(bone_name, pose);
            }
        }
        return YOST_SKELETON_ERROR_SKELETON_NOT_FOUND;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_setBoneLength(yost_skeleton_id skel_id, const char* bone_name, float length)
    {
        if (skel_id >= YOST_SKELETON_ID && skel_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID))
        {
            uint32_t skel_idx = ~(~skel_id | YOST_SKELETON_ID);
            if (skel_idx < yost::stored_skeletons.size())
            {
                return yost::stored_skeletons[skel_idx]->utilitySetBoneLength(bone_name, length);
            }
        }
        return YOST_SKELETON_ERROR_SKELETON_NOT_FOUND;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_getBoneLength(yost_skeleton_id skel_id, const char* bone_name, float* length)
    {
        if (skel_id >= YOST_SKELETON_ID && skel_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID))
        {
            uint32_t skel_idx = ~(~skel_id | YOST_SKELETON_ID);
            if (skel_idx < yost::stored_skeletons.size())
            {
                return yost::stored_skeletons[skel_idx]->utilityGetBoneLength(bone_name, length);
            }
        }
        return YOST_SKELETON_ERROR_SKELETON_NOT_FOUND;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_setBoneMass(yost_skeleton_id skel_id, const char* bone_name, float mass)
    {
        if (skel_id >= YOST_SKELETON_ID && skel_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID))
        {
            uint32_t skel_idx = ~(~skel_id | YOST_SKELETON_ID);
            if (skel_idx < yost::stored_skeletons.size())
            {
                return yost::stored_skeletons[skel_idx]->utilitySetBoneMass(bone_name, mass);
            }
        }
        return YOST_SKELETON_ERROR_SKELETON_NOT_FOUND;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_getBoneMass(yost_skeleton_id skel_id, const char* bone_name, float* mass)
    {
        if (skel_id >= YOST_SKELETON_ID && skel_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID))
        {
            uint32_t skel_idx = ~(~skel_id | YOST_SKELETON_ID);
            if (skel_idx < yost::stored_skeletons.size())
            {
                return yost::stored_skeletons[skel_idx]->utilityGetBoneMass(bone_name, mass);
            }
        }
        return YOST_SKELETON_ERROR_SKELETON_NOT_FOUND;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_setControllerUpdateRate(yost_skeleton_id prio_proc_id, U32 update_rate)
    {
        if (prio_proc_id >= YOST_SKELETON_PROCESSOR_ID && prio_proc_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID | YOST_SKELETON_PROCESSOR_ID))
        {
            uint32_t proc_idx = ~(~prio_proc_id | YOST_SKELETON_PROCESSOR_ID);
            if (proc_idx < yost::stored_processors.size())
            {
                yost::SkeletonProcessor* proc = yost::stored_processors[proc_idx].get();
                if (proc->getProcessorType() == yost::PROCESSOR_TYPE_PRIO)
                {
                    yost::PrioConnection* prio_proc = static_cast<yost::PrioConnection*>(proc);
                    prio_proc->setControllerUpdateRate(update_rate);
                    return YOST_SKELETON_NO_ERROR;
                }
            }
        }
        return YOST_SKELETON_ERROR_PROCESSOR_NOT_FOUND;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_getControllerUpdateRate(yost_skeleton_id prio_proc_id, U32* update_rate)
    {
        if (prio_proc_id >= YOST_SKELETON_PROCESSOR_ID && prio_proc_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID | YOST_SKELETON_PROCESSOR_ID))
        {
            uint32_t proc_idx = ~(~prio_proc_id | YOST_SKELETON_PROCESSOR_ID);
            if (proc_idx < yost::stored_processors.size())
            {
                yost::SkeletonProcessor* proc = yost::stored_processors[proc_idx].get();
                if (proc->getProcessorType() == yost::PROCESSOR_TYPE_PRIO)
                {
                    yost::PrioConnection* prio_proc = static_cast<yost::PrioConnection*>(proc);
                     prio_proc->getControllerUpdateRate(update_rate);
                    return YOST_SKELETON_NO_ERROR;
                }
            }
        }
        return YOST_SKELETON_ERROR_PROCESSOR_NOT_FOUND;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_startControllerUpdating(yost_skeleton_id prio_proc_id)
    {
        if (prio_proc_id >= YOST_SKELETON_PROCESSOR_ID && prio_proc_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID | YOST_SKELETON_PROCESSOR_ID))
        {
            uint32_t proc_idx = ~(~prio_proc_id | YOST_SKELETON_PROCESSOR_ID);
            if (proc_idx < yost::stored_processors.size())
            {
                yost::SkeletonProcessor* proc = yost::stored_processors[proc_idx].get();
                if (proc->getProcessorType() == yost::PROCESSOR_TYPE_PRIO)
                {
                    yost::PrioConnection* prio_proc = static_cast<yost::PrioConnection*>(proc);
                    prio_proc->startControllerUpdating();
                    return YOST_SKELETON_NO_ERROR;
                }
            }
        }
        return YOST_SKELETON_ERROR_PROCESSOR_NOT_FOUND;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_stopControllerUpdating(yost_skeleton_id prio_proc_id)
    {
        if (prio_proc_id >= YOST_SKELETON_PROCESSOR_ID && prio_proc_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID | YOST_SKELETON_PROCESSOR_ID))
        {
            uint32_t proc_idx = ~(~prio_proc_id | YOST_SKELETON_PROCESSOR_ID);
            if (proc_idx < yost::stored_processors.size())
            {
                yost::SkeletonProcessor* proc = yost::stored_processors[proc_idx].get();
                if (proc->getProcessorType() == yost::PROCESSOR_TYPE_PRIO)
                {
                    yost::PrioConnection* prio_proc = static_cast<yost::PrioConnection*>(proc);
                    prio_proc->stopControllerUpdating();
                    return YOST_SKELETON_NO_ERROR;
                }
            }
        }
        return YOST_SKELETON_ERROR_PROCESSOR_NOT_FOUND;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_loadDeviceXMLMap(yost_skeleton_id skel_id, const char* device_xml_file)
    {
        if (skel_id >= YOST_SKELETON_ID && skel_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID))
        {
            uint32_t skel_idx = ~(~skel_id | YOST_SKELETON_ID);
            if (skel_idx < yost::stored_skeletons.size())
            {
                return yost::stored_skeletons[skel_idx]->utilityLoadDeviceXmlMap(device_xml_file);
            }
        }
        return YOST_SKELETON_ERROR_SKELETON_NOT_FOUND;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_saveDeviceXMLMap(yost_skeleton_id skel_id, const char* device_xml_file)
    {
        if (skel_id >= YOST_SKELETON_ID && skel_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID))
        {
            uint32_t skel_idx = ~(~skel_id | YOST_SKELETON_ID);
            if (skel_idx < yost::stored_skeletons.size())
            {
                return yost::stored_skeletons[skel_idx]->utilitySaveDeviceXmlMap(device_xml_file);
            }
        }
        return YOST_SKELETON_ERROR_SKELETON_NOT_FOUND;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_loadBoneXMLHierarchy(yost_skeleton_id skel_id, const char* hierarchy_file)
    {
        if (skel_id >= YOST_SKELETON_ID && skel_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID))
        {
            uint32_t skel_idx = ~(~skel_id | YOST_SKELETON_ID);
            if (skel_idx < yost::stored_skeletons.size())
            {
                return yost::stored_skeletons[skel_idx]->loadSkeletonHierarchy(hierarchy_file);
            }
        }
        return YOST_SKELETON_ERROR_SKELETON_NOT_FOUND;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_saveBoneXMLHierarchy(yost_skeleton_id skel_id, const char* file_name)
    {
        if (skel_id >= YOST_SKELETON_ID && skel_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID))
        {
            uint32_t skel_idx = ~(~skel_id | YOST_SKELETON_ID);
            if (skel_idx < yost::stored_skeletons.size())
            {
                yost::stored_skeletons[skel_idx]->utilitySaveXMLHierarchy(file_name);
            }
        }
        return YOST_SKELETON_ERROR_SKELETON_NOT_FOUND;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_setStandardDeviceXmlMapPrioLiteLayout(yost_skeleton_id skel_id)
    {
        if (skel_id >= YOST_SKELETON_ID && skel_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID))
        {
            uint32_t skel_idx = ~(~skel_id | YOST_SKELETON_ID);
            if (skel_idx < yost::stored_skeletons.size())
            {
                std::string device_xml = (
                    "<yost>\n"
                    "    <sensors SUIT_LAYOUT=\"Prio_Lite\">\n"
                    "        <chest       SERIAL=\"0\" BONE=\"" + yost::getStandardBoneName(YOST_SKELETON_BONE_SPINE) + "\" />\n"
                    "        <head        SERIAL=\"1\" BONE=\"" + yost::getStandardBoneName(YOST_SKELETON_BONE_HEAD) + "\" />\n"
                    "        <l_upper_arm SERIAL=\"7\" BONE=\"" + yost::getStandardBoneName(YOST_SKELETON_BONE_LEFT_UPPER_ARM) + "\" />\n"
                    "        <l_lower_arm SERIAL=\"8\" BONE=\"" + yost::getStandardBoneName(YOST_SKELETON_BONE_LEFT_LOWER_ARM) + "\" />\n"
                    "        <l_hand      SERIAL=\"9\" BONE=\"" + yost::getStandardBoneName(YOST_SKELETON_BONE_LEFT_HAND) + "\" />\n"
                    "        <l_joystick  SERIAL=\"A\" BONE=\"" + yost::getStandardBoneName(YOST_SKELETON_BONE_LEFT_JOYSTICK) + "\" />\n"
                    "        <r_upper_arm SERIAL=\"E\" BONE=\"" + yost::getStandardBoneName(YOST_SKELETON_BONE_RIGHT_UPPER_ARM) + "\" />\n"
                    "        <r_lower_arm SERIAL=\"F\" BONE=\"" + yost::getStandardBoneName(YOST_SKELETON_BONE_RIGHT_LOWER_ARM) + "\" />\n"
                    "        <r_hand      SERIAL=\"10\" BONE=\"" + yost::getStandardBoneName(YOST_SKELETON_BONE_RIGHT_HAND) + "\" />\n"
                    "        <r_joystick  SERIAL=\"11\" BONE=\"" + yost::getStandardBoneName(YOST_SKELETON_BONE_RIGHT_JOYSTICK) + "\" />\n"
                    "    </sensors>\n"
                    "</yost>"
                    );
                yost::stored_skeletons[skel_idx]->utilitySetDeviceXmlMap(device_xml);
                return YOST_SKELETON_NO_ERROR;
            }
        }
        return YOST_SKELETON_ERROR_SKELETON_NOT_FOUND;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_setStandardDeviceXmlMapPrioCoreLayout(yost_skeleton_id skel_id)
    {
        if (skel_id >= YOST_SKELETON_ID && skel_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID))
        {
            uint32_t skel_idx = ~(~skel_id | YOST_SKELETON_ID);
            if (skel_idx < yost::stored_skeletons.size())
            {
                std::string device_xml = (
                    "<yost>\n"
                    "    <sensors SUIT_LAYOUT=\"Prio_Core\">\n"
                    "        <chest       SERIAL=\"0\" BONE=\"" + yost::getStandardBoneName(YOST_SKELETON_BONE_SPINE) + "\" />\n"
                    "        <head        SERIAL=\"1\" BONE=\"" + yost::getStandardBoneName(YOST_SKELETON_BONE_HEAD) + "\" />\n"
                    "        <l_upper_leg SERIAL=\"1C\" BONE=\"" + yost::getStandardBoneName(YOST_SKELETON_BONE_LEFT_UPPER_LEG) + "\" />\n"
                    "        <l_lower_leg SERIAL=\"1D\" BONE=\"" + yost::getStandardBoneName(YOST_SKELETON_BONE_LEFT_LOWER_LEG) + "\" />\n"
                    "        <l_upper_arm SERIAL=\"7\" BONE=\"" + yost::getStandardBoneName(YOST_SKELETON_BONE_LEFT_UPPER_ARM) + "\" />\n"
                    "        <l_lower_arm SERIAL=\"8\" BONE=\"" + yost::getStandardBoneName(YOST_SKELETON_BONE_LEFT_LOWER_ARM) + "\" />\n"
                    "        <l_hand      SERIAL=\"9\" BONE=\"" + yost::getStandardBoneName(YOST_SKELETON_BONE_LEFT_HAND) + "\" />\n"
                    "        <l_joystick  SERIAL=\"A\" BONE=\"" + yost::getStandardBoneName(YOST_SKELETON_BONE_LEFT_JOYSTICK) + "\" />\n"
                    "        <r_upper_arm SERIAL=\"E\" BONE=\"" + yost::getStandardBoneName(YOST_SKELETON_BONE_RIGHT_UPPER_ARM) + "\" />\n"
                    "        <r_lower_arm SERIAL=\"F\" BONE=\"" + yost::getStandardBoneName(YOST_SKELETON_BONE_RIGHT_LOWER_ARM) + "\" />\n"
                    "        <r_hand      SERIAL=\"10\" BONE=\"" + yost::getStandardBoneName(YOST_SKELETON_BONE_RIGHT_HAND) + "\" />\n"
                    "        <r_joystick  SERIAL=\"11\" BONE=\"" + yost::getStandardBoneName(YOST_SKELETON_BONE_RIGHT_JOYSTICK) + "\" />\n"
                    "        <r_upper_leg SERIAL=\"23\" BONE=\"" + yost::getStandardBoneName(YOST_SKELETON_BONE_RIGHT_UPPER_LEG) + "\" />\n"
                    "        <r_lower_leg SERIAL=\"24\" BONE=\"" + yost::getStandardBoneName(YOST_SKELETON_BONE_RIGHT_LOWER_LEG) + "\" />\n"
                    "    </sensors>\n"
                    "</yost>"
                    );
                yost::stored_skeletons[skel_idx]->utilitySetDeviceXmlMap(device_xml);
                return YOST_SKELETON_NO_ERROR;
            }
        }
        return YOST_SKELETON_ERROR_SKELETON_NOT_FOUND;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_setStandardDeviceXmlMapPrioProLayout(yost_skeleton_id skel_id)
    {
        if (skel_id >= YOST_SKELETON_ID && skel_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID))
        {
            uint32_t skel_idx = ~(~skel_id | YOST_SKELETON_ID);
            if (skel_idx < yost::stored_skeletons.size())
            {
                std::string device_xml = (
                "<yost>\n"
                "    <sensors SUIT_LAYOUT=\"Prio_Pro\">\n"
                "        <hips        SERIAL=\"15\" BONE=\"" + yost::getStandardBoneName(YOST_SKELETON_BONE_HIPS) + "\" />\n"
                "        <chest       SERIAL=\"0\" BONE=\"" + yost::getStandardBoneName(YOST_SKELETON_BONE_SPINE) + "\" />\n"
                "        <head        SERIAL=\"1\" BONE=\"" + yost::getStandardBoneName(YOST_SKELETON_BONE_HEAD) + "\" />\n"
                "        <l_shoulder  SERIAL=\"7\" BONE=\"" + yost::getStandardBoneName(YOST_SKELETON_BONE_LEFT_SHOULDER) + "\" />\n"
                "        <l_upper_arm SERIAL=\"8\" BONE=\"" + yost::getStandardBoneName(YOST_SKELETON_BONE_LEFT_UPPER_ARM) + "\" />\n"
                "        <l_lower_arm SERIAL=\"9\" BONE=\"" + yost::getStandardBoneName(YOST_SKELETON_BONE_LEFT_LOWER_ARM) + "\" />\n"
                "        <l_hand      SERIAL=\"A\" BONE=\"" + yost::getStandardBoneName(YOST_SKELETON_BONE_LEFT_HAND) + "\" />\n"
                "        <l_joystick  SERIAL=\"B\" BONE=\"" + yost::getStandardBoneName(YOST_SKELETON_BONE_LEFT_JOYSTICK) + "\" />\n"
                "        <r_shoulder  SERIAL=\"E\" BONE=\"" + yost::getStandardBoneName(YOST_SKELETON_BONE_RIGHT_SHOULDER) + "\" />\n"
                "        <r_upper_arm SERIAL=\"F\" BONE=\"" + yost::getStandardBoneName(YOST_SKELETON_BONE_RIGHT_UPPER_ARM) + "\" />\n"
                "        <r_lower_arm SERIAL=\"10\" BONE=\"" + yost::getStandardBoneName(YOST_SKELETON_BONE_RIGHT_LOWER_ARM) + "\" />\n"
                "        <r_hand      SERIAL=\"11\" BONE=\"" + yost::getStandardBoneName(YOST_SKELETON_BONE_RIGHT_HAND) + "\" />\n"
                "        <r_joystick  SERIAL=\"12\" BONE=\"" + yost::getStandardBoneName(YOST_SKELETON_BONE_RIGHT_JOYSTICK) + "\" />\n"
                "        <l_upper_leg SERIAL=\"1C\" BONE=\"" + yost::getStandardBoneName(YOST_SKELETON_BONE_LEFT_UPPER_LEG) + "\" />\n"
                "        <l_lower_leg SERIAL=\"1D\" BONE=\"" + yost::getStandardBoneName(YOST_SKELETON_BONE_LEFT_LOWER_LEG) + "\" />\n"
                "        <l_foot      SERIAL=\"1E\" BONE=\"" + yost::getStandardBoneName(YOST_SKELETON_BONE_LEFT_FOOT) + "\" />\n"
                "        <r_upper_leg SERIAL=\"23\" BONE=\"" + yost::getStandardBoneName(YOST_SKELETON_BONE_RIGHT_UPPER_LEG) + "\" />\n"
                "        <r_lower_leg SERIAL=\"24\" BONE=\"" + yost::getStandardBoneName(YOST_SKELETON_BONE_RIGHT_LOWER_LEG) + "\" />\n"
                "        <r_foot      SERIAL=\"25\" BONE=\"" + yost::getStandardBoneName(YOST_SKELETON_BONE_RIGHT_FOOT) + "\" />\n"
                "    </sensors>\n"
                "</yost>"
                );
                yost::stored_skeletons[skel_idx]->utilitySetDeviceXmlMap(device_xml);
                return YOST_SKELETON_NO_ERROR;
            }
        }
        return YOST_SKELETON_ERROR_SKELETON_NOT_FOUND;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_hasPrioProcessorUpdatedState(yost_skeleton_id prio_proc_id, bool* changed)
    {
        if (prio_proc_id >= YOST_SKELETON_PROCESSOR_ID && prio_proc_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID | YOST_SKELETON_PROCESSOR_ID))
        {
            uint32_t proc_idx = ~(~prio_proc_id | YOST_SKELETON_PROCESSOR_ID);
            if (proc_idx < yost::stored_processors.size())
            {
                yost::SkeletonProcessor* proc = yost::stored_processors[proc_idx].get();
                if (proc->getProcessorType() == yost::PROCESSOR_TYPE_PRIO)
                {
                    yost::PrioConnection* prio_proc = static_cast<yost::PrioConnection*>(proc);
                    prio_proc->activeSensorsHaveChanged(changed);
                    return YOST_SKELETON_NO_ERROR;
                }
            }
        }
        return YOST_SKELETON_ERROR_PROCESSOR_NOT_FOUND;
    }

    // Processor Methods
    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_destroyProcessor(yost_skeleton_id proc_id)
    {
        if (proc_id >= YOST_SKELETON_PROCESSOR_ID && proc_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID | YOST_SKELETON_PROCESSOR_ID))
        {
            uint32_t proc_idx = ~(~proc_id | YOST_SKELETON_PROCESSOR_ID);
            if (proc_idx < yost::stored_processors.size())
            {
                if (yost::stored_processors[proc_idx]->getSkeleton() == nullptr)
                {
                    yost::stored_processors[proc_idx].reset();
                    return YOST_SKELETON_NO_ERROR;
                }
                else
                {
                    return YOST_SKELETON_ERROR_PROCESSOR_IN_USE;
                }
            }
        }
        return YOST_SKELETON_ERROR_PROCESSOR_NOT_FOUND;
    }

    SKEL_EXPORT yost_skeleton_id yostskel_createPrioProcessorWithComOffset(uint8_t device_type, uint8_t com_offset)
    {
        std::unique_ptr<yost::PrioConnection> new_proc(new yost::PrioConnection(device_type, com_offset));
        if (new_proc->isConnected())
        {
            yost::stored_processors.emplace(yost::stored_processors.end(), new_proc.release());
            return (yost_skeleton_id)(YOST_SKELETON_PROCESSOR_ID + yost::stored_processors.size() - 1);
        }
        return YOST_SKELETON_INVALID_ID;
    }

    SKEL_EXPORT yost_skeleton_id yostskel_createPrioProcessorWithPrioVRBasestation(prio_device_id device_id)
    {
        std::unique_ptr<yost::PrioConnection> new_proc(new yost::PrioConnection(device_id, PRIO_TYPE::PRIO_BS));
        if (new_proc->isConnected())
        {
            yost::stored_processors.emplace(yost::stored_processors.end(), new_proc.release());
            return (yost_skeleton_id)(YOST_SKELETON_PROCESSOR_ID + yost::stored_processors.size() - 1);
        }
        return YOST_SKELETON_INVALID_ID;
    }

    SKEL_EXPORT yost_skeleton_id yostskel_createPrioProcessorWithPrioVRHub(prio_device_id device_id)
    {
        std::unique_ptr<yost::PrioConnection> new_proc(new yost::PrioConnection(device_id, PRIO_TYPE::PRIO_HUB));
        if (new_proc->isConnected())
        {
            yost::stored_processors.emplace(yost::stored_processors.end(), new_proc.release());
            return (yost_skeleton_id)(YOST_SKELETON_PROCESSOR_ID + yost::stored_processors.size() - 1);
        }
        return YOST_SKELETON_INVALID_ID;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_getPrioProcessorDeviceIndex(yost_skeleton_id prio_proc_id, uint32_t* device_id)
    {
        if (prio_proc_id >= YOST_SKELETON_PROCESSOR_ID && prio_proc_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID | YOST_SKELETON_PROCESSOR_ID))
        {
            uint32_t proc_idx = ~(~prio_proc_id | YOST_SKELETON_PROCESSOR_ID);
            if (proc_idx < yost::stored_processors.size())
            {
                yost::SkeletonProcessor* proc = yost::stored_processors[proc_idx].get();
                if (proc->getProcessorType() == yost::PROCESSOR_TYPE_PRIO)
                {
                    yost::PrioConnection* prio_proc = static_cast<yost::PrioConnection*>(proc);
                    *device_id =  prio_proc->getDeviceId();
                    return YOST_SKELETON_NO_ERROR;
                }
            }
        }
        return YOST_SKELETON_ERROR_PROCESSOR_NOT_FOUND;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_getPrioProcessorDeviceType(yost_skeleton_id prio_proc_id, uint8_t* device_type)
    {
        if (prio_proc_id >= YOST_SKELETON_PROCESSOR_ID && prio_proc_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID | YOST_SKELETON_PROCESSOR_ID))
        {
            uint32_t proc_idx = ~(~prio_proc_id | YOST_SKELETON_PROCESSOR_ID);
            if (proc_idx < yost::stored_processors.size())
            {
                yost::SkeletonProcessor* proc = yost::stored_processors[proc_idx].get();
                if (proc->getProcessorType() == yost::PROCESSOR_TYPE_PRIO)
                {
                    yost::PrioConnection* prio_proc = static_cast<yost::PrioConnection*>(proc);
                    *device_type = prio_proc->getDeviceType();
                    return YOST_SKELETON_NO_ERROR;
                }
            }
        }
        return YOST_SKELETON_ERROR_PROCESSOR_NOT_FOUND;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_calibratePrioProcessor(yost_skeleton_id prio_proc_id, float wait_time)
    {
        if (prio_proc_id >= YOST_SKELETON_PROCESSOR_ID && prio_proc_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID | YOST_SKELETON_PROCESSOR_ID))
        {
            uint32_t proc_idx = ~(~prio_proc_id | YOST_SKELETON_PROCESSOR_ID);
            if (proc_idx < yost::stored_processors.size())
            {
                yost::SkeletonProcessor* proc = yost::stored_processors[proc_idx].get();
                if (proc->getProcessorType() == yost::PROCESSOR_TYPE_PRIO)
                {
                    yost::PrioConnection* prio_proc = static_cast<yost::PrioConnection*>(proc);
                    if (prio_proc->calibrate(wait_time))
                    {
                        return YOST_SKELETON_NO_ERROR;
                    }
                    return YOST_SKELETON_ERROR_PROCESSOR_COMMAND;
                }
            }
        }
        return YOST_SKELETON_ERROR_PROCESSOR_NOT_FOUND;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_setupStandardStreamPrioProcessor(yost_skeleton_id prio_proc_id)
    {
        if (prio_proc_id >= YOST_SKELETON_PROCESSOR_ID && prio_proc_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID | YOST_SKELETON_PROCESSOR_ID))
        {
            uint32_t proc_idx = ~(~prio_proc_id | YOST_SKELETON_PROCESSOR_ID);
            if (proc_idx < yost::stored_processors.size())
            {
                yost::SkeletonProcessor* proc = yost::stored_processors[proc_idx].get();
                if (proc->getProcessorType() == yost::PROCESSOR_TYPE_PRIO)
                {
                    yost::PrioConnection* prio_proc = static_cast<yost::PrioConnection*>(proc);
                    prio_proc->setupStreamingData();
                    return YOST_SKELETON_NO_ERROR;
                }
            }
        }
        return YOST_SKELETON_ERROR_PROCESSOR_NOT_FOUND;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_setupStandardStreamWithAccelerationPrioProcessor(yost_skeleton_id prio_proc_id)
    {
        if (prio_proc_id >= YOST_SKELETON_PROCESSOR_ID && prio_proc_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID | YOST_SKELETON_PROCESSOR_ID))
        {
            uint32_t proc_idx = ~(~prio_proc_id | YOST_SKELETON_PROCESSOR_ID);
            if (proc_idx < yost::stored_processors.size())
            {
                yost::SkeletonProcessor* proc = yost::stored_processors[proc_idx].get();
                if (proc->getProcessorType() == yost::PROCESSOR_TYPE_PRIO)
                {
                    yost::PrioConnection* prio_proc = static_cast<yost::PrioConnection*>(proc);
					printf("setting up stream\n");
                    prio_proc->setupStreamingDataWithAcceleration();
                    return YOST_SKELETON_NO_ERROR;
                }
            }
        }
        return YOST_SKELETON_ERROR_PROCESSOR_NOT_FOUND;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_startPrioProcessor(yost_skeleton_id prio_proc_id)
    {
        if (prio_proc_id >= YOST_SKELETON_PROCESSOR_ID && prio_proc_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID | YOST_SKELETON_PROCESSOR_ID))
        {
            uint32_t proc_idx = ~(~prio_proc_id | YOST_SKELETON_PROCESSOR_ID);
            if (proc_idx < yost::stored_processors.size())
            {
                yost::SkeletonProcessor* proc = yost::stored_processors[proc_idx].get();
                if (proc->getProcessorType() == yost::PROCESSOR_TYPE_PRIO)
                {
                    yost::PrioConnection* prio_proc = static_cast<yost::PrioConnection*>(proc);
                    prio_proc->startStreaming();
                    return YOST_SKELETON_NO_ERROR;
                }
            }
        }
        return YOST_SKELETON_ERROR_PROCESSOR_NOT_FOUND;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_stopPrioProcessor(yost_skeleton_id prio_proc_id)
    {
        if (prio_proc_id >= YOST_SKELETON_PROCESSOR_ID && prio_proc_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID | YOST_SKELETON_PROCESSOR_ID))
        {
            uint32_t proc_idx = ~(~prio_proc_id | YOST_SKELETON_PROCESSOR_ID);
            if (proc_idx < yost::stored_processors.size())
            {
                yost::SkeletonProcessor* proc = yost::stored_processors[proc_idx].get();
                if (proc->getProcessorType() == yost::PROCESSOR_TYPE_PRIO)
                {
                    yost::PrioConnection* prio_proc = static_cast<yost::PrioConnection*>(proc);
                    prio_proc->stopStreaming();
                    return YOST_SKELETON_NO_ERROR;
                }
            }
        }
        return YOST_SKELETON_ERROR_PROCESSOR_NOT_FOUND;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_getHubBatteryLevel(yost_skeleton_id prio_proc_id, uint8_t* battery_level)
    {
        if (prio_proc_id >= YOST_SKELETON_PROCESSOR_ID && prio_proc_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID | YOST_SKELETON_PROCESSOR_ID))
        {
            uint32_t proc_idx = ~(~prio_proc_id | YOST_SKELETON_PROCESSOR_ID);
            if (proc_idx < yost::stored_processors.size())
            {
                yost::SkeletonProcessor* proc = yost::stored_processors[proc_idx].get();
                if (proc->getProcessorType() == yost::PROCESSOR_TYPE_PRIO)
                {
                    yost::PrioConnection* prio_proc = static_cast<yost::PrioConnection*>(proc);
                    prio_proc->getHubBatteryLevel(battery_level);
                    return YOST_SKELETON_NO_ERROR;
                }
            }
        }
        return YOST_SKELETON_ERROR_PROCESSOR_NOT_FOUND;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_getHubBatteryStatus(yost_skeleton_id prio_proc_id, uint8_t* battery_status)
    {
        if (prio_proc_id >= YOST_SKELETON_PROCESSOR_ID && prio_proc_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID | YOST_SKELETON_PROCESSOR_ID))
        {
            uint32_t proc_idx = ~(~prio_proc_id | YOST_SKELETON_PROCESSOR_ID);
            if (proc_idx < yost::stored_processors.size())
            {
                yost::SkeletonProcessor* proc = yost::stored_processors[proc_idx].get();
                if (proc->getProcessorType() == yost::PROCESSOR_TYPE_PRIO)
                {
                    yost::PrioConnection* prio_proc = static_cast<yost::PrioConnection*>(proc);
                    prio_proc->getHubBatteryStatus(battery_status);
                    return YOST_SKELETON_NO_ERROR;
                }
            }
        }
        return YOST_SKELETON_ERROR_PROCESSOR_NOT_FOUND;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_getHubButtonPrioProcessor(yost_skeleton_id prio_proc_id, uint8_t* button_state)
    {
        if (prio_proc_id >= YOST_SKELETON_PROCESSOR_ID && prio_proc_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID | YOST_SKELETON_PROCESSOR_ID))
        {
            uint32_t proc_idx = ~(~prio_proc_id | YOST_SKELETON_PROCESSOR_ID);
            if (proc_idx < yost::stored_processors.size())
            {
                yost::SkeletonProcessor* proc = yost::stored_processors[proc_idx].get();
                if (proc->getProcessorType() == yost::PROCESSOR_TYPE_PRIO)
                {
                    yost::PrioConnection* prio_proc = static_cast<yost::PrioConnection*>(proc);
                    prio_proc->getHubButtonState(button_state);
                    return YOST_SKELETON_NO_ERROR;
                }
            }
        }
        return YOST_SKELETON_ERROR_PROCESSOR_NOT_FOUND;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_getHubButtonByIndexPrioProcessor(yost_skeleton_id prio_proc_id, uint8_t button_index, uint8_t* button_state)
    {
        if (prio_proc_id >= YOST_SKELETON_PROCESSOR_ID && prio_proc_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID | YOST_SKELETON_PROCESSOR_ID))
        {
            uint32_t proc_idx = ~(~prio_proc_id | YOST_SKELETON_PROCESSOR_ID);
            if (proc_idx < yost::stored_processors.size())
            {
                yost::SkeletonProcessor* proc = yost::stored_processors[proc_idx].get();
                if (proc->getProcessorType() == yost::PROCESSOR_TYPE_PRIO)
                {
                    yost::PrioConnection* prio_proc = static_cast<yost::PrioConnection*>(proc);
                    prio_proc->getHubButton(button_index, button_state);
                    return YOST_SKELETON_NO_ERROR;
                }
            }
        }
        return YOST_SKELETON_ERROR_PROCESSOR_NOT_FOUND;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_getJoyStickStatePrioProcessor(yost_skeleton_id prio_proc_id, uint8_t* left_joystick_state, uint8_t* right_joystick_state )
    {
        if (prio_proc_id >= YOST_SKELETON_PROCESSOR_ID && prio_proc_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID | YOST_SKELETON_PROCESSOR_ID))
        {
            uint32_t proc_idx = ~(~prio_proc_id | YOST_SKELETON_PROCESSOR_ID);
            if (proc_idx < yost::stored_processors.size())
            {
                yost::SkeletonProcessor* proc = yost::stored_processors[proc_idx].get();
                if (proc->getProcessorType() == yost::PROCESSOR_TYPE_PRIO)
                {
                    yost::PrioConnection* prio_proc = static_cast<yost::PrioConnection*>(proc);
                    PrioJoystick left_joystick;
                    PrioJoystick right_joystick;
                    prio_proc->getJoystickState(&left_joystick, &right_joystick);

                    //We have to convert the joystick data to a format that can be used with memcpy
                    uint8_t left_joystick_array[4];
                    left_joystick_array[0] = left_joystick.x_Axis;
                    left_joystick_array[1] = left_joystick.y_Axis;
                    left_joystick_array[2] = left_joystick.Trigger;
                    left_joystick_array[3] = left_joystick.ButtonState;
                    memcpy(left_joystick_state, left_joystick_array, 4);

                    uint8_t right_joystick_array[4];
                    right_joystick_array[0] = right_joystick.x_Axis;
                    right_joystick_array[1] = right_joystick.y_Axis;
                    right_joystick_array[2] = right_joystick.Trigger;
                    right_joystick_array[3] = right_joystick.ButtonState;
                    memcpy(right_joystick_state, right_joystick_array, 4);

                    return YOST_SKELETON_NO_ERROR;
                }
            }
        }
        return YOST_SKELETON_ERROR_PROCESSOR_NOT_FOUND;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_getJoyStickStateByIndexPrioProcessor(yost_skeleton_id prio_proc_id, uint8_t joystick_index, uint8_t* joystick_state)
    {
        if (prio_proc_id >= YOST_SKELETON_PROCESSOR_ID && prio_proc_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID | YOST_SKELETON_PROCESSOR_ID))
        {
            uint32_t proc_idx = ~(~prio_proc_id | YOST_SKELETON_PROCESSOR_ID);
            if (proc_idx < yost::stored_processors.size())
            {
                yost::SkeletonProcessor* proc = yost::stored_processors[proc_idx].get();
                if (proc->getProcessorType() == yost::PROCESSOR_TYPE_PRIO)
                {
                    yost::PrioConnection* prio_proc = static_cast<yost::PrioConnection*>(proc);
                    PrioJoystick joystick;
                    PrioJoystick other_joystick;
                    if ( joystick_index == 0 )
                    {
                        prio_proc->getJoystickState(&joystick, &other_joystick);
                    }
                    else
                    {
                        prio_proc->getJoystickState(&other_joystick, &joystick);
                    }

                    uint8_t joystick_array[4];
                    joystick_array[0] = joystick.x_Axis;
                    joystick_array[1] = joystick.y_Axis;
                    joystick_array[2] = joystick.Trigger;
                    joystick_array[3] = joystick.ButtonState;
                    memcpy(joystick_state, joystick_array, 4);

                    if (joystick.ButtonState == 0)
                    {
                        return YOST_SKELETON_ERROR_JOYSTICK_INACTIVE;
                    }

                    return YOST_SKELETON_NO_ERROR;
                }
            }
        }
        return YOST_SKELETON_ERROR_PROCESSOR_NOT_FOUND;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_getJoyStickButtonStatePrioProcessor(yost_skeleton_id prio_proc_id, uint8_t button_index, uint8_t* button_state)
    {
        if (prio_proc_id >= YOST_SKELETON_PROCESSOR_ID && prio_proc_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID | YOST_SKELETON_PROCESSOR_ID))
        {
            uint32_t proc_idx = ~(~prio_proc_id | YOST_SKELETON_PROCESSOR_ID);
            if (proc_idx < yost::stored_processors.size())
            {
                yost::SkeletonProcessor* proc = yost::stored_processors[proc_idx].get();
                if (proc->getProcessorType() == yost::PROCESSOR_TYPE_PRIO)
                {
                    bool is_valid = true;
                    yost::PrioConnection* prio_proc = static_cast<yost::PrioConnection*>(proc);
                    prio_proc->getJoystickButtonState(button_index, button_state, &is_valid);
                   
                    if (!is_valid)
                    {
                        return YOST_SKELETON_ERROR_JOYSTICK_INACTIVE;
                    }
                    return YOST_SKELETON_NO_ERROR;
                }
            }
        }
        return YOST_SKELETON_ERROR_PROCESSOR_NOT_FOUND;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_getJoyStickAxisStatePrioProcessor(yost_skeleton_id prio_proc_id, uint8_t axis_index, uint8_t* axis_state)
    {
        if (prio_proc_id >= YOST_SKELETON_PROCESSOR_ID && prio_proc_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID | YOST_SKELETON_PROCESSOR_ID))
        {
            uint32_t proc_idx = ~(~prio_proc_id | YOST_SKELETON_PROCESSOR_ID);
            if (proc_idx < yost::stored_processors.size())
            {
                yost::SkeletonProcessor* proc = yost::stored_processors[proc_idx].get();
                if (proc->getProcessorType() == yost::PROCESSOR_TYPE_PRIO)
                {
                    bool is_valid = true;
                    yost::PrioConnection* prio_proc = static_cast<yost::PrioConnection*>(proc);
                    prio_proc->getJoystickAxis(axis_index, axis_state, &is_valid);

                    if (!is_valid)
                    {
                        return YOST_SKELETON_ERROR_JOYSTICK_INACTIVE;
                    }
                    return YOST_SKELETON_NO_ERROR;
                }
            }
        }
        return YOST_SKELETON_ERROR_PROCESSOR_NOT_FOUND;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_getActiveJoySticksPrioProcessor(yost_skeleton_id prio_proc_id, uint8_t* active_joysticks)
    {
        if (prio_proc_id >= YOST_SKELETON_PROCESSOR_ID && prio_proc_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID | YOST_SKELETON_PROCESSOR_ID))
        {
            uint32_t proc_idx = ~(~prio_proc_id | YOST_SKELETON_PROCESSOR_ID);
            if (proc_idx < yost::stored_processors.size())
            {
                yost::SkeletonProcessor* proc = yost::stored_processors[proc_idx].get();
                if (proc->getProcessorType() == yost::PROCESSOR_TYPE_PRIO)
                {
                    yost::PrioConnection* prio_proc = static_cast<yost::PrioConnection*>(proc);
                    prio_proc->getActiveJoysticks(active_joysticks);
                    return YOST_SKELETON_NO_ERROR;
                }
            }
        }
        return YOST_SKELETON_ERROR_PROCESSOR_NOT_FOUND;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_isJoystickActiveByIndexPrioProcessor(yost_skeleton_id prio_proc_id, uint8_t joystick_index, bool* active)
    {
        if (prio_proc_id >= YOST_SKELETON_PROCESSOR_ID && prio_proc_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID | YOST_SKELETON_PROCESSOR_ID))
        {
            uint32_t proc_idx = ~(~prio_proc_id | YOST_SKELETON_PROCESSOR_ID);
            if (proc_idx < yost::stored_processors.size())
            {
                yost::SkeletonProcessor* proc = yost::stored_processors[proc_idx].get();
                if (proc->getProcessorType() == yost::PROCESSOR_TYPE_PRIO)
                {
                    yost::PrioConnection* prio_proc = static_cast<yost::PrioConnection*>(proc);
                    prio_proc->getActiveJoystickByIndex(joystick_index, active);
                    return YOST_SKELETON_NO_ERROR;
                }
            }
        }
        return YOST_SKELETON_ERROR_PROCESSOR_NOT_FOUND;
    }


    SKEL_EXPORT yost_skeleton_id yostskel_createPedestrianTrackingProcessor()
    {
        std::unique_ptr<yost::PedestrianTracking> new_proc(new yost::PedestrianTracking());
        yost::stored_processors.emplace(yost::stored_processors.end(), new_proc.release());
        return (yost_skeleton_id)(YOST_SKELETON_PROCESSOR_ID + yost::stored_processors.size() - 1);
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_resetPedestrianTrackingProcessor(yost_skeleton_id proc_id)
    {
        if (proc_id >= YOST_SKELETON_PROCESSOR_ID && proc_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID | YOST_SKELETON_PROCESSOR_ID))
        {
            uint32_t proc_idx = ~(~proc_id | YOST_SKELETON_PROCESSOR_ID);
            if (proc_idx < yost::stored_processors.size())
            {
                yost::SkeletonProcessor* proc = yost::stored_processors[proc_idx].get();
                if (proc->getProcessorType() == yost::PROCESSOR_TYPE_PED_TRACK)
                {
                    yost::PedestrianTracking* ped_proc = static_cast<yost::PedestrianTracking*>(proc);
                    ped_proc->resetPosition();
                    return YOST_SKELETON_NO_ERROR;
                }
            }
        }
        return YOST_SKELETON_ERROR_PROCESSOR_NOT_FOUND;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_getPinnedBoneDataPedestrianTrackingProcessor(yost_skeleton_id proc_id, float* anchor_point, char* bone_name, int bone_name_length)
    {
        if (proc_id >= YOST_SKELETON_PROCESSOR_ID && proc_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID | YOST_SKELETON_PROCESSOR_ID))
        {
            uint32_t proc_idx = ~(~proc_id | YOST_SKELETON_PROCESSOR_ID);
            if (proc_idx < yost::stored_processors.size())
            {
                yost::SkeletonProcessor* proc = yost::stored_processors[proc_idx].get();
                if (proc->getProcessorType() == yost::PROCESSOR_TYPE_PED_TRACK)
                {
                    yost::PedestrianTracking* ped_proc = static_cast<yost::PedestrianTracking*>(proc);
                    std::string bn = std::string(bone_name);
                    yost::PedestrianTracking::pinnedBoneData pinned_bone_data = ped_proc->getPinnedBoneData();
                    if (pinned_bone_data.pinned_bone_name.length() > bone_name_length){
                        return YOST_SKELETON_ERROR_BUFFER_TOO_SMALL;
                    }
                    memcpy(bone_name, pinned_bone_data.pinned_bone_name.c_str(), pinned_bone_data.pinned_bone_name.size());
                    anchor_point[0] = pinned_bone_data.anchor_point.data[0];
                    anchor_point[1] = pinned_bone_data.anchor_point.data[1];
                    anchor_point[2] = pinned_bone_data.anchor_point.data[2];

                    return YOST_SKELETON_NO_ERROR;
                }
            }
        }
        return YOST_SKELETON_ERROR_PROCESSOR_NOT_FOUND;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_startTrackingBonePedestrianTrackingProcessor(yost_skeleton_id proc_id, const char *bone_name)
    {
        if (proc_id >= YOST_SKELETON_PROCESSOR_ID && proc_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID | YOST_SKELETON_PROCESSOR_ID))
        {
            uint32_t proc_idx = ~(~proc_id | YOST_SKELETON_PROCESSOR_ID);
            if (proc_idx < yost::stored_processors.size())
            {
                yost::SkeletonProcessor* proc = yost::stored_processors[proc_idx].get();
                if (proc->getProcessorType() == yost::PROCESSOR_TYPE_PED_TRACK)
                {
                    yost::PedestrianTracking* ped_proc = static_cast<yost::PedestrianTracking*>(proc);
                    std::string bn = std::string(bone_name);
                    ped_proc->startTrackingBone(bn);
                    return YOST_SKELETON_NO_ERROR;
                }
            }
        }
        return YOST_SKELETON_ERROR_PROCESSOR_NOT_FOUND;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_setRootBonePedestrianTrackingProcessor(yost_skeleton_id proc_id, const char *bone_name)
    {
        if (proc_id >= YOST_SKELETON_PROCESSOR_ID && proc_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID | YOST_SKELETON_PROCESSOR_ID))
        {
            uint32_t proc_idx = ~(~proc_id | YOST_SKELETON_PROCESSOR_ID);
            if (proc_idx < yost::stored_processors.size())
            {
                yost::SkeletonProcessor* proc = yost::stored_processors[proc_idx].get();
                if (proc->getProcessorType() == yost::PROCESSOR_TYPE_PED_TRACK)
                {
                    yost::PedestrianTracking* ped_proc = static_cast<yost::PedestrianTracking*>(proc);
                    std::string bn = std::string(bone_name);
                    ped_proc->setRootBone(bn);
                    return YOST_SKELETON_NO_ERROR;
                }
            }
        }
        return YOST_SKELETON_ERROR_PROCESSOR_NOT_FOUND;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_stopTrackingBonePedestrianTrackingProcessor(yost_skeleton_id proc_id, const char *bone_name)
    {
        if (proc_id >= YOST_SKELETON_PROCESSOR_ID && proc_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID | YOST_SKELETON_PROCESSOR_ID))
        {
            uint32_t proc_idx = ~(~proc_id | YOST_SKELETON_PROCESSOR_ID);
            if (proc_idx < yost::stored_processors.size())
            {
                yost::SkeletonProcessor* proc = yost::stored_processors[proc_idx].get();
                if (proc->getProcessorType() == yost::PROCESSOR_TYPE_PED_TRACK)
                {
                    yost::PedestrianTracking* ped_proc = static_cast<yost::PedestrianTracking*>(proc);
                    std::string bn = std::string(bone_name);
                    ped_proc->stopTrackingBone(bn);
                    return YOST_SKELETON_NO_ERROR;
                }
            }
        }
        return YOST_SKELETON_ERROR_PROCESSOR_NOT_FOUND;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_getSkeletonTranslation(yost_skeleton_id proc_id, float* translation_vector)
    {
        if (proc_id >= YOST_SKELETON_PROCESSOR_ID && proc_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID | YOST_SKELETON_PROCESSOR_ID))
        {
            uint32_t proc_idx = ~(~proc_id | YOST_SKELETON_PROCESSOR_ID);
            if (proc_idx < yost::stored_processors.size())
            {
                yost::SkeletonProcessor* proc = yost::stored_processors[proc_idx].get();
                if (proc->getProcessorType() == yost::PROCESSOR_TYPE_PED_TRACK)
                {
                    yost::PedestrianTracking* ped_proc = static_cast<yost::PedestrianTracking*>(proc);
                    Vector3 tmp = ped_proc->getSkeletonTranslation();
                    translation_vector[0] = tmp.data[0];
                    translation_vector[1] = tmp.data[1];
                    translation_vector[2] = tmp.data[2];
                    return YOST_SKELETON_NO_ERROR;
                }
            }
        }
        return YOST_SKELETON_ERROR_PROCESSOR_NOT_FOUND;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_getSkeletonTranslationAsPrecentageOfHeight(yost_skeleton_id proc_id, float* translation_vector)
    {
        if (proc_id >= YOST_SKELETON_PROCESSOR_ID && proc_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID | YOST_SKELETON_PROCESSOR_ID))
        {
            uint32_t proc_idx = ~(~proc_id | YOST_SKELETON_PROCESSOR_ID);
            if (proc_idx < yost::stored_processors.size())
            {
                yost::SkeletonProcessor* proc = yost::stored_processors[proc_idx].get();
                if (proc->getProcessorType() == yost::PROCESSOR_TYPE_PED_TRACK)
                {
                    yost::PedestrianTracking* ped_proc = static_cast<yost::PedestrianTracking*>(proc);
                    Vector3 tmp = ped_proc->getSkeletonTranslationAsPrecentOfHeight();
                    translation_vector[0] = tmp.data[0];
                    translation_vector[1] = tmp.data[1];
                    translation_vector[2] = tmp.data[2];
                    return YOST_SKELETON_NO_ERROR;
                }
            }
        }
        return YOST_SKELETON_ERROR_PROCESSOR_NOT_FOUND;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_setMaxCertaintyPedestrianTrackingProcessor(yost_skeleton_id proc_id, float certainty)
    {
        if (proc_id >= YOST_SKELETON_PROCESSOR_ID && proc_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID | YOST_SKELETON_PROCESSOR_ID))
        {
            uint32_t proc_idx = ~(~proc_id | YOST_SKELETON_PROCESSOR_ID);
            if (proc_idx < yost::stored_processors.size())
            {
                yost::SkeletonProcessor* proc = yost::stored_processors[proc_idx].get();
                if (proc->getProcessorType() == yost::PROCESSOR_TYPE_PED_TRACK)
                {
                    yost::PedestrianTracking* ped_proc = static_cast<yost::PedestrianTracking*>(proc);
                    ped_proc->setMaxCertainty(certainty);
                    return YOST_SKELETON_NO_ERROR;
                }
            }
        }
        return YOST_SKELETON_ERROR_PROCESSOR_NOT_FOUND;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_getMaxCertaintyPedestrianTrackingProcessor(yost_skeleton_id proc_id, float* certainty)
    {
        if (proc_id >= YOST_SKELETON_PROCESSOR_ID && proc_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID | YOST_SKELETON_PROCESSOR_ID))
        {
            uint32_t proc_idx = ~(~proc_id | YOST_SKELETON_PROCESSOR_ID);
            if (proc_idx < yost::stored_processors.size())
            {
                yost::SkeletonProcessor* proc = yost::stored_processors[proc_idx].get();
                if (proc->getProcessorType() == yost::PROCESSOR_TYPE_PED_TRACK)
                {
                    yost::PedestrianTracking* ped_proc = static_cast<yost::PedestrianTracking*>(proc);
                    *certainty = ped_proc->getMaxCertainty();
                    return YOST_SKELETON_NO_ERROR;
                }
            }
        }
        return YOST_SKELETON_ERROR_PROCESSOR_NOT_FOUND;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_setVarianceMultiplyFactorPedestrianTrackingProcessor(yost_skeleton_id proc_id, float multiply_factor)
    {
        if (proc_id >= YOST_SKELETON_PROCESSOR_ID && proc_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID | YOST_SKELETON_PROCESSOR_ID))
        {
            uint32_t proc_idx = ~(~proc_id | YOST_SKELETON_PROCESSOR_ID);
            if (proc_idx < yost::stored_processors.size())
            {
                yost::SkeletonProcessor* proc = yost::stored_processors[proc_idx].get();
                if (proc->getProcessorType() == yost::PROCESSOR_TYPE_PED_TRACK)
                {
                    yost::PedestrianTracking* ped_proc = static_cast<yost::PedestrianTracking*>(proc);
                    ped_proc->setVarianceMultiplyFactor(multiply_factor);
                    return YOST_SKELETON_NO_ERROR;
                }
            }
        }
        return YOST_SKELETON_ERROR_PROCESSOR_NOT_FOUND;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_getVarianceMultiplyFactorPedestrianTrackingProcessor(yost_skeleton_id proc_id, float* multiply_factor)
    {
        if (proc_id >= YOST_SKELETON_PROCESSOR_ID && proc_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID | YOST_SKELETON_PROCESSOR_ID))
        {
            uint32_t proc_idx = ~(~proc_id | YOST_SKELETON_PROCESSOR_ID);
            if (proc_idx < yost::stored_processors.size())
            {
                yost::SkeletonProcessor* proc = yost::stored_processors[proc_idx].get();
                if (proc->getProcessorType() == yost::PROCESSOR_TYPE_PED_TRACK)
                {
                    yost::PedestrianTracking* ped_proc = static_cast<yost::PedestrianTracking*>(proc);
                    *multiply_factor = ped_proc->getVarianceMultiplyFactor();
                    return YOST_SKELETON_NO_ERROR;
                }
            }
        }
        return YOST_SKELETON_ERROR_PROCESSOR_NOT_FOUND;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_setVarianceDataLengthPedestrianTrackingProcessor(yost_skeleton_id proc_id, int variance_length)
    {
        if (proc_id >= YOST_SKELETON_PROCESSOR_ID && proc_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID | YOST_SKELETON_PROCESSOR_ID))
        {
            uint32_t proc_idx = ~(~proc_id | YOST_SKELETON_PROCESSOR_ID);
            if (proc_idx < yost::stored_processors.size())
            {
                yost::SkeletonProcessor* proc = yost::stored_processors[proc_idx].get();
                if (proc->getProcessorType() == yost::PROCESSOR_TYPE_PED_TRACK)
                {
                    yost::PedestrianTracking* ped_proc = static_cast<yost::PedestrianTracking*>(proc);
                    ped_proc->setVarianceDataLength(variance_length);
                    return YOST_SKELETON_NO_ERROR;
                }
            }
        }
        return YOST_SKELETON_ERROR_PROCESSOR_NOT_FOUND;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_getVarianceDataLengthPedestrianTrackingProcessor(yost_skeleton_id proc_id, int* variance_length)
    {
        if (proc_id >= YOST_SKELETON_PROCESSOR_ID && proc_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID | YOST_SKELETON_PROCESSOR_ID))
        {
            uint32_t proc_idx = ~(~proc_id | YOST_SKELETON_PROCESSOR_ID);
            if (proc_idx < yost::stored_processors.size())
            {
                yost::SkeletonProcessor* proc = yost::stored_processors[proc_idx].get();
                if (proc->getProcessorType() == yost::PROCESSOR_TYPE_PED_TRACK)
                {
                    yost::PedestrianTracking* ped_proc = static_cast<yost::PedestrianTracking*>(proc);
                    *variance_length = ped_proc->getVarianceDataLength();
                    return YOST_SKELETON_NO_ERROR;
                }
            }
        }
        return YOST_SKELETON_ERROR_PROCESSOR_NOT_FOUND;
    }

    SKEL_EXPORT yost_skeleton_id yostskel_createSmoothingProcessor()
    {
        std::unique_ptr<yost::Smoothing> new_proc(new yost::Smoothing());
        yost::stored_processors.emplace(yost::stored_processors.end(), new_proc.release());
        return (yost_skeleton_id)(YOST_SKELETON_PROCESSOR_ID + yost::stored_processors.size() - 1);
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_setAllMinSmoothingFactorSmoothingProcessor(yost_skeleton_id proc_id, float max_smoothing_factor)
    {
        if (proc_id >= YOST_SKELETON_PROCESSOR_ID && proc_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID | YOST_SKELETON_PROCESSOR_ID))
        {
            uint32_t proc_idx = ~(~proc_id | YOST_SKELETON_PROCESSOR_ID);
            if (proc_idx < yost::stored_processors.size())
            {
                yost::SkeletonProcessor* proc = yost::stored_processors[proc_idx].get();
                if (proc->getProcessorType() == yost::PROCESSOR_TYPE_SMOOTHING)
                {
                    yost::Smoothing* smooth_proc = static_cast<yost::Smoothing*>(proc);
                    smooth_proc->setAllMinSmoothingFactor(max_smoothing_factor);
                    return YOST_SKELETON_NO_ERROR;
                }
            }
        }
        return YOST_SKELETON_ERROR_PROCESSOR_NOT_FOUND;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_setAllMaxSmoothingFactorSmoothingProcessor(yost_skeleton_id proc_id, float max_smoothing_factor)
    {
        if (proc_id >= YOST_SKELETON_PROCESSOR_ID && proc_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID | YOST_SKELETON_PROCESSOR_ID))
        {
            uint32_t proc_idx = ~(~proc_id | YOST_SKELETON_PROCESSOR_ID);
            if (proc_idx < yost::stored_processors.size())
            {
                yost::SkeletonProcessor* proc = yost::stored_processors[proc_idx].get();
                if (proc->getProcessorType() == yost::PROCESSOR_TYPE_SMOOTHING)
                {
                    yost::Smoothing* smooth_proc = static_cast<yost::Smoothing*>(proc);
                    smooth_proc->setAllMaxSmoothingFactor(max_smoothing_factor);
                    return YOST_SKELETON_NO_ERROR;
                }
            }
        }
        return YOST_SKELETON_ERROR_PROCESSOR_NOT_FOUND;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_setAllLowerVarianceBoundSmoothingProcessor(yost_skeleton_id proc_id, float lower_variance_bound)
    {
        if (proc_id >= YOST_SKELETON_PROCESSOR_ID && proc_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID | YOST_SKELETON_PROCESSOR_ID))
        {
            uint32_t proc_idx = ~(~proc_id | YOST_SKELETON_PROCESSOR_ID);
            if (proc_idx < yost::stored_processors.size())
            {
                yost::SkeletonProcessor* proc = yost::stored_processors[proc_idx].get();
                if (proc->getProcessorType() == yost::PROCESSOR_TYPE_SMOOTHING)
                {
                    yost::Smoothing* smooth_proc = static_cast<yost::Smoothing*>(proc);
                    smooth_proc->setAllLowerVarianceBound(lower_variance_bound);
                    return YOST_SKELETON_NO_ERROR;
                }
            }
        }
        return YOST_SKELETON_ERROR_PROCESSOR_NOT_FOUND;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_setAllUpperVarianceBoundSmoothingProcessor(yost_skeleton_id proc_id, float upper_variance_bound)
    {
        if (proc_id >= YOST_SKELETON_PROCESSOR_ID && proc_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID | YOST_SKELETON_PROCESSOR_ID))
        {
            uint32_t proc_idx = ~(~proc_id | YOST_SKELETON_PROCESSOR_ID);
            if (proc_idx < yost::stored_processors.size())
            {
                yost::SkeletonProcessor* proc = yost::stored_processors[proc_idx].get();
                if (proc->getProcessorType() == yost::PROCESSOR_TYPE_SMOOTHING)
                {
                    yost::Smoothing* smooth_proc = static_cast<yost::Smoothing*>(proc);
                    smooth_proc->setAllUpperVarianceBound(upper_variance_bound);
                    return YOST_SKELETON_NO_ERROR;
                }
            }
        }
        return YOST_SKELETON_ERROR_PROCESSOR_NOT_FOUND;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_setAllVarianceMultiplyFactorSmoothingProcessor(yost_skeleton_id proc_id, float multiply_factor)
    {
        if (proc_id >= YOST_SKELETON_PROCESSOR_ID && proc_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID | YOST_SKELETON_PROCESSOR_ID))
        {
            uint32_t proc_idx = ~(~proc_id | YOST_SKELETON_PROCESSOR_ID);
            if (proc_idx < yost::stored_processors.size())
            {
                yost::SkeletonProcessor* proc = yost::stored_processors[proc_idx].get();
                if (proc->getProcessorType() == yost::PROCESSOR_TYPE_SMOOTHING)
                {
                    yost::Smoothing* smooth_proc = static_cast<yost::Smoothing*>(proc);
                    smooth_proc->setAllVarianceMultiplyFactor(multiply_factor);
                    return YOST_SKELETON_NO_ERROR;
                }
            }
        }
        return YOST_SKELETON_ERROR_PROCESSOR_NOT_FOUND;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_setAllVarianceDataLengthSmoothingProcessor(yost_skeleton_id proc_id, int variance_length)
    {
        if (proc_id >= YOST_SKELETON_PROCESSOR_ID && proc_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID | YOST_SKELETON_PROCESSOR_ID))
        {
            uint32_t proc_idx = ~(~proc_id | YOST_SKELETON_PROCESSOR_ID);
            if (proc_idx < yost::stored_processors.size())
            {
                yost::SkeletonProcessor* proc = yost::stored_processors[proc_idx].get();
                if (proc->getProcessorType() == yost::PROCESSOR_TYPE_SMOOTHING)
                {
                    yost::Smoothing* smooth_proc = static_cast<yost::Smoothing*>(proc);
                    smooth_proc->setAllVarianceDataLength(variance_length);
                    return YOST_SKELETON_NO_ERROR;
                }
            }
        }
        return YOST_SKELETON_ERROR_PROCESSOR_NOT_FOUND;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_setBoneMinSmoothingFactorSmoothingProcessor(yost_skeleton_id proc_id, const char* bone_name, float min_smoothing_factor)
    {
        if (proc_id >= YOST_SKELETON_PROCESSOR_ID && proc_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID | YOST_SKELETON_PROCESSOR_ID))
        {
            uint32_t proc_idx = ~(~proc_id | YOST_SKELETON_PROCESSOR_ID);
            if (proc_idx < yost::stored_processors.size())
            {
                yost::SkeletonProcessor* proc = yost::stored_processors[proc_idx].get();
                if (proc->getProcessorType() == yost::PROCESSOR_TYPE_SMOOTHING)
                {
                    yost::Smoothing* smooth_proc = static_cast<yost::Smoothing*>(proc);
                    return smooth_proc->setBoneMinSmoothingFactor(bone_name, min_smoothing_factor);
                }
            }
        }
        return YOST_SKELETON_ERROR_PROCESSOR_NOT_FOUND;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_getBoneMinSmoothingFactorSmoothingProcessor(yost_skeleton_id proc_id, const char* bone_name, float* min_smoothing_factor)
    {
        if (proc_id >= YOST_SKELETON_PROCESSOR_ID && proc_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID | YOST_SKELETON_PROCESSOR_ID))
        {
            uint32_t proc_idx = ~(~proc_id | YOST_SKELETON_PROCESSOR_ID);
            if (proc_idx < yost::stored_processors.size())
            {
                yost::SkeletonProcessor* proc = yost::stored_processors[proc_idx].get();
                if (proc->getProcessorType() == yost::PROCESSOR_TYPE_SMOOTHING)
                {
                    yost::Smoothing* smooth_proc = static_cast<yost::Smoothing*>(proc);
                    return smooth_proc->getBoneMinSmoothingFactor(bone_name, min_smoothing_factor);
                }
            }
        }
        return YOST_SKELETON_ERROR_PROCESSOR_NOT_FOUND;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_setBoneMaxSmoothingFactorSmoothingProcessor(yost_skeleton_id proc_id, const char* bone_name, float max_smoothing_factor)
    {
        if (proc_id >= YOST_SKELETON_PROCESSOR_ID && proc_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID | YOST_SKELETON_PROCESSOR_ID))
        {
            uint32_t proc_idx = ~(~proc_id | YOST_SKELETON_PROCESSOR_ID);
            if (proc_idx < yost::stored_processors.size())
            {
                yost::SkeletonProcessor* proc = yost::stored_processors[proc_idx].get();
                if (proc->getProcessorType() == yost::PROCESSOR_TYPE_SMOOTHING)
                {
                    yost::Smoothing* smooth_proc = static_cast<yost::Smoothing*>(proc);
                    return smooth_proc->setBoneMaxSmoothingFactor(bone_name, max_smoothing_factor);
                }
            }
        }
        return YOST_SKELETON_ERROR_PROCESSOR_NOT_FOUND;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_getBoneMaxSmoothingFactorSmoothingProcessor(yost_skeleton_id proc_id, const char* bone_name, float* max_smoothing_factor)
    {
        if (proc_id >= YOST_SKELETON_PROCESSOR_ID && proc_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID | YOST_SKELETON_PROCESSOR_ID))
        {
            uint32_t proc_idx = ~(~proc_id | YOST_SKELETON_PROCESSOR_ID);
            if (proc_idx < yost::stored_processors.size())
            {
                yost::SkeletonProcessor* proc = yost::stored_processors[proc_idx].get();
                if (proc->getProcessorType() == yost::PROCESSOR_TYPE_SMOOTHING)
                {
                    yost::Smoothing* smooth_proc = static_cast<yost::Smoothing*>(proc);
                    return smooth_proc->getBoneMaxSmoothingFactor(bone_name, max_smoothing_factor);
                }
            }
        }
        return YOST_SKELETON_ERROR_PROCESSOR_NOT_FOUND;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_setBoneLowerVarianceBoundSmoothingProcessor(yost_skeleton_id proc_id, const char* bone_name, float lower_variance_bound)
    {
        if (proc_id >= YOST_SKELETON_PROCESSOR_ID && proc_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID | YOST_SKELETON_PROCESSOR_ID))
        {
            uint32_t proc_idx = ~(~proc_id | YOST_SKELETON_PROCESSOR_ID);
            if (proc_idx < yost::stored_processors.size())
            {
                yost::SkeletonProcessor* proc = yost::stored_processors[proc_idx].get();
                if (proc->getProcessorType() == yost::PROCESSOR_TYPE_SMOOTHING)
                {
                    yost::Smoothing* smooth_proc = static_cast<yost::Smoothing*>(proc);
                    return smooth_proc->setBoneLowerVarianceBound(bone_name, lower_variance_bound);
                }
            }
        }
        return YOST_SKELETON_ERROR_PROCESSOR_NOT_FOUND;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_getBoneLowerVarianceBoundSmoothingProcessor(yost_skeleton_id proc_id, const char* bone_name, float* lower_variance_bound)
    {
        if (proc_id >= YOST_SKELETON_PROCESSOR_ID && proc_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID | YOST_SKELETON_PROCESSOR_ID))
        {
            uint32_t proc_idx = ~(~proc_id | YOST_SKELETON_PROCESSOR_ID);
            if (proc_idx < yost::stored_processors.size())
            {
                yost::SkeletonProcessor* proc = yost::stored_processors[proc_idx].get();
                if (proc->getProcessorType() == yost::PROCESSOR_TYPE_SMOOTHING)
                {
                    yost::Smoothing* smooth_proc = static_cast<yost::Smoothing*>(proc);
                    return smooth_proc->getBoneLowerVarianceBound(bone_name, lower_variance_bound);
                }
            }
        }
        return YOST_SKELETON_ERROR_PROCESSOR_NOT_FOUND;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_setBoneUpperVarianceBoundSmoothingProcessor(yost_skeleton_id proc_id, const char* bone_name, float upper_variance_bound)
    {
        if (proc_id >= YOST_SKELETON_PROCESSOR_ID && proc_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID | YOST_SKELETON_PROCESSOR_ID))
        {
            uint32_t proc_idx = ~(~proc_id | YOST_SKELETON_PROCESSOR_ID);
            if (proc_idx < yost::stored_processors.size())
            {
                yost::SkeletonProcessor* proc = yost::stored_processors[proc_idx].get();
                if (proc->getProcessorType() == yost::PROCESSOR_TYPE_SMOOTHING)
                {
                    yost::Smoothing* smooth_proc = static_cast<yost::Smoothing*>(proc);
                    return smooth_proc->setBoneUpperVarianceBound(bone_name, upper_variance_bound);
                }
            }
        }
        return YOST_SKELETON_ERROR_PROCESSOR_NOT_FOUND;
    }
    
    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_getBoneUpperVarianceBoundSmoothingProcessor(yost_skeleton_id proc_id, const char* bone_name, float* upper_variance_bound)
    {
        if (proc_id >= YOST_SKELETON_PROCESSOR_ID && proc_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID | YOST_SKELETON_PROCESSOR_ID))
        {
            uint32_t proc_idx = ~(~proc_id | YOST_SKELETON_PROCESSOR_ID);
            if (proc_idx < yost::stored_processors.size())
            {
                yost::SkeletonProcessor* proc = yost::stored_processors[proc_idx].get();
                if (proc->getProcessorType() == yost::PROCESSOR_TYPE_SMOOTHING)
                {
                    yost::Smoothing* smooth_proc = static_cast<yost::Smoothing*>(proc);
                    return smooth_proc->getBoneUpperVarianceBound(bone_name, upper_variance_bound);
                }
            }
        }
        return YOST_SKELETON_ERROR_PROCESSOR_NOT_FOUND;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_setBoneVarianceMultiplyFactorSmoothingProcessor(yost_skeleton_id proc_id, const char* bone_name, float multiply_factor)
    {
        if (proc_id >= YOST_SKELETON_PROCESSOR_ID && proc_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID | YOST_SKELETON_PROCESSOR_ID))
        {
            uint32_t proc_idx = ~(~proc_id | YOST_SKELETON_PROCESSOR_ID);
            if (proc_idx < yost::stored_processors.size())
            {
                yost::SkeletonProcessor* proc = yost::stored_processors[proc_idx].get();
                if (proc->getProcessorType() == yost::PROCESSOR_TYPE_SMOOTHING)
                {
                    yost::Smoothing* smooth_proc = static_cast<yost::Smoothing*>(proc);
                    return smooth_proc->setBoneVarianceMultiplyFactor(bone_name, multiply_factor);
                }
            }
        }
        return YOST_SKELETON_ERROR_PROCESSOR_NOT_FOUND;
    }
    
    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_getBoneVarianveMultiplyFactorSmoothingProcessor(yost_skeleton_id proc_id, const char* bone_name, float* multiply_factor)
    {
        if (proc_id >= YOST_SKELETON_PROCESSOR_ID && proc_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID | YOST_SKELETON_PROCESSOR_ID))
        {
            uint32_t proc_idx = ~(~proc_id | YOST_SKELETON_PROCESSOR_ID);
            if (proc_idx < yost::stored_processors.size())
            {
                yost::SkeletonProcessor* proc = yost::stored_processors[proc_idx].get();
                if (proc->getProcessorType() == yost::PROCESSOR_TYPE_SMOOTHING)
                {
                    yost::Smoothing* smooth_proc = static_cast<yost::Smoothing*>(proc);
                    return smooth_proc->getBoneVarianceMultiplyFactor(bone_name, multiply_factor);
                }
            }
        }
        return YOST_SKELETON_ERROR_PROCESSOR_NOT_FOUND;
    }
    
    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_setBoneVarianceDataLengthSmoothingProcessor(yost_skeleton_id proc_id, const char* bone_name, int variance_length)
    {
        if (proc_id >= YOST_SKELETON_PROCESSOR_ID && proc_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID | YOST_SKELETON_PROCESSOR_ID))
        {
            uint32_t proc_idx = ~(~proc_id | YOST_SKELETON_PROCESSOR_ID);
            if (proc_idx < yost::stored_processors.size())
            {
                yost::SkeletonProcessor* proc = yost::stored_processors[proc_idx].get();
                if (proc->getProcessorType() == yost::PROCESSOR_TYPE_SMOOTHING)
                {
                    yost::Smoothing* smooth_proc = static_cast<yost::Smoothing*>(proc);
                    return smooth_proc->setBoneVarianceDataLength(bone_name, variance_length);
                }
            }
        }
        return YOST_SKELETON_ERROR_PROCESSOR_NOT_FOUND;
    }
    
    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_getBoneVarianceDataLengthSmoothingProcessor(yost_skeleton_id proc_id, const char* bone_name, int* variance_length)
    {
        if (proc_id >= YOST_SKELETON_PROCESSOR_ID && proc_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID | YOST_SKELETON_PROCESSOR_ID))
        {
            uint32_t proc_idx = ~(~proc_id | YOST_SKELETON_PROCESSOR_ID);
            if (proc_idx < yost::stored_processors.size())
            {
                yost::SkeletonProcessor* proc = yost::stored_processors[proc_idx].get();
                if (proc->getProcessorType() == yost::PROCESSOR_TYPE_SMOOTHING)
                {
                    yost::Smoothing* smooth_proc = static_cast<yost::Smoothing*>(proc);
                    return smooth_proc->getBoneVarianceDataLength(bone_name, variance_length);
                }
            }
        }
        return YOST_SKELETON_ERROR_PROCESSOR_NOT_FOUND;
    }

    SKEL_EXPORT yost_skeleton_id yostskel_createTssProcessor()
    {
        std::unique_ptr<yost::TssConnection> new_proc(new yost::TssConnection());
        if (new_proc->isConnected())
        {
            yost::stored_processors.emplace(yost::stored_processors.end(), new_proc.release());
            return (yost_skeleton_id)(YOST_SKELETON_PROCESSOR_ID + yost::stored_processors.size() - 1);
        }
        return YOST_SKELETON_INVALID_ID;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_calibrateTssProcessor(yost_skeleton_id proc_id, float wait_time)
    {
        if (proc_id >= YOST_SKELETON_PROCESSOR_ID && proc_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID | YOST_SKELETON_PROCESSOR_ID))
        {
            uint32_t proc_idx = ~(~proc_id | YOST_SKELETON_PROCESSOR_ID);
            if (proc_idx < yost::stored_processors.size())
            {
                yost::SkeletonProcessor* proc = yost::stored_processors[proc_idx].get();
                if (proc->getProcessorType() == yost::PROCESSOR_TYPE_TSS)
                {
                    yost::TssConnection* tss_proc = static_cast<yost::TssConnection*>(proc);
                    if (tss_proc->calibrate(wait_time))
                    {
                        return YOST_SKELETON_NO_ERROR;
                    }
                    return YOST_SKELETON_ERROR_PROCESSOR_COMMAND;
                }
            }
        }
        return YOST_SKELETON_ERROR_PROCESSOR_NOT_FOUND;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_startTssProcessor(yost_skeleton_id proc_id)
    {
        if (proc_id >= YOST_SKELETON_PROCESSOR_ID && proc_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID | YOST_SKELETON_PROCESSOR_ID))
        {
            uint32_t proc_idx = ~(~proc_id | YOST_SKELETON_PROCESSOR_ID);
            if (proc_idx < yost::stored_processors.size())
            {
                yost::SkeletonProcessor* proc = yost::stored_processors[proc_idx].get();
                if (proc->getProcessorType() == yost::PROCESSOR_TYPE_TSS)
                {
                    yost::TssConnection* prio_proc = static_cast<yost::TssConnection*>(proc);
                    prio_proc->startStreaming();
                    return YOST_SKELETON_NO_ERROR;
                }
            }
        }
        return YOST_SKELETON_ERROR_PROCESSOR_NOT_FOUND;
    }

    SKEL_EXPORT YOST_SKELETON_ERROR yostskel_stopTssProcessor(yost_skeleton_id proc_id)
    {
        if (proc_id >= YOST_SKELETON_PROCESSOR_ID && proc_id < (YOST_SKELETON_INVALID_ID | YOST_SKELETON_ID | YOST_SKELETON_PROCESSOR_ID))
        {
            uint32_t proc_idx = ~(~proc_id | YOST_SKELETON_PROCESSOR_ID);
            if (proc_idx < yost::stored_processors.size())
            {
                yost::SkeletonProcessor* proc = yost::stored_processors[proc_idx].get();
                if (proc->getProcessorType() == yost::PROCESSOR_TYPE_TSS)
                {
                    yost::TssConnection* prio_proc = static_cast<yost::TssConnection*>(proc);
                    prio_proc->stopStreaming();
                    return YOST_SKELETON_NO_ERROR;
                }
            }
        }
        return YOST_SKELETON_ERROR_PROCESSOR_NOT_FOUND;
    }

}