/********************************************//**
                                              * Copyright 1998-2014, Yost Labs Corporation
                                              * This Source Code Form is subject to the terms of the YOST 3-Space Open
                                              * Source License available online at:
                                              * http://www.yeitechnology.com/yei-3-space-open-source-license
                                              ***********************************************/
#include "yost_smoothing_processor.hpp"
#include "yost_skeleton_core_api.hpp"
#include <mutex>
#include <thread>

namespace yost
{
    struct variances
    {
        std::deque<float>* x;
        std::deque<float>* y;
        std::deque<float>* z;
        std::deque<float>* w;
    };

    static std::mutex smoothing_io_mutex;

    Smoothing::Smoothing()
    {
        _type = PROCESSOR_TYPE_SMOOTHING;
        float test = 0;

        smooth_bone tmp;
        tmp.max_variance_data_length = 10;
        tmp.min_smoothing_factor = 0;
        tmp.max_smoothing_factor = .6f;
        tmp.lower_variance_bound = .02f;
        tmp.upper_variance_bound = .25f;
        tmp.variance_multiply_factor = 4;

    }

    Smoothing::~Smoothing()
    {
    }

    void Smoothing::startSmoothingBone(std::string bone_name)
    {
        smooth_bone tmp;
        tmp.max_variance_data_length = 10;
        tmp.min_smoothing_factor = 0;
        tmp.max_smoothing_factor = .6f;
        tmp.lower_variance_bound = .02f;
        tmp.upper_variance_bound = .25f;
        tmp.variance_multiply_factor = 4;
        _bone_options[bone_name] = tmp;
    }

    void Smoothing::stopSmoothingBone(std::string bone_name)
    {
        _bone_options.erase(bone_name);
    }

    void Smoothing::setAllMinSmoothingFactor(float min_smoothing_factor)
    {
        for (std::map<std::string, smooth_bone>::iterator iter = _bone_options.begin(), itr_end = _bone_options.end(); iter != itr_end; ++iter)
        {
            iter->second.min_smoothing_factor = min_smoothing_factor;
        }
    }

    void Smoothing::setAllMaxSmoothingFactor(float max_smoothing_factor)
    {
        for (std::map<std::string, smooth_bone>::iterator iter = _bone_options.begin(), itr_end = _bone_options.end(); iter != itr_end; ++iter)
        {
            iter->second.max_smoothing_factor = max_smoothing_factor;
        }
    }

    void Smoothing::setAllLowerVarianceBound(float lower_variance_bound)
    {
        for (std::map<std::string, smooth_bone>::iterator iter = _bone_options.begin(), itr_end = _bone_options.end(); iter != itr_end; ++iter)
        {
            iter->second.lower_variance_bound = lower_variance_bound;
        }
    }

    void Smoothing::setAllUpperVarianceBound(float upper_variance_bound)
    {
        for (std::map<std::string, smooth_bone>::iterator iter = _bone_options.begin(), itr_end = _bone_options.end(); iter != itr_end; ++iter)
        {
            iter->second.upper_variance_bound = upper_variance_bound;
        }
    }

    void Smoothing::setAllVarianceMultiplyFactor(float multiply_factor)
    {
        for (std::map<std::string, smooth_bone>::iterator iter = _bone_options.begin(), itr_end = _bone_options.end(); iter != itr_end; ++iter)
        {
            iter->second.variance_multiply_factor = multiply_factor;
        }
    }

    void Smoothing::setAllVarianceDataLength(int variance_length)
    {
        for (std::map<std::string, smooth_bone>::iterator iter = _bone_options.begin(), itr_end = _bone_options.end(); iter != itr_end; ++iter)
        {
            iter->second.max_variance_data_length = variance_length;
        }
    }

    YOST_SKELETON_ERROR Smoothing::setBoneMinSmoothingFactor(std::string bone_name, float min_smoothing_factor)
    {
        auto name_it = _bone_options.find(bone_name);
        if (name_it == _bone_options.end())
        {
            return YOST_SKELETON_ERROR_BONE_NOT_FOUND;
        }

        name_it->second.min_smoothing_factor = min_smoothing_factor;
        return YOST_SKELETON_NO_ERROR;
    }

    YOST_SKELETON_ERROR Smoothing::getBoneMinSmoothingFactor(std::string bone_name, float* min_smoothing_factor)
    {
        auto name_it = _bone_options.find(bone_name);
        if (name_it == _bone_options.end())
        {
            return YOST_SKELETON_ERROR_BONE_NOT_FOUND;
        }

        *min_smoothing_factor = name_it->second.min_smoothing_factor;
        return YOST_SKELETON_NO_ERROR;
    }

    YOST_SKELETON_ERROR Smoothing::setBoneMaxSmoothingFactor(std::string bone_name, float max_smoothing_factor)
    {
        auto name_it = _bone_options.find(bone_name);
        if (name_it == _bone_options.end())
        {
            return YOST_SKELETON_ERROR_BONE_NOT_FOUND;
        }

        name_it->second.max_smoothing_factor = max_smoothing_factor;
        return YOST_SKELETON_NO_ERROR;
    }

    YOST_SKELETON_ERROR Smoothing::getBoneMaxSmoothingFactor(std::string bone_name, float* max_smoothing_factor)
    {
        auto name_it = _bone_options.find(bone_name);
        if (name_it == _bone_options.end())
        {
            return YOST_SKELETON_ERROR_BONE_NOT_FOUND;
        }

        *max_smoothing_factor = name_it->second.max_smoothing_factor;
        return YOST_SKELETON_NO_ERROR;
    }

    YOST_SKELETON_ERROR Smoothing::setBoneLowerVarianceBound(std::string bone_name, float lower_variance_bound)
    {
        auto name_it = _bone_options.find(bone_name);
        if (name_it == _bone_options.end())
        {
            return YOST_SKELETON_ERROR_BONE_NOT_FOUND;
        }

        name_it->second.lower_variance_bound = lower_variance_bound;
        return YOST_SKELETON_NO_ERROR;
    }

    YOST_SKELETON_ERROR Smoothing::getBoneLowerVarianceBound(std::string bone_name, float* lower_variance_bound)
    {
        auto name_it = _bone_options.find(bone_name);
        if (name_it == _bone_options.end())
        {
            return YOST_SKELETON_ERROR_BONE_NOT_FOUND;
        }

        *lower_variance_bound = name_it->second.lower_variance_bound;
        return YOST_SKELETON_NO_ERROR;
    }

    YOST_SKELETON_ERROR Smoothing::setBoneUpperVarianceBound(std::string bone_name, float upper_variance_bound)
    {
        auto name_it = _bone_options.find(bone_name);
        if (name_it == _bone_options.end())
        {
            return YOST_SKELETON_ERROR_BONE_NOT_FOUND;
        }

        name_it->second.upper_variance_bound = upper_variance_bound;
        return YOST_SKELETON_NO_ERROR;
    }

    YOST_SKELETON_ERROR Smoothing::getBoneUpperVarianceBound(std::string bone_name, float* upper_variance_bound)
    {
        auto name_it = _bone_options.find(bone_name);
        if (name_it == _bone_options.end())
        {
            return YOST_SKELETON_ERROR_BONE_NOT_FOUND;
        }

        *upper_variance_bound = name_it->second.upper_variance_bound;
        return YOST_SKELETON_NO_ERROR;
    }

    YOST_SKELETON_ERROR Smoothing::setBoneVarianceMultiplyFactor(std::string bone_name, float multiply_factor)
    {
        auto name_it = _bone_options.find(bone_name);
        if (name_it == _bone_options.end())
        {
            return YOST_SKELETON_ERROR_BONE_NOT_FOUND;
        }

        name_it->second.variance_multiply_factor = multiply_factor;
        return YOST_SKELETON_NO_ERROR;
    }

    YOST_SKELETON_ERROR Smoothing::getBoneVarianceMultiplyFactor(std::string bone_name, float* multiply_factor)
    {
        auto name_it = _bone_options.find(bone_name);
        if (name_it == _bone_options.end())
        {
            return YOST_SKELETON_ERROR_BONE_NOT_FOUND;
        }

        *multiply_factor = name_it->second.variance_multiply_factor;
        return YOST_SKELETON_NO_ERROR;
    }

    YOST_SKELETON_ERROR Smoothing::setBoneVarianceDataLength(std::string bone_name, int variance_length)
    {
        auto name_it = _bone_options.find(bone_name);
        if (name_it == _bone_options.end())
        {
            return YOST_SKELETON_ERROR_BONE_NOT_FOUND;
        }

        name_it->second.max_variance_data_length = variance_length;
        return YOST_SKELETON_NO_ERROR;
    }

    YOST_SKELETON_ERROR Smoothing::getBoneVarianceDataLength(std::string bone_name, int* variance_length)
    {
        auto name_it = _bone_options.find(bone_name);
        if (name_it == _bone_options.end())
        {
            return YOST_SKELETON_ERROR_BONE_NOT_FOUND;
        }

        *variance_length = name_it->second.max_variance_data_length;
        return YOST_SKELETON_NO_ERROR;
    }

    Orient Smoothing::_slerp(Orient qa, Orient qb, double t)
    {
        // Quaternion to return
        Orient qm;
        // Calculate angle between them.
        double cosHalfTheta = qa.data[QUAT_W] * qb.data[QUAT_W] + qa.data[QUAT_X] * qb.data[QUAT_X] + qa.data[QUAT_Y] * qb.data[QUAT_Y] + qa.data[QUAT_Z] * qb.data[QUAT_Z];
        // If qa=qb or qa=-qb then theta = 0 and we can return qa
        if (abs(cosHalfTheta) >= 1.0)
        {
            qm.data[QUAT_W] = qa.data[QUAT_W]; qm.data[QUAT_X] = qa.data[QUAT_X]; qm.data[QUAT_Y] = qa.data[QUAT_Y]; qm.data[QUAT_Z] = qa.data[QUAT_Z];
            return qm;
        }
        // Calculate temporary values.
        double halfTheta = acos(cosHalfTheta);
        double sinHalfTheta = sqrt(1.0 - cosHalfTheta*cosHalfTheta);
        // If theta = 180 degrees then result is not fully defined
        // we could rotate around any axis normal to qa or qb
        if (fabs(sinHalfTheta) < 0.001) // fabs is floating point absolute
        {
            qm.data[QUAT_W] = (qa.data[QUAT_W] * 0.5f + qb.data[QUAT_W] * 0.5f);
            qm.data[QUAT_X] = (qa.data[QUAT_X] * 0.5f + qb.data[QUAT_X] * 0.5f);
            qm.data[QUAT_Y] = (qa.data[QUAT_Y] * 0.5f + qb.data[QUAT_Y] * 0.5f);
            qm.data[QUAT_Z] = (qa.data[QUAT_Z] * 0.5f + qb.data[QUAT_Z] * 0.5f);
            return qm;
        }
        double ratioA = sin((1 - t) * halfTheta) / sinHalfTheta;
        double ratioB = sin(t * halfTheta) / sinHalfTheta;
        // Calculate Quaternion.
        qm.data[QUAT_W] = (qa.data[QUAT_W] * (float)ratioA + qb.data[QUAT_W] * (float)ratioB);
        qm.data[QUAT_X] = (qa.data[QUAT_X] * (float)ratioA + qb.data[QUAT_X] * (float)ratioB);
        qm.data[QUAT_Y] = (qa.data[QUAT_Y] * (float)ratioA + qb.data[QUAT_Y] * (float)ratioB);
        qm.data[QUAT_Z] = (qa.data[QUAT_Z] * (float)ratioA + qb.data[QUAT_Z] * (float)ratioB);
        orientNormalize(&qm);
        return qm;
    }

    void Smoothing::runProcess()
    {
        bool islocked = smoothing_io_mutex.try_lock();
        if (islocked)
        {
            _runSmoothing();
            smoothing_io_mutex.unlock();
        }
    }

    float Smoothing::_calculateVariance(std::deque<float> arr)
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

    void Smoothing::_runSmoothing()
    {
        std::shared_ptr<Bone> root_bone = _skeleton->getRootBone();
        if (root_bone == nullptr)
        {
            return;
        }

        std::vector<std::shared_ptr<Bone>> bones = _skeleton->getBoneList();

        for (int i = 0; i < bones.size(); i++)
        {
            if (_bone_variance_deques.find(bones[i]->getName()) == _bone_variance_deques.end())
            {
                smooth_bone tmp;
                tmp.max_variance_data_length = 10;
                tmp.min_smoothing_factor = 0;
                tmp.max_smoothing_factor = .6f;
                tmp.lower_variance_bound = .02f;
                tmp.upper_variance_bound = .25f;
                tmp.variance_multiply_factor = 4;
                _bone_options[bones[i]->getName()] = tmp;
                _bone_variance_deques[bones[i]->getName()] = new struct variances;
                _bone_variance_deques[bones[i]->getName()]->x = new std::deque < float >;
                _bone_variance_deques[bones[i]->getName()]->y = new std::deque < float >;
                _bone_variance_deques[bones[i]->getName()]->z = new std::deque < float >;
                _bone_variance_deques[bones[i]->getName()]->w = new std::deque < float >;
            }

            Orient tmpOrient = bones[i]->getOrientation();
            (*_bone_variance_deques[bones[i]->getName()]->x).push_back(tmpOrient.data[QUAT_X]);
            (*_bone_variance_deques[bones[i]->getName()]->y).push_back(tmpOrient.data[QUAT_Y]);
            (*_bone_variance_deques[bones[i]->getName()]->z).push_back(tmpOrient.data[QUAT_Z]);
            (*_bone_variance_deques[bones[i]->getName()]->w).push_back(tmpOrient.data[QUAT_W]);

            if ((*_bone_variance_deques[bones[i]->getName()]->x).size() > _bone_options[bones[i]->getName()].max_variance_data_length)
            {
                (*_bone_variance_deques[bones[i]->getName()]->x).pop_front();
            }

            if ((*_bone_variance_deques[bones[i]->getName()]->y).size() > _bone_options[bones[i]->getName()].max_variance_data_length)
            {
                (*_bone_variance_deques[bones[i]->getName()]->y).pop_front();
            }

            if ((*_bone_variance_deques[bones[i]->getName()]->z).size() > _bone_options[bones[i]->getName()].max_variance_data_length)
            {
                (*_bone_variance_deques[bones[i]->getName()]->z).pop_front();
            }

            if ((*_bone_variance_deques[bones[i]->getName()]->w).size() > _bone_options[bones[i]->getName()].max_variance_data_length)
            {
                (*_bone_variance_deques[bones[i]->getName()]->w).pop_front();
            }

            float varianceX = _calculateVariance(*_bone_variance_deques[bones[i]->getName()]->x);
            float varianceY = _calculateVariance(*_bone_variance_deques[bones[i]->getName()]->y);
            float varianceZ = _calculateVariance(*_bone_variance_deques[bones[i]->getName()]->z);
            float varianceW = _calculateVariance(*_bone_variance_deques[bones[i]->getName()]->w);

            float tVariance = varianceX + varianceY + varianceZ + varianceW;
            //printf("the total variance is: %f\n", tVariance);

            if (tVariance < _bone_options[bones[i]->getName()].lower_variance_bound)
            {
                tVariance = _bone_options[bones[i]->getName()].lower_variance_bound;
            }

            if (tVariance > _bone_options[bones[i]->getName()].upper_variance_bound)
            {
                tVariance = _bone_options[bones[i]->getName()].upper_variance_bound;
            }

            float smoothingFactor = (tVariance - _bone_options[bones[i]->getName()].lower_variance_bound) / (_bone_options[bones[i]->getName()].upper_variance_bound - _bone_options[bones[i]->getName()].lower_variance_bound);
            smoothingFactor *= _bone_options[bones[i]->getName()].max_smoothing_factor;
            smoothingFactor = 1 - smoothingFactor;

            if (smoothingFactor < _bone_options[bones[i]->getName()].min_smoothing_factor)
            {
                smoothingFactor = _bone_options[bones[i]->getName()].min_smoothing_factor;
            }

            if (smoothingFactor > _bone_options[bones[i]->getName()].max_smoothing_factor)
            {
                smoothingFactor = _bone_options[bones[i]->getName()].max_smoothing_factor;
            }

            if (_old_quaternions.find(bones[i]->getName()) == _old_quaternions.end())
            {
                _old_quaternions[bones[i]->getName()] = bones[i]->getOrientation();
            }

            Orient slerpO = this->_slerp(_old_quaternions[bones[i]->getName()], bones[i]->getOrientation(), 1 - smoothingFactor);
            bones[i]->setOrientation(slerpO);
            _old_quaternions[bones[i]->getName()] = slerpO;
        }
        _skeleton->getRootBone()->updatePosition();
    }
};