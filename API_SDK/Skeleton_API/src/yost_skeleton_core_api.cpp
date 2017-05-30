/********************************************//**
* Copyright 1998-2014, Yost Labs Corporation
* This Source Code Form is subject to the terms of the YOST 3-Space Open
* Source License available online at:
* http://www.yeitechnology.com/yei-3-space-open-source-license
***********************************************/
#include "yost_skeleton_core_api.hpp"
#include <iostream>
#include <fstream>

namespace yost
{

    bool axisDirectionsEnabled = false;

    const char g_version[] = "YOST SKELETAL API 0.1.0";

    const char* g_standard_bone_names[67] =
    {
        "Hips",
        "Spine2",
        "Spine1",
        "Spine",
        "Neck",
        "Head",
        "HeadEnd",
        "LeftShoulder",
        "LeftUpperArm",
        "LeftLowerArm",
        "LeftHand",
        "LeftHandThumb1",
        "LeftHandThumb2",
        "LeftHandThumb3",
        "LeftHandThumbEnd",
        "LeftHandIndex1",
        "LeftHandIndex2",
        "LeftHandIndex3",
        "LeftHandIndexEnd",
        "LeftHandMiddle1",
        "LeftHandMiddle2",
        "LeftHandMiddle3",
        "LeftHandMiddleEnd",
        "LeftHandRing1",
        "LeftHandRing2",
        "LeftHandRing3",
        "LeftHandRingEnd",
        "LeftHandPinky1",
        "LeftHandPinky2",
        "LeftHandPinky3",
        "LeftHandPinkyEnd",
        "LeftJoystick",
        "RightShoulder",
        "RightUpperArm",
        "RightLowerArm",
        "RightHand",
        "RightHandThumb1",
        "RightHandThumb2",
        "RightHandThumb3",
        "RightHandThumbEnd",
        "RightHandIndex1",
        "RightHandIndex2",
        "RightHandIndex3",
        "RightHandIndexEnd",
        "RightHandMiddle1",
        "RightHandMiddle2",
        "RightHandMiddle3",
        "RightHandMiddleEnd",
        "RightHandRing1",
        "RightHandRing2",
        "RightHandRing3",
        "RightHandRingEnd",
        "RightHandPinky1",
        "RightHandPinky2",
        "RightHandPinky3",
        "RightHandPinkyEnd",
        "RightJoystick",
        "LeftUpperLeg",
        "LeftLowerLeg",
        "LeftFoot",
        "LeftFootHeel",
        "LeftFootBall",
        "RightUpperLeg",
        "RightLowerLeg",
        "RightFoot",
        "RightFootHeel",
        "RightFootBall"
    };

    std::vector<std::unique_ptr<Skeleton>> stored_skeletons;
    std::vector<std::unique_ptr<SkeletonProcessor>> stored_processors;

    char defaultAxesDirections[] = { RIGHT, UP, FORWARD };
    char storedAxesDirections[] = { RIGHT, UP, FORWARD };
    char negatedDirections[] = { 0, 0, 0 };

    std::string getVersion()
    {
        return std::string(g_version);
    }

    std::string getStandardBoneName(YOST_SKELETON_BONE standard_bone)
    {
        return std::string(g_standard_bone_names[standard_bone]);
    }

    void setAxisDirections(int8_t* order, int8_t* negate)
    {
        uint8_t i = 0;
        for (i = 0; i < 3; i++)
        {
            storedAxesDirections[i] = order[i];
            negatedDirections[i] = negate[i];
            //printf("Axis D %d: %d", i, storedAxesDirections[i]);
        }
        axisDirectionsEnabled = true;
    }

    // Bone
    Bone::Bone()
    {
        _name = "";
        _device_id = 0xffffffff;
        _length = 0.0f;
        _mass = 0.0f;
        _update_type = YOST_SKELETON_BONE_UPDATE_NULL;
        _parent = nullptr;
        _has_updated = false;
    }

    Bone::~Bone()
    {
    }

    bool Bone::hasUpdated()
    {
        return _has_updated;
    }

    void Bone::resetUpdateFlag()
    {
        _has_updated = false;
    }

    void Bone::update()
    {
        _deltaTime = _prevTime.time_since_epoch();
        _prevTime = std::chrono::high_resolution_clock::now();
        float deltaTimeMS = (float)std::chrono::duration_cast<std::chrono::milliseconds>(_deltaTime).count();

        Orient par_orient;
        if (_parent != nullptr)
        {
            par_orient = _parent->getOrientation();
        }

        Orient finalOrient;

        //Update the orientation of the Bone if needed
        if (_update_type == YOST_SKELETON_BONE_UPDATE_FUSED)
        {
            orientMul(&par_orient, &_pose_orientation, &finalOrient);

        }
        else if (_update_type != YOST_SKELETON_BONE_UPDATE_NULL)
        {
            if (_update_type == YOST_SKELETON_BONE_UPDATE_INTERPOLATE)
            {
                // interpolate
                orientMul(&par_orient, &_pose_orientation, &finalOrient);
            }
            orientMul(&_offset_orientation, &_raw_orient, &finalOrient);
        }
        else
        {
            finalOrient = _pose_orientation;
        }

        _orientation = finalOrient;

        // Update the position of the Bone
        if (_parent != nullptr)
        {
            Vector3 curr_offset = _position_offset;
            Orient parent_orientation = _parent->getOrientation();
            orientRotate(&parent_orientation, &curr_offset);

            Vector3 parent_position = _parent->getPosition();
            vectorAdd(&parent_position, &curr_offset, &_position);
        }
        else
        {
            _position = _position_offset;
        }

        Vector3 _last_velocity = _velocity;

        // Calculate velocity
        vectorDiff(&_last_position, &_position, &_velocity);

        for (auto value : _velocity.data)
        {
            value = value / deltaTimeMS;
        }

        for (auto& child : _children)
        {
            child->update();
        }
        _has_updated = true;
    }
    void Bone::updatePosition(){
        // Update the position of the Bone
        if (_parent != nullptr)
        {
            Vector3 curr_offset = _position_offset;
            Orient parent_orientation = _parent->getOrientation();
            orientRotate(&parent_orientation, &curr_offset);

            Vector3 parent_position = _parent->getPosition();
            vectorAdd(&parent_position, &curr_offset, &_position);
        }
        else
        {
            _position = _position_offset;
        }

        for (auto& child : _children)
        {
            child->updatePosition();
        }
    }

    std::string Bone::createXMLHierarchy(uint32_t tabs)
    {
        std::string hierarchy = "";
        std::string tabstart = "";
        for (uint32_t i = 0; i < tabs; i++)
        {
            tabstart += "\t";
        }
        hierarchy += tabstart + "<bone NAME=\"" + _name + "\">\n";

        hierarchy += tabstart + ("\t<offset X=\"" + std::to_string(_position_offset.data[0]) + "\""
            " Y=\"" + std::to_string(_position_offset.data[1]) + "\""
            " Z=\"" + std::to_string(_position_offset.data[2]) + "\"/>\n");

        hierarchy += tabstart + ("\t<pose_orientation X=\"" + std::to_string(_pose_orientation.data[0]) + "\""
            " Y=\"" + std::to_string(_pose_orientation.data[1]) + "\""
            " Z=\"" + std::to_string(_pose_orientation.data[2]) + "\""
            " W=\"" + std::to_string(_pose_orientation.data[3]) + "\"/>\n");

        hierarchy += tabstart + "\t<length VALUE=\"" + std::to_string(_length) + "\"/>\n";

        hierarchy += tabstart + "\t<mass VALUE=\"" + std::to_string(_mass) + "\"/>\n";

        hierarchy += tabstart + "\t<update_type VALUE=\"" + yost_skeleton_bone_update_string[_update_type] + "\"/>\n";

        for (auto& child : _children)
        {
            hierarchy += child->createXMLHierarchy(tabs + 1);
        }
        hierarchy += tabstart + "</bone>\n";

        return hierarchy;
    }

    std::string Bone::getName()
    {
        return _name;
    }

    void Bone::setName(std::string bone_name)
    {
        _name = bone_name;
    }

    uint32_t Bone::getDeviceId()
    {
        return _device_id;
    }

    void Bone::setDeviceId(uint32_t device_id)
    {
        _device_id = device_id;
    }

    Orient Bone::getCalibrationOffset()
    {
        return _calibration_offset;
    }

    void Bone::setCalibrationOffset(Orient offset)
    {
        _calibration_offset = offset;
    }

    Orient Bone::getOrientationOffset()
    {
        return _offset_orientation;
    }

    void Bone::setOrientationOffset(Orient offset)
    {
        _offset_orientation = offset;
    }

    Orient Bone::getCalibrationTare()
    {
        return _calibration_tare;
    }

    void Bone::setCalibrationTare(Orient tare)
    {
        _calibration_tare = tare;
    }

    Orient Bone::getOrientation()
    {
        return _orientation;
    }

    void Bone::setOrientation(Orient orientation)
    {
        _orientation = orientation;
    }

    Orient Bone::getRawOrientation()
    {
        return _raw_orient;
    }

    void Bone::setRawOrientation(Orient orientation)
    {
        _raw_orient = orientation;
    }

    Vector3 Bone::getPositionOffset()
    {
        return _position_offset;
    }

    void Bone::setPositionOffset(Vector3 offset)
    {
        _position_offset = offset;
    }

    Vector3 Bone::getPosition()
    {
        return _position;
    }

    void Bone::setPosition(Vector3 position)
    {
        _position = position;
    }

    Vector3 Bone::getLastPosition()
    {
        return _last_position;
    }

    void Bone::setLastPosition(Vector3 last_position)
    {
        _last_position = last_position;
    }

    Orient Bone::getPoseOrientation()
    {
        return _pose_orientation;
    }

    void Bone::setPoseOrientation(Orient pose_orientation)
    {
        _pose_orientation = pose_orientation;
    }

    Vector3 Bone::getAcceleration()
    {
        return _acceleration;
    }

    void Bone::setAcceleration(Vector3 acceleration)
    {
        _acceleration = acceleration;
    }

    Vector3 Bone::getVelocity()
    {
        return _velocity;
    }

    void Bone::setVelocity(Vector3 velocity)
    {
        _velocity = velocity;
    }

    float Bone::getLength()
    {
        return _length;
    }

    void Bone::setLength(float length)
    {
        _length = length;
    }

    void Bone::setLengthFromChildren()
    {
        _length = 0.0f;
        uint32_t children_size = _children.size();
        if (children_size > 0)
        {
            for (auto& child : _children)
            {
                Vector3 child_position = child->getPositionOffset();
                _length += vectorLength(&child_position);
            }

            _length /= children_size;
        }
    }

    Vector3 Bone::calculateBoneCenter()
    {
        Vector3 center;
        if (_children.size() > 0)
        {
            for (auto& child : _children)
            {
                Vector3 child_position = child->getPositionOffset();
                vectorAdd(&center, &child_position, &center);
            }

            vectorNormalize(&center);
        }
        vectorMul(&center, _length / 2.0f);
        orientRotate(&_orientation, &center);
        vectorAdd(&_position, &center, &center);
        return center;
    }

    float Bone::getMass()
    {
        return _mass;
    }

    void Bone::setMass(float mass)
    {
        _mass = mass;
    }

    YOST_SKELETON_BONE_UPDATE_TYPE Bone::getUpdateType()
    {
        return _update_type;
    }

    void Bone::setUpdateType(YOST_SKELETON_BONE_UPDATE_TYPE type)
    {
        _update_type = type;
    }

    void Bone::setAllChildrenUpdateType(YOST_SKELETON_BONE_UPDATE_TYPE type)
    {
        auto children = getChildren();
        for (auto& child : children)
        {
            child->setUpdateType(type);
            child->setAllChildrenUpdateType(type);
        }
    }

    std::shared_ptr<Bone> Bone::getParent()
    {
        return _parent;
    }

    void Bone::setParent(std::shared_ptr<Bone>& parent)
    {
        _parent = parent;
    }

    std::vector<std::shared_ptr<Bone>> Bone::getChildren()
    {
        return _children;
    }

    void Bone::addChild(std::shared_ptr<Bone>& child)
    {
        _children.push_back(child);
    }

    void Bone::removeChild(std::shared_ptr<Bone>& child)
    {
        removeChild(child->getName());
    }

    void Bone::removeChild(std::string child_name)
    {
        auto child_it = std::find_if(_children.begin(), _children.end(), [&](const std::shared_ptr<Bone>& child) {return child->getName() == child_name; });
        if (child_it != _children.end())
        {
            _children.erase(child_it);
        }
    }

    // Skeleton
    Skeleton::Skeleton()
    {
        _name = "";
        _hierarchy = "";
        _device_xml_map = "";
		_unit = YOST_SKELETON_UNIT_CENTIMETERS;
        _calibrated = false;
        _poll_List.empty();
        _alias_map.empty();
        _reverse_alias_map.empty();
        _root_bone = nullptr;
        _pinned_bone = nullptr;
    }

    Skeleton::~Skeleton()
    {
        _processors.clear();
    }

    void Skeleton::createStandardSkeletonHierarchyWithAge(bool male, uint32_t age)
    {
        float height;

        //from wolfram alpha
        uint32_t lowest_user_age = 0;                       // approximate youngest age of a user
        uint32_t highest_height_age = 19;                   // approximate age at which people reach their maximum height

        float average_male_height_min_perc_50 = 1.0922f;    // in meters
        float average_female_height_min_perc_50 = 1.0668f;  // in meters

        float average_male_height_max_perc_50 = 1.6764f;    // in meters
        float average_female_height_max_perc_50 = 1.5748f;  // in meters

        if (male)
        {
            if (age >= highest_height_age)
            {
                height = average_male_height_max_perc_50;
            }
            else if (age > lowest_user_age)
            {
                float age_percent = (age - lowest_user_age) / (float)(highest_height_age - lowest_user_age);
                height = average_male_height_min_perc_50 + (average_male_height_max_perc_50 - average_male_height_min_perc_50) * age_percent;
            }
            else
            {
                return;
            }
        }
		else
		{
			if (age >= highest_height_age)
			{
				height = average_female_height_max_perc_50;
			}
			else if (age > lowest_user_age)
			{
				float age_percent = (age - lowest_user_age) / (float)(highest_height_age - lowest_user_age);
				height = average_female_height_min_perc_50 + (average_female_height_max_perc_50 - average_female_height_min_perc_50) * age_percent;
			}
			else
			{
				return;
			}
		}

        createStandardSkeletonHierarchy(male, height);
    }

    void Skeleton::createStandardSkeletonHierarchy(bool male, float height)
    {
        _height = height;
        if (height == 0)
        {
            return;
        }

        switch (_unit)
        {
			case YOST_SKELETON_UNIT_CENTIMETERS:
				height *= 100.0f;
				break;
			case YOST_SKELETON_UNIT_METERS:
				break;
			case YOST_SKELETON_UNIT_INCHES:
				height *= 39.37007874015748f;
				break;
			case YOST_SKELETON_UNIT_FEET:
				height *= 3.280839895013123f;
				break;
			default:
				break;
        }

        // The percentages are a combination from the following links/books (all percentages have been converted to a scale of 1 instead of 100)
        // http://www.exrx.net/Kinesiology/Segments.html
        // http://www.rusnauka.com/31_PRNT_2008/Tecnic/36223.doc.htm
        // Architectural Graphic Standards, Tenth Edition

        float head_and_neck_length = 0.1075f * height;
        float pelvis_length = 0.093f * height;
        float abdomen_length = 0.081f * height;
        float thorax_length = 0.127f * height;
        float hand_length = 0.1075f * height;
        float foot_length = 0.1525f * height;
        float upper_arm_length;
        float lower_arm_length;
        float upper_leg_length;
        float lower_leg_length;
        float shoulder_width;
        float pelvis_width;
        float hand_width;
        float ankle_height;

        //float lower_body_length;
        float total_spine_length = 0.0f;

        // The total mass of a standard skeleton is going to be 1 for simplicity
        float head_mass;
        float pelvis_mass;
        float abdomen_mass;
        float thorax_mass;
        float upper_arm_mass;
        float lower_arm_mass;
        float hand_mass;
        float upper_leg_mass;
        float lower_leg_mass;
        float foot_mass;

        // The finger ratios were determined from http://journals.tubitak.gov.tr/medical/issues/sag-12-42-3/sag-42-3-24-1006-858.pdf
        float d3 = hand_length / 2.0f;  // middle
        float d1;   // thumb
        float d2;   // index
        float d4;   // ring
        float d5;   // pinky
        float d1d3;
        float d2d3;
        float d4d3;
        float d5d3;

        if (male)
        {
            upper_arm_length = 0.172f * height;
            lower_arm_length = 0.157f * height;
            upper_leg_length = 0.232f * height;
            lower_leg_length = 0.247f * height;
            shoulder_width = 0.245f * height;
            pelvis_width = 0.113f * height;
            hand_width = 0.055f * height;
            ankle_height = 0.0508f * height;

            head_mass = 0.0826f;
            pelvis_mass = 0.1366f;
            abdomen_mass = 0.1306f;
            thorax_mass = 0.201f;
            upper_arm_mass = 0.0325f;
            lower_arm_mass = 0.0187f;
            hand_mass = 0.0065f;
            upper_leg_mass = 0.105f;
            lower_leg_mass = 0.0475f;
            foot_mass = 0.0143f;

            d1d3 = 0.84f;
            d2d3 = 0.91f;
            d4d3 = 0.93f;
            d5d3 = 0.76f;
        }
        else
        {
            upper_arm_length = 0.173f * height;
            lower_arm_length = 0.16f * height;
            upper_leg_length = 0.249f * height;
            lower_leg_length = 0.257f * height;
            shoulder_width = 0.2f * height;
            pelvis_width = 0.12f * height;
            hand_width = 0.048f * height;
            ankle_height = 0.048f * height;
            //lower_body_length = upper_leg_length + lower_leg_length + ankle_height;

            head_mass = 0.082f;
            pelvis_mass = 0.1596f;
            abdomen_mass = 0.1224f;
            thorax_mass = 0.1702f;
            upper_arm_mass = 0.029f;
            lower_arm_mass = 0.0157f;
            hand_mass = 0.005f;
            upper_leg_mass = 0.1175f;
            lower_leg_mass = 0.0535f;
            foot_mass = 0.0133f;

            d1d3 = 0.825f;
            d2d3 = 0.905f;
            d4d3 = 0.928f;
            d5d3 = 0.755f;
        }

        // Calculate finger sizes
        d1 = d3 * d1d3;
        d2 = d3 * d2d3;
        d4 = d3 * d4d3;
        d5 = d3 * d5d3;

        // Calculate offsets
        Vector3 hips_offset(0.0f, upper_leg_length + lower_leg_length + ankle_height, 0.0f);
        Vector3 spine_offset(0.0f, 1.0f, 0.0f);

        Orient tmp = Orient(-0.087156f, 0.0f, 0.0f, 0.996195f); // Rotate -10 degrees on X axis
        orientRotate(&tmp, &spine_offset);
        vectorMul(&spine_offset, abdomen_length);
        total_spine_length += spine_offset.data[1];
        Vector3 spine1_offset(0.0f, 1.0f, 0.0f);

        tmp = Orient(0.130526f, 0.0f, 0.0f, -0.991445f); // Rotate 15 degrees on X axis
        orientRotate(&tmp, &spine1_offset);
        vectorMul(&spine1_offset, (abdomen_length / 2.0f));
        total_spine_length += spine1_offset.data[1];
        Vector3 spine2_offset(0.0f, 1.0f, 0.0f);

        //tmp = Orient(0.130526f, 0.0f, 0.0f, -0.991445f); // Rotate 15 degrees on X axis
        //orientRotate(&tmp, &spine2_offset);
        vectorMul(&spine2_offset, pelvis_length);
        total_spine_length += spine2_offset.data[1];
        Vector3 spine_children_offset(0.0f, 0.84f, 0.0f);

        tmp = Orient(0.043619f, 0.0f, 0.0f, -0.999048f); // Rotate 5 degrees on X axis
        orientRotate(&tmp, &spine_children_offset);

        tmp = Orient(0.130526f, 0.0f, 0.0f, 0.991445f);
        Vector3 neck_offset(spine_children_offset);
        orientRotate(&tmp, &neck_offset);
        vectorMul(&neck_offset, thorax_length);
        tmp = Orient(0.087156f, 0.0f, 0.0f, 0.996195f); // Rotate -10 degrees on X axis
        Vector3 head_offset(0.0f, head_and_neck_length / 2.0f, 0.0f);
        orientRotate(&tmp, &head_offset);
        Vector3 head_end_offset(0.0f, head_and_neck_length / 2.0f, 0.0f);

        Vector3 left_shoulder_offset(spine_children_offset);
        vectorMul(&left_shoulder_offset, (thorax_length * 2.0f / 3.0f));

        Vector3 vector_tmp = Vector3(-(shoulder_width / 6.0f), 0.0f, 0.0f);
        vectorAdd(&left_shoulder_offset, &vector_tmp, &left_shoulder_offset);
        left_shoulder_offset.data[0] = -1;
        left_shoulder_offset.data[1] = 1;
        left_shoulder_offset.data[2] = 0;
        tmp = Orient(0.0f, 0.087156f, 0.0f, 0.996195f);
        Vector3 left_upper_arm_offset(-(shoulder_width / 2.0f), (thorax_length / 3.0f), 0.0f);
        orientRotate(&tmp, &left_upper_arm_offset);
        left_upper_arm_offset.data[0] += 1;
        Vector3 left_lower_arm_offset(-upper_arm_length, 0.0f, 0.0f);
        Vector3 left_hand_offset(-lower_arm_length, 0.0f, 0.0f);

        Vector3 left_hand_thumb_1_offset(-(hand_length / 40.0f), 0.0f, hand_width / 4.0f);
        Vector3 left_hand_thumb_2_offset(-d1, 0.0f, hand_width / 2.0f);
        Vector3 left_hand_thumb_3_offset(-(d1 / 2.0f), 0.0f, 0.0f);
        Vector3 left_hand_thumb_end_offset(-(d1 / 2.0f), 0.0f, 0.0f);
        Vector3 left_hand_index_1_offset(-d2, 0.0f, hand_width / 4.0f);
        Vector3 left_hand_index_2_offset(-(d2 / 2.0f), 0.0f, 0.0f);
        Vector3 left_hand_index_3_offset(-(d2 / 4.0f), 0.0f, 0.0f);
        Vector3 left_hand_index_end_offset(-(d2 / 4.0f), 0.0f, 0.0f);
        Vector3 left_hand_middle_1_offset(-d3, 0.0f, 0.0f);
        Vector3 left_hand_middle_2_offset(-(d3 / 2.0f), 0.0f, 0.0f);
        Vector3 left_hand_middle_3_offset(-(d3 / 4.0f), 0.0f, 0.0f);
        Vector3 left_hand_middle_end_offset(-(d3 / 4.0f), 0.0f, 0.0f);
        Vector3 left_hand_ring_1_offset(-d4, 0.0f, -(hand_width / 4.0f));
        Vector3 left_hand_ring_2_offset(-(d4 / 2.0f), 0.0f, 0.0f);
        Vector3 left_hand_ring_3_offset(-(d4 / 4.0f), 0.0f, 0.0f);
        Vector3 left_hand_ring_end_offset(-(d4 / 4.0f), 0.0f, 0.0f);
        Vector3 left_hand_pinky_1_offset(-d5, 0.0f, -(hand_width / 2.0f));
        Vector3 left_hand_pinky_2_offset(-(d5 / 2.0f), 0.0f, 0.0f);
        Vector3 left_hand_pinky_3_offset(-(d5 / 4.0f), 0.0f, 0.0f);
        Vector3 left_hand_pinky_end_offset(-(d5 / 4.0f), 0.0f, 0.0f);

        Vector3 right_shoulder_offset(spine_children_offset);
        vectorMul(&right_shoulder_offset, (thorax_length * 2.0f / 3.0f));
        vector_tmp = Vector3((shoulder_width / 6.0f), 0.0f, 0.0f);
        vectorAdd(&right_shoulder_offset, &vector_tmp, &right_shoulder_offset);
        right_shoulder_offset.data[0] = 1;
        right_shoulder_offset.data[1] = 1;
        right_shoulder_offset.data[2] = 0;
        tmp = Orient(0.0f, -0.087156f, 0.0f, 0.996195f);
        Vector3 right_upper_arm_offset((shoulder_width / 2.0f), (thorax_length / 3.0f), 0.0f);
        orientRotate(&tmp, &right_upper_arm_offset);
        right_upper_arm_offset.data[0] -= 1;
        Vector3 right_lower_arm_offset(upper_arm_length, 0.0f, 0.0f);
        Vector3 right_hand_offset(lower_arm_length, 0.0f, 0.0f);

        Vector3 right_hand_thumb_1_offset(hand_length / 40.0f, 0.0f, hand_width / 4.0f);
        Vector3 right_hand_thumb_2_offset(d1, 0.0f, hand_width / 2.0f);
        Vector3 right_hand_thumb_3_offset(d1 / 2.0f, 0.0f, 0.0f);
        Vector3 right_hand_thumb_end_offset(d1 / 2.0f, 0.0f, 0.0f);
        Vector3 right_hand_index_1_offset(d2, 0.0f, hand_width / 4.0f);
        Vector3 right_hand_index_2_offset(d2 / 2.0f, 0.0f, 0.0f);
        Vector3 right_hand_index_3_offset(d2 / 4.0f, 0.0f, 0.0f);
        Vector3 right_hand_index_end_offset(d2 / 4.0f, 0.0f, 0.0f);
        Vector3 right_hand_middle_1_offset(d3, 0.0f, 0.0f);
        Vector3 right_hand_middle_2_offset(d3 / 2.0f, 0.0f, 0.0f);
        Vector3 right_hand_middle_3_offset(d3 / 4.0f, 0.0f, 0.0f);
        Vector3 right_hand_middle_end_offset(d3 / 4.0f, 0.0f, 0.0f);
        Vector3 right_hand_ring_1_offset(d4, 0.0f, -(hand_width / 4.0f));
        Vector3 right_hand_ring_2_offset(d4 / 2.0f, 0.0f, 0.0f);
        Vector3 right_hand_ring_3_offset(d4 / 4.0f, 0.0f, 0.0f);
        Vector3 right_hand_ring_end_offset(d4 / 4.0f, 0.0f, 0.0f);
        Vector3 right_hand_pinky_1_offset(d5, 0.0f, -(hand_width / 2.0f));
        Vector3 right_hand_pinky_2_offset(d5 / 2.0f, 0.0f, 0.0f);
        Vector3 right_hand_pinky_3_offset(d5 / 4.0f, 0.0f, 0.0f);
        Vector3 right_hand_pinky_end_offset(d5 / 4.0f, 0.0f, 0.0f);

        tmp = Orient(0.130526f, 0.0f, 0.0f, -0.991445f);
        Vector3 left_upper_leg_offset(-(pelvis_width / 2.0f), -(pelvis_length / 2.0f), 0.0f);
        orientRotate(&tmp, &left_upper_leg_offset);
        tmp = Orient(0.0262f, 0.0f, 0.0f, 0.9997f);
        Vector3 left_lower_leg_offset(0.0f, -upper_leg_length, 0.0f);
        vectorMul(&left_lower_leg_offset, 6.0f / 5.0f);
        orientRotate(&tmp, &left_lower_leg_offset);
        tmp = Orient(0.0698f, 0.0f, 0.0f, 0.9976f);
        Vector3 left_foot_offset(0.0f, -lower_leg_length, 0.0f);
        orientRotate(&tmp, &left_foot_offset);
        Vector3 left_foot_ball_offset(0.0f, -ankle_height, foot_length * 2.0f / 3.0f);
        Vector3 left_foot_heel_offset(0.0f, -ankle_height, 0.0f);

        tmp = Orient(0.130526f, 0.0f, 0.0f, -0.991445f);
        Vector3 right_upper_leg_offset((pelvis_width / 2.0f), -(pelvis_length / 2.0f), 0.0f);
        orientRotate(&tmp, &right_upper_leg_offset);
        tmp = Orient(0.0262f, 0.0f, 0.0f, 0.9997f);
        Vector3 right_lower_leg_offset(0.0f, -upper_leg_length, 0.0f);
        vectorMul(&right_lower_leg_offset, 6.0f / 5.0f);
        orientRotate(&tmp, &right_lower_leg_offset);
        tmp = Orient(0.0698f, 0.0f, 0.0f, 0.9976f);
        Vector3 right_foot_offset(0.0f, -lower_leg_length, 0.0f);
        orientRotate(&tmp, &right_foot_offset);
        Vector3 right_foot_ball_offset(0.0f, -ankle_height, foot_length * 2.0f / 3.0f);
        Vector3 right_foot_heel_offset(0.0f, -ankle_height, 0.0f);

        // Create bones
        Vector3 vector_zero = Vector3(0, 0, 0);

        utilityAddBone(g_standard_bone_names[YOST_SKELETON_BONE_HIPS], YOST_SKELETON_BONE_UPDATE_NULL, hips_offset);
        utilityAddBone(g_standard_bone_names[YOST_SKELETON_BONE_LOWER_SPINE_2], YOST_SKELETON_BONE_UPDATE_NULL, spine2_offset, g_standard_bone_names[YOST_SKELETON_BONE_HIPS]);
        utilityAddBone(g_standard_bone_names[YOST_SKELETON_BONE_LOWER_SPINE_1], YOST_SKELETON_BONE_UPDATE_NULL, spine1_offset, g_standard_bone_names[YOST_SKELETON_BONE_LOWER_SPINE_2]);
        utilityAddBone(g_standard_bone_names[YOST_SKELETON_BONE_SPINE], YOST_SKELETON_BONE_UPDATE_NULL, spine_offset, g_standard_bone_names[YOST_SKELETON_BONE_LOWER_SPINE_1]);
        utilityAddBone(g_standard_bone_names[YOST_SKELETON_BONE_NECK], YOST_SKELETON_BONE_UPDATE_NULL, neck_offset, g_standard_bone_names[YOST_SKELETON_BONE_SPINE]);
        utilityAddBone(g_standard_bone_names[YOST_SKELETON_BONE_HEAD], YOST_SKELETON_BONE_UPDATE_NULL, head_offset, g_standard_bone_names[YOST_SKELETON_BONE_NECK]);
        utilityAddBone(g_standard_bone_names[YOST_SKELETON_BONE_HEAD_END], YOST_SKELETON_BONE_UPDATE_NULL, head_end_offset, g_standard_bone_names[YOST_SKELETON_BONE_HEAD]);

        utilityAddBone(g_standard_bone_names[YOST_SKELETON_BONE_LEFT_SHOULDER], YOST_SKELETON_BONE_UPDATE_NULL, left_shoulder_offset, g_standard_bone_names[YOST_SKELETON_BONE_SPINE]);
        utilityAddBone(g_standard_bone_names[YOST_SKELETON_BONE_LEFT_UPPER_ARM], YOST_SKELETON_BONE_UPDATE_NULL, left_upper_arm_offset, g_standard_bone_names[YOST_SKELETON_BONE_LEFT_SHOULDER]);
        utilityAddBone(g_standard_bone_names[YOST_SKELETON_BONE_LEFT_LOWER_ARM], YOST_SKELETON_BONE_UPDATE_NULL, left_lower_arm_offset, g_standard_bone_names[YOST_SKELETON_BONE_LEFT_UPPER_ARM]);
        utilityAddBone(g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND], YOST_SKELETON_BONE_UPDATE_NULL, left_hand_offset, g_standard_bone_names[YOST_SKELETON_BONE_LEFT_LOWER_ARM]);

        utilityAddBone(g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND_THUMB_1], YOST_SKELETON_BONE_UPDATE_NULL, left_hand_thumb_1_offset, g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND]);
        utilityAddBone(g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND_THUMB_2], YOST_SKELETON_BONE_UPDATE_NULL, left_hand_thumb_2_offset, g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND_THUMB_1]);
        utilityAddBone(g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND_THUMB_3], YOST_SKELETON_BONE_UPDATE_NULL, left_hand_thumb_3_offset, g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND_THUMB_2]);
        utilityAddBone(g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND_THUMB_END], YOST_SKELETON_BONE_UPDATE_NULL, left_hand_thumb_end_offset, g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND_THUMB_3]);

        utilityAddBone(g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND_INDEX_1], YOST_SKELETON_BONE_UPDATE_NULL, left_hand_index_1_offset, g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND]);
        utilityAddBone(g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND_INDEX_2], YOST_SKELETON_BONE_UPDATE_NULL, left_hand_index_2_offset, g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND_INDEX_1]);
        utilityAddBone(g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND_INDEX_3], YOST_SKELETON_BONE_UPDATE_NULL, left_hand_index_3_offset, g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND_INDEX_2]);
        utilityAddBone(g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND_INDEX_END], YOST_SKELETON_BONE_UPDATE_NULL, left_hand_index_end_offset, g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND_INDEX_3]);

        utilityAddBone(g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND_MIDDLE_1], YOST_SKELETON_BONE_UPDATE_NULL, left_hand_middle_1_offset, g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND]);
        utilityAddBone(g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND_MIDDLE_2], YOST_SKELETON_BONE_UPDATE_NULL, left_hand_middle_2_offset, g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND_MIDDLE_1]);
        utilityAddBone(g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND_MIDDLE_3], YOST_SKELETON_BONE_UPDATE_NULL, left_hand_middle_3_offset, g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND_MIDDLE_2]);
        utilityAddBone(g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND_MIDDLE_END], YOST_SKELETON_BONE_UPDATE_NULL, left_hand_middle_end_offset, g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND_MIDDLE_3]);

        utilityAddBone(g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND_RING_1], YOST_SKELETON_BONE_UPDATE_NULL, left_hand_ring_1_offset, g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND]);
        utilityAddBone(g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND_RING_2], YOST_SKELETON_BONE_UPDATE_NULL, left_hand_ring_2_offset, g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND_RING_1]);
        utilityAddBone(g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND_RING_3], YOST_SKELETON_BONE_UPDATE_NULL, left_hand_ring_3_offset, g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND_RING_2]);
        utilityAddBone(g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND_RING_END], YOST_SKELETON_BONE_UPDATE_NULL, left_hand_ring_end_offset, g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND_RING_3]);

        utilityAddBone(g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND_PINKY_1], YOST_SKELETON_BONE_UPDATE_NULL, left_hand_pinky_1_offset, g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND]);
        utilityAddBone(g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND_PINKY_2], YOST_SKELETON_BONE_UPDATE_NULL, left_hand_pinky_2_offset, g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND_PINKY_1]);
        utilityAddBone(g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND_PINKY_3], YOST_SKELETON_BONE_UPDATE_NULL, left_hand_pinky_3_offset, g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND_PINKY_2]);
        utilityAddBone(g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND_PINKY_END], YOST_SKELETON_BONE_UPDATE_NULL, left_hand_pinky_end_offset, g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND_PINKY_3]);

        utilityAddBone(g_standard_bone_names[YOST_SKELETON_BONE_LEFT_JOYSTICK], YOST_SKELETON_BONE_UPDATE_NULL, vector_zero, g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND]);

        utilityAddBone(g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_SHOULDER], YOST_SKELETON_BONE_UPDATE_NULL, right_shoulder_offset, g_standard_bone_names[YOST_SKELETON_BONE_SPINE]);
        utilityAddBone(g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_UPPER_ARM], YOST_SKELETON_BONE_UPDATE_NULL, right_upper_arm_offset, g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_SHOULDER]);
        utilityAddBone(g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_LOWER_ARM], YOST_SKELETON_BONE_UPDATE_NULL, right_lower_arm_offset, g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_UPPER_ARM]);
        utilityAddBone(g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND], YOST_SKELETON_BONE_UPDATE_NULL, right_hand_offset, g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_LOWER_ARM]);

        utilityAddBone(g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND_THUMB_1], YOST_SKELETON_BONE_UPDATE_NULL, right_hand_thumb_1_offset, g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND]);
        utilityAddBone(g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND_THUMB_2], YOST_SKELETON_BONE_UPDATE_NULL, right_hand_thumb_2_offset, g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND_THUMB_1]);
        utilityAddBone(g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND_THUMB_3], YOST_SKELETON_BONE_UPDATE_NULL, right_hand_thumb_3_offset, g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND_THUMB_2]);
        utilityAddBone(g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND_THUMB_END], YOST_SKELETON_BONE_UPDATE_NULL, right_hand_thumb_end_offset, g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND_THUMB_3]);

        utilityAddBone(g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND_INDEX_1], YOST_SKELETON_BONE_UPDATE_NULL, right_hand_index_1_offset, g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND]);
        utilityAddBone(g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND_INDEX_2], YOST_SKELETON_BONE_UPDATE_NULL, right_hand_index_2_offset, g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND_INDEX_1]);
        utilityAddBone(g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND_INDEX_3], YOST_SKELETON_BONE_UPDATE_NULL, right_hand_index_3_offset, g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND_INDEX_2]);
        utilityAddBone(g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND_INDEX_END], YOST_SKELETON_BONE_UPDATE_NULL, right_hand_index_end_offset, g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND_INDEX_3]);

        utilityAddBone(g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND_MIDDLE_1], YOST_SKELETON_BONE_UPDATE_NULL, right_hand_middle_1_offset, g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND]);
        utilityAddBone(g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND_MIDDLE_2], YOST_SKELETON_BONE_UPDATE_NULL, right_hand_middle_2_offset, g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND_MIDDLE_1]);
        utilityAddBone(g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND_MIDDLE_3], YOST_SKELETON_BONE_UPDATE_NULL, right_hand_middle_3_offset, g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND_MIDDLE_2]);
        utilityAddBone(g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND_MIDDLE_END], YOST_SKELETON_BONE_UPDATE_NULL, right_hand_middle_end_offset, g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND_MIDDLE_3]);

        utilityAddBone(g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND_RING_1], YOST_SKELETON_BONE_UPDATE_NULL, right_hand_ring_1_offset, g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND]);
        utilityAddBone(g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND_RING_2], YOST_SKELETON_BONE_UPDATE_NULL, right_hand_ring_2_offset, g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND_RING_1]);
        utilityAddBone(g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND_RING_3], YOST_SKELETON_BONE_UPDATE_NULL, right_hand_ring_3_offset, g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND_RING_2]);
        utilityAddBone(g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND_RING_END], YOST_SKELETON_BONE_UPDATE_NULL, right_hand_ring_end_offset, g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND_RING_3]);

        utilityAddBone(g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND_PINKY_1], YOST_SKELETON_BONE_UPDATE_NULL, right_hand_pinky_1_offset, g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND]);
        utilityAddBone(g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND_PINKY_2], YOST_SKELETON_BONE_UPDATE_NULL, right_hand_pinky_2_offset, g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND_PINKY_1]);
        utilityAddBone(g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND_PINKY_3], YOST_SKELETON_BONE_UPDATE_NULL, right_hand_pinky_3_offset, g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND_PINKY_2]);
        utilityAddBone(g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND_PINKY_END], YOST_SKELETON_BONE_UPDATE_NULL, right_hand_pinky_end_offset, g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND_PINKY_3]);

        utilityAddBone(g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_JOYSTICK], YOST_SKELETON_BONE_UPDATE_NULL, vector_zero, g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND]);

        utilityAddBone(g_standard_bone_names[YOST_SKELETON_BONE_LEFT_UPPER_LEG], YOST_SKELETON_BONE_UPDATE_NULL, left_upper_leg_offset, g_standard_bone_names[YOST_SKELETON_BONE_HIPS]);
        utilityAddBone(g_standard_bone_names[YOST_SKELETON_BONE_LEFT_LOWER_LEG], YOST_SKELETON_BONE_UPDATE_NULL, left_lower_leg_offset, g_standard_bone_names[YOST_SKELETON_BONE_LEFT_UPPER_LEG]);
        utilityAddBone(g_standard_bone_names[YOST_SKELETON_BONE_LEFT_FOOT], YOST_SKELETON_BONE_UPDATE_NULL, left_foot_offset, g_standard_bone_names[YOST_SKELETON_BONE_LEFT_LOWER_LEG]);
        utilityAddBone(g_standard_bone_names[YOST_SKELETON_BONE_LEFT_FOOT_BALL], YOST_SKELETON_BONE_UPDATE_NULL, left_foot_ball_offset, g_standard_bone_names[YOST_SKELETON_BONE_LEFT_FOOT]);
        utilityAddBone(g_standard_bone_names[YOST_SKELETON_BONE_LEFT_FOOT_HEEL], YOST_SKELETON_BONE_UPDATE_NULL, left_foot_heel_offset, g_standard_bone_names[YOST_SKELETON_BONE_LEFT_FOOT]);

        utilityAddBone(g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_UPPER_LEG], YOST_SKELETON_BONE_UPDATE_NULL, right_upper_leg_offset, g_standard_bone_names[YOST_SKELETON_BONE_HIPS]);
        utilityAddBone(g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_LOWER_LEG], YOST_SKELETON_BONE_UPDATE_NULL, right_lower_leg_offset, g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_UPPER_LEG]);
        utilityAddBone(g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_FOOT], YOST_SKELETON_BONE_UPDATE_NULL, right_foot_offset, g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_LOWER_LEG]);
        utilityAddBone(g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_FOOT_BALL], YOST_SKELETON_BONE_UPDATE_NULL, right_foot_ball_offset, g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_FOOT]);
        utilityAddBone(g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_FOOT_HEEL], YOST_SKELETON_BONE_UPDATE_NULL, right_foot_heel_offset, g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_FOOT]);

        // Set bone length
        utilitySetBoneLengths();

        // Set unique properties
        // Spine
        //utilitySetBoneDefaultOrient(g_bone_name_spine, Orient(-.7071f, 0, 0, .7071f));
        utilitySetBoneLength(g_standard_bone_names[YOST_SKELETON_BONE_SPINE], thorax_length);
        // Left and Right hand
        utilitySetBoneLength(g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND], hand_length);
        utilitySetBoneLength(g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND], hand_length);

        // Set bone mass
        utilitySetBoneMass(g_standard_bone_names[YOST_SKELETON_BONE_HIPS], pelvis_mass + abdomen_mass);
        utilitySetBoneMass(g_standard_bone_names[YOST_SKELETON_BONE_SPINE], thorax_mass);
        utilitySetBoneMass(g_standard_bone_names[YOST_SKELETON_BONE_HEAD], head_mass);

        utilitySetBoneMass(g_standard_bone_names[YOST_SKELETON_BONE_LEFT_UPPER_ARM], upper_arm_mass);
        utilitySetBoneMass(g_standard_bone_names[YOST_SKELETON_BONE_LEFT_LOWER_ARM], lower_arm_mass);
        utilitySetBoneMass(g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND], hand_mass);

        utilitySetBoneMass(g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_UPPER_ARM], upper_arm_mass);
        utilitySetBoneMass(g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_LOWER_ARM], lower_arm_mass);
        utilitySetBoneMass(g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND], hand_mass);

        utilitySetBoneMass(g_standard_bone_names[YOST_SKELETON_BONE_LEFT_UPPER_LEG], upper_leg_mass);
        utilitySetBoneMass(g_standard_bone_names[YOST_SKELETON_BONE_LEFT_LOWER_LEG], lower_leg_mass);
        utilitySetBoneMass(g_standard_bone_names[YOST_SKELETON_BONE_LEFT_FOOT], foot_mass);

        utilitySetBoneMass(g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_UPPER_LEG], upper_leg_mass);
        utilitySetBoneMass(g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_LOWER_LEG], lower_leg_mass);
        utilitySetBoneMass(g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_FOOT], foot_mass);

        // Build xml hierarchy
        utilityCreateXMLHierarchy();
    }

    YOST_SKELETON_ERROR Skeleton::loadSkeletonHierarchy(std::string hierarchy_file)
    {
        /*
        OLD WAY++++++++++++++++++++++
        // Read hierarchy
        std::FILE* file_ptr = std::fopen(hierarchy_file.c_str(), "r");
        if (file_ptr == nullptr)
        {
        return YOST_SKELETON_ERROR_FILE_READ;
        }
        std::string test = "";

        //Fseek sees end of lines a 2 char, need to remove those extra char some how or rewrite  - WILL WANT TO REDO LATER
        int ch, number_of_lines = 0;
        do
        {
        ch = fgetc(file_ptr);
        if (ch == '\n')
        number_of_lines++;
        } while (ch != EOF);

        std::fseek(file_ptr, 0, SEEK_END);
        int resizeNumber = std::ftell(file_ptr) - number_of_lines;
        test.resize(resizeNumber);
        std::rewind(file_ptr);
        std::fread(&test[0], sizeof(char), test.size(), file_ptr);
        std::fclose(file_ptr);
        */
        std::string test;
        std::ifstream file_ptr;
        file_ptr.open(hierarchy_file);
        if (file_ptr.is_open())
        {
            std::string line;
            test = "";
            while (std::getline(file_ptr, line))
            {
                test += line + "\n";
            }
        }
        else
        {
            return YOST_SKELETON_ERROR_FILE_READ;
        }
        file_ptr.close();


        // Build skeleton
        rapidxml::xml_document<> xml_doc;

        // This causes a memory access violation for some reason
        //memcpy(&_hierarchy[0], &test[0], test.size());

        // Parse the string using the rapidxml file parsing library into xml_doc, will mangle the file
        xml_doc.parse<0>(&test[0]);

        // Find our root node
        rapidxml::xml_node<>* root_node = xml_doc.first_node("yost");
        if (root_node == nullptr)
        {
            return YOST_SKELETON_ERROR_HIERARCHY_NO_ROOT_NODE;
        }

        // Get our skeleton node
        rapidxml::xml_node<>* skel_node = root_node->first_node("skeleton");
        if (skel_node == nullptr)
        {
            return YOST_SKELETON_ERROR_HIERARCHY_NO_SKELETON_NODE;
        }
        rapidxml::xml_attribute<>* xml_attr = skel_node->first_attribute("NAME");
        if (xml_attr == nullptr)
        {
            return YOST_SKELETON_ERROR_HIERARCHY_NODE_NO_NAME;
        }
        _name = xml_attr->value();
        xml_attr = skel_node->last_attribute("UNITS");
        if (xml_attr == nullptr)
        {
            return YOST_SKELETON_ERROR_HIERARCHY_NODE_NO_UNIT;
        }
        _unit = _convertStringToUnitType(xml_attr->value());

        // Get the root bone
        rapidxml::xml_node<>* bone_node = skel_node->first_node("bone");
        if (bone_node == nullptr)
        {
            return YOST_SKELETON_ERROR_HIERARCHY_NO_ROOT_BONE_NODE;
        }

        // Since memcpy causes a memory access violation, and the parser mangles the string reload into the varible- I not a fan of this but it's all I can think of at the moment  - WILL WANT TO REDO LATER
        _hierarchy = "";
        file_ptr.open(hierarchy_file);
        if (file_ptr.is_open())
        {
            std::string line;
            _hierarchy = "";
            while (std::getline(file_ptr, line))
            {
                _hierarchy += line + "\n";
            }
        }
        else
        {
            return YOST_SKELETON_ERROR_FILE_READ;
        }
        file_ptr.close();

        // Get our bone nodes
        return utilityCreateBoneFromHierarchy(bone_node);
    }

    YOST_SKELETON_ERROR Skeleton::addBoneAlias(std::string your_bone_name, std::string bone_name)
    {
        if (your_bone_name.size() == 0 || bone_name.size() == 0)
        {
            return YOST_SKELETON_ERROR_EMPTY_NAME;
        }

        auto name_it = _reverse_alias_map.find(bone_name);
        if (name_it == _reverse_alias_map.end())
        {
            return YOST_SKELETON_ERROR_BONE_NOT_FOUND;
        }

        _alias_map.erase(name_it->second);
        _alias_map[your_bone_name] = bone_name;
        _reverse_alias_map[bone_name] = your_bone_name;
        return YOST_SKELETON_NO_ERROR;
    }

    YOST_SKELETON_ERROR Skeleton::addProcessor(uint32_t index, yost_skeleton_id processor)
    {
        if (index > _processors.size())
        {
            if (index == YOST_SKELETON_PROCESSOR_LIST_END)
            {
                index = _processors.size();
            }
            else
            {
                return YOST_SKELETON_ERROR_INDEX_OUT_OF_RANGE;
            }
        }

        stored_processors[~(~processor | YOST_SKELETON_PROCESSOR_ID)]->setSkeleton(this);
        auto it = _processors.begin();
        for (uint32_t i = 0; i < index; i++)
        {
            it++;
        }
        _processors.insert(it, processor);
        return YOST_SKELETON_NO_ERROR;
    }

    YOST_SKELETON_ERROR Skeleton::removeProcessor(uint32_t index)
    {
        if (index > _processors.size() || _processors.size() == 0)
        {
            if (index == YOST_SKELETON_PROCESSOR_LIST_END)
            {
                index = _processors.size() - 1;
            }
            return YOST_SKELETON_ERROR_INDEX_OUT_OF_RANGE;
        }

        auto it = _processors.begin();
        for (uint32_t i = 0; i < index; i++)
        {
            it++;
        }
        _processors.erase(it);
        return YOST_SKELETON_NO_ERROR;
    }

    std::deque<yost_skeleton_id> Skeleton::getProcessorList()
    {
        return _processors;
    }

    std::vector<std::string> Skeleton::getProcessorNameList()
    {
        std::vector<std::string> processor_list;
        processor_list.resize(_processors.size());

        uint32_t i = 0;
        for (auto& processor : _processors)
        {
            processor_list[i] = stored_processors[~(~processor | YOST_SKELETON_PROCESSOR_ID)]->getProcessorTypeString();
            i++;
        }

        return processor_list;
    }

    YOST_SKELETON_ERROR Skeleton::update()
    {
        //store the last position
        for (auto& it : _bone_map)
        {
            it.second->setLastPosition(it.second->getPosition());
        }

        for (auto& processor : _processors)
        {
            stored_processors[~(~processor | YOST_SKELETON_PROCESSOR_ID)]->runProcess();
        }

        return YOST_SKELETON_NO_ERROR;
    }

    bool Skeleton::hasUpdated()
    {
        bool updated = false;
        for (auto& it : _bone_map)
        {
            updated = it.second->hasUpdated();
            it.second->resetUpdateFlag();
        }
        return updated;
    }

    std::string Skeleton::getPinnedBoneName()
    {
        if (_pinned_bone != nullptr)
        {
            return _reverse_alias_map[_pinned_bone->getName()];
        }

        return "";
    }

    std::shared_ptr<Bone> Skeleton::getPinnedBone()
    {
        return _pinned_bone;
    }

    void Skeleton::setPinnedBone(std::shared_ptr<Bone>* pin_bone)
    {
        _pinned_bone = *pin_bone;
    }

    std::vector<std::shared_ptr<Bone>> Skeleton::getPinBones()
    {
        return pinning_bones;
    }

    YOST_SKELETON_ERROR Skeleton::addPinBone(std::string pin_bone)
    {
        auto name_it = _alias_map.find(pin_bone);
        if (name_it == _alias_map.end())
        {
            return YOST_SKELETON_ERROR_BONE_NOT_FOUND;
        }
        pinning_bones.push_back(_bone_map[name_it->second]);
        return YOST_SKELETON_NO_ERROR;
    }

    std::string Skeleton::getRootBoneName()
    {
        if (_root_bone != nullptr)
        {
            return _alias_map[_root_bone->getName()];
        }

        return "";
    }

    std::shared_ptr<Bone> Skeleton::getRootBone()
    {
        return _root_bone;
    }

    std::vector<std::string> Skeleton::getBoneNameList()
    {
        std::vector<std::string> name_list;
        name_list.resize(_alias_map.size());

        uint32_t i = 0;
        for (auto& bone_name : _alias_map)
        {
            name_list[i] = bone_name.second;
            i++;
        }
        return name_list;
    }

    std::vector<std::shared_ptr<Bone>> Skeleton::getBoneList()
    {
        std::vector<std::shared_ptr<Bone>> bone_list;
        bone_list.resize(_bone_map.size());

        uint32_t i = 0;
        for (auto& bone : _bone_map)
        {
            bone_list[i] = bone.second;
            i++;
        }
        return bone_list;
    }

    std::map<std::string, Orient> Skeleton::getSkeletonPose()
    {
        return _skeleton_pose;
    }

    std::map<std::string, Orient> Skeleton::getBoneOffsets()
    {
        return _bone_offsets;
    }

    void Skeleton::setToStandardTPose()
    {
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_HIPS]] = Orient(-0.7071f, 0.0f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_LOWER_SPINE_2]] = Orient(-0.7071f, 0.0f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_LOWER_SPINE_1]] = Orient(-0.7071f, 0.0f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_SPINE]] = Orient(-0.7071f, 0.0f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_NECK]] = Orient(-0.7071f, 0.0f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_HEAD]] = Orient(-0.7071f, 0.0f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_HEAD_END]] = Orient();

        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_LEFT_SHOULDER]] = Orient(0.0f, -0.7071f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_LEFT_UPPER_ARM]] = Orient(0.0f, -0.7071f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_LEFT_LOWER_ARM]] = Orient(0.0f, -0.7071f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND]] = Orient(0.0f, -0.7071f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND_THUMB_1]] = Orient(0.0f, -0.7071f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND_THUMB_2]] = Orient(0.0f, -0.7071f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND_THUMB_3]] = Orient(0.0f, -0.7071f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND_THUMB_END]] = Orient();
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND_INDEX_1]] = Orient(0.0f, -0.7071f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND_INDEX_2]] = Orient(0.0f, -0.7071f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND_INDEX_3]] = Orient(0.0f, -0.7071f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND_INDEX_END]] = Orient();
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND_MIDDLE_1]] = Orient(0.0f, -0.7071f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND_MIDDLE_2]] = Orient(0.0f, -0.7071f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND_MIDDLE_3]] = Orient(0.0f, -0.7071f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND_MIDDLE_END]] = Orient();
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND_RING_1]] = Orient(0.0f, -0.7071f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND_RING_2]] = Orient(0.0f, -0.7071f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND_RING_3]] = Orient(0.0f, -0.7071f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND_RING_END]] = Orient();
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND_PINKY_1]] = Orient(0.0f, -0.7071f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND_PINKY_2]] = Orient(0.0f, -0.7071f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND_PINKY_3]] = Orient(0.0f, -0.7071f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND_PINKY_END]] = Orient();

        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_SHOULDER]] = Orient(0.0f, 0.7071f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_UPPER_ARM]] = Orient(0.0f, 0.7071f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_LOWER_ARM]] = Orient(0.0f, 0.7071f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND]] = Orient(0.0f, 0.7071f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND_THUMB_1]] = Orient(0.0f, 0.7071f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND_THUMB_2]] = Orient(0.0f, 0.7071f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND_THUMB_3]] = Orient(0.0f, 0.7071f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND_THUMB_END]] = Orient();
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND_INDEX_1]] = Orient(0.0f, 0.7071f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND_INDEX_2]] = Orient(0.0f, 0.7071f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND_INDEX_3]] = Orient(0.0f, 0.7071f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND_INDEX_END]] = Orient();
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND_MIDDLE_1]] = Orient(0.0f, 0.7071f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND_MIDDLE_2]] = Orient(0.0f, 0.7071f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND_MIDDLE_3]] = Orient(0.0f, 0.7071f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND_MIDDLE_END]] = Orient();
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND_RING_1]] = Orient(0.0f, 0.7071f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND_RING_2]] = Orient(0.0f, 0.7071f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND_RING_3]] = Orient(0.0f, 0.7071f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND_RING_END]] = Orient();
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND_PINKY_1]] = Orient(0.0f, 0.7071f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND_PINKY_2]] = Orient(0.0f, 0.7071f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND_PINKY_3]] = Orient(0.0f, 0.7071f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND_PINKY_END]] = Orient();

        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_LEFT_UPPER_LEG]] = Orient(0.7071f, 0.0f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_LEFT_LOWER_LEG]] = Orient(0.7071f, 0.0f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_LEFT_FOOT]] = Orient(0.7071f, 0.0f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_LEFT_FOOT_HEEL]] = Orient();
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_LEFT_FOOT_BALL]] = Orient();

        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_UPPER_LEG]] = Orient(0.7071f, 0.0f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_LOWER_LEG]] = Orient(0.7071f, 0.0f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_FOOT]] = Orient(0.7071f, 0.0f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_FOOT_HEEL]] = Orient();
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_FOOT_BALL]] = Orient();
    }

    void Skeleton::setToStandardClaspedPose()
    {
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_HIPS]] = Orient(-0.7071f, 0.0f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_LOWER_SPINE_2]] = Orient(-0.7071f, 0.0f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_LOWER_SPINE_1]] = Orient(-0.7071f, 0.0f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_SPINE]] = Orient(-0.7071f, 0.0f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_NECK]] = Orient(-0.7071f, 0.0f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_HEAD]] = Orient(-0.7071f, 0.0f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_HEAD_END]] = Orient();

        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_LEFT_SHOULDER]] = Orient(0.0f, -0.7071f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_LEFT_UPPER_ARM]] = Orient();
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_LEFT_LOWER_ARM]] = Orient();
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND]] = Orient();
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND_THUMB_1]] = Orient();
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND_THUMB_2]] = Orient();
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND_THUMB_3]] = Orient();
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND_THUMB_END]] = Orient();
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND_INDEX_1]] = Orient();
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND_INDEX_2]] = Orient();
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND_INDEX_3]] = Orient();
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND_INDEX_END]] = Orient();
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND_MIDDLE_1]] = Orient();
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND_MIDDLE_2]] = Orient();
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND_MIDDLE_3]] = Orient();
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND_MIDDLE_END]] = Orient();
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND_RING_1]] = Orient();
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND_RING_2]] = Orient();
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND_RING_3]] = Orient();
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND_RING_END]] = Orient();
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND_PINKY_1]] = Orient();
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND_PINKY_2]] = Orient();
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND_PINKY_3]] = Orient();
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND_PINKY_END]] = Orient();

        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_SHOULDER]] = Orient(0.0f, 0.7071f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_UPPER_ARM]] = Orient();
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_LOWER_ARM]] = Orient();
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND]] = Orient();
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND_THUMB_1]] = Orient();
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND_THUMB_2]] = Orient();
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND_THUMB_3]] = Orient();
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND_THUMB_END]] = Orient();
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND_INDEX_1]] = Orient();
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND_INDEX_2]] = Orient();
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND_INDEX_3]] = Orient();
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND_INDEX_END]] = Orient();
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND_MIDDLE_1]] = Orient();
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND_MIDDLE_2]] = Orient();
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND_MIDDLE_3]] = Orient();
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND_MIDDLE_END]] = Orient();
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND_RING_1]] = Orient();
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND_RING_2]] = Orient();
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND_RING_3]] = Orient();
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND_RING_END]] = Orient();
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND_PINKY_1]] = Orient();
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND_PINKY_2]] = Orient();
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND_PINKY_3]] = Orient();
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND_PINKY_END]] = Orient();

        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_LEFT_UPPER_LEG]] = Orient(0.7071f, 0.0f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_LEFT_LOWER_LEG]] = Orient(0.7071f, 0.0f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_LEFT_FOOT]] = Orient(0.7071f, 0.0f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_LEFT_FOOT_HEEL]] = Orient();
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_LEFT_FOOT_BALL]] = Orient();

        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_UPPER_LEG]] = Orient(0.7071f, 0.0f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_LOWER_LEG]] = Orient(0.7071f, 0.0f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_FOOT]] = Orient(0.7071f, 0.0f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_FOOT_HEEL]] = Orient();
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_FOOT_BALL]] = Orient();
    }

    void Skeleton::setToStandardNeutralPose()
    {
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_HIPS]] = Orient(-0.7071f, 0.0f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_LOWER_SPINE_2]] = Orient(-0.7071f, 0.0f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_LOWER_SPINE_1]] = Orient(-0.7071f, 0.0f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_SPINE]] = Orient(-0.7071f, 0.0f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_NECK]] = Orient(-0.7071f, 0.0f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_HEAD]] = Orient(-0.7071f, 0.0f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_HEAD_END]] = Orient();

        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_LEFT_SHOULDER]] = Orient(0.0f, -0.7071f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_LEFT_UPPER_ARM]] = Orient(0.7071f, 0.0f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_LEFT_UPPER_ARM]] = Orient(0.7071f, 0.0f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_LEFT_LOWER_ARM]] = Orient(0.7071f, 0.0f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND]] = Orient(0.7071f, 0.0f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND_THUMB_1]] = Orient(0.7071f, 0.0f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND_THUMB_2]] = Orient(0.7071f, 0.0f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND_THUMB_3]] = Orient(0.7071f, 0.0f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND_THUMB_END]] = Orient();
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND_INDEX_1]] = Orient(0.7071f, 0.0f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND_INDEX_2]] = Orient(0.7071f, 0.0f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND_INDEX_3]] = Orient(0.7071f, 0.0f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND_INDEX_END]] = Orient();
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND_MIDDLE_1]] = Orient(0.7071f, 0.0f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND_MIDDLE_2]] = Orient(0.7071f, 0.0f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND_MIDDLE_3]] = Orient(0.7071f, 0.0f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND_MIDDLE_END]] = Orient();
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND_RING_1]] = Orient(0.7071f, 0.0f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND_RING_2]] = Orient(0.7071f, 0.0f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND_RING_3]] = Orient(0.7071f, 0.0f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND_RING_END]] = Orient();
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND_PINKY_1]] = Orient(0.7071f, 0.0f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND_PINKY_2]] = Orient(0.7071f, 0.0f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND_PINKY_3]] = Orient(0.7071f, 0.0f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_LEFT_HAND_PINKY_END]] = Orient();

        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_SHOULDER]] = Orient(0.0f, 0.7071f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_UPPER_ARM]] = Orient(0.7071f, 0.0f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_UPPER_ARM]] = Orient(0.7071f, 0.0f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_LOWER_ARM]] = Orient(0.7071f, 0.0f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND]] = Orient(0.7071f, 0.0f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND_THUMB_1]] = Orient(0.7071f, 0.0f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND_THUMB_2]] = Orient(0.7071f, 0.0f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND_THUMB_3]] = Orient(0.7071f, 0.0f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND_THUMB_END]] = Orient();
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND_INDEX_1]] = Orient(0.7071f, 0.0f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND_INDEX_2]] = Orient(0.7071f, 0.0f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND_INDEX_3]] = Orient(0.7071f, 0.0f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND_INDEX_END]] = Orient();
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND_MIDDLE_1]] = Orient(0.7071f, 0.0f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND_MIDDLE_2]] = Orient(0.7071f, 0.0f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND_MIDDLE_3]] = Orient(0.7071f, 0.0f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND_MIDDLE_END]] = Orient();
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND_RING_1]] = Orient(0.7071f, 0.0f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND_RING_2]] = Orient(0.7071f, 0.0f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND_RING_3]] = Orient(0.7071f, 0.0f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND_RING_END]] = Orient();
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND_PINKY_1]] = Orient(0.7071f, 0.0f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND_PINKY_2]] = Orient(0.7071f, 0.0f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND_PINKY_3]] = Orient(0.7071f, 0.0f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_HAND_PINKY_END]] = Orient();

        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_LEFT_UPPER_LEG]] = Orient(0.7071f, 0.0f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_LEFT_LOWER_LEG]] = Orient(0.7071f, 0.0f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_LEFT_FOOT]] = Orient(0.7071f, 0.0f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_LEFT_FOOT_HEEL]] = Orient();
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_LEFT_FOOT_BALL]] = Orient();

        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_UPPER_LEG]] = Orient(0.7071f, 0.0f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_LOWER_LEG]] = Orient(0.7071f, 0.0f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_FOOT]] = Orient(0.7071f, 0.0f, 0.0f, 0.7071f);
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_FOOT_HEEL]] = Orient();
        _skeleton_pose[g_standard_bone_names[YOST_SKELETON_BONE_RIGHT_FOOT_BALL]] = Orient();
    }

    std::string Skeleton::extractHierarchy()
    {
        return _hierarchy;
    }

    bool Skeleton::isCalibrated()
    {
        return _calibrated;
    }

    void Skeleton::setCalibration(bool calibration)
    {
        _calibrated = calibration;
    }

    std::string Skeleton::getName()
    {
        return _name;
    }

    void Skeleton::setName(std::string skeleton_name)
    {
        _name = skeleton_name;
    }

    YOST_SKELETON_UNIT Skeleton::getSkeletonUnit()
    {
        return _unit;
    }

    float Skeleton::getHeight()
    {
        return _height;
    }

    void Skeleton::setSkeletonUnit(YOST_SKELETON_UNIT unit_type)
    {
		if (_unit == unit_type)
		{
			return; //Already that unit nothing to do
		}
		YOST_SKELETON_UNIT old = _unit;
		float cF = 1;
		
		if (old == YOST_SKELETON_UNIT_METERS)
		{
			cF = 100;
		}
		else if (old == YOST_SKELETON_UNIT_INCHES)
		{
			cF = 2.54f;
		}
		else if (old == YOST_SKELETON_UNIT_FEET)
		{
			cF = 30.48f;
		}
        
		_unit = unit_type;
		
		
		if (_unit == YOST_SKELETON_UNIT_METERS)
		{
			cF *= .01f;
		}
		else if (_unit == YOST_SKELETON_UNIT_INCHES)
		{
			cF *= 1/2.54f;
		}
		else if (_unit == YOST_SKELETON_UNIT_FEET)
		{
			cF *= 1/30.48f;
		}

		auto bone = _bone_map.begin();
		while (bone != _bone_map.end())
		{
			Vector3 pos = _bone_map[bone->first]->getPositionOffset();
			pos = Vector3(pos.data[0] * cF, pos.data[1] * cF, pos.data[2] * cF);
			_bone_map[bone->first]->setPositionOffset(pos);
			bone++;
		}

		_root_bone->updatePosition();
    }

    std::shared_ptr<Bone> Skeleton::getBone(std::string bone_name)
    {
        auto name_it = _alias_map.find(bone_name);
        if (name_it == _alias_map.end())
        {
            name_it = _reverse_alias_map.find(bone_name);
            if (name_it == _reverse_alias_map.end())
            {
                return nullptr;
            }
            return _bone_map[name_it->first];
        }
        return _bone_map[name_it->second];
    }

    YOST_SKELETON_ERROR Skeleton::setBoneOrientationOffset(std::string bone_name, Orient offset)
    {
        if (axisDirectionsEnabled)
        {
            orientConvertAxes(&offset, storedAxesDirections, defaultAxesDirections, negatedDirections);
        }
        getBone(bone_name)->setOrientationOffset(offset);

        return YOST_SKELETON_NO_ERROR;
    }

    YOST_SKELETON_ERROR Skeleton::getBoneOrientationOffset(std::string bone_name, Orient* orient)
    {
        auto name_it = _alias_map.find(bone_name);
        if (name_it == _alias_map.end())
        {
            name_it = _reverse_alias_map.find(bone_name);
            if (name_it == _reverse_alias_map.end())
            {
                return YOST_SKELETON_ERROR_BONE_NOT_FOUND;
            }
        }

        Orient boneOrient = _bone_map[name_it->second]->getOrientationOffset();

        if (axisDirectionsEnabled)
        {
            orientConvertAxes(&boneOrient, defaultAxesDirections, storedAxesDirections, negatedDirections);
        }

        *orient = boneOrient;

        return YOST_SKELETON_NO_ERROR;
    }

    YOST_SKELETON_ERROR Skeleton::getBoneOrientation(std::string bone_name, Orient* orient)
    {
        auto name_it = _alias_map.find(bone_name);
        if (name_it == _alias_map.end())
        {
            name_it = _reverse_alias_map.find(bone_name);
            if (name_it == _reverse_alias_map.end())
            {
                return YOST_SKELETON_ERROR_BONE_NOT_FOUND;
            }
        }

        Orient boneOrient = _bone_map[name_it->second]->getOrientation();
        if (axisDirectionsEnabled)
        {
            orientConvertAxes(&boneOrient, defaultAxesDirections, storedAxesDirections, negatedDirections);
        }
        *orient = boneOrient;

        return YOST_SKELETON_NO_ERROR;
    }

    YOST_SKELETON_ERROR Skeleton::getBonePosition(std::string bone_name, Vector3* pos)
    {
        auto name_it = _alias_map.find(bone_name);
        if (name_it == _alias_map.end())
        {
            return YOST_SKELETON_ERROR_BONE_NOT_FOUND;
        }

        *pos = _bone_map[name_it->second]->getPosition();
        return YOST_SKELETON_NO_ERROR;
    }

    YOST_SKELETON_ERROR Skeleton::setBonePositionOffset(std::string bone_name, Vector3 pos_offset)
    {
        auto name_it = _alias_map.find(bone_name);
        if (name_it == _alias_map.end())
        {
            return YOST_SKELETON_ERROR_BONE_NOT_FOUND;
        }

        _bone_map[name_it->second]->setPositionOffset(pos_offset);
        return YOST_SKELETON_NO_ERROR;
    }

    YOST_SKELETON_ERROR Skeleton::getBonePositionOffset(std::string bone_name, Vector3* pos_offset)
    {
        auto name_it = _alias_map.find(bone_name);
        if (name_it == _alias_map.end())
        {
            return YOST_SKELETON_ERROR_BONE_NOT_FOUND;
        }

        *pos_offset = _bone_map[name_it->second]->getPositionOffset();
        return YOST_SKELETON_NO_ERROR;
    }

    YOST_SKELETON_ERROR Skeleton::getBoneCenter(std::string bone_name, Vector3* pos)
    {
        auto name_it = _alias_map.find(bone_name);
        if (name_it == _alias_map.end())
        {
            return YOST_SKELETON_ERROR_BONE_NOT_FOUND;
        }

        Vector3 bonePos = _bone_map[name_it->second]->getPosition();

        if (axisDirectionsEnabled)
        {
            vectorConvertAxes(&bonePos, defaultAxesDirections, storedAxesDirections, negatedDirections);
        }

        *pos = bonePos;
        return YOST_SKELETON_NO_ERROR;
    }

    YOST_SKELETON_ERROR Skeleton::getBoneVelocity(std::string bone_name, Vector3* vel)
    {
        auto name_it = _alias_map.find(bone_name);
        if (name_it == _alias_map.end())
        {
            return YOST_SKELETON_ERROR_BONE_NOT_FOUND;
        }

        Vector3 boneVel = _bone_map[name_it->second]->getVelocity();

        if (axisDirectionsEnabled)
        {
            vectorConvertAxes(&boneVel, defaultAxesDirections, storedAxesDirections, negatedDirections);
        }

        *vel = boneVel;
        return YOST_SKELETON_NO_ERROR;
    }

    YOST_SKELETON_ERROR Skeleton::getBoneAcceleration(std::string bone_name, Vector3* accel)
    {
        auto name_it = _alias_map.find(bone_name);
        if (name_it == _alias_map.end())
        {
            return YOST_SKELETON_ERROR_BONE_NOT_FOUND;
        }

        Vector3 boneAccel = _bone_map[name_it->second]->getAcceleration();

        if (axisDirectionsEnabled)
        {
            vectorConvertAxes(&boneAccel, defaultAxesDirections, storedAxesDirections, negatedDirections);
        }

        *accel = boneAccel;
        return YOST_SKELETON_NO_ERROR;
    }

    YOST_SKELETON_ERROR Skeleton::getAllBoneOrientationOffsets(float* offset)
    {
        int i = 0;
        for (auto bone : _poll_List)
        {
            Orient boneOffset = bone->getOrientationOffset();

            if (axisDirectionsEnabled)
            {
                orientConvertAxes(&boneOffset, defaultAxesDirections, storedAxesDirections, negatedDirections);
            }

            offset[i] = boneOffset.data[0];
            offset[i + 1] = boneOffset.data[1];
            offset[i + 2] = boneOffset.data[2];
            offset[i + 3] = boneOffset.data[3];
            i += 4;
        }
        return YOST_SKELETON_NO_ERROR;
    }

    YOST_SKELETON_ERROR Skeleton::getAllBoneOrientations(float* orient)
    {
        int i = 0;
        for (auto bone : _poll_List)
        {

            Orient boneOrient = bone->getOrientation();

            if (axisDirectionsEnabled)
            {
                orientConvertAxes(&boneOrient, defaultAxesDirections, storedAxesDirections, negatedDirections);
            }

            orient[i] = boneOrient.data[0];
            orient[i + 1] = boneOrient.data[1];
            orient[i + 2] = boneOrient.data[2];
            orient[i + 3] = boneOrient.data[3];
            i += 4;
        }
        return YOST_SKELETON_NO_ERROR;
    }

    YOST_SKELETON_ERROR Skeleton::getAllBonePositions(float* pos)
    {
        int i = 0;
        for (auto bone : _poll_List)
        {
            Vector3 bonePos = bone->getPosition();
            pos[i] = bonePos.data[0];
            pos[i + 1] = bonePos.data[1];
            pos[i + 2] = bonePos.data[2];
            i += 3;
        }
        return YOST_SKELETON_NO_ERROR;
    }

    YOST_SKELETON_ERROR Skeleton::getAllBoneVelocitites(float* vel)
    {
        int i = 0;
        for (auto bone : _poll_List)
        {
            Vector3 boneVel = bone->getVelocity();
            if (axisDirectionsEnabled)
            {
                vectorConvertAxes(&boneVel, defaultAxesDirections, storedAxesDirections, negatedDirections);
            }

            vel[i] = boneVel.data[0];
            vel[i + 1] = boneVel.data[1];
            vel[i + 2] = boneVel.data[2];
            i += 3;
        }
        return YOST_SKELETON_NO_ERROR;

    }

    YOST_SKELETON_ERROR Skeleton::getAllBoneAccelerations(float* accel)
    {
        int i = 0;
        for (auto bone : _poll_List)
        {
            Vector3 boneAccel = bone->getAcceleration();

            if (axisDirectionsEnabled)
            {
                vectorConvertAxes(&boneAccel, defaultAxesDirections, storedAxesDirections, negatedDirections);
            }
            accel[i] = boneAccel.data[0];
            accel[i + 1] = boneAccel.data[1];
            accel[i + 2] = boneAccel.data[2];
            i += 3;
        }
        return YOST_SKELETON_NO_ERROR;
    }

    YOST_SKELETON_ERROR Skeleton::utilityRegisterBone(std::string bone_name)
    {
        auto name_it = _alias_map.find(bone_name);
        if (name_it == _alias_map.end())
        {
            return YOST_SKELETON_ERROR_BONE_NOT_FOUND;
        }

        _poll_List.push_back(_bone_map[name_it->second]);
        return YOST_SKELETON_NO_ERROR;
    }

    YOST_SKELETON_ERROR Skeleton::utilityDeregisterBone(std::string bone_name)
    {
        for (int i = 0; i < _poll_List.size(); i++)
        {
            if (_poll_List[i] == _bone_map[_alias_map.find(bone_name)->second])
            {
                _poll_List.erase(_poll_List.begin() + i);
                return YOST_SKELETON_NO_ERROR;
            }
        }
        return YOST_SKELETON_ERROR_BONE_NOT_FOUND;
    }

    std::vector<std::string> Skeleton::utilityRegisteredBoneList()
    {
        std::vector<std::string> bone_list;
        for (int i = 0; i < _poll_List.size(); i++)
        {
            std::string tmp = _poll_List[i]->getName();
            std::string name;
            for (std::map<std::string,std::string>::iterator it = _alias_map.begin(); it != _alias_map.end(); ++it)
            {
                if (it->second == tmp)
                {
                    name = it->first;
                }
            }

            bone_list.push_back(name);
        }
        return bone_list;
    }

    U32 Skeleton::utilityRegisteredBoneCount()
    {
        return _poll_List.size();
    }

    YOST_SKELETON_ERROR Skeleton::utilityGetBoneLength(std::string bone_name, float* length)
    {
        auto name_it = _alias_map.find(bone_name);
        if (name_it == _alias_map.end())
        {
            return YOST_SKELETON_ERROR_BONE_NOT_FOUND;
        }

        *length = _bone_map[name_it->second]->getLength();
        return YOST_SKELETON_NO_ERROR;
    }

    YOST_SKELETON_ERROR Skeleton::getBoneMass(std::string bone_name, float* mass)
    {
        auto name_it = _alias_map.find(bone_name);
        if (name_it == _alias_map.end())
        {
            return YOST_SKELETON_ERROR_BONE_NOT_FOUND;
        }

        *mass = _bone_map[name_it->second]->getMass();
        return YOST_SKELETON_NO_ERROR;
    }



    void Skeleton::utilityCreateXMLHierarchy()
    {
        _hierarchy = "";
        _hierarchy += "<yost>\n";
        _hierarchy += "\t<skeleton NAME=\"" + _name + "\" UNITS=\"" + yost_skeleton_unit_string[_unit] + "\">\n";
        _hierarchy += _root_bone->createXMLHierarchy(2);
        _hierarchy += "\t</skeleton>\n";
        _hierarchy += "</yost>";
    }

    void Skeleton::utilitySaveXMLHierarchy(std::string file_name)
    {
        std::ofstream myfile;
        myfile.open(file_name, std::ios::out | std::ios::trunc);
        myfile << _hierarchy;
        myfile.close();
    }

    YOST_SKELETON_ERROR Skeleton::utilityAddBone(std::string bone_name, YOST_SKELETON_BONE_UPDATE_TYPE update_type, Vector3 offset, std::string parent_name)
    {
        if (bone_name == "")
        {
            // bone_name is empty, not allowed
            return YOST_SKELETON_ERROR_EMPTY_NAME;
        }

        if (_bone_map.find(bone_name) != _bone_map.end())
        {
            // bone_name already exists
            return YOST_SKELETON_ERROR_DUPLICATE_BONE;
        }

        _alias_map[bone_name] = bone_name;
        _reverse_alias_map[bone_name] = bone_name;

        std::shared_ptr<Bone> new_bone(new Bone());
        new_bone->setName(bone_name);
        new_bone->setUpdateType(update_type);
        new_bone->setPositionOffset(offset);

        _bone_map.emplace(bone_name, new_bone);


        if (parent_name != "")
        {
            std::string p_name = _alias_map[parent_name];
            if (_bone_map.find(p_name) == _bone_map.end())
            {
                //parent doesn't exist
                _bone_map.erase(bone_name);
                return YOST_SKELETON_ERROR_PARENT_BONE_NOT_FOUND;
            }

            new_bone->setParent(_bone_map[p_name]);
            _bone_map[p_name]->addChild(new_bone);
        }
        else
        {
            _root_bone = new_bone;
        }

        return YOST_SKELETON_NO_ERROR;
    }

    YOST_SKELETON_ERROR Skeleton::utilityRemoveBone(std::string bone_name)
    {
        if (bone_name == "")
        {
            // bone_name is empty, not allowed
            return YOST_SKELETON_ERROR_EMPTY_NAME;
        }

        auto name_it = _bone_map.find(bone_name);
        if (name_it == _bone_map.end())
        {
            // bone_name not found
            return YOST_SKELETON_ERROR_BONE_NOT_FOUND;
        }

        // Add out childern to our parent
        Vector3 bone_offset = name_it->second->getPositionOffset();
        for (auto child : name_it->second->getChildren())
        {
            // Recalculate our childerns offset 
            Vector3 old_offset = child->getPositionOffset();
            Vector3 new_offset = Vector3();

            new_offset.data[0] = bone_offset.data[0] + old_offset.data[0];
            new_offset.data[1] = bone_offset.data[1] + old_offset.data[1];
            new_offset.data[2] = bone_offset.data[2] + old_offset.data[2];

            child->setPositionOffset(new_offset);

            name_it->second->getParent()->addChild(child);
        }

        // Remove the bone from it's parent
        name_it->second->getParent()->removeChild(name_it->second);

        // Erase the bone from the maps, and deregister from polling
        utilityDeregisterBone(bone_name);
        _reverse_alias_map.erase(bone_name);
        _alias_map.erase(bone_name);
        _bone_map.erase(name_it);

        return YOST_SKELETON_NO_ERROR;
    }

    YOST_SKELETON_ERROR Skeleton::utilitySetBonePoseOrientation(std::string bone_name, Orient pose)
    {
        auto name_it = _alias_map.find(bone_name);
        if (name_it == _alias_map.end())
        {
            return YOST_SKELETON_ERROR_BONE_NOT_FOUND;
        }

        _bone_map[name_it->second]->setPoseOrientation(pose);
        return YOST_SKELETON_NO_ERROR;
    }

    YOST_SKELETON_ERROR Skeleton::utilitySetBoneLength(std::string bone_name, float length)
    {
        auto name_it = _alias_map.find(bone_name);
        if (name_it == _alias_map.end())
        {
            return YOST_SKELETON_ERROR_BONE_NOT_FOUND;
        }

        _bone_map[name_it->second]->setLength(length);
        return YOST_SKELETON_NO_ERROR;
    }

    void Skeleton::utilitySetBoneLengths()
    {
        for (auto& bone : _bone_map)
        {
            bone.second->setLengthFromChildren();
        }
    }

    YOST_SKELETON_ERROR Skeleton::utilitySetBoneMass(std::string bone_name, float mass)
    {
        auto name_it = _alias_map.find(bone_name);
        if (name_it == _alias_map.end())
        {
            return YOST_SKELETON_ERROR_BONE_NOT_FOUND;
        }

        _bone_map[name_it->second]->setMass(mass);
        return YOST_SKELETON_NO_ERROR;
    }

    YOST_SKELETON_ERROR Skeleton::utilityGetBoneMass(std::string bone_name, float* mass)
    {
        auto name_it = _alias_map.find(bone_name);
        if (name_it == _alias_map.end())
        {
            return YOST_SKELETON_ERROR_BONE_NOT_FOUND;
        }

        *mass = _bone_map[name_it->second]->getMass();
        return YOST_SKELETON_NO_ERROR;
    }

    YOST_SKELETON_ERROR Skeleton::utilityCreateBoneFromHierarchy(rapidxml::xml_node<>* node, std::string parent_name)
    {
        rapidxml::xml_attribute<>* xml_attr = node->first_attribute("NAME");
        if (xml_attr == nullptr)
        {
            return YOST_SKELETON_ERROR_HIERARCHY_NODE_NO_NAME;
        }
        std::string name = xml_attr->value();

        Vector3 offset;
        if (!yost::_extractVectorFromNode(node->first_node("offset"), &offset))
        {
            return YOST_SKELETON_ERROR_HIERARCHY_NODE_NO_OFFSET;
        }

        Orient pose;
        if (!yost::_extractQuaternionFromNode(node->first_node("pose_orientation"), &pose))
        {
            return YOST_SKELETON_ERROR_HIERARCHY_NODE_NO_POSE_ORIENTATION;
        }

        float length;
        if (!_extractFloatFromNode(node->first_node("length"), &length))
        {
            return YOST_SKELETON_ERROR_HIERARCHY_NODE_NO_LENGTH;
        }

        float mass;
        if (!_extractFloatFromNode(node->first_node("mass"), &mass))
        {
            return YOST_SKELETON_ERROR_HIERARCHY_NODE_NO_MASS;
        }

        YOST_SKELETON_BONE_UPDATE_TYPE update_type;
        if (!yost::_extractUpdateTypeFromNode(node->first_node("update_type"), &update_type))
        {
            return YOST_SKELETON_ERROR_HIERARCHY_NODE_NO_UPDATE_TYPE;
        }

        utilityAddBone(name, update_type, offset, parent_name);
        utilitySetBonePoseOrientation(name, pose);
        utilitySetBoneLength(name, length);
        utilitySetBoneMass(name, mass);

        rapidxml::xml_node<>* child = node->first_node("bone");
        YOST_SKELETON_ERROR result;
        while (child != nullptr)
        {
            result = utilityCreateBoneFromHierarchy(child, name);
            if (result != YOST_SKELETON_NO_ERROR)
            {
                return result;
            }

            child = child->next_sibling("bone");
        }

        return YOST_SKELETON_NO_ERROR;
    }

    YOST_SKELETON_ERROR Skeleton::utilitySetPinnedBone(std::shared_ptr<Bone>* bone)
    {
        _pinned_bone = *bone;
        return YOST_SKELETON_NO_ERROR;
    }

    YOST_SKELETON_ERROR Skeleton::utilitySetPinnedBone(std::string bone_name)
    {
        if (bone_name == "")
        {
            _pinned_bone = nullptr;
            return YOST_SKELETON_NO_ERROR;
        }

        auto name_it = _alias_map.find(bone_name);
        if (name_it == _alias_map.end())
        {
            return YOST_SKELETON_ERROR_BONE_NOT_FOUND;
        }

        _pinned_bone = _bone_map[name_it->second];
        return YOST_SKELETON_NO_ERROR;
    }

    void Skeleton::utilityTranslate(Vector3 translation)
    {
        Vector3 root_offset = _root_bone->getPositionOffset();
        vectorAdd(&root_offset, &translation, &root_offset);
        _root_bone->setPositionOffset(root_offset);
        //_root_bone->update();
        _root_bone->updatePosition();
    }

    std::string Skeleton::utilityGetDeviceXmlMap()
    {
        return _device_xml_map;
    }

    void Skeleton::utilitySetDeviceXmlMap(std::string device_xml)
    {
        _device_xml_map = device_xml;
    }

    YOST_SKELETON_ERROR Skeleton::utilityLoadDeviceXmlMap(std::string device_xml_file)
    {
        std::ifstream file_ptr;
        file_ptr.open(device_xml_file);
        if (file_ptr.is_open())
        {
            std::string line;
            _device_xml_map = "";
            while (std::getline(file_ptr, line))
            {
                _device_xml_map += line + "\n";
            }
        }
        else
        {
            return YOST_SKELETON_ERROR_FILE_READ;
        }
        file_ptr.close();

        return YOST_SKELETON_NO_ERROR;
    }

    YOST_SKELETON_ERROR Skeleton::utilitySaveDeviceXmlMap(std::string device_xml_file)
    {
        std::FILE* file_ptr = std::fopen(device_xml_file.c_str(), "w");
        if (file_ptr == nullptr)
        {
            return YOST_SKELETON_ERROR_FILE_WRITE;
        }

        std::fwrite(&_device_xml_map[0], sizeof(char), _device_xml_map.size(), file_ptr);
        std::fclose(file_ptr);

        return YOST_SKELETON_NO_ERROR;
    }

    // Private functions
    YOST_SKELETON_UNIT _convertStringToUnitType(std::string unit_str)
    {
        return (YOST_SKELETON_UNIT)std::distance(yost_skeleton_unit_string, std::find(yost_skeleton_unit_string, yost_skeleton_unit_string + 4, unit_str.c_str()));
    }

    YOST_SKELETON_BONE_UPDATE_TYPE _convertStringToUpdateType(std::string update_str)
    {
        return (YOST_SKELETON_BONE_UPDATE_TYPE)std::distance(yost_skeleton_bone_update_string, std::find(yost_skeleton_bone_update_string, yost_skeleton_unit_string + 4, update_str.c_str()));
    }

    bool _extractUpdateTypeFromNode(rapidxml::xml_node<>* node, YOST_SKELETON_BONE_UPDATE_TYPE* type)
    {
        if (node == nullptr)
        {
            return false;
        }

        *type = _convertStringToUpdateType(node->first_attribute("VALUE")->value());

        return true;
    }

    bool _extractFloatFromNode(rapidxml::xml_node<>* node, float* float_val)
    {
        if (node == nullptr)
        {
            return false;
        }

        *float_val = std::stof(node->first_attribute("VALUE")->value());

        return true;
    }

    bool _extractVectorFromNode(rapidxml::xml_node<>* node, Vector3* vec)
    {
        if (node == nullptr)
        {
            return false;
        }

        vec->data[0] = std::stof(node->first_attribute("X")->value());
        vec->data[1] = std::stof(node->first_attribute("Y")->value());
        vec->data[2] = std::stof(node->first_attribute("Z")->value());

        return true;
    }

    bool _extractQuaternionFromNode(rapidxml::xml_node<>* node, Orient* quat)
    {
        if (!node)
        {
            return false;
        }

        quat->data[QUAT_X] = std::stof(node->first_attribute("X")->value());
        quat->data[QUAT_Y] = std::stof(node->first_attribute("Y")->value());
        quat->data[QUAT_Z] = std::stof(node->first_attribute("Z")->value());
        quat->data[QUAT_W] = std::stof(node->first_attribute("W")->value());

        return true;
    }

    //conversion methods to help with C calls
    void _floatArrayToOrient(float* arr, Orient* orient)
    {
        orient->data[QUAT_X] = arr[QUAT_X];
        orient->data[QUAT_Y] = arr[QUAT_Y];
        orient->data[QUAT_Z] = arr[QUAT_Z];
        orient->data[QUAT_W] = arr[QUAT_W];
    }

    void _orientToFloatArray(Orient orient, float* arr)
    {
        arr[QUAT_X] = orient.data[QUAT_X];
        arr[QUAT_Y] = orient.data[QUAT_Y];
        arr[QUAT_Z] = orient.data[QUAT_Z];
        arr[QUAT_W] = orient.data[QUAT_W];
    }

    void _floatArrayToVector3(float* arr, Vector3* vec)
    {
        vec->data[0] = arr[0];
        vec->data[1] = arr[1];
        vec->data[2] = arr[2];
    }

    void _vector3ToFloatArray(Vector3 vec, float* arr)
    {
        arr[0] = vec.data[0];
        arr[1] = vec.data[1];
        arr[2] = vec.data[2];
    }
};
