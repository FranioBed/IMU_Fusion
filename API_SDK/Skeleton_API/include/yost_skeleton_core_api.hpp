/********************************************//**
 * Copyright 1998-2014, Yost Labs Corporation
 * This Source Code Form is subject to the terms of the YOST 3-Space Open
 * Source License available online at:
 * http://www.yeitechnology.com/yei-3-space-open-source-license
 ***********************************************/
#ifndef _YOST_SKELETON_CORE_API_H_
#define _YOST_SKELETON_CORE_API_H_
#include <stdint.h>
#include <string>
#include <map>
#include <memory>
#include <vector>
#include <deque>
#include <algorithm>

#include "yost_skeleton_api.h"
#include "yost_skeleton_processor.hpp"
#include "yost_math.hpp"
#include "rapidxml/rapidxml.hpp"

//useful types
typedef uint8_t U8;
typedef uint16_t U16;
typedef uint32_t U32;
typedef uint64_t U64;

namespace yost
{
    std::string getVersion();
    std::string getStandardBoneName(YOST_SKELETON_BONE standard_bone);

    void setAxisDirections(int8_t* order, int8_t* negate);

    // Defines a single bone of the skeleton
    class Bone
    {
    public:
        Bone();
        ~Bone();

        void update();
		void updatePosition();

        // Produces an XML hierarchy.
        // Calls recursively on the bone's children
        std::string createXMLHierarchy(uint32_t tabs = 0);

        std::string getName();
        void setName(std::string bone_name);

        uint32_t getDeviceId();
        void setDeviceId(uint32_t device_id);

        Orient getCalibrationOffset();
        void setCalibrationOffset(Orient offset);

        Orient getOrientationOffset();
        void setOrientationOffset(Orient offset);

        Orient getCalibrationTare();
        void setCalibrationTare(Orient tare);

        Orient getOrientation();
        void setOrientation(Orient orientation);

        Orient getRawOrientation();
        void setRawOrientation(Orient orientation);

        Vector3 getPositionOffset();
        void setPositionOffset(Vector3 offset);

        Vector3 getPosition();
        void setPosition(Vector3 position);

        Vector3 getLastPosition();
        void setLastPosition(Vector3 last_position);

        Orient getPoseOrientation();
        void setPoseOrientation(Orient pose_orientation);

        Vector3 getAcceleration();
        void setAcceleration(Vector3 acceleration);

        Vector3 getVelocity();
        void setVelocity(Vector3 velocity);

        float getLength();
        void setLength(float length);
        void setLengthFromChildren();

        Vector3 calculateBoneCenter();

        float getMass();
        void setMass(float mass);

        bool hasUpdated();
        void resetUpdateFlag();

        YOST_SKELETON_BONE_UPDATE_TYPE getUpdateType();
        void setUpdateType(YOST_SKELETON_BONE_UPDATE_TYPE type);
        void setAllChildrenUpdateType(YOST_SKELETON_BONE_UPDATE_TYPE type);

        std::shared_ptr<Bone> getParent();
        void setParent(std::shared_ptr<Bone>& parent);

        std::vector<std::shared_ptr<Bone>> getChildren();
        void addChild(std::shared_ptr<Bone>& child);
        void removeChild(std::shared_ptr<Bone>& child);
        void removeChild(std::string child_name);

    private:
        std::string _name;
        uint32_t _device_id;
        bool _has_updated;

        Orient _calibration_offset;
        Orient _calibration_tare;

        Orient _raw_orient;
        Orient _orientation;
        Vector3 _position_offset;
        Vector3 _position;
        Vector3 _last_position;

        Orient _pose_orientation;
        Orient _offset_orientation;

        Vector3 _acceleration;
        Vector3 _velocity;
        std::chrono::steady_clock::time_point _prevTime;
        std::chrono::steady_clock::duration _deltaTime;

        float _length;
        float _mass;

        YOST_SKELETON_BONE_UPDATE_TYPE _update_type;

        std::shared_ptr<Bone> _parent;

        std::vector<std::shared_ptr<Bone>> _children;
    };

    // The main class of the Skeletal API
    class Skeleton
    {
    public:
        Skeleton();
        ~Skeleton();

        // Creates a skeleton hierarchy using statistics
        void createStandardSkeletonHierarchyWithAge(bool male = true, uint32_t age = YOST_SKELETON_DEFAULT_AGE);
        void createStandardSkeletonHierarchy(bool male = true, float height = YOST_SKELETON_DEFAULT_HEIGHT);

        // Loads in a skeleton hierarchy from a file
        YOST_SKELETON_ERROR loadSkeletonHierarchy(std::string hierarchy_file);

        // Add a new processor to be used by the skeleton when updating which is based on the index
        YOST_SKELETON_ERROR addProcessor(uint32_t index, yost_skeleton_id processor);

        // Remove a processor from the skeleton
        YOST_SKELETON_ERROR removeProcessor(uint32_t index);

        std::deque<yost_skeleton_id> getProcessorList();
        std::vector<std::string> getProcessorNameList();

        YOST_SKELETON_ERROR update();

        // Get the name of the currently pinned bone, if any
        std::string getPinnedBoneName();

        // Get the currently pinned bone, if any
        std::shared_ptr<Bone> getPinnedBone();

        void setPinnedBone(std::shared_ptr<Bone>* pin_bone);

        // Get the pinning bones, if any
        std::vector<std::shared_ptr<Bone>> getPinBones();

        YOST_SKELETON_ERROR addPinBone(std::string pin_bone);

        // Get the name of the current root bone, if any
        std::string getRootBoneName();

        // Get the current root bone, if any
        std::shared_ptr<Bone> getRootBone();

        // Get the list of the names of all bones
        std::vector<std::string> getBoneNameList();

        // Get the list of all the bones
        std::vector<std::shared_ptr<Bone>> getBoneList();

        // A map of the skeleton calibration pose
        std::map<std::string, Orient> getSkeletonPose();

        // A map of the skeleton offsets
        std::map<std::string, Orient> getBoneOffsets();

        // Sets the skeleton in a pose with the orientations for a standard T pose
        void setToStandardTPose();

        // Sets the skeleton in a pose with the orientations for a standard clasped pose
        void setToStandardClaspedPose();

        // Sets the skeleton in a pose with the orientations for a standard neutral pose
        void setToStandardNeutralPose();

        // Read out the existing hierarchy as an XML string
        std::string extractHierarchy();

        // Tells if the skeleton has been calibrated yet or not
        bool isCalibrated();

        //Sets whether or not the skeleton has been calibrated
        void setCalibration(bool calibration);

        // Returns whether or not the skeleton has updated since it's last calling
        bool hasUpdated();

        std::string getName();
        void setName(std::string skeleton_name);

        YOST_SKELETON_UNIT getSkeletonUnit();
        void setSkeletonUnit(YOST_SKELETON_UNIT unit_type);

        // Sets an alias you can use to refer to one of the bones in the skeleton hierarchy
        YOST_SKELETON_ERROR addBoneAlias(std::string your_bone_name, std::string bone_name);

        std::shared_ptr<Bone> getBone(std::string bone_name);

        // Sets the orentation offset of the given bone
        YOST_SKELETON_ERROR setBoneOrientationOffset(std::string bone_name, Orient offset);

        // Retrieve the orentation offset of the given bone
        YOST_SKELETON_ERROR getBoneOrientationOffset(std::string bone_name, Orient* offset);

        // Retrieve the orientation from the given bone
        YOST_SKELETON_ERROR getBoneOrientation(std::string bone_name, Orient* orient);

        // Retrieve the position from the given bone
        YOST_SKELETON_ERROR getBonePosition(std::string bone_name, Vector3* pos);

        // Sets the position from the given bone
        YOST_SKELETON_ERROR setBonePositionOffset(std::string bone_name, Vector3 pos_offset);

        // Retrieve the position from the given bone
        YOST_SKELETON_ERROR getBonePositionOffset(std::string bone_name, Vector3* pos_offset);

        // Retrieve the center position from the given bone
        YOST_SKELETON_ERROR getBoneCenter(std::string bone_name, Vector3* pos);

        // Retrieve the velocity, based on position change from the previous frame, from the given bone
        YOST_SKELETON_ERROR getBoneVelocity(std::string bone_name, Vector3* vel);

        // Retrieve the acceleration, based on sensor data.
        YOST_SKELETON_ERROR getBoneAcceleration(std::string bone_name, Vector3* accel);

        // Retrieve the orentation offsets of the given bone
        YOST_SKELETON_ERROR getAllBoneOrientationOffsets(float* offset);

        // Retrieve the orientations from the bones
        YOST_SKELETON_ERROR getAllBoneOrientations(float* orient);

        // Retrieve the positions from the bones
        YOST_SKELETON_ERROR getAllBonePositions(float* pos);

        // Retrieve the velocitites, based on position change from the previous frame, from the bones
        YOST_SKELETON_ERROR getAllBoneVelocitites(float* vel);

        // Retrieve the accelerations, based on sensor data.
        YOST_SKELETON_ERROR getAllBoneAccelerations(float* accel);

        // Add a bone for getting all skeleton info in one call
        YOST_SKELETON_ERROR utilityRegisterBone(std::string bone_name);

        // Remove a bone for getting all skeleton info in one call
        YOST_SKELETON_ERROR utilityDeregisterBone(std::string bone_name);

        // Retrieve the bones registered for polling as a list of names
        std::vector<std::string> utilityRegisteredBoneList();

        // Retrieve the count of registerd bone
        U32 utilityRegisteredBoneCount();

        // Retrieve the lengths from the bones
        YOST_SKELETON_ERROR utilityGetBoneLength(std::string bone_name, float* length);

        // Retrieve the mass from the given bone
        YOST_SKELETON_ERROR getBoneMass(std::string bone_name, float* mass);

        // Utility functions, not usually necessary to call
        void utilityCreateXMLHierarchy();

        // Utility functions, not usually necessary to call
        void utilitySaveXMLHierarchy(std::string file_name);

        // Add a bone to the current hierarchy
        YOST_SKELETON_ERROR utilityAddBone(std::string bone_name, YOST_SKELETON_BONE_UPDATE_TYPE update_type, Vector3 offset, std::string parent_name="");

        // Remove a bone from the current hierarchy
        YOST_SKELETON_ERROR utilityRemoveBone(std::string bone_name);

        // Set an existing bone's default orientation
        YOST_SKELETON_ERROR utilitySetBonePoseOrientation(std::string bone_name, Orient pose);

        // Set an existing bone's length
        YOST_SKELETON_ERROR utilitySetBoneLength(std::string bone_name, float length);

        // Set the bones' lengths based off children
        void utilitySetBoneLengths();

        // Set an existing bone's mass
        YOST_SKELETON_ERROR utilitySetBoneMass(std::string bone_name, float mass);

        // Retrieve the bone's mass
        YOST_SKELETON_ERROR utilityGetBoneMass(std::string bone_name, float* mass);

        YOST_SKELETON_ERROR utilityCreateBoneFromHierarchy(rapidxml::xml_node<>* node, std::string parent_name="");

        // Set the current pinned bone for this skeleton
        YOST_SKELETON_ERROR utilitySetPinnedBone(std::shared_ptr<Bone>* bone);

        // Set the current pinned bone for this skeleton by name
        YOST_SKELETON_ERROR utilitySetPinnedBone(std::string bone_name);

        void utilityTranslate(Vector3 translation);

        std::string utilityGetDeviceXmlMap();
        void utilitySetDeviceXmlMap(std::string device_xml);
        YOST_SKELETON_ERROR utilitySaveDeviceXmlMap(std::string device_xml_file);
        YOST_SKELETON_ERROR utilityLoadDeviceXmlMap(std::string device_xml_file);

        float getHeight();

    private:
        // This hierarchy defines the skeletal structure
        std::string _hierarchy;

        std::string _name;
        YOST_SKELETON_UNIT _unit;
        float _height;

        // This map is responsible for ownership of the data associated with bones.
        std::map<std::string, std::shared_ptr<Bone>> _bone_map;
        std::map<std::string, Orient> _bone_offsets;

        std::map<std::string, Orient> _skeleton_pose;

        // Root of the bone hierarchy. Forward kinematics begins here.
        std::shared_ptr<Bone> _root_bone;

        std::vector<std::shared_ptr<Bone>> pinning_bones;

        // Mark the currently pinned bone...there may be more than one later.
        std::shared_ptr<Bone> _pinned_bone;

        // This is the list of processor ids that will be applied to the skeleton during each update.
        std::deque<yost_skeleton_id> _processors;

        // Maps used to map bone names to the API's standard, and back.
        std::map<std::string, std::string> _alias_map;
        std::map<std::string, std::string> _reverse_alias_map;

        std::vector<std::shared_ptr<Bone>> _poll_List;

        // Value indicating if calibration has occurred yet.
        bool _calibrated;

        // This xml data defines how the skeletal structure and device interact
        std::string _device_xml_map;
    };

    extern std::vector<std::unique_ptr<Skeleton>> stored_skeletons;
    extern std::vector<std::unique_ptr<SkeletonProcessor>> stored_processors;

    // Private functions
    YOST_SKELETON_UNIT _convertStringToUnitType(std::string unit_str);

    YOST_SKELETON_BONE_UPDATE_TYPE _convertStringToUpdateType(std::string update_str);

    bool _extractUpdateTypeFromNode(rapidxml::xml_node<>* node, YOST_SKELETON_BONE_UPDATE_TYPE* type);

    bool _extractFloatFromNode(rapidxml::xml_node<>* node, float* float_val);

    bool _extractVectorFromNode(rapidxml::xml_node<>* node, Vector3* vec);

    bool _extractQuaternionFromNode(rapidxml::xml_node<>* node, Orient* quat);

    // Conversion methods to help with C calls
    void _floatArrayToOrient(float* quat4, Orient* orient);

    void _orientToFloatArray(Orient orient, float* quat4);

    void _floatArrayToVector3(float* vec3, Vector3* vec);

    void _vector3ToFloatArray(Vector3 vec, float* vec3);
};

#endif // _YOST_SKELETON_CORE_API_H_
