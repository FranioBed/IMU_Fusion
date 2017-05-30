/********************************************//**
 * \ingroup skel_api
 * \file yost_skeleton_api.h
 * \section Details
 * \author Chris George
 * \author Steve Landers
 * \author Travis Lynn
 * \author Nick Leyder
 * \version YOST Skeletal API 1.0.0
 * \copyright Copyright 1998-2016, Yost Labs Corporation
 *
 * \brief This file is a collection of convenience functions for creating and using skeletons with Yost Prio and 3-Space devices
 * for use in a program written in C/C++ or any language that can import a compiled library (.dll, .so, etc).
 *
 * The Yost Skeletal API is released under the Yost 3-Space Open Source License, which allows for both
 * non-commercial use and commercial use with certain restrictions.
 *
 * For Non-Commercial Use, your use of Covered Works is governed by the GNU GPL v.3, subject to the Yost 3-Space Open
 * Source Licensing Overview and Definitions.
 *
 * For Commercial Use, a Yost Commercial/Redistribution License is required, pursuant to the Yost 3-Space Open Source
 * Licensing Overview and Definitions. Commercial Use, for the purposes of this License, means the use, reproduction
 * and/or Distribution, either directly or indirectly, of the Covered Works or any portion thereof, or a Compilation,
 * Improvement, or Modification, for Pecuniary Gain. A Yost Commercial/Redistribution License may or may not require
 * payment, depending upon the intended use.
 *
 * Full details of the Yost 3-Space Open Source License can be found in license.txt
 * License also available online at http://www.yeitechnology.com/yei-3-space-open-source-license
 ***********************************************/

#ifndef _YOST_SKELETON_API_H_
#define _YOST_SKELETON_API_H_
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

#if defined(_MSC_VER)
#define SKEL_EXPORT __declspec(dllexport)
#elif defined(__GNUC__) && !defined(__APPLE__)
#define SKEL_EXPORT __attribute__ ((dllexport))
#elif defined(__GNUC__) && defined(__APPLE__)
#define SKEL_EXPORT
#endif

/********************************************//**
 * \ingroup skel_api
 * Default user age.
 ***********************************************/
#define YOST_SKELETON_DEFAULT_AGE 21

/********************************************//**
 * \ingroup skel_api
 * Default user height.
 ***********************************************/
#define YOST_SKELETON_DEFAULT_HEIGHT 1.7526f

/********************************************//**
 * \ingroup skel_api
 * Denotes adding to the end of the processor list.
 ***********************************************/
#define YOST_SKELETON_PROCESSOR_LIST_END 0xffffffff

/********************************************//**
 * \ingroup skel_api
 * An enum expressing orientation directions.
 ***********************************************/
typedef enum YOST_AXIS_DIRECTION
{
    RIGHT = 0,
    UP = 1,
    FORWARD = 2
}YOST_AXIS_DIRECTION;

/********************************************//**
 * \ingroup skel_api
 * YOST default axis directions.
 ***********************************************/
//const char defaultAxesDirections[] = { RIGHT, UP, FORWARD };
//const int8_t defaultNegatedDirections[] = { 0, 0, 0 };

/********************************************//**
 * \ingroup skel_api
 * An enum expressing masks used to quickly determine the type of id.
 ***********************************************/
typedef enum YOST_SKELETON_ID_MASK
{
    YOST_SKELETON_INVALID_ID     = 0x0040000,    /**< Invalid ID */
    YOST_SKELETON_ID             = 0x0080000,    /**< Skeleton ID */
    YOST_SKELETON_PROCESSOR_ID   = 0x01000000    /**< Processor ID */
}YOST_SKELETON_ID_MASK;

/********************************************//**
 * \ingroup skel_api
 * YOST Skeletal API type identifier, a common parameter needed for most Skeletal API calls.
 ***********************************************/
typedef uint32_t yost_skeleton_id;

/********************************************//**
* \ingroup skel_api
* YOST Prio API type identifier, a common parameter needed for most Prio API calls.
***********************************************/
typedef uint32_t prio_device_id;

/********************************************//**
 * \ingroup skel_api
 * An enum expressing the different types of errors a Yost Skeletal API call can return.
 ***********************************************/
typedef enum YOST_SKELETON_ERROR
{
    YOST_SKELETON_NO_ERROR,                                  /**< The API call successfully executed */
    YOST_SKELETON_ERROR_SKELETON_NOT_FOUND,                  /**< Invalid skeleton index given */
    YOST_SKELETON_ERROR_PROCESSOR_NOT_FOUND,                 /**< Invalid processor index given */
    YOST_SKELETON_ERROR_BONE_NOT_FOUND,                      /**< Invalid bone name given */
    YOST_SKELETON_ERROR_PARENT_BONE_NOT_FOUND,               /**< Invalid parent bone name given */
    YOST_SKELETON_ERROR_NO_ROOT_BONE,                        /**< Skeleton has no root bone */
    YOST_SKELETON_ERROR_UPDATE_NOT_NEEDED,                   /**< No new data to update skeleton with */
    YOST_SKELETON_ERROR_ANCHOR_SENSOR_MISSING,               /**< The anchor sensor is missing during calibration */
    YOST_SKELETON_ERROR_EMPTY_NAME,                          /**< Given name was empty */
    YOST_SKELETON_ERROR_DUPLICATE_BONE,                      /**< Bone with given name already exists in skeleton */
    YOST_SKELETON_ERROR_DUPLICATE_ROOT_BONE,                 /**< Skeleton already has a root bone */
    YOST_SKELETON_ERROR_HIERARCHY_NO_ROOT_NODE,              /**< There is no root node in hierarchy */
    YOST_SKELETON_ERROR_HIERARCHY_NO_SKELETON_NODE,          /**< There is no skeleton node in hierarchy */
    YOST_SKELETON_ERROR_HIERARCHY_NO_ROOT_BONE_NODE,         /**< There is no root bone node in hierarchy */
    YOST_SKELETON_ERROR_HIERARCHY_NODE_NO_NAME,              /**< Node in hierarchy has no name */
    YOST_SKELETON_ERROR_HIERARCHY_NODE_NO_UNIT,              /**< Node in hierarchy has no unit */
    YOST_SKELETON_ERROR_HIERARCHY_NODE_NO_OFFSET,            /**< Node in hierarchy has no offset */
    YOST_SKELETON_ERROR_HIERARCHY_NODE_NO_LENGTH,            /**< Node in hierarchy has no length */
    YOST_SKELETON_ERROR_HIERARCHY_NODE_NO_MASS,              /**< Node in hierarchy has no mass */
    YOST_SKELETON_ERROR_HIERARCHY_NODE_NO_POSE_ORIENTATION,  /**< Node in hierarchy has no pose orientation */
    YOST_SKELETON_ERROR_HIERARCHY_NODE_NO_UPDATE_TYPE,       /**< Node in hierarchy has no update type */
    YOST_SKELETON_ERROR_NO_MORE_DATA,                        /**< Indicates that all data in a list has been read */
    YOST_SKELETON_ERROR_BUFFER_TOO_SMALL,                    /**< Buffer could not hold the data */
    YOST_SKELETON_ERROR_FILE_READ,                           /**< File could not be read */
    YOST_SKELETON_ERROR_FILE_WRITE,                          /**< File could not be written to*/
    YOST_SKELETON_ERROR_INDEX_OUT_OF_RANGE,                  /**< Index is out of range */
    YOST_SKELETON_ERROR_PROCESSOR_COMMAND,                   /**< Processor command failed */
    YOST_SKELETON_ERROR_PROCESSOR_IN_USE,                    /**< Processor is being used by a skeleton */
    YOST_SKELETON_ERROR_JOYSTICK_INACTIVE,                   /**< The joystick is not plugged in */
    YOST_SKELETON_ERROR_RESET_API                            /**< The API failed to reset itself */
}YOST_SKELETON_ERROR;

/********************************************//**
 * \ingroup skel_api
 * A c_string array to help express the different types of errors a YOST Skeletal API call can return.
 ***********************************************/
static const char* const yost_skeleton_error_string[] =
{
    "YOST_SKELETON_NO_ERROR",
    "YOST_SKELETON_ERROR_SKELETON_NOT_FOUND",
    "YOST_SKELETON_ERROR_PROCESSOR_NOT_FOUND",
    "YOST_SKELETON_ERROR_BONE_NOT_FOUND",
    "YOST_SKELETON_ERROR_PARENT_BONE_NOT_FOUND",
    "YOST_SKELETON_ERROR_NO_ROOT_BONE",
    "YOST_SKELETON_ERROR_UPDATE_NOT_NEEDED",
    "YOST_SKELETON_ERROR_ANCHOR_SENSOR_MISSING",
    "YOST_SKELETON_ERROR_EMPTY_NAME",
    "YOST_SKELETON_ERROR_DUPLICATE_BONE",
    "YOST_SKELETON_ERROR_DUPLICATE_ROOT_BONE",
    "YOST_SKELETON_ERROR_HIERARCHY_NO_ROOT_NODE",
    "YOST_SKELETON_ERROR_HIERARCHY_NO_SKELETON_NODE",
    "YOST_SKELETON_ERROR_HIERARCHY_NO_ROOT_BONE_NODE",
    "YOST_SKELETON_ERROR_HIERARCHY_NODE_NO_NAME",
    "YOST_SKELETON_ERROR_HIERARCHY_NODE_NO_UNIT",
    "YOST_SKELETON_ERROR_HIERARCHY_NODE_NO_OFFSET",
    "YOST_SKELETON_ERROR_HIERARCHY_NODE_NO_LENGTH",
    "YOST_SKELETON_ERROR_HIERARCHY_NODE_NO_MASS",
    "YOST_SKELETON_ERROR_HIERARCHY_NODE_NO_POSE_ORIENTATION",
    "YOST_SKELETON_ERROR_HIERARCHY_NODE_NO_UPDATE_TYPE",
    "YOST_SKELETON_ERROR_NO_MORE_DATA",
    "YOST_SKELETON_ERROR_BUFFER_TOO_SMALL",
    "YOST_SKELETON_ERROR_FILE_READ",
    "YOST_SKELETON_ERROR_INDEX_OUT_OF_RANGE",
    "YOST_SKELETON_ERROR_PROCESSOR_COMMAND",
    "YOST_SKELETON_ERROR_PROCESSOR_IN_USE",
    "YOST_SKELETON_ERROR_RESET_API"
};

/********************************************//**
 * \ingroup skel_api
 * An enum expressing the standard names of bones for a skeleton.
***********************************************/
typedef enum YOST_SKELETON_BONE
{
    YOST_SKELETON_BONE_HIPS,                    /**< This will get the standard bone name for the hips */
    YOST_SKELETON_BONE_LOWER_SPINE_2,           /**< This will get the standard bone name for the second lower spine */
    YOST_SKELETON_BONE_LOWER_SPINE_1,           /**< This will get the standard bone name for the first lower spine */
    YOST_SKELETON_BONE_SPINE,                   /**< This will get the standard bone name for the spine */
    YOST_SKELETON_BONE_NECK,                    /**< This will get the standard bone name for the neck */
    YOST_SKELETON_BONE_HEAD,                    /**< This will get the standard bone name for the head */
    YOST_SKELETON_BONE_HEAD_END,                /**< This will get the standard bone name for the head end */
    YOST_SKELETON_BONE_LEFT_SHOULDER,           /**< This will get the standard bone name for the left shoulder */
    YOST_SKELETON_BONE_LEFT_UPPER_ARM,          /**< This will get the standard bone name for the left upper arm */
    YOST_SKELETON_BONE_LEFT_LOWER_ARM,          /**< This will get the standard bone name for the left lower arm */
    YOST_SKELETON_BONE_LEFT_HAND,               /**< This will get the standard bone name for the left hand */
    YOST_SKELETON_BONE_LEFT_HAND_THUMB_1,       /**< This will get the standard bone name for the first left hand thumb */
    YOST_SKELETON_BONE_LEFT_HAND_THUMB_2,       /**< This will get the standard bone name for the second left hand thumb */
    YOST_SKELETON_BONE_LEFT_HAND_THUMB_3,       /**< This will get the standard bone name for the third left hand thumb */
    YOST_SKELETON_BONE_LEFT_HAND_THUMB_END,     /**< This will get the standard bone name for the end left hand thumb */
    YOST_SKELETON_BONE_LEFT_HAND_INDEX_1,       /**< This will get the standard bone name for the first left hand index */
    YOST_SKELETON_BONE_LEFT_HAND_INDEX_2,       /**< This will get the standard bone name for the second left hand index */
    YOST_SKELETON_BONE_LEFT_HAND_INDEX_3,       /**< This will get the standard bone name for the third left hand index */
    YOST_SKELETON_BONE_LEFT_HAND_INDEX_END,     /**< This will get the standard bone name for the end left hand index */
    YOST_SKELETON_BONE_LEFT_HAND_MIDDLE_1,      /**< This will get the standard bone name for the first left hand middle */
    YOST_SKELETON_BONE_LEFT_HAND_MIDDLE_2,      /**< This will get the standard bone name for the second left hand middle */
    YOST_SKELETON_BONE_LEFT_HAND_MIDDLE_3,      /**< This will get the standard bone name for the third left hand middle */
    YOST_SKELETON_BONE_LEFT_HAND_MIDDLE_END,    /**< This will get the standard bone name for the end left hand middle */
    YOST_SKELETON_BONE_LEFT_HAND_RING_1,        /**< This will get the standard bone name for the first left hand ring */
    YOST_SKELETON_BONE_LEFT_HAND_RING_2,        /**< This will get the standard bone name for the second left hand ring */
    YOST_SKELETON_BONE_LEFT_HAND_RING_3,        /**< This will get the standard bone name for the third left hand ring */
    YOST_SKELETON_BONE_LEFT_HAND_RING_END,      /**< This will get the standard bone name for the end left hand ring */
    YOST_SKELETON_BONE_LEFT_HAND_PINKY_1,       /**< This will get the standard bone name for the first left hand pinky */
    YOST_SKELETON_BONE_LEFT_HAND_PINKY_2,       /**< This will get the standard bone name for the second left hand pinky */
    YOST_SKELETON_BONE_LEFT_HAND_PINKY_3,       /**< This will get the standard bone name for the third left hand pinky */
    YOST_SKELETON_BONE_LEFT_HAND_PINKY_END,     /**< This will get the standard bone name for the end left hand pinky */
    YOST_SKELETON_BONE_LEFT_JOYSTICK,           /**< This will get the standard bone name for the left joystick */
    YOST_SKELETON_BONE_RIGHT_SHOULDER,          /**< This will get the standard bone name for the right shoulder */
    YOST_SKELETON_BONE_RIGHT_UPPER_ARM,         /**< This will get the standard bone name for the right upper arm */
    YOST_SKELETON_BONE_RIGHT_LOWER_ARM,         /**< This will get the standard bone name for the right lower arm */
    YOST_SKELETON_BONE_RIGHT_HAND,              /**< This will get the standard bone name for the right hand */
    YOST_SKELETON_BONE_RIGHT_HAND_THUMB_1,      /**< This will get the standard bone name for the first right hand thumb */
    YOST_SKELETON_BONE_RIGHT_HAND_THUMB_2,      /**< This will get the standard bone name for the second right hand thumb */
    YOST_SKELETON_BONE_RIGHT_HAND_THUMB_3,      /**< This will get the standard bone name for the third right hand thumb */
    YOST_SKELETON_BONE_RIGHT_HAND_THUMB_END,    /**< This will get the standard bone name for the end right hand thumb */
    YOST_SKELETON_BONE_RIGHT_HAND_INDEX_1,      /**< This will get the standard bone name for the first right hand index */
    YOST_SKELETON_BONE_RIGHT_HAND_INDEX_2,      /**< This will get the standard bone name for the second right hand index */
    YOST_SKELETON_BONE_RIGHT_HAND_INDEX_3,      /**< This will get the standard bone name for the third right hand index */
    YOST_SKELETON_BONE_RIGHT_HAND_INDEX_END,    /**< This will get the standard bone name for the end right hand index */
    YOST_SKELETON_BONE_RIGHT_HAND_MIDDLE_1,     /**< This will get the standard bone name for the first right hand middle */
    YOST_SKELETON_BONE_RIGHT_HAND_MIDDLE_2,     /**< This will get the standard bone name for the second right hand middle */
    YOST_SKELETON_BONE_RIGHT_HAND_MIDDLE_3,     /**< This will get the standard bone name for the third right hand middle */
    YOST_SKELETON_BONE_RIGHT_HAND_MIDDLE_END,   /**< This will get the standard bone name for the end right hand middle */
    YOST_SKELETON_BONE_RIGHT_HAND_RING_1,       /**< This will get the standard bone name for the first right hand ring */
    YOST_SKELETON_BONE_RIGHT_HAND_RING_2,       /**< This will get the standard bone name for the second right hand ring */
    YOST_SKELETON_BONE_RIGHT_HAND_RING_3,       /**< This will get the standard bone name for the third right hand ring */
    YOST_SKELETON_BONE_RIGHT_HAND_RING_END,     /**< This will get the standard bone name for the end right hand ring */
    YOST_SKELETON_BONE_RIGHT_HAND_PINKY_1,      /**< This will get the standard bone name for the first right hand pinky */
    YOST_SKELETON_BONE_RIGHT_HAND_PINKY_2,      /**< This will get the standard bone name for the second right hand pinky */
    YOST_SKELETON_BONE_RIGHT_HAND_PINKY_3,      /**< This will get the standard bone name for the third right hand pinky */
    YOST_SKELETON_BONE_RIGHT_HAND_PINKY_END,    /**< This will get the standard bone name for the end right hand pinky */
    YOST_SKELETON_BONE_RIGHT_JOYSTICK,           /**< This will get the standard bone name for the right joystick */
    YOST_SKELETON_BONE_LEFT_UPPER_LEG,          /**< This will get the standard bone name for the left upper leg */
    YOST_SKELETON_BONE_LEFT_LOWER_LEG,          /**< This will get the standard bone name for the left lower leg */
    YOST_SKELETON_BONE_LEFT_FOOT,               /**< This will get the standard bone name for the left foot */
    YOST_SKELETON_BONE_LEFT_FOOT_HEEL,          /**< This will get the standard bone name for the left foot heel */
    YOST_SKELETON_BONE_LEFT_FOOT_BALL,          /**< This will get the standard bone name for the left foot ball */
    YOST_SKELETON_BONE_RIGHT_UPPER_LEG,         /**< This will get the standard bone name for the right upper leg */
    YOST_SKELETON_BONE_RIGHT_LOWER_LEG,         /**< This will get the standard bone name for the right lower leg */
    YOST_SKELETON_BONE_RIGHT_FOOT,              /**< This will get the standard bone name for the right foot */
    YOST_SKELETON_BONE_RIGHT_FOOT_HEEL,         /**< This will get the standard bone name for the right foot heel */
    YOST_SKELETON_BONE_RIGHT_FOOT_BALL,         /**< This will get the standard bone name for the right foot ball */
}YOST_SKELETON_BONE;

/********************************************//**
 * \ingroup skel_api
 * An enum expressing the different kinds of orientation updating methods a bone can have.
***********************************************/
typedef enum YOST_SKELETON_BONE_UPDATE_TYPE
{
    YOST_SKELETON_BONE_UPDATE_NULL,          /**< This bone's orientation will not update */
    YOST_SKELETON_BONE_UPDATE_DEVICE,        /**< This bone's orientation is updated by a device */
    YOST_SKELETON_BONE_UPDATE_FUSED,         /**< This bone's orientation is updated by its parent's orientation */
    YOST_SKELETON_BONE_UPDATE_INTERPOLATE,   /**< This bone's orientation is updated by interpolation */
}YOST_SKELETON_BONE_UPDATE_TYPE;

/********************************************//**
 * \ingroup skel_api
 * A c_string array to help express the different kinds of orientation updating methods a bone can have.
***********************************************/
static const char* const yost_skeleton_bone_update_string[] =
{
    "YOST_SKELETON_BONE_UPDATE_NULL",
    "YOST_SKELETON_BONE_UPDATE_DEVICE",
    "YOST_SKELETON_BONE_UPDATE_FUSED",
    "YOST_SKELETON_BONE_UPDATE_INTERPOLATE"
};

/********************************************//**
 * \ingroup skel_api
 * An enum expressing the different kinds of units a skeleton can have.
***********************************************/
typedef enum YOST_SKELETON_UNIT
{
    YOST_SKELETON_UNIT_CENTIMETERS,  /**< This skeleton's units are in centimeters */
    YOST_SKELETON_UNIT_METERS,       /**< This skeleton's units are in meters */
    YOST_SKELETON_UNIT_INCHES,       /**< This skeleton's units are in inches */
    YOST_SKELETON_UNIT_FEET,         /**< This skeleton's units are in feet */
}YOST_SKELETON_UNIT;

/********************************************//**
 * \ingroup skel_api
 * A c_string array to help express the different kinds of units a skeleton can have.
***********************************************/
static const char* const yost_skeleton_unit_string[] =
{
    "YOST_SKELETON_UNIT_CENTIMETERS",
    "YOST_SKELETON_UNIT_METERS",
    "YOST_SKELETON_UNIT_INCHES",
    "YOST_SKELETON_UNIT_FEET"
};

typedef enum YOST_SKELETON_JOYSTICK_AXIS
{
    YOST_SKELETON_LEFT_JOYSTICK_X_AXIS,     /**<This will get the state of left joystick X axis*/
    YOST_SKELETON_LEFT_JOYSTICK_Y_AXIS,     /**<This will get the state of left joystick Y axis*/
    YOST_SKELETON_RIGHT_JOYSTICK_X_AXIS,    /**<This will get the state of right joystick X axis*/
    YOST_SKELETON_RIGHT_JOYSTICK_Y_AXIS,    /**<This will get the state of right joystick Y axis*/
}YOST_SKELETON_JOYSTICK_AXIS;

typedef enum YOST_SKELETON_JOYSTICK_BUTTON
{
    YOST_SKELETON_LEFT_JOYSTICK_TRIGGER,        /**<This will get the state of left joystick trigger*/
    YOST_SKELETON_LEFT_JOYSTICK_BUMPER_BUTTON,  /**<This will get the state of left joystick bumper button*/
    YOST_SKELETON_LEFT_JOYSTICK_X_BUTTON,       /**<This will get the state of left joystick X button*/
    YOST_SKELETON_LEFT_JOYSTICK_A_BUTTON,       /**<This will get the state of left joystick A button*/
    YOST_SKELETON_LEFT_JOYSITCK_B_BUTTON,       /**<This will get the state of left joystick B button*/
    YOST_SKELETON_LEFT_JOYSTICK_Y_BUTTON,       /**<This will get the state of left joystick Y button*/
    YOST_SKELETON_RIGHT_JOYSTICK_TRIGGER,       /**<This will get the state of right joystick trigger*/
    YOST_SKELETON_RIGHT_JOYSTICK_BUMPER_BUTTON, /**<This will get the state of right joystick bumper button*/
    YOST_SKELETON_RIGHT_JOYSTICK_X_BUTTON,      /**<This will get the state of right joystick X button*/
    YOST_SKELETON_RIGHT_JOYSTICK_A_BUTTON,      /**<This will get the state of right joystick A button*/
    YOST_SKELETON_RIGHT_JOYSITCK_B_BUTTON,      /**<This will get the state of right joystick B button*/
    YOST_SKELETON_RIGHT_JOYSTICK_Y_BUTTON,      /**<This will get the state of right joystick Y button*/
}YOST_SKELETON_JOYSTICK_BUTTON;

// API Specific Methods
/********************************************//**
 * \ingroup skel_api_methods
 * \brief Fill the given buffer with the version string.
 * \param[out] buffer The buffer to fill with the version string.
 * \param[in] buffer_size Size of the buffer to store the data in. Recommended size: 32 bytes.
 * \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
 ***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_getVersionString(char* buffer, uint32_t buffer_size);
/********************************************//**
 * \ingroup skel_api_methods
 * \brief Reset the Skeletal API.
 *
 * This will clear all data created by the API.
 * All device ids created before the reset are invalid and should be discarded.
 * \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
 ***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_resetSkeletalApi();
/********************************************//**
 * \ingroup skel_api_methods
 * \brief Set the coordinate system
 * \param[in] order A 3 element array of ordered YOST_AXIS_DIRECTIONS;
 * \param[in] negate A 3 element array to list which YOST_AXIS_DIRECTIONS are negated;
 * \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
 ***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_setAxisDirections(int8_t* order, int8_t* negate);
/********************************************//**
 * \ingroup skel_api_methods
 * \brief Retrieves the standard name of the bone.
 * \param[in] bone The standard bone type.
 * \param[out] bone_name A C string to write the bone's name to.
 * \param[in] name_len The size of the C string entered. Recommended size: 32 bytes.
 * \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
 ***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_getStandardBoneName(YOST_SKELETON_BONE bone, char* bone_name, uint32_t name_len);
// Skeleton Methods
/********************************************//**
 * \ingroup skel_skeleton_methods
 * \brief Create a standard skeleton using user's age.
 * \param[in] male Whether the user is male or not.
 * \param[in] age Age of the user.
 * \return A yost_skeleton ID, YOST_SKELETON_INVALID_ID is returned if the creation failed.
 ***********************************************/
SKEL_EXPORT yost_skeleton_id yostskel_createStandardSkeletonWithAge(uint8_t male, uint32_t age);
/********************************************//**
 * \ingroup skel_skeleton_methods
 * \brief Create a standard skeleton using user's height.
 * \param[in] male Whether the user is male or not.
 * \param[in] height Height of the user in meters.
 * \return A yost_skeleton ID, YOST_SKELETON_INVALID_ID is returned if the creation failed.
 ***********************************************/
SKEL_EXPORT yost_skeleton_id yostskel_createStandardSkeletonWithHeight(uint8_t male, float height);
/********************************************//**
 * \ingroup skel_skeleton_methods
 * \brief Create a skeleton from file.
 * \param[in] hierarchy_file File containing the hierarchy.
 * \return A yost_skeleton ID, YOST_SKELETON_INVALID_ID is returned if the creation failed.
 ***********************************************/
SKEL_EXPORT yost_skeleton_id yostskel_createSkeletonFromFile(const char* hierarchy_file);
/********************************************//**
 * \ingroup skel_skeleton_methods
 * \brief Destroys the skeleton.
 * \param[in] skel_id A yost_skeleton ID of the skeleton.
 * \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
 ***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_destroySkeleton(yost_skeleton_id skel_id);
/********************************************//**
 * \ingroup skel_skeleton_methods
 * \brief Set the skeleton to a standard T pose.
 * \param[in] skel_id A yost_skeleton ID of the skeleton to operate on.
 * \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
 ***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_setSkeletonToStandardTPose(yost_skeleton_id skel_id);
/********************************************//**
 * \ingroup skel_skeleton_methods
 * \brief Set the skeleton to a clasped pose with arms forward.
 * \param[in] skel_id A yost_skeleton ID of the skeleton to operate on.
 * \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
 ***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_setSkeletonToStandardClaspedPose(yost_skeleton_id skel_id);
/********************************************//**
 * \ingroup skel_skeleton_methods
 * \brief Set the skeleton to a neutral pose with arms straight down.
 * \param[in] skel_id A yost_skeleton ID of the skeleton to operate on.
 * \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
 ***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_setSkeletonToStandardNeutralPose(yost_skeleton_id skel_id);
/********************************************//**
 * \ingroup skel_skeleton_methods
 * \brief Set an alias for a bone of the skeleton.
 * \param[in] skel_id A yost_skeleton ID of the skeleton to operate on.
 * \param[in] your_bone_name The name you would like to call the bone.
 * \param[in] bone_name The name of the bone given upon creation.
 * \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
 ***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_addBoneAlias(yost_skeleton_id skel_id, const char* your_bone_name, const char* bone_name);
/********************************************//**
 * \ingroup skel_skeleton_methods
 * \brief Add a processor to the skeleton at the given index.
 *
 * The index is to help determine when the process should run. 0 being the first process and 0xfffffffe being the last.
 * An index of YOST_SKELETON_PROCESSOR_LIST_END well add the processor to the end of the current list of processors.
 * \param[in] skel_id A yost_skeleton ID of the skeleton to operate on.
 * \param[in] index The index where the processor is to be placed.
 * \param[in] proc_id A yost_skeleton ID of the processor to give to the skeleton.
 * \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
 ***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_addProcessorToSkeleton(yost_skeleton_id skel_id, uint32_t index, yost_skeleton_id proc_id);
/********************************************//**
 * \ingroup skel_skeleton_methods
 * \brief Remove a processor from the skeleton at the given index.
 *
 * An index of YOST_SKELETON_PROCESSOR_LIST_END well remove the processor at the end of the current list of processors.
 * \param[in] skel_id A yost_skeleton ID of the skeleton to operate on.
 * \param[in] index The index of the processor to be removed.
 * \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
 ***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_removeProcessorFromSkeleton(yost_skeleton_id skel_id, uint32_t index);
/********************************************//**
 * \ingroup skel_skeleton_methods
 * \brief Get the list of processors of the skeleton.
 * \param[in] skel_id A yost_skeleton ID of the skeleton to operate on.
 * \param[out] processor_list A list of yost_skeleton IDs of the processors in the order of operation.
 * \param[in] list_len The size of the list entered.
 * \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
 ***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_getProcessorListFromSkeleton(yost_skeleton_id skel_id, yost_skeleton_id* processor_list, uint32_t list_len);
/********************************************//**
 * \ingroup skel_skeleton_methods
 * \brief Get the names of processors of the skeleton.
 * \param[in] skel_id A yost_skeleton ID of the skeleton to operate on.
 * \param[out] processor_list A list of C strings of the processors in the order of operation.
 * \param[in] list_len The size of the list entered.
 * \param[in] name_len The max size of the C strings for the list entered.
 * \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
 ***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_getProcessorNameListFromSkeleton(yost_skeleton_id skel_id, char** processor_list, uint32_t list_len, uint32_t name_len);
/********************************************//**
 * \ingroup skel_skeleton_methods
 * \brief Update the given skeleton.
 * \param[in] skel_id A yost_skeleton ID of the skeleton to operate on.
 * \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
 ***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_update(yost_skeleton_id skel_id);
/********************************************//**
* \ingroup skel_skeleton_methods
* \brief See if the skeleton has updated.
* \param[in] skel_id A yost_skeleton ID of the skeleton to operate on.
* \param[out] has_changed Whether or not the skeleton has changed since last call.
* \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_hasUpdated(yost_skeleton_id skel_id, bool* has_changed);
/********************************************//**
 * \ingroup skel_skeleton_methods
 * \brief Get the name of the root bone of the skeleton.
 * \param[in] skel_id A yost_skeleton ID of the skeleton to operate on.
 * \param[out] bone_name A C string name of the root bone of the skeleton.
 * \param[in] name_len The size of the C string entered.
 * \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
 ***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_getRootBoneName(yost_skeleton_id skel_id, char* bone_name, uint32_t name_len);
/********************************************//**
 * \ingroup skel_skeleton_methods
 * \brief Get the names of bones of the skeleton.
 * \param[in] skel_id A yost_skeleton ID of the skeleton to operate on.
 * \param[out] bone_list A list of C strings of the bone names of the skeleton.
 * \param[in] list_len The size of the list entered.
 * \param[in] name_len The max size of the C strings for the list entered.
 * \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
 ***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_getBoneNameList(yost_skeleton_id skel_id, char** bone_list, uint32_t list_len, uint32_t name_len);
/********************************************//**
 * \ingroup skel_skeleton_methods
 * \brief Set the orientation offset of the bone.
 * \param[in] skel_id A yost_skeleton ID of the skeleton to operate on.
 * \param[in] bone_name The name of the bone.
 * \param[in] quat4 A float array denoting a quaternion (X, Y, Z, W).
 * \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_setBoneOrientationOffset(yost_skeleton_id skel_id, const char* bone_name, float* quat4);
/********************************************//**
 * \ingroup skel_skeleton_methods
 * \brief Get the orientation offset of the bone.
 * \param[in] skel_id A yost_skeleton ID of the skeleton to operate on.
 * \param[in] bone_name The name of the bone.
 * \param[out] quat4 A float array denoting a quaternion (X, Y, Z, W).
 * \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_getBoneOrientationOffset(yost_skeleton_id skel_id, const char* bone_name, float* quat4);
/********************************************//**
 * \ingroup skel_skeleton_methods
 * \brief Get the orientation of the bone.
 * \param[in] skel_id A yost_skeleton ID of the skeleton to operate on.
 * \param[in] bone_name The name of the bone.
 * \param[out] quat4 A float array denoting a quaternion (X, Y, Z, W).
 * \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_getBoneOrientation(yost_skeleton_id skel_id, const char* bone_name, float* quat4);
/********************************************//**
 * \ingroup skel_skeleton_methods
 * \brief Get the position of the bone.
 * \param[in] skel_id A yost_skeleton ID of the skeleton to operate on.
 * \param[in] bone_name The name of the bone.
 * \param[out] pos3 A float array denoting a position.
 * \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_getBonePosition(yost_skeleton_id skel_id, const char* bone_name, float* pos3);
/********************************************//**
 * \ingroup skel_skeleton_methods
 * \brief Get the velocity of the bone.
 * \param[in] skel_id A yost_skeleton ID of the skeleton to operate on.
 * \param[in] bone_name The name of the bone.
 * \param[out] vel3 A float array denoting a velocity.
 * \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_getBoneVelocity(yost_skeleton_id skel_id, const char* bone_name, float* vel3);
/********************************************//**
 * \ingroup skel_skeleton_methods
 * \brief Get the acceleration of the bone.
 * \param[in] skel_id A yost_skeleton ID of the skeleton to operate on.
 * \param[in] bone_name The name of the bone.
 * \param[out] accel3 A float array denoting an acceleration.
 * \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_getBoneAcceleration(yost_skeleton_id skel_id, const char* bone_name, float* accel3);
/********************************************//**
* \ingroup skel_skeleton_methods
* \brief Get the orientation offsets from all the bones.
* \param[in] skel_id A yost_skeleton ID of the skeleton to operate on.
* \param[out] quat4 A float array denoting a quaternion (X, Y, Z, W).
* \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_getAllBoneOrientationOffset(yost_skeleton_id skel_id, float* quat4);
/********************************************//**
* \ingroup skel_skeleton_methods
* \brief Get the orientations from all the bones.
* \param[in] skel_id A yost_skeleton ID of the skeleton to operate on.
* \param[out] quat4 A float array denoting the quaternions (X, Y, Z, W).
* \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_getAllBoneOrientations(yost_skeleton_id skel_id, float* quat4);
/********************************************//**
* \ingroup skel_skeleton_methods
* \brief Get the positions from all the bones.
* \param[in] skel_id A yost_skeleton ID of the skeleton to operate on.
* \param[out] pos3 A float array denoting the positions.
* \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_getAllBonePositions(yost_skeleton_id skel_id, float* pos3);
/********************************************//**
* \ingroup skel_skeleton_methods
* \brief Get the velocities from all the bones.
* \param[in] skel_id A yost_skeleton ID of the skeleton to operate on.
* \param[out] vel3 A float array denoting the velocities.
* \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_getAllBoneVelocities(yost_skeleton_id skel_id, float* vel3);
/********************************************//**
* \ingroup skel_skeleton_methods
* \brief Get the accelerations from all the bones.
* \param[in] skel_id A yost_skeleton ID of the skeleton to operate on.
* \param[out] accel3 A float array denoting the accelerations.
* \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_getAllBoneAccelerations(yost_skeleton_id skel_id, float* accel3);
/********************************************//**
* \ingroup skel_skeleton_methods
* \brief Set the position offset of the bone.
* \param[in] skel_id A yost_skeleton ID of the skeleton to operate on.
* \param[in] bone_name The name of the bone.
* \param[in] pos_offset3 A float array denoting a position.
* \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_setBonePositionOffset(yost_skeleton_id skel_id, const char* bone_name, float* pos_offset3);
/********************************************//**
* \ingroup skel_skeleton_methods
* \brief Get the position offset of the bone.
* \param[in] skel_id A yost_skeleton ID of the skeleton to operate on.
* \param[in] bone_name The name of the bone.
* \param[out] pos_offset3 A float array denoting a position.
* \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_getBonePositionOffset(yost_skeleton_id skel_id, const char* bone_name, float* pos_offset3);
/********************************************//**
 * \ingroup skel_skeleton_methods
 * \brief Read out the skeleton hierarchy as an XML string.
 * \param[in] skel_id A yost_skeleton ID of the skeleton to operate on.
 * \param[out] buffer A C array buffer to put the hierarchy in.
 * \param[in] buffer_len The size of the buffer. Recommended size: 10000 bytes.
 * \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_extractSkeletonHierarchy(yost_skeleton_id skel_id, char* buffer, uint32_t buffer_len);
/********************************************//**
 * \ingroup skel_skeleton_methods
 * \brief Tells if the skeleton has been calibrated yet or not.
 * \param[in] skel_id A yost_skeleton ID of the skeleton to operate on.
 * \param[out] calibrated A byte value indicating if the skeleton has been calibrated.
 * \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_isCalibrated(yost_skeleton_id skel_id, bool* calibrated);
/********************************************//**
 * \ingroup skel_skeleton_methods
 * \brief Get the name of the skeleton.
 * \param[in] skel_id A yost_skeleton ID of the skeleton to operate on.
 * \param[out] skel_name A C string name of the skeleton.
 * \param[in] name_len The size of the C string entered.
 * \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
 ***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_getSkeletonName(yost_skeleton_id skel_id, char* skel_name, uint32_t name_len);
/********************************************//**
 * \ingroup skel_skeleton_methods
 * \brief Set the name of the skeleton.
 * \param[in] skel_id A yost_skeleton ID of the skeleton to operate on.
 * \param[in] skel_name A C string name of the skeleton.
 * \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
 ***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_setSkeletonName(yost_skeleton_id skel_id, const char* skel_name);
/********************************************//**
 * \ingroup skel_skeleton_methods
 * \brief Get the unit length measurement of the skeleton.
 * \param[in] skel_id A yost_skeleton ID of the skeleton to operate on.
 * \param[out] skel_unit An enum of YOST_SKELETON_UNIT.
 * \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
 ***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_getSkeletonUnit(yost_skeleton_id skel_id, YOST_SKELETON_UNIT* skel_unit);
/********************************************//**
 * \ingroup skel_skeleton_methods
 * \brief Set the unit length measurement of the skeleton.
 * \param[in] skel_id A yost_skeleton ID of the skeleton to operate on.
 * \param[in] skel_unit An enum of YOST_SKELETON_UNIT.
 * \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
 ***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_setSkeletonUnit(yost_skeleton_id skel_id, YOST_SKELETON_UNIT skel_unit);
/********************************************//**
 * \ingroup skel_skeleton_methods
 * \brief Add a bone to the skeleton.
 * \param[in] skel_id A yost_skeleton ID of the skeleton to operate on.
 * \param[in] bone_name The name of the bone to add.
 * \param[in] update_type The type of updating the bone will be using.
 * \param[in] offset3 A float array denoting a positional offset from its parent.
 * \param[in] parent_name The name of this bone's parent, or the empty string if it is the root.
 * \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
 ***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_addBoneToSkeleton(yost_skeleton_id skel_id, const char* bone_name, YOST_SKELETON_BONE_UPDATE_TYPE update_type, float* offset3, const char* parent_name);
/********************************************//**
* \ingroup skel_skeleton_methods
* \brief Remove a bone from the skeleton and reparent it's childern to it's parent.
* \param[in] skel_id A yost_skeleton ID of the skeleton to operate on.
* \param[in] bone_name The name of the bone to remove.
* \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_removeBoneFromSkeleton(yost_skeleton_id skel_id, const char* bone_name);
/********************************************//**
* \ingroup skel_skeleton_methods
* \brief Registers a bone for the polling method of gathering bone data.
* \param[in] skel_id A yost_skeleton ID of the skeleton to operate on.
* \param[in] bone_name The name of the bone to register.
* \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_utilityRegisterBone(yost_skeleton_id skel_id, const char* bone_name);
/********************************************//**
* \ingroup skel_skeleton_methods
* \brief Deregisters a bone for the polling method of gathering bone data.
* \param[in] skel_id A yost_skeleton ID of the skeleton to operate on.
* \param[in] bone_name The name of the bone to deregister.
* \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_utilityDeregisterBone(yost_skeleton_id skel_id, const char* bone_name);
/********************************************//**
* \ingroup skel_skeleton_methods
* \brief Get the names of bones registered for polling.
* \param[in] skel_id A yost_skeleton ID of the skeleton to operate on.
* \param[out] bone_list A list of C strings of the bone names in registered order.
* \param[in] list_len The size of the list entered.
* \param[in] name_len The max size of the C strings for the list entered.
* \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_utilityRegisteredBoneList(yost_skeleton_id skel_id, char** bone_list, uint32_t list_len, uint32_t name_len);
/********************************************//**
* \ingroup skel_skeleton_methods
* \brief Get the current nunmber of bones registered for polling.
* \param[in] skel_id A yost_skeleton ID of the skeleton to operate on.
* \param[out] bone_count The number of bones registered.
* \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_utilityRegisteredBoneCount(yost_skeleton_id skel_id, uint32_t* bone_count);
/********************************************//**
 * \ingroup skel_skeleton_methods
 * \brief Set the pose orientation of a bone.
 * \param[in] skel_id A yost_skeleton ID of the skeleton to operate on.
 * \param[in] bone_name The name of the bone.
 * \param[in] quat4 A float array denoting a quaternion (X, Y, Z, W).
 * \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_setBonePoseOrientation(yost_skeleton_id skel_id, const char* bone_name, float* quat4);
/********************************************//**
 * \ingroup skel_skeleton_methods
 * \brief Set the length of a bone.
 * \param[in] skel_id A yost_skeleton ID of the skeleton to operate on.
 * \param[in] bone_name The name of the bone.
 * \param[in] length The length of the bone. (This will be in the units of the skeleton).
 * \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_setBoneLength(yost_skeleton_id skel_id, const char* bone_name, float length);
/********************************************//**
* \ingroup skel_skeleton_methods
* \brief Get the length of a bone.
* \param[in] skel_id A yost_skeleton ID of the skeleton to operate on.
* \param[in] bone_name The name of the bone.
* \param[out] length The length of the bone. (This will be in the units of the skeleton).
* \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_getBoneLength(yost_skeleton_id skel_id, const char* bone_name, float* length);
/********************************************//**
 * \ingroup skel_skeleton_methods
 * \brief Set the mass of a bone.
 * \param[in] skel_id A yost_skeleton ID of the skeleton to operate on.
 * \param[in] bone_name The name of the bone.
 * \param[in] mass The mass of the bone. (Standard skeletons have a total mass of 1).
 * \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_setBoneMass(yost_skeleton_id skel_id, const char* bone_name, float mass);
/********************************************//**
* \ingroup skel_skeleton_methods
* \brief Get the mass of a bone.
* \param[in] skel_id A yost_skeleton ID of the skeleton to operate on.
* \param[in] bone_name The name of the bone.
* \param[in] mass The mass of the bone. (Standard skeletons have a total mass of 1).
* \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_getBoneMass(yost_skeleton_id skel_id, const char* bone_name, float* mass);
/********************************************//**
* \ingroup skel_processor_methods
* \brief Set the update rate for the controller data, the controller data includes the hub button state, and joystick states.
* \param[in] prio_proc_id A yost_skeleton ID of the Prio processor.
* \param[in] update_rate The rate to update the controller data. (Hub Button, and Joystick States)
* \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_setControllerUpdateRate(yost_skeleton_id prio_proc_id, uint32_t update_rate);
/********************************************//**
* \ingroup skel_processor_methods
* \brief Get the update rate for the controller data, the controller data includes the hub button state, and joystick states.
* \param[in] prio_proc_id A yost_skeleton ID of the Prio processor.
* \param[in] update_rate The rate to update the controller data. (Hub Button, and Joystick States)
* \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_getControllerUpdateRate(yost_skeleton_id prio_proc_id, uint32_t* update_rate);
/********************************************//**
*\ingroup skel_processor_methods
* \brief Starts the updating of the controller data for the PrioVR Suit.
* \param[in] prio_proc_id A yost_skeleton ID of the Prio processor.
* \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_startControllerUpdating(yost_skeleton_id prio_proc_id);
/********************************************//**
*\ingroup skel_processor_methods
* \brief Stop the updating of the controller data for the PrioVR Suit.
* \param[in] prio_proc_id A yost_skeleton ID of the Prio processor.
* \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_stopControllerUpdating(yost_skeleton_id prio_proc_id);
/********************************************//**
 * \ingroup skel_skeleton_methods
 * \brief Maps the skeleton with the device of the given device xml file.
 * \param[in] skel_id A yost_skeleton ID of the skeleton to operate on.
 * \param[in] device_xml_file File containing the mapping.
 * \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
 ***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_loadDeviceXMLMap(yost_skeleton_id skel_id, const char* device_xml_file);
/********************************************//**
* \ingroup skel_skeleton_methods
* \brief Saves the skeleton's device map to the given xml file.
* \param[in] skel_id A yost_skeleton ID of the skeleton to operate on.
* \param[in] device_xml_file The name of the exported file.
* \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_saveDeviceXMLMap(yost_skeleton_id skel_id, const char* device_xml_file);
/********************************************//**
* \ingroup skel_skeleton_methods
* \brief Resets up the skeleton with the contents of the given hierarchy file.
* \param[in] skel_id A yost_skeleton ID of the skeleton to operate on.
* \param[in] hierarchy_file File containing the hierarchy.
* \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_loadBoneXMLHierarchy(yost_skeleton_id skel_id, const char* hierarchy_file);
/********************************************//**
* \ingroup skel_skeleton_methods
* \brief Saves the XML of the skeleton to the given file name.
* \param[in] skel_id A yost_skeleton ID of the skeleton to operate on.
* \param[in] file_name The name of the exported file.
* \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_saveBoneXMLHierarchy(yost_skeleton_id skel_id, const char* file_name);
/********************************************//**
 * \ingroup skel_skeleton_methods
 * \brief Sets the skeleton to be mapped to a standard PrioVR Lite Suit layout.
 * \param[in] skel_id A yost_skeleton ID of the skeleton to operate on.
 * \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
 ***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_setStandardDeviceXmlMapPrioLiteLayout(yost_skeleton_id skel_id);
/********************************************//**
 * \ingroup skel_skeleton_methods
 * \brief Sets the skeleton to be mapped to a standard PrioVR Core Suit layout.
 * \param[in] skel_id A yost_skeleton ID of the skeleton to operate on.
 * \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
 ***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_setStandardDeviceXmlMapPrioCoreLayout(yost_skeleton_id skel_id);
/********************************************//**
 * \ingroup skel_skeleton_methods
 * \brief Sets the skeleton to be mapped to a standard PrioVR Pro Suit layout.
 * \param[in] skel_id A yost_skeleton ID of the skeleton to operate on.
 * \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
 ***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_setStandardDeviceXmlMapPrioProLayout(yost_skeleton_id skel_id);
// Processor Methods
/********************************************//**
 * \ingroup skel_processor_methods
 * \brief Destroys the processor if it is not being used.
 * \param[in] proc_id A yost_skeleton ID of the processor.
 * \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
 ***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_destroyProcessor(yost_skeleton_id proc_id);
/********************************************//**
 * \ingroup skel_processor_methods
 * \brief Creates a Prio processor, with a comunication port.
 * \param[in] com_offest This allows you to call this function multiple times to increment through the COM ports to continue getting more Prio devices.
 * \return A yost_skeleton ID, YOST_SKELETON_INVALID_ID is returned if the creation failed.
 ***********************************************/
SKEL_EXPORT yost_skeleton_id yostskel_createPrioProcessorWithComOffset(uint8_t device_type, uint8_t com_offest);
/********************************************//**
* \ingroup skel_processor_methods
* \brief Creates a Prio processor, with a PrioVR Basestation.
* \param[in] device_id The id of an already created PrioVR Basestation.
* \return A yost_skeleton ID, YOST_SKELETON_INVALID_ID is returned if the creation failed.
***********************************************/
SKEL_EXPORT yost_skeleton_id yostskel_createPrioProcessorWithPrioVRBasestation(prio_device_id device_id);
/********************************************//**
* \ingroup skel_processor_methods
* \brief Creates a Prio processor, with a PrioVR Hub.
* \param[in] device_id The id of an already created PrioVR Hub.
* \return A yost_skeleton ID, YOST_SKELETON_INVALID_ID is returned if the creation failed.
***********************************************/
SKEL_EXPORT yost_skeleton_id yostskel_createPrioProcessorWithPrioVRHub(prio_device_id device_id);
/********************************************//**
 * \ingroup skel_processor_methods
 * \brief Retrieves the Prio API's device id for the Prio processor.
 * \param[in] prio_proc_id A yost_skeleton ID of the Prio processor.
 * \param[in] device_id A device ID used for Prio API commands.
 * \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
 ***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_getPrioProcessorDeviceIndex(yost_skeleton_id prio_proc_id, uint32_t* device_id);
/********************************************//**
* \ingroup skel_processor_methods
* \brief Retrieves the Prio API's device type for the Prio processor.
* \param[in] prio_proc_id A yost_skeleton ID of the Prio processor.
* \param[in] device_type A device type used for Prio API commands. \li 0: PrioVR Type Unknown \li 1: PrioVR Basestation \li 2: PrioVR Hub
* \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_getPrioProcessorDeviceType(yost_skeleton_id prio_proc_id, uint8_t* device_type);
/********************************************//**
 * \ingroup skel_processor_methods
 * \brief Calibrates the PrioVR Suit.
 * \param[in] prio_proc_id A yost_skeleton ID of the Prio processor.
 * \param[in] wait_time The amount of time in seconds to wait before calibrating.
 * \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
 ***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_calibratePrioProcessor(yost_skeleton_id prio_proc_id, float wait_time);
/********************************************//**
 * \ingroup skel_processor_methods
 * \brief Sets up the standard streaming of data for the PrioVR Suit. The standard stream includes only the quaterion data.
 * \param[in] prio_proc_id A yost_skeleton ID of the Prio processor.
 * \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
 ***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_setupStandardStreamPrioProcessor(yost_skeleton_id prio_proc_id);
/********************************************//**
 * \ingroup skel_processor_methods
 * \brief Sets up the standard streaming of data with the inclusion of acceleration for the PrioVR Suit.
 * \param[in] prio_proc_id A yost_skeleton ID of the Prio processor.
 * \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
 ***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_setupStandardStreamWithAccelerationPrioProcessor(yost_skeleton_id prio_proc_id);
/********************************************//**
 * \ingroup skel_processor_methods
 * \brief Starts the streaming of data for the PrioVR Suit.
 * \param[in] prio_proc_id A yost_skeleton ID of the Prio processor.
 * \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
 ***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_startPrioProcessor(yost_skeleton_id prio_proc_id);
/********************************************//**
 * \ingroup skel_processor_methods
 * \brief Stops the streaming of data for the PrioVR suit.
 * \param[in] prio_proc_id A yost_skeleton ID of the Prio processor.
 * \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
 ***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_stopPrioProcessor(yost_skeleton_id prio_proc_id);
/********************************************//**
* \ingroup skel_processor_methods
* \brief Retrieves the battery level of the PrioVR Hub.
* \param[in] prio_proc_id A yost_skeleton ID of the Prio processor.
* \param[out] battery_level The current battery level.
* \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_getHubBatteryLevel(yost_skeleton_id prio_proc_id, uint8_t* battery_level);
/********************************************//**
* \ingroup skel_processor_methods
* \brief Retrieves the battery status of the PrioVR Hub.
* \param[in] prio_proc_id A yost_skeleton ID of the Prio processor.
* \param[out] battery_level The current battery level.
* \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_getHubBatteryStatus(yost_skeleton_id prio_proc_id, uint8_t* battery_status);
/********************************************//**
 * \ingroup skel_processor_methods
 * \brief Retrieves the state of the PrioVR Hub button.
 * \param[in] prio_proc_id A yost_skeleton ID of the Prio processor.
 * \param[out] button_state Writes the state of the hub buttons [0-Neither Pressed, 1-Left Pressed, 2-Right Pressed, 3-Both Pressed].
 * \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
 ***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_getHubButtonPrioProcessor(yost_skeleton_id prio_proc_id, uint8_t* button_state);
/********************************************//**
 * \ingroup skel_processor_methods
 * \brief Retrieves the state of the PrioVR Hub button.
 * \param[in] prio_proc_id A yost_skeleton ID of the Prio processor.
 * \param[in] button_index The button wanting to retrive. [0-Left or 1-Right]
 * \param[out] button_state Writes the state of the hub buttons.
 * \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
 ***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_getHubButtonByIndexPrioProcessor(yost_skeleton_id prio_proc_id, uint8_t button_index, uint8_t* button_state);
/********************************************//**
* \ingroup skel_processor_methods
* \brief Retrieves the state of the PrioVR joysticks.
* \param[in] prio_proc_id A yost_skeleton ID of the Prio processor.
* \param[out] left_joystick_state Writes the state of the left joystick, in the order (X-axis, Y-axis, Trigger, Button state).
* \param[out] right_joystick_state Writes the state of the right joystick, in the order (X-axis, Y-axis, Trigger, Button state).
* \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_getJoyStickStatePrioProcessor(yost_skeleton_id prio_proc_id, uint8_t* left_joystick_state, uint8_t* right_joystick_state);
/********************************************//**
* \ingroup skel_processor_methods
* \brief Retrieves the state of the requested PrioVR joystick.
* \param[in] prio_proc_id A yost_skeleton ID of the Prio processor.
* \param[in] joystick_index The joystick the user wants to retrive. [0-Left or 1-Right]
* \param[out] joystick_state Writes the state of the given joystick, in the order (X-axis, Y-axis, Trigger, Button state).
* \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_getJoyStickStateByIndexPrioProcessor(yost_skeleton_id prio_proc_id, uint8_t joystick_index, uint8_t* joystick_state);
/********************************************//**
* \ingroup skel_processor_methods
* \brief Retrieves the state of the requested PrioVR joystick button.
* \param[in] prio_proc_id A yost_skeleton ID of the Prio processor.
* \param[in] button_index The button the user wants to retrive.
* \param[out] button_state Writes the state of the joystick button.
* \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_getJoyStickButtonStatePrioProcessor(yost_skeleton_id prio_proc_id, uint8_t button_index, uint8_t* button_state);
/********************************************//**
* \ingroup skel_processor_methods
* \brief Retrieves the state of the requested PrioVR joystick axis.
* \param[in] prio_proc_id A yost_skeleton ID of the Prio processor.
* \param[in] axis_index The axis the user wants to retrive.
* \param[out] axis_state Writes the state of the joystick axis.
* \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_getJoyStickAxisStatePrioProcessor(yost_skeleton_id prio_proc_id, uint8_t axis_index, uint8_t* axis_state);
/********************************************//**
* \ingroup skel_processor_methods
* \brief Retrieves a bitfield representing where PrioVR joysticks are plugged in.
* \param[in] prio_proc_id A yost_skeleton ID of the Prio processor.
* \param[out] active_joysticks A bitfield representing which PrioVR Joystick are plugged in. \li 0: None \li 1: Left Joystick \li 2: Right Joystick \li 3: Both
* \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_getActiveJoySticksPrioProcessor(yost_skeleton_id prio_proc_id, uint8_t* active_joysticks);
/********************************************//**
* \ingroup skel_processor_methods
* \brief Retrieve whether or not a PrioVR joystick is plugged in on that index.
* \param[in] prio_proc_id A yost_skeleton ID of the Prio processor.
* \param[in] joystick_index The axis the user wants to see is active.
* \param[out] active Whether or not a PrioVR joystick is active at the index.
* \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_isJoystickActiveByIndexPrioProcessor(yost_skeleton_id prio_proc_id, uint8_t joystick_index, bool* active);
/********************************************//**
* \ingroup skel_processor_methods
* \brief Retrieves if the PrioVR suit has changed configuration since the last time the skeleton was calibrated.
* \param[in] prio_proc_id A yost_skeleton ID of the Prio processor.
* \param[out] changed Whether or not the Prio processor's suit has changed state.
* \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_hasPrioProcessorUpdatedState(yost_skeleton_id prio_proc_id, bool* changed);
/********************************************//**
 * \ingroup skel_processor_methods
 * \brief Creates a Pedestrian Tracking processor.
 * \return A yost_skeleton ID, YOST_SKELETON_INVALID_ID is returned if the creation failed.
 ***********************************************/
SKEL_EXPORT yost_skeleton_id yostskel_createPedestrianTrackingProcessor();
/********************************************//**
* \ingroup skel_processor_methods
* \brief Resets the Pedestrian Tracking processor.
* \param[in] proc_id A yost_skeleton ID of the processor.
* \return A yost_skeleton ID, YOST_SKELETON_INVALID_ID is returned if the creation failed.
***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_resetPedestrianTrackingProcessor(yost_skeleton_id proc_id);
/********************************************//**
* \ingroup skel_processor_methods
* \brief Retrieves the translation of the root bone since the last call.
* \param[in] proc_id A yost_skeleton ID of the processor.
* \param[out] float[3] The translation of the root bone as an array of floats.
* \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_getSkeletonTranslation(yost_skeleton_id proc_id, float* translation_vector);
/********************************************//**
* \ingroup skel_processor_methods
* \brief Retrieves the translation of the root bone as a precentage of the skeleton height since the last call.
* \param[in] proc_id A yost_skeleton ID of the processor.
* \param[out] float[3] The translation of the root bone as an array of floats.
* \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_getSkeletonTranslationAsPrecentageOfHeight(yost_skeleton_id proc_id, float* translation_vector);
/********************************************//**
* \ingroup skel_processor_methods
* \brief starts tracking the given bone.
* \param[in] proc_id A yost_skeleton ID of the processor.
* \param[in] char[] A null ended char array of the bone name.
* \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_startTrackingBonePedestrianTrackingProcessor(yost_skeleton_id proc_id, const char* bone_name);
/********************************************//**
* \ingroup skel_processor_methods
* \brief stops tracking the given bone.
* \param[in] proc_id A yost_skeleton ID of the processor.
* \param[in] char[] A null ended char array of the bone name.
* \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_stopTrackingBonePedestrianTrackingProcessor(yost_skeleton_id proc_id, const char* bone_name);
/********************************************//**
* \ingroup skel_processor_methods
* \brief Sets the maximum distance a bone will be lowered when processing the lowest bone.
* \param[in] proc_id A yost_skeleton ID of the processor.
* \param[in] float A max distance for the bone to be lowered.
* \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_setMaxCertaintyPedestrianTrackingProcessor(yost_skeleton_id proc_id, float certainty);
/********************************************//**
* \ingroup skel_processor_methods
* \brief Gets the maximum distance a bone will be lowered when processing the lowest bone.
* \param[in] proc_id A yost_skeleton ID of the processor.
* \param[out] float A max distance for the bone to be lowered.
* \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_getMaxCertaintyPedestrianTrackingProcessor(yost_skeleton_id proc_id, float* certainty);
/********************************************//**
* \ingroup skel_processor_methods
* \brief Sets the amount that the variance in quaternions should be multiplied when calculating the distance to lower the bone.
* \param[in] proc_id A yost_skeleton ID of the processor.
* \param[in] float The amount to multiply the variance by.
* \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_setVarianceMultiplyFactorPedestrianTrackingProcessor(yost_skeleton_id proc_id, float multiply_factor);
/********************************************//**
* \ingroup skel_processor_methods
* \brief Gets the amount that the variance in quaternions should be multiplied when calculating the distance to lower the bone.
* \param[in] proc_id A yost_skeleton ID of the processor.
* \param[out] float The amount the variance is being multiplied by.
* \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_getVarianceMultiplyFactorPedestrianTrackingProcessor(yost_skeleton_id proc_id, float* multiply_factor);
/********************************************//**
* \ingroup skel_processor_methods
* \brief Sets the maximum number of quaternions to be used when calculating the variance.
* \param[in] proc_id A yost_skeleton ID of the processor.
* \param[in] int The maximum number of quaternions to be used.
* \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_setVarianceDataLengthPedestrianTrackingProcessor(yost_skeleton_id proc_id, int variance_length);
/********************************************//**
* \ingroup skel_processor_methods
* \brief Gets the maximum number of quaternions to be used when calculating the variance.
* \param[in] proc_id A yost_skeleton ID of the processor.
* \param[out] int The maximum number of quaternions to be used.
* \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_getVarianceDataLengthPedestrianTrackingProcessor(yost_skeleton_id proc_id, int* variance_length);
/********************************************//**
* \ingroup skel_processor_methods
* \brief Sets which bone is used as the root bone in calculations of translation
* \param[in] proc_id A yost_skeleton ID of the processor.
* \param[in] char[] A null ended char array of the bone name.
* \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_setRootBonePedestrianTrackingProcessor(yost_skeleton_id proc_id, const char* bone_name);
/********************************************//**
* \ingroup skel_processor_methods
* \brief Gets the currently pinned bones name and location.
* \param[in] proc_id A yost_skeleton ID of the processor.
* \param[out] float[3] where the bone is anchored.
* \param[out] char[] A null ended char array of the bone name.
* \param[out] int The length of the buffer for the bone name.
* \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_getPinnedBoneDataPedestrianTrackingProcessor(yost_skeleton_id proc_id, float* anchor_point, char* bone_name, int bone_name_length);
/********************************************//**
* \ingroup skel_processor_methods
* \brief Creates a Smoothing processor.
* \return A yost_skeleton ID, YOST_SKELETON_INVALID_ID is returned if the creation failed.
***********************************************/
SKEL_EXPORT yost_skeleton_id yostskel_createSmoothingProcessor();
/********************************************//**
* \ingroup skel_processor_methods
* \brief Sets the minimum amount all the bones can be smoothed by.
* \param[in] proc_id A yost_skeleton ID of the processor.
* \param[in] min_smoothing_factor The amount from 0 to 1 (0% to 100%) to smooth by.
* \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_setAllMinSmoothingFactorSmoothingProcessor(yost_skeleton_id proc_id, float min_smoothing_factor);
/********************************************//**
* \ingroup skel_processor_methods
* \brief Sets the maximum amount the all bones can be smoothed by.
* \param[in] proc_id A yost_skeleton ID of the processor.
* \param[in] max_smoothing_factor The amount from 0 to 1 (0% to 100%) to smooth by.
* \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_setAllMaxSmoothingFactorSmoothingProcessor(yost_skeleton_id proc_id, float max_smoothing_factor);
/********************************************//**
*\ingroup skel_processor_methods
* \brief Sets the lower bound of the quaternion variance to map to a smoothing facto for all bones.
* \param[in] proc_id A yost_skeleton ID of the processor.
* \param[in] lower_variance_bound The amount of variance in the quaternions.
* \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_setAllLowerVarianceBoundSmoothingProcessor(yost_skeleton_id proc_id, float lower_variance_bound);
/********************************************//**
* \ingroup skel_processor_methods
* \brief Sets the upper bound of the quaternion variance to map to a smoothing factor for all bones.
* \param[in] proc_id A yost_skeleton ID of the processor.
* \param[in] upper_variance_bound The amount of variance in the quaternions.
* \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_setAllUpperVarianceBoundSmoothingProcessor(yost_skeleton_id proc_id, float upper_variance_bound);
/********************************************//**
* \ingroup skel_processor_methods
* \brief Sets the amount that the variance in quaternions should be multiplied when calculating the distance to lower all the bones.
* \param[in] proc_id A yost_skeleton ID of the processor.
* \param[in] multiply_factor The amount the variance is to be multiplied by.
* \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_setAllVarianceMultiplyFactorSmoothingProcessor(yost_skeleton_id proc_id, float multiply_factor);
/********************************************//**
* \ingroup skel_processor_methods
* \brief Sets the maximum number of quaternions to be used when calculating the variance for all the bones.
* \param[in] proc_id A yost_skeleton ID of the processor.
* \param[in] variance_length The maximum number of quaternions to be used.
* \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_setAllVarianceDataLengthSmoothingProcessor(yost_skeleton_id proc_id, int variance_length);
/********************************************//**
* \ingroup skel_processor_methods
* \brief Sets the minimum amount the bone can be smoothed by.
* \param[in] proc_id A yost_skeleton ID of the processor.
* \param[in] bone_name A null ended char array of the bone name.
* \param[in] min_smoothing_factor The amount from 0 to 1 (0% to 100%) to smooth by.
* \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_setBoneMinSmoothingFactorSmoothingProcessor(yost_skeleton_id proc_id, const char* bone_name, float min_smoothing_factor);
/********************************************//**
* \ingroup skel_processor_methods
* \brief Gets the minimum amount the bone can be smoothed by.
* \param[in] proc_id A yost_skeleton ID of the processor.
* \param[in] bone_name A null ended char array of the bone name.
* \param[out] min_smoothing_factor The amount from 0 to 1 (0% to 100%) that the bone is being smoothed by.
* \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_getBoneMinSmoothingFactorSmoothingProcessor(yost_skeleton_id proc_id, const char* bone_name, float* min_smoothing_factor);
/********************************************//**
* \ingroup skel_processor_methods
* \brief Sets the maximum amount the bone can be smoothed by.
* \param[in] proc_id A yost_skeleton ID of the processor.
* \param[in] bone_name A null ended char array of the bone name.
* \param[in] max_smoothing_factor The amount from 0 to 1 (0% to 100%) to smooth by.
* \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_setBoneMaxSmoothingFactorSmoothingProcessor(yost_skeleton_id proc_id, const char* bone_name, float max_smoothing_factor);
/********************************************//**
* \ingroup skel_processor_methods
* \brief Gets the maximum amount the bone can be smoothed by.
* \param[in] proc_id A yost_skeleton ID of the processor.
* \param[in] bone_name A null ended char array of the bone name.
* \param[out] max_smoothing_factor The amount from 0 to 1 (0% to 100%) that the bone is being smoothed by.
* \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_getBoneMaxSmoothingFactorSmoothingProcessor(yost_skeleton_id proc_id, const char* bone_name, float* max_smoothing_factor);
/********************************************//**
* \ingroup skel_processor_methods
* \brief Sets the lower bound of the quaternion variance to map to a smoothing factor for a bone.
* \param[in] proc_id A yost_skeleton ID of the processor.
* \param[in] bone_name A null ended char array of the bone name.
* \param[in] lower_variance_bound The amount of variance in the quaternions.
* \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_setBoneLowerVarianceBoundSmoothingProcessor(yost_skeleton_id proc_id, const char* bone_name, float lower_variance_bound);
/********************************************//**
* \ingroup skel_processor_methods
* \brief Gets the lower bound of the quaternion variance to map to a smoothing factor for a bone.
* \param[in] proc_id A yost_skeleton ID of the processor.
* \param[in] bone_name A null ended char array of the bone name.
* \param[out] lower_variance_bound The amount of variance in the quaternions used for the lower bound.
* \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_getBoneLowerVarianceBoundSmoothingProcessor(yost_skeleton_id proc_id, const char* bone_name, float* lower_variance_bound);
/********************************************//**
* \ingroup skel_processor_methods
* \brief Sets the upper bound of the quaternion variance to map to a smoothing factor for a bone.
* \param[in] proc_id A yost_skeleton ID of the processor.
* \param[in] bone_name A null ended char array of the bone name.
* \param[in] upper_variance_bound The amount of variance in the quaternions.
* \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_setBoneUpperVarianceBoundSmoothingProcessor(yost_skeleton_id proc_id, const char* bone_name, float upper_variance_bound);
/********************************************//**
* \ingroup skel_processor_methods
* \brief Gets the upper bound of the quaternion variance to map to a smoothing factor for a bone.
* \param[in] proc_id A yost_skeleton ID of the processor.
* \param[in] bone_name A null ended char array of the bone name.
* \param[out] upper_variance_bound The amount of variance in the quaternions used for the upper bound.
* \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_getBoneUpperVarianceBoundSmoothingProcessor(yost_skeleton_id proc_id, const char* bone_name, float* upper_variance_bound);
/********************************************//**
* \ingroup skel_processor_methods
* \brief Sets the amount that the variance in quaternions should be multiplied when calculating the distance to lower the bone.
* \param[in] proc_id A yost_skeleton ID of the processor.
* \param[in] bone_name A null ended char array of the bone name.
* \param[in] multiply_factor The amount the variance is to be multiplied by.
* \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_setBoneVarianceMultiplyFactorSmoothingProcessor(yost_skeleton_id proc_id, const char* bone_name, float multiply_factor);
/********************************************//**
* \ingroup skel_processor_methods
* \brief Gets the amount that the variance in quaternions should be multiplied when calculating the distance to lower the bone.
* \param[in] proc_id A yost_skeleton ID of the processor.
* \param[in] bone_name A null ended char array of the bone name.
* \param[out] multiply_factor The amount the variance is being multiplied by.
* \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_getBoneVarianveMultiplyFactorSmoothingProcessor(yost_skeleton_id proc_id, const char* bone_name, float* multiply_factor);
/********************************************//**
* \ingroup skel_processor_methods
* \brief Sets the maximum number of quaternions to be used when calculating the variance for a bone.
* \param[in] proc_id A yost_skeleton ID of the processor.
* \param[in] bone_name A null ended char array of the bone name.
* \param[in] variance_length The maximum number of quaternions to be used.
* \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_setBoneVarianceDataLengthSmoothingProcessor(yost_skeleton_id proc_id, const char* bone_name, int variance_length);
/********************************************//**
* \ingroup skel_processor_methods
* \brief Gets the maximum number of quaternions to be used when calculating the variance for a bone.
* \param[in] proc_id A yost_skeleton ID of the processor.
* \param[in] bone_name A null ended char array of the bone name.
* \param[out] variance_length The maximum number of quaternions to be used.
* \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_getBoneVarianceDataLengthSmoothingProcessor(yost_skeleton_id proc_id, const char* bone_name, int* variance_length);
/********************************************//**
 * \ingroup skel_processor_methods
 * \brief Creates a TSS processor.
 * \return A yost_skeleton ID, YOST_SKELETON_INVALID_ID is returned if the creation failed.
 ***********************************************/
SKEL_EXPORT yost_skeleton_id yostskel_createTssProcessor();
/********************************************//**
 * \ingroup skel_processor_methods
 * \brief Calibrates the Tss suit.
 * \param[in] proc_id A yost_skeleton ID of the Tss processor.
 * \param[in] wait_time The amount of time in seconds to wait before calibrating.
 * \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
 ***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_calibrateTssProcessor(yost_skeleton_id proc_id, float wait_time);
/********************************************//**
 * \ingroup skel_processor_methods
 * \brief Starts the streaming of data for the Tss suit.
 * \param[in] proc_id A yost_skeleton ID of the Tss processor.
 * \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
 ***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_startTssProcessor(yost_skeleton_id proc_id);
/********************************************//**
 * \ingroup skel_processor_methods
 * \brief Stops the streaming of data for the Tss suit.
 * \param[in] proc_id A yost_skeleton ID of the Tss processor.
 * \return YOST_SKELETON_NO_ERROR(0) if successful, non zero if an error occurred.
 ***********************************************/
SKEL_EXPORT YOST_SKELETON_ERROR yostskel_stopTssProcessor(yost_skeleton_id proc_id);

#ifdef __cplusplus
}
#endif

#endif // _YOST_SKELETON_API_H_