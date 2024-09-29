/*****************************************************************************
*                                                                            *
*  OpenNI 2.x Alpha                                                          *
*  Copyright (C) 2012 PrimeSense Ltd.                                        *
*                                                                            *
*  This file is part of OpenNI.                                              *
*                                                                            *
*  Licensed under the Apache License, Version 2.0 (the "License");           *
*  you may not use this file except in compliance with the License.          *
*  You may obtain a copy of the License at                                   *
*                                                                            *
*      http://www.apache.org/licenses/LICENSE-2.0                            *
*                                                                            *
*  Unless required by applicable law or agreed to in writing, software       *
*  distributed under the License is distributed on an "AS IS" BASIS,         *
*  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  *
*  See the License for the specific language governing permissions and       *
*  limitations under the License.                                            *
*                                                                            *
*****************************************************************************/
#ifndef ONICENUMS_H
#define ONICENUMS_H

/** Possible failure values */
typedef enum
{
    ONI_STATUS_OK = 0,
    ONI_STATUS_ERROR = 1,
    ONI_STATUS_NOT_IMPLEMENTED = 2,
    ONI_STATUS_NOT_SUPPORTED = 3,
    ONI_STATUS_BAD_PARAMETER = 4,
    ONI_STATUS_OUT_OF_FLOW = 5,
    ONI_STATUS_NO_DEVICE = 6,
    ONI_STATUS_NOT_WRITE_PUBLIC_KEY = 7,
    ONI_STATUS_PUBLIC_KEY_MD5_VERIFY_FAILED = 8,
    ONI_STATUS_NOT_WRITE_MD5 = 9,
    ONI_STATUS_RSKEY_VERIFY_FAILED = 10,
    ONI_STATUS_TIME_OUT = 102,
} OniStatus;

/** The source of the stream */
typedef enum
{
    ONI_SENSOR_IR = 1,
    ONI_SENSOR_COLOR = 2,
    ONI_SENSOR_DEPTH = 3,
    ONI_SENSOR_PHASE = 4,
    ONI_SENSOR_AI = 5,
} OniSensorType;

/** All available formats of the output of a stream */
typedef enum
{
    ONI_PIXEL_FORMAT_NONE,

    /// Depth
    ONI_PIXEL_FORMAT_DEPTH_1_MM = 100,
    ONI_PIXEL_FORMAT_DEPTH_100_UM = 101,
    ONI_PIXEL_FORMAT_SHIFT_9_2 = 102,
    ONI_PIXEL_FORMAT_SHIFT_9_3 = 103,

    /// Color
    ONI_PIXEL_FORMAT_RGB888 = 200,
    ONI_PIXEL_FORMAT_YUV422 = 201,
    ONI_PIXEL_FORMAT_GRAY8 = 202,
    ONI_PIXEL_FORMAT_GRAY16 = 203,
    ONI_PIXEL_FORMAT_JPEG = 204,
    ONI_PIXEL_FORMAT_YUYV = 205,
    ONI_PIXEL_FORMAT_H264 = 206,
    ONI_PIXEL_FORMAT_MJPEG = 207,

    /// AI
    ONI_PIXEL_FORMAT_JOINT_2D = 0x0001,
    ONI_PIXEL_FORMAT_JOINT_3D = 0x0002,
    ONI_PIXEL_FORMAT_BODY_MASK = 0x0004,
    ONI_PIXEL_FORMAT_FLOOR_INFO = 0x0008,
    ONI_PIXEL_FORMAT_BODY_SHAPE = 0x0010,
    ONI_PIXEL_FORMAT_PHASE = 0x0020,
    ONI_PIXEL_FORMAT_DEPTH_IR = 0x0040,
    ONI_PIXEL_FORMAT_DEPTH_PHASE = 0x0080,
    ONI_PIXEL_FORMAT_FACE = 0x0100,
    ONI_PIXEL_FORMAT_GESTURE = 0x0200,
} OniPixelFormat;

typedef enum
{
    ONI_DEVICE_STATE_OK = 0,
    ONI_DEVICE_STATE_ERROR = 1,
    ONI_DEVICE_STATE_NOT_READY = 2,
    ONI_DEVICE_STATE_EOF = 3
} OniDeviceState;

typedef enum
{
    ONI_SENSOR_ID_NONE,
    ONI_SENSOR_ID_PLECO = 0x005A,
    ONI_SENSOR_ID_S5K33D = 0x303D,
} OniSensorID;

typedef enum
{
    ONI_SERIAL_NONE,
    ONI_SERIAL_IR,
    ONI_SERIAL_RGB,
    ONI_SERIAL_DEPTH,
    ONI_SERIAL_DEVICE,
    ONI_SERIAL_PRODUCT,
} OniSerialType;

typedef enum
{
    ONI_IMAGE_REGISTRATION_OFF = 0,
    ONI_IMAGE_REGISTRATION_DEPTH_TO_COLOR = 1,
} OniImageRegistrationMode;

typedef enum
{
    ONI_FREQUENCY_MODE_NONE = -1,
    ONI_SINGLE_FREQ_NOSHUFFLE = 0,
    ONI_DUAL_FREQ_NOSHUFFLE = 1,
    ONI_SINGLE_FREQ_SHUFFLE = 2,
    ONI_DUAL_FREQ_SHUFFLE = 3,
    ONI_SINGLE_FREQ_NOSHUFFLE_BINNING = 4,
    ONI_DUAL_FREQ_NOSHUFFLE_BINNING = 5,
    ONI_SINGLE_FREQ_SHUFFLE_BINNING = 6,
    ONI_DUAL_FREQ_SHUFFLE_BINNING = 7,
    ONI_FREQUENCY_MODE_MAX,
} OniFrequencyMode;

typedef enum
{
    ONI_FRAME_TYPE_NONE = -1,
    ONI_FRAME_TYPE_PHASE,
    ONI_FRAME_TYPE_IR,
    ONI_FRAME_TYPE_DEPTH,
    ONI_FRAME_TYPE_AMP,
} OniFrameType;

typedef enum
{
    ONI_TOF_SENSOR_MODE_NONE,
    ONI_S5K_4TAP_SINGLE_FREQ_1FRAME = 1,
    ONI_S5K_4TAP_DUAL_FREQ_2FRAME = 2,
    ONI_S5K_4TAP_SINGLE_FREQ_2FRAME = 3,
    ONI_S5K_4TAP_DUAL_FREQ_4FRAME = 4,
    ONI_S5K_4TAP_SINGLE_FREQ_1FRAME_BINNING = 5,
    ONI_S5K_4TAP_DUAL_FREQ_2FRAME_BINNING = 6,
    ONI_S5K_4TAP_SINGLE_FREQ_2FRAME_BINNING = 7,
    ONI_S5K_4TAP_DUAL_FREQ_4FRAME_BINNING = 8,
    ONI_PLECO_3TAP_SINGLE_FREQ_1FRAME = 100,
    ONI_PLECO_3TAP_DUAL_FREQ_2FRAME = 101,
    ONI_PLECO_3TAP_SINGLE_FREQ_3FRAME = 102,
    ONI_PLECO_3TAP_DUAL_FREQ_6FRAME = 103,
    ONI_PLECO_3TAP_SINGLE_FREQ_1FRAME_BINNING = 104,
    ONI_PLECO_3TAP_DUAL_FREQ_2FRAME_BINNING = 105,
    ONI_PLECO_3TAP_SINGLE_FREQ_3FRAME_BINNING = 106,
    ONI_PLECO_3TAP_DUAL_FREQ_6FRAME_BINNING = 107,
} OniTOFSensorMode;

enum
{
    ONI_TIMEOUT_NONE = 0,
    ONI_TIMEOUT_FOREVER = -1,
};

typedef enum
{
    ONI_JOINT_HEAD = 0,
    ONI_JOINT_NECK = 1,
    ONI_JOINT_MID_SPINE = 2,
    ONI_JOINT_RIGHT_SHOULDER = 3,
    ONI_JOINT_LEFT_SHOULDER = 4,
    ONI_JOINT_RIGHT_ELBOW = 5,
    ONI_JOINT_LEFT_ELBOW = 6,
    ONI_JOINT_RIGHT_WRIST = 7,
    ONI_JOINT_LEFT_WRIST = 8,
    ONI_JOINT_RIGHT_HIP = 9,
    ONI_JOINT_LEFT_HIP = 10,
    ONI_JOINT_RIGHT_KNEE = 11,
    ONI_JOINT_LEFT_KNEE = 12,
    ONI_JOINT_RIGHT_ANKLE = 13,
    ONI_JOINT_LEFT_ANKLE = 14,
    ONI_JOINT_MAX = 15,
} OniJointType;

typedef enum
{
    ONI_FIGURE_NONE,
    ONI_FIGURE_THIN,
    ONI_FIGURE_OVERWEIGHT,
    ONI_FIGURE_MEDIUM_BUILT,
} OniFigureTypes;

typedef enum
{
    ONI_AI_STATUS_OK,
    ONI_AI_STATUS_NOT_TRACKING,
    ONI_AI_STATUS_TRACKING_STARTED,
    ONI_AI_STATUS_TRACKING,
    ONI_AI_STATUS_LOST,
} OniAIStatus;

typedef enum
{
    ONI_FILE_SUFFIX_NONE,
    ONI_FILE_SUFFIX_TEXT,
    ONI_FILE_SUFFIX_BINARY,
} OniFileSuffix;

typedef enum
{
    ONI_FILE_CATEGORY_NONE,
    ONI_FILE_CATEGORY_FW,
    ONI_FILE_CATEGORY_ROM,
    ONI_FILE_CATEGORY_D2C,
    ONI_FILE_CATEGORY_CALIB,
    ONI_FILE_CATEGORY_FILTER,
    ONI_FILE_CATEGORY_COMMON,
} OniFileCategory;

typedef enum
{
    ONI_SERVICE_ADB = 0x0010,
    ONI_SERVICE_RNDIS = 0x0020,
    ONI_SERVICE_MAX = 0xFFFF,
} OniServiceType;

typedef enum
{
    ONI_SERVICE_OFF,
    ONI_SERVICE_ON,
} OniServiceState;

#endif // ONICENUMS_H
