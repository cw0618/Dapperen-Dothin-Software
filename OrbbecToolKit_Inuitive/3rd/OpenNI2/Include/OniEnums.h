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
#ifndef ONIENUMS_H
#define ONIENUMS_H

namespace openni
{

    /** Possible failure values */
    typedef enum
    {
        STATUS_OK = 0,
        STATUS_ERROR = 1,
        STATUS_NOT_IMPLEMENTED = 2,
        STATUS_NOT_SUPPORTED = 3,
        STATUS_BAD_PARAMETER = 4,
        STATUS_OUT_OF_FLOW = 5,
        STATUS_NO_DEVICE = 6,
        STATUS_NOT_WRITE_PUBLIC_KEY = 7,
        STATUS_PUBLIC_KEY_MD5_VERIFY_FAILED = 8,
        STATUS_NOT_WRITE_MD5 = 9,
        STATUS_RSKEY_VERIFY_FAILED = 10,
        STATUS_TIME_OUT = 102,
    } Status;

    /** The source of the stream */
    typedef enum
    {
        SENSOR_IR = 1,
        SENSOR_COLOR = 2,
        SENSOR_DEPTH = 3,
        SENSOR_PHASE = 4,
        SENSOR_AI = 5,
    } SensorType;

    typedef enum
    {
        SENSOR_ID_NONE,
        SENSOR_ID_PLECO = 0x005A,
        SENSOR_ID_S5K33D = 0x303D,
    } SensorID;

    /** All available formats of the output of a stream */
    typedef enum
    {
        PIXEL_FORMAT_NONE,

        // Depth
        PIXEL_FORMAT_DEPTH_1_MM = 100,
        PIXEL_FORMAT_DEPTH_100_UM = 101,
        PIXEL_FORMAT_SHIFT_9_2 = 102,
        PIXEL_FORMAT_SHIFT_9_3 = 103,

        // Color
        PIXEL_FORMAT_RGB888 = 200,
        PIXEL_FORMAT_YUV422 = 201,
        PIXEL_FORMAT_GRAY8 = 202,
        PIXEL_FORMAT_GRAY16 = 203,
        PIXEL_FORMAT_JPEG = 204,
        PIXEL_FORMAT_YUYV = 205,
        PIXEL_FORMAT_H264 = 206,
        PIXEL_FORMAT_MJPEG = 207,

        /// AI
        PIXEL_FORMAT_JOINT_2D = 0x0001,
        PIXEL_FORMAT_JOINT_3D = 0x0002,
        PIXEL_FORMAT_BODY_MASK = 0x0004,
        PIXEL_FORMAT_FLOOR_INFO = 0x0008,
        PIXEL_FORMAT_BODY_SHAPE = 0x0010,
        PIXEL_FORMAT_PHASE = 0x0020,
        PIXEL_FORMAT_DEPTH_IR = 0x0040,
        PIXEL_FORMAT_DEPTH_PHASE = 0x0080,
        PIXEL_FORMAT_FACE = 0x0100,
        PIXEL_FORMAT_GESTURE = 0x0200,
    } PixelFormat;

    typedef enum
    {
        DEVICE_STATE_OK = 0,
        DEVICE_STATE_ERROR = 1,
        DEVICE_STATE_NOT_READY = 2,
        DEVICE_STATE_EOF = 3
    } DeviceState;

    typedef enum
    {
        SERIAL_NONE,
        SERIAL_IR,
        SERIAL_RGB,
        SERIAL_DEPTH,
        SERIAL_DEVICE,
        SERIAL_PRODUCT,
    } SerialType;

    typedef enum
    {
        IMAGE_REGISTRATION_OFF = 0,
        IMAGE_REGISTRATION_DEPTH_TO_COLOR = 1,
    } ImageRegistrationMode;

    typedef enum
    {
        PARAMS_REGISTRATION_OFF = 0,
        PARAMS_REGISTRATION_DEPTH_TO_COLOR = 1,
        PARAMS_REGISTRATION_USE_DISTORTION = 2,
    } ParamsRegistrationMode;

    typedef enum
    {
        JOINT_HEAD = 0,
        JOINT_NECK = 1,
        JOINT_MID_SPINE = 2,
        JOINT_RIGHT_SHOULDER = 3,
        JOINT_LEFT_SHOULDER = 4,
        JOINT_RIGHT_ELBOW = 5,
        JOINT_LEFT_ELBOW = 6,
        JOINT_RIGHT_WRIST = 7,
        JOINT_LEFT_WRIST = 8,
        JOINT_RIGHT_HIP = 9,
        JOINT_LEFT_HIP = 10,
        JOINT_RIGHT_KNEE = 11,
        JOINT_LEFT_KNEE = 12,
        JOINT_RIGHT_ANKLE = 13,
        JOINT_LEFT_ANKLE = 14,
        JOINT_MAX = 15,
    } JointType;

    typedef enum
    {
        FIGURE_NONE,
        FIGURE_THIN,
        FIGURE_OVERWEIGHT,
        FIGURE_MEDIUM_BUILT,
    } FigureTypes;

    typedef enum
    {
        AI_STATUS_NONE,
        AI_STATUS_NOT_TRACKING,
        AI_STATUS_TRACKING_STARTED,
        AI_STATUS_TRACKING,
        AI_STATUS_LOST,
    } AIStatus;

    typedef enum
    {
        FREQUENCY_MODE_NONE = -1,
        SINGLE_FREQ_NOSHUFFLE = 0,
        DUAL_FREQ_NOSHUFFLE = 1,
        SINGLE_FREQ_SHUFFLE = 2,
        DUAL_FREQ_SHUFFLE = 3,
        SINGLE_FREQ_NOSHUFFLE_BINNING = 4,
        DUAL_FREQ_NOSHUFFLE_BINNING = 5,
        SINGLE_FREQ_SHUFFLE_BINNING = 6,
        DUAL_FREQ_SHUFFLE_BINNING = 7,
        FREQUENCY_MODE_MAX,
    } FrequencyMode;

    typedef enum
    {
        FRAME_TYPE_NONE = -1,
        FRAME_TYPE_PHASE,
        FRAME_TYPE_IR,
        FRAME_TYPE_DEPTH,
        FRAME_TYPE_AMP,
    } FrameType;

    typedef enum
    {
        TOF_SENSOR_MODE_NONE,
        S5K_4TAP_SINGLE_FREQ_1FRAME = 1,
        S5K_4TAP_DUAL_FREQ_2FRAME = 2,
        S5K_4TAP_SINGLE_FREQ_2FRAME = 3,
        S5K_4TAP_DUAL_FREQ_4FRAME = 4,
        S5K_4TAP_SINGLE_FREQ_1FRAME_BINNING = 5,
        S5K_4TAP_DUAL_FREQ_2FRAME_BINNING = 6,
        S5K_4TAP_SINGLE_FREQ_2FRAME_BINNING = 7,
        S5K_4TAP_DUAL_FREQ_4FRAME_BINNING = 8,
        PLECO_3TAP_SINGLE_FREQ_1FRAME = 100,
        PLECO_3TAP_DUAL_FREQ_2FRAME = 101,
        PLECO_3TAP_SINGLE_FREQ_3FRAME = 102,
        PLECO_3TAP_DUAL_FREQ_6FRAME = 103,
        PLECO_3TAP_SINGLE_FREQ_1FRAME_BINNING = 104,
        PLECO_3TAP_DUAL_FREQ_2FRAME_BINNING = 105,
        PLECO_3TAP_SINGLE_FREQ_3FRAME_BINNING = 106,
        PLECO_3TAP_DUAL_FREQ_6FRAME_BINNING = 107,
    } TOFSensorMode;

    typedef enum
    {
        FILE_SUFFIX_NONE,
        FILE_SUFFIX_TEXT,
        FILE_SUFFIX_BINARY,
    } FileSuffix;

    typedef enum
    {
        FILE_CATEGORY_NONE,
        FILE_CATEGORY_FW,
        FILE_CATEGORY_ROM,
        FILE_CATEGORY_D2C,
        FILE_CATEGORY_CALIB,
        FILE_CATEGORY_FILTER,
        FILE_CATEGORY_COMMON,
    } FileCategory;

    typedef enum
    {
        SERVICE_ADB = 0x0010,
        SERVICE_RNDIS = 0x0020,
        SERVICE_MAX = 0xFFFF,
    } ServiceType;

    typedef enum
    {
        SERVICE_OFF,
        SERVICE_ON,
    } ServiceState;

    static const int TIMEOUT_NONE = 0;
    static const int TIMEOUT_FOREVER = -1;

} // namespace openni

#endif // ONIENUMS_H
