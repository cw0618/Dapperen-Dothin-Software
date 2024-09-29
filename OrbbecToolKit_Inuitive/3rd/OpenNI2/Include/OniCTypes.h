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
#ifndef ONICTYPES_H
#define ONICTYPES_H

#include <vector>
#include "OniPlatform.h"
#include "OniCEnums.h"

#if ONI_PLATFORM == ONI_PLATFORM_ANDROID_ARM
#include <XnLog.h>
#endif

/** Basic types **/
typedef int OniBool;

#ifndef TRUE
#define TRUE 1
#endif //TRUE
#ifndef FALSE
#define FALSE 0
#endif //FALSE

#define ONI_MAX_STR                256
#define ONI_MAX_SENSORS            10
#define ONI_LOG_MAX_MESSAGE_LENGTH 2048
#define ONI_MAX_BODIES             10
#define ONI_BODY_POINT_NUM         3
#define ONI_MAX_META               6
#define ONI_MAX_FRAME              3

struct OniCallbackHandleImpl;
typedef struct OniCallbackHandleImpl* OniCallbackHandle;

/** Holds an OpenNI version number, which consists of four separate numbers in the format: @c major.minor.maintenance.build. For example: 2.0.0.20. */
typedef struct
{
    /** Major version number, incremented for major API restructuring. */
    int major;
    /** Minor version number, incremented when significant new features added. */
    int minor;
    /** Maintenance build number, incremented for new releases that primarily provide minor bug fixes. */
    int maintenance;
    /** Build number. Incremented for each new API build. Generally not shown on the installer and download site. */
    int build;
} OniVersion;

typedef int OniHardwareVersion;

/** Description of the output: format and resolution */
typedef struct
{
    OniPixelFormat pixelFormat;
    int resolutionX;
    int resolutionY;
    int fps;
} OniVideoMode;

/** List of supported video modes by a specific source */
typedef struct
{
    OniSensorType sensorType;
    int numSupportedVideoModes;
    OniVideoMode *pSupportedVideoModes;
} OniSensorInfo;

typedef struct
{
    OniSensorType type;
    OniSensorID id;
} OniSensorIDMap;

typedef struct
{
    OniSerialType type;
    char serial[ONI_MAX_STR];
} OniSerialNumberMap;

/** Basic description of a device */
typedef struct
{
    char uri[ONI_MAX_STR];
    char vendor[ONI_MAX_STR];
    char name[ONI_MAX_STR];
    uint16_t usbVendorId;
    uint16_t usbProductId;
} OniDeviceInfo;

typedef struct
{
    float l_intr_p[4]; ///< [fx,fy,cx,cy]
    float r_intr_p[4]; ///< [fx,fy,cx,cy]
    float r2l_r[9];    ///< [r00,r01,r02;r10,r11,r12;r20,r21,r22]
    float r2l_t[3];    ///< [t1,t2,t3]
    float l_k[5];      ///< [k1,k2,p1,p2,k3]
    float r_k[5];
    uint32_t width;
    uint32_t height;
} OBCameraParams;

typedef struct
{
    float rotation[9];    ///< 3x3 Rotation matrix stored in row major order
    float translation[3]; ///< Translation vector, x,y,z (in millimeters)
} OniCameraExtrinsics;

typedef struct
{
    float fx;             ///< Focal length x
    float fy;             ///< Focal length y
    float cx;             ///< Principal point in image, x
    float cy;             ///< Principal point in image, y
    float k1;             ///< k1 radial distortion coefficient
    float k2;             ///< k2 radial distortion coefficient
    float k3;             ///< k3 radial distortion coefficient
    float p1;             ///< Tangential distortion coefficient 1
    float p2;             ///< Tangential distortion coefficient 2
} OniCameraIntrinsics;

typedef struct
{
    int width;                      ///< Resolution width of the calibration sensor
    int height;                     ///< Resolution height of the calibration sensor
    float metric_radius;            ///< Reserve: Max FOV of the camera
    OniCameraIntrinsics intrinsics; ///< Intrinsic calibration data
    OniCameraExtrinsics extrinsics; ///< Extrinsic calibration data
} OniCameraMatrix;

typedef struct
{
    OniCameraMatrix color;
    OniCameraMatrix depth;
} OniCalibrationCamera;

template<class T> struct OniPoint2D
{
    T x;
    T y;
};

template<class T> struct OniPoint3D
{
    T x;
    T y;
    T z;
};

typedef struct
{
    double score;
    OniJointType type;
    OniPoint3D<double> position;
} OniJoint;

typedef struct
{
    float ratio;
    float height;
    float waist;
    float waistline;                       ///< Reserve
    float bust;                            ///< Reserve
    float hips;                            ///< Reserve
    float shoulder;                        ///< Reserve
    OniFigureTypes figure;                 ///< Reserve (@see OniFigureTypes)
    OniPoint2D<int> points[ONI_BODY_POINT_NUM];
} OniBodyShape;

typedef struct
{
    int id;
    OniAIStatus jointStatus;
    OniAIStatus bodyShapeStatus;

    int jointWidth;
    int jointHeight;
    OniPixelFormat jointFormat;
    OniJoint joints[ONI_JOINT_MAX];
    OniBodyShape bodyShape;
} OniBody;

typedef struct
{
    int num;
    OniBody bodies[ONI_MAX_BODIES];
} OniBodyList;

typedef struct
{
    int width;
    int height;
    uint64_t address;
} OniBitmapMask;

typedef struct
{
    OniBitmapMask mask;
    OniAIStatus status;
} OniBodyMask;

typedef struct
{
    OniPoint3D<float> center;
    OniPoint3D<float> normal;
} OniPlane;

typedef struct
{
    OniPlane plane;
    OniBitmapMask mask;
    OniAIStatus status;
} OniFloorInfo;

typedef struct
{
    int width;
    int height;
    int frameIndex;
    int groupIndex;
    bool shuffle;
    float temperTX;
    float temperRX;
    float temperDelay;
    int frequency[2];
    int integration[2];
    float dutyCycle[2];
    OniSensorID id;
    OniFrameType type;
    OniTOFSensorMode mode;
} OniMetadata;

typedef struct
{
    int width;
    int height;
    int stride;
    int planeNum;
    uint64_t address;
    OniMetadata meta[ONI_MAX_META];
} OniTOFFrame;

typedef struct
{
    int size;
    OniAIStatus status;
    OniTOFFrame frames[ONI_MAX_FRAME];
} OniFrameSet;

/** All information of the current frame */
typedef struct
{
    int dataSize;
    void* data;

    OniSensorType sensorType;
    uint64_t timestamp;
    int frameIndex;

    int width;
    int height;

    OniVideoMode videoMode;
    OniBool croppingEnabled;
    int cropOriginX;
    int cropOriginY;

    int stride;
    int extraLine;
} OniFrame;

typedef struct
{
    OniBodyList bodyList;
    OniBodyMask bodyMask;
    OniFloorInfo floorInfo;
    OniFrameSet frameSet;
} OniAIFrame;

typedef struct
{
    char cmd[ONI_MAX_STR];
    char resp[ONI_MAX_STR];
} OniSerialCmd;

typedef struct
{
    char* path;
    OniFileSuffix suffix;
    OniFileCategory category;
} OniFileAttributes;

typedef struct
{
    uint16_t type;  ///< @see OniServiceType
    uint16_t value; ///< @see OniServiceState
} OniService;

struct _OniDevice;
typedef struct _OniDevice* OniDeviceHandle;

struct _OniStream;
typedef struct _OniStream* OniStreamHandle;

struct _OniRecorder;
typedef struct _OniRecorder* OniRecorderHandle;

typedef void (ONI_CALLBACK_TYPE* OniNewFrameCallback)(OniStreamHandle stream, void* pCookie);
typedef void (ONI_CALLBACK_TYPE* OniGeneralCallback)(void* pCookie);
typedef void (ONI_CALLBACK_TYPE* OniDeviceInfoCallback)(const OniDeviceInfo* pInfo, void* pCookie);
typedef void (ONI_CALLBACK_TYPE* OniDeviceStateCallback)(const OniDeviceInfo* pInfo, OniDeviceState deviceState, void* pCookie);

typedef void* (ONI_CALLBACK_TYPE* OniFrameAllocBufferCallback)(int size, void* pCookie);
typedef void (ONI_CALLBACK_TYPE* OniFrameFreeBufferCallback)(void* data, void* pCookie);

#if ONI_PLATFORM == ONI_PLATFORM_ANDROID_ARM

typedef void (ONI_CALLBACK_TYPE* OniAndroidLogRedirectCallback)(const XnLogEntry* pEntry, void* pCookie);

#endif

typedef struct
{
    OniDeviceInfoCallback       deviceConnected;
    OniDeviceInfoCallback       deviceDisconnected;
    OniDeviceStateCallback      deviceStateChanged;
} OniDeviceCallbacks;

typedef struct
{
    int enabled;
    int originX;
    int originY;
    int width;
    int height;
} OniCropping;

// Pixel types
/**
Pixel type used to store depth images.
*/
typedef uint16_t OniDepthPixel;
typedef uint16_t OniIRPixel;
typedef uint16_t OniPhasePixel;
typedef uint16_t OniBodyPixel;

/**
Pixel type used to store 16-bit grayscale images
*/
typedef uint16_t OniGrayscale16Pixel;

/**
Pixel type used to store 8-bit grayscale/bayer images
*/
typedef uint8_t OniGrayscale8Pixel;

#pragma pack (push, 1)

/** Holds the value of a single color image pixel in 24-bit RGB format. */
typedef struct
{
    /* Red value of this pixel. */
    uint8_t r;
    /* Green value of this pixel. */
    uint8_t g;
    /* Blue value of this pixel. */
    uint8_t b;
} OniRGB888Pixel;

/**
 Holds the value of two pixels in YUV422 format (Luminance/Chrominance,16-bits/pixel).
 The first pixel has the values y1, u, v.
 The second pixel has the values y2, u, v.
 */
typedef struct
{
    /** First chrominance value for two pixels, stored as blue luminance difference signal. */
    uint8_t u;
    /** Overall luminance value of first pixel. */
    uint8_t y1;
    /** Second chrominance value for two pixels, stored as red luminance difference signal. */
    uint8_t v;
    /** Overall luminance value of second pixel. */
    uint8_t y2;
} OniYUV422DoublePixel;

/**
 Holds the value of two pixels in YUV422 format (Luminance/Chrominance,16-bits/pixel).
 The first pixel has the values y1, u, v.
 The second pixel has the values y2, u, v.
 */
typedef struct
{
    /** Overall luminance value of first pixel. */
    uint8_t y1;
    /** First chrominance value for two pixels, stored as blue luminance difference signal. */
    uint8_t u;
    /** Overall luminance value of second pixel. */
    uint8_t y2;
    /** Second chrominance value for two pixels, stored as red luminance difference signal. */
    uint8_t v;
} OniYUYVDoublePixel;

typedef struct
{
    uint8_t p1;
    uint8_t p2;
    uint8_t p3;
    uint8_t p4;
    uint8_t p5;
} OniRaw10Packed;

#pragma pack (pop)

typedef struct
{
    int frameIndex;
    OniStreamHandle stream;
} OniSeek;

#endif // ONICTYPES_H
