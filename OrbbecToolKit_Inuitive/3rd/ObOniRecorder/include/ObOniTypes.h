#ifndef _OB_ONI_TYPES_H_
#define _OB_ONI_TYPES_H_

#include <stdint.h>
#include "ObOniDefines.h"


namespace ob_oni_record_api
{
	typedef enum EObOniStatus
	{
		OB_ONI_STATUS_OK = 0,
		OB_ONI_STATUS_ERROR = 1,
		OB_ONI_STATUS_NOT_IMPLEMENTED = 2,
		OB_ONI_STATUS_NOT_SUPPORTED = 3,
		OB_ONI_STATUS_BAD_PARAMETER = 4,
		OB_ONI_STATUS_OUT_OF_FLOW = 5,
		OB_ONI_STATUS_NO_DEVICE = 6,
		OB_ONI_STATUS_NOT_WRITE_PUBLIC_KEY = 7,
		OB_ONI_STATUS_PUBLIC_KEY_MD5_VERIFY_FAILED = 8,
		OB_ONI_STATUS_NOT_WRITE_MD5 = 9,
		OB_ONI_STATUS_RSKEY_VERIFY_FAILED = 10,
		OB_ONI_STATUS_TIME_OUT = 102,
	}ObOniStatus;

	typedef enum EObOniSensorType
	{
		OB_ONI_SENSOR_IR = 1,
		OB_ONI_SENSOR_COLOR = 2,
		OB_ONI_SENSOR_DEPTH = 3,
		OB_ONI_SENSOR_PHASE = 4,
	}ObOniSensorType;

	typedef enum EObOniPixelFormat
	{
		// Depth
		OB_ONI_PIXEL_FORMAT_DEPTH_1_MM = 100,
		OB_ONI_PIXEL_FORMAT_DEPTH_100_UM = 101,
		OB_ONI_PIXEL_FORMAT_SHIFT_9_2 = 102,
		OB_ONI_PIXEL_FORMAT_SHIFT_9_3 = 103,

		// Color
		OB_ONI_PIXEL_FORMAT_RGB888 = 200,
		OB_ONI_PIXEL_FORMAT_YUV422 = 201,
		OB_ONI_PIXEL_FORMAT_GRAY8 = 202,
		OB_ONI_PIXEL_FORMAT_GRAY16 = 203,
		OB_ONI_PIXEL_FORMAT_JPEG = 204,
		OB_ONI_PIXEL_FORMAT_YUYV = 205,
	}ObOniPixelFormat;

	typedef struct tagObOniVersion
	{
		int major;
		int minor;
		int maintenance;
		int build;
	}ObOniVersion, *ObOniVersionHandle;

	typedef struct tagObOniFovInfo
	{
		float hFov;
		float vFov;
		bool hasFovProperty;
	}ObOniFovInfo, *ObOniFovInfoHandle;

	typedef struct tagCropInfo
	{
		int enabled;
		int originX;
		int originY;
		int width;
		int height;
	}CropInfo, *CropInfoHandle;

	typedef struct tagObOniCropInfo
	{
		CropInfo info;
		bool hasCropProperty;
	}ObOniCropInfo, *ObOniCropInfoHandle;

	typedef struct tagObOniMirrorInfo
	{
		int mirror;
		bool hasMirrorProperty;
	}ObOniMirrorInfo, *ObOniMirrorInfoHandle;

	typedef struct tagObOniVideoMode
	{
		ObOniPixelFormat pixelFormat;
		int resolutionX;
		int resolutionY;
		int fps;
	}ObOniVideoMode, *ObOniVideoModeHandle;

	typedef struct tagObOniDeviceInfo
	{
		char uri[OB_ONI_MAX_STR];
		char vendor[OB_ONI_MAX_STR];
		char name[OB_ONI_MAX_STR];
		uint16_t usbVendorId;
		uint16_t usbProductId;
	}ObOniDeviceInfo, *ObOniDeviceInfoHandle;

	typedef struct tagObOniSensorInfo
	{
		ObOniSensorType sensorType;
		int numSupportedVideoModes;
		ObOniVideoModeHandle pSupportedVideoModes;
	}ObOniSensorInfo, *ObOniSensorInfoHandle;

	typedef struct tagObOniAttachInfo
	{
		int maxDepth;
		int requiredFrameSize;

		ObOniFovInfo fovInfo;
		ObOniCropInfo cropInfo;
		ObOniVideoMode videoMode;
		ObOniDeviceInfo devInfo;
		ObOniSensorInfo sensorInfo;
		ObOniMirrorInfo mirrorInfo;
	}ObOniAttachInfo, *ObOniAttachInfoHandle;

	typedef struct tagObOniFrame
	{
		int dataSize;
		void* data;

		ObOniSensorType sensorType;
		uint64_t timestamp;
		int frameIndex;

		int width;
		int height;

		ObOniVideoMode videoMode;
		bool croppingEnabled;
		int cropOriginX;
		int cropOriginY;

		int stride;
	}ObOniFrame, *ObOniFrameHandle;

	typedef struct tagObOniRecorder
	{
		void* precorder;
	}ObOniRecorder, *ObOniRecorderHandle;
}

#endif //_OB_ONI_TYPES_H_
