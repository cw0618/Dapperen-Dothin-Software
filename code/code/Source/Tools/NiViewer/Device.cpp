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
// --------------------------------
// Includes
// --------------------------------
#include "OpenNI.h"
#include "Device.h"
#include "Draw.h"
#include <math.h>
#include <XnLog.h>
#include <PS1080.h>
#include <KinectProperties.h>
#include"OBCProperties.h"
// --------------------------------
// Defines
// --------------------------------
#define MAX_STRINGS 20

#define ZOOM_CROP_VGA_MODE_X_RES 640
#define ZOOM_CROP_VGA_MODE_Y_RES 480
#define ZOOM_CROP_HIRES_MODE_X_RES 1280
#define ZOOM_CROP_HIRES_MODE_Y_RES 1024

// --------------------------------
// Global Variables
// --------------------------------

DeviceParameter g_Registration;
DeviceParameter g_Resolution;
DeviceParameter g_frequencyMode;

bool g_bIsDepthOn = false;
bool g_bIsColorOn = false;
bool g_bIsIROn = false;
bool g_bIsPhaseOn = false;
bool g_bIsAIOn = false;
bool g_bIsJoint2D = false;

openni::Device g_device;
openni::PlaybackControl* g_pPlaybackControl;

openni::VideoStream g_depthStream;
openni::VideoStream g_colorStream;
openni::VideoStream g_irStream;
openni::VideoStream g_phaseStream;
openni::VideoStream g_AIStream;

openni::VideoFrameRef g_depthFrame;
openni::VideoFrameRef g_colorFrame;
openni::VideoFrameRef g_irFrame;
openni::VideoFrameRef g_phaseFrame;
openni::AIFrameRef g_AIFrame;

const openni::SensorInfo* g_depthSensorInfo = NULL;
const openni::SensorInfo* g_colorSensorInfo = NULL;
const openni::SensorInfo* g_irSensorInfo = NULL;
const openni::SensorInfo* g_phaseSensorInfo = NULL;
const openni::SensorInfo* g_AISensorInfo = NULL;

// --------------------------------
// Code
// --------------------------------
void initConstants()
{
	// 	// Primary Streams
	int nIndex = 0;

	// Registration
	nIndex = 0;
	g_Registration.pValues[nIndex++] = openni::IMAGE_REGISTRATION_OFF;
	g_Registration.pValueToName[FALSE] = "Off";
	g_Registration.pValues[nIndex++] = openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR;
	g_Registration.pValueToName[TRUE] = "Depth -> Image";
	g_Registration.nValuesCount = nIndex;

	// Frequency mode.
	nIndex = 0;
	g_frequencyMode.pValues[nIndex++] = openni::SINGLE_FREQ_NOSHUFFLE;
	g_frequencyMode.pValueToName[0] = "Single Freq Noshuffle";
	g_frequencyMode.pValues[nIndex++] = openni::DUAL_FREQ_NOSHUFFLE;
	g_frequencyMode.pValueToName[1] = "Dual Freq Noshuffle";
	g_frequencyMode.pValues[nIndex++] = openni::SINGLE_FREQ_SHUFFLE;
	g_frequencyMode.pValueToName[2] = "Single Freq Shuffle";
	g_frequencyMode.pValues[nIndex++] = openni::DUAL_FREQ_SHUFFLE;
	g_frequencyMode.pValueToName[3] = "Dual Freq Shuffle";
	g_frequencyMode.pValues[nIndex++] = openni::SINGLE_FREQ_NOSHUFFLE_BINNING;
	g_frequencyMode.pValueToName[4] = "Single Freq Noshuffle Binning";
	g_frequencyMode.pValues[nIndex++] = openni::DUAL_FREQ_NOSHUFFLE_BINNING;
	g_frequencyMode.pValueToName[5] = "Dual Freq Noshuffle Binning";
	g_frequencyMode.pValues[nIndex++] = openni::SINGLE_FREQ_SHUFFLE_BINNING;
	g_frequencyMode.pValueToName[6] = "Single Freq Shuffle Binning";
	g_frequencyMode.pValues[nIndex++] = openni::DUAL_FREQ_SHUFFLE_BINNING;
	g_frequencyMode.pValueToName[7] = "Dual Freq Shuffle Binning";
	g_frequencyMode.nValuesCount = nIndex;
}

const char* getFormatName(openni::PixelFormat format)
{
	switch (format)
	{
	case openni::PIXEL_FORMAT_DEPTH_1_MM:
		return "1 mm";
	case openni::PIXEL_FORMAT_DEPTH_100_UM:
		return "100 um";
	case openni::PIXEL_FORMAT_SHIFT_9_2:
		return "Shifts 9.2";
	case openni::PIXEL_FORMAT_SHIFT_9_3:
		return "Shifts 9.3";
	case openni::PIXEL_FORMAT_RGB888:
		return "RGB888";
	case openni::PIXEL_FORMAT_YUV422:
		return "YUV422";
	case openni::PIXEL_FORMAT_YUYV:
		return "YUYV";
	case openni::PIXEL_FORMAT_GRAY8:
		return "Grayscale 8-bit";
	case openni::PIXEL_FORMAT_GRAY16:
		return "Grayscale 16-bit";
	case openni::PIXEL_FORMAT_JPEG:
		return "JPEG";
	case openni::PIXEL_FORMAT_MJPEG:
		return "MJPEG";
	case openni::PIXEL_FORMAT_H264:
		return "H264";
	case openni::PIXEL_FORMAT_JOINT_2D:
		return "Joint 2D";
	case openni::PIXEL_FORMAT_JOINT_3D:
		return "Joint 3D";
	case openni::PIXEL_FORMAT_BODY_MASK:
		return "Body Mask";
	case openni::PIXEL_FORMAT_FLOOR_INFO:
		return "Floor Plane";
	case openni::PIXEL_FORMAT_BODY_SHAPE:
		return "Body Shape";
	case openni::PIXEL_FORMAT_PHASE:
		return "Phase";
	case openni::PIXEL_FORMAT_DEPTH_IR:
		return "Depth & IR";
	default:
		return "Unknown";
	}
}

int checkResFmt(openni::VideoStream& stream)
{
	openni::VideoMode mode = stream.getVideoMode();
	const openni::Array<openni::VideoMode>& modes = stream.getSensorInfo().getSupportedVideoModes();
	for (int i = 0; i < modes.getSize(); ++i)
	{
		if (mode.getResolutionX() == modes[i].getResolutionX() &&
			mode.getResolutionY() == modes[i].getResolutionY() &&
			mode.getPixelFormat() == modes[i].getPixelFormat())
			return openni::STATUS_OK;
	}

	printf("Not supported video mode <%d x %d, fmt %d>, used the default one <%d x %d, fmt %d>\n",
		mode.getResolutionX(),
		mode.getResolutionY(),
		mode.getPixelFormat(),
		modes[0].getResolutionX(),
		modes[0].getResolutionY(),
		modes[0].getPixelFormat());

	return stream.setVideoMode(modes[0]);
}

int openStream(openni::Device& device, const char* name, openni::SensorType sensorType, SensorOpenType openType, openni::VideoStream& stream, const openni::SensorInfo** ppSensorInfo, bool* pbIsStreamOn)
{
	*ppSensorInfo = device.getSensorInfo(sensorType);
	*pbIsStreamOn = false;

	if (openType == SENSOR_OFF)
	{
		return 0;
	}

	if (*ppSensorInfo == NULL)
	{
		if (openType == SENSOR_ON)
		{
			printf("No %s sensor available\n", name);
			return -1;
		}
		else
		{
			return 0;
		}
	}

	openni::Status nRetVal = stream.create(device, sensorType);
	if (nRetVal != openni::STATUS_OK)
	{
		if (openType == SENSOR_ON)
		{
			printf("Failed to create %s stream:\n%s\n", openni::OpenNI::getExtendedError(), name);
			return -2;
		}
		else
		{
			return 0;
		}
	}

	checkResFmt(stream);
	nRetVal = stream.start();
	if (nRetVal != openni::STATUS_OK)
	{
		stream.destroy();

		if (openType == SENSOR_ON)
		{
			printf("Failed to start '%s' stream: \n%s\n", name, openni::OpenNI::getExtendedError());
			return -3;
		}
		else
		{
			return 0;
		}
	}
	if (sensorType == openni::SENSOR_PHASE)
	{
		stream.stop();
	}
	*pbIsStreamOn = true;

	return 0;
}

int openCommon(openni::Device& device, DeviceConfig config)
{
	g_pPlaybackControl = g_device.getPlaybackControl();

	int ret = openStream(device, "color", openni::SENSOR_COLOR, config.openColor, g_colorStream, &g_colorSensorInfo, &g_bIsColorOn);
	if (ret != 0)
	{
		return ret;
	}

	ret = openStream(device, "depth", openni::SENSOR_DEPTH, config.openDepth, g_depthStream, &g_depthSensorInfo, &g_bIsDepthOn);
	if (ret != 0)
	{
		return ret;
	}

	ret = openStream(device, "IR", openni::SENSOR_IR, config.openIR, g_irStream, &g_irSensorInfo, &g_bIsIROn);
	if (ret != 0)
	{
		return ret;
	}

	ret = openStream(device, "phase", openni::SENSOR_PHASE, config.openPhase, g_phaseStream, &g_phaseSensorInfo, &g_bIsPhaseOn);
	if (ret != 0)
	{
		return ret;
	}

	ret = openStream(device, "AI", openni::SENSOR_AI, config.openAI, g_AIStream, &g_AISensorInfo, &g_bIsAIOn);
	if (ret != 0)
	{
		return ret;
	}

	initConstants();

	readFrame();

	return 0;
}

class OpenNIDeviceListener : public openni::OpenNI::DeviceStateChangedListener,
	public openni::OpenNI::DeviceDisconnectedListener
{
public:
	virtual void onDeviceStateChanged(const openni::DeviceInfo* pInfo, openni::DeviceState errorState)
	{
		if (strcmp(pInfo->getUri(), g_device.getDeviceInfo().getUri()) == 0)
		{
			if (errorState != 0)
			{
				setErrorState("Device is in error state! (error %d)", errorState);
			}
			else
			{
				setErrorState("");
			}
		}
	}
	virtual void onDeviceDisconnected(const openni::DeviceInfo* pInfo)
	{
		if (strcmp(pInfo->getUri(), g_device.getDeviceInfo().getUri()) == 0)
		{
			setErrorState("Device disconnected!");
		}
	}
};

openni::Status openDevice(const char* uri, DeviceConfig config)
{
	openni::Status nRetVal = openni::OpenNI::initialize();
	if (nRetVal != openni::STATUS_OK)
	{
		return nRetVal;
	}

	// Register to OpenNI events.
	static OpenNIDeviceListener deviceListener;

	openni::OpenNI::addDeviceDisconnectedListener(&deviceListener);
	openni::OpenNI::addDeviceStateChangedListener(&deviceListener);

	// Open the requested device.
	nRetVal = g_device.open(uri);
	if (nRetVal != openni::STATUS_OK)
	{
		return nRetVal;
	}
	//ObcTofFrameMode tofFrameMode;
	//tofFrameMode.out_mode = OB_TOF_OUT_MODE_A_B;
	//tofFrameMode.freqmode = OBC_DUAL_FREQUENCY;
	//tofFrameMode.requestfreq[0].frequency = 80;
	//tofFrameMode.requestfreq[1].frequency = 100;
	//tofFrameMode.requestfreq[0].duty_cycle = 13;
	//tofFrameMode.requestfreq[1].duty_cycle = 15;
	//tofFrameMode.requestfreq[0].integration_time = 1000;
	//int iRet = g_device.setProperty(OBC_GROUP_FRAME_MODE, &tofFrameMode, sizeof(ObcTofFrameMode));
	//ObcTofFrameMode frameMode;
	//memset(&frameMode, 0, sizeof(ObcTofFrameMode));
	//int frameModeSize = sizeof(ObcTofFrameMode);
	//iRet = g_device.getProperty(OBC_GROUP_FRAME_MODE, &frameMode, &frameModeSize);
	//printf("howard freone %d two %d\n", frameMode.requestfreq[0].frequency, frameMode.requestfreq[1].frequency);
	//int phaseCount = 8;
	//iRet = g_device.setProperty(310, &phaseCount, sizeof(phaseCount));
	//iRet = g_device.setProperty(233, &value, sizeof(value));
	//int ret = g_device.setProperty(207, &freq, sizeof(freq));
	//g_device.setProperty(209, &duty, sizeof(duty));
	//g_device.setProperty(206, &time, sizeof(time));
	int value = 3500;
	//int twosize = sizeof(value);
	//printf("howard freqMode %d \n", value);
	//ret = g_device.getProperty(207, &getfre, &size);
	//printf("howard freone %d two %d\n", (getfre >> 8), (getfre & 0xff));
	//ret = g_device.getProperty(209, &getfre, &size);
	//printf("howard dutyone %d two %d\n", (getfre >> 8), (getfre & 0xff));
	//int iRet = g_device.setProperty(12, &value, sizeof(value));
	if (0 != openCommon(g_device, config))
	{
		return openni::STATUS_ERROR;
	}

	return openni::STATUS_OK;
}

openni::Status openDeviceFromList(DeviceConfig config)
{
	openni::Status rc = openni::OpenNI::initialize();
	if (rc != openni::STATUS_OK)
	{
		return rc;
	}

	openni::Array<openni::DeviceInfo> deviceList;
	openni::OpenNI::enumerateDevices(&deviceList);

	for (int i = 0; i < deviceList.getSize(); ++i)
	{
		printf("[%d] %s [%s] (%s)\n", i + 1, deviceList[i].getName(), deviceList[i].getVendor(), deviceList[i].getUri());
	}

	printf("\n");
	int chosen = 1;

	do
	{
		printf("Choose device to open (1) [0 to exit]: ");

		int rc = scanf("%d", &chosen);

		if (rc <= 0 || chosen == 0)
		{
			return openni::STATUS_ERROR;
		}

	} while (chosen < 1 || chosen > deviceList.getSize());

	g_device.open(deviceList[chosen - 1].getUri());

	if (rc != openni::STATUS_OK)
	{
		return rc;
	}

	if (0 != openCommon(g_device, config))
	{
		return openni::STATUS_ERROR;
	}

	return openni::STATUS_OK;
}

void closeDevice()
{
	g_bIsIROn = false;
	g_bIsDepthOn = false;
	g_bIsColorOn = false;
	g_bIsPhaseOn = false;
	g_bIsAIOn = false;

	g_depthStream.stop();
	g_colorStream.stop();
	g_irStream.stop();
	g_phaseStream.stop();
	g_AIStream.stop();

	g_depthStream.destroy();
	g_colorStream.destroy();
	g_irStream.destroy();
	g_phaseStream.destroy();
	g_AIStream.destroy();

	g_device.close();

	openni::OpenNI::shutdown();
}

void readFrame()
{
	openni::Status rc = openni::STATUS_OK;

	openni::VideoStream* streams[] = { &g_depthStream, &g_colorStream, &g_irStream, &g_phaseStream, &g_AIStream };
	uint32_t numStream = sizeof(streams) / sizeof(streams[0]);

	int changedIndex = -1;
	while (rc == openni::STATUS_OK)
	{
		rc = openni::OpenNI::waitForAnyStream(streams, numStream, &changedIndex, 0);
		if (rc == openni::STATUS_OK)
		{
			switch (changedIndex)
			{
			case 0:
				g_depthStream.readFrame(&g_depthFrame); break;
			case 1:
				g_colorStream.readFrame(&g_colorFrame); break;
			case 2:
				g_irStream.readFrame(&g_irFrame); break;
			case 3:
				g_phaseStream.readFrame(&g_phaseFrame);
				
				break;
			case 4:
				g_AIStream.readFrame(&g_AIFrame); break;
			default:
				printf("Error in wait\n");
			}
			openni::Metadata metadata = g_phaseFrame.getMetadata();
			OniFrame* oniFrame = g_phaseFrame._getFrame();
			int phaseCount = oniFrame->extraLine;
			//printf(" width %d,height %d , highfreq %d ,lowfreq %d,phaseCount %d\n", metadata.getWidth(), metadata.getHeight(),metadata.getHighFreq(),metadata.getLowFreq() ,phaseCount);
		}
	}
}

void changeRegistration(int value)
{
	openni::ImageRegistrationMode mode = (openni::ImageRegistrationMode)value;
	if (!g_device.isValid() || !g_device.isImageRegistrationModeSupported(mode))
	{
		return;
	}

	g_device.setImageRegistrationMode(mode);
}

void changeFrequencyMode(int nValue)
{
	openni::FrequencyMode mode = (openni::FrequencyMode)nValue;
	if (!g_device.isValid())
		return;

	g_device.setFrequencyMode(mode);
}

void toggleMirror(int)
{
	toggleDepthMirror(0);
	toggleColorMirror(0);
	toggleIRMirror(0);
	togglePhaseMirror(0);

	displayMessage("Mirror: %s", g_depthStream.getMirroringEnabled() ? "On" : "Off");
}


void toggleCloseRange(int)
{
	static OniBool bCloseRange = FALSE;

	if (g_depthStream.getProperty(XN_STREAM_PROPERTY_CLOSE_RANGE, &bCloseRange) != XN_STATUS_OK &&
		g_depthStream.getProperty(KINECT_DEPTH_PROPERTY_CLOSE_RANGE, &bCloseRange) != XN_STATUS_OK)
	{
		// Continue with the latest value even in case of error
	}

	bCloseRange = !bCloseRange;

	if (g_depthStream.setProperty(XN_STREAM_PROPERTY_CLOSE_RANGE, bCloseRange) != XN_STATUS_OK &&
		g_depthStream.setProperty(KINECT_DEPTH_PROPERTY_CLOSE_RANGE, bCloseRange) != XN_STATUS_OK)
	{
		displayError("Couldn't set the close range");
		return;
	}

	displayMessage("Close range: %s", bCloseRange ? "On" : "Off");
}

void toggleEmitterState(int)
{
	static OniBool bEmitterState = TRUE;

	if (g_device.getProperty(XN_MODULE_PROPERTY_EMITTER_STATE, &bEmitterState) != XN_STATUS_OK &&
		g_device.getProperty(KINECT_DEVICE_PROPERTY_EMITTER_STATE, &bEmitterState) != XN_STATUS_OK)
	{
		// Continue with the latest value even in case of error
	}

	bEmitterState = !bEmitterState;

	if (g_device.setProperty(XN_MODULE_PROPERTY_EMITTER_STATE, bEmitterState) != XN_STATUS_OK &&
		g_device.setProperty(KINECT_DEVICE_PROPERTY_EMITTER_STATE, bEmitterState) != XN_STATUS_OK)
	{
		displayError("Couldn't set the emitter state");
		return;
	}

	displayMessage("Emitter state: %s", bEmitterState ? "On" : "Off");
}

void toggleFloodState(int)
{
	static OniBool bFloodState = TRUE;

	if (g_device.getProperty(XN_MODULE_PROPERTY_IRFLOOD_STATE, &bFloodState) != XN_STATUS_OK)
	{
		displayError("Couldn't set the flood state");
		return;
	}

	bFloodState = !bFloodState;
	if (g_device.setProperty(XN_MODULE_PROPERTY_IRFLOOD_STATE, bFloodState) != XN_STATUS_OK)
	{

		displayError("Couldn't set the flood state");
		return;
	}
	displayMessage("flood state: %s", bFloodState ? "On" : "Off");
}

void toggleImageRegistration(int)
{
	openni::ImageRegistrationMode mode = g_device.getImageRegistrationMode();

	openni::ImageRegistrationMode newMode = openni::IMAGE_REGISTRATION_OFF;
	if (mode == openni::IMAGE_REGISTRATION_OFF)
	{
		newMode = openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR;
	}

	if (g_device.isImageRegistrationModeSupported(newMode))
	{
		g_device.setImageRegistrationMode(newMode);
	}
	else
	{
		displayError("Couldn't change image registration to unsupported mode");
	}

}

openni::VideoStream* getSeekingStream(openni::VideoFrameRef*& pCurFrame)
{
	if (g_pPlaybackControl == NULL)
	{
		return NULL;
	}

	if (g_bIsDepthOn)
	{
		pCurFrame = &g_depthFrame;
		return &g_depthStream;
	}
	else if (g_bIsColorOn)
	{
		pCurFrame = &g_colorFrame;
		return &g_colorStream;
	}
	else if (g_bIsIROn)
	{
		pCurFrame = &g_irFrame;
		return &g_irStream;
	}
	else if (g_bIsPhaseOn)
	{
		pCurFrame = &g_phaseFrame;
		return &g_phaseStream;
	}
	/*else if (g_bIsAIOn)
	{
	pCurFrame = &g_AIFrame;
	return &g_AIStream;
	}*/
	else
	{
		return NULL;
	}
}

void seekStream(openni::VideoStream* pStream, openni::VideoFrameRef* pCurFrame, int frameId)
{
	int numberOfFrames = 0;

	// Get number of frames
	numberOfFrames = g_pPlaybackControl->getNumberOfFrames(*pStream);

	// Seek
	openni::Status rc = g_pPlaybackControl->seek(*pStream, frameId);
	if (rc == openni::STATUS_OK)
	{
		// Read next frame from all streams.
		if (g_bIsDepthOn)
		{
			g_depthStream.readFrame(&g_depthFrame);
		}
		if (g_bIsColorOn)
		{
			g_colorStream.readFrame(&g_colorFrame);
		}
		if (g_bIsIROn)
		{
			g_irStream.readFrame(&g_irFrame);
		}
		if (g_bIsPhaseOn)
		{
			g_phaseStream.readFrame(&g_phaseFrame);
		}

		// the new frameId might be different than expected (due to clipping to edges)
		frameId = pCurFrame->isValid() ? pCurFrame->getFrameIndex() : 0;

		displayMessage("Current frame: %u/%u", frameId, numberOfFrames);
	}
	else if ((rc == openni::STATUS_NOT_IMPLEMENTED) || (rc == openni::STATUS_NOT_SUPPORTED) || (rc == openni::STATUS_BAD_PARAMETER) || (rc == openni::STATUS_NO_DEVICE))
	{
		displayError("Seeking is not supported");
	}
	else
	{
		displayError("Error seeking to frame:\n%s", openni::OpenNI::getExtendedError());
	}
}

void seekFrame(int nDiff)
{
	// Make sure seek is required.
	if (nDiff == 0)
	{
		return;
	}

	openni::VideoStream* pStream = NULL;
	openni::VideoFrameRef* pCurFrame = NULL;

	pStream = getSeekingStream(pCurFrame);
	if (pStream == NULL)
		return;

	int frameId = pCurFrame->isValid() ? pCurFrame->getFrameIndex() : 0;
	// Calculate the new frame ID
	frameId = (frameId + nDiff < 1) ? 1 : frameId + nDiff;

	seekStream(pStream, pCurFrame, frameId);
}

void seekFrameAbs(int frameId)
{
	openni::VideoStream* pStream = NULL;
	openni::VideoFrameRef* pCurFrame = NULL;

	pStream = getSeekingStream(pCurFrame);
	if (pStream == NULL)
		return;

	seekStream(pStream, pCurFrame, frameId);
}

void toggleStreamState(openni::VideoStream& stream, openni::VideoFrameRef& frame, bool& isOn, openni::SensorType type, const char* name)
{
	openni::Status nRetVal = openni::STATUS_OK;

	if (!stream.isValid())
	{
		nRetVal = stream.create(g_device, type);
		if (nRetVal != openni::STATUS_OK)
		{
			displayError("Failed to create %s stream:\n%s", name, openni::OpenNI::getExtendedError());
			return;
		}
	}

	if (isOn)
	{
		stream.stop();
		if (openni::SENSOR_AI != type)
			frame.release();
	}
	else
	{
		checkResFmt(stream);
		nRetVal = stream.start();
		if (nRetVal != openni::STATUS_OK)
		{
			displayError("Failed to start %s stream:\n%s", name, openni::OpenNI::getExtendedError());
			return;
		}
	}

	isOn = !isOn;
}

void toggleDepthState(int)
{
	toggleStreamState(g_depthStream, g_depthFrame, g_bIsDepthOn, openni::SENSOR_DEPTH, "Depth");
}

void toggleColorState(int)
{
	toggleStreamState(g_colorStream, g_colorFrame, g_bIsColorOn, openni::SENSOR_COLOR, "Color");
}

void toggleIRState(int)
{
	toggleStreamState(g_irStream, g_irFrame, g_bIsIROn, openni::SENSOR_IR, "IR");
}

void togglePhaseState(int)
{
	toggleStreamState(g_phaseStream, g_phaseFrame, g_bIsPhaseOn, openni::SENSOR_PHASE, "Phase");
}

void toggleAIState(int)
{
	openni::VideoFrameRef frameRef;
	toggleStreamState(g_AIStream, frameRef, g_bIsAIOn, openni::SENSOR_AI, "AI");
}

bool isDepthOn()
{
	return (g_bIsDepthOn);
}

bool isColorOn()
{
	return (g_bIsColorOn);
}

bool isIROn()
{
	return (g_bIsIROn);
}

bool isPhaseOn()
{
	return (g_bIsPhaseOn);
}

bool isAIOn()
{
	return (g_bIsAIOn);
}

const openni::SensorInfo* getDepthSensorInfo()
{
	return g_depthSensorInfo;
}

const openni::SensorInfo* getColorSensorInfo()
{
	return g_colorSensorInfo;
}

const openni::SensorInfo* getIRSensorInfo()
{
	return g_irSensorInfo;
}

const openni::SensorInfo* getPhaseSensorInfo()
{
	return g_phaseSensorInfo;
}

const openni::SensorInfo* getAISensorInfo()
{
	return g_AISensorInfo;
}

void setDepthVideoMode(int mode)
{
	bool bIsStreamOn = g_bIsDepthOn;
	if (bIsStreamOn)
	{
		g_bIsDepthOn = false;
		g_depthStream.stop();
	}

	g_depthStream.setVideoMode(g_depthSensorInfo->getSupportedVideoModes()[mode]);
	if (bIsStreamOn)
	{
		g_depthStream.start();
		g_bIsDepthOn = true;
	}
}

void setColorVideoMode(int mode)
{
	bool bIsStreamOn = g_bIsColorOn;
	if (bIsStreamOn)
	{
		g_bIsColorOn = false;
		g_colorStream.stop();
	}

	g_colorFrame.release();
	g_colorStream.setVideoMode(g_colorSensorInfo->getSupportedVideoModes()[mode]);
	if (bIsStreamOn)
	{
		g_colorStream.start();
		g_bIsColorOn = true;
	}
}

void setIRVideoMode(int mode)
{
	bool bIsStreamOn = g_bIsIROn;
	if (bIsStreamOn)
	{
		g_bIsIROn = false;
		g_irStream.stop();
	}

	g_irFrame.release();
	g_irStream.setVideoMode(g_irSensorInfo->getSupportedVideoModes()[mode]);
	if (bIsStreamOn)
	{
		g_irStream.start();
		g_bIsIROn = true;
	}
}

void setPhaseVideoMode(int mode)
{
	bool bIsStreamOn = g_bIsPhaseOn;
	if (bIsStreamOn)
	{
		g_bIsPhaseOn = false;
		g_phaseStream.stop();
	}

	g_phaseFrame.release();
	g_phaseStream.setVideoMode(g_phaseSensorInfo->getSupportedVideoModes()[mode]);
	if (bIsStreamOn)
	{
		g_phaseStream.start();
		g_bIsPhaseOn = true;
	}
}

void setAIVideoMode(int mode)
{
	bool bIsStreamOn = g_bIsAIOn;
	if (bIsStreamOn)
	{
		g_bIsAIOn = false;
		g_AIStream.stop();
	}

	g_AIFrame.release();
	g_AIStream.setVideoMode(g_AISensorInfo->getSupportedVideoModes()[mode]);
	if (bIsStreamOn)
	{
		g_AIStream.start();
		g_bIsAIOn = true;
	}
}

void toggleDepthMirror(int)
{
	g_depthStream.setMirroringEnabled(!g_depthStream.getMirroringEnabled());
}

void toggleColorMirror(int)
{
	g_colorStream.setMirroringEnabled(!g_colorStream.getMirroringEnabled());
}

void toggleIRMirror(int)
{
	g_irStream.setMirroringEnabled(!g_irStream.getMirroringEnabled());
}

void togglePhaseMirror(int)
{
	g_phaseStream.setMirroringEnabled(!g_phaseStream.getMirroringEnabled());
}

void toggleImageAutoExposure(int)
{
	if (g_colorStream.getCameraSettings() == NULL)
	{
		displayError("Color stream doesn't support camera settings");
		return;
	}
	g_colorStream.getCameraSettings()->setAutoExposureEnabled(!g_colorStream.getCameraSettings()->getAutoExposureEnabled());
	displayMessage("Auto Exposure: %s", g_colorStream.getCameraSettings()->getAutoExposureEnabled() ? "ON" : "OFF");
}

void toggleImageAutoWhiteBalance(int)
{
	if (g_colorStream.getCameraSettings() == NULL)
	{
		displayError("Color stream doesn't support camera settings");
		return;
	}
	g_colorStream.getCameraSettings()->setAutoWhiteBalanceEnabled(!g_colorStream.getCameraSettings()->getAutoWhiteBalanceEnabled());
	displayMessage("Auto White balance: %s", g_colorStream.getCameraSettings()->getAutoWhiteBalanceEnabled() ? "ON" : "OFF");
}

void changeImageExposure(int delta)
{
	if (g_colorStream.getCameraSettings() == NULL)
	{
		displayError("Color stream doesn't support camera settings");
		return;
	}
	int exposure = g_colorStream.getCameraSettings()->getExposure();
	openni::Status rc = g_colorStream.getCameraSettings()->setExposure(exposure + delta);
	if (rc != openni::STATUS_OK)
	{
		displayMessage("Can't change exposure");
		return;
	}
	displayMessage("Changed exposure to: %d", g_colorStream.getCameraSettings()->getExposure());
}

void changeIrExposure(int delta)
{
	int size = 4;
	int exposure = 0;
	openni::Status  rc = g_device.getProperty(openni::OBEXTENSION_ID_IR_EXP, (uint8_t *)&exposure, &size);

	if (rc != openni::STATUS_OK)
	{
		displayMessage("get ir exposure value failed");
		return;
	}

	exposure += delta;

	if (exposure < 0)
	{
		displayMessage("Can't change ir exposure,because exposure<0");
		return;
	}

	rc = g_device.setProperty(openni::OBEXTENSION_ID_IR_EXP, (uint8_t *)&exposure, size);

	if (rc != openni::STATUS_OK)
	{
		displayMessage("Can't change ir exposure");
		return;
	}

	displayMessage("Changed ir exposure to: %d", exposure);
}

void changeImageGain(int delta)
{
	if (g_colorStream.getCameraSettings() == NULL)
	{
		displayError("Color stream doesn't support camera settings");
		return;
	}
	int gain = g_colorStream.getCameraSettings()->getGain();
	openni::Status rc = g_colorStream.getCameraSettings()->setGain(gain + delta);
	if (rc != openni::STATUS_OK)
	{
		displayMessage("Can't change gain");
		return;
	}
	displayMessage("Changed gain to: %d", g_colorStream.getCameraSettings()->getGain());
}

void toggleDepthAutoExposure(int)
{
	XnUInt32 val = 0;
	g_device.getProperty(XN_MODULE_PROPERTY_AE, &val);
	g_device.setProperty(XN_MODULE_PROPERTY_AE, !val);
	g_device.getProperty(XN_MODULE_PROPERTY_AE, &val);
	displayMessage("Depth Auto Exposure: %s", val ? "ON" : "OFF");
}

void setStreamCropping(openni::VideoStream& stream, int originX, int originY, int width, int height)
{
	if (!stream.isValid())
	{
		displayMessage("Stream does not exist!");
		return;
	}

	if (!stream.isCroppingSupported())
	{
		displayMessage("Stream does not support cropping!");
		return;
	}

	openni::Status nRetVal = stream.setCropping(originX, originY, width, height);
	if (nRetVal != openni::STATUS_OK)
	{
		displayMessage("Failed to set cropping: %s", xnGetStatusString(nRetVal));
		return;
	}
}


void resetStreamCropping(openni::VideoStream& stream)
{
	if (!stream.isValid())
	{
		displayMessage("Stream does not exist!");
		return;
	}

	if (!stream.isCroppingSupported())
	{
		displayMessage("Stream does not support cropping!");
		return;
	}

	openni::Status nRetVal = stream.resetCropping();
	if (nRetVal != openni::STATUS_OK)
	{
		displayMessage("Failed to reset cropping: %s", xnGetStatusString(nRetVal));
		return;
	}
}

void resetDepthCropping(int)
{
	getDepthStream().resetCropping();
}

void resetColorCropping(int)
{
	getColorStream().resetCropping();
}

void resetIRCropping(int)
{
	getIRStream().resetCropping();
}

void resetPhaseCropping(int)
{
	getPhaseStream().resetCropping();
}

void resetAllCropping(int)
{
	if (getDepthStream().isValid())
		resetDepthCropping(0);

	if (getColorStream().isValid())
		resetColorCropping(0);

	if (getIRStream().isValid())
		resetIRCropping(0);
}

void switchIRCamera(int iNum)
{
	uint8_t num = (uint8_t)iNum;
	getDevice().setProperty(XN_MODULE_PROPERTY_SWITCH_IR, num);
}

void togglePlaybackRepeat(int /*ignored*/)
{
	if (g_pPlaybackControl == NULL)
	{
		return;
	}

	bool bLoop = g_pPlaybackControl->getRepeatEnabled();
	bLoop = !bLoop;
	g_pPlaybackControl->setRepeatEnabled(bLoop);
	char msg[100];
	sprintf(msg, "Repeat playback: %s", (bLoop ? "ON" : "OFF"));
	displayMessage(msg);
}

openni::Status setPlaybackSpeed(float speed)
{
	if (g_pPlaybackControl == NULL)
	{
		return openni::STATUS_NOT_SUPPORTED;
	}
	return g_pPlaybackControl->setSpeed(speed);
}

float getPlaybackSpeed()
{
	if (g_pPlaybackControl == NULL)
	{
		return 0.0f;
	}
	return g_pPlaybackControl->getSpeed();
}

void changePlaybackSpeed(int ratioDiff)
{
	float ratio = (float)pow(2.0, ratioDiff);
	float speed = getPlaybackSpeed() * ratio;
	if (speed < 0)
	{
		speed = 0;
	}
	openni::Status rc = setPlaybackSpeed(speed);
	if (rc == openni::STATUS_OK)
	{
		if (speed == 0)
		{
			displayMessage("Playback speed set to fastest");
		}
		else
		{
			displayMessage("Playback speed set to x%.2f", speed);
		}
	}
	else if ((rc == openni::STATUS_NOT_IMPLEMENTED) || (rc == openni::STATUS_NOT_SUPPORTED) || (rc == openni::STATUS_BAD_PARAMETER))
	{
		displayError("Playback speed is not supported");
	}
	else
	{
		displayError("Error setting playback speed:\n%s", openni::OpenNI::getExtendedError());
	}
}

openni::Device& getDevice()
{
	return g_device;
}

openni::VideoStream& getDepthStream()
{
	return g_depthStream;
}
openni::VideoStream& getColorStream()
{
	return g_colorStream;
}
openni::VideoStream& getIRStream()
{
	return g_irStream;
}
openni::VideoStream& getPhaseStream()
{
	return g_phaseStream;
}
openni::VideoStream& getAIStream()
{
	return g_AIStream;
}

openni::VideoFrameRef& getDepthFrame()
{
	return g_depthFrame;
}
openni::VideoFrameRef& getColorFrame()
{
	return g_colorFrame;
}
openni::VideoFrameRef& getIRFrame()
{
	return g_irFrame;
}
openni::VideoFrameRef& getPhaseFrame()
{
	return g_phaseFrame;
}
openni::AIFrameRef& getAIFrame()
{
	return g_AIFrame;
}

bool g_bFrameSyncOn = false;
void toggleFrameSync(int)
{
	if (g_bFrameSyncOn)
	{
		g_device.setDepthColorSyncEnabled(false);
		displayMessage("Frame sync off");
	}
	else
	{
		openni::Status rc = g_device.setDepthColorSyncEnabled(true);
		if (rc != openni::STATUS_OK)
		{
			displayMessage("Can't frame sync");
			return;
		}
		displayMessage("Frame sync on");
	}
	g_bFrameSyncOn = !g_bFrameSyncOn;
}

bool g_bZoomCropOn = false;
void toggleZoomCrop(int)
{
	if (g_bZoomCropOn)
	{
		displayMessage("Fast zoom crop off");

		openni::VideoMode vm = g_colorStream.getVideoMode();
		vm.setResolution(ZOOM_CROP_VGA_MODE_X_RES, ZOOM_CROP_VGA_MODE_Y_RES);

		g_colorStream.setVideoMode(vm);

		g_colorStream.resetCropping();

		g_colorStream.start();

		g_colorStream.setProperty(XN_STREAM_PROPERTY_FAST_ZOOM_CROP, FALSE);
	}
	else
	{
		g_colorStream.setProperty(XN_STREAM_PROPERTY_FAST_ZOOM_CROP, TRUE);

		displayMessage("Fast zoom crop on");

		g_colorStream.stop();

		g_colorStream.setProperty(XN_STREAM_PROPERTY_CROPPING_MODE, XN_CROPPING_MODE_INCREASED_FPS);

		openni::VideoMode vm = g_colorStream.getVideoMode();
		vm.setResolution(ZOOM_CROP_HIRES_MODE_X_RES, ZOOM_CROP_HIRES_MODE_Y_RES);
		g_colorStream.setVideoMode(vm);

		g_colorStream.setCropping(ZOOM_CROP_VGA_MODE_X_RES / 2, ZOOM_CROP_VGA_MODE_Y_RES / 2, ZOOM_CROP_VGA_MODE_X_RES, ZOOM_CROP_VGA_MODE_Y_RES);

		g_colorStream.start();
	}

	g_bZoomCropOn = !g_bZoomCropOn;
}

bool convertDepthPointToColor(int depthX, int depthY, openni::DepthPixel depthZ, int* pColorX, int* pColorY)
{
	if (!g_depthStream.isValid() || !g_colorStream.isValid())
		return false;

	return (openni::STATUS_OK == openni::CoordinateConverter::convertDepthToColor(g_depthStream, g_colorStream, depthX, depthY, depthZ, pColorX, pColorY));
}
