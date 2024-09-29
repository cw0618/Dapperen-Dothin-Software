/******************************************************************************
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
package org.openni;

import java.util.ArrayList;
import java.util.List;
import java.lang.RuntimeException;
import java.nio.ByteBuffer;

class NativeMethods {

    static private final int STATUS_OK = 0;
    static private final int STATUS_ERROR = 1;
    static private final int STATUS_NOT_IMPLEMENTED = 2;
    static private final int STATUS_NOT_SUPPORTED = 3;
    static private final int STATUS_BAD_PARAMETER = 4;
    static private final int STATUS_OUT_OF_FLOW = 5;
    static private final int STATUS_NO_DEVICE = 6;
    static private final int STATUS_TIME_OUT = 102;

    static public final int STREAM_PROPERTY_CROPPING = 0; // OniCropping*
    static public final int STREAM_PROPERTY_HORIZONTAL_FOV = 1; // float: radians
    static public final int STREAM_PROPERTY_VERTICAL_FOV = 2; // float: radians
    static public final int STREAM_PROPERTY_MAX_VALUE = 4; // int
    static public final int STREAM_PROPERTY_MIN_VALUE = 5; // int
    static public final int STREAM_PROPERTY_STRIDE = 6; // int
    static public final int STREAM_PROPERTY_MIRRORING = 7; // OniBool
    static public final int STREAM_PROPERTY_NUMBER_OF_FRAMES = 8; // int
    static public final int STREAM_PROPERTY_AUTO_EXPOSURE = 101;
    static public final int STREAM_PROPERTY_AUTO_WHITE_BALANCE = 100;
    static public final int STREAM_PROPERTY_EXPOSURE = 102;
    static public final int STREAM_PROPERTY_GAIN = 103;

    static public final int DEVICE_PROPERTY_RT_IR_TEMP = 0x1080FFB4;//IR real time Temperature mode
    static public final int DEVICE_PROPERTY_RT_LDMP_TEMP = 0x1080FFB5;//LDMP real time Temperature mode
    static public final int STREAM_PROPERTY_SOFTWARE_REGISTRATION = 0x2080FF42;
    static public final int DEVICE_PROPERTY_SENSOR_PLATFORM_STRING = 0x1080FF7C; // "SensorPlatformString"
    static public final int DEVICE_PROPERTY_SERIALNUMBER = 16;
    static public final int DEVICE_PROPERTY_SOFTWARE_FILTER = 0x2080FF43;        //soft filter enable

    static public final int DEVICE_PROPERTY_FIRMWARE_VERSION = 0; // By implementation
    static public final int DEVICE_PROPERTY_DRIVER_VERSION = 1; // OniVersion
    static public final int DEVICE_PROPERTY_HARDWARE_VERSION = 2; // int
    static public final int DEVICE_PROPERTY_SERIAL_NUMBER = 3; // string
    static public final int DEVICE_PROPERTY_ERROR_STATE = 4; // ??
    static public final int DEVICE_PROPERTY_IMAGE_REGISTRATION = 5; // OniImageRegistrationMode
    static public final int DEVICE_PROPERTY_FREQUENCY_MODE = 6;

    // add for Attribute value by LiuDongBing start

    static public final int DEVICE_PROPERTY_GAIN = 11;
    static public final int OBEXTENSION_ID_IR_EXP = 12;
    static public final int OBEXTENSION_ID_LDP_EN = 13;
    static public final int OBEXTENSION_ID_CAM_PARAMS = 14;
    static public final int OBEXTENSION_ID_LASER_EN = 15;
    static public final int OBEXTENSION_ID_SERIALNUMBER = 16;
    static public final int OBEXTENSION_ID_DEVICETYPE = 17;
    static public final int OBEXTENSION_ID_UPDATE_FIRMWARE = 18;

    static public final int XN_MODULE_PROPERTY_LDP_ENABLE = 0x1080FFBE;
    /**
     * get Emitter enable
     */
    static public final int XN_MODULE_PROPERTY_EMITTER_STATE_V1 = 0x1080FFBF;
    /**
     * EmitterState
     */
    static public final int XN_MODULE_PROPERTY_EMITTER_STATE = 0x1080E007;

    /**
     * IrFlood
     */
    static public final int XN_MODULE_PROPERTY_IRFLOOD_STATE = 0x1080FF94;

    /**
     * MinDepthValue
     */
    static public final int XN_STREAM_PROPERTY_MIN_DEPTH = 0x1080FF40;
    /**
     * MaxDepthValue
     */
    static public final int XN_STREAM_PROPERTY_MAX_DEPTH = 0x1080FF41;

    /**
     * InputFormat
     */
    static public final int XN_STREAM_PROPERTY_INPUT_FORMAT = 0x10800001;
    /**
     * atlas depth rotate
     */
    static public final int XN_STREAM_PROPERTY_DEPTH_ROTATE = 0x2080FF44;
    /**
     * "HoleFilter"
     */
    static public final int XN_STREAM_PROPERTY_HOLE_FILTER = 0x10801004;

    /**
     * soft filter enable
     */
    static public final int XN_STREAM_PROPERTY_SOFTWARE_FILTER = 0x2080FF43;

    // add for Attribute value by LiuDongBing End
    static public final int XN_MODULE_PROPERTY_DEPTH_IR_MODE = 0x1080FFF0;

    // Files
    static public final int DEVICE_PROPERTY_PLAYBACK_SPEED = 100; // float
    static public final int DEVICE_PROPERTY_PLAYBACK_REPEAT_ENABLED = 101; // OniBool

    static public final int DEVICE_COMMAND_SEEK = 1; // OniSeek

    static {
        System.loadLibrary("OpenNI2.jni");
    }

    static void checkReturnStatus(int status) {
        switch (status) {
            case STATUS_OK:
                break;

            case STATUS_ERROR:
                throw new RuntimeException(oniGetExtendedError());

            case STATUS_NOT_IMPLEMENTED:
                throw new UnsupportedOperationException(oniGetExtendedError());

            case STATUS_NOT_SUPPORTED:
                throw new UnsupportedOperationException(oniGetExtendedError());

            case STATUS_BAD_PARAMETER:
                throw new IllegalArgumentException(oniGetExtendedError());

            case STATUS_OUT_OF_FLOW:
                throw new IllegalStateException(oniGetExtendedError());

            default:
                throw new RuntimeException(oniGetExtendedError());
        }
    }

    static native void oniFrameRelease(long frame);

    static native void oniFrameAddRef(long frame);

    static native int oniDeviceCreateStream(long deviceHandle, int sensorType, VideoStream stream);

    static native void oniStreamDestroy(long streamHandle, long callbackHandle);

    static native int oniStreamStart(long streamHandle);

    static native void oniStreamStop(long streamHandle);

    static native int oniStreamReadFrame(long streamHandle, OutArg<VideoFrameRef> videoFrame);

    static native int oniStreamReadFrameEx(long streamHandle, OutArg<AIFrameRef> aiFrame);

    static native int getCropping(long streamHandle, OutArg<Integer> originX,
                                  OutArg<Integer> originY, OutArg<Integer> width, OutArg<Integer> height);

    static native int setCropping(long streamHandle, int originX, int originY, int width, int height);

    static native boolean isCroppingSupported(long streamHandle);

    static native int resetCropping(long streamHandle);

    static native int getVideoMode(long streamHandle, OutArg<VideoMode> videoMode);

    static native int setVideoMode(long streamHandle, int resX, int resY, int fps, int pixelFormat);

    static native SensorInfo oniStreamGetSensorInfo(long streamHandle);

    static native boolean hasSensor(long deviceHandle, int sensorType);

    static native int oniStreamGetIntProperty(long streamHandle, int propertyId,
                                              OutArg<Integer> property);

    static native int oniStreamGetBoolProperty(long streamHandle, int propertyId,
                                               OutArg<Boolean> property);

    static native int oniStreamGetFloatProperty(long streamHandle, int propertyId,
                                                OutArg<Float> property);

    static native int oniStreamSetProperty(long streamHandle, int propertyId, int property);

    static native int oniStreamSetProperty(long streamHandle, int propertyId, boolean property);

    static native int oniStreamSetProperty(long streamHandle, int propertyId, float property);

    static native boolean oniStreamIsPropertySupported(long streamHandle, int propertyId);

    static native void oniStreamfilter(long streamHandle, ByteBuffer buf, int newVal,
                                       int maxSpeckleSize, int maxDiff);

    static native SensorInfo oniDeviceGetSensorInfo(long deviceHandle, int sensorType);

    static native int oniDeviceEnableDepthColorSync(long deviceHandle);

    static native void oniDeviceDisableDepthColorSync(long deviceHandle);

    static native boolean oniDeviceGetDepthColorSyncEnabled(long deviceHandle);

    static native int oniDeviceEnableDepthOptimization(long deviceHandle, boolean enable);

    static native int oniDeviceGetDepthOptimizationEnabled(long deviceHandle, OutArg<Boolean> property);

    static native int oniDeviceGetMultiDistanceCalibrationEnabled(long deviceHandle, OutArg<Boolean> property);

    static native int oniDeviceEnableMultiDistanceCalibration(long deviceHandle, boolean enable);

    static native int oniDeviceEnableLaser(long deviceHandle, boolean enable);

    static native int seek(long deviceHandle, long streamHandle, int frameIndex);

    static native boolean isImageRegistrationModeSupported(long deviceHandle, int mode);

    static native int getImageRegistrationMode(long deviceHandle, OutArg<Integer> property);

    static native int setImageRegistrationMode(long deviceHandle, int regMode);

    static native DeviceInfo oniDeviceGetInfo(long deviceHandle);

    static native int oniRecorderStart(long recorderHandle);

    static native int oniRecorderDestroy(long recorderHandle);

    static native void oniRecorderStop(long recorderHandle);

    static native int oniRecorderAttachStream(long recorderHandle, long streamHandle,
                                              boolean allowLossyCompression);

    static native int oniDeviceGetIntProperty(long deviceHandle, int propertyId,
                                              OutArg<Integer> property);

    static native int oniDeviceGetBoolProperty(long deviceHandle, int propertyId,
                                               OutArg<Boolean> property);

    static native int oniDeviceGetFloatProperty(long deviceHandle, int propertyId,
                                                OutArg<Float> property);

    static native int oniDeviceGetDoubleProperty(long deviceHandle, int propertyId,
                                                 OutArg<Double> property);

    static native int oniDeviceSetProperty(long deviceHandle, int propertyId, int property);

    static native int oniDeviceSetProperty(long deviceHandle, int propertyId, boolean property);

    static native int oniDeviceSetProperty(long deviceHandle, int propertyId, float property);

    static native boolean oniDeviceIsPropertySupported(long deviceHandle, int propertyId);

    static native boolean oniDeviceIsCommandSupported(long deviceHandle, int commandId);

    static native int oniInitialize();

    static native void oniShutdown();

    static native Version oniGetVersion();

    static native String oniGetExtendedError();

    static native int oniGetDeviceList(List<DeviceInfo> devices);

    static native boolean oniWaitForAnyStream(long[] streams, OutArg<Integer> readyStream, int timeout);

    static native int oniCoordinateConverterWorldToDepth(long streamHandle, float worldX,
                                                         float worldY, float worldZ, OutArg<Float> depthX, OutArg<Float> depthY, OutArg<Float> depthZ);

    static native int oniCoordinateConverterDepthToWorld(long streamHandle, float worldX,
                                                         float worldY, float worldZ, OutArg<Float> depthX, OutArg<Float> depthY, OutArg<Float> depthZ);

    static native int oniCoordinateConverterDepthToColor(long depthStream, long colorStream,
                                                         int depthX, int depthY, short depthZ, OutArg<Integer> ColorX, OutArg<Integer> ColorY);

    static native int oniCoordinateConverterD2C(long depthStream,
                                                int depthX, int depthY, short depthZ, OutArg<Integer> ColorX, OutArg<Integer> ColorY);

    static native int oniCoordinateConverterC2D(long depthStream,
                                                int colorX, int colorY, short depthZ, OutArg<Integer> DepthX, OutArg<Integer> DepthY);

    static native int oniCreateRecorder(String fileName, Recorder recorder);

    static native int oniDeviceOpen(String uri, Device device);

    static native int oniDeviceOpen(Device device);

    static native int oniDeviecSwitchIRCamera(long deviceHandle, int irType);

    static native int oniDeviceGetCameraParams(long deviceHandle, OBCameraParams softD2C);

    static native int oniDeviceGetSerialNumber(long deviceHandle, int propertyId, OutArg<String> property);

    static native int oniGetDeviceUSBSpeed(long deviceHandle, OutArg<Integer> property);

    static native int oniDeviceGetStringProperty(long deviceHandle, int propertyId, OutArg<String> property);

    static native int oniDeviceClose(long deviceHandle);

    static native int oniSetLogOutputFolder(String outputFolder);

    static native String oniGetLogFileName();

    static native int oniSetLogMinSeverity(int nMinSeverity);

    static native int oniSetLogConsoleOutput(boolean bConsoleOutput);

    static native int oniSetLogFileOutput(boolean bConsoleOutput);

    static native int oniSetLogAndroidOutput(boolean bConsoleOutput);

    static native int oniSetLogAndroidOutputRedirect(boolean bConsoleOutput);
}
