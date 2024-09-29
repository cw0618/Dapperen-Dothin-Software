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

/**
 * The Device object abstracts a specific device; either a single hardware device, or a file device
 * holding a recording from a hardware device. It offers the ability to connect to the device, and
 * obtain information about its configuration and the data streams it can offer.
 * <p>
 * It provides the means to query and change all configuration parameters that apply to the device
 * as a whole. This includes enabling depth/color image registration and frame synchronization.
 * <p>
 * Devices are used when creating and initializing {@link VideoStream} "VideoStreams" -- you will
 * need a a Device in order to use the VideoStream.create() function. This, along with
 * configuration, is the primary use of this class for application developers.
 * <p>
 * Before devices can be created, {@link org.openni.OpenNI#initialize()} must have been run to make
 * the device drivers on the system available to the API.
 **/
public class Device {
    /**
     * Opens a device. This can either open a device chosen arbitrarily from all devices on the
     * system, or open a specific device selected by passing this function the device URI.
     * <p>
     * To open any device, simply {@link Device#open()} function. If multiple devices
     * are connected to the system, then one of them will be opened. This procedure is most useful
     * when it is known that exactly one device is (or can be) connected to the system. In that case,
     * requesting a list of all devices and iterating through it would be a waste of effort.
     * <p>
     * If multiple devices are (or may be) connected to a system, then a URI will be required to
     * select a specific device to open. There are two ways to obtain a URI: from a DeviceConnected
     * event, or by calling {@link org.openni.OpenNI#enumerateDevices()}.
     * <p>
     * In the case of a DeviceConnected event, the {@link OpenNI.DeviceConnectedListener} will be
     * provided with a DeviceInfo object as an argument to its {}
     * {@link OpenNI.DeviceConnectedListener#onDeviceConnected(DeviceInfo)} function. The
     * {@link org.openni.DeviceInfo#getUri()} function can then be used to obtain the URI.
     * <p>
     * If the application is not using event handlers, then it can also call the static function
     * {@link org.openni.OpenNI#enumerateDevices()}. This will return an array of {@link DeviceInfo}
     * objects, one for each device currently available to the system. The application can then
     * iterate through this list and select the desired device. The URI is again obtained via the
     * {@link org.openni.DeviceInfo#getUri()} function.
     * <p>
     * Standard codes of type Status are returned indicating whether opening was successful. For
     * opening a recording file, pass the file path as a uri.
     *
     * @param uri String containing the URI of the device to be opened.
     */
    public static Device open(String uri) {
        Device device = new Device();
        NativeMethods.checkReturnStatus(NativeMethods.oniDeviceOpen(uri, device));
        if (device.isFile()) {
            device.mPlaybackControl = new PlaybackControl(device);
        }

        return device;
    }

    /**
     * Opens a any device. This can either open a any device. If multiple devices
     * are connected to the system, then one of them will be opened. This procedure is most useful
     * when it is known that exactly one device is (or can be) connected to the system. In that case,
     * requesting a list of all devices and iterating through it would be a waste of effort.
     */
    public static Device open() {
        Device device = new Device();
        NativeMethods.checkReturnStatus(NativeMethods.oniDeviceOpen(device));
        if (device.isFile()) {
            device.mPlaybackControl = new PlaybackControl(device);
        }

        return device;
    }

    /**
     * Switching left and right IR camera.
     *
     * @param type {@link org.openni.IRCameraType}
     */
    public void switchIRCamera(IRCameraType type) {
        NativeMethods.checkReturnStatus(NativeMethods.oniDeviecSwitchIRCamera(getHandle(), type.toNative()));
    }

    /**
     * Closes the device. This properly closes any files or shuts down hardware, as appropriate. This
     * function is currently called by the destructor if not called manually by application code, but
     * it is considered a best practice to manually close any device that was opened.
     */
    public void close() {
        NativeMethods.checkReturnStatus(NativeMethods.oniDeviceClose(getHandle()));
        mDeviceHandle = 0;
        mPlaybackControl = null;
    }

    /**
     * Provides information about this device in the form of a DeviceInfo object. This object can be
     * used to access the URI of the device, as well as various USB descriptor strings that might be
     * useful to an application.
     * <p>
     * Note that valid device info will not be available if this device has not yet been opened. If
     * you are trying to obtain a URI to open a device, use OpenNI::enumerateDevices() instead.
     *
     * @return DeviceInfo object for this Device
     */
    public final DeviceInfo getDeviceInfo() {
        return NativeMethods.oniDeviceGetInfo(getHandle());
    }

    /**
     * This function checks to see if one of the specific sensor types defined in {@link SensorType}
     * is available on this device. This allows an application to, for example, query for the presence
     * of a depth sensor, or color sensor.
     *
     * @param sensorType of sensor to query for
     * @return true if the Device supports the sensor queried, false otherwise.
     */
    public boolean hasSensor(SensorType sensorType) {
        return NativeMethods.hasSensor(getHandle(), sensorType.toNative());
    }

    /**
     * Get the {@link SensorInfo} for a specific sensor type on this device. The {@link SensorInfo} is
     * useful primarily for determining which video modes are supported by the sensor.
     *
     * @param sensorType of sensor to get information about.
     * @return SensorInfo object corresponding to the sensor type specified, or NULL if such a sensor
     * is not available from this device.
     */
    public final SensorInfo getSensorInfo(SensorType sensorType) {
        return NativeMethods.oniDeviceGetSensorInfo(getHandle(), sensorType.toNative());
    }

    /**
     * This function return device handle.
     *
     * @return OpenNI device handle.
     */
    public long getHandle() {
        return mDeviceHandle;
    }

    /**
     * Gets an object through which playback of a file device can be controlled.
     *
     * @return null if this device is not a file device.
     */
    public PlaybackControl getPlaybackControl() {
        return mPlaybackControl;
    }

    /**
     * Checks to see if this device can support registration of color video and depth video. Image
     * registration is used to properly superimpose two images from cameras located at different
     * points in space. Please see the OpenNi 2.0 Programmer's Guide for more information about
     * registration.
     *
     * @return true if image registration is supported by this device, false otherwise.
     */
    public boolean isImageRegistrationModeSupported(ImageRegistrationMode mode) {
        return NativeMethods.isImageRegistrationModeSupported(getHandle(), mode.toNative());
    }

    /**
     * Gets the current image registration mode of this device. Image registration is used to properly
     * superimpose two images from cameras located at different points in space. Please see the OpenNi
     * 2.0 Programmer's Guide for more information about registration.
     *
     * @return Current image registration mode. See {@link ImageRegistrationMode} for possible return
     * values.
     */
    public ImageRegistrationMode getImageRegistrationMode() {
        OutArg<Integer> value = new OutArg<Integer>();
        int rc = NativeMethods.getImageRegistrationMode(getHandle(), value);
        if (rc != 0) {
            return ImageRegistrationMode.OFF;
        }
        return ImageRegistrationMode.fromNative(value.mValue);
    }

    /**
     * Sets the image registration on this device. Image registration is used to properly superimpose
     * two images from cameras located at different points in space. Please see the OpenNi 2.0
     * Programmer's Guide for more information about registration.
     * <p>
     * See {@link ImageRegistrationMode} for a list of valid settings to pass to this function.
     * <p>
     * It is a good practice to first check if the mode is supported by calling
     * {@link #isImageRegistrationModeSupported(ImageRegistrationMode)} .
     *
     * @param mode Desired new value for the image registration mode.
     */
    public void setImageRegistrationMode(ImageRegistrationMode mode) {
        NativeMethods.checkReturnStatus(NativeMethods.setImageRegistrationMode(getHandle(), mode.toNative()));
    }

    /**
     * Gets frequency mode for corresponding sensor.
     *
     * @return Current frequency mode. See {@link FrequencyMode} for possible return.
     */
    public FrequencyMode getFrequencyMode() {
        OutArg<Integer> value = new OutArg<Integer>();
        if (0 != NativeMethods.oniDeviceGetIntProperty(getHandle(), NativeMethods.DEVICE_PROPERTY_FREQUENCY_MODE, value)) {
            return null;
        }

        return FrequencyMode.fromNative(value.mValue.intValue());
    }

    /**
     * Sets frequency mode for corresponding sensor.
     *
     * @param mode Desired new value for the frequency mode.
     */
    public void setFrequencyMode(FrequencyMode mode) {
        NativeMethods.oniDeviceSetProperty(getHandle(), NativeMethods.DEVICE_PROPERTY_FREQUENCY_MODE, mode.toNative());
    }

    /**
     * Checks whether this device is a file device (i.e. a recording).
     *
     * @return true if this is a file device, false otherwise.
     */
    public boolean isFile() {
        return NativeMethods.oniDeviceIsPropertySupported(getHandle(), NativeMethods.DEVICE_PROPERTY_PLAYBACK_SPEED) &&
                NativeMethods.oniDeviceIsPropertySupported(getHandle(), NativeMethods.DEVICE_PROPERTY_PLAYBACK_REPEAT_ENABLED) &&
                NativeMethods.oniDeviceIsCommandSupported(getHandle(), NativeMethods.DEVICE_COMMAND_SEEK);
    }

    /**
     * Used to turn the depth/color frame synchronization feature on and off. When frame
     * synchronization is enabled, the device will deliver depth and image frames that are separated
     * in time by some maximum value. When disabled, the phase difference between depth and image
     * frame generation cannot be guaranteed.
     *
     * @param isEnabled Set to TRUE to enable synchronization, FALSE to disable it
     */
    public void setDepthColorSyncEnabled(boolean isEnabled) {
        if (isEnabled) {
            NativeMethods.checkReturnStatus(NativeMethods.oniDeviceEnableDepthColorSync(getHandle()));
        } else {
            NativeMethods.oniDeviceDisableDepthColorSync(getHandle());
        }
    }

    public boolean getDepthColorSyncEnabled() {
        return NativeMethods.oniDeviceGetDepthColorSyncEnabled(getHandle());
    }

    /**
     * Used to turn the depth Optimization on and off.
     *
     * @param enable
     */
    public void setDepthOptimizationEnable(boolean enable) {
        NativeMethods.oniDeviceEnableDepthOptimization(getHandle(), enable);
    }

    /**
     * Used to get Optimization switch state.
     *
     * @return
     */
    public boolean getDepthOptimizationEnabled() {
        OutArg<Boolean> val = new OutArg<Boolean>();
        NativeMethods.oniDeviceGetDepthOptimizationEnabled(getHandle(), val);
        return val.mValue;
    }

    /**
     * Used to get Multi distance calibration switch state enable.
     *
     * @return
     */
    public boolean getMultiDistanceCalibrationEnabled() {
        OutArg<Boolean> val = new OutArg<Boolean>();
        NativeMethods.oniDeviceGetMultiDistanceCalibrationEnabled(getHandle(), val);
        return val.mValue;
    }

    /**
     * Set multi distance calibration switch state to enable.
     *
     * @param enable
     */
    public void setMultiDistanceCalibrationEnable(boolean enable) {
        NativeMethods.oniDeviceEnableMultiDistanceCalibration(getHandle(), enable);
    }

    /**
     * Used to turn the device laser on and off {@link #setEmitterEnable(int)}
     *
     * @param enable
     */
    public void setLaserEnable(boolean enable) {
        NativeMethods.oniDeviceEnableLaser(getHandle(), enable);
    }

    public void setGain(int gain) {
        NativeMethods.checkReturnStatus(NativeMethods.oniDeviceSetProperty(getHandle(), NativeMethods.DEVICE_PROPERTY_GAIN, gain));
    }

    public int getGain() {
        OutArg<Integer> val = new OutArg<Integer>();
        NativeMethods.checkReturnStatus(NativeMethods.oniDeviceGetIntProperty(getHandle(), NativeMethods.DEVICE_PROPERTY_GAIN, val));
        return val.mValue;
    }

    /* add for Orbbec extend API by LiuDongBing Start; */

    /**
     * 获取设备的类型
     *
     * @return
     */
    public String getDeviceType() {
        OutArg<String> val = new OutArg<String>();
        NativeMethods.checkReturnStatus(NativeMethods.oniDeviceGetStringProperty(getHandle(), NativeMethods.OBEXTENSION_ID_DEVICETYPE, val));
        return val.mValue;
    }

    /**
     * 获取 IR 曝光
     *
     * @return
     */
    public int getIrExp() {
        OutArg<Integer> val = new OutArg<Integer>();
        NativeMethods.checkReturnStatus(NativeMethods.oniDeviceGetIntProperty(getHandle(), NativeMethods.OBEXTENSION_ID_IR_EXP, val));
        return val.mValue;
    }

    /**
     * 设置 IR 曝光
     *
     * @param irExp
     */
    public void setIrExp(int irExp) {
        NativeMethods.checkReturnStatus(NativeMethods.oniDeviceSetProperty(getHandle(), NativeMethods.OBEXTENSION_ID_IR_EXP, irExp));
    }

    /**
     * Astra pro plus，大白，Gemini，projector等带LDP功能的设备支持这个接口，这个接口断电不会保存，Astra系列的设备不支持这个接口。
     *
     * @return 0 disable;1 enable
     */
    public int getLdpEnableRegister() {
        OutArg<Integer> val = new OutArg<Integer>();
        NativeMethods.checkReturnStatus(NativeMethods.oniDeviceGetIntProperty(getHandle(), NativeMethods.XN_MODULE_PROPERTY_LDP_ENABLE, val));
        return val.mValue;
    }

    /**
     * LDP 使能,写的是寄存器，断电重启后，设置的值不会保存。
     * Astra pro plus，大白，Gemini，projector，支持这个接口，Astra系列的设备不支持这个接口
     *
     * @param ldpEnable 0或者1
     */
    public void setLdpEnableRegister(int ldpEnable) {
        NativeMethods.checkReturnStatus(NativeMethods.oniDeviceSetProperty(getHandle(), NativeMethods.XN_MODULE_PROPERTY_LDP_ENABLE, ldpEnable));
    }

    /**
     * 这个扩展API，一代Astra系列和一代升级的Astra 系列某些带LDP产品有这个功能
     * 接口写的是flash，断电后，LDP设置的状态会保存
     *
     * @param ldpEnable 0或者1
     */
    public void setLdpEnable(int ldpEnable) {
        NativeMethods.checkReturnStatus(NativeMethods.oniDeviceSetProperty(getHandle(), NativeMethods.OBEXTENSION_ID_LDP_EN, ldpEnable));
    }


    /**
     * Astra pro plus，大白，Gemini，projector，支持这个接口，astra系列的产品没有获取激光状态的接口。
     *
     * @return
     */
    public int getEmitterEnable() {
        OutArg<Integer> val = new OutArg<Integer>();
        NativeMethods.checkReturnStatus(NativeMethods.oniDeviceGetIntProperty(getHandle(), NativeMethods.XN_MODULE_PROPERTY_EMITTER_STATE_V1, val));
        return val.mValue;
    }

    /**
     * 设置LDM激光开关 ,与 {@link #setLaserEnable(boolean)} 效果一致
     *
     * @param enable
     */
    public void setEmitterEnable(int enable) {
        NativeMethods.checkReturnStatus(NativeMethods.oniDeviceSetProperty(getHandle(), NativeMethods.XN_MODULE_PROPERTY_EMITTER_STATE, enable));
    }


    /**
     * 设置泛光灯，需要设备支持泛光灯功能，如：Gemini支持泛光灯。
     *
     * @return
     */
    public int getIrFloodEnable() {
        OutArg<Integer> val = new OutArg<Integer>();
        NativeMethods.checkReturnStatus(NativeMethods.oniDeviceGetIntProperty(getHandle(), NativeMethods.XN_MODULE_PROPERTY_IRFLOOD_STATE, val));
        return val.mValue;
    }

    /**
     * 获取泛光灯状态，需要设备支持泛光灯功能，如：Gemini支持泛光灯。
     *
     * @param enable
     */
    public void setIrFloodEnable(int enable) {
        NativeMethods.checkReturnStatus(NativeMethods.oniDeviceSetProperty(getHandle(), NativeMethods.XN_MODULE_PROPERTY_IRFLOOD_STATE, enable));
    }


    /* add for Orbbec extend API by LiuDongBing End; */

    /**
     * get depth and ir alternate mode (need firmware support)
     *
     * @return 0 :depth and NIR alternate ;1 :speckle ir
     */
    public int getDepthIrAlternateMode() {
        OutArg<Integer> val = new OutArg<Integer>();
        NativeMethods.checkReturnStatus(NativeMethods.oniDeviceGetIntProperty(getHandle(), NativeMethods.XN_MODULE_PROPERTY_DEPTH_IR_MODE, val));
        return val.mValue;
    }

    /**
     * set depth and ir alternate mode (need firmware support)
     *
     * @param
     */
    public void setDepthIrAlternateMode(int nMode) {
        NativeMethods.checkReturnStatus(NativeMethods.oniDeviceSetProperty(getHandle(), NativeMethods.XN_MODULE_PROPERTY_DEPTH_IR_MODE, nMode));
    }

    /**
     * This interface can get the type of transmission used by the current USB device.
     * the return value can be compared with the USB speed type in DeviceUSBSpeed class.The
     * result may be: USB_LOW_SPEED, USB_FULL_SPEED, USB_HIGH_SPEED or USB_SUPER_SPEED.
     * <p>
     * See {@link DeviceUSBSpeed} for USB speed type.
     *
     * @return
     */
    public int getUSBSpeed() {
        OutArg<Integer> val = new OutArg<Integer>();
        NativeMethods.oniGetDeviceUSBSpeed(getHandle(), val);

        return val.mValue;
    }

    /**
     * Request IR real time temperature.
     *
     * @return
     */
    public double getIrRealTimeTemperature() {
        OutArg<Double> val = new OutArg<Double>();
        NativeMethods.oniDeviceGetDoubleProperty(getHandle(), NativeMethods.DEVICE_PROPERTY_RT_IR_TEMP, val);
        return val.mValue;
    }

    /**
     * Request LDMP real time temperature.
     *
     * @return
     */
    public double getLdmpRealTimeTemperature() {
        OutArg<Double> val = new OutArg<Double>();
        NativeMethods.oniDeviceGetDoubleProperty(getHandle(), NativeMethods.DEVICE_PROPERTY_RT_LDMP_TEMP, val);
        return val.mValue;
    }

    /**
     * Get Device internal and external parameters, can be used as needed.
     *
     * @return {@link org.openni.OBCameraParams}
     */
    public OBCameraParams getOBCameraParams() {
        OBCameraParams softD2C = new OBCameraParams();
        NativeMethods.checkReturnStatus(NativeMethods.oniDeviceGetCameraParams(getHandle(), softD2C));
        return softD2C;
    }

    /**
     * Get firmware version
     *
     * @return
     */
    public String getFirmwareVersion() {
        OutArg<String> val = new OutArg<String>();
        NativeMethods.checkReturnStatus(NativeMethods.oniDeviceGetStringProperty(getHandle(), NativeMethods.DEVICE_PROPERTY_SENSOR_PLATFORM_STRING, val));
        return val.mValue;
    }

    /**
     * Get device of SerialNumber.
     *
     * @return
     */
    public String getSerialNumber() {
        OutArg<String> val = new OutArg<String>();
        NativeMethods.checkReturnStatus(NativeMethods.oniDeviceGetSerialNumber(getHandle(), NativeMethods.DEVICE_PROPERTY_SERIALNUMBER, val));
        return val.mValue;
    }

    private long mDeviceHandle;
    private PlaybackControl mPlaybackControl;
}

