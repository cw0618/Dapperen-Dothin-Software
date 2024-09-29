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

import android.util.Log;

import java.nio.ByteBuffer;
import java.util.Iterator;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

/**
 * The {@link VideoStream} object encapsulates a single video stream from a device. Once created, it
 * is used to start data flow from the device, and to read individual frames of data. This is the
 * central class used to obtain data in OpenNI. It provides the ability to manually read data in a
 * polling loop, as well as providing events and a Listener class that can be used to implement
 * event-driven data acquisition.
 * <p>
 * Aside from the video data frames themselves, the class offers a number of functions used for
 * obtaining information about a {@link VideoStream}. Field of view, available video modes, and
 * minimum and maximum valid pixel values can all be obtained.
 * <p>
 * In addition to obtaining data, the {@link VideoStream} object is used to set all configuration
 * properties that apply to a specific stream (rather than to an entire device). In particular, it
 * is used to control cropping, mirroring, and video modes.
 * <p>
 * A valid, initialized device that provides the desired stream type is required to create a stream.
 * <p>
 * Several video streams can be created to stream data from the same sensor. This is useful if
 * several components of an application need to read frames separately.
 * <p>
 * While some device might allow different streams from the same sensor to have different
 * configurations, most devices will have a single configuration for the sensor, shared by all
 * streams.
 */
public class VideoStream {
    /**
     * The {@link CameraSettings} object encapsulates camera setting for a single device. Once
     * created, it is used to get and set auto exposure and auto white balance modes.
     */
    public class CameraSettings {
        /**
         * Set Auto Exposure Enabled for corresponding sensor
         *
         * @param enabled boolean value for set and unset auto exposure mode
         */
        public void setAutoExposureEnabled(boolean enabled) {
            NativeMethods.checkReturnStatus(NativeMethods.oniStreamSetProperty(mVideoStream.getHandle(),
                    NativeMethods.STREAM_PROPERTY_AUTO_EXPOSURE, enabled));
        }

        /**
         * Set Auto White Balance for corresponding sensor
         *
         * @param enabled boolean value for set and unset auto white balance mode
         */
        public void setAutoWhiteBalanceEnabled(boolean enabled) {
            NativeMethods.checkReturnStatus(NativeMethods.oniStreamSetProperty(mVideoStream.getHandle(),
                    NativeMethods.STREAM_PROPERTY_AUTO_WHITE_BALANCE, enabled));
        }

        /**
         * Set Auto Exposure Enabled for corresponding sensor
         *
         * @return enabled boolean value which define auto exposure mode state
         */
        public boolean getAutoExposureEnabled() {
            OutArg<Boolean> val = new OutArg<Boolean>();
            NativeMethods.oniStreamGetBoolProperty(mVideoStream.getHandle(),
                    NativeMethods.STREAM_PROPERTY_AUTO_EXPOSURE, val);
            return val.mValue;
        }

        /**
         * Set Auto White Balance Enabled for corresponding sensor
         *
         * @return enabled boolean value which define auto white balance mode state
         */
        public boolean getAutoWhiteBalanceEnabled() {
            OutArg<Boolean> val = new OutArg<Boolean>();
            NativeMethods.oniStreamGetBoolProperty(mVideoStream.getHandle(),
                    NativeMethods.STREAM_PROPERTY_AUTO_WHITE_BALANCE, val);
            return val.mValue;
        }

        public void setGain(int gain) {
            NativeMethods.checkReturnStatus(
                    NativeMethods.oniStreamSetProperty(mVideoStream.getHandle(), NativeMethods.STREAM_PROPERTY_GAIN, gain));
        }

        public int getGain() {
            OutArg<Integer> val = new OutArg<Integer>();
            NativeMethods.checkReturnStatus(
                    NativeMethods.oniStreamGetIntProperty(mVideoStream.getHandle(), NativeMethods.STREAM_PROPERTY_GAIN, val));
            return val.mValue;
        }

        public void setExposure(int exposure) {
            NativeMethods.checkReturnStatus(
                    NativeMethods.oniStreamSetProperty(mVideoStream.getHandle(), NativeMethods.STREAM_PROPERTY_EXPOSURE, exposure));
        }

        public int getExposure() {
            OutArg<Integer> val = new OutArg<Integer>();
            NativeMethods.checkReturnStatus(
                    NativeMethods.oniStreamGetIntProperty(mVideoStream.getHandle(), NativeMethods.STREAM_PROPERTY_EXPOSURE, val));
            return val.mValue;
        }

        private CameraSettings(VideoStream videoStream) {
            this.mVideoStream = videoStream;
        }

        final private VideoStream mVideoStream;
    }

    /**
     * The VideoStream::NewFrameListener interface is provided to allow the implementation of event
     * driven frame reading. To use it, create a class that inherits from it and implement override
     * the onNewFrame() method. Then, register your created class with an active {@link VideoStream}
     * using the {@link #addNewFrameListener(NewFrameListener)} function. Once
     * this is done, the event handler function you implemented will be called whenever a new frame
     * becomes available. You may call {@link VideoStream#readFrame()} from within the
     * event handler.
     */
    public interface NewFrameListener {
        /**
         * Derived classes should implement this function to handle new frames.
         */
        public void onFrameReady(VideoStream stream);
    }

    /**
     * Creates a stream of frames from a specific sensor type of a specific device. You must supply a
     * reference to a Device that supplies the sensor type requested. You can use
     * {@link Device#hasSensor(SensorType)} to check whether a given sensor is available on
     * your target device before calling create().
     *
     * @param device     A reference to the {@link Device} you want to create the stream on.
     * @param sensorType The type of sensor the stream should produce data from.
     */
    public static VideoStream create(Device device, SensorType sensorType) {
        VideoStream videoStream = new VideoStream(sensorType);
        if (mFrameListeners == null) mFrameListeners = new ConcurrentHashMap<>();
        NativeMethods.checkReturnStatus(NativeMethods.oniDeviceCreateStream(device.getHandle(),
                sensorType.toNative(), videoStream));
        return videoStream;
    }

    /**
     * Destroy this stream. This function is currently called automatically by the destructor, but it
     * is considered a best practice for applications to manually call this function on any
     * {@link VideoStream} that they call create() for.
     */
    public void destroy() {
        NativeMethods.oniStreamDestroy(getHandle(), mCallbackHandle);
        mStreamHandle = 0;
    }

    public ParamsRegistrationMode getSoftwareRegistratorMode() {

        OutArg<Integer> value = new OutArg<Integer>();
        int rc = NativeMethods.oniStreamGetIntProperty(getHandle(), NativeMethods.STREAM_PROPERTY_SOFTWARE_REGISTRATION, value);
        if (rc != 0) {
            return ParamsRegistrationMode.OFF;
        }
        return ParamsRegistrationMode.fromNative(value.mValue);
    }

    public void setSoftwareRegistrator(ParamsRegistrationMode mode) {
        NativeMethods.checkReturnStatus(NativeMethods.oniStreamSetProperty(getHandle(), NativeMethods.STREAM_PROPERTY_SOFTWARE_REGISTRATION, mode.toNative()));
    }

    public void setSoftFilterEnable(boolean isEnabled) {
        NativeMethods.checkReturnStatus(NativeMethods.oniStreamSetProperty(getHandle(), NativeMethods.DEVICE_PROPERTY_SOFTWARE_FILTER, isEnabled));
    }

    /**
     * Provides the {@link SensorInfo} object associated with the sensor that is producing this
     * {@link VideoStream}.
     * <p>
     * {@link SensorInfo} is useful primarily as a means of learning which video modes are valid for
     * this VideoStream.
     *
     * @return SensorInfo object associated with the sensor providing this stream.
     */
    public final SensorInfo getSensorInfo() {
        return NativeMethods.oniStreamGetSensorInfo(getHandle());
    }

    /**
     * Starts data generation from this video stream.
     */
    public void start() {
        NativeMethods.checkReturnStatus(NativeMethods.oniStreamStart(getHandle()));
    }

    /**
     * Stops data generation from this video stream.
     */
    public void stop() {
        NativeMethods.oniStreamStop(getHandle());
    }

    /**
     * Read the next frame from this video stream, delivered as a {@link VideoFrameRef}. This is the
     * primary method for manually obtaining frames of video data. If no new frame is available, the
     * call will block until one is available. To avoid blocking, use
     * {@link NewFrameListener} to implement an event driven architecture. Another
     * alternative is to use {@link OpenNI#waitForAnyStream(java.util.List, int)} to wait
     * for new frames from several streams
     *
     * @return VideoFrameRef object which hold the data of the new frame.
     */
    public VideoFrameRef readFrame() {
        OutArg<VideoFrameRef> frame = new OutArg<VideoFrameRef>();
        int statusCode = NativeMethods.oniStreamReadFrame(getHandle(), frame);
        if (statusCode != 0) {
            Log.w("VideoStream", "VideoStream read frame is null");
            return null;
        }
        //NativeMethods.checkReturnStatus(NativeMethods.oniStreamReadFrame(getHandle(), frame));
        return frame.mValue;
    }

    /**
     * Read the next frame from this AI stream, delivered as a {@link AIFrameRef}. This is the
     * primary method for manually obtaining frames of AI data. If no new frame is available, the
     * call will block until one is available. To avoid blocking, use {@link NewFrameListener} to
     * implement an event driven architecture. Another alternative is to use {@link
     * OpenNI#waitForAnyStream(java.util.List, int)} to wait for new frames from several streams.
     *
     * @return AIFrameRef object which hold the data of the new frame.
     */
    public AIFrameRef readFrameEx() {
        OutArg<AIFrameRef> frame = new OutArg<AIFrameRef>();
        int statusCode = NativeMethods.oniStreamReadFrameEx(getHandle(), frame);
        if (statusCode != 0) {
            Log.w("VideoStream", "VideoStream read AI frame is null");
            return null;
        }
        return frame.mValue;
    }

    /**
     * Adds a new Listener to receive this VideoStream onNewFrame event. See
     * {@link NewFrameListener} for more information on implementing an event driven
     * frame reading architecture.
     *
     * @param streamListener Object which implements {@link NewFrameListener} that will
     *                       respond to this event.
     */
    public void addNewFrameListener(NewFrameListener streamListener) {
        mFrameListeners.put(this, streamListener);
    }

    /**
     * Removes a Listener from this video stream list. The listener removed will no longer receive new
     * frame events from this stream.
     *
     * @param streamListener Object of the listener to be removed.
     */
    public void removeNewFrameListener(NewFrameListener streamListener) {
        for (Map.Entry<VideoStream, NewFrameListener> pairs : mFrameListeners.entrySet()) {
            VideoStream videoStream = pairs.getKey();
            if (videoStream.getHandle() == mStreamHandle) {
                if (streamListener.equals(pairs.getValue())) {
                    mFrameListeners.remove(pairs.getKey());
                    return;
                }
            }
        }
    }

    /**
     * This function return stream handle.
     *
     * @return OpenNI stream handle.
     */
    public long getHandle() {
        return mStreamHandle;
    }

    /**
     * Gets an object through which several camera settings can be configured.
     *
     * @return null if the stream doesn't support camera settings.
     */
    public CameraSettings getCameraSettings() {
        if (NativeMethods.oniStreamIsPropertySupported(getHandle(),
                NativeMethods.STREAM_PROPERTY_AUTO_EXPOSURE)
                && NativeMethods.oniStreamIsPropertySupported(getHandle(),
                NativeMethods.STREAM_PROPERTY_AUTO_WHITE_BALANCE)) {
            return new CameraSettings(this);
        }
        return null;
    }

    /**
     * Get the current video mode information for this video stream. This includes its resolution, fps
     * and stream format.
     *
     * @return Current video mode information for this video stream.
     */
    public final VideoMode getVideoMode() {
        OutArg<VideoMode> videoMode = new OutArg<VideoMode>();
        NativeMethods.checkReturnStatus(NativeMethods.getVideoMode(getHandle(), videoMode));
        return videoMode.mValue;
    }

    /**
     * Changes the current video mode of this stream. Recommended practice is to use
     * {@link Device#getSensorInfo(SensorType)}, and then
     * {@link SensorInfo#getSupportedVideoModes()} to obtain a list of valid video mode
     * settings for this stream. Then, pass a valid {@link VideoMode} to setVideoMode(VideoMode) to
     * ensure correct operation.
     *
     * @param videoMode Desired new video mode for this stream. returns Status code indicating success
     *                  or failure of this operation.
     */
    public void setVideoMode(VideoMode videoMode) {
        NativeMethods.checkReturnStatus(NativeMethods.setVideoMode(getHandle(), videoMode
                .getResolutionX(), videoMode.getResolutionY(), videoMode.getFps(), videoMode
                .getPixelFormat().toNative()));
    }

    /**
     * Provides the maximum possible value for pixels obtained by this stream. This is most useful for
     * getting the maximum possible value of depth streams.
     *
     * @return Maximum possible pixel value.
     */
    public int getMaxPixelValue() {
        OutArg<Integer> val = new OutArg<Integer>();
        NativeMethods
                .oniStreamGetIntProperty(getHandle(), NativeMethods.STREAM_PROPERTY_MAX_VALUE, val);
        return val.mValue;
    }

    /**
     * Provides the smallest possible value for pixels obtains by this VideoStream. This is most
     * useful for getting the minimum possible value that will be reported by a depth stream.
     *
     * @return Minimum possible pixel value that can come from this stream.
     */
    public int getMinPixelValue() {
        OutArg<Integer> val = new OutArg<Integer>();
        NativeMethods.checkReturnStatus(NativeMethods.oniStreamGetIntProperty(getHandle(),
                NativeMethods.STREAM_PROPERTY_MIN_VALUE, val));
        return val.mValue;
    }

    /**
     * Checks whether this stream supports cropping.
     *
     * @return true if the stream supports cropping, false if it does not.
     */
    public boolean isCroppingSupported() {
        return NativeMethods.oniStreamIsPropertySupported(getHandle(),
                NativeMethods.STREAM_PROPERTY_CROPPING);
    }

    /**
     * Obtains the current cropping settings for this stream.
     *
     * @return CropArea CropArea object which encapsulated cropping info.
     */
    public CropArea getCropping() {
        OutArg<Integer> xRes = new OutArg<Integer>();
        OutArg<Integer> yRes = new OutArg<Integer>();
        OutArg<Integer> w = new OutArg<Integer>();
        OutArg<Integer> h = new OutArg<Integer>();

        NativeMethods.checkReturnStatus(NativeMethods.getCropping(getHandle(), xRes, yRes, w, h));
        return new CropArea(xRes.mValue, yRes.mValue, w.mValue, h.mValue);
    }

    /**
     * Changes the cropping settings for this stream. You can use the {@link #isCroppingSupported()}
     * function to make sure cropping is supported before calling this function.
     *
     * @param cropping CropArea object which set corresponding cropping information.
     */
    public void setCropping(CropArea cropping) {
        NativeMethods.checkReturnStatus(NativeMethods.setCropping(getHandle(), cropping.getOriginX(),
                cropping.getOriginY(), cropping.getWidth(), cropping.getHeight()));
    }

    /**
     * Disables cropping.
     */
    public void resetCropping() {
        NativeMethods.checkReturnStatus(NativeMethods.resetCropping(getHandle()));
    }

    public void filterSpeckles(ByteBuffer buf, int newVal, int maxSpeckleSize, int maxDiff) {
        NativeMethods.oniStreamfilter(getHandle(), buf, newVal, maxSpeckleSize, maxDiff);
    }

    /**
     * Check whether mirroring is currently turned on for this stream.
     *
     * @return true if mirroring is currently enabled, false otherwise.
     */
    public boolean getMirroringEnabled() {
        OutArg<Boolean> val = new OutArg<Boolean>();
        NativeMethods.checkReturnStatus(NativeMethods.oniStreamGetBoolProperty(getHandle(),
                NativeMethods.STREAM_PROPERTY_MIRRORING, val));
        return val.mValue;
    }

    /**
     * Enable or disable mirroring for this stream.
     *
     * @param isEnabled true to enable mirroring, false to disable it.
     */
    public void setMirroringEnabled(boolean isEnabled) {
        NativeMethods.checkReturnStatus(NativeMethods.oniStreamSetProperty(getHandle(),
                NativeMethods.STREAM_PROPERTY_MIRRORING, isEnabled));
    }

    /**
     * Gets the horizontal field of view of frames received from this stream.
     *
     * @return Horizontal field of view, in radians.
     */
    public float getHorizontalFieldOfView() {
        OutArg<Float> val = new OutArg<Float>();
        NativeMethods.checkReturnStatus(NativeMethods.oniStreamGetFloatProperty(getHandle(),
                NativeMethods.STREAM_PROPERTY_HORIZONTAL_FOV, val));
        return val.mValue;
    }

    /**
     * Gets the vertical field of view of frames received from this stream.
     *
     * @return Vertical field of view, in radians.
     */
    public float getVerticalFieldOfView() {
        OutArg<Float> val = new OutArg<Float>();
        NativeMethods.checkReturnStatus(NativeMethods.oniStreamGetFloatProperty(getHandle(),
                NativeMethods.STREAM_PROPERTY_VERTICAL_FOV, val));
        return val.mValue;
    }

    /**
     * Gets the sensor type for this stream.
     *
     * @return sensor type.
     */
    public SensorType getSensorType() {
        return mSensorType;
    }

    private VideoStream(SensorType sensorType) {
        this.mSensorType = sensorType;
    }

    @SuppressWarnings("unused") /* Called from JNI */
    private static void onFrameReady(long streamHandle) {
        Iterator<ConcurrentHashMap.Entry<VideoStream, NewFrameListener>> pairs = mFrameListeners.entrySet().iterator();
        while (pairs.hasNext()) {
            ConcurrentHashMap.Entry<VideoStream, NewFrameListener> entry = pairs.next();
            VideoStream videoStream = entry.getKey();
            if (videoStream.getHandle() == streamHandle) {
                entry.getValue().onFrameReady(videoStream);
            }

        }

    }

    private final SensorType mSensorType;
    private static ConcurrentHashMap<VideoStream, NewFrameListener> mFrameListeners;
    private long mStreamHandle;
    private long mCallbackHandle;

    //add for VideoStream Property by LiuDongBing Start

    /**
     * 设置 Depth 最小值
     *
     * @param minDepth Min Depth
     */
    public void setStreamMinDepth(int minDepth) {
        NativeMethods.checkReturnStatus(
                NativeMethods.oniStreamSetProperty(getHandle(), NativeMethods.XN_STREAM_PROPERTY_MIN_DEPTH, minDepth));
    }

    /**
     * 设置 Depth 最大值
     *
     * @param maxDepth max Depth
     */
    public void setStreamMaxDepth(int maxDepth) {
        NativeMethods.checkReturnStatus(
                NativeMethods.oniStreamSetProperty(getHandle(), NativeMethods.XN_STREAM_PROPERTY_MAX_DEPTH, maxDepth));
    }

    /**
     * 6.6设置Depth传输格式
     *
     * @param inputFormat 2-> 10bit「atlas only」，3-> 11bit, 4-> 12bit
     */
    public void setStreamInputFormat(int inputFormat) {
        NativeMethods.checkReturnStatus(
                NativeMethods.oniStreamSetProperty(getHandle(), NativeMethods.XN_STREAM_PROPERTY_INPUT_FORMAT, inputFormat));
    }

    /**
     * atlas depth rotate
     *
     * @param enable 1-> enable Rotate,0-> unEnableRotate
     */
    public void setStreamDepthRotate(int enable) {
        NativeMethods.checkReturnStatus(
                NativeMethods.oniStreamSetProperty(getHandle(), NativeMethods.XN_STREAM_PROPERTY_DEPTH_ROTATE, enable));
    }

    /**
     * 设置 HoleFilter「HoleFilter代表窗口滤波器的大小」
     *
     * @param holeFilter 0->close HoleFilter 1-> 3*3，2-> 5*5，3-> 7*7，4-> 9*9
     */
    public void setStreamHoleFilter(int holeFilter) {
        NativeMethods.checkReturnStatus(
                NativeMethods.oniStreamSetProperty(getHandle(), NativeMethods.XN_STREAM_PROPERTY_HOLE_FILTER, holeFilter));
    }

    /**
     * 设置软件滤波
     *
     * @param softFilter CLOSE-> close；OPEN-> open
     */
    public void setSoftFilter(SoftFilterType softFilter) {
        NativeMethods.checkReturnStatus(
                NativeMethods.oniStreamSetProperty(getHandle(), NativeMethods.XN_STREAM_PROPERTY_SOFTWARE_FILTER, softFilter.toNative()));
    }
    //add for VideoStream Property by LiuDongBing End
}
