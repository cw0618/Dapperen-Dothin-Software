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
 ******************************************************************************/
package org.openni;

import java.util.ArrayList;

public class AIFrameRef {

    private final int mIndex;
    private final long mTimestamp;
    private final PixelFormat mPixFmt;
    private final BodyMask mBodyMask;
    private final FloorInfo mFloorInfo;
    private final ArrayList<Body> mBodyList;
    private static final String TAG = "AIFrameRef";

    private AIFrameRef(int index, int pixFmt, long timestamp, ArrayList<Body> bodyList, BodyMask bodyMask, FloorInfo floorInfo) {
        this.mIndex = index;
        this.mTimestamp = timestamp;
        this.mBodyMask = bodyMask;
        this.mFloorInfo = floorInfo;
        this.mBodyList = bodyList;
        this.mPixFmt = PixelFormat.fromNative(pixFmt);
    }

    /**
     * Check if this object references an actual frame.
     */
    public final boolean isValid()
    {
        return (mBodyList.size() > 0);
    }

    /**
     * Frames are provided sequential frame ID numbers by the sensor that produced them. If frame synchronization has been enabled
     * for a device via {@link Device#setDepthColorSyncEnabled(boolean)}, then frame numbers for corresponding frames of depth and
     * color are guaranteed to match.
     * <p>
     * If frame synchronization is not enabled, then there is no guarantee of matching frame indexes between {@link VideoStream} "VideoStreams".
     * In the latter case, applications should use timestamps instead of frame indexes to align frames in time.
     *
     * @return Index number for this frame.
     */
    public final int getFrameIndex() {
        return mIndex;
    }

    /**
     * Provides a timestamp for the frame.  The 'zero' point for this stamp is implementation specific, but all
     * streams from the same device are guaranteed to use the same zero. This value can therefore be used to compute
     * time deltas between frames from the same device, regardless of whether they are from the same stream.
     *
     * @returns Timestamp of frame, measured in microseconds from an arbitrary zero.
     */
    public final long getTimestamp()
    {
        return mTimestamp;
    }

    /**
     * Gives the pixel format of current AI frame. See the {@link PixelFormat} for all possible values.
     *
     * @return Pixel format.
     **/
    public final PixelFormat getPixelFormat() {
        return mPixFmt;
    }

    /**
     * Gives the number of bodies produced by this frame.
     *
     * @return The size of bodies.
     **/
    public final int getBodySize() {
        return mBodyList.size();
    }

    /**
     * Gives the specified body by index.
     *
     * @param index of body linked list.
     *
     * @return The specified body or null if such a body is not available from this body frame.
     **/
    public final Body getBody(int index) {
        if (index < 0 || index >= mBodyList.size()) {
            return null;
        }

        return mBodyList.get(index);
    }

    /**
     * Provides an array list of body info of the sensor this object is associated with.
     *
     * @return The array list of body info.
     */
    public final ArrayList<Body> getBodyList() {
        return mBodyList;
    }

    /**
     * Gives the body mask produced by this frame. See the {@link BodyMask} for all possible values.
     *
     * @return Body mask.
     */
    public final BodyMask getBodyMask() {
        return mBodyMask;
    }

    /**
     * Gives the floor info produced by this frame. See the {@link FloorInfo} for all possible values.
     *
     * @return Floor info.
     */
    public final FloorInfo getFloorInfo() {
        return mFloorInfo;
    }

    @Override
    public String toString() {
        return "BodyFrame{ " +
                "index=" + mIndex +
                ", timestamp=" + mTimestamp +
                ", body list={ " + mBodyList.toString() + " }" +
                " }";
    }
}
