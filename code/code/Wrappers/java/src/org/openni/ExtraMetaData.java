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

import java.nio.ByteBuffer;

public class ExtraMetaData {

    private final int mWidth;
    private final int mHeight;
    private final int mStride;
    private final ByteBuffer mData;

    public ExtraMetaData(int width, int height, int stride, ByteBuffer buffer) {
        this.mWidth = width;
        this.mHeight = height;
        this.mStride = stride;
        this.mData = buffer;
    }

    /**
     * Getter function for the array of meta data pointed to by this object.
     *
     * @return ByteBuffer object to the actual frame data array. Type of data can be determined
     * according to the pixel format.
     */
    public final ByteBuffer getData() {
        return mData;
    }

    /**
     * Gives the current width of this frame, measured in pixels.
     *
     * @return Width of this frame in pixels.
     */
    public final int getWidth() {
        return mWidth;
    }

    /**
     * Gives the current height of this frame, measured in pixels.
     *
     * @return Height of this frame in pixels.
     */
    public final int getHeight() {
        return mHeight;
    }

    /**
     * Gives the length of one row of pixels, measured in bytes. Primarily useful for indexing the
     * array which contains the data.
     *
     * @return Stride of the array which contains the meta data for this frame, in bytes
     */
    public final int getStrideInBytes() {
        return mStride;
    }

    @Override
    public String toString() {
        return "Extra Meta Data{ " +
                "width=" + mWidth +
                ", height=" + mHeight +
                ", stride=" + mStride +
                ", data=" + mData.toString() +
                " }";
    }
}
