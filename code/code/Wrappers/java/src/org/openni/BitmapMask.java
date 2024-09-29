package org.openni;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

public class BitmapMask {

    private final int mWidth;
    private final int mHeight;
    private final ByteBuffer mMask;

    BitmapMask(int width, int height, ByteBuffer mask) {
        this.mWidth = width;
        this.mHeight = height;
        this.mMask = mask;
    }

    /**
     * Gives the current width of this mask frame, measured in pixels.
     *
     * @return Width of this mask frame in pixels.
     **/
    public final int getWidth() {
        return mWidth;
    }

    /**
     * Gives the current height of this mask frame, measured in pixels.
     *
     * @return Height of this mask frame in pixels.
     **/
    public final int getHeight() {
        return mHeight;
    }

    /**
     * Getter function for the array of mask data pointed to by this object.
     *
     * @return ByteBuffer object to the actual mask data array.
     */
    public final ByteBuffer getMask() { return mMask; }

    @Override
    public String toString() {
        return "BitmapMask{" +
                "width=" + mWidth +
                ", height=" + mHeight +
                ", mask=" + mMask +
                '}';
    }
}
