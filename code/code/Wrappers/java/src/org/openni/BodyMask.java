package org.openni;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

public class BodyMask {

    private final AIStatus mStatus;
    private final BitmapMask mBitmapMask;

    BodyMask(int status, BitmapMask bitmapMask) {
        this.mBitmapMask = bitmapMask;
        this.mStatus = AIStatus.fromNative(status);
    }

    /**
     * Check if the body mask are tracking. See the {@link AIStatus} for all possible values.
     *
     * @return The tracking status.
     */
    public final AIStatus getStatus() {
        return mStatus;
    }

    /**
     * Gives the current bitmap mask of this body frame. See the {@link BitmapMask} for all possible values.
     *
     * @return The bitmap mask.
     **/
    public final BitmapMask getBitmapMask() { return mBitmapMask; }

    @Override
    public String toString() {
        return "BodyMask{" +
                "status=" + mStatus +
                ", mask=" + mBitmapMask +
                '}';
    }
}
