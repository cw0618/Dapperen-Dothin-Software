package org.openni;

public class FloorInfo {

    private final Plane mPlane;
    private final AIStatus mStatus;
    private final BitmapMask mBitmapMask;

    FloorInfo(int status, Plane plane, BitmapMask bitmapMask) {
        this.mPlane = plane;
        this.mBitmapMask = bitmapMask;
        this.mStatus = AIStatus.fromNative(status);
    }

    /**
     * Check if the floor info are tracking. See the {@link AIStatus} for all possible values.
     *
     * @return The tracking status.
     */
    public final AIStatus getStatus() {
        return mStatus;
    }

    /**
     * Gives the plane info of this floor info.
     *
     * @return The plane.
     */
    public final Plane getPlane() {
        return mPlane;
    }

    /**
     * Gives the current bitmap mask of this floor info. See the {@link BitmapMask} for all possible values.
     *
     * @return The bitmap mask.
     **/
    public final BitmapMask getBitmapMask() { return mBitmapMask; }

    @Override
    public String toString() {
        return "FloorInfo{" +
                "status=" + mStatus +
                ", plane=" + mPlane +
                ", bitmapMask=" + mBitmapMask +
                '}';
    }
}
