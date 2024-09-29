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

public class Body {

    private final int mID;
    private final int mJointWidth;
    private final int mJointHeight;

    private PixelFormat mJointFormat;
    private final AIStatus mJointStatus;
    private final AIStatus mBodyShapeStatus;

    private final BodyShape mBodyShape;
    private final ArrayList<Joint> mJointList;

    public Body(int id, int width, int height, int jointFormat, int jointStatus, int bodyShapeStatus, ArrayList<Joint> jointList, BodyShape bodyShape) {
        this.mID = id;
        this.mJointWidth = width;
        this.mJointHeight = height;
        this.mJointList = jointList;
        this.mBodyShape = bodyShape;
        this.mJointFormat = PixelFormat.fromNative(jointFormat);
        this.mJointStatus = AIStatus.fromNative(jointStatus);
        this.mBodyShapeStatus = AIStatus.fromNative(bodyShapeStatus);
    }

    /**
     * Gives the identity of the currently detected body frame.
     *
     * @return ID
     **/
    public final int getID() {
        return mID;
    }

    /**
     * Check if the body shape are tracking. See the {@link AIStatus} for all possible values.
     *
     * @return The tracking status.
     */
    public final AIStatus getBodyShapeStatus() {
        return mBodyShapeStatus;
    }

    /**
     * Gives the body shape produced by this frame. See the {@link BodyShape} for all possible values.
     *
     * @return Body shape.
     */
    public final BodyShape getBodyShape() {
        return mBodyShape;
    }

    /**
     * Check if the joints are tracking. See the {@link AIStatus} for all possible values.
     *
     * @return The tracking status.
     */
    public final AIStatus getJointStatus() {
        return mJointStatus;
    }

    /**
     * Gives the format of current joint. See the {@link PixelFormat} for all possible values.
     *
     * @return Joint format (2D OR 3D)
     **/
    public final PixelFormat getJointFormat() {
        return mJointFormat;
    }

    /**
     * Gives the current color width of this frame which used to calculate joint info, measured in pixels.
     *
     * @return Color width of this frame in pixels.
     **/
    public final int getJointWidth() {
        return mJointWidth;
    }

    /**
     * Gives the current color height of this frame which used to calculate joint info, measured in pixels.
     *
     * @return Color height of this frame in pixels.
     **/
    public final int getJointHeight() {
        return mJointHeight;
    }

    /**
     * Gives the specified joint by type.
     *
     * @param type object corresponding to the joint type specified.
     *
     * @return a specified joint or null if such a joint is not available from this body frame.
     **/
    public final Joint getJoint(JointType type) {
        if (type.equals(JointType.JOINT_MAX)) {
            return null;
        }

        return mJointList.get(type.toNative());
    }

    /**
     * Gives an array list of joints of the currently tracking body frame.
     *
     * @return an array list of joints
     **/
    public final ArrayList<Joint> getJointList() { return mJointList; }

    @Override
    public String toString() {
        return "Body{ " +
                "id=" + mID +
                ", color resolution={" + mJointWidth + "x" + mJointHeight + "}" +
                ", joint list={" + mJointList.toString() + "}" +
                " }";
    }
}
