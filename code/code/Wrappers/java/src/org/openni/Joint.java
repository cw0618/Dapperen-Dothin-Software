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


public class Joint {

    private final JointType mType;
    private final double mScore;
    private final Point3D<Double> mWorldPosition;

    public Joint(int type, double score, Point3D<Double> position) {
        this.mScore = score;
        this.mType = JointType.fromNative(type);
        this.mWorldPosition = position;
    }

    /**
     * Gives the type of current joint. See the {@link JointType} for all possible values.
     *
     * @return Joint type
     **/
    public final JointType getType() {
        return mType;
    }

    /**
     * Gives the confidence of current joint.
     *
     * @return Joint confidence
     **/
    public final double getScore() {
        return mScore;
    }

    /**
     * Gives the current joint in world space.
     *
     * @return World coordinate of joint
     **/
    public final Point3D<Double> getWorldPosition() {
        return mWorldPosition;
    }

    @Override
    public String toString() {
        return "Joint{ " +
                "type=" + mType +
                ", score=" + mScore +
                ", world position=" + mWorldPosition.toString() +
                " }";
    }
}
