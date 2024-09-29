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

public class Plane {

    private final Point3D<Float> mCenter;
    private final Point3D<Float> mNormal;

    public Plane(Point3D<Float> center, Point3D<Float> normal) {
        this.mCenter = center;
        this.mNormal = normal;
    }

    /**
     * Gives the center point of the detected plane.
     *
     * @return Center point.
     */
    public final Point3D<Float> getCenterPoint() {
        return mCenter;
    }

    /**
     * Gives the normal vector of the detected plane.
     *
     * @return Normal vector.
     */
    public final Point3D<Float> getNormalVector() {
        return mNormal;
    }

    @Override
    public String toString() {
        return "Plane{ " +
                "center point={" + mCenter.getX() + ", " + mCenter.getY() + ", " + mCenter.getZ() + "}" +
                ", normal vector={" + mNormal.getX() + ", " + mNormal.getY() + ", " + mNormal.getZ() + "}" +
                " }";
    }
}
