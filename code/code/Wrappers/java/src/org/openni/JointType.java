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

import java.util.NoSuchElementException;

/**
 * Provides joint type for all joint type codes. <BR>
 * <BR>
 */
public enum JointType {
    JOINT_HEAD (0),
    JOINT_NECK (1),
    JOINT_MID_SPINE (2),
    JOINT_RIGHT_SHOULDER (3),
    JOINT_LEFT_SHOULDER (4),
    JOINT_RIGHT_ELBOW (5),
    JOINT_LEFT_ELBOW (6),
    JOINT_RIGHT_WRIST (7),
    JOINT_LEFT_WRIST (8),
    JOINT_RIGHT_HIP (9),
    JOINT_LEFT_HIP (10),
    JOINT_RIGHT_KNEE (11),
    JOINT_LEFT_KNEE (12),
    JOINT_RIGHT_ANKLE (13),
    JOINT_LEFT_ANKLE (14),
    JOINT_MAX (15);

    public int toNative() {
        return this.mValue;
    }

    public static JointType fromNative(int value) {
        for (JointType type : JointType.values()) {
            if (type.mValue == value) return type;
        }

        throw new NoSuchElementException();
    }

    private final int mValue;

    private JointType(int value) {
        this.mValue = value;
    }
}
