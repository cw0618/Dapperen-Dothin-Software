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

public enum AIStatus {

    AI_STATUS_NONE(0),
    AI_STATUS_NOT_TRACKING(1),
    AI_STATUS_TRACKING_STARTED(2),
    AI_STATUS_TRACKING(3),
    AI_STATUS_LOST(4);

    public int toNative() {
        return this.mValue;
    }

    public static AIStatus fromNative(int value) {
        for (AIStatus status : AIStatus.values()) {
            if (status.mValue == value) return status;
        }

        throw new NoSuchElementException();
    }

    private final int mValue;

    private AIStatus(int value) {
        this.mValue = value;
    }
}
