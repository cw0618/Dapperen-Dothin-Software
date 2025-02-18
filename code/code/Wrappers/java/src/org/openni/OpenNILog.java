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

import org.openni.android.IOpenNILogCat;

/**
 * Created by lixiaobin.
 * DateTime: 2020/1/7 9:34
 * Description:Redirect log output object.
 */
public class OpenNILog {

    private static IOpenNILogCat oniLogcat;

    public static void setLogRedirect(IOpenNILogCat logcat) {
        oniLogcat = logcat;
    }


    public static void v(String tag, String msg) {
        if (oniLogcat != null) {
            oniLogcat.verbose(tag, msg);
        }

    }

    public static void d(String tag, String msg) {
        if (oniLogcat != null) {
            oniLogcat.debug(tag, msg);
        }

    }

    public static void i(String tag, String msg) {
        if (oniLogcat != null) {
            oniLogcat.info(tag, msg);
        }

    }

    public static void w(String tag, String msg) {
        if (oniLogcat != null) {
            oniLogcat.warning(tag, msg);
        }

    }

    public static void e(String tag, String msg) {
        if (oniLogcat != null) {
            oniLogcat.error(tag, msg);
        }

    }
}
