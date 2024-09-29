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


public class OBCameraParams {

    private int width;
    private int height;

    private float[] d_intr_p;
    private float[] c_intr_p;

    private float[] d_k;
    private float[] c_k;

    private float[] d2c_t;
    private float[] d2c_r;

    /**
     * Gives the width of calibration image, measured in pixels.
     *
     * @return Width of calibration image.
     */
    public int getWidth() { return width; }

    /**
     * Gives the height of calibration image, measured in pixels.
     *
     * @return Height of calibration image.
     */
    public int getHeight() { return height; }

    /**
     * Gives the intrinsic parameters of Depth sensor.
     *
     * @return [fx, fy, cx, cy]
     **/
    public float[] getD_intr_p() {
        return d_intr_p;
    }

    public void setD_intr_p(float[] d_intr_p) {
        this.d_intr_p = d_intr_p;
    }

    /**
     * Gives the intrinsic parameters of Color sensor.
     *
     * @return [fx, fy, cx, cy]
     **/
    public float[] getC_intr_p() {
        return c_intr_p;
    }

    public void setC_intr_p(float[] c_intr_p) {
        this.c_intr_p = c_intr_p;
    }

    /**
     * Gives the distortion parameters of Depth sensor.
     *
     * @return [k1, k2, k3, p1, p2]
     **/
    public float[] getD_k() {
        return d_k;
    }

    public void setD_k(float[] d_k) {
        this.d_k = d_k;
    }

    /**
     * Gives the distortion parameters of Color sensor.
     *
     * @return [k1, k2, k3, p1, p2]
     **/
    public float[] getC_k() {
        return c_k;
    }

    public void setC_k(float[] c_k) {
        this.c_k = c_k;
    }

    /**
     * Gives the translation matrix.
     *
     * @return [t1, t2, t3]
     **/
    public float[] getD2c_t() {
        return d2c_t;
    }

    public void setD2c_t(float[] d2c_t) {
        this.d2c_t = d2c_t;
    }

    /**
     * Gives the rotation matrix.
     *
     * @return [r00, r01, r02; r10, r11, r12; r20, r21, r22]
     **/
    public float[] getD2c_r() {
        return d2c_r;
    }

    public void setD2c_r(float[] d2c_r) {
        this.d2c_r = d2c_r;
    }
}
