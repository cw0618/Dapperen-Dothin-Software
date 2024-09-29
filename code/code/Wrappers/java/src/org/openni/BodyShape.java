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

public class BodyShape {

    private final float mRatio;
    private final float mHeight;
    private final float mWaist;
    private final float mWaistline;
    private final float mBust;
    private final float mHips;
    private final float mShoulder;
    private final FigureType mFigure;
    private final ArrayList<Point2D<Integer>> mPointList;

    public BodyShape(int figure, float ratio, float height, float waist, float waistline, float bust, float hips, float shoulder, ArrayList<Point2D<Integer>> pointList) {
        this.mRatio = ratio;
        this.mHeight = height;
        this.mWaist = waist;
        this.mWaistline = waistline;
        this.mBust = bust;
        this.mHips = hips;
        this.mShoulder = shoulder;
        this.mPointList = pointList;
        this.mFigure = FigureType.fromNative(figure);
    }

    /**
     * Gives the body ratio of the person.
     *
     * @return body ratio
     * @note
     **/
    public final float getRatio() { return mRatio; }

    /**
     * Gives the height of the person.
     *
     * @return height
     **/
    public final float getHeight() { return mHeight; }

    /**
     * Gives the waist width of the person.
     *
     * @return waist
     **/
    public final float getWaist() {
        return mWaist;
    }

    /**
     * Gives the waistline of the person.
     *
     * @return waistline
     **/
    public final float getWaistline() {
        return mWaistline;
    }

    /**
     * Gives the bust of the person.
     *
     * @return bust
     **/
    public final float getBust() {
        return mBust;
    }

    /**
     * Gives the hips of the person.
     *
     * @return hips
     **/
    public final float getHips() {
        return mHips;
    }

    /**
     * Gives the shoulder width of the person.
     *
     * @return shoulder width
     **/
    public final float getShoulder() {
        return mShoulder;
    }

    /**
     * Gives the body figure type {@link FigureType}.
     *
     * @return figure type
     **/
    public final FigureType getFigureType() { return mFigure; }

    /**
     * Gives the body points {@link Point2D}.
     *
     * @return an array list of point
     **/
    public final ArrayList<Point2D<Integer>> getPointList() { return mPointList; }

    @Override
    public String toString() {
        return "BodyShapes{ " +
                "ratio=" + mRatio +
                ", height=" + mHeight +
                ", waist=" + mWaist +
                ", waistline=" + mWaistline +
                ", bust=" + mBust +
                ", hips=" + mHips +
                ", shoulder=" + mShoulder +
                ", figure=" + mFigure +
                ", points={" + mPointList.toString() + "}" +
                " }";
    }
}
