/*****************************************************************************
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
#include <stdio.h>
#include <OpenNI.h>

#include "OniSampleUtilities.h"

#define SAMPLE_READ_WAIT_TIMEOUT 2000

using namespace openni;

void analyzeFrame(const VideoFrameRef &frame)
{
    DepthPixel* pDepth = NULL;
    RGB888Pixel* pColor = NULL;

    int middleIndex = (frame.getHeight() + 1) * frame.getWidth() / 2;
    switch (frame.getVideoMode().getPixelFormat())
    {
    case PIXEL_FORMAT_DEPTH_1_MM:
    case PIXEL_FORMAT_DEPTH_100_UM:
        pDepth = (DepthPixel*)frame.getData();
        printf("[%08llu] %8d\n", (long long)frame.getTimestamp(), pDepth[middleIndex]);
        break;
    case PIXEL_FORMAT_RGB888:
        pColor = (RGB888Pixel*)frame.getData();
        printf("[%08llu] 0x%02x%02x%02x\n",
            (long long)frame.getTimestamp(),
            pColor[middleIndex].r & 0xff,
            pColor[middleIndex].g & 0xff,
            pColor[middleIndex].b & 0xff);
        break;
    default:
        printf("Unknown format\n");
    }
}

int main()
{
    Status rc = OpenNI::initialize();
    if (STATUS_OK != rc)
    {
        printf("Initialize failed\n%s\n", OpenNI::getExtendedError());
        return 1;
    }

    Device device;
    rc = device.open(ANY_DEVICE);
    if (STATUS_OK != rc)
    {
        printf("Couldn't open device\n%s\n", OpenNI::getExtendedError());
        return 2;
    }

    const SensorInfo* pInfoColor = device.getSensorInfo(SENSOR_COLOR);
    if (NULL == pInfoColor)
    {
        printf("Not supported color stream...\n");
        return 4;
    }

    const SensorInfo* pInfoDepth = device.getSensorInfo(SENSOR_DEPTH);
    if (NULL == pInfoDepth)
    {
        printf("Not supported depth stream...\n");
        return 3;
    }

    VideoStream color;
    rc = color.create(device, SENSOR_COLOR);
    if (STATUS_OK != rc)
    {
        printf("Couldn't create color stream\n%s\n", OpenNI::getExtendedError());
        return 7;
    }

    rc = color.start();
    if (STATUS_OK != rc)
    {
        printf("Couldn't start the color stream\n%s\n", OpenNI::getExtendedError());
        return 8;
    }

    VideoStream depth;
    rc = depth.create(device, SENSOR_DEPTH);
    if (STATUS_OK != rc)
    {
        printf("Couldn't create depth stream\n%s\n", OpenNI::getExtendedError());
        return 5;
    }

    rc = depth.start();
    if (STATUS_OK != rc)
    {
        printf("Couldn't start the depth stream\n%s\n", OpenNI::getExtendedError());
        return 6;
    }

    VideoFrameRef frame;
    VideoStream* streams[] = { &color, &depth };
    while (!wasKeyboardHit())
    {
        int readyStream = -1;
        rc = OpenNI::waitForAnyStream(streams, 2, &readyStream, SAMPLE_READ_WAIT_TIMEOUT);
        if (STATUS_OK != rc)
        {
            printf("Wait failed! (timeout is %d ms)\n%s\n", SAMPLE_READ_WAIT_TIMEOUT, OpenNI::getExtendedError());
            break;
        }

        switch (readyStream)
        {
        case 0:
            color.readFrame(&frame);
            break;
        case 1:
            depth.readFrame(&frame);
            break;
        default:
            printf("Unxpected stream\n");
        }

        analyzeFrame(frame);
    }

    depth.stop();
    color.stop();
    depth.destroy();
    color.destroy();
    device.close();
    OpenNI::shutdown();

    return 0;
}
