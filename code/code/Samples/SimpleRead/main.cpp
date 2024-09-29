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

int main(int argc, char *argv[])
{
    Status rc = OpenNI::initialize();
    if (STATUS_OK != rc)
    {
        printf("Initialize failed\n%s\n", OpenNI::getExtendedError());
        return 1;
    }

    Device device;
    if (argc < 2)
        rc = device.open(ANY_DEVICE);
    else
        rc = device.open(argv[1]);

    if (STATUS_OK != rc)
    {
        printf("Couldn't open device\n%s\n", OpenNI::getExtendedError());
        return 2;
    }

    const SensorInfo* pInfo = device.getSensorInfo(SENSOR_DEPTH);
    if (NULL == pInfo)
    {
        printf("Not supported depth stream...\n");
        return 3;
    }

    VideoStream stream;
    rc = stream.create(device, SENSOR_DEPTH);
    if (STATUS_OK != rc)
    {
        printf("Couldn't create depth stream\n%s\n", OpenNI::getExtendedError());
        return 4;
    }

    rc = stream.start();
    if (STATUS_OK != rc)
    {
        printf("Couldn't start the depth stream\n%s\n", OpenNI::getExtendedError());
        return 5;
    }

    VideoFrameRef frame;
    while (!wasKeyboardHit())
    {
        int changedStreamDummy;
        VideoStream* pStream = &stream;
        rc = OpenNI::waitForAnyStream(&pStream, 1, &changedStreamDummy, SAMPLE_READ_WAIT_TIMEOUT);
        if (STATUS_OK != rc)
        {
            printf("Wait failed! (timeout is %d ms)\n%s\n", SAMPLE_READ_WAIT_TIMEOUT, OpenNI::getExtendedError());
            continue;
        }

        rc = stream.readFrame(&frame);
        if (STATUS_OK != rc)
        {
            printf("Read failed!\n%s\n", OpenNI::getExtendedError());
            continue;
        }

        if (PIXEL_FORMAT_DEPTH_1_MM != frame.getVideoMode().getPixelFormat() && PIXEL_FORMAT_DEPTH_100_UM != frame.getVideoMode().getPixelFormat())
        {
            printf("Unexpected frame format\n");
            continue;
        }

        DepthPixel* pDepth = (DepthPixel*)frame.getData();
        int middleIndex = (frame.getHeight() + 1) * frame.getWidth() / 2;
        printf("[%08llu] %8d\n", (long long)frame.getTimestamp(), pDepth[middleIndex]);
    }

    stream.stop();
    stream.destroy();
    device.close();
    OpenNI::shutdown();

    return 0;
}
