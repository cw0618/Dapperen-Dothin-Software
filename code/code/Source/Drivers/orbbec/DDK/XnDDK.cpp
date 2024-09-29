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
//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include <XnDDK.h>
#include <Formats/XnFormats.h>
#include <XnOS.h>

//---------------------------------------------------------------------------
// Code
//---------------------------------------------------------------------------

XnResolutions XnDDKGetResolutionFromXY(XnUInt32 nXRes, XnUInt32 nYRes)
{
    if (nXRes == 320 && nYRes == 240)     return XN_RESOLUTION_QVGA;
    if (nXRes == 640 && nYRes == 480)     return XN_RESOLUTION_VGA;
    if (nXRes == 1280 && nYRes == 1024)   return XN_RESOLUTION_SXGA;
    if (nXRes == 1600 && nYRes == 1200)   return XN_RESOLUTION_UXGA;
    if (nXRes == 160 && nYRes == 120)     return XN_RESOLUTION_QQVGA;
    if (nXRes == 176 && nYRes == 144)     return XN_RESOLUTION_QCIF;
    if (nXRes == 423 && nYRes == 240)     return XN_RESOLUTION_240P;
    if (nXRes == 352 && nYRes == 288)     return XN_RESOLUTION_CIF;
    if (nXRes == 640 && nYRes == 360)     return XN_RESOLUTION_WVGA;
    if (nXRes == 864 && nYRes == 480)     return XN_RESOLUTION_480P;
    if (nXRes == 800 && nYRes == 600)     return XN_RESOLUTION_SVGA;
    if (nXRes == 1024 && nYRes == 576)    return XN_RESOLUTION_576P;
    if (nXRes == 960 && nYRes == 720)     return XN_RESOLUTION_DV;
    if (nXRes == 1280 && nYRes == 720)    return XN_RESOLUTION_720P;
    // check if this is one of our special resolutions
    if (nXRes == 800 && nYRes == 448)     return XN_RESOLUTION_800_448;
    if (nXRes == 1280 && nYRes == 960)    return XN_RESOLUTION_1280_960;
    // support orbbec Astra 2
    if (nXRes == 320 && nYRes == 200)     return XN_RESOLUTION_320_200;
    if (nXRes == 640 && nYRes == 400)     return XN_RESOLUTION_640_400;
    if (nXRes == 1280 && nYRes == 800)    return XN_RESOLUTION_1280_800;
    if (nXRes == 800 && nYRes == 1280)    return XN_RESOLUTION_800_1280;
    if (nXRes == 400 && nYRes == 640)     return XN_RESOLUTION_400_640;

    if (nXRes == 480 && nYRes == 640)     return XN_RESOLUTION_480_640;
    if (nXRes == 1080 && nYRes == 1280)   return XN_RESOLUTION_1080_1280;
    if (nXRes == 960 && nYRes == 1280)    return XN_RESOLUTION_960_1280;
    if (nXRes == 540 && nYRes == 640)     return XN_RESOLUTION_540_640;
    if (nXRes == 720 && nYRes == 960)     return XN_RESOLUTION_720_960;
    if (nXRes == 1024 && nYRes == 1280)   return XN_RESOLUTION_1024_1280;
    if (nXRes == 1328 && nYRes == 1120)   return XN_RESOLUTION_1328_1120;
    if (nXRes == 384 && nYRes == 640)     return XN_RESOLUTION_384_640;
    if (nXRes == 1280 && nYRes == 1120)   return XN_RESOLUTION_1280_1120;
    if (nXRes == 664 && nYRes == 560)     return XN_RESOLUTION_664_560;
    if (nXRes == 640 && nYRes == 560)     return XN_RESOLUTION_640_560;
    if (nXRes == 960 && nYRes == 1120)    return XN_RESOLUTION_960_1120;
    if (nXRes == 480 && nYRes == 560)     return XN_RESOLUTION_480_560;
    if (nXRes == 800 && nYRes == 450)     return XN_RESOLUTION_800_450;
    if (nXRes == 1920 && nYRes == 1080)   return XN_RESOLUTION_1920_1080;
    if (nXRes == 2560 && nYRes == 1440)   return XN_RESOLUTION_2560_1440;
    if (nXRes == 3840 && nYRes == 2160)   return XN_RESOLUTION_3840_2160;
    if (nXRes == 1920 && nYRes == 480)    return XN_RESOLUTION_1920_480;
    if (nXRes == 640 && nYRes == 960)    return XN_RESOLUTION_640_960;
    if (nXRes == 640 && nYRes == 600)    return XN_RESOLUTION_640_600;
    if (nXRes == 960 && nYRes == 240)    return XN_RESOLUTION_960_240;
	if (nXRes == 720 && nYRes == 180)    return XN_RESOLUTION_720_180;
	if (nXRes == 360 && nYRes == 90)    return XN_RESOLUTION_360_90;
	if (nXRes == 240 && nYRes == 180)    return XN_RESOLUTION_240_180;
	if (nXRes == 120 && nYRes == 90)    return XN_RESOLUTION_120_90;
	if (nXRes == 192 && nYRes == 45)    return XN_RESOLUTION_192_45;
	if (nXRes == 64 && nYRes == 45)    return XN_RESOLUTION_64_45;
	if (nXRes == 96 && nYRes == 20)    return XN_RESOLUTION_96_20;
	if (nXRes == 32 && nYRes == 20)    return XN_RESOLUTION_32_20;
	if (nXRes == 528 && nYRes == 140)    return XN_RESOLUTION_528_140;
	if (nXRes == 176 && nYRes == 140)    return XN_RESOLUTION_176_140;

    return XN_RESOLUTION_CUSTOM;
}

XnBool XnDDKGetXYFromResolution(XnResolutions res, XnUInt32* pnXRes, XnUInt32* pnYRes)
{
    switch (res)
    {
    case XN_RESOLUTION_QVGA: *pnXRes = 320; *pnYRes = 240; break;
    case XN_RESOLUTION_VGA: *pnXRes = 640; *pnYRes = 480; break;
    case XN_RESOLUTION_SXGA: *pnXRes = 1280; *pnYRes = 1024; break;
    case XN_RESOLUTION_UXGA: *pnXRes = 1600; *pnYRes = 1200; break;
    case XN_RESOLUTION_QQVGA: *pnXRes = 160; *pnYRes = 120; break;
    case XN_RESOLUTION_QCIF: *pnXRes = 176; *pnYRes = 144; break;
    case XN_RESOLUTION_240P: *pnXRes = 423; *pnYRes = 240; break;
    case XN_RESOLUTION_CIF: *pnXRes = 352; *pnYRes = 288; break;
    case XN_RESOLUTION_WVGA: *pnXRes = 640; *pnYRes = 360; break;
    case XN_RESOLUTION_480P: *pnXRes = 864; *pnYRes = 480; break;
    case XN_RESOLUTION_SVGA: *pnXRes = 800; *pnYRes = 600; break;
    case XN_RESOLUTION_576P: *pnXRes = 1024; *pnYRes = 576; break;
    case XN_RESOLUTION_DV: *pnXRes = 960; *pnYRes = 720; break;
    case XN_RESOLUTION_720P: *pnXRes = 1280; *pnYRes = 720; break;
        // check if this is one of our special resolutions
    case XN_RESOLUTION_800_448: *pnXRes = 800; *pnYRes = 448; break;
    case XN_RESOLUTION_1280_960: *pnXRes = 1280; *pnYRes = 960;	break;
        // support orbbec Astra 2
    case XN_RESOLUTION_320_200: *pnXRes = 320; *pnYRes = 200; break;
    case XN_RESOLUTION_640_400: *pnXRes = 640; *pnYRes = 400; break;
    case XN_RESOLUTION_1280_800: *pnXRes = 1280; *pnYRes = 800;	break;
    case XN_RESOLUTION_320_180: *pnXRes = 320; *pnYRes = 180; break;
    case XN_RESOLUTION_160_90: *pnXRes = 160; *pnYRes = 90;	break;
    case XN_RESOLUTION_800_1280: *pnXRes = 800; *pnYRes = 1280;	break;
    case XN_RESOLUTION_400_640: *pnXRes = 400; *pnYRes = 640; break;

    case XN_RESOLUTION_480_640: *pnXRes = 480; *pnYRes = 640; break;
    case XN_RESOLUTION_1080_1280: *pnXRes = 1080; *pnYRes = 1280; break;
    case XN_RESOLUTION_960_1280: *pnXRes = 960; *pnYRes = 1280;	break;
    case XN_RESOLUTION_540_640: *pnXRes = 540; *pnYRes = 640; break;
    case XN_RESOLUTION_720_960: *pnXRes = 720; *pnYRes = 960; break;
    case XN_RESOLUTION_1024_1280: *pnXRes = 1024; *pnYRes = 1280; break;
    case XN_RESOLUTION_1328_1120: *pnXRes = 1328; *pnYRes = 1120; break;
    case XN_RESOLUTION_384_640: *pnXRes = 384; *pnYRes = 640; break;
    case XN_RESOLUTION_1280_1120: *pnXRes = 1280; *pnYRes = 1120; break;
    case XN_RESOLUTION_664_560: *pnXRes = 664; *pnYRes = 560; break;
    case XN_RESOLUTION_640_560: *pnXRes = 640; *pnYRes = 560; break;
    case XN_RESOLUTION_960_1120: *pnXRes = 960; *pnYRes = 1120;	break;
    case XN_RESOLUTION_480_560: *pnXRes = 480; *pnYRes = 560; break;
    case XN_RESOLUTION_800_450: *pnXRes = 800; *pnYRes = 450; break;
    case XN_RESOLUTION_1920_1080: *pnXRes = 1920; *pnYRes = 1080; break;
    case XN_RESOLUTION_2560_1440: *pnXRes = 2560; *pnYRes = 1440; break;
    case XN_RESOLUTION_3840_2160: *pnXRes = 3840; *pnYRes = 2160; break;
    case XN_RESOLUTION_1920_480: *pnXRes = 1920; *pnYRes = 480; break;
    case XN_RESOLUTION_640_960: *pnXRes = 640; *pnYRes = 960; break;
    case XN_RESOLUTION_640_600: *pnXRes = 640; *pnYRes = 600; break;
    case XN_RESOLUTION_960_240: *pnXRes = 960; *pnYRes = 240; break;
	case XN_RESOLUTION_720_180: *pnXRes = 720; *pnYRes = 180; break;
	case XN_RESOLUTION_360_90: *pnXRes = 360; *pnYRes = 90; break;
	case XN_RESOLUTION_240_180: *pnXRes = 240; *pnYRes = 180; break;
	case XN_RESOLUTION_120_90: *pnXRes = 120; *pnYRes = 90; break;
	case XN_RESOLUTION_192_45: *pnXRes = 192; *pnYRes = 45; break;
	case XN_RESOLUTION_64_45: *pnXRes = 64; *pnYRes = 45; break;
	case XN_RESOLUTION_96_20: *pnXRes = 96; *pnYRes = 20; break;
	case XN_RESOLUTION_32_20: *pnXRes = 32; *pnYRes = 20; break;
	case XN_RESOLUTION_528_140: *pnXRes = 528; *pnYRes = 140; break;
	case XN_RESOLUTION_176_140: *pnXRes = 176; *pnYRes = 140; break;
    case XN_RESOLUTION_CUSTOM: return FALSE;
    }

    return TRUE;
}

const XnChar* XnDDKGetResolutionName(XnResolutions res)
{
    switch (res)
    {
    case XN_RESOLUTION_QVGA: return "QVGA";
    case XN_RESOLUTION_VGA: return "VGA";
    case XN_RESOLUTION_SXGA: return "SXGA";
    case XN_RESOLUTION_UXGA: return "UXGA";
    case XN_RESOLUTION_QQVGA: return "QQVGA";
    case XN_RESOLUTION_QCIF: return "QCIF";
    case XN_RESOLUTION_240P: return "240P";
    case XN_RESOLUTION_CIF: return "CIF";
    case XN_RESOLUTION_WVGA: return "WVGA";
    case XN_RESOLUTION_480P: return "480P";
    case XN_RESOLUTION_SVGA: return "SVGA";
    case XN_RESOLUTION_576P: return "576P";
    case XN_RESOLUTION_DV: return "DV";
    case XN_RESOLUTION_720P: return "720P";
        // check if this is one of our special resolutions
    case XN_RESOLUTION_CUSTOM: return "Custom";
    case XN_RESOLUTION_800_448: return "800x448";
    case XN_RESOLUTION_1280_960: return "1280x960";
        //support orbbec Astra 2
    case XN_RESOLUTION_640_400: return "640x400";
    case XN_RESOLUTION_1280_800: return "1280x800";
    case XN_RESOLUTION_320_200: return "320x200";
    case XN_RESOLUTION_800_1280: return "800x1280";
    case XN_RESOLUTION_400_640: return "400x640";

    case XN_RESOLUTION_480_640: return "480x640";
    case XN_RESOLUTION_1080_1280: return "1080x1280";
    case XN_RESOLUTION_960_1280: return "960x1280";
    case XN_RESOLUTION_540_640: return "540x640";
    case XN_RESOLUTION_720_960: return "720x960";
    case XN_RESOLUTION_1024_1280: return "1024x1280";
    case XN_RESOLUTION_1328_1120: return "1328x1120";
    case XN_RESOLUTION_384_640: return "384x640";
    case XN_RESOLUTION_1280_1120: return "1280x1120";
    case XN_RESOLUTION_664_560: return "664x560";
    case XN_RESOLUTION_640_560: return "640x560";
    case XN_RESOLUTION_960_1120: return "960x1120";
    case XN_RESOLUTION_480_560: return "480x560";
    case XN_RESOLUTION_800_450: return "800x450";
    case XN_RESOLUTION_1920_1080: return "1920x1080";
    case XN_RESOLUTION_2560_1440: return "2560x1440";
    case XN_RESOLUTION_3840_2160: return "3840x2160";
    case XN_RESOLUTION_1920_480: return "1920x480";
    case XN_RESOLUTION_640_960: return "640x960";
    case XN_RESOLUTION_640_600: return "640x600";
    case XN_RESOLUTION_960_240: return "960x240";
	case XN_RESOLUTION_720_180: return "720x180";
	case XN_RESOLUTION_360_90: return "360x90";
	case XN_RESOLUTION_240_180: return "240x180";
	case XN_RESOLUTION_120_90: return "120x90";
	case XN_RESOLUTION_192_45: return "192x45";
	case XN_RESOLUTION_64_45: return "64x45";
	case XN_RESOLUTION_96_20: return "96x20";
	case XN_RESOLUTION_32_20: return "32x20";
	case XN_RESOLUTION_528_140: return "528x140";
	case XN_RESOLUTION_176_140: return "176x140";
    default:
        XN_ASSERT(FALSE);
        return "Custom";
    }
}
