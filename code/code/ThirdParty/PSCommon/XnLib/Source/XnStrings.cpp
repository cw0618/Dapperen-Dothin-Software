/*****************************************************************************
*                                                                            *
*  PrimeSense PSCommon Library                                               *
*  Copyright (C) 2012 PrimeSense Ltd.                                        *
*                                                                            *
*  This file is part of PSCommon.                                            *
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
#include <ctype.h>
#include "XnLib.h"

XN_C_API XnChar* xnOSStrDup(const XnChar* strSource)
{
    XnSizeT nLen = strlen(strSource);
    ++nLen;

    XnChar* result = (XnChar*)xnOSMalloc(nLen);
    strcpy(result, strSource);
    return result;
}

XN_C_API XnStatus xnOSStrFormat(XnChar* cpDestString, const XnUInt32 nDestLength, XnUInt32* pnCharsWritten, const XnChar* cpFormat, ...)
{
    XnStatus nRetVal = XN_STATUS_OK;

    va_list args;
    va_start(args, cpFormat);

    nRetVal = xnOSStrFormatV(cpDestString, nDestLength, pnCharsWritten, cpFormat, args);
    XN_IS_STATUS_OK(nRetVal);

    va_end(args);

    // All is good...
    return (XN_STATUS_OK);
}

XN_C_API XnChar* xnOSStrToLower(XnChar* strSource)
{
    XnChar* p = strSource;
    while (*p != '\0')
    {
        *p = (XnChar)tolower(*p);
        ++p;
    }

    return strSource;
}

XN_C_API const XnChar* xnOSStrStr(const XnChar* strDst, const XnChar* strSrc)
{
    return strstr(strDst, strSrc);
}
