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
//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include <XnPlatform.h>
#if XN_PLATFORM == XN_PLATFORM_ANDROID_ARM
#include "XnLogAndroidWriter.h"
#include <XnLog.h>

#ifdef XN_PLATFORM_ANDROID_OS
	#include <cutils/log.h>
#else
	#include <android/log.h>
    #define  LOGD(x...)  __android_log_print(ANDROID_LOG_INFO,"LogAndroidWrite",x)
    #define  LOGE(x...)  __android_log_print(ANDROID_LOG_ERROR,"LogAndroidWrite",x)
#endif
	
android_LogPriority OpenNISeverityToAndroid(XnLogSeverity nSeverity)
{
	switch (nSeverity)
	{
	case XN_LOG_VERBOSE:
		return ANDROID_LOG_VERBOSE;
	case XN_LOG_INFO:
		return ANDROID_LOG_INFO;
	case XN_LOG_WARNING:
		return ANDROID_LOG_WARN;
	case XN_LOG_ERROR:
		return ANDROID_LOG_ERROR;
	default:
		XN_ASSERT(FALSE);
		return ANDROID_LOG_VERBOSE;
	}
}

//---------------------------------------------------------------------------
// Code
//---------------------------------------------------------------------------
void XnLogAndroidWriter::WriteEntry(const XnLogEntry* pEntry)
{
#ifdef XN_PLATFORM_ANDROID_OS
	ALOGE("OpenNI2: %s\n", pEntry->strMessage);
#else
	if(m_logOutput)
	{
		__android_log_write(OpenNISeverityToAndroid(pEntry->nSeverity), "OpenNI", pEntry->strMessage);
	}

	if(m_logOutputRedirect)
	{
		WriteEntryRedirect(pEntry);
	}
#endif
}

void XnLogAndroidWriter::WriteEntryRedirect(const XnLogEntry* pEntry)
{
	m_LogRedirectEvent.Raise(pEntry);
}

void XnLogAndroidWriter::WriteUnformatted(const XnChar* strMessage)
{
}



XnStatus XnLogAndroidWriter::RegisterAndroidOutputRedirectCallback(XnLogRedirectEventHandler handler, void* pCookie, XnCallbackHandle* handle)
{
	XN_VALIDATE_INPUT_PTR(handler);
    return m_LogRedirectEvent.Register(handler, pCookie, *handle);
}


void XnLogAndroidWriter::UnRegisterAndroidOutputRedirectCallback(XnCallbackHandle handle)
{
	m_LogRedirectEvent.Unregister(handle);
}


#endif
