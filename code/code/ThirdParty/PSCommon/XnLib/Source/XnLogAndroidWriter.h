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
#ifndef __XN_LOG_ANDROID_WRITER_H__
#define __XN_LOG_ANDROID_WRITER_H__

//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include <XnLogWriterBase.h>
#include <XnEvent.h>
#include <XnPlatform.h>


class XnLogAndroidWriter : public XnLogWriterBase
{
public:
	XnLogAndroidWriter():m_logOutput(false),m_logOutputRedirect(false){}

public:
	xnl::Event1Arg<const XnLogEntry*> m_LogRedirectEvent;

	virtual void WriteEntry(const XnLogEntry* pEntry);
	virtual void WriteEntryRedirect(const XnLogEntry* pEntry);
	virtual void WriteUnformatted(const XnChar* strMessage);

	virtual XnStatus RegisterAndroidOutputRedirectCallback(XnLogRedirectEventHandler handler, void* pCookie, XnCallbackHandle* handle);
	virtual void UnRegisterAndroidOutputRedirectCallback(XnCallbackHandle handle);

public:
	XnBool m_logOutput;
	XnBool m_logOutputRedirect;


};

#endif // __XN_LOG_ANDROID_WRITER_H__
