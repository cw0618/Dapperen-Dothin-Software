/*****************************************************************************
*									     *
*  OpenNI 2.x Alpha							     *
*  Copyright (C) 2012 PrimeSense Ltd.					     *
*									     *
*  This file is part of OpenNI. 					     *
*									     *
*  Licensed under the Apache License, Version 2.0 (the "License");	     *
*  you may not use this file except in compliance with the License.	     *
*  You may obtain a copy of the License at				     *
*									     *
*      http://www.apache.org/licenses/LICENSE-2.0			     *
*									     *
*  Unless required by applicable law or agreed to in writing, software	     *
*  distributed under the License is distributed on an "AS IS" BASIS,	     *
*  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  *
*  See the License for the specific language governing permissions and	     *
*  limitations under the License.					     *
*									     *
*****************************************************************************/
//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include "XnPacked10DepthProcessor.h"
#include <XnProfiling.h>
#ifdef XN_NEON
#include <arm_neon.h>
#endif

//---------------------------------------------------------------------------
// Defines
//---------------------------------------------------------------------------
/* The size of an input element in the stream. */
#define XN_INPUT_ELEMENT_SIZE 5
/* The size of an output element in the stream. */
#define XN_OUTPUT_ELEMENT_SIZE 8

//---------------------------------------------------------------------------
// Macros
//---------------------------------------------------------------------------
/* Returns a set of <count> bits. For example XN_ON_BITS(4) returns 0xF */
#define XN_ON_BITS(count)				((1 << count)-1)

/* Creates a mask of <count> bits in offset <offset> */
#define XN_CREATE_MASK(count, offset)	(XN_ON_BITS(count) << offset)

/* Takes the <count> bits in offset <offset> from <source>.
*  For example:
*  If we want 3 bits located in offset 2 from 0xF4:
*  11110100
*     ---
*  we get 101, which is 0x5.
*  and so, XN_TAKE_BITS(0xF4,3,2) == 0x5.
*/
#define XN_TAKE_BITS(source, count, offset)		((source & XN_CREATE_MASK(count, offset)) >> offset)

//---------------------------------------------------------------------------
// Code
//---------------------------------------------------------------------------
XnPacked10DepthProcessor::XnPacked10DepthProcessor(XnSensorDepthStream* pStream, XnSensorStreamHelper* pHelper, XnFrameBufferManager* pBufferManager) :
	XnDepthProcessor(pStream, pHelper, pBufferManager)
{
}

XnStatus XnPacked10DepthProcessor::Init()
{
	XnStatus nRetVal = XN_STATUS_OK;

	nRetVal = XnDepthProcessor::Init();
	XN_IS_STATUS_OK(nRetVal);

	XN_VALIDATE_BUFFER_ALLOCATE(m_ContinuousBuffer, XN_INPUT_ELEMENT_SIZE);

	return (XN_STATUS_OK);
}

XnPacked10DepthProcessor::~XnPacked10DepthProcessor()
{
}

XnStatus XnPacked10DepthProcessor::Unpack10to16(const XnUInt8* pcInput, const XnUInt32 nInputSize, XnUInt32* pnActualRead)
{
	const XnUInt8* pOrigInput = pcInput;

	XnUInt32 nElements = nInputSize / XN_INPUT_ELEMENT_SIZE; // floored
	XnUInt32 nNeededOutput = nElements * XN_OUTPUT_ELEMENT_SIZE;

	*pnActualRead = 0;
	XnBuffer* pWriteBuffer = GetWriteBuffer();

	// Check there is enough room for the depth pixels
	if (!CheckWriteBufferForOverflow(nNeededOutput))
	{
		return XN_STATUS_OUTPUT_BUFFER_OVERFLOW;
	}

	XnUInt16* pnOutput = (XnUInt16*)pWriteBuffer->GetUnsafeWritePointer();

	XnUInt16 a0,a1,a2,a3;

	// Convert the 10bit packed data into 16bit shorts
	for (XnUInt32 nElem = 0; nElem < nElements; ++nElem)
	{
		// input:	0,  1,	2,3,  4,
		//			-,---,---,-,---,
		// bits:	8,2,6,4,4,6,2,8
		//			---,---,-----,---
		// output:	  0,  1,    2,	3

		a0 = (XN_TAKE_BITS(pcInput[0],8,0) << 2) | XN_TAKE_BITS(pcInput[1],2,6);

		a1 = (XN_TAKE_BITS(pcInput[1],6,0) << 4) | XN_TAKE_BITS(pcInput[2],4,4);

		a2 = (XN_TAKE_BITS(pcInput[2],4,0) << 6) | (XN_TAKE_BITS(pcInput[3],6,2));

		a3 = (XN_TAKE_BITS(pcInput[3],2,0) << 8) | XN_TAKE_BITS(pcInput[4],8,0);

		pnOutput[0] = GetOutput(a0);
		pnOutput[1] = GetOutput(a1);
		pnOutput[2] = GetOutput(a2);
		pnOutput[3] = GetOutput(a3);

		pcInput += XN_INPUT_ELEMENT_SIZE;
		pnOutput += 4;
	}

	*pnActualRead = (XnUInt32)(pcInput - pOrigInput);
	pWriteBuffer->UnsafeUpdateSize(nNeededOutput);

	return XN_STATUS_OK;
}

void XnPacked10DepthProcessor::ProcessFramePacketChunk(const XnSensorProtocolResponseHeader* /*pHeader*/, const XnUChar* pData, XnUInt32 /*nDataOffset*/, XnUInt32 nDataSize)
{
	XN_PROFILING_START_SECTION("XnPacked10DepthProcessor::ProcessFramePacketChunk")

	XnStatus nRetVal = XN_STATUS_OK;

	// check if we have data from previous packet
	if (m_ContinuousBuffer.GetSize() != 0)
	{
		// fill in to a whole element
		XnUInt32 nReadBytes = XN_MIN(nDataSize, XN_INPUT_ELEMENT_SIZE - m_ContinuousBuffer.GetSize());
		m_ContinuousBuffer.UnsafeWrite(pData, nReadBytes);
		pData += nReadBytes;
		nDataSize -= nReadBytes;

		if (m_ContinuousBuffer.GetSize() == XN_INPUT_ELEMENT_SIZE)
		{
			// process it
			XnUInt32 nActualRead = 0;
			Unpack10to16(m_ContinuousBuffer.GetData(), XN_INPUT_ELEMENT_SIZE, &nActualRead);
			m_ContinuousBuffer.Reset();
		}
	}

	// find out the number of input elements we have
	XnUInt32 nActualRead = 0;
	nRetVal = Unpack10to16(pData, nDataSize, &nActualRead);
	if (nRetVal == XN_STATUS_OK)
	{
		pData += nActualRead;
		nDataSize -= nActualRead;

		// if we have any bytes left, store them for next packet.
		if (nDataSize > 0)
		{
			// no need to check for overflow. there can not be a case in which more than XN_INPUT_ELEMENT_SIZE
			// are left.
			m_ContinuousBuffer.UnsafeWrite(pData, nDataSize);
		}
	}

	CheckIgnoreEOF();// ADD by ZW

	XN_PROFILING_END_SECTION
}

void XnPacked10DepthProcessor::OnStartOfFrame(const XnSensorProtocolResponseHeader* pHeader)
{
	XnDepthProcessor::OnStartOfFrame(pHeader);
	m_ContinuousBuffer.Reset();
}

void XnPacked10DepthProcessor::OnEndOfFrame(const XnSensorProtocolResponseHeader* pHeader)
{
	XnDepthProcessor::OnEndOfFrame(pHeader);
	m_ContinuousBuffer.Reset();
}
