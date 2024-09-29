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
#include <XnOS.h>
#include <XnLog.h>
#include <XnJpeg.h>
#include <jerror.h>
#include <jpeglib.h>
#include <setjmp.h>

//---------------------------------------------------------------------------
// Definitions
//---------------------------------------------------------------------------
#define XN_MASK_JPEG "JPEG"
#define XN_STREAM_STRING_BAD_FORMAT -1

//---------------------------------------------------------------------------
// Types
//---------------------------------------------------------------------------
XN_PRAGMA_START_DISABLED_WARNING_SECTION(XN_STRUCT_PADDED_WARNING_ID);
typedef struct XnLibJpegErrorMgr
{
	struct jpeg_error_mgr pub;

	jmp_buf setjmpBuffer;
} XnLibJpegErrorMgr; 
XN_PRAGMA_STOP_DISABLED_WARNING_SECTION;

typedef struct XnStreamCompJPEGContext
{
	jpeg_compress_struct		jCompStruct;
	jpeg_error_mgr				jErrMgr;
	struct jpeg_destination_mgr	jDestMgr;
} XnStreamCompJPEGContext;

typedef struct XnStreamUncompJPEGContext
{
	jpeg_decompress_struct	jDecompStruct;
	XnLibJpegErrorMgr		jErrMgr;
	struct jpeg_source_mgr	jSrcMgr;
} XnStreamUncompJPEGContext;

void XnStreamJPEGDecompSkipFunction(struct jpeg_decompress_struct* pjDecompStruct, long nNumBytes)
{
	// Skip bytes in the internal buffer
	pjDecompStruct->src->next_input_byte += (size_t)nNumBytes;
	pjDecompStruct->src->bytes_in_buffer -= (size_t)nNumBytes;
}

boolean XnStreamJPEGDecompDummyFailFunction(struct jpeg_decompress_struct* /*pjDecompStruct*/)
{
	// If we ever got to the point we need to allocate more memory, something is wrong!
	return (FALSE);
}

void XnStreamJPEGDecompDummyFunction(struct jpeg_decompress_struct* /*pjDecompStruct*/)
{
	// Dummy libjpeg function to wrap internal buffers usage...
}

void XnStreamJPEGDummyErrorExit(j_common_ptr cinfo)
{
	XnLibJpegErrorMgr* errMgr = (XnLibJpegErrorMgr*)cinfo->err; 

	longjmp(errMgr->setjmpBuffer, 1); 
}

void  XnStreamJPEGCompDummyFunction(struct jpeg_compress_struct* /*pjCompStruct*/)
{
	// Dummy libjpeg function to wrap internal buffers usage...
}

boolean XnStreamJPEGCompDummyFailFunction(struct jpeg_compress_struct* /*pjCompStruct*/)
{
	// If we ever got to the point we need to allocate more memory, something is wrong!
	return (FALSE);
}

XnStatus XnStreamFreeCompressImageJ(XnStreamCompJPEGContext** ppStreamCompJPEGContext)
{
	// Validate the input/output pointers (to make sure none of them is NULL)
    XN_VALIDATE_INPUT_PTR(ppStreamCompJPEGContext);

    if (NULL == *ppStreamCompJPEGContext)
        return XN_STATUS_OK; // Already NULL. Nothing to do.

    jpeg_destroy_compress(&(*ppStreamCompJPEGContext)->jCompStruct);

    XN_DELETE(*ppStreamCompJPEGContext);

    *ppStreamCompJPEGContext = NULL;

	// All is good...
	return (XN_STATUS_OK);
}

XnStatus XnStreamCompressImage8J(XnStreamCompJPEGContext** ppStreamCompJPEGContext, const XnUInt8* pInput, XnUInt8* pOutput, XnUInt32* pnOutputSize, const XnUInt32 nXRes, const XnUInt32 nYRes, const XnUInt32 nQuality)
{
	// Local function variables
	XnUInt8* pCurrScanline = (XnUInt8*)pInput;
	XnUInt32 nYIndex = 0;
	jpeg_compress_struct* pjCompStruct = NULL;	

	// Validate the input/output pointers (to make sure none of them is NULL)
    XN_VALIDATE_INPUT_PTR( ppStreamCompJPEGContext);
    XN_VALIDATE_INPUT_PTR(*ppStreamCompJPEGContext);
	XN_VALIDATE_INPUT_PTR(pInput);
	XN_VALIDATE_OUTPUT_PTR(pOutput);
	XN_VALIDATE_OUTPUT_PTR(pnOutputSize);

    pjCompStruct = &(*ppStreamCompJPEGContext)->jCompStruct;

	pjCompStruct->in_color_space = JCS_GRAYSCALE;
	jpeg_set_defaults(pjCompStruct);
	pjCompStruct->input_components = 1;
	pjCompStruct->num_components = 1;
	pjCompStruct->image_width = nXRes;
	pjCompStruct->image_height = nYRes;
	pjCompStruct->data_precision = 8;
	pjCompStruct->input_gamma = 1.0;

	jpeg_set_quality(pjCompStruct, nQuality, FALSE);

	pjCompStruct->dest->next_output_byte = (JOCTET*)pOutput;
	pjCompStruct->dest->free_in_buffer = *pnOutputSize;

	jpeg_start_compress(pjCompStruct, TRUE);

	for (nYIndex = 0; nYIndex < nYRes; nYIndex++)
	{
		jpeg_write_scanlines(pjCompStruct, &pCurrScanline, 1);

		pCurrScanline += nXRes;
	}

	jpeg_finish_compress(pjCompStruct);

	*pnOutputSize -= (XnUInt32)pjCompStruct->dest->free_in_buffer;

	// All is good...
	return (XN_STATUS_OK);
}

XnStatus XnStreamCompressImage24J(XnStreamCompJPEGContext** ppStreamCompJPEGContext, const XnUInt8* pInput, XnUInt8* pOutput, XnUInt32* pnOutputSize, const XnUInt32 nXRes, const XnUInt32 nYRes, const XnUInt32 nQuality)
{
	// Local function variables
	XnUInt8* pCurrScanline = (XnUChar*)pInput;
	XnUInt32 nYIndex = 0;
	XnUInt32 nScanLineSize = 0;
	jpeg_compress_struct* pjCompStruct = NULL;	

	// Validate the input/output pointers (to make sure none of them is NULL)
    XN_VALIDATE_INPUT_PTR( ppStreamCompJPEGContext);
    XN_VALIDATE_INPUT_PTR(*ppStreamCompJPEGContext);
	XN_VALIDATE_INPUT_PTR(pInput);
	XN_VALIDATE_OUTPUT_PTR(pOutput);
	XN_VALIDATE_OUTPUT_PTR(pnOutputSize);

    pjCompStruct = &(*ppStreamCompJPEGContext)->jCompStruct;

	pjCompStruct->in_color_space = JCS_RGB;
	jpeg_set_defaults(pjCompStruct);
	pjCompStruct->input_components = 3;
	pjCompStruct->num_components = 3;
	pjCompStruct->image_width = nXRes;
	pjCompStruct->image_height = nYRes;
	pjCompStruct->data_precision = 8;
	pjCompStruct->input_gamma = 1.0;

	jpeg_set_quality(pjCompStruct, nQuality, FALSE);

	pjCompStruct->dest->next_output_byte = (JOCTET*)pOutput;
	pjCompStruct->dest->free_in_buffer = *pnOutputSize;

	jpeg_start_compress(pjCompStruct, TRUE);

	nScanLineSize = nXRes * 3;
	for (nYIndex = 0; nYIndex < nYRes; nYIndex++)
	{
		jpeg_write_scanlines(pjCompStruct, &pCurrScanline, 1);

		pCurrScanline += nScanLineSize;
	}

	jpeg_finish_compress(pjCompStruct);

	*pnOutputSize -= (XnUInt32)pjCompStruct->dest->free_in_buffer;

	// All is good...
	return (XN_STATUS_OK);
}

void XnStreamJPEGOutputMessage(j_common_ptr cinfo)
{
	struct jpeg_error_mgr* err = cinfo->err;
	int msg_code = err->msg_code;
	if (msg_code == JWRN_EXTRANEOUS_DATA)
	{
		// NOTE: we are aware this problem occurs. Log a warning every once in a while
		static XnUInt32 nTimes = 0;
		if (++nTimes == 50)
		{
			char buffer[JMSG_LENGTH_MAX];

			/* Create the message */
			(*cinfo->err->format_message) (cinfo, buffer);

			//Temporary disabled this error since it happens all the time and it's a known issue.
			//xnLogWarning(XN_MASK_JPEG, "JPEG: The following warning occurred 50 times: %s", buffer);
			nTimes = 0;
		}
	}
	else
	{
		char buffer[JMSG_LENGTH_MAX];

		/* Create the message */
		(*cinfo->err->format_message) (cinfo, buffer);

		xnLogWarning(XN_MASK_JPEG, "JPEG: %s", buffer);
	}
}

XnStatus XnStreamInitCompressImageJ(XnStreamCompJPEGContext** ppStreamCompJPEGContext)
{
	// Validate the input/output pointers (to make sure none of them is NULL)
    XN_VALIDATE_OUTPUT_PTR(ppStreamCompJPEGContext);

    // Destroy the previous context if necessary.
    XnStreamFreeCompressImageJ(ppStreamCompJPEGContext);

    // Allocate the new context.
    XnStreamCompJPEGContext* pStreamCompJPEGContext = XN_NEW(XnStreamCompJPEGContext);

	if (NULL == pStreamCompJPEGContext)
		return XN_STATUS_ERROR;
 
 	pStreamCompJPEGContext->jCompStruct.err = jpeg_std_error(&pStreamCompJPEGContext->jErrMgr);
 
 	jpeg_create_compress(&pStreamCompJPEGContext->jCompStruct);
 
	pStreamCompJPEGContext->jCompStruct.dest = &pStreamCompJPEGContext->jDestMgr;
 	pStreamCompJPEGContext->jCompStruct.dest->init_destination = XnStreamJPEGCompDummyFunction;
 	pStreamCompJPEGContext->jCompStruct.dest->empty_output_buffer = XnStreamJPEGCompDummyFailFunction;
 	pStreamCompJPEGContext->jCompStruct.dest->term_destination = XnStreamJPEGCompDummyFunction;

    // Update the output context pointer.
	*ppStreamCompJPEGContext = pStreamCompJPEGContext;

	// All is good...
	return (XN_STATUS_OK);
}

XnStatus XnStreamInitUncompressImageJ(XnStreamUncompJPEGContext** ppStreamUncompJPEGContext)
{
	// Validate the input/output pointers (to make sure none of them is NULL)
    XN_VALIDATE_OUTPUT_PTR(ppStreamUncompJPEGContext);

    // Destroy the previous context if necessary.
    XnStreamFreeUncompressImageJ(ppStreamUncompJPEGContext);

    // Allocate the new context.
    XnStreamUncompJPEGContext* pStreamUncompJPEGContext = XN_NEW(XnStreamUncompJPEGContext);

	if (NULL == pStreamUncompJPEGContext)
		return XN_STATUS_ERROR;

	pStreamUncompJPEGContext->jDecompStruct.err = jpeg_std_error(&pStreamUncompJPEGContext->jErrMgr.pub);
	pStreamUncompJPEGContext->jErrMgr.pub.output_message = XnStreamJPEGOutputMessage;
 	pStreamUncompJPEGContext->jErrMgr.pub.error_exit = XnStreamJPEGDummyErrorExit;

	jpeg_create_decompress(&pStreamUncompJPEGContext->jDecompStruct);
 
	pStreamUncompJPEGContext->jDecompStruct.src = &pStreamUncompJPEGContext->jSrcMgr;
	pStreamUncompJPEGContext->jDecompStruct.src->init_source = XnStreamJPEGDecompDummyFunction;
	pStreamUncompJPEGContext->jDecompStruct.src->fill_input_buffer = XnStreamJPEGDecompDummyFailFunction;
	pStreamUncompJPEGContext->jDecompStruct.src->skip_input_data = XnStreamJPEGDecompSkipFunction;
	pStreamUncompJPEGContext->jDecompStruct.src->resync_to_restart = jpeg_resync_to_restart;
	pStreamUncompJPEGContext->jDecompStruct.src->term_source = XnStreamJPEGDecompDummyFunction;

    // Update the output context pointer.
	*ppStreamUncompJPEGContext = pStreamUncompJPEGContext;

	// All is good...
	return (XN_STATUS_OK);
}

XnStatus XnStreamFreeUncompressImageJ(XnStreamUncompJPEGContext** ppStreamUncompJPEGContext)
{
	// Validate the input/output pointers (to make sure none of them is NULL)
    XN_VALIDATE_INPUT_PTR(ppStreamUncompJPEGContext);

    if (NULL == *ppStreamUncompJPEGContext)
        return XN_STATUS_OK; // Already NULL. Nothing to do.

    jpeg_destroy_decompress(&(*ppStreamUncompJPEGContext)->jDecompStruct);

    XN_DELETE(*ppStreamUncompJPEGContext);

    ppStreamUncompJPEGContext = NULL;

	// All is good...
	return (XN_STATUS_OK);
}

// to allow the use of setjmp
#if (XN_PLATFORM == XN_PLATFORM_WIN32)
#pragma warning(push)
#pragma warning(disable: 4611)
#endif

XnStatus XnStreamUncompressImageJ(XnStreamUncompJPEGContext** ppStreamUncompJPEGContext, const XnUInt8* pInput, const XnUInt32 nInputSize, XnUInt8* pOutput, XnUInt32* pnOutputSize)
{
	// Local function variables
	XnUInt8* pCurrScanline = pOutput;
	XnUInt8* pNextScanline = NULL;
	XnUInt8* pOutputEnd = 0;
	XnUInt32 nScanLineSize = 0;
	XnUInt32 nOutputSize = 0;
	jpeg_decompress_struct* pjDecompStruct = NULL;

	// Validate the input/output pointers (to make sure none of them is NULL)
    XN_VALIDATE_INPUT_PTR(ppStreamUncompJPEGContext);
    XN_VALIDATE_INPUT_PTR(*ppStreamUncompJPEGContext);
	XN_VALIDATE_INPUT_PTR(pInput);
	XN_VALIDATE_OUTPUT_PTR(pOutput);
	XN_VALIDATE_OUTPUT_PTR(pnOutputSize);

	if (nInputSize == 0)
	{
		xnLogError(XN_MASK_JPEG, "The compressed input buffer is too small to be valid!");
		return (XN_STATUS_INPUT_BUFFER_OVERFLOW);
	}

	pOutputEnd = pOutput + *pnOutputSize;

    pjDecompStruct = &(*ppStreamUncompJPEGContext)->jDecompStruct;

	pjDecompStruct->src->bytes_in_buffer = nInputSize;
	pjDecompStruct->src->next_input_byte = pInput;

    if (setjmp((*ppStreamUncompJPEGContext)->jErrMgr.setjmpBuffer))
	{
		//If we get here, the JPEG code has signaled an error.
        XnStreamFreeUncompressImageJ(ppStreamUncompJPEGContext);
        XnStreamInitUncompressImageJ(ppStreamUncompJPEGContext);

		*pnOutputSize = 0;
		xnLogError(XN_MASK_JPEG, "Xiron I/O decompression failed!");
		return (XN_STATUS_ERROR);
	} 

	jpeg_read_header(pjDecompStruct, TRUE);

	jpeg_start_decompress(pjDecompStruct);

	nScanLineSize = pjDecompStruct->output_width * pjDecompStruct->num_components;

	nOutputSize = pjDecompStruct->output_height * nScanLineSize;
	if (nOutputSize > *pnOutputSize)
	{
        XnStreamFreeUncompressImageJ(ppStreamUncompJPEGContext);
        XnStreamInitUncompressImageJ(ppStreamUncompJPEGContext);

		*pnOutputSize = 0;

		return (XN_STATUS_OUTPUT_BUFFER_OVERFLOW);
	}

    while ((*ppStreamUncompJPEGContext)->jDecompStruct.output_scanline < (*ppStreamUncompJPEGContext)->jDecompStruct.output_height)
	{
		pNextScanline = pCurrScanline+nScanLineSize;

		if (pNextScanline > pOutputEnd)
		{
            XnStreamFreeUncompressImageJ(ppStreamUncompJPEGContext);
            XnStreamInitUncompressImageJ(ppStreamUncompJPEGContext);

			*pnOutputSize = 0;

			return (XN_STATUS_OUTPUT_BUFFER_OVERFLOW);
		}

		jpeg_read_scanlines(pjDecompStruct, &pCurrScanline, 1);
		pCurrScanline = pNextScanline;
	}

	jpeg_finish_decompress(pjDecompStruct);

	*pnOutputSize = nOutputSize;

	// All is good...
	return (XN_STATUS_OK);
}

/* ISO/IEC 10918-1:1993(E) K.3.3. Default Huffman tables used by MJPEG UVC devices
which don't specify a Huffman table in the JPEG stream. */
static const unsigned char dc_lumi_len[] = {
	0, 0, 1, 5, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0 };
static const unsigned char dc_lumi_val[] = {
	0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 };

static const unsigned char dc_chromi_len[] = {
	0, 0, 3, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0 };
static const unsigned char dc_chromi_val[] = {
	0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 };

static const unsigned char ac_lumi_len[] = {
	0, 0, 2, 1, 3, 3, 2, 4, 3, 5, 5, 4, 4, 0, 0, 1, 0x7d };
static const unsigned char ac_lumi_val[] = {
	0x01, 0x02, 0x03, 0x00, 0x04, 0x11, 0x05, 0x12,
	0x21, 0x31, 0x41, 0x06, 0x13, 0x51, 0x61, 0x07,
	0x22, 0x71, 0x14, 0x32, 0x81, 0x91, 0xa1, 0x08,
	0x23, 0x42, 0xb1, 0xc1, 0x15, 0x52, 0xd1, 0xf0,
	0x24, 0x33, 0x62, 0x72, 0x82, 0x09, 0x0a, 0x16,
	0x17, 0x18, 0x19, 0x1a, 0x25, 0x26, 0x27, 0x28,
	0x29, 0x2a, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39,
	0x3a, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49,
	0x4a, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59,
	0x5a, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69,
	0x6a, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79,
	0x7a, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89,
	0x8a, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98,
	0x99, 0x9a, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7,
	0xa8, 0xa9, 0xaa, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6,
	0xb7, 0xb8, 0xb9, 0xba, 0xc2, 0xc3, 0xc4, 0xc5,
	0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xd2, 0xd3, 0xd4,
	0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda, 0xe1, 0xe2,
	0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea,
	0xf1, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8,
	0xf9, 0xfa
};
static const unsigned char ac_chromi_len[] = {
	0, 0, 2, 1, 2, 4, 4, 3, 4, 7, 5, 4, 4, 0, 1, 2, 0x77 };
static const unsigned char ac_chromi_val[] = {
	0x00, 0x01, 0x02, 0x03, 0x11, 0x04, 0x05, 0x21,
	0x31, 0x06, 0x12, 0x41, 0x51, 0x07, 0x61, 0x71,
	0x13, 0x22, 0x32, 0x81, 0x08, 0x14, 0x42, 0x91,
	0xa1, 0xb1, 0xc1, 0x09, 0x23, 0x33, 0x52, 0xf0,
	0x15, 0x62, 0x72, 0xd1, 0x0a, 0x16, 0x24, 0x34,
	0xe1, 0x25, 0xf1, 0x17, 0x18, 0x19, 0x1a, 0x26,
	0x27, 0x28, 0x29, 0x2a, 0x35, 0x36, 0x37, 0x38,
	0x39, 0x3a, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48,
	0x49, 0x4a, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58,
	0x59, 0x5a, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68,
	0x69, 0x6a, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78,
	0x79, 0x7a, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87,
	0x88, 0x89, 0x8a, 0x92, 0x93, 0x94, 0x95, 0x96,
	0x97, 0x98, 0x99, 0x9a, 0xa2, 0xa3, 0xa4, 0xa5,
	0xa6, 0xa7, 0xa8, 0xa9, 0xaa, 0xb2, 0xb3, 0xb4,
	0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xc2, 0xc3,
	0xc4, 0xc5, 0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xd2,
	0xd3, 0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda,
	0xe2, 0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9,
	0xea, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8,
	0xf9, 0xfa
};

#define COPY_HUFF_TABLE(dinfo,tbl,name) do { \
	if (dinfo->tbl == NULL) dinfo->tbl = jpeg_alloc_huff_table((j_common_ptr)dinfo); \
		memcpy(dinfo->tbl->bits, name##_len, sizeof(name##_len)); \
		memset(dinfo->tbl->huffval, 0, sizeof(dinfo->tbl->huffval)); \
		memcpy(dinfo->tbl->huffval, name##_val, sizeof(name##_val)); \
	} while(0)


static inline void insert_huff_tables(j_decompress_ptr dinfo) {
	COPY_HUFF_TABLE(dinfo, dc_huff_tbl_ptrs[0], dc_lumi);
	COPY_HUFF_TABLE(dinfo, dc_huff_tbl_ptrs[1], dc_chromi);
	COPY_HUFF_TABLE(dinfo, ac_huff_tbl_ptrs[0], ac_lumi);
	COPY_HUFF_TABLE(dinfo, ac_huff_tbl_ptrs[1], ac_chromi);
}

#define YCbCr_YUYV_2(YCbCr, yuyv) \
	{ \
		*(yuyv++) = *(YCbCr+0); \
		*(yuyv++) = (*(YCbCr+1) + *(YCbCr+4)) >> 1; \
		*(yuyv++) = *(YCbCr+3); \
		*(yuyv++) = (*(YCbCr+2) + *(YCbCr+5)) >> 1; \
	}

#define MAX_READLINE 8

XnStatus XnStreamUncompressImjpegToYuyv(XnStreamUncompJPEGContext** ppStreamUncompJPEGContext, const XnUInt8* pInput, const XnUInt32 nInputSize, XnUInt8* pOutput, XnUInt32* pnOutputSize, XnUInt32 XRes, XnUInt32 YRes)
{
	XN_VALIDATE_INPUT_PTR(ppStreamUncompJPEGContext);
	XN_VALIDATE_INPUT_PTR(*ppStreamUncompJPEGContext);
	XN_VALIDATE_INPUT_PTR(pInput);
	XN_VALIDATE_INPUT_PTR(pnOutputSize);
	XN_VALIDATE_OUTPUT_PTR(pOutput);

	if (nInputSize == 0)
	{
		xnLogError(XN_MASK_JPEG, "The compressed input buffer is too small to be valid!");
		return (XN_STATUS_ERROR);
	}

	jpeg_decompress_struct* pjDecompStruct = NULL;
	XnUInt32 nScanLineSize = 0;
	XnUInt32 nOutputSize = 0;
	XnUInt32 lines_read = 0;
	XnUInt32 i, j;
	XnUInt32 num_scanlines;
	register XnUInt8 *yuyv, *ycbcr;

	pjDecompStruct = &(*ppStreamUncompJPEGContext)->jDecompStruct;
	//jpeg_error_mgr	jErrMgr = (*ppStreamUncompJPEGContext)->jErrMgr.pub;

	if (setjmp((*ppStreamUncompJPEGContext)->jErrMgr.setjmpBuffer))
	{
		//If we get here, the JPEG code has signaled an error.
		XnStreamFreeUncompressImageJ(ppStreamUncompJPEGContext);
		XnStreamInitUncompressImageJ(ppStreamUncompJPEGContext);

		*pnOutputSize = 0;
		xnLogError(XN_MASK_JPEG, "Xiron I/O decompression failed!");
		return (XN_STATUS_ERROR);
	}

	pjDecompStruct->src->bytes_in_buffer = nInputSize;
	pjDecompStruct->src->next_input_byte = pInput;

	//Read JPEG header information
	jpeg_read_header(pjDecompStruct, TRUE);

	if (pjDecompStruct->dc_huff_tbl_ptrs[0] == NULL) {
		/* This frame is missing the Huffman tables: fill in the standard ones */
		insert_huff_tables(pjDecompStruct);
	}

	pjDecompStruct->out_color_space = JCS_YCbCr;
	pjDecompStruct->dct_method = JDCT_IFAST;

	//Start decompression
	jpeg_start_decompress(pjDecompStruct);

	//Data length of a row
	nScanLineSize = pjDecompStruct->output_width * pjDecompStruct->output_components;

	// allocate buffer
	register JSAMPARRAY buffer = (pjDecompStruct->mem->alloc_sarray)
		((j_common_ptr)pjDecompStruct, JPOOL_IMAGE, nScanLineSize, MAX_READLINE);

	// local copy
	XnUInt8 *data = pOutput;
	const XnUInt32 out_step = XRes * 2;
#if 0
	while (pjDecompStruct->output_scanline < pjDecompStruct->output_height) {
		// convert lines of mjpeg data to YCbCr
		num_scanlines = jpeg_read_scanlines(pjDecompStruct, buffer, MAX_READLINE);
		// convert YCbCr to yuyv(YUV422)
		for (j = 0; j < num_scanlines; j++) {
			yuyv = data + (lines_read + j) * out_step;
			ycbcr = buffer[j];
			for (i = 0; i < nScanLineSize; i += 24) {	// step by YCbCr x 8 pixels = 3 x 8 bytes
				YCbCr_YUYV_2(ycbcr + i, yuyv);
				YCbCr_YUYV_2(ycbcr + i + 6, yuyv);
				YCbCr_YUYV_2(ycbcr + i + 12, yuyv);
				YCbCr_YUYV_2(ycbcr + i + 18, yuyv);
			}
		}
		printf("num_scanlines: = %d, \n");
		lines_read += num_scanlines;
	}
#else
	if (pjDecompStruct->output_height == YRes)
	{
		for (; pjDecompStruct->output_scanline < pjDecompStruct->output_height;) {
			// convert lines of mjpeg data to YCbCr
			num_scanlines = jpeg_read_scanlines(pjDecompStruct, buffer, MAX_READLINE);
			// convert YCbCr to yuyv(YUV422)
			for (j = 0; j < num_scanlines; j++) {
				yuyv = data + (lines_read + j) * out_step;
				ycbcr = buffer[j];

				for (i = 0; i < nScanLineSize; i += 24) {	// step by YCbCr x 8 pixels = 3 x 8 bytes
					YCbCr_YUYV_2(ycbcr + i, yuyv);
					YCbCr_YUYV_2(ycbcr + i + 6, yuyv);
					YCbCr_YUYV_2(ycbcr + i + 12, yuyv);
					YCbCr_YUYV_2(ycbcr + i + 18, yuyv);
				}
			}
			if (num_scanlines == 0) {
				jpeg_finish_decompress(pjDecompStruct);
				return XN_STATUS_ERROR;
			}
			lines_read += num_scanlines;
			//printf("lines_read: %d, num_scanlines: %d\n", lines_read, num_scanlines);
		}

		*pnOutputSize = XRes * YRes * 2;
	}
#endif

	*pnOutputSize = XRes * YRes * 2;

	jpeg_finish_decompress(pjDecompStruct);

	return lines_read == YRes ? XN_STATUS_OK : XN_STATUS_ERROR;
}

#if (XN_PLATFORM == XN_PLATFORM_WIN32)
#pragma warning(pop)
#endif
