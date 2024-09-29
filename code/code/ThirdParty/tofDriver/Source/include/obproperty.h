/*****************************************************************************
*  Orbbec TOF SDK
*  Copyright (C) 2019 by ORBBEC Technology., Inc.
*
*  This file is part of Orbbec TOF SDK.
*
*  This file belongs to ORBBEC Technology., Inc.
*  It is considered a trade secret, and is not to be divulged or used by
* parties who have NOT received written authorization from the owner.
*
*  Description:
*
****************************************************************************/
#ifndef __OB_PROPERTY__
#define __OB_PROPERTY__


/**
 *@brief  旋转角度
 */
typedef enum ref_rotation_e
{
	HW_REF_ROTATION_0 = 0,
	HW_REF_ROTATION_CLOCKWISE_90 = 1,
	HW_REF_ROTATION_ANTI_CLOCKWISE_90 = 2,
	HW_REF_ROTATION_180 = 3,
} OB_REF_ROTATION;

typedef struct
{
	float rx, ry, rz;
	float tx, ty, tz;
} ob_external_info;

typedef struct
{
	uint8_t disto_model;
	struct
	{
		float k1;
		float k2;
		float t1;
		float t2;
		float k3;
		///
	} disto_data;
} ob_distortion_info;

/**
 *@brief 参考图摘要信息
 */
typedef struct ref_summary_t {
    float baseline;    /*!< baseline*/
	float ref_dist;
	float pixel_size;

	OB_REF_ROTATION ir_rotate;
	uint8_t ir_mirror;
	ob_distortion_info disto_info;
	ob_external_info external_info;
}ob_ref_summary;
#endif
