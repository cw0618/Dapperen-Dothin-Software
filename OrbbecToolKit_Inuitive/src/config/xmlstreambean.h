/***************************************************************************/
/* */
/* Copyright (c) 2013-2021 Orbbec 3D Technology, Inc */
/* 奥比中光科技有限公司 版权所有 2013-2031 */
/* */
/* PROPRIETARY RIGHTS of Orbbec 3D Technology are involved in the */
/* subject matter of this material. All manufacturing, reproduction, use, */
/* and sales rights pertaining to this subject matter are governed by the */
/* license agreement. The recipient of this software implicitly accepts */
/* the terms of the license. */
/* */
/* 本软件文档资料是奥比中光科技有限公司的资产, 任何人士阅读和使用本资料必须获得 */
/* 相应的书面授权, 承担保密责任和接受相应的法律约束. */
/* */
/***************************************************************************/
#ifndef XMLSTREAMBEAN_H
#define XMLSTREAMBEAN_H
#include<QString>
#include<qlist.h>
typedef struct STREAM_BEAN_DATA_
{
	int mType;/**< //0为task，1为action，2为stream_type,3为wait_time,4为error_type,5为status_type */
	int mStatusType;
	int mWaitTime;
	bool mSwitchType;
	bool mRenderType;
	QString mStreamType;
	QString mResolution;

}STREAM_BEAN_DATA;
class XmlStreamBean
{

public:
	XmlStreamBean();
	const int kTaskType = 0;
	const int kActionType = 1;
	const int kStreamType = 2;
	const int kWaitTimeType = 3;
	const int kErrorType = 4;
	const int kStatusType = 5;
	const QString kColorUVCType = "uvc";
	const QString kColorType = "color";
	const QString kDepthType = "depth";
	const QString kIrType = "ir";
	const QString kPhaseType = "phase";
	const QString kAmplitudeType = "amplitude";
	QList<STREAM_BEAN_DATA> mStreamBeanList;
	bool mErrorContinue = false;
	int getTaskCount();
	void setTaskCount(int count);
private:

	int mTaskCount = 0;
};

#endif // XMLSTREAMBEAN_H
