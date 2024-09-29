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
#pragma once
#include<vector>
#include <QList> 
using namespace std;
typedef struct
{
	double score;
	int type;
	double positionX;
	double positionY;
	double positionZ;
} AIJoint;
class AIBody
{

public:
	AIBody();
	~AIBody();
	const int kPixelFormatNone = 0;
	const int kPixelFormatJoint2D = 0x0001;
	const int kPixelFormatJoint3D = 0x0002;
	const int kPixelFormatBodyMask = 0x0004;
	const int kPixelFormatFloodInfo = 0x0008;
	const int kPixelFormatBodyShape = 0x0010;
	const int kPixelFormatFace = 0x0020;
	const int kPixelFormatGesture = 0x0040;
	QList<AIJoint> mAiJoints;
	float mBodyHeight = 0;
	int mDataSize = 0;
	int mJointFormat = 0;
	uint32_t mJointWidth;
	uint32_t mJointHeight;
	float mPlaneCenter[3] = { 0 };
	float mPlaneVector[3] = { 0 };
	int8_t* mDataFrame{ nullptr };
	void reset();
	uint32_t getId();
	void setId(uint32_t id);
	uint32_t getJointWidth();
	void setJointWidth(uint32_t width);
	uint32_t getJointHeight();
	void setJointHeight(uint32_t height);
	int getJointFormat();
	void setJointFormat(int joint_format);

	uint64_t getTimestamp();
	void setTimestamp(uint64_t time_stamp);
	int getJointSize();
private:
	uint32_t mId;
	uint64_t mTimeStamp;
};
