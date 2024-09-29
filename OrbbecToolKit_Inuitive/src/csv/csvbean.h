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
#ifndef CSVBEAN_H
#define CSVBEAN_H
#include"3rd\OpenNI2\Include\OniEnums.h"
#include <stdint.h>
#include <memory.h>
#include <queue>
using namespace openni;
typedef struct DataBean_t {
	int mData[4] = { 0 };
	int mTempr[2] = { 0 };

	DataBean_t& operator=(const DataBean_t& db) {
		if (this != &db) {


			memcpy(this->mData, db.mData, 4 * sizeof(int));
			memcpy(this->mTempr, db.mTempr, 2 * sizeof(int));
		}
		return *this;
	}
}DataBean;

class CSVBean
{
public:
    CSVBean();
	DataBean mDataBean ;
	/*uint16_t mFrequencyTwo[4] = { 0 };
	uint16_t mFrequencyThree[4] = { 0 };
	uint16_t mFrequencyFour[4] = { 0 };
	uint16_t mFrequencyFive[4] = { 0 };
	uint16_t mFrequencySix[4] = { 0 };*/
	std::queue<DataBean> mPhaseOne;
	std::queue<DataBean> mPhaseTwo;
	std::queue<DataBean> mPhaseThree;
	std::queue<DataBean> mPhaseFour;
	std::queue<DataBean> mPhaseFive;
	std::queue<DataBean> mPhaseSix;
	std::queue<DataBean> mPhaseSeven;
	std::queue<DataBean> mPhaseEight;
	std::queue<DataBean> mPhaseNine;
	std::queue<DataBean> mPhaseTen;
	std::queue<DataBean> mPhaseEleven;
	std::queue<DataBean> mPhaseTwelve;
	std::queue<DataBean> mPhaseThirteen;
	std::queue<DataBean> mPhaseFourteen;
	std::queue<DataBean> mPhaseFifteen;
	std::queue<DataBean> mPhaseSixteen;
	int mRoi = 0;
	int mRxTemp;
	int mTxTemp;
	bool mReady = false;
	int mType = 0;
	int mFrameIndex = 0;
	//void setFrequencyOne(uint16_t* frequency,int length);
	//void setFrequencyTwo(uint16_t* frequency,int length);

	void clear();
private:
	

};

#endif // CSVBEAN_H
