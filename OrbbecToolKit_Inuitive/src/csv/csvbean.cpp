#include "csvbean.h"
/** \class CaptureDialog
*
* csv格式的数据类
*
*/
CSVBean::CSVBean()
{

}
//void CSVBean::setFrequencyOne(uint16_t* frequency, int length) {
//	for (int i = 0; i < length; i++)
//	{
//		mFrequencyOne[i] = frequency[i];
//	}
//}
//void CSVBean::setFrequencyTwo(uint16_t* frequency, int length) {
//	for (int i = 0; i < length; i++)
//	{
//		mFrequencyTwo[i] = frequency[i];
//	}
//}

void CSVBean::clear()
{
	
	memset(mDataBean.mData, 0, 4 * sizeof(uint16_t));
	memset(mDataBean.mTempr, 0, 2 * sizeof(uint16_t));
	//memset(mFrequencyThree, 0, 4 * sizeof(uint16_t));
	//memset(mFrequencyFour, 0, 4 * sizeof(uint16_t));
	//memset(mFrequencyFive, 0, 4 * sizeof(uint16_t));
	//memset(mFrequencySix, 0, 4 * sizeof(uint16_t));
}
