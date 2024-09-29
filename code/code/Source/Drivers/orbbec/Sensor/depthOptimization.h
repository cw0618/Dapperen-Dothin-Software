#ifndef _DEPTHOPTIMIZATION_H_
#define _DEPTHOPTIMIZATION_H_

#ifdef __cpluscplus
extern "C" {
#endif
#if(defined WIN32 || defined _WIN32 || defined WINCE)
#	define DLLEXPORT __declspec(dllexport)
#else
#	define DLLEXPORT
#endif

/**
 * @brief   深度优化函数
 * @param[in]   orginDepth  原始深度
 * @param[in]   numDepth    深度个数
 * @param[out]  optDepth    优化后的深度，需要提前给optDepth分配好内存，内存大小与输入深度所占内存一致
 * @param[in]   param1      优化参数
 * @param[in]   param2      优化参数
 * @param[in]   param3      优化参数
 * @param[in]   depthScale  深度刻度
 * @retval  0   success     优化成功
 * @retval -1   failed      优化失败,一般只会由于输入优化参数异常导致

 */
DLLEXPORT int depthOptimization(unsigned short* orginDepth, unsigned int numDepth, unsigned short* optDepth,
	const double param1, const double param2, const double param3, unsigned int depthScale);

#ifdef __cpluscplus
};
#endif
#endif