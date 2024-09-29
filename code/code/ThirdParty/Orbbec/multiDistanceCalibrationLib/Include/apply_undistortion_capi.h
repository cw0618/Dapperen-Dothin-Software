//
// Created by zer0 on 18-8-27.
//

#ifndef DEPTHIMGUNDISTORTION_APPLY_UNDISTORTION_CAPI_H
#define DEPTHIMGUNDISTORTION_APPLY_UNDISTORTION_CAPI_H

#include <stdint.h>

#if (defined WIN32 || defined _WIN32 || defined WINCE) && defined EFIMPL
# define EXPORT __declspec(dllexport)
#else
# define EXPORT
#endif

#ifdef __cplusplus
extern "C"
{
#endif

	typedef void* ApplyUndistHandle;
	
	EXPORT bool undistortion(ApplyUndistHandle auh,int w, int h);
	/**
	* @brief 创建应用去畸变处理器, 使用完后需要调用销毁该处理器
	* @return
	*/
	EXPORT ApplyUndistHandle CreateApplyUndistHandle(void);

	/**
	* @brief 销毁已经创建的应用去畸变处理器
	* @param auh 应用去畸变处理器
	*/
	EXPORT void DestroyApplyUndistHandle(ApplyUndistHandle* auh);

	/**
	* @brief 读取去畸变参数
	* @param auh   应用去畸变处理器
	* @param path  去畸变参数文件的地址以及文件名
	* @return 如果成功则返回0
	*/
	EXPORT int ReadBinaryFile(ApplyUndistHandle auh, const char* path, int size);

	//add
	/**
	* @brief 获取单目相机某个像素去畸变后的深度
	* @param auh       应用去畸变处理器
	* @param u         像素坐标u
	* @param v         像素坐标v
	* @param d_depth   去畸变之前的深度
	* @param ud_depth  去畸变之后的深度
	*/
	EXPORT void GetUndistortionDepthMonocular(ApplyUndistHandle auh, uint16_t u, uint16_t v, float d_depth, float* ud_depth);

	/**
	* @brief 获取双目相机某个像素去畸变后的深度
	* @param auh       应用去畸变处理器
	* @param u         像素坐标u
	* @param v         像素坐标v
	* @param d_depth   去畸变之前的深度
	* @param ud_depth  去畸变之后的深度
	*/
	EXPORT void GetUndistortionDepthStereo(ApplyUndistHandle auh, uint16_t u, uint16_t v, float d_depth, float& ud_depth);

	/**
	* @brief 获取双目相机某个像素去畸变后的视差
	* @param auh       应用去畸变处理器
	* @param u         像素坐标u
	* @param v         像素坐标v
	* @param d_disp    去畸变之前的视差
	* @param ud_disp   去畸变之后的视差
	*/
	EXPORT void GetUndistortionDispStereo(ApplyUndistHandle auh, uint16_t u, uint16_t v, float d_disp, float& ud_disp);

	/**
	* @brief 获取单目相机整幅图像去畸变后的深度，做了Neon优化，目前只支持SIMDv2/VFPv4，在MT6763平台进行测试验证
	* @param auh       应用去畸变处理器
	* @param cols      图像的列
	* @param rows      图像的行
	* @param d_depth   去畸变之前的深度
	* @param ud_depth  去畸变之后的深度
	* @param scale     缩放尺度
	*/
	EXPORT void GetUndistortionDepthStereo_MT6763(ApplyUndistHandle auh, uint16_t clos, uint16_t rows, uint16_t *d_depth, uint16_t *ud_depth, float scale);

	/**
	* @brief 获取双目相机整幅图像去畸变后的深度，该函数是为了验证GetUndistortionDepthStereo_MT6763函数的正确性
	* @param auh       应用去畸变处理器
	* @param cols      图像的列
	* @param rows      图像的行
	* @param d_depth   去畸变之前的深度
	* @param ud_depth  去畸变之后的深度
	* @param scale     缩放尺度
	*/
	EXPORT void GetUndistortionDepthStereo_no_opt(ApplyUndistHandle auh, uint16_t cols, uint16_t rows, uint16_t *d_depth, uint16_t* ud_depth, float scale);

#ifdef __cplusplus
}
#endif

#endif //DEPTHIMGUNDISTORTION_APPLY_UNDISTORTION_CAPI_H
