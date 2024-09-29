#ifndef __DEPTH_OBSTATUS_H_
#define __DEPTH_OBSTATUS_H_

/**
 *@brief  深度库出错码定义
 */
typedef enum{
             DEPTH_OK = 0,   /*!<成功 */


			 DEPTH_LIB_NULL_INPUT_PTR = 5000,  //!< 深度库模块空指针参数
			 DEPTH_LIB_INVALID_PARAM ,  //!< 非法参数
			 DEPTH_LIB_NO_SUPPORT,  //!< 不支持的操作
			 DEPTH_LIB_ERR_RW ,  //!< 数据读写(一般是指SPI)出错
			 DEPTH_LIB_ERR_LIB ,  //!< 库函数调用出错
			 DEPTH_LIB_ERR_TIMEOUT,  //!< 超时
			 DEPTH_LIB_ERR_TEE_OPS_NULL,  //!<空操作
			 DEPTH_LIB_ERR_NO_MEM ,  //!< 内存不足
			 DEPTH_LIB_ERR_NO_INIT=5008,  //!<未初始化
			 DEPTH_LIB_NO_SUPPORT_OPTION,   //!<不支持此选项
			 DEPTH_LIB_NO_ACHIEVED,		//!< 未实现完美
			 DEPTH_LIB_NO_IMPLEMENT,		//!< 未实现
			
			
			
             DEPTH_END,
}DepthErrorCode;

#define OB_IS_STATUS_OK_RET(x, y)	\
		if (x != STATUS_OK)		    \
		{							\
			return (y);				\
		}

#define OB_IS_STATUS_OK(x)					OB_IS_STATUS_OK_RET(x, x)

#endif
