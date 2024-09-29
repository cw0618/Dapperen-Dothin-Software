#ifndef __HW_OBSTATIS__
#define __HW_OBSTATIS__

typedef enum HW_ERR_T {
    HW_NO_ERR = 0,  // 函数调用正常
    HW_ERR_NULL = 2000,  // 空指针
    HW_ERR_INVALID = 2001,  // 非法参数
    HW_ERR_NO_SUPPORT = 2002,  // 不支持的操作
    HW_ERR_RW = 2003,  // 数据读写(一般是指SPI)出错
    HW_ERR_LIB = 2004,  // 库函数调用出错
    HW_ERR_TIMEOUT = 2005,  // 超时
    HW_ERR_TEE_OPS_NULL = 2006,  //空操作
    HW_ERR_NO_MEM = 2007,  // 内存不足
    HW_ERR_CAM_PARAMS_NULL = 2008,  // 空参数
	HW_ERR_NO_INIT = 2009,
    HW_ERR_BAD_SENSOR_ID_VALUE = 2010,  // !<未初始化
    HW_ERR_BAD_VALUE = 2011,  // !<未初始化
	HW_ERR_NO_MATCH
} HW_ERR;




#endif