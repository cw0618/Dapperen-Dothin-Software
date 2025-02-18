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
#ifndef INCLUDE_OBSTATUS_H_
#define INCLUDE_OBSTATUS_H_

/**
 *@brief  出错码定义
 *@retval   0:  成功
 *@retval   2xxx:  驱动模块报错
 *@retval   3xxx: device 模块出错
 *@retval   4xxx: ref报错
 *@retval   5xxx: 深度库模块报错
 */
typedef enum 
{
        STATUS_OK = 0,   // !< 成功

        STATUS_MX6X_ERR_NULL = 2000,  //!< 空指针
        STATUS_MX6X_ERR_INVALID = 2001,  //!< 非法参数
        STATUS_MX6X_ERR_NO_SUPPORT = 2002,  //!< 不支持的操作
        STATUS_MX6X_ERR_RW = 2003,  //!< 数据读写(一般是指SPI)出错
        STATUS_MX6X_ERR_LIB = 2004,  //!< 库函数调用出错
        STATUS_MX6X_ERR_TIMEOUT = 2005,  //!< 超时
        STATUS_MX6X_ERR_TEE_OPS_NULL = 2006,  // !<空操作
        STATUS_MX6X_ERR_NO_MEM = 2007,  //!< 内存不足
        STATUS_MX6X_ERR_CAM_PARAMS_NULL = 2008,  //!< 空参数
        STATUS_MX6X_ERR_NO_INIT = 2009,  // !<未初始化
        STATUS_MX6X_ERR_BAD_SENSOR_ID_VALUE = 2010,  // !<未初始化

        STATUS_ERROR = 3001,   // !<度信操作失败
        STATUS_NOT_SUPPORT,     // !<不支持此选项
        STATUS_NOT_INIT,        // !<未初始化
        STATUS_ALREADY_INIT,    // !<
        STATUS_NULL_INPUT_PTR = 3005,   // !< 输入参数指针为空
        STATUS_NULL_OUTPUT_PTR,         // !<输出参数指针为空
        STATUS_INPUT_BUFFER_OVERFLOW,   // !<输入缓冲区溢出
        STATUS_OUTPUT_BUFFER_OVERFLOW,      // !<输出缓冲区溢出
        STATUS_INTERNAL_BUFFER_TOO_SMALL,   // !<内部缓存区太小
        STATUS_INVALID_BUFFER_SIZE,  // !<无效的缓存区大小retry read
        STATUS_NO_MATCH = 3011,             // !<不匹配
        STATUS_IS_EMPTY,
        STATUS_IS_NOT_EMPTY,
        STATUS_MODULE_IS_NULL,        // !<module 为空
        STATUS_NOT_IMPLEMENTED = 3015,       // !< 未实施
        STATUS_NO_MODULES_FOUND,           // !<没有找到module
        STATUS_INVALID_MODE,           // !<无效的模式
        STATUS_UNKNOWN_GENERATOR_TYPE,
        STATUS_INVALID_OPERATION,       // !<无效的操作
        STATUS_MISSING_NEEDED_TREE,
        STATUS_CORRUPT_FILE = 3021,
        STATUS_BAD_PARAM,      // !<错误的参数
        STATUS_NODE_IS_LOCKED,
        STATUS_WAIT_DATA_TIMEOUT,
        STATUS_BAD_TYPE = 3025,
        STATUS_UNSUPPORTED_VERSION,
        STATUS_PROPERTY_NOT_SET,
        STATUS_BAD_FILE_EXT,
        STATUS_NODE_NOT_LOADED,
        STATUS_NO_NODE_PRESENT,
        STATUS_BAD_NODE_NAME = 3031,
        STATUS_UNSUPPORTED_CODEC,   // !<不支持的操作和选项
        STATUS_EOF,
        STATUS_MULTIPLE_NODES_ERROR,
        STATUS_DEVICE_NOT_CONNECTED = 3035,
        STATUS_NO_LICENSE,
        STATUS_NO_SUCH_PROPERTY,    // !< 没有此属性
        STATUS_NODE_ALREADY_RECORDED,
        STATUS_PROTO_BAD_INTERFACE,
        STATUS_PROTO_BAD_MSG_TYPE = 3040,
        STATUS_PROTO_BAD_CID,
        STATUS_PROTO_BAD_NODE_ID,
        STATUS_PROTO_BAD_MSG_SIZE,
        STATUS_NO_SUCH_FILE,
        STATUS_IS_NOT_ACTIVE,
        STATUS_NOT_MATCH_REFERENCE,
        STATUS_NOT_MATCH_SENSORMODULE,  // !< 不匹配的sensor
        STATUS_NOT_OPEN_FILE,       // !< 文件未打开
        STATUS_ALLOC_MEM_FAILED,    // !< 分配内存失败
        STATUS_ALREADY_OPENED,      // !<
        STATUS_OPEN_LIBRARY_FAILED, // !< 打开库失败
        STATUS_FRAMESIZE_ERROR,     // !< 帧大小错误
        STATUS_LOAD_REF_FAILED,     // !< 加载ref 失败
        STATUS_NO_ACHIEVED,         // !< 尚未实现
        STATUS_LOG_MODULE_ERROR,    // !< 初始化日志模块失败
        STATUS_NOT_LOAD_REF,        // !< 未加载参考图
        STATUS_NO_SUPPORT_OPTION,
        STATUS_BAD_FRAME_INDEX,
        STATUS_NO_SUCH_DEVICE,      // !< 没有设备
        STATUS_BAD_TEMPERATURE,     // !< 异常的温度
        STATUS_CAPTURE_RETRY_TIMESOUT = 3061,      // !< 内部重试超次
        STATUS_ERROR_LOAD_REF,      // !< 解析REF报错
        STATUS_WRONG_FREQ_MODE,     // !< 频率模式不支持
        STATUS_WRONG_VCSEL,         // !< VCSEL状态异常
        STATUS_WRONG_DOTHIN_ID,


        STATUS_OB_REF_PARSE_MAGIC_UNSUPP = 4096,
        STATUS_OB_REF_PARSE_CHECK_SUM_FAILED = 4097,
        STATUS_OB_REF_PARSE_FORMAT_UNRECOG = 4098,
        STATUS_OB_REF_PARSE_CHUNK_LEN_INVALID = 4099,
        STATUS_OB_REF_PARSE_PARSE_FAILED = 4100,
        STATUS_OB_REF_PARSE_UNEXPECT_END = 4101,     // !< buffer长度不够意外的结尾
        STATUS_OB_REF_SERIAL_INSUFF_BUFFER = 4102,   //!< buffer长度不够
        STATUS_OB_REF_SERIAL_INVALID_ARGS = 4103,    //!< 错误的传入参数


        STATUS_DEPTH_LIB_NULL_INPUT_PTR = 5000,  // !< 深度库模块空指针参数
        STATUS_DEPTH_LIB_INVALID_PARAM,      // !< 非法参数
        STATUS_DEPTH_LIB_ERR_RW,             // !< 数据读写(一般是指SPI)出错
        STATUS_DEPTH_LIB_ERR_LIB,            // !< 库函数调用出错
        STATUS_DEPTH_LIB_ERR_TIMEOUT,        // !< 超时
        STATUS_DEPTH_LIB_ERR_TEE_OPS_NULL,   // !< 空操作
        STATUS_DEPTH_LIB_ERR_NO_MEM,         // !< 内存不足
        STATUS_DEPTH_LIB_ERR_NO_INIT,        // !< 未初始化
        STATUS_DEPTH_LIB_WRONG_RESULT,       // !< 结果错误
        STATUS_DEPTH_LIB_NO_SUPPORT,         // !< 不支持的操作
        STATUS_DEPTH_LIB_NO_ACHIEVED,        // !< 未实现完美
        STATUS_DEPTH_LIB_NO_IMPLEMENT,       // !< 未实现
        STATUS_DEPTH_LIB_NO_MATCH,           // !< 不匹配

        STATUS_END,
}ErrorCode;

#define OB_IS_STATUS_OK_RET(x, y)   \
        if (x != STATUS_OK)         \
        {                           \
            return (y);             \
        }

#define OB_IS_STATUS_OK(x)  OB_IS_STATUS_OK_RET(x, x)

#endif  // INCLUDE_OBSTATUS_H_
