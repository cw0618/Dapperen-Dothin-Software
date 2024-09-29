/******************************************************************************
*
* slog.h   is part of oblogger Library
*
* COPYRIGHT (c) 2017 by ORBBEC Technology, Inc.
*
*   This file belongs to ORBBEC Technology, Inc. It is considered a trade secret,
* and is not to be divulged or used by parties who have not
* received written authorization from the owner.
*
* Description : 
*
*****************************************************************************/

#ifndef _S_LOG_H_
#define _S_LOG_H_

#ifndef NULL
#define NULL         (0)
#endif

#ifndef TRUE
#define TRUE         (1)
#endif

#ifndef FALSE        
#define FALSE        (0)
#endif

#ifdef __cplusplus
extern "C"  {
#endif

#if defined(WIN32)
#define OS_API_EXPORT __declspec(dllexport)
#else 
#define OS_API_EXPORT  extern
#endif


typedef enum _slog_level {
    S_TRACE = 1,
    S_DEBUG = 2,
    S_INFO = 3,
    S_WARN = 4,
    S_ERROR = 5
} slog_level;

typedef enum _slog_out_location {
	LOCATION_NULL = 0,
	LOCATION_STDOUT = 1,
	LOCATION_FILE = 2
} slog_out_location;


//#define  PRINT_STACKTRACE
#define PRINT_STDOUT    0


/** Major version number, incremented for major API restructuring. */
/** Minor version number, incremented when significant new features added. */
/** Maintenance build number, incremented for new releases that primarily provide minor bug fixes. */
/** Build number. Incremented for each new API build. Generally not shown on the installer and download site. */

#define LOGGER_VERSION_MAJOR	0
#define LOGGER_VERSION_MINOR	0
#define LOGGER_VERSION_REVISION 1
#define LOGGER_VERSION_BUILD	1



OS_API_EXPORT inline int getLoggerVersion(unsigned int * version) {
	if (NULL == version) {
		return -1;
	}
	*version = LOGGER_VERSION_MAJOR << 24 | LOGGER_VERSION_MINOR << 16 | LOGGER_VERSION_REVISION << 8 | LOGGER_VERSION_BUILD;
	return  0;
}


/**
 * @brief  初始化日志记录器，日志记录器在使用前必须初始化，否则不会记录日志。
 * @param  const char * log_dir     日志记录目录
 * @param  slog_level level			日志过滤级别，slog_level
 * @param  slog_out_location logLocation   日志输出位置： LOCATION_NULL： 不输出，LOCATION_STDOUT：输出到控制台，LOCATION_FILE:输出的文件
 * @return OS_API_EXPORT int		返回0 代表成功。
 */
 OS_API_EXPORT int init_logger(const char *log_dir, slog_level level, slog_out_location logLocation);



/**
 * @brief  输出一条日志，建议使用宏调用。
 * @param  slog_level level
 * @param  int print_stacktrace			 是否打印堆栈
 * @param  const char * func_name
 * @param  int line
 * @param  const char * fmt
 * @param  ...
 * @return OS_API_EXPORT void
 */
 OS_API_EXPORT void write_log(slog_level level, int print_stacktrace, const char *func_name, int line, const char *fmt, ...);

 OS_API_EXPORT  void obc_log(unsigned char level, const char *fmt, ...);


#if 0

#define SLOG_ST_ERROR(fmt, ...) write_log(S_ERROR, TRUE, __FUNCTION__, __LINE__, fmt, ##__VA_ARGS__)
#define SLOG_ST_WARN(fmt, ...) write_log(S_WARN, TRUE, __FUNCTION__, __LINE__, fmt, ##__VA_ARGS__)
#define SLOG_ST_INFO(fmt, ...) write_log(S_INFO, TRUE, __FUNCTION__, __LINE__, fmt, ##__VA_ARGS__)
#define SLOG_ST_DEBUG(fmt, ...) write_log(S_DEBUG, TRUE, __FUNCTION__, __LINE__, fmt, ##__VA_ARGS__)
#define SLOG_ST_TRACE(fmt, ...) write_log(S_TRACE, TRUE, __FUNCTION__, __LINE__, fmt, ##__VA_ARGS__)

#define SLOG_ERROR(fmt, ...) write_log(S_ERROR, FALSE, __FUNCTION__, __LINE__, fmt, ##__VA_ARGS__)
#define SLOG_WARN(fmt, ...) write_log(S_WARN, FALSE, __FUNCTION__, __LINE__, fmt, ##__VA_ARGS__)
#define SLOG_INFO(fmt, ...) write_log(S_INFO, FALSE, __FUNCTION__, __LINE__, fmt, ##__VA_ARGS__)
#define SLOG_DEBUG(fmt, ...) write_log(S_DEBUG, FALSE, __FUNCTION__, __LINE__, fmt, ##__VA_ARGS__)
#define SLOG_TRACE(fmt, ...) write_log(S_TRACE, FALSE, __FUNCTION__, __LINE__, fmt, ##__VA_ARGS__)

#endif





#ifdef  PRINT_STACKTRACE


#define LOG_ERROR(fmt, ...)		write_log(S_ERROR, TRUE, __FUNCTION__, __LINE__, fmt, ##__VA_ARGS__)
#define LOG_WARN(fmt, ...)		write_log(S_WARN, TRUE, __FUNCTION__, __LINE__, fmt, ##__VA_ARGS__)
#define LOG_INFO(fmt, ...)		write_log(S_INFO, TRUE, __FUNCTION__, __LINE__, fmt, ##__VA_ARGS__)
#define LOG_DEBUG(fmt, ...)		write_log(S_DEBUG, TRUE, __FUNCTION__, __LINE__, fmt, ##__VA_ARGS__)
#define LOG_TRACE(fmt, ...)		write_log(S_TRACE, TRUE, __FUNCTION__, __LINE__, fmt, ##__VA_ARGS__)


#else

#define LOG_ERROR(fmt, ...)		write_log(S_ERROR, FALSE, __FUNCTION__, __LINE__, fmt, ##__VA_ARGS__)
#define LOG_WARN(fmt, ...)		write_log(S_WARN, FALSE, __FUNCTION__, __LINE__, fmt, ##__VA_ARGS__)
#define LOG_INFO(fmt, ...)		write_log(S_INFO, FALSE, __FUNCTION__, __LINE__, fmt, ##__VA_ARGS__)
#define LOG_DEBUG(fmt, ...)		write_log(S_DEBUG, FALSE, __FUNCTION__, __LINE__, fmt, ##__VA_ARGS__)
#define LOG_TRACE(fmt, ...)		write_log(S_TRACE, FALSE, __FUNCTION__, __LINE__, fmt, ##__VA_ARGS__)

#endif

#ifdef __cplusplus
}
#endif

#endif // !_S_LOG_H_
