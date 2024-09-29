#ifndef __DEBUG2LOG_H__
#define __DEBUG2LOG_H__

#include <tof_sensors.h>

#ifdef __linux__
#include <unistd.h>
#endif

#define ENABLE_DEBUG 1

#ifdef WIN32
#include <stdio.h>
#define __FUNCTION__ __FUNCTION__ 
#endif

#define TEE_LOG_LEVEL_ERROR        8
#define ALOGE(fmt,...)			   tops_t.qsee_log(TEE_LOG_LEVEL_ERROR, "[ERROR][TOF_DLL][%s(%d)]:" fmt"\n",__FUNCTION__,__LINE__,##__VA_ARGS__)
#define TEE_LOG_LEVEL_DEBUG        2
#define ALOGD(fmt,...)			   tops_t.qsee_log(TEE_LOG_LEVEL_DEBUG, "[DEBUG][TOF_DLL][%s(%d)]:" fmt"\n",__FUNCTION__,__LINE__,##__VA_ARGS__)
#define DDEBUG(fmt, ...)           tops_t.qsee_log(TEE_LOG_LEVEL_DEBUG, "[ERROR][TOF_DLL][%s(%d)]:" fmt"\n",TrimFilePath(__FILE__),__LINE__,##__VA_ARGS__)

#ifdef WIN32
#define TrimFilePath(x) strrchr(x,'\\')?strrchr(x,'\\')+1:x
#else //*nix
#define TrimFilePath(x) strrchr(x,'/')?strrchr(x,'/')+1:x
#endif

#if ENABLE_DEBUG
//#define DDEBUG(fmt, ...)            tops_t.qsee_log(TEE_LOG_LEVEL_DEBUG, "[ERROR] [TOF_DLL] [%s(%d)]:=====>" fmt"\n",TrimFilePath(__FILE__),__LINE__,##__VA_ARGS__)
//#define DEBUG2LOG(fmt, ...)         printf("[DEBUG] [tof_dll] [%s(%d)]:=====>" fmt"\n",TrimFilePath(__FILE__),__LINE__,##__VA_ARGS__)
#define DEBUG2LOG(fmt, ...)         tops_t.qsee_log(TEE_LOG_LEVEL_DEBUG, "[DEBUG][TOF_DLL][%s][%s(%d)] =====>" fmt"\n", TrimFilePath(__FILE__), __FUNCTION__, __LINE__, ##__VA_ARGS__)
#else 
#define DEBUG2LOG(fmt, ...) 
#endif

#endif

