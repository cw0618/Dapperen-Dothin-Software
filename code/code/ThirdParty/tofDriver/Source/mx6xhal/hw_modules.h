#ifndef __HW_H__
#define __HW_H__

#include <stdint.h>
//#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "hw_property.h"

#define TEE_LOG_LEVEL_LOW          1
#define TEE_LOG_LEVEL_DEBUG        (TEE_LOG_LEVEL_LOW)
#define TEE_LOG_LEVEL_MED          2
#define TEE_LOG_LEVEL_HIGH         4
#define TEE_LOG_LEVEL_ERROR        8
#define TEE_LOG_LEVEL_FATAL        10

#if defined __linux || defined __linux__
#define PACK( __Declaration__ ) __Declaration__ __attribute__ ((packed))
#define OBC_CALLBACK_TYPE
#else
	#ifndef PACK
	#define PACK( __Declaration__ ) (__Declaration__)
	#define OBC_CALLBACK_TYPE
	#endif
#endif

#ifdef __cplusplus
extern "C"{
#endif

#if 0
typedef void (* func_ptr_void_uint8_vr)(uint8_t lv, const char * fmt, ...);
typedef void * (* func_ptr_voidp_uint64)(uint64_t len);
typedef void (* func_ptr_void_voidp)(void * ptr);
typedef int  (* func_ptr_int_uint8p_uint32)(uint8_t * buf, uint32_t size);
typedef void (* func_ptr_void_uint64)(uint32_t usec);

typedef struct obc_ops obc_ops_t;
#endif

typedef struct module_version
{
	uint8_t major;     //MSB
	uint8_t minor;
	uint8_t patch_level;
	uint8_t release;
}module_version_t;



/**
 * Name of the hal_module_info
 */
#define HAL_MODULE_SYM         MX6X_HMI

/**
 * Name of the hal_module_info as a string
 */
#define HAL_MODULE_SYM_AS_STR  "MX6X_HMI"

#define HAL_PROTOCOL_SYM      PROTOCOL_HMI

#define HAL_PROTOCOL_SYM_AS_STR  "PROTOCOL_HMI"

typedef struct mx6x_module{
   int ( *init_ops)(void* ops);
   int ( *set_property)(hw_command_t command, command_data_t* data);
   int ( *get_property)(hw_command_t command, command_data_t* data);
   int ( *deinit)();

}mx6x_module_t;


int hw_module_get(char* path, mx6x_module_t* module);


typedef struct rgb_module {
    int(*init_ops)(void* ops);
    int(*deinit)();

}rgb_module_t;

#ifdef __cplusplus
}
#endif

#endif /* __MX6300_H__ */
