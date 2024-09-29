#ifndef _OBC_TEE_FUNCS_H_
#define _OBC_TEE_FUNCS_H_
#include <stdint.h>

#ifdef __cplusplus
extern "C"{
#endif


typedef void(*func_ptr_void_uint8_vr)(uint8_t lv, const char * fmt, ...);
typedef void * (*func_ptr_voidp_uint64)(uint64_t len);
typedef void(*func_ptr_void_voidp)(void * ptr);
typedef int(*func_ptr_int_uint8p_uint32)(uint8_t * buf, uint32_t size);
typedef void(*func_ptr_void_uint64)(uint32_t usec);
struct obc_ops
{
	func_ptr_void_uint8_vr qsee_log;
	func_ptr_voidp_uint64 qsee_malloc;
	func_ptr_void_voidp qsee_free;
	func_ptr_void_uint64 tee_usleep;
	func_ptr_int_uint8p_uint32 ops_writeread;
};

typedef struct obc_ops obc_ops_t;

#ifdef __cplusplus
}
#endif

#endif // !_OBC_TEE_FUNCS_H_