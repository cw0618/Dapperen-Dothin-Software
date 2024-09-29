#ifndef UTILS_H_
#define UTILS_H_

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <memory.h>

#ifdef WIN32
#define inline __inline
#endif

//extern const uchar* g_chunk_type_codes[];
static const char* g_chunk_type_codes[] = {
    "devi",
    "irsr",
    "proj",
    "extr",
    "driv",
    "envr",
    "dinf",
    "dist",
    "obbc",
	"tdev",
	"tdrv",
	"tenv",
	"tdat",
	"tirs",
};

static inline void _pack_uint8(uchar* buf, uint8_t val)
{
    buf[0] = val;
}

static inline void _pack_uint16(uchar* buf, uint16_t val)
{
    buf[0] = ((val >> 8) & 0xffU);
    buf[1] = ( val       & 0xffU);
}

static inline void _pack_uint32(uchar* buf, uint32_t val)
{
    buf[0] = ((val >> 24) & 0xffU);
    buf[1] = ((val >> 16) & 0xffU);
    buf[2] = ((val >>  8) & 0xffU);
    buf[3] = ( val        & 0xffU);
}

static inline void _pack_float(uchar* buf, float val)
{
    uint32_t* p = ((uint32_t*)&val);
    _pack_uint32(buf, *p);
}

static inline void _pack_string(uchar* buf, const uchar* str, int size)
{
    memcpy(buf, str, size);
}

static inline uint8_t _unpack_uint8(uchar* buf)
{
    return buf[0];
}

static inline uint16_t _unpack_uint16(uchar* buf)
{
    return (buf[0] << 8) + buf[1];
}

static inline uint32_t _unpack_uint32(uchar* buf)
{
    return (buf[0] << 24) +
           (buf[1] << 16) +
           (buf[2] <<  8) +
           (buf[3]);
}

static inline float _unpack_float(uchar* buf)
{
    uint32_t p = _unpack_uint32(buf);
    return *((float*)&p);
}

static inline void _unpack_string(uchar* str, uchar* buf, uint32_t size)
{
    memcpy(str, buf, size);
}

#define PACK_UINT8(buf, val) { _pack_uint8(buf, val); buf++; }
#define PACK_UINT16(buf, val) { _pack_uint16(buf, val); buf += 2; }
#define PACK_UINT32(buf, val) { _pack_uint32(buf, val); buf += 4; }
#define PACK_FLOAT(buf, val) { _pack_float(buf, val); buf += 4; }
#define PACK_STRING(buf, str, size) { _pack_string(buf, str, size); buf += size; }
#define PACK_OBSTRING(buf, val)                \
{                                              \
    PACK_UINT8(buf, val.length);               \
    if (val.length > ORBBEC_STRING_MAX_LEN)    \
        val.length = ORBBEC_STRING_MAX_LEN;    \
    PACK_STRING(buf, val.name, val.length);    \
}

#define UNPACK_UINT8(val, buf) { val = _unpack_uint8(buf); buf++; }
#define UNPACK_UINT16(val, buf) { val = _unpack_uint16(buf); buf += 2; }
#define UNPACK_UINT32(val, buf) { val = _unpack_uint32(buf); buf += 4; }
#define UNPACK_FLOAT(val, buf) { val = _unpack_float(buf); buf += 4; }
#define UNPACK_STRING(str, buf, size) { _unpack_string(str, buf, size); buf += size; }
#define UNPACK_OBSTRING(val, buf)              \
{                                              \
    UNPACK_UINT8(val.length, buf);             \
    if (val.length > ORBBEC_STRING_MAX_LEN)    \
        val.length = ORBBEC_STRING_MAX_LEN;    \
    UNPACK_STRING(val.name, buf, val.length);  \
    val.name[val.length] = 0;                  \
}
#ifdef __cplusplus
extern "C"{
#endif

int device_info2chunk(PNG_CHUNK* chunk, DEVICE_INFO* info, uint32_t info_size);
int ir_sensor_info2chunk(PNG_CHUNK* chunk, IR_SENSOR_INFO* info, uint32_t info_size);
int projector_info2chunk(PNG_CHUNK* chunk, PROJECTOR_INFO* info, uint32_t info_size);
int external_info2chunk(PNG_CHUNK* chunk, EXTERNAL_INFO* info, uint32_t info_size);
int driver_info2chunk(PNG_CHUNK* chunk, DRIVER_INFO* info, uint32_t info_size);
int env_info2chunk(PNG_CHUNK* chunk, ENV_INFO* info, uint32_t info_size);
int data_info2chunk(PNG_CHUNK* chunk, DATA_INFO* info, uint32_t info_size);
int distortion_info2chunk(PNG_CHUNK* chunk, DISTORTION_INFO* info, uint32_t info_size);
int obpng_info2chunk(PNG_CHUNK* chunk, OB_PNG_INFO* info, uint32_t info_size);
int tof_device_info2chunk(PNG_CHUNK* chunk, TOF_DEVICE_INFO* info, uint32_t info_size);
int tof_driver_info2chunk(PNG_CHUNK* chunk, TOF_DRIVER_INFO* info, uint32_t info_size);
int tof_env_info2chunk(PNG_CHUNK* chunk, TOF_ENV_INFO* info, uint32_t info_size);
int tof_data_info2chunk(PNG_CHUNK* chunk, TOF_DATA_INFO* info, uint32_t info_size);
int tof_irsensor_info2chunk(PNG_CHUNK* chunk, TOF_IRSENSOR_INFO* info, uint32_t info_size);

int chunk2device_info(DEVICE_INFO* info, uint32_t info_size, PNG_CHUNK* chunk);
int chunk2ir_sensor_info(IR_SENSOR_INFO* info, uint32_t info_size, PNG_CHUNK* chunk);
int chunk2projector_info(PROJECTOR_INFO* info, uint32_t info_size, PNG_CHUNK* chunk);
int chunk2external_info(EXTERNAL_INFO* info, uint32_t info_size, PNG_CHUNK* chunk);
int chunk2driver_info(DRIVER_INFO* info, uint32_t info_size, PNG_CHUNK* chunk);
int chunk2env_info(ENV_INFO* info, uint32_t info_size, PNG_CHUNK* chunk);
int chunk2data_info(DATA_INFO* info, uint32_t info_size, PNG_CHUNK* chunk);
int chunk2distortion_info(DISTORTION_INFO* info, uint32_t info_size, PNG_CHUNK* chunk);
int chunk2obpng_info(OB_PNG_INFO* info, uint32_t info_size, PNG_CHUNK* chunk);
int chunk2tof_device_info(TOF_DEVICE_INFO* info, uint32_t info_size, PNG_CHUNK* chunk);
int chunk2tof_driver_info(TOF_DRIVER_INFO* info, uint32_t info_size, PNG_CHUNK* chunk);
int chunk2tof_env_info(TOF_ENV_INFO* info, uint32_t info_size, PNG_CHUNK* chunk);
int chunk2tof_data_info(TOF_DATA_INFO* info, uint32_t info_size, PNG_CHUNK* chunk);
int chunk2tof_irsensor_info(TOF_IRSENSOR_INFO* info, uint32_t info_size, PNG_CHUNK* chunk);

#ifdef __cplusplus
}
#endif
#endif // !UTILS_H_
