#ifndef _ORBBEC_PNG_INFO_H_
#define _ORBBEC_PNG_INFO_H_

#include <stdio.h>
#include <stdbool.h>
#include "orbbec_chunk_type.h"

#ifdef _WIN32
#define OB_PNG_API __declspec(dllexport)
#else
#define OB_PNG_API __attribute__ ((visibility("default")))
#endif


#define SUCCESS                0
#define ERR_FILE_IS_NOT_PNG   -2000
#define ERR_CRC_CHECK_ERROR   -2001
#define ERR_NOT_FOUND_CHUNK   -2002
#define ERR_INVALID_PNG_FILE  -2003
#define ERR_CREATE_WRITE_PNG  -2004
#define ERR_CREATE_READ_PNG   -2005
#define ERR_READ_IMAGE        -2006
#define ERR_WRITE_IMAGE       -2007
#define ERR_ALLOC_MEM         -2008
#define ERR_INVALID_ARGS      -2009

#define CHUNK_TYPE_CODE_SIZE  4

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct
{
    uint32_t length;
    uchar chunk_type_code[CHUNK_TYPE_CODE_SIZE + 1];
    uchar* chunk_data;
    uint32_t crc;
} PNG_CHUNK;

typedef struct OBPNG_VER
{
    uint8_t major;
    uint8_t minor;
    uint8_t maintenance;
    uint8_t build;
} OBPNG_VERSION;

typedef struct{
	int width;
	int height;
	int channels;
	int bit_depth;
	int rowbytes;
	char* data;
}PNG_IMAGE;

typedef struct
{
    uchar signature[8];
    PNG_IMAGE image;
    uint32_t chunk_num;
    uint32_t chunk_max_num;
    PNG_CHUNK* chunk_list;
} PNG_FILE;


typedef void* png_struct_ptr;
typedef void* png_info_ptr;

OB_PNG_API OBPNG_VERSION obpng_version();

/// @fn OB_PNG_API int load_png_file(FILE* file, PNG_FILE* png_file);
///
/// @brief  Loads PNG file.
///
/// @param [in] file            Must open with "rb".
/// @param [out]    png_file    Shouldn't be copied. When don't need it, use free_png_file to
///                             release it.
///
/// @return Must check this. Return value will be SUCCESS, ERR_FILE_IS_NOT_PNG or
///         ERR_CRC_CHECK_ERROR.

OB_PNG_API int load_png_file(FILE* file, PNG_FILE* png_file);

/// @fn OB_PNG_API int save_png_file(FILE* file, PNG_FILE* png_file);
///
/// @brief  Saves a PNG file.
///
/// @param [out] file        Must open with "wb".
/// @param [in] png_file    If non-null, the PNG file.
///
/// @return Return value will be SUCCESS, ERR_INVALID_PNG_FILE.
///
/// @seealso    .

OB_PNG_API int save_png_file(FILE* file, PNG_FILE* png_file);

/// @fn OB_PNG_API void free_png_file(PNG_FILE* png_file);
///
/// @brief  Free PNG file.
///
/// @param [in,out] png_file    If non-null, the PNG file.
///
/// @seealso    .

OB_PNG_API int free_png_file(PNG_FILE* png_file);

/// @fn OB_PNG_API int get_chunk_info(PNG_FILE* png_file, CHUNK_TYPE chunk_type, void* info, uint32_t info_size);
///
/// @brief  Gets chunk information.
///
/// @param [in] png_file    If non-null, the PNG file.
/// @param  chunk_type      Type of the chunk.
/// @param [out]    info    Can't be NULL, should match chunk_type.
/// @param  info_size       Size of the info struct.
///
/// @return The chunk information.
///
/// @seealso    .

OB_PNG_API int get_chunk_info(PNG_FILE* png_file, CHUNK_TYPE chunk_type, void* info, uint32_t info_size);

/// @fn OB_PNG_API int set_chunk_info(PNG_FILE* png_file, CHUNK_TYPE chunk_type, void* info, uint32_t info_size);
///
/// @brief  Sets chunk information.
///
/// @param [in,out] png_file    If non-null, the PNG file.
/// @param  chunk_type          Type of the chunk.
/// @param [in] info            Can't be NULL, should match chunk_type.
/// @param  info_size           Size of the info struct.
///
/// @return An int.
///
/// @seealso    .

OB_PNG_API int set_chunk_info(PNG_FILE* png_file, CHUNK_TYPE chunk_type, void* info, uint32_t info_size);

/// @fn OB_PNG_API int add_chunk_info(png_struct_ptr pstruct, CHUNK_TYPE chunk_type, void* info, uint32_t info_size);
///
/// @brief  add chunk information.
///
/// @param [in,out] png struct  If non-null, the PNG struct.
/// @param  chunk_type          Type of the chunk.
/// @param [in] info            Can't be NULL, should match chunk_type.
/// @param  info_size           Size of the info struct.
///
/// @return An int.
///
/// @seealso    .
OB_PNG_API int add_chunk_info(png_struct_ptr pstruct, CHUNK_TYPE chunk_type, void* info, uint32_t info_size);

/// @fn OB_PNG_API int remove_chunk(PNG_FILE* png_file, CHUNK_TYPE chunk_type);
///
/// @brief  Removes the chunk.
///
/// @param [in,out] png_file    If non-null, the PNG file.
/// @param  chunk_type          Type of the chunk.
///
/// @return An int.
///
/// @seealso    .

OB_PNG_API int remove_chunk(PNG_FILE* png_file, CHUNK_TYPE chunk_type);

/// @fn OB_PNG_API png_struct_ptr create_writepng_struct();
///
/// @brief  create png struct.
///
/// @return An png handle struct.
///
/// @seealso    .
OB_PNG_API png_struct_ptr create_writepng_struct();

/// @fn OB_PNG_API png_struct_ptr create_readpng_struct();
///
/// @brief  create png struct.
///
/// @return An png handle struct.
///
/// @seealso    .
OB_PNG_API png_struct_ptr create_readpng_struct();

/// @fn OB_PNG_API png_info_ptr create_writepng_info(FILE* png_file, png_struct_ptr png_ptr, int width, int height, int bit_depth, int channels, int compression_level);
///
/// @brief  create write png info handle.
///
/// @param [in,out]             png_file If non-null, the PNG file.
/// @param png_ptr              An png handle struct.
/// @param width                png image's width.
/// @param height               png image's height.
/// @param bit_depth            bit depth of row .
/// @param channels             number of channels .
/// @param compression_level    IDAT compression level .
///
/// @return An write png info struct.
///
/// @seealso    .
OB_PNG_API png_info_ptr create_writepng_info(FILE* png_file, png_struct_ptr png_ptr, int width, int height, int bit_depth, int channels, int compression_level);

/// @fn OB_PNG_API png_info_ptr create_readpng_info(FILE* png_file, png_struct_ptr png_ptr);
///
/// @brief  create read png info handle.
///
/// @param [in,out] png_file    If non-null, the PNG file.
/// @param  png_ptr          Type of the read png struct.
///
/// @return An read png info struct.
///
/// @seealso    .
OB_PNG_API png_info_ptr create_readpng_info(FILE* png_file, png_struct_ptr png_ptr);

/// @fn OB_PNG_API int write_png_image(png_struct_ptr png_ptr, png_info_ptr infoptr, int width, int height, char* pixels);
///
/// @brief  write image data to IDAT.
///
/// @param [in] png_ptr    If non-null, the png struct handle.
/// @param [in] infoptr    If non-null, the info struct handle.
/// @param [in] width      If non-null, the image's width.
/// @param [in] height     If non-null, the image's height.
/// @param [in] pixels     If non-null, the image's data buffer.
///
/// @return An int.
///
/// @seealso    .
OB_PNG_API int write_png_image(png_struct_ptr png_ptr, png_info_ptr infoptr, int width, int height, char* pixels);

/// @fn OB_PNG_API int write_chunk_end(png_struct_ptr pstruct, png_info_ptr pinfo);
///
/// @brief  write png IEND.
///
/// @param [in] png_ptr    If non-null, the png struct handle.
/// @param [in] infoptr    If non-null, the info struct handle.
///
/// @return An int.
///
/// @seealso    .
OB_PNG_API int write_chunk_end(png_struct_ptr pstruct, png_info_ptr pinfo);

/// @fn OB_PNG_API int read_png_image(png_struct_ptr png_ptr, png_info_ptr infoptr, PNG_IMAGE* pimage);
///
/// @brief  write png IEND.
///
/// @param [in] png_ptr       If non-null, the png struct handle.
/// @param [in] infoptr       If non-null, the info struct handle.
/// @param [in,out] pimage    If non-null, read image info and data from png_ptr.
///
/// @return An int.
///
/// @seealso    .
OB_PNG_API int read_png_image(png_struct_ptr png_ptr, png_info_ptr infoptr, PNG_IMAGE* pimage);

/// @fn OB_PNG_API int write_png_chunk(png_struct_ptr pstruct, const char* name, void* data, int size);
///
/// @brief  write png chunk.
///
/// @param [in] png_ptr       If non-null, the png struct handle.
/// @param [in] name          If non-null, the png chunk name.
/// @param [in] data          If non-null, the chunk info buffer.
/// @param [in] size          If non-null, the chunk info's length.
///
/// @return An int.
///
/// @seealso    .
OB_PNG_API int write_png_chunk(png_struct_ptr pstruct, const char* name, void* data, int size);

/// @fn OB_PNG_API int pngwrite_release(png_struct_ptr* pstruct, png_info_ptr* pinfo);
///
/// @brief  release write handle.
///
/// @param [in] png_ptr       If non-null, the png struct handle.
/// @param [in] pinfo         If non-null, the png info handle.
///
/// @return An int.
///
/// @seealso    .
OB_PNG_API void pngwrite_release(png_struct_ptr* pstruct, png_info_ptr* pinfo);

/// @fn OB_PNG_API int pngwrite_release(png_struct_ptr* pstruct, png_info_ptr* pinfo);
///
/// @brief  release read handle.
///
/// @param [in] png_ptr       If non-null, the png struct handle.
/// @param [in] pinfo         If non-null, the png info handle.
///
/// @return An int.
///
/// @seealso    .
OB_PNG_API void pngread_release(png_struct_ptr* pstruct, png_info_ptr* pinfo);

/// @fn OB_PNG_API bool is_obpng(char* path);
///
/// @brief  Judge whether or not obpng.
///
/// @param [in] path  If non-null, the file path
///
/// @return An bool.
///
/// @seealso    .
OB_PNG_API bool is_obpng(const char* path);

/// @fn OB_PNG_API int mark_tool_name(void* png_ptr, char* toolname, int length);
///
/// @brief mark save png from obpng.
///
/// @param [in] png_ptr  If non-null, the png struct handle
///
/// @param [in] toolname  If non-null, mark tool name
//
/// @param [in] length  If non-null, toolname's length
//
/// @return An bool.
///
/// @seealso    .
OB_PNG_API int mark_tool_name(void* png_ptr, const char* toolname, int length);

#ifdef __cplusplus
}
#endif


#endif // !_ORBBEC_PNG_INFO_H_
