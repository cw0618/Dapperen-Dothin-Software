/***************************************************************************/
/* */
/* Copyright (c) 2013-2021 Orbbec 3D Technology, Inc */
/* 奥比中光科技有限公司 版权所有 2013-2031 */
/* */
/* PROPRIETARY RIGHTS of Orbbec 3D Technology are involved in the */
/* subject matter of this material. All manufacturing, reproduction, use, */
/* and sales rights pertaining to this subject matter are governed by the */
/* license agreement. The recipient of this software implicitly accepts */
/* the terms of the license. */
/* */
/* 本软件文档资料是奥比中光科技有限公司的资产, 任何人士阅读和使用本资料必须获得 */
/* 相应的书面授权, 承担保密责任和接受相应的法律约束. */
/* */
/***************************************************************************/
#ifndef OBPNG_H
#define OBPNG_H
#pragma once

#include "3rd/obpng/include/orbbec_png_info.h"

struct writeInfo 
{
    int width;
    int height;
    int bit_depth;
    int channels;
    int compression_level;

    float fx, fy, cx, cy;   //鍐呭弬

    writeInfo()
    {
        width = 0;
        height = 0;
        bit_depth = 0;
        channels = 0;
        compression_level = 0;

        fx = fy = cx = cy = 0;
    }
};

class ObPng
{
public:
    ObPng();
    ~ObPng();

public:
    OBPNG_VERSION m_obpngVersion;   // struct
    PNG_FILE m_PngFile;             //struct

    png_struct_ptr m_png_ptr = NULL;  // void*
    png_info_ptr   m_info_ptr = NULL; // void*

    bool LoadObpng(const char* fileName);   // 璇籭r png
    bool WriteObpng(const char* fileName, uint16_t* data, writeInfo info);  // 鍐欏叆ir png

    float ir_fx() { return m_ir_fx; };
    float ir_fy() { return m_ir_fy; };
    float ir_cx() { return m_ir_cx; };
    float ir_cy() { return m_ir_cy; };
    int get_ir_width() { return m_ir_width; };
    int get_ir_height() { return m_ir_height; };
    int get_ir_size() { return m_ir_size; };
    char* get_ir_data() { return m_ir_data; };

    // write

private:
    float m_ir_fx;
    float m_ir_fy;
    float m_ir_cx;
    float m_ir_cy;
    char* m_ir_data;

    int m_ir_width;
    int m_ir_height;
    int m_ir_size;
    void get_ir_param();
    void write_ir_param(writeInfo info);

};

#endif 