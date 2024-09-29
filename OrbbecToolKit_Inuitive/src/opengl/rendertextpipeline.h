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

#ifndef RENDERTEXTPIPELINE_H
#define RENDERTEXTPIPELINE_H

#include "renderPipeLine.h"
#include <glm/glm.hpp>
#include <ft2build.h>
#include FT_FREETYPE_H

struct Character {
	GLuint     TextureID;  // 字形纹理的ID
	glm::ivec2 Size;       // 字形大小
	glm::ivec2 Bearing;    // 从基准线到字形左部/顶部的偏移值
	GLuint     Advance;    // 原点距下一个字形原点的距离

};

typedef std::map<GLchar, Character> Characters;


class RenderTextPipeLine : public IPipeLine
{
private:
    GLint textColorHandler;
    GLint projectionHandler;
    GLuint VAO, VBO;

    Characters characters;

    QString text;
    GLfloat pos_x;
    GLfloat pos_y;
    GLfloat scale;
    GLfloat r;
    GLfloat g;
    GLfloat b;
    bool enableText;
public:
    RenderTextPipeLine():IPipeLine(TextVertexShader, TextFragmentShader){
        textColorHandler = 0;
        projectionHandler = 0;
        VAO = 0;
        VBO = 0;

        pos_x = 0.0f;
        pos_y = 0.0f;
        enableText = false;
        scale = 0.5f;

        r = 1.0f;
        g = 0.0f;
        b = 0.0f;
    }

    ~RenderTextPipeLine(){
        if(VAO != 0) glDeleteVertexArrays(1, &VAO);
        if(VBO != 0) glDeleteBuffers(1, &VBO);
    }

public:
    void createProgram();
    virtual void draw();

    void updateRenderText(QString text, float x, float y);
    void enableRenderText(bool enable);


private:
    void initCharacters();
    void initVAO();

};

#endif // RENDERTEXTPIPELINE_H
