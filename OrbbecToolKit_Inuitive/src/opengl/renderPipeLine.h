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
#ifndef RENDERPIPELINE_H
#define RENDERPIPELINE_H
#include <QOpenGLWidget>
#include <QOpenGLFunctions_3_3_Core>
#include "renderShader.h"
#include <QOpenGLBuffer>
#include<qglobal.h>
#include "shape.h"
#include "texturebuffer.h"
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include"src/device/aibody.h"
#include <QOpenGLShaderProgram>
typedef struct SKELETON_DATA
{
    int index;
    float x;
    float y;
    float z;
    float score;
}SKELETON_DATA_T;

typedef struct SKELETON_GROUP {
    int id;
    QVector<SKELETON_DATA_T> mSkeletons;
}SKELETON_GROUP_t;
class IPipeLine : protected QOpenGLFunctions_3_3_Core
{
private:
    const char* vertexShaderSrc;
    const char* fragmentShaderSrc;

protected:
    GLuint mProgramId;
    int surfaceWidth,surfaceHeight;
    int frameWidth, frameHeight;
    IShape *shape;


public:
    IPipeLine(const char* vertexShader,const char* fragmentShader):
        vertexShaderSrc(vertexShader), fragmentShaderSrc(fragmentShader)
    {
        mProgramId = 0;
        surfaceWidth = 0;
        surfaceHeight = 0;
        frameWidth = 0;
        frameHeight = 0;
        shape = NULL;
    }

    virtual ~IPipeLine(){
        if(mProgramId != 0){
            glDeleteProgram(mProgramId);
            mProgramId = 0;
        }
    }

    virtual void createProgram(){
        initializeOpenGLFunctions();
        mProgramId = esCreateProgram(vertexShaderSrc, fragmentShaderSrc);
        if(mProgramId == 0){
            qDebug("Failed to create program.");
        }
    }

    virtual void updateSurfaceSize(int surfaceWidth,int surfaceHeight){
        this->surfaceWidth  = surfaceWidth;
        this->surfaceHeight = surfaceHeight;
    }


    virtual void draw() = 0;

    void setShape(IShape *value);
    SKELETON_GROUP_t mGroups;
    AIBody body_2d;
protected:
    virtual void useProgram() {
        if(mProgramId == 0){
            return;
        }
        glUseProgram(mProgramId);
        if(esCheckError()){
            qDebug("useProgram error. ");
        }
    }

    GLuint esCreateProgram(const char* vertexShaderSrc, const char* fragmentShaderSrc);
    GLuint esLoadShader(GLenum type, const char *shaderSrc);
    int esCheckError();
};

class PlanePipeLine : public IPipeLine{
private:
    GLint textureSamplerHandler;
    GLint depthSamplerHandler;
    GLint planeSamplerHandler;
    GLint hChangeType;
	GLint hPlaneCenter;
    GLuint textureId[3] = { 0,0,0 };
    TextureBuffer textureBuf;




public:
    PlanePipeLine():IPipeLine(D2CVertexShader, D2CFragmentShader){
        //PlanePipeLine() :IPipeLine(PlaneVertexShader, PlaneFragmentShader) {
        textureSamplerHandler = 0;
        depthSamplerHandler = 0;
        planeSamplerHandler = 0;
        hChangeType = 0;
		hPlaneCenter = 0;
    }
    bool d2CStart = false;
    void createProgram();
    void setD2CStart(bool state);
    void updateAIData(AIBody& body);
    void updateTextureBufferData(int8_t* pixel, int width, int height, int pointSize);
    void updateDepthBufferData(int8_t* pixel, int width, int height, int pointSize);
    virtual void draw();

private:
    void bindTextureBuffer();

};


class PointCloudPipeLine : public IPipeLine{
private:

    glm::mat4 mvpMatrix;

    glm::mat4 projection;
    float fov;
    float aspect;

    glm::mat4 view;
    glm::vec3 cameraPos;
    glm::vec3 cameraFront;
    glm::vec3 cameraUp;

    glm::mat4 model;
    glm::mat4 rotationM;
    glm::mat4 baseModel;
    float m_angleX;
    float m_angleY;

    GLint mvpMatrixHandler;
public:
    PointCloudPipeLine():IPipeLine(PCloudVertexShader,PCloudFragmentShader),
        mvpMatrixHandler(0),m_angleX(0),m_angleY(0)
    {
        fov = 45.0f;
        aspect = 1;

        cameraPos    = glm::vec3(0.0f, 0.0f,  -6000.0f);
        cameraFront  = glm::vec3(0.0f, 0.0f,  -1.0f);
        cameraUp     = glm::vec3(0.0f, -1.0f,  0.0f);
        view         = glm::lookAt(cameraPos,cameraFront,cameraUp);
        rotationModel(-6.48, 6.255);
        translateViewer(new float[3]{0,0,-3600});
    }

    void createProgram();

    void updateSurfaceSize(int width, int height);
    void rotationModel(float x, float y);
    void translateViewer(float* vec3);
    virtual void draw();

private:
    void updateMvpMatrix();
};
class SkeletonPipeLine : public IPipeLine {
    typedef struct DisplayInfo_
    {
        int x_start;
        int y_start;
        int w_win;
        int h_win;
    }DisplayInfo;
private:
    GLint vertexSamplerHandler;
    GLint colorSamplerHandler;
    GLint muMVPMatrixHandle;
    QMatrix4x4 matrix_mvp;
    GLuint textureId;
    GLfloat vertices_line[6] = {0};
    GLfloat colors_line[8] = { 1.0f,0.0f,0.0f,0.0f,
                               1.0f,0.0f,0.0f,0.0f
                             };
    GLfloat vertices_circle[3] = {0};
    GLfloat colors_circle[4] = {0.0f,0.0f,1.0f,0.0f};
	GLfloat colors_circle_red[4] = { 1.0f,0.0f,0.0f,0.0f };
    bool d2CStart = false;
    DisplayInfo_ displayInfo;
public:
    SkeletonPipeLine() :IPipeLine(SkeletonVertexShader, SkeletonFragmentShader) {
        vertexSamplerHandler = -1;
        colorSamplerHandler = -1;
    }
    float mWidthRatio = 0, mHeightRatio = 0;
    float rgb_width = 0, rgb_height = 0;
    float ratioX, ratioY;
    void createProgram();
    void drawPartLine(int index, int index0, int index1);
    void setRGBSize(int width,int height);
    virtual void draw();
    void updata2DBody(AIBody& body);
private:


};

#endif // RENDERPIPELINE_H
