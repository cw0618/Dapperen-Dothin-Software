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
#ifndef GLCOLORWINDOW_H
#define GLCOLORWINDOW_H
#include <QOpenGLWidget>
#include <QOpenGLFunctions>

#include <QMatrix4x4>
#include <QQuaternion>
#include <QVector2D>
#include <QOpenGLShaderProgram>
#include <QOpenGLShader>
#include <QOpenGLTexture>

#include "src/calculate/calc.h"


class glColorWindow:public QOpenGLWidget, protected QOpenGLFunctions
{
    Q_OBJECT
public:
    explicit glColorWindow(QWidget *parent = 0);
    ~glColorWindow();
bool UpdateTexture(const unsigned char * rgb_data, int rgb_width, int rgb_height,const unsigned char * depth_data, int depth_width, int depth_height,bool d2c);
protected:
    void initializeGL() Q_DECL_OVERRIDE;
    void paintGL() Q_DECL_OVERRIDE;
    void resizeGL(int w, int h) Q_DECL_OVERRIDE;
    //void timerEvent(QTimerEvent *e) Q_DECL_OVERRIDE;
    void InitShaders();
    void InitTextures();
    void InitData();

    void Draw();
private:
    QOpenGLShaderProgram *program_;
    QOpenGLTexture *rgb_texture_;
    QOpenGLTexture *depth_texture_;
    QVector<QVector3D>  vec_vertices_;
    QVector<QVector2D>  vec_texcoords_;

    GLuint mvp_matrix_handle_;
    GLuint vertices_handle_;
    GLuint texcoor_handle_;
    GLuint texcoor_rgb_handle_;
    GLuint texcoor_depth_handle_;
    GLuint sampler_rgb;
    GLuint sampler_depth;
    GLuint hChangeType;

    QMatrix4x4 model_matrix_;
    QMatrix4x4 view_matrix_;
    QMatrix4x4 projection_matrix_;
    QMatrix4x4 mvp_matrix_;

    int frame_width_=0;
    int frame_height_=0;
    qreal   frame_w_h_ratio_;
    int     view_width_=0;
    int     view_height_=0;
    qreal   view_w_h_ratio_;
    bool d2c_switch=false;
};

#endif // GLCOLORWINDOW_H
