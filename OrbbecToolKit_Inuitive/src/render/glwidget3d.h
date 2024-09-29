#ifndef GLWIDGET3D_H
#define GLWIDGET3D_H

#include <QOpenGLWidget>
#include <QOpenGLFunctions>

#include <QOpenGLShaderProgram>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLBuffer>
#include <QMatrix4x4>

#include "camera.h"

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>

using namespace std;

#include "src/calculate/calc.h"
//#include "src/dialog/pcdialog.h"

//enum DepthDrawModel
//{
//    RAINBOW = 0,
//    GRAY,
//    HISTOGRAM,
//    NONE
//};

/**
 * @brief The GlWidget3D class
 * 基于QOpenGLWidget的窗口类，用于显示点云图
 */

class GlWidget3D : public QOpenGLWidget, protected QOpenGLFunctions
{
    Q_OBJECT
public:
    GlWidget3D(QWidget *parent = 0);
    ~GlWidget3D();

    void LoadPly(const QString &ply_file);

    void UpdateData(const QVector<OBPoint> &data_vec);

    void SetPointSize(float ps);


    void UpdateMouseInfo(TextInfo mouse_info);

    void UpdateFPS(TextInfo fps);

    void UpdateHowTo(TextInfo how_to);


public slots:

    // 滑块值设置camera值
    void SetXRotation(int angle);
    void SetYRotation(int angle);
    void SetZRotation(int angle);
    void SetXPosition(int pos);
    void SetYPosition(int pos);
    void SetZPosition(int pos);

    void slotSetPointSize(int size);

    // camera值设置滑块值
    void SetXSliderRotation(int angle);
    void SetYSliderRotation(int angle);
    void SetZSliderRotation(int angle);
    void SetXSliderPosition(float pos);
    void SetYSliderPosition(float pos);
    void SetZSliderPosition(float pos);
    
private:

    enum class ColorAxisMode { COLOR_BY_ROW, COLOR_BY_Z };

    void initializeGL() override;
    void paintGL() override;
    void resizeGL(int width, int height) override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;

    void keyPressEvent(QKeyEvent *event) override;

    void mouseDoubleClickEvent(QMouseEvent *event) Q_DECL_OVERRIDE;

protected:
    void wheelEvent(QWheelEvent *event) override;

    void DrawText();

private:
    void DrawFrameAxis();      // 用3中颜色画笛卡尔坐标轴

    //void ClearUp();

private slots:
    void CameraChanged(const CameraState &state);

signals:
    void SigMouseDoubleClick();

    void xRotationChanged(int angle);
    void yRotationChanged(int angle);
    void zRotationChanged(int angle);

    //void xPositionChanged(int angle);
    //void yPositionChanged(int angle);
    //void zPositionChanged(int angle);

private:
    QOpenGLVertexArrayObject    vao_;
    QOpenGLBuffer               vertex_buffer_;
    QOpenGLShaderProgram       *program_ptr_;

    float           point_size_;
    ColorAxisMode   color_mode_;
    std::vector<std::pair<QVector3D, QColor> > axes_color_vec_;

    int proj_matrix_loc_;
    int mv_matrix_loc_;
    int normal_matrix_loc_;
    int light_pos_loc_;
    int point_count_loc_;

    int win_width_;
    int win_height_;

    QPoint mouse_position_pre_;
    QPoint mouse_position_cur_;

    QMatrix4x4 projection_matrix_;
    QMatrix4x4 camera_matrix_;
    QMatrix4x4 world_matrix_;

    // pc data
    QVector<OBPoint> points_data_vec_;
    //QVector<float>  points_data_vec_;
    size_t          points_count_;
    QVector3D       points_bound_min_vec_;
    QVector3D       points_bound_max_vec_;

    QSharedPointer<Camera> camera_sp_;

    //PcDialog* pc_dialog_;

    TextInfo text_mouse_info_;
    TextInfo text_fps_;
    TextInfo text_how_to_;
};

#endif // GLWIDGED3D_H
