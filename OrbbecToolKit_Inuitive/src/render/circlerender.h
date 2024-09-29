#pragma once

#include <QtMath>
#include <QList> 
#include <QOpenGLBuffer>
#include <QMap>
#include <QDebug>
class CircleRender
{
	
public:
	CircleRender();
	~CircleRender();
	 float radius = 0.01f;    // 圆半径
	 int cut_num = 30;               // 切割份数
	 int circle_num = 15;
	 int circle_point = 96;
	 int user_id = -1;
	 void UpdateSize(int circle_num);
	 void createPointS(float x, float y, float);
	 //单个圆的顶点坐标
	 GLfloat circle_vertex[96];
	 GLfloat circle_color[4] = { 1.0f, 0.0f, 0.0f, 1.0f };
	 //所有圆点的坐标
	 GLfloat line_vertex[45];
	 void addLineVertex(GLfloat* vertex, int size);
private:
	QList<GLfloat> vertex_data;
};
