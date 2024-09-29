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
	 float radius = 0.01f;    // Բ�뾶
	 int cut_num = 30;               // �и����
	 int circle_num = 15;
	 int circle_point = 96;
	 int user_id = -1;
	 void UpdateSize(int circle_num);
	 void createPointS(float x, float y, float);
	 //����Բ�Ķ�������
	 GLfloat circle_vertex[96];
	 GLfloat circle_color[4] = { 1.0f, 0.0f, 0.0f, 1.0f };
	 //����Բ�������
	 GLfloat line_vertex[45];
	 void addLineVertex(GLfloat* vertex, int size);
private:
	QList<GLfloat> vertex_data;
};
