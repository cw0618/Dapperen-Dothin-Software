#include "circlerender.h"

CircleRender::CircleRender()
{
}

CircleRender::~CircleRender()
{
}
void CircleRender::UpdateSize(int circle_num) {
	if (this->circle_num!= circle_num)
	{
		this->circle_num = circle_num;
	}
}
void CircleRender::createPointS(float x, float y, float z) {
	float angDegSpan = 360.0f / cut_num;
	vertex_data.clear();
	vertex_data.append(x);
	vertex_data.append(y);
	vertex_data.append(z);
	for (float i = 0; i <= 360; i += angDegSpan) {
		vertex_data.append((float)(radius * qSin(i * M_PI / 180.0f)) + x);
		vertex_data.append((float)(radius * qCos(i * M_PI / 180.0f)) + y);
		vertex_data.append(z);
	}
	int data_size=vertex_data.size();
	//qDebug() << "howard data_size==" << data_size;
	for (int i = 0; i < data_size; i++)
	{
		if (i<circle_point)
		{
			circle_vertex[i] = vertex_data.at(i);
		}
		
	}
}
void CircleRender::addLineVertex(GLfloat* vertex,int size) {

	
}
