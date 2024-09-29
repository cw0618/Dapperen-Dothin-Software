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

#ifndef MAIN_GL_WIDGET_H
#define MAIN_GL_WIDGET_H
#include <QOpenGLFunctions_3_3_Core>
#include <QOpenGLWidget>
#include "renderPipeline.h"
#include "renderShader.h"
#include "rendertextpipeline.h"
#include "src/calculate/calc.h"
#include"src/device/aibody.h"
#include"src/render/circlerender.h"
enum RenderType {
	kRenderNull = 0,
	kRenderPlane,
	kRenderCloud,
	kSkeletoneRender,
	kRenderIR,
	kRenderPhase,
};

class MainGLWidget : public QOpenGLWidget, protected QOpenGLFunctions_3_3_Core
{
	Q_OBJECT
private:
	PlanePipeLine planePipeLine;
	PlaneShape planeShape;

	SkeletonPipeLine skeletonPipeLine;
	PointCloudPipeLine pCloudLine;
	PointCloudPipeLine pCloudLineCoord;
	PCloudShape pCloudShape;
	PCloudShape pCloudShapeCoord;

	//RenderTextPipeLine textPipeLine;
	QCursor defaultCursor;

	int surfaceWidth, surfaceHeight;
	int posX, posY, posW, posH;

	int textPosX, textPosY;
	int pixelWidth, pixelHeight;
	int pixelPosX, pixelPosY;
	//uint16_t* raw16;
	int raw16_type{ 0 };  //0是无符号， 1有符号
	int rawSize;

	RenderType renderType;
	RenderType ai_renderType = kRenderNull;
	bool isMouseButtonHold;
	QPoint cursePos;

	int textUpdatePeriod;
	int TEXT_UPDATE_PERIOD{ 3 };

	float camPosX{ 0 }, camPosY{ 0 }, camPosZ{ 0 };

public:
	MainGLWidget(QWidget *parent = 0);

	~MainGLWidget() {
	}
	bool d2CStart = false;
	bool ai_body_shape = false;
	void updateTextureBufferData(int8_t* pixel, int width, int height, int pointSize);
	void updateDepthBufferData(int8_t* pixel, int width, int height, int pointSize);
	void updatePointCloudData(float *data, int pointCount);
	void setRaw16(uint8_t *value, int size, int raw16_type);
	void updata2DBody(AIBody& body);
	void setTEXT_UPDATE_PERIOD(int value);
	void setD2CStart(bool state);
	QPoint GetMousePos() { return QPoint(mOnFrameX, mOnFrameY); };
	void UpdateMouseInfo(TextInfo mouse_info);
	void UpdateFPS(TextInfo fps);
	void setAIRenderType(RenderType type);
	void resetMousePos(int frameX,int frameY);
protected:
	// QOpenGLWidget interface

	void initializeGL();
	void resizeGL(int w, int h);
	void paintGL();
	void DrawText();
	// QWidget interface
protected:
	void mouseMoveEvent(QMouseEvent *event) Q_DECL_OVERRIDE;
	void wheelEvent(QWheelEvent *event) Q_DECL_OVERRIDE;

	virtual void keyPressEvent(QKeyEvent *event) Q_DECL_OVERRIDE;
	//void keyReleaseEvent(QKeyEvent *event) Q_DECL_OVERRIDE;

	void mousePressEvent(QMouseEvent *event) override;
	void mouseReleaseEvent(QMouseEvent *event)Q_DECL_OVERRIDE;

	void mouseDoubleClickEvent(QMouseEvent *event);


private:
	bool isInRectArea(QPoint point);
	void updateTextValue(int value);
	void updatePosition();

	void updateTextByIndex(int index);
	bool is_offline_model_ = true;
	TextInfo mTextMouseInfo;
	TextInfo mTextFps;
	TextInfo mTextHowTo;
	TextInfo mTextMirror;
	TextInfo mTextAlinment;

	float   mScaleValue = 1.0f;
	int     mScaleAddCount = 0;
	int     mScaleDecreaseCount = 0;
	int  mAiBodyHeight = 0;
	qreal   mWindowRatio;
	qreal   mFrameRatio;
	qreal   mWHRatio;
	int mMouseLeftX = 0;
	int mMouseLeftY = 0;
	bool mMouseLeftPress;
	bool mMouseRightPress;
	int mOnFrameX;
	int mOnFrameY;

signals:
	void mainGLSigMouseMove();
	void mainGLSigMirror();
	void renderDoubleClickEvent(QMouseEvent *event);


};

#endif // MAIN_GL_WIDGET_H
