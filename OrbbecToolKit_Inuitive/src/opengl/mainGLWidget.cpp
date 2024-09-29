#include "mainGLWidget.h"
#include <QDebug>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QApplication>
#include <QPainter>
/** \class MainGLWidget
*
* 图像数据渲染类
*
*/

MainGLWidget::MainGLWidget(QWidget *parent) :QOpenGLWidget(parent)
{
    planePipeLine.setShape(&planeShape);

    //skeletonPipeLine.setShape(&skeletonShape);
    pCloudLine.setShape(&pCloudShape);
    pCloudLineCoord.setShape(&pCloudShapeCoord);

    pixelWidth = surfaceWidth = 0;
    pixelHeight = surfaceHeight = 0;
    renderType = kRenderNull;
    posX = posY = posW = posH = 0;
    textUpdatePeriod = 0;

    textPosX = textPosY = 0;
    pixelPosX = pixelPosY = 0;
    defaultCursor = cursor();
    mWindowRatio = 0;
    mFrameRatio = 0;
    mOnFrameX = -1;
    mOnFrameY = -1;

    isMouseButtonHold = false;
    rawSize = 0;
    //raw16 = NULL;

    setMouseTracking(true);
    setFocusPolicy(Qt::ClickFocus);
}

void MainGLWidget::setTEXT_UPDATE_PERIOD(int value)
{
    TEXT_UPDATE_PERIOD = value;
}

void MainGLWidget::initializeGL()
{
    initializeOpenGLFunctions();
    planePipeLine.createProgram();
    planeShape.createVAO();

    skeletonPipeLine.createProgram();
    //skeletonShape.createVAO();

    pCloudLine.createProgram();
    pCloudLineCoord.createProgram();
    pCloudShape.createVAO();
    pCloudShapeCoord.createVAO();
    //textPipeLine.createProgram();
}

void MainGLWidget::resizeGL(int w, int h)
{
    planePipeLine.updateSurfaceSize(w, h);
    skeletonPipeLine.updateSurfaceSize(w, h);
    //textPipeLine.updateSurfaceSize(w,h);
    pCloudLine.updateSurfaceSize(w,h);
    pCloudLineCoord.updateSurfaceSize(w,h);

    mWindowRatio = static_cast<qreal>(w) / h;

    surfaceWidth = w;
    surfaceHeight = h;

    updatePosition();
}

void MainGLWidget::paintGL()
{
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    if (renderType == kRenderPlane) {
        glViewport(posX, posY, posW, posH);
        planePipeLine.draw();

        //glViewport(0, 0, surfaceWidth,surfaceHeight);
        //textPipeLine.draw();


    }else if(renderType == kRenderCloud){
        glViewport(0, 0, surfaceWidth,surfaceHeight);
        pCloudLine.draw();
        pCloudLineCoord.draw();

    }
    else if (renderType == kRenderCloud) {
        //glViewport(posX, posY, posW, posH);
        glViewport(0, 0, surfaceWidth, surfaceHeight);
        pCloudLine.draw();
        pCloudLineCoord.draw();
    }
    if (ai_renderType == kSkeletoneRender && (skeletonPipeLine.body_2d.mJointFormat == skeletonPipeLine.body_2d.kPixelFormatJoint2D || skeletonPipeLine.body_2d.mJointFormat == skeletonPipeLine.body_2d.kPixelFormatBodyShape))
    {
        skeletonPipeLine.draw();
    }
    DrawText();
}

void MainGLWidget::setRaw16(uint8_t *buf, int size, int raw16_type)
{
    if (buf == NULL || size == 0) {
        return;
    }

    /*if(raw16 == NULL || rawSize != size){
        if(raw16 != NULL){
            delete raw16;
            raw16 = NULL;
        }
        raw16 = (uint16_t *)new char[size];
        rawSize = size;
    }

    memcpy(raw16 , buf, size);

    int index = (pixelPosY * pixelWidth) + pixelPosX;
    if(index >= (pixelHeight * pixelWidth)){
        return;
    }*/

    //this->raw16_type = raw16_type;
    //updateTextByIndex(index);

}

void MainGLWidget::mousePressEvent(QMouseEvent *event)
{
    if (renderType == kRenderCloud && event->buttons() == Qt::LeftButton) {
        isMouseButtonHold = true;

        QCursor cursor;
        cursor.setShape(Qt::ClosedHandCursor);
        setCursor(cursor);

        cursePos = event->pos();
    }

}

void MainGLWidget::mouseReleaseEvent(QMouseEvent *event)
{

    if (isMouseButtonHold && event->buttons() == Qt::NoButton) {
        isMouseButtonHold = false;
        setCursor(defaultCursor);
    }

}

void MainGLWidget::wheelEvent(QWheelEvent *event)
{

    float tempZ = -(float)(event->delta() * 5);
    camPosZ += tempZ;
    float vec3[] = { 0.0f, 0.0f,tempZ };
    pCloudLine.translateViewer(vec3);
    pCloudLineCoord.translateViewer(vec3);
    update();


}

void MainGLWidget::mouseDoubleClickEvent(QMouseEvent *event)
{

    emit renderDoubleClickEvent(event);

}

void MainGLWidget::mouseMoveEvent(QMouseEvent *event)
{

    QPoint tempPos = event->pos();
    if (renderType == kRenderCloud) {
        if (isMouseButtonHold) {
            int dx = tempPos.x() - cursePos.x();
            int dy = tempPos.y() - cursePos.y();
            cursePos = tempPos;
            pCloudLine.rotationModel(-0.005*dx, 0.005*dy);
            pCloudLineCoord.rotationModel(-0.005*dx, 0.005*dy);
            update();
        }
	}
	else
	{
		int h_boder = 0;
		int v_boder = 0;

		int img_w = 0;
		int img_h = 0;
		if (abs(mWindowRatio - mFrameRatio) < 1e-5)
		{
			h_boder = 0;
			v_boder = 0;
			img_w = surfaceWidth;
			img_h = surfaceHeight;
		}
		else if (mWindowRatio > mFrameRatio)
		{

			img_h = surfaceHeight;
			img_w = img_h * mFrameRatio;
			v_boder = 0;
			h_boder = (surfaceWidth - img_w) / 2;
		}
		if (mWindowRatio < mFrameRatio)
		{

			img_w = surfaceWidth;
			img_h = img_w / mFrameRatio;

			v_boder = (surfaceHeight - img_h) / 2;
			h_boder = 0;
		}

		QPoint mouse_p = event->pos();
		mOnFrameX = mouse_p.x() - h_boder;
		mOnFrameY = mouse_p.y() - v_boder;

		if (mOnFrameX >= 0) {
			mOnFrameX = mOnFrameX*1.0 / img_w * pixelWidth;
		}


		if (mOnFrameY >= 0) {
			mOnFrameY = mOnFrameY*1.0 / img_h * pixelHeight;
		}
	}

}

void MainGLWidget::updateTextValue(int value)
{
    if(textUpdatePeriod == 0){
        textUpdatePeriod = TEXT_UPDATE_PERIOD;
        //textPipeLine.updateRenderText(QString("x:%1 y:%2 v:%3").arg(pixelPosX).arg(pixelPosY).arg(value), textPosX, textPosY);
    }else{
        textUpdatePeriod--;
    }
}

bool MainGLWidget::isInRectArea(QPoint point)
{
    if (point.x() >= posX && point.x() <= posX + posW &&
            point.y() >= posY && point.y() <= posY + posH) {
        return true;
    }
    return false;
}

void MainGLWidget::updatePosition()
{
    if (pixelWidth > 0 && pixelHeight > 0) {
        float surfD = float(surfaceWidth) / surfaceHeight;
        float textD = float(pixelWidth) / pixelHeight;

        if(surfD >= textD){
            posH = surfaceHeight;
            posW = posH*textD;
            posX = (surfaceWidth - posW)/2;
            posY = 0;
        }else{
            posW = surfaceWidth;
            posH = posW/textD;
            posX = 0;
            posY = (surfaceHeight - posH)/2;
        }
    }
}

void MainGLWidget::updateTextureBufferData(int8_t* pixel, int width, int height, int pointSize)
{
    planePipeLine.updateTextureBufferData(pixel, width, height, pointSize);
    renderType = kRenderPlane;
    skeletonPipeLine.setRGBSize(width, height);
    pixelWidth = width;
    pixelHeight = height;
    uint32_t img_data_size = height * width * 3 * sizeof(char);
	mFrameRatio = static_cast<qreal>(pixelWidth) / pixelHeight;
    mWHRatio = static_cast<qreal>(width) / height;
    if (mWHRatio != mFrameRatio)
    {
        mFrameRatio = static_cast<qreal>(width) / height;
    }
    updatePosition();
}
void MainGLWidget::updateDepthBufferData(int8_t* pixel, int width, int height, int pointSize)
{
    planePipeLine.updateDepthBufferData(pixel, width, height, pointSize);
    //renderType = RENDER_PLANE;

    //pixelWidth = width;
    //pixelHeight = height;

    //updatePosition();
}
void MainGLWidget::updata2DBody(AIBody& body) {
	//qDebug() << "howard body=="<< body.jointFormat;
    if(body.mJointFormat ==body.kPixelFormatJoint2D){
        skeletonPipeLine.updata2DBody(body);
	}
	else if (body.mJointFormat == body.kPixelFormatBodyShape)
	{
		skeletonPipeLine.body_2d.mJointFormat = skeletonPipeLine.body_2d.kPixelFormatBodyShape;
		mAiBodyHeight = body.mBodyHeight / 10;
	}
	else if (body.mJointFormat == body.kPixelFormatBodyMask)
	{
		skeletonPipeLine.body_2d.mJointFormat = skeletonPipeLine.body_2d.kPixelFormatBodyMask;
		planePipeLine.updateAIData(body);
	}
	else if(body.mJointFormat ==body.kPixelFormatFloodInfo){
		skeletonPipeLine.body_2d.mJointFormat = skeletonPipeLine.body_2d.kPixelFormatFloodInfo;
		planePipeLine.updateAIData(body);
    }else{
		skeletonPipeLine.body_2d.mJointFormat = skeletonPipeLine.body_2d.kPixelFormatNone;
        planePipeLine.updateAIData(body);
    }

}
void MainGLWidget::updatePointCloudData(float *data, int pointCount)
{
    renderType = kRenderCloud;
    pCloudShape.updatePointCloudData(data, pointCount);
    pCloudShapeCoord.setNeedDrawCoord(true);

}
void MainGLWidget::UpdateMouseInfo(TextInfo mouse_info)
{
    mTextMouseInfo = mouse_info;
    update();
}

void MainGLWidget::UpdateFPS(TextInfo fps)
{
    mTextFps = fps;
    //update();
}
void MainGLWidget::setAIRenderType(RenderType type) {
    ai_renderType = type;
	//if (type == RenderType::RENDER_NULL)
	//{
	//	skeletonPipeLine.body_2d.jointFormat = skeletonPipeLine.body_2d.PIXEL_FORMAT_NONE;
	//}
}
void MainGLWidget::setD2CStart(bool state) {
    d2CStart = state;
    planePipeLine.setD2CStart(state);
}
void MainGLWidget::updateTextByIndex(int index)
{
    //if(raw16_type == 0) // 无符号
    //{
    //    uint16_t value = raw16[index];
    //    updateTextValue(value);
    //}else{
    //    int16_t value = raw16[index];
    //    updateTextValue(value);
    //}

}
void MainGLWidget::DrawText()
{

    QPainter painter(this);

    /// mouse infor
    QFont font = painter.font();
    font.setPixelSize(mTextMouseInfo.fontSize);
    painter.setFont(font);

    painter.setPen(mTextMouseInfo.penColor);
    painter.setRenderHint(QPainter::TextAntialiasing);
    painter.drawText(2, surfaceHeight - 30, mTextMouseInfo.text);

    /// how to
    font.setPixelSize(mTextHowTo.fontSize);
    painter.setFont(font);

    painter.setPen(mTextHowTo.penColor);
    painter.setRenderHint(QPainter::TextAntialiasing);
    painter.drawText(2, 20, mTextHowTo.text);

    /// fps
    font.setPixelSize(mTextFps.fontSize);
    painter.setFont(font);

    painter.setPen(mTextFps.penColor);
    painter.setRenderHint(QPainter::TextAntialiasing);
    painter.drawText(2, surfaceHeight - 3, mTextFps.text);

    /// mirror
    font.setPixelSize(mTextMirror.fontSize);
    painter.setFont(font);

    painter.setPen(mTextMirror.penColor);
    painter.setRenderHint(QPainter::TextAntialiasing);
    painter.drawText(surfaceWidth / 2 - 20, surfaceHeight - 3, mTextMirror.text);

    /// alignment
    font.setPixelSize(mTextAlinment.fontSize);
    painter.setFont(font);

    painter.setPen(mTextAlinment.penColor);
    painter.setRenderHint(QPainter::TextAntialiasing);
    painter.drawText(surfaceWidth - 100, surfaceHeight - 3, mTextAlinment.text);
	if (ai_body_shape)
	{   //显示身高值
		painter.setPen(mTextMouseInfo.penColor);
		painter.setRenderHint(QPainter::TextAntialiasing);
		QString height_value = QString("Height:%1 cm").arg(mAiBodyHeight);
		painter.drawText(surfaceWidth/2-20, 20, height_value);
	}


}
//void MainGLWidget::mouseMoveEvent(QMouseEvent *event)
//{
//	int h_boder = 0;    ///  水平方向留空
//	int v_boder = 0;    ///  垂直方向留空
//
//	int img_w = 0;
//	int img_h = 0;
//
//	// 一样大小, 四边不留空白
//	if (abs(win_w_h_ratio_ - img_w_h_ratio_) < 1e-5)
//	{
//		h_boder = 0;
//		v_boder = 0;
//		img_w = surfaceWidth;
//		img_h = surfaceHeight;
//	}
//	else if (win_w_h_ratio_ > img_w_h_ratio_)
//	{
//		// 窗口比较宽， 两边留空
//		img_h = surfaceHeight;
//		img_w = img_h * img_w_h_ratio_;
//		v_boder = 0;
//		h_boder = (surfaceWidth - img_w) / 2;
//	}
//	if (win_w_h_ratio_ < img_w_h_ratio_)
//	{
//		// 图片比较宽， 上下留空
//		img_w = surfaceWidth;
//		img_h = img_w / img_w_h_ratio_;
//
//		v_boder = (surfaceHeight - img_h) / 2;
//		h_boder = 0;
//	}
//
//	QPoint mouse_p = event->pos();
//	x_on_img_ = mouse_p.x() - h_boder;
//	y_on_img_ = mouse_p.y() - v_boder;
//
//	if (x_on_img_ >= 0) {
//		x_on_img_ = x_on_img_*1.0 / img_w * pixelWidth;
//	}
//
//
//	if (y_on_img_ >= 0) {
//		y_on_img_ = y_on_img_*1.0 / img_h * pixelHeight;
//	}
//
//	if (true == is_offline_model_)
//	{
//		/// 离线模式，鼠标移动的时候更新鼠标当前位置信息
//		emit mainGLSigMouseMove();
//	}
//}
//
//void MainGLWidget::wheelEvent(QWheelEvent *event)
//{
//	int delta = event->delta();
//	if (delta>0)
//	{
//		scale_decrease_count = 0;
//		if (scale_add_count == 0)
//		{
//			scale_value = 1;
//			scale_add_count = 1;
//		}
//		else if (scale_add_count == 2)
//		{
//			scale_add_count = 1;
//			if (scale_value < 2.0f)
//			{
//				scale_value += 0.1;
//			}
//		}
//		else {
//			scale_add_count++;
//		}
//
//
//	}
//	else if (delta<0) {
//		scale_add_count = 0;
//		if (scale_decrease_count == 0)
//		{
//			scale_value = 1;
//			scale_decrease_count = 1;
//		}
//		else if (scale_decrease_count == 2)
//		{
//			scale_decrease_count = 1;
//			if (scale_value>0.4f)
//			{
//				scale_value -= 0.1;
//			}
//		}
//		else
//		{
//			scale_decrease_count++;
//		}
//
//
//	}
//	//qDebug() << "howard wheelEvent== " << scale_value;
//}
//
void MainGLWidget::keyPressEvent(QKeyEvent *event)
{
	float tempX{ 0 }, tempY{ 0 }, tempZ{ 0 };
	if (event->key() == Qt::Key_A) {
		tempX -= 10;


	}
	else if (event->key() == Qt::Key_D) {
		tempX += 10;

	}
	else if (event->key() == Qt::Key_W) {
		tempY -= 10;

	}
	else if (event->key() == Qt::Key_S) {
		tempY += 10;

	}
	else if (event->key() == Qt::Key_Space) {
		tempX = -camPosX;
		tempY = -camPosY;
		tempZ = -camPosZ;
		camPosZ = 0;
	}
	camPosX += tempX;
	camPosY += tempY;
	float vec3[] = { tempX * 3, tempY * 3, tempZ };
	pCloudLine.translateViewer(vec3);
	pCloudLineCoord.translateViewer(vec3);
	update();
}
void MainGLWidget::resetMousePos(int frameX, int frameY)
{
	mOnFrameX = frameX;
	mOnFrameY = frameY;
}
//
//void MainGLWidget::keyReleaseEvent(QKeyEvent *event)
//{
//	QWidget::keyPressEvent(event);
//
//
//}
//
//void MainGLWidget::mousePressEvent(QMouseEvent * event)
//{
//	if (event->button() == Qt::LeftButton)
//	{
//		mouse_left_x = event->x();
//		mouse_left_y = event->y();
//		mouse_left_press_ = true;
//	}
//}
//void MainGLWidget::mouseReleaseEvent(QMouseEvent *event) {
//	if (event->button() == Qt::LeftButton && mouse_left_press_)
//	{
//		int x = event->x();
//		int y = event->y();
//		int move_x = x - mouse_left_x;
//		int move_y = mouse_left_y - y;
//
//		float translate_x = 1.0*move_x * 2 / surfaceWidth;
//		float translate_y = 1.0*move_y * 2 / surfaceHeight;
//		translate_x = QString::number(translate_x, 'f', 1).toFloat();
//		translate_y = QString::number(translate_y, 'f', 1).toFloat();
//		//qDebug() << "howard move_x==" << translate_x << " move_y==" << translate_y;
//
//
//		mouse_left_press_ = false;
//	}
//}
