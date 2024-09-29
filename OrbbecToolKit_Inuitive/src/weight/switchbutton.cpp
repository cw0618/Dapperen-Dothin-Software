#include "switchbutton.h"
#include "qpainter.h"
#include "qevent.h"
#include "qtimer.h"
#include "qdebug.h"

SwitchButton::SwitchButton(QWidget *parent) : QWidget(parent)
{
	mChecked = false;
	mButtonStyle = kButtonStyleRect;

	mBgColorOff = QColor(195, 213, 229);
	mBgColorOn = QColor(250, 250, 250);

	mSliderColorOff = QColor(36, 44, 51);
	mSliderColorOn = QColor(100, 184, 255);

	mTextColorOff = QColor(255, 0, 0);
	mTextColorOn = QColor(10, 10, 10);

	mTextOff = "";
	mTextOn = "";

	mImageOff = "://res/image/turn_off.png";
	mImageOn = "://res/image/turn_on.png";

	mSpace = 2;
	mRectRadius = 5;

	mStep = width() / 50;
	mStartX = 0;
	mEndX = 0;

	mTimer = new QTimer(this);
	mTimer->setInterval(5);
	connect(mTimer, SIGNAL(timeout()), this, SLOT(updateValue()));

	setFont(QFont("Microsoft Yahei", 10));
}

SwitchButton::~SwitchButton()
{

}

void SwitchButton::mousePressEvent(QMouseEvent *)
{
	mChecked = !mChecked;
	emit checkedChanged(mChecked);

	//姣忔绉诲姩鐨勬闀夸负瀹藉害鐨?50鍒嗕箣涓€
	mStep = width() / 50;

	//鐘舵€佸垏鎹㈡敼鍙樺悗鑷姩璁＄畻缁堢偣鍧愭爣
	if (mChecked) {
		if (mButtonStyle == kButtonStyleRect) {
			mEndX = width() - width() / 2;
		}
		else if (mButtonStyle == kButtonStyleCircleIn) {
			mEndX = width() - height();
		}
		else if (mButtonStyle == kButtonStyleCircleOut) {
			mEndX = width() - height() + mSpace;
		}
	}
	else {
		mEndX = 0;
	}

	mTimer->start();
}

void SwitchButton::resizeEvent(QResizeEvent *)
{
	//姣忔绉诲姩鐨勬闀夸负瀹藉害鐨?50鍒嗕箣涓€
	mStep = width() / 50;

	//灏哄澶у皬鏀瑰彉鍚庤嚜鍔ㄨ缃捣鐐瑰潗鏍囦负缁堢偣
	if (mChecked) {
		if (mButtonStyle == kButtonStyleRect) {
			mStartX = width() - width() / 2;
		}
		else if (mButtonStyle == kButtonStyleCircleIn) {
			mStartX = width() - height();
		}
		else if (mButtonStyle == kButtonStyleCircleOut) {
			mStartX = width() - height() + mSpace;
		}
	}
	else {
		mStartX = 0;
	}

	update();
}

void SwitchButton::paintEvent(QPaintEvent *)
{
	//缁樺埗鍑嗗宸ヤ綔,鍚敤鍙嶉敮榻?
	QPainter painter(this);
	painter.setRenderHint(QPainter::Antialiasing);

	if (mButtonStyle == kButtonStyleImage) {
		//缁樺埗鍥剧墖
		drawImage(&painter);
	}
	else {
		//缁樺埗鑳屾櫙
		drawBg(&painter);
		//缁樺埗婊戝潡
		drawSlider(&painter);
		//缁樺埗鏂囧瓧
		drawText(&painter);
	}
}

void SwitchButton::drawBg(QPainter *painter)
{
	painter->save();
	painter->setPen(Qt::NoPen);

	if (!mChecked) {
		painter->setBrush(mBgColorOff);
	}
	else {
		painter->setBrush(mBgColorOn);
	}

	if (mButtonStyle == kButtonStyleRect) {
		painter->drawRoundedRect(rect(), mRectRadius, mRectRadius);
	}
	else if (mButtonStyle == kButtonStyleCircleIn) {
		QRect rect(0, 0, width(), height());
		//鍗婂緞涓洪珮搴︾殑涓€鍗?
		int radius = rect.height() / 2;
		//鍦嗙殑瀹藉害涓洪珮搴?
		int circleWidth = rect.height();

		QPainterPath path;
		path.moveTo(radius, rect.left());
		path.arcTo(QRectF(rect.left(), rect.top(), circleWidth, circleWidth), 90, 180);
		path.lineTo(rect.width() - radius, rect.height());
		path.arcTo(QRectF(rect.width() - rect.height(), rect.top(), circleWidth, circleWidth), 270, 180);
		path.lineTo(radius, rect.top());

		painter->drawPath(path);
	}
	else if (mButtonStyle == kButtonStyleCircleOut) {
		QRect rect(mSpace, mSpace, width() - mSpace * 2, height() - mSpace * 2);
		painter->drawRoundedRect(rect, mRectRadius, mRectRadius);
	}

	painter->restore();
}

void SwitchButton::drawSlider(QPainter *painter)
{
	painter->save();
	painter->setPen(Qt::NoPen);

	if (!mChecked) {
		painter->setBrush(mSliderColorOff);
	}
	else {
		painter->setBrush(mSliderColorOn);
	}

	if (mButtonStyle == kButtonStyleRect) {
		int sliderWidth = width() / 2 - mSpace * 2;
		int sliderHeight = height() - mSpace * 2;
		QRect sliderRect(mStartX + mSpace, mSpace, sliderWidth, sliderHeight);
		painter->drawRoundedRect(sliderRect, mRectRadius, mRectRadius);
	}
	else if (mButtonStyle == kButtonStyleCircleIn) {
		QRect rect(0, 0, width(), height());
		int sliderWidth = rect.height() - mSpace * 2;
		QRect sliderRect(mStartX + mSpace, mSpace, sliderWidth, sliderWidth);
		painter->drawEllipse(sliderRect);
	}
	else if (mButtonStyle == kButtonStyleCircleOut) {
		QRect rect(0, 0, width() - mSpace, height() - mSpace);
		int sliderWidth = rect.height();
		QRect sliderRect(mStartX, mSpace / 2, sliderWidth, sliderWidth);
		painter->drawEllipse(sliderRect);
	}

	painter->restore();
}

void SwitchButton::drawText(QPainter *painter)
{
	painter->save();

	if (!mChecked) {
		painter->setPen(mTextColorOff);
		painter->drawText(width() / 2, 0, width() / 2 - mSpace, height(), Qt::AlignCenter, mTextOff);
	}
	else {
		painter->setPen(mTextColorOn);
		painter->drawText(0, 0, width() / 2 + mSpace * 2, height(), Qt::AlignCenter, mTextOn);
	}

	painter->restore();
}

void SwitchButton::drawImage(QPainter *painter)
{
	painter->save();

	QPixmap pix;

	if (!mChecked) {
		pix = QPixmap(mImageOff);
	}
	else {
		pix = QPixmap(mImageOn);
	}

	//鑷姩绛夋瘮渚嬪钩婊戠缉鏀惧眳涓樉绀?
	int targetWidth = pix.width();
	int targetHeight = pix.height();
	pix = pix.scaled(targetWidth, targetHeight, Qt::KeepAspectRatio, Qt::SmoothTransformation);

	int pixX = rect().center().x() - targetWidth / 2;
	int pixY = rect().center().y() - targetHeight / 2;
	QPoint point(pixX, pixY);
	painter->drawPixmap(point, pix);

	painter->restore();
}

void SwitchButton::updateValue()
{
	if (mChecked) {
		if (mStartX < mEndX) {
			mStartX = mStartX + mStep;
		}
		else {
			mStartX = mEndX;
			mTimer->stop();
		}
	}
	else {
		if (mStartX > mEndX) {
			mStartX = mStartX - mStep;
		}
		else {
			mStartX = mEndX;
			mTimer->stop();
		}
	}

	update();
}

void SwitchButton::setChecked(bool checked)
{
	if (this->mChecked != checked) {
		this->mChecked = checked;
		//emit checkedChanged(checked);
		//姣忔绉诲姩鐨勬闀夸负瀹藉害鐨?50鍒嗕箣涓€
		mStep = width() / 50;

		//鐘舵€佸垏鎹㈡敼鍙樺悗鑷姩璁＄畻缁堢偣鍧愭爣
		if (checked) {
			if (mButtonStyle == kButtonStyleRect) {
				mEndX = width() - width() / 2;
			}
			else if (mButtonStyle == kButtonStyleCircleIn) {
				mEndX = width() - height();
			}
			else if (mButtonStyle == kButtonStyleCircleOut) {
				mEndX = width() - height() + mSpace;
			}
		}
		else {
			mEndX = 0;
		}

		mTimer->start();
		update();
	}
}

void SwitchButton::setButtonStyle(SwitchButton::ButtonStyle buttonStyle)
{
	this->mButtonStyle = buttonStyle;
	update();
}

void SwitchButton::setBgColor(QColor bgColorOff, QColor bgColorOn)
{
	this->mBgColorOff = bgColorOff;
	this->mBgColorOn = bgColorOn;
	update();
}

void SwitchButton::setSliderColor(QColor sliderColorOff, QColor sliderColorOn)
{
	this->mSliderColorOff = sliderColorOff;
	this->mSliderColorOn = sliderColorOn;
	update();
}

void SwitchButton::setTextColor(QColor textColorOff, QColor textColorOn)
{
	this->mTextColorOff = textColorOff;
	this->mTextColorOn = textColorOn;
	update();
}

void SwitchButton::setText(QString textOff, QString textOn)
{
	this->mTextOff = textOff;
	this->mTextOn = textOn;
	update();
}

void SwitchButton::setImage(QString imageOff, QString imageOn)
{
	this->mImageOff = imageOff;
	this->mImageOn = imageOn;
	update();
}

void SwitchButton::setSpace(int space)
{
	this->mSpace = space;
	update();
}

void SwitchButton::setRectRadius(int rectRadius)
{
	this->mRectRadius = rectRadius;
	update();
}

