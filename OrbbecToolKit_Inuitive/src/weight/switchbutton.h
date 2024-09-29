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
#ifndef SWITCHBUTTON_H
#define SWITCHBUTTON_H

#include <QWidget>

class QTimer;

class SwitchButton : public QWidget
{
	Q_OBJECT
public:
	enum ButtonStyle {
		kButtonStyleRect = 0,     /**< 圆角矩形 */
		kButtonStyleCircleIn = 1, /**< 内圆形 */
		kButtonStyleCircleOut = 2,/**< 外圆形 */
		kButtonStyleImage = 3     /**< 图片 */

	};

	SwitchButton(QWidget *parent = 0);
	~SwitchButton();

protected:
	void mousePressEvent(QMouseEvent *);
	void resizeEvent(QResizeEvent *);
	void paintEvent(QPaintEvent *);
	void drawBg(QPainter *painter);
	void drawSlider(QPainter *painter);
	void drawText(QPainter *painter);
	void drawImage(QPainter *painter);

private:
	/** 是否选中 */
	bool mChecked;
	/** 开关按钮样式 */
	ButtonStyle mButtonStyle;
	/** 关闭时背景颜色 */
	QColor mBgColorOff;
	/** 打开时背景颜色 */
	QColor mBgColorOn;
	/** 关闭时滑块颜色 */
	QColor mSliderColorOff;
	/** 打开时滑块颜色 */
	QColor mSliderColorOn;
	/** 关闭时文本颜色*/
	QColor mTextColorOff;
	/** 打开时文本颜色 */
	QColor mTextColorOn;
	/** 关闭时显示的文字 */
	QString mTextOff;
	/** 打开时显示的文字 */
	QString mTextOn;
	/** 关闭时显示的图片 */
	QString mImageOff;
	/** 打开时显示的图片 */
	QString mImageOn;
	/** 滑块离背景间隔 */
	int mSpace;
	/** 圆角角度 */
	int mRectRadius;
	/** 每次移动的步长 */
	int mStep;
	/** 滑块开始X轴坐标 */
	int mStartX;
	/** 滑块结束X轴坐标 */
	int mEndX;
	/** 定时器绘制 */
	QTimer *mTimer;

	private slots:
	void updateValue();

public:
	bool getChecked()const
	{
		return mChecked;
	}
	ButtonStyle getButtonStyle()const
	{
		return mButtonStyle;
	}

	QColor getBgColorOff()const
	{
		return mBgColorOff;
	}
	QColor getBgColorOn()const
	{
		return mBgColorOn;
	}

	QColor getSliderColorOff()const
	{
		return mSliderColorOff;
	}
	QColor getSliderColorOn()const
	{
		return mSliderColorOn;
	}

	QColor getTextColorOff()const
	{
		return mTextColorOff;
	}
	QColor getTextColorOn()const
	{
		return mTextColorOn;
	}

	QString getTextOff()const
	{
		return mTextOff;
	}
	QString getTextOn()const
	{
		return mTextOn;
	}

	QString getImageOff()const
	{
		return mImageOff;
	}
	QString getImageOn()const
	{
		return mImageOn;
	}

	int getSpace()const
	{
		return mSpace;
	}
	int getRectRadius()const
	{
		return mRectRadius;
	}

	public slots:
	/**
	* setChecked 设置是否选中
	* \param checked
	*
	*/
	void setChecked(bool checked);
	/**
	* setButtonStyle 设置风格样式
	* \param buttonStyle
	*
	*/
	void setButtonStyle(ButtonStyle buttonStyle);
	/**
	* setChecked 设置背景颜色
	* \param bgColorOff
	*
	*/
	void setBgColor(QColor bgColorOff, QColor bgColorOn);
	/**
	* setSliderColor 设置滑块颜色
	* \param checked
	*
	*/
	void setSliderColor(QColor sliderColorOff, QColor sliderColorOn);
	/**
	* setSliderColor 设置文本颜色
	* \param textColorOff
	*
	*/
	void setTextColor(QColor textColorOff, QColor textColorOn);
	/**
	* setText 设置文本
	* \param textOff
	*
	*/
	void setText(QString textOff, QString textOn);
	/**
	* setSliderColor 设置背景图片
	* \param imageOff
	*
	*/
	void setImage(QString imageOff, QString imageOn);
	/**
	* setSpace 设置间隔
	* \param space
	*
	*/
	void setSpace(int space);
	/**
	* setRectRadius 设置圆角角度
	* \param rectRadius
	*
	*/
	void setRectRadius(int rectRadius);


signals:
	void checkedChanged(bool checked);
};

#endif // SWITCHBUTTON_H
