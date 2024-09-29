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
#ifndef INICONFIG_H
#define INICONFIG_H

#include <QSettings>
#include <QMap>


struct DefaultRes
{
	QString depth_res;
	QString ir_res;
	QString phase_res;
	QString color_res;

	bool mOpenDepth;
	bool mOpenColor;
	bool mOpenIr;
	bool mOpenPhase;
	bool mHideDepth;
	bool mHideColor;
	bool mHideIr;
	bool mHidePhase;
	bool mHideAiStream;
	bool mHideUvcColor;
	bool mPassword;
	bool mStreamTest;
	int  mStreamMode;
	bool mFilterSetOnUI;

	DefaultRes()
	{
		depth_res.clear();
		ir_res.clear();
		phase_res.clear();
		color_res.clear();

		mOpenDepth = true;
		mOpenColor = true;
		mOpenIr = false;
		mOpenPhase = false;
		mHideDepth = false;
		mHideColor = false;
		mHideIr = false;
		mHidePhase = false;
		mHideAiStream = false;
		mHideUvcColor = false;
		mPassword = false;
		mStreamTest = false;
	    mFilterSetOnUI=false;
		mStreamMode = -1;
	}
};

class IniConfig
{
public:
	IniConfig();
	~IniConfig();
	int LoadIni(QString ini_file);
	void setPasswordValue(int value);
	DefaultRes config_res_;

private:
	void InitResMap();
	QSettings *settingsread = nullptr;
};

#endif // INICONFIG_H
