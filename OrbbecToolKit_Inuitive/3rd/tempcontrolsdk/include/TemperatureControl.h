#pragma once
#include "SerialPort.h"
#ifdef TemperatureControlSdk
#define TemperatureControlAPI _declspec(dllexport)
#else
#define TemperatureControlAPI  _declspec(dllimport)
#endif

class TemperatureControlAPI TemperatureControl
{
public:
	/*******************************
	<summary>
	  构造函数
	</summary>
	<params>
	  @param name="controlerType": 传入温度控制器类型,0温控仪, 1水冷平台
	</params>
	<returns>
	 无
	</returns>*********************/
	TemperatureControl(int controlerType = 0);
	~TemperatureControl();

	/*******************************
	<summary>
	  初始化串口
	</summary>
	<params>
	  @param name="params": 串口配置参数
	  波特率115200、无较验、停止位1 
	</params>
	<returns>
	  成功,返回true
	  失败,返回false
	</returns>*********************/
	bool initSerialPort(SerialParams params);
	
	/*******************************
	<summary>
	  开始温控
	</summary>
	<params>
	  @param name="": 
	</params>
	<returns>
	  成功,返回true
	  失败,返回false
	</returns>*********************/
	bool startTempControl();
	/*******************************
	<summary>
	  停止温控
	</summary>
	<params>
	  @param name="": 
	</params>
	<returns>
	  成功,返回true
	  失败,返回false
	</returns>*********************/
	bool stopTempControl();

	/*******************************
	<summary>
	  设置目标温度
	</summary>
	<params>
	  @param name="temp": 目标温度
	  @param name="channel": 通道,1,2,3,4,两通道分别控制rx,tx
	</params>
	<returns>
	  成功,返回true
	  失败,返回false
	</returns>*********************/
	bool setTemperature(float temp,int channel);

	/*******************************
	<summary>
	  获取温控器目前温度
	</summary>
	<params>
	  @param name="temp": 获取的温度
	  @param name="channel": 当前通道
	</params>
	<returns>
	  成功,返回true
	  失败,返回false
	</returns>*********************/
	bool getTemperature(float &temp, int channel);
	/*******************************
	<summary>
	  设置最高温度
	</summary>
	<params>
	  @param name="temp": 温度
	</params>
	<returns>
	  成功,返回true
	  失败,返回false
	</returns>*********************/
	bool setMaxTemp(float temp,int channel);
	/*******************************
	<summary>
	  设置最小控制温度
	</summary>
	<params>
	  @param name="temp": 温度
	</params>
	<returns>
	  成功,返回true
	  失败,返回false
	</returns>*********************/
	bool setMinTemp(float temp, int channel);



	

private:
	unsigned char bbcVerify(unsigned char *buf, unsigned int len);
	void temperatureToHex(float temp, char* dstHex);
	void getCmdForSetTemp(char* channel, float temp, char* dstCmd);
	bool setMaxMinTemp(bool isMax, float temp, int channel);
private:
	
	SerialPort m_serial_port;
	int m_controler_type = 0;//0:温控仪, 1:水冷平台
	
};

