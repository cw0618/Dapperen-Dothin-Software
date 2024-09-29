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
	  ���캯��
	</summary>
	<params>
	  @param name="controlerType": �����¶ȿ���������,0�¿���, 1ˮ��ƽ̨
	</params>
	<returns>
	 ��
	</returns>*********************/
	TemperatureControl(int controlerType = 0);
	~TemperatureControl();

	/*******************************
	<summary>
	  ��ʼ������
	</summary>
	<params>
	  @param name="params": �������ò���
	  ������115200���޽��顢ֹͣλ1 
	</params>
	<returns>
	  �ɹ�,����true
	  ʧ��,����false
	</returns>*********************/
	bool initSerialPort(SerialParams params);
	
	/*******************************
	<summary>
	  ��ʼ�¿�
	</summary>
	<params>
	  @param name="": 
	</params>
	<returns>
	  �ɹ�,����true
	  ʧ��,����false
	</returns>*********************/
	bool startTempControl();
	/*******************************
	<summary>
	  ֹͣ�¿�
	</summary>
	<params>
	  @param name="": 
	</params>
	<returns>
	  �ɹ�,����true
	  ʧ��,����false
	</returns>*********************/
	bool stopTempControl();

	/*******************************
	<summary>
	  ����Ŀ���¶�
	</summary>
	<params>
	  @param name="temp": Ŀ���¶�
	  @param name="channel": ͨ��,1,2,3,4,��ͨ���ֱ����rx,tx
	</params>
	<returns>
	  �ɹ�,����true
	  ʧ��,����false
	</returns>*********************/
	bool setTemperature(float temp,int channel);

	/*******************************
	<summary>
	  ��ȡ�¿���Ŀǰ�¶�
	</summary>
	<params>
	  @param name="temp": ��ȡ���¶�
	  @param name="channel": ��ǰͨ��
	</params>
	<returns>
	  �ɹ�,����true
	  ʧ��,����false
	</returns>*********************/
	bool getTemperature(float &temp, int channel);
	/*******************************
	<summary>
	  ��������¶�
	</summary>
	<params>
	  @param name="temp": �¶�
	</params>
	<returns>
	  �ɹ�,����true
	  ʧ��,����false
	</returns>*********************/
	bool setMaxTemp(float temp,int channel);
	/*******************************
	<summary>
	  ������С�����¶�
	</summary>
	<params>
	  @param name="temp": �¶�
	</params>
	<returns>
	  �ɹ�,����true
	  ʧ��,����false
	</returns>*********************/
	bool setMinTemp(float temp, int channel);



	

private:
	unsigned char bbcVerify(unsigned char *buf, unsigned int len);
	void temperatureToHex(float temp, char* dstHex);
	void getCmdForSetTemp(char* channel, float temp, char* dstCmd);
	bool setMaxMinTemp(bool isMax, float temp, int channel);
private:
	
	SerialPort m_serial_port;
	int m_controler_type = 0;//0:�¿���, 1:ˮ��ƽ̨
	
};

