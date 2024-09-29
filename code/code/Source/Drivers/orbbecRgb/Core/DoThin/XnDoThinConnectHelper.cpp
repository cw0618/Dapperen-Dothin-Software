#include "XnDothinConnectHelper.h"
#include "XnDothinStatus.h"



XnDothinConnectHelper::XnDothinConnectHelper() :dothin_id(-1), i2c_write(1)
{
	xnOSMemSet(&sensor_Config, 0, sizeof(SensorTab));
	xnOSMemSet(&i2c_Config, 0, sizeof(i2c_power_t));
}

XnStatus XnDothinConnectHelper::DeviceOpen(const char *pszDeviceName)
{
	XnInt32 iDevID = 0;
	XnInt res = OpenDevice(pszDeviceName, &dothin_id, iDevID);
	currend_id = dothin_id;
	xnLogInfo(XN_MASK_DOTHIN_CONNECT, "Successful opening rgb device , dothin_devId: %d", dothin_id);

	return XN_STATUS_OK;
}


XnStatus XnDothinConnectHelper::LoadSensorConfiguration()
{
	XnStatus res = InitSensorConfig();
	XN_IS_STATUS_OK(res);

	res = ConfigSensor();
	XN_IS_STATUS_OK(res);

	xnLogWarning(XN_MASK_DOTHIN_CONNECT, "Dothin ConnectHelper load sensor configuration success.");

	return XN_STATUS_OK;
}


XnStatus XnDothinConnectHelper::InitSensorConfig()
{
	if (dothin_id == -1)
	{
		xnLogError(XN_MASK_DOTHIN_CONNECT, "Failed to initialize Dothin sensor configuration, failed to find open Dothin device...");
		return XN_STATUS_ERROR;
	}

	sensor_Config.width = 640;
	sensor_Config.height = 480;
	sensor_Config.type = 7;
	sensor_Config.port = 0;

	sensor_Config.pin = 3;
	sensor_Config.SlaveID = 0x20;

	sensor_Config.mode = 3;
	sensor_Config.FlagReg = 0x0000;

	sensor_Config.FlagMask = 0xff;
	sensor_Config.FlagData = 0x63;

	sensor_Config.FlagReg1 = 0x0001;
	sensor_Config.FlagMask1 = 0xff;

	sensor_Config.FlagData1 = 0;
	sensor_Config.outformat = 2;
	sensor_Config.mclk = 24;

	i2c_Config.avdd = 3300;
	i2c_Config.dovdd = 1800;
	i2c_Config.dvdd = 1200;
	i2c_Config.afvcc = 2800;

	sensor_Config.ParaList = NULL;
	sensor_Config.ParaListSize = 0;
	sensor_Config.SleepParaList = NULL;
	sensor_Config.SleepParaListSize = 0;

	return XN_STATUS_OK;
}


XnStatus XnDothinConnectHelper::ConfigSensor()
{
	if (dothin_id == -1)
	{
		xnLogError(XN_MASK_DOTHIN_CONNECT, "Failed to config Dothin sensor configuration, failed to find open Dothin device...");
		return XN_STATUS_ERROR;
	}

	XnStatus ret = XN_STATUS_OK;

	ret = InitDiskInfo();
	XN_IS_STATUS_OK(ret);

	ret = InitRoi(0, 0, sensor_Config.width, sensor_Config.height, 0, 0, 1, 1, sensor_Config.type, TRUE, dothin_id);
	XN_IS_DOTHINSTATUS_OK(ret);
	if (ret != XN_STATUS_OK)
	{
		xnLogError(XN_MASK_DOTHIN_CONNECT, "Dothin ConnectHelper init roi error: %d", ret);
	}
	xnLogWarning(XN_MASK_DOTHIN_CONNECT, "Dothin ConnectHelper init roi success...");


	ret = SetSensorPort(sensor_Config.port, sensor_Config.width, sensor_Config.height, dothin_id);
	XN_IS_DOTHINSTATUS_OK(ret);
	if (ret != XN_STATUS_OK)
	{
		xnLogError(XN_MASK_DOTHIN_CONNECT, "Dothin ConnectHelper Set SensorPort error: %d", ret);
	}
	xnLogWarning(XN_MASK_DOTHIN_CONNECT, "Dothin ConnectHelper Set SensorPort success...");


	//Sleep(10);

	unsigned long  grabSize;
	ret = CalculateGrabSize(&grabSize, dothin_id);
	XN_IS_DOTHINSTATUS_OK(ret);
	if (ret != XN_STATUS_OK)
	{
		xnLogError(XN_MASK_DOTHIN_CONNECT, "Dothin ConnectHelper CalculateGrabSize error: %d", ret);
	}
	xnLogWarning(XN_MASK_DOTHIN_CONNECT, "Dothin ConnectHelper CalculateGrabSize success");

	ret = OpenVideo(grabSize, dothin_id);
	XN_IS_DOTHINSTATUS_OK(ret);
	if (ret != XN_STATUS_OK)
	{
		xnLogError(XN_MASK_DOTHIN_CONNECT, "Dothin ConnectHelper OpenVideo error: %d", ret);
	}
	xnLogWarning(XN_MASK_DOTHIN_CONNECT, "Dothin ConnectHelper OpenVideo success...");

	ret = InitIsp(sensor_Config.width, sensor_Config.height, sensor_Config.type, CHANNEL_A, dothin_id);
	XN_IS_DOTHINSTATUS_OK(ret);
	if (ret != XN_STATUS_OK)
	{
		xnLogError(XN_MASK_DOTHIN_CONNECT, "Dothin ConnectHelper InitIsp error: %d", ret);
	}
	xnLogWarning(XN_MASK_DOTHIN_CONNECT, "Dothin ConnectHelper InitIsp success...");


	MasterSpiConfig_t SPIConfig;
	SPIConfig.fMhz = 10;
	SPIConfig.byWordLen = 0;
	SPIConfig.byCtrl = 0;

	ret = MasterSpiConfig(2, &SPIConfig, dothin_id);
	XN_IS_DOTHINSTATUS_OK(ret);
	if (ret != XN_STATUS_OK)
	{
		xnLogError(XN_MASK_DOTHIN_CONNECT, "Dothin ConnectHelper MasterSpiConfig error: %d", ret);
	}
	xnLogWarning(XN_MASK_DOTHIN_CONNECT, "Dothin ConnectHelper MasterSpiConfig success...");

	xnLogWarning(XN_MASK_DOTHIN_CONNECT, "Dothin ConnectHelper ConfigSensor completed.");
	return XN_STATUS_OK;
}


XnStatus XnDothinConnectHelper::InitDiskInfo()
{
	if (dothin_id == -1)
	{
		xnLogError(XN_MASK_DOTHIN_CONNECT, "Failed to initialize disk info, failed to find open Dothin device...");
		return XN_STATUS_ERROR;
	}

	XnStatus ret = XN_STATUS_OK;

	ret = SetSoftPinPullUp(IO_NOPULL, dothin_id);
	XN_IS_DOTHINSTATUS_OK(ret);

	/*first set sensor working condition....*/
	{

		//first set pin definition...
		{
			XnUInt8  pinDef[40] = { 0 };

			//mipi....
			if (sensor_Config.port == PORT_MIPI || sensor_Config.port == PORT_HISPI)
			{
				pinDef[0] = 20;
				pinDef[1] = 0;
				pinDef[2] = 2;
				pinDef[3] = 1;
				pinDef[4] = 3;
				pinDef[5] = 4;
				pinDef[6] = 5;
				pinDef[7] = 6;
				pinDef[8] = 7;
				pinDef[9] = 8;
				pinDef[10] = 9;
				pinDef[11] = 20;
				pinDef[12] = 10;
				pinDef[13] = 11;
				pinDef[14] = 12;
				pinDef[15] = 20;
				pinDef[16] = 20;
				pinDef[17] = 13;
				pinDef[18] = 15;
				pinDef[19] = 14;
				pinDef[20] = 19;
				pinDef[21] = 18;
				/*
				pinDef[22] = 20;
				pinDef[23] = 16;
				pinDef[24] = 20;
				pinDef[25] = 20;
				*/
				/*
				pinDef[22] = PIN_SPI_SCK;
				pinDef[23] = PIN_SPI_CS;
				pinDef[24] = PIN_SPI_SDO;
				pinDef[25] = PIN_SPI_SDI;
				*/
				pinDef[22] = PIN_SPI_SCK;
				pinDef[23] = PIN_SPI_CS;
				pinDef[24] = PIN_SPI_SDI;
				pinDef[25] = PIN_SPI_SDO;

			}
			else  //standard parallel..
			{
				//20140317 closed .

				pinDef[0] = 16;
				pinDef[1] = 0;
				pinDef[2] = 2;
				pinDef[3] = 1;
				pinDef[4] = 3;
				pinDef[5] = 4;
				pinDef[6] = 5;
				pinDef[7] = 6;
				pinDef[8] = 7;
				pinDef[9] = 8;
				pinDef[10] = 9;
				pinDef[11] = 20;
				pinDef[12] = 10;
				pinDef[13] = 11;
				pinDef[14] = 12;
				pinDef[15] = 20;
				pinDef[16] = 20;
				pinDef[17] = 20;
				pinDef[18] = 20;
				pinDef[19] = 20;
				pinDef[20] = 13;
				pinDef[21] = 20;
				pinDef[22] = 14;
				pinDef[23] = 15;
				pinDef[24] = 18;
				pinDef[25] = 19;
			}
		
			ret = SetSoftPin(pinDef, dothin_id);
		}

		ret = EnableSoftPin(TRUE, dothin_id);
		XN_IS_DOTHINSTATUS_OK(ret);

		ret = EnableGpio(TRUE, dothin_id);
		XN_IS_DOTHINSTATUS_OK(ret);

		//set voltage and mclk.....
		float fvpp = 0;
		float fmclk = sensor_Config.mclk / 1000.0f;//ini_getf("Sensor", "mclk", 24, filename);
		xnLogWarning(XN_MASK_DOTHIN_CONNECT, "Dothin ConnectHelper avdd: %f, dovdd: %f, dvdd: %f", i2c_Config.avdd, i2c_Config.dovdd, i2c_Config.dvdd);

		SENSOR_POWER Power[10];
		XnInt voltage[10];
		XnInt Current[10];
		BOOL OnOff[10];
		CURRENT_RANGE range[5];

		Power[0] = POWER_AVDD;
		voltage[0] = (XnInt)i2c_Config.avdd; // 2.8V
		Current[0] = 300; // 300mA
		OnOff[0] = TRUE;
		range[0] = CURRENT_RANGE_MA;

		Power[1] = POWER_DOVDD;
		voltage[1] = (XnInt)i2c_Config.dovdd; // 1.8V
		Current[1] = 300; // 300mA
		OnOff[1] = TRUE;
		range[1] = CURRENT_RANGE_MA;

		Power[2] = POWER_DVDD;
		voltage[2] = (XnInt)i2c_Config.dvdd;// 1.2V
		Current[2] = 300;// 300mA
		OnOff[2] = TRUE;
		range[2] = CURRENT_RANGE_MA;

		Power[3] = POWER_AFVCC;
		voltage[3] = (XnInt)i2c_Config.afvcc; // 2.8V
		Current[3] = 300; // 300mA
		OnOff[3] = TRUE;
		range[3] = CURRENT_RANGE_MA;

		Power[4] = POWER_VPP;
		voltage[4] = (XnInt)(fvpp * 1000);
		Current[4] = 300; // 300mA
		OnOff[4] = FALSE;
		range[4] = CURRENT_RANGE_MA;

		ret = PmuSetVoltage(Power, voltage, 4, dothin_id);
		XN_IS_DOTHINSTATUS_OK(ret);
		xnLogWarning(XN_MASK_DOTHIN_CONNECT, "Dothin ConnectHelper voltage: %d, %d, %d, %d, %d", voltage[0], voltage[1], voltage[2], voltage[3], voltage[4]);

		ret = PmuSetOnOff(Power, OnOff, 4, dothin_id);
		XN_IS_DOTHINSTATUS_OK(ret);

		ret = SetSensorClock(TRUE, (USHORT)(fmclk * 10), dothin_id);
		XN_IS_DOTHINSTATUS_OK(ret);

		ret = SetSoftPinPullUp(IO_PULLUP, dothin_id);
		XN_IS_DOTHINSTATUS_OK(ret);
	}

	//Sleep(10);

	//i2C init....
	ret = SetSensorI2cRate(I2C_100K, dothin_id);
	XN_IS_DOTHINSTATUS_OK(ret);

	ret = SensorEnable(sensor_Config.pin ^ 0x02, 1, dothin_id); //reset
	XN_IS_DOTHINSTATUS_OK(ret);

	//Sleep(20);
	ret = SensorEnable(sensor_Config.pin, 1, dothin_id);
	XN_IS_DOTHINSTATUS_OK(ret);
	//Sleep(50);

	//init sensor....
	if (i2c_write)
	{
		i2c_write = false;
		//InitSensor(sensor_config_.SlaveID, sensor_config_.ParaList, sensor_config_.ParaListSize, sensor_config_.mode, id);
		//CHECK_DOTHIN_STATUS(rtn);
	}

	if (strcmp(sensor_Config.name, "MX6300") == 0)
	{

#if 0
		SPITest();
		log.Append("Reset Chip\r\n");
		m_tbLog.SetWindowText(log);
		m_tbLog.LineScroll(m_tbLog.GetLineCount());
#endif

		xnLogWarning(XN_MASK_DOTHIN_CONNECT, "Dothin ConnectHelper reset  chip \n");
	}

	return XN_STATUS_OK;
}

XnStatus XnDothinConnectHelper::SetConfigType(XnInt bits)
{
    // 这些值定义在度信 imagekit.h
    //sensor中的mipi pack 对应设置度信的type
    switch (bits) {
    case 8: sensor_Config.type = D_MIPI_RAW8;
        break;
    case 10: sensor_Config.type = D_MIPI_RAW10;
        break;
    case 12: sensor_Config.type = D_MIPI_RAW12;
        break;
    case 16: sensor_Config.type = D_MIPI_RAW16;
        break;
    }
    return XN_STATUS_OK;
}


XnStatus XnDothinConnectHelper::Restart(XnUInt32 XRes, XnUInt32 YRes, XnUInt32* disBit)
{
	if (dothin_id == -1)
	{
		xnLogError(XN_MASK_DOTHIN_CONNECT, "Failed to restart, failed to find open Dothin device...");
		return XN_STATUS_ERROR;
	}

	XnInt rtn = CloseVideo(dothin_id);
	XN_IS_DOTHINSTATUS_OK(rtn);

    /*sensor_Config.type = 1;//原先IR，这里为1，即mipi 8位，应该在stream类中赋值，待确定会不会影响到ir   GetSensorInputFormatFromDot() 确认下InputFormat是否为8
    XnUInt32 framesize = XRes * YRes* (*disBit) / 8;
    sensor_Config.height = YRes;
    sensor_Config.width = framesize / YRes;
    */

    SetConfigType(*disBit);

    sensor_Config.width = XRes;
    sensor_Config.height = YRes;

	rtn = ResetFrameBuffer(dothin_id);
	XN_IS_DOTHINSTATUS_OK(rtn);

	rtn = InitRoi(0, 0, sensor_Config.width, sensor_Config.height, 0, 0, 1, 1, sensor_Config.type, TRUE, dothin_id);
	XN_IS_DOTHINSTATUS_OK(rtn);

	rtn = SetSensorPort(sensor_Config.port, sensor_Config.width, sensor_Config.height, dothin_id);
    XN_IS_DOTHINSTATUS_OK(rtn);

	//Sleep(10);
	ULONG m_GrabSize = 0;
	rtn = CalculateGrabSize(&m_GrabSize, dothin_id);
	XN_IS_DOTHINSTATUS_OK(rtn);
	xnLogWarning(XN_MASK_DOTHIN_CONNECT, "CalculateGrabSize: %d, GrabWidth: %d, GrabHeight: %d, dothinMipiPackBit: %d", 
        m_GrabSize, sensor_Config.width, sensor_Config.height, *disBit);

    //rtn = SetRoiEx(0, 0, sensor_Config.width, sensor_Config.height, 0, 0, 1, 1, false, dothin_id);//先注释掉，后续看下tof需不需要这个设置
    //rtn = SetGrabFrameSizeEx(true, m_GrabSize, dothin_id);

	rtn = OpenVideo(m_GrabSize, dothin_id);
	XN_IS_DOTHINSTATUS_OK(rtn);

	rtn = InitIsp(sensor_Config.width, sensor_Config.height, sensor_Config.type, CHANNEL_A, dothin_id);
	XN_IS_DOTHINSTATUS_OK(rtn);
	xnLogWarning(XN_MASK_DOTHIN_CONNECT, "ReStart: %d X %d: m_GrabSize: %d\n", sensor_Config.width, sensor_Config.height, m_GrabSize);

	return XN_STATUS_OK;
}


XnStatus XnDothinConnectHelper::ReadFrame(XnUChar* buf, XnInt32 GrabSize, unsigned long * framesize)
{
	if (dothin_id <0)
	{
		xnLogError(XN_MASK_DOTHIN_CONNECT, "Read frame failed, Dothin_id is invalid...");
		return XN_STATUS_ERROR;
	}

	XnInt rtn = GrabFrame((BYTE*)buf, GrabSize, framesize, &frameInfo, dothin_id);
	//xnLogError(XN_MASK_DOTHIN_CONNECT, "howard dothin_id=%d,GrabSize=%d,framesize=%d,ret=%d", dothin_id, GrabSize, *framesize, rtn);
	if (rtn != DT_ERROR_OK)
	{
		xnLogError(XN_MASK_DOTHIN_CONNECT, "Dothin Grab frame failed %d...", rtn);
		XN_IS_DOTHINSTATUS_OK(rtn);
	}

	return XN_STATUS_OK;
}

XnStatus XnDothinConnectHelper::DeviceClose()
{

	XnInt rtn = CloseDevice(currend_id);
		if (rtn != DT_ERROR_OK)
		{
			xnLogError(XN_MASK_DOTHIN_CONNECT, "Dothin close device failed %d...", rtn);
		}
		dothin_id = -1;
		xnLogInfo(XN_MASK_DOTHIN_CONNECT, "DeviceClose , rgb dothin_devId: %d", currend_id);
	return XN_STATUS_OK;
}


XnDothinConnectHelper::~XnDothinConnectHelper()
{
	xnOSMemSet(&sensor_Config, 0, sizeof(SensorTab));
	xnOSMemSet(&i2c_Config, 0, sizeof(i2c_power_t));

	this->dothin_id = -1;
}
