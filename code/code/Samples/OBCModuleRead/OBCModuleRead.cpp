#include <OpenNI.h>
#include <stdio.h>
#include <vector>
#include <memory>
#include "OniSampleUtilities.h"
#include "OniCTypes.h"
#include "OBCProperties.h"
#include <iostream>
#include <fstream>
#include <cstdlib>



#define SAMPLE_READ_WAIT_TIMEOUT 1000 //ms

using namespace openni;

typedef struct OBC_FILE_t
{
	uint32_t  size;
	unsigned char* buffer;
}OBC_FILE_t;

uint32_t GetBinSize(char *filename)
{
	uint32_t  siz = 0;
	FILE  *fp = NULL;
	fp = fopen(filename, "rb");
	if (fp)
	{
		fseek(fp, 0, SEEK_END);
		siz = ftell(fp);
		fclose(fp);
	}

	return siz;
}

bool read_bin(char *path, uint8_t *buf, uint32_t size)
{
	FILE *infile;
	infile = fopen(path, "rb");
	if (!infile)
	{
		return false;
	}
	uint32_t nActSize = (uint32_t)fread(buf, sizeof(uint8_t), size, infile);
	if (nActSize != size)
	{
		return false;
	}

	fclose(infile);

	return true;
}

bool read_OBC_FILE(char* fileName, OBC_FILE_t* obc_file)
{

	uint32_t  fwSize = GetBinSize(fileName);
	if (fwSize == 0)
	{
		printf("Error: file size is 0!\n");
		return false;
	}

	obc_file->size = fwSize;
	obc_file->buffer = (unsigned char *)malloc(sizeof(unsigned char)*fwSize);
	bool res = read_bin(fileName, obc_file->buffer, fwSize);
	if (!res)
	{
		printf("Error: read file failed!\n");
		return false;
	}

	return true;
}


//相位流测试
int main()
{
    //1.OpenNI::initialize
    Status rc = OpenNI::initialize();
    if (rc != STATUS_OK)
    {
        printf("Initialize failed\n%s\n", OpenNI::getExtendedError());
        return 1;
    }

    //2.Device open
    Device device;
    rc = device.open(ANY_DEVICE);
    if (rc != STATUS_OK)
    {
        printf("Couldn't open device\n%s\n", OpenNI::getExtendedError());
        return 2;
    }

    //get property
    uint32_t sensor_id = 0;
    rc = device.getProperty(OBC_SENSOR_ID, &sensor_id);
    printf("OBC_SENSOR_ID: %d, ret = %d\n", sensor_id, rc);

    uint8_t freq_mode = 0;
    rc = device.getProperty(OBC_FREQUENCY_MODE, &freq_mode);
    printf("OBC_FREQUENCY_MODE: %d, ret = %d\n", freq_mode, rc);
    
    uint8_t vltg = 0;
    rc = device.getProperty(OBC_VMGHI_VOLTAGE, &vltg);
    printf("OBC_VMGHI_VOLTAGE: %d, ret = %d\n", vltg, rc);
           
    ObTxABPower tx_a_b_power = { 0, 0 };
    int size = sizeof(tx_a_b_power);
    rc = device.getProperty(OBC_TX_A_B_POWER, (void *)&tx_a_b_power, &size);
    printf("default OBC_TX_A_B_POWER: a:%d,b:%d\n", tx_a_b_power.a_state, tx_a_b_power.b_state);
    
    uint32_t freq = 0;
    rc = device.getProperty(OBC_MODULATION_FREQUENCY, &freq);
    printf("OBC_MODULATION_FREQUENCY f1: %d, f2: %d\n", freq & 0xff, freq >> 8);

    uint32_t duty = 0; 
    rc = device.getProperty(OBC_DUTY_CYCLE, &duty);
    printf("OBC_DUTY_CYCLE duty1: %d, duty2: %d\n", duty & 0xff, duty >> 8);

    uint32_t integr = 0;
    rc = device.getProperty(OBC_INTEGRATION_TIME, &integr);
    printf("OBC_INTEGRATION_TIME integration time: %d\n", integr);

    //设置频率模式
    uint8_t freqmode = TOF_DUAL_FREQUENCY_MODE;
    //uint8_t freqmode = TOF_SINGLE_FREQUENCY_MODE;
    rc = device.setProperty(OBC_FREQUENCY_MODE, freqmode);//若在create前设置频率模式， 双频两个分辨率没问题，单频非binning也没问题，单频binning有问题
    printf("OBC_FREQUENCY_MODE freqmode: %d ret: %d\n", freqmode, rc);

    //若在create后设置频率模式 arrVM = info->getSupportedVideoModes() 为错误值

    //3.Create phase stream
    VideoStream phase;

    if (device.getSensorInfo(SENSOR_PHASE) != NULL)
    {
        rc = phase.create(device, SENSOR_PHASE);
        if (rc != STATUS_OK)
        {
            printf("Couldn't create phase stream\n%s\n", OpenNI::getExtendedError());
            return 3;
        }
    }    
    
    //分辨率设置调试
    VideoMode vm = phase.getVideoMode();

    const SensorInfo *info = device.getSensorInfo(SENSOR_PHASE);

    const Array<VideoMode> &arrVM = info->getSupportedVideoModes();

    const int selectNum = 1;
    vm.setResolution(arrVM[selectNum].getResolutionX(), arrVM[selectNum].getResolutionY());
    vm.setFps(arrVM[selectNum].getFps());
    vm.setPixelFormat(arrVM[selectNum].getPixelFormat());

    //vm.setResolution(960, 1440);//成功

    rc = phase.setVideoMode(vm);
    printf("setVideoMode %d\n", rc);

    rc = phase.start();
    if (rc != STATUS_OK)
    {
        printf("Couldn't start the phase stream\n%s\n", OpenNI::getExtendedError());
        return 4;
    }

    VideoFrameRef frame;
    
    while (!wasKeyboardHit())
    {
        int changedStreamDummy;
        VideoStream* pStream = &phase;
        rc = OpenNI::waitForAnyStream(&pStream, 1, &changedStreamDummy, SAMPLE_READ_WAIT_TIMEOUT);
        if (rc != STATUS_OK)
        {
            printf("Wait failed! (timeout is %d ms)\n%s\n", SAMPLE_READ_WAIT_TIMEOUT, OpenNI::getExtendedError());
            continue;
        }

        rc = phase.readFrame(&frame);
        if (rc != STATUS_OK)
        {
            printf("Read failed!\n%s\n", OpenNI::getExtendedError());
            continue;   
        }

        if (frame.getVideoMode().getPixelFormat() != PIXEL_FORMAT_GRAY16)
        {
            printf("Unexpected frame format\n");
            continue;
        }

        Grayscale16Pixel* pPhase = (Grayscale16Pixel*)frame.getData();

        Metadata meta = frame.getMetadata();

        int middleIndex = (frame.getHeight() + 1)*frame.getWidth() / 2;

        printf("[%08llu] %8d\n", frame.getTimestamp(), pPhase[middleIndex]);
    }

    phase.stop();
    phase.destroy();
    device.close();
    OpenNI::shutdown();
    
    return 0;
}



