#include "XnDothinProtocol.h"
#include "XnLog.h"
#include "imagekit.h"
#include "dtccm2.h"
#include "XnDothinStatus.h"
#include "XnMx6xSLog.h"

#define XN_MASK_DOTHIN_PROTOCOL "XnDothinProtocol"

#define MX6300_CMD_ID_DEVICE_ID     0x90
#define MX6300_CMD_ID_FAST_READ     0x0B
#define MX6300_CMD_ID_PAGE_PROGRAM  0x02
#define MX6300_CMD_ID_STATUS        0x05

#define I2C_M_RD     1
#define I2C_M_WT     0

typedef struct i2c_msg {
	uint8_t slave_addr;
	uint8_t rw_mode;
	uint16_t reg;
	uint8_t  reg_size;
	uint32_t* data;
	uint16_t data_size;
}i2c_msg_t;



XnDothinProtocol::XnDothinProtocol()
{

}



//---------------------------------------------------------------------------Init-SPI---------------------------------------------------------------------------------//


int Spi_Wread(uint8_t * buf, uint32_t size, int id)
{
	int ret = 0;
	uint8_t cmd = buf[0];
	uint32_t addr = (buf[1] << 16) | (buf[2] << 8) | (buf[3] & 0xff);

	switch (cmd)
	{
	case MX6300_CMD_ID_DEVICE_ID:
		//printf("ReadSensorSPI cmd: %d, addr: 0x%04x, size: %d\n", cmd, addr, 2);
		ret = ReadSensorSPI(cmd, 0, 3, &buf[4], 2, id);
		break;
	case MX6300_CMD_ID_FAST_READ:
		ret = ReadSensorSPI(cmd, addr, 3, &buf[4], size - 4, id);
		//printf("ReadSensorSPI cmd: 0x%x, addr: 0x%08x, size: %d, 0x%x%x%x%x, id = %d\n", cmd, addr, size - 4, buf[6], buf[7], buf[8], buf[9], id);
		break;
	case MX6300_CMD_ID_PAGE_PROGRAM:
		//printf("WriteSensorSPI cmd: %d, addr: 0x%04x, size: %d, id: %d\n", cmd, addr, size-4, id);
		//WriteSensorSPI(0x02, (WriteAddress + WriteAddressOffset), 3, (WriteBuffer + WriteAddressOffset), WriteBlockSize, dothin_id_);
		ret = WriteSensorSPI(cmd, addr, 3, &buf[4], size - 4, id);
		break;
	case MX6300_CMD_ID_STATUS:
		ret = -1;
		break;
	default:
		ret = -2;
		break;
	}

	XN_IS_DOTHINSTATUS_OK(ret);
	return 0;
}



/**
*  功能描述: 度信的spi访问接口, 对应度信设备id为0的设备
*  @param buf  spi协议传输的buf
*  @param size 传输buf的大小
*
*  @return  成功, 0; 失败 < 0
*/
int spi_wread_device0(uint8_t * buf, uint32_t size) {

	int ret = Spi_Wread(buf, size, 0);
	return ret;
}

/**
*  功能描述: 度信的spi访问接口, 对应度信设备id为1的设备
*  @param buf  spi协议传输的buf
*  @param size 传输buf的大小
*
*  @return  成功, 0; 失败 < 0
*/
int spi_wread_device1(uint8_t * buf, uint32_t size) {
	int ret = Spi_Wread(buf, size, 1);
	return ret;
}

/**
*  功能描述: 度信的spi访问接口, 对应度信设备id为2的设备
*  @param buf  spi协议传输的buf
*  @param size 传输buf的大小
*
*  @return  成功, 0; 失败 < 0
*/
int spi_wread_device2(uint8_t * buf, uint32_t size) {
	int ret = Spi_Wread(buf, size, 2);
	return ret;
}

/**
*  功能描述: 度信的spi访问接口, 对应度信设备id为3的设备
*  @param buf  spi协议传输的buf
*  @param size 传输buf的大小
*
*  @return  成功, 0; 失败 < 0
*/
int spi_wread_device3(uint8_t * buf, uint32_t size) {
	int ret = Spi_Wread(buf, size, 3);
	return ret;
}


func_ptr_int_uint8p_uint32 spi_funcs[] = {
	(func_ptr_int_uint8p_uint32)spi_wread_device0,
	(func_ptr_int_uint8p_uint32)spi_wread_device1,
	(func_ptr_int_uint8p_uint32)spi_wread_device2,
	(func_ptr_int_uint8p_uint32)spi_wread_device3,
};



void  mprintf(uint8_t level, const char * fmt, ...){

	//printf(fmt);
	xnLogError(XN_MASK_DOTHIN_PROTOCOL, "%s", fmt);

	return;
}

void ObUsleep(uint32_t us)
{
	Sleep(us / 1000);
}


/**
* @brief   初始化SPI接口,并初始化obc_ops_t
* @return  接口调用结果: 失败 < 0. 成功 0.
*/
XnStatus XnDothinProtocol::InitSpi(obc_ops_t* ops, int dotinid)
{
	int ret = 0;
	ops->qsee_log = (func_ptr_void_uint8_vr)obc_log;
	ops->qsee_malloc = (func_ptr_voidp_uint64)malloc;
	ops->qsee_free = (func_ptr_void_voidp)free;
	ops->tee_usleep = (func_ptr_void_uint64)ObUsleep;

	int size = sizeof(spi_funcs) / sizeof(spi_funcs[0]);
	if (dotinid >= size) {
		xnLogError(XN_MASK_DOTHIN_PROTOCOL, " m_DevId more than the func arr  %d, size: %d.", dotinid, size);
		return XN_STATUS_ERROR;
	}

	ops->ops_writeread = spi_funcs[dotinid];
	return XN_STATUS_OK;
}


//---------------------------------------------------------------------------Init-I2C---------------------------------------------------------------------------------//


static inline uint16_t __le16_to_be16(uint16_t val)
{
	return (((val & 0xFF) << 8) | ((val & 0xFF00) >> 8));
}


static inline uint32_t __le32_to_be32(uint32_t val)
{
	return (((val & 0xFF) << 24) | ((val & 0xFF00) << 8) | ((val & 0xFF0000) >> 8) | ((val & 0xFF000000) >> 24));
}


void  LittleEndian2BigEndian(unsigned char *p, int size) {
	if (size == 2) {
		*(uint16_t*)p = __le16_to_be16(*(uint16_t*)p);
	}
	else if (size == 4) {
		*(uint32_t*)p = __le32_to_be32(*(uint32_t*)p);
	}
}


int i2c_wread(uint8_t * buf, uint32_t size, int dothin_id)
{

	int ret = 0;
	i2c_msg_t* p_i2cmsg = (i2c_msg_t*)buf;

#if 0
	uint8_t* poffset = buf;
	uint8_t slave_addr = p_i2cmsg->reg;
	uint8_t cmd = poffset[1];

	uint16_t reg = *(uint16_t*)(poffset + 2);
	poffset += 4;
#endif

	//xnLogVerbose(XN_MASK_DOTHIN_PROTOCOL, "p_i2cmsg: rw_mode: %d, slave: 0x%x,  reg: 0x%x, value: 0x%x.", p_i2cmsg->rw_mode, p_i2cmsg->slave_addr, p_i2cmsg->reg, *(uint16_t*)p_i2cmsg->data);
	//uint32_t addr = (buf[1] << 16) | (buf[2] << 8) | (buf[3] & 0xff);
	switch (p_i2cmsg->rw_mode)
	{
	case I2C_M_WT:
		LittleEndian2BigEndian((BYTE*)p_i2cmsg->data, p_i2cmsg->data_size);
		ret = WriteSensorI2c(p_i2cmsg->slave_addr, p_i2cmsg->reg, p_i2cmsg->reg_size, (BYTE*)p_i2cmsg->data, p_i2cmsg->data_size, dothin_id);

		//xnLogVerbose(XN_MASK_DOTHIN_PROTOCOL, "writeSensorI2C: slave: 0x%x, reg: 0x%x, regsize: %d, data: 0x%04x datasize = %d, dothinId = %d,ret = %d.", p_i2cmsg->slave_addr, p_i2cmsg->reg, p_i2cmsg->reg_size, *(uint32_t*)p_i2cmsg->data, p_i2cmsg->data_size, dothin_id, ret);
		break;
	case I2C_M_RD:
		ret = ReadSensorI2c(p_i2cmsg->slave_addr, p_i2cmsg->reg, p_i2cmsg->reg_size, (BYTE*)p_i2cmsg->data, p_i2cmsg->data_size, 1, dothin_id);
        LittleEndian2BigEndian((BYTE*)p_i2cmsg->data, p_i2cmsg->data_size);
		//xnLogVerbose(XN_MASK_DOTHIN_PROTOCOL, "ReadSensorI2c: slave: 0x%x, reg: 0x%x, regsize: %d, data: 0x%04x datasize = %d, dothinId = %d, ret = %d.", p_i2cmsg->slave_addr, p_i2cmsg->reg, p_i2cmsg->reg_size, *(uint32_t*)p_i2cmsg->data, p_i2cmsg->data_size, dothin_id, ret);
		break;
	default:
		ret = -2;
		break;
	}

	XN_IS_DOTHINSTATUS_OK(ret);
	return ret;
}


/**
*  功能描述: 度信的I2C访问接口, 对应度信设备id为0的设备
*  @param buf  spi协议传输的buf
*  @param size 传输buf的大小
*
*  @return  成功, 0; 失败 < 0
*/
int i2c_wread_device0(uint8_t * buf, uint32_t size) {
	int ret = i2c_wread(buf, size, 0);
	return ret;
}

/**
*  功能描述: 度信的I2C访问接口, 对应度信设备id为1的设备
*  @param buf  spi协议传输的buf
*  @param size 传输buf的大小
*
*  @return  成功, 0; 失败 < 0
*/
int i2c_wread_device1(uint8_t * buf, uint32_t size) {
	int ret = i2c_wread(buf, size, 1);
	return ret;
}

/**
*  功能描述: 度信的I2C访问接口, 对应度信设备id为2的设备
*  @param buf  spi协议传输的buf
*  @param size 传输buf的大小
*
*  @return  成功, 0; 失败 < 0
*/
int i2c_wread_device2(uint8_t * buf, uint32_t size) {
	int ret = i2c_wread(buf, size, 2);
	return ret;
}

int i2c_wread_device3(uint8_t * buf, uint32_t size) {
	int ret = i2c_wread(buf, size, 3);
	return ret;
}

func_ptr_int_uint8p_uint32 i2c_funcs[] = {
	(func_ptr_int_uint8p_uint32)i2c_wread_device0,
	(func_ptr_int_uint8p_uint32)i2c_wread_device1,
	(func_ptr_int_uint8p_uint32)i2c_wread_device2,
	(func_ptr_int_uint8p_uint32)i2c_wread_device3,
};

/**
*  功能描述: 度信的I2C访问接口
*  @param buf  spi协议传输的buf
*  @param size 传输buf的大小
*
*  @return  成功, 0; 失败 < 0
*/
XnStatus XnDothinProtocol::InitI2C(obc_ops_t* ops, int dotinid)
{

	int ret = 0;
	ops->qsee_log = mprintf;
	ops->qsee_malloc = (func_ptr_voidp_uint64)malloc;
	ops->qsee_free = (func_ptr_void_voidp)free;
	ops->tee_usleep = (func_ptr_void_uint64)ObUsleep;

	int size = sizeof(i2c_funcs) / sizeof(i2c_funcs[0]);
	if (dotinid >= size) {
		xnLogError(XN_MASK_DOTHIN_PROTOCOL, " m_DevId more than the func arr  %d, size: %d.", dotinid, size);
		return XN_STATUS_ERROR;
	}
	ops->ops_writeread = i2c_funcs[dotinid];

	return XN_STATUS_OK;
}


XnStatus XnDothinProtocol::InitApOps(obc_ops_t *ops, int dothin_id) 
{
    if (nullptr == ops)
    {
        return  XN_STATUS_NULL_INPUT_PTR;
    }
    ops->ap_ops.device_id = dothin_id;
    ops->ap_ops.EnableSoftPin = EnableSoftPin;
    ops->ap_ops.EnableGpio = EnableGpio;
    ops->ap_ops.PmuSetVoltage = (func_ptr_int_uintptr_intptr_int_int)PmuSetVoltage;
    ops->ap_ops.PmuSetOnOff = (func_ptr_int_uintptr_intptr_int_int)PmuSetOnOff;
    ops->ap_ops.SetSensorClock = SetSensorClock;
    ops->ap_ops.SetSoftPinPullUp = SetSoftPinPullUp;
    ops->ap_ops.SetSensorI2cRate = SetSensorI2cRate;
    ops->ap_ops.SensorEnable = SensorEnable;

    ops->ap_ops.SetGpioPinLevel = SetGpioPinLevel;
    ops->ap_ops.SetGpioPinDir = SetGpioPinDir;
    ops->ap_ops.SetMipiConfiguration = nullptr;
    ops->ap_ops.SensorSpiRWEx = SensorSpiRWEx;

    ops->ap_ops.SetSoftPin = SetSoftPin;
    ops->ap_ops.MasterSpiConfig = (func_MasterSpiConfig)MasterSpiConfig;
    ops->ap_ops.WriteSensorReg = WriteSensorReg;

    return  0;
}



XnDothinProtocol::~XnDothinProtocol()
{

}
