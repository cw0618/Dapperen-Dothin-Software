#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <hw_obstatus.h>
#include "mlx75027.h"

#define DAC5574         5574
#define QUAD_CXA4016    44016

static uint16_t HMAX = 694;
static uint16_t vcsel_driver_type = 5574;
static uint8_t vcsel_number = 1;

const static uint8_t use_trigger_mode = 1;


struct regList {
	uint16_t reg;
	uint8_t val;
};	

#if !DEBUG_MLX75027_IN_QT
#include <tof_sensors.h>
#include <obc_tee_funcs.h>
//#include "hw_modules.h"

#ifdef __linux__
#include <unistd.h>
#endif

#define DDEBUG(fmt, ...)   \
	printf("[DEBUG] [%s(%d)] : " fmt"\n",__FUNCTION__,__LINE__,##__VA_ARGS__)

//#define ALOGE(...) tops.qsee_log(TEE_LOG_LEVEL_ERROR, __VA_ARGS__)
#define TEE_LOG_LEVEL_ERROR        8
#define ALOGE(fmt,...) tops_t.qsee_log(TEE_LOG_LEVEL_ERROR, "[ERROR] [%s(%d)] : " fmt"\n",__FUNCTION__,__LINE__,##__VA_ARGS__)


//#define ALOGE(...)
#define malloc tops_t.qsee_malloc
#define free tops_t.qsee_free
#define usleep tops_t.tee_usleep
#define orbbec_i2c_writeread tops_t.ops_writeread

#define dothin_enable_softpin      tops_t.ap_ops.EnableSoftPin
#define dothin_enable_gpio         tops_t.ap_ops.EnableGpio
#define dothin_set_sensor_clock    tops_t.ap_ops.SetSensorClock
#define dothin_set_softpin_pullup  tops_t.ap_ops.SetSoftPinPullUp
#define dothin_set_sensor_i2c_rate tops_t.ap_ops.SetSensorI2cRate
#define dothin_sensor_enable       tops_t.ap_ops.SensorEnable
#define dothin_device_id           tops_t.ap_ops.device_id

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


static int i2c_reg_read(uint8_t addr, uint16_t reg, uint8_t reg_size, uint32_t *data, uint16_t data_size)
{
    int ret = 0;
    i2c_msg_t msg;

    msg.slave_addr = addr;
    msg.rw_mode = I2C_M_RD;
    msg.reg = reg;
    msg.reg_size = reg_size;
    msg.data = data;
    msg.data_size = data_size;

    ret = orbbec_i2c_writeread(&msg, 1);

    return ret;
}

static int i2c_reg_write(uint8_t addr, uint16_t reg, uint8_t reg_size, uint32_t data, uint16_t data_size)
{
    int ret = 0;
    i2c_msg_t msg;

    msg.slave_addr = addr;
    msg.rw_mode = I2C_M_WT;
    msg.reg = reg;
    msg.reg_size = reg_size;
    msg.data = &data;
    msg.data_size = data_size;

    ret = orbbec_i2c_writeread(&msg, 1);

    return ret;
}

static int gpio_control(int pin, bool level)
{
    return 0; // should wrap method by SDK
}

#else
//#include <QDebug.h>
void* iic_handle = NULL;
post_iic_write iic_write_callback = NULL;
post_iic_read iic_read_callback = NULL;
post_gpio gpio_callback = NULL;
void set_iic_write_callback(post_iic_write cb, void* handle)
{
    iic_write_callback = cb;
    iic_handle = handle;
}
void set_iic_read_callback(post_iic_read cb, void* handle)
{
    iic_read_callback = cb;
    iic_handle = handle;
}
void set_gpio_callback(post_gpio cb, void* handle)
{
    gpio_callback = cb;
    iic_handle = handle;
}

static int i2c_reg_read(uint8_t addr, uint16_t reg, uint8_t reg_size, uint32_t *data, uint16_t data_size)
{
    if(iic_handle == NULL){
        //qDebug() << "null iic handle";
        return -1;
    }
    return iic_read_callback(addr, reg, reg_size, data, data_size, iic_handle);

}

static int i2c_reg_write(uint8_t addr, uint16_t reg, uint8_t reg_size, uint32_t data, uint16_t data_size)
{
    if(iic_handle == NULL){
        //qDebug() << "null iic handle";
        return -1;
    }
    return iic_write_callback(addr, reg, reg_size, data, data_size, iic_handle);

}

static int gpio_control(int pin, bool level)
{
    if(iic_handle == NULL){
        //qDebug() << "null iic handle";
        return -1;
    }
    return gpio_callback(pin, level, iic_handle);
}
#endif

int dothin_config()
{
#if !DEBUG_MLX75027_IN_QT
	int ret = 0;

	DDEBUG("dothin_device_id=%d\n", dothin_device_id);

	ret =dothin_enable_softpin(true, dothin_device_id);
	if (ret < 0) {
		DDEBUG("dothin_enable_softpin ret=%d\n",ret);
	}
	ret=dothin_enable_gpio(true, dothin_device_id);
	if (ret < 0) {
		DDEBUG("dothin_enable_gpio ret=%d\n", ret);
	}
	ret = dothin_set_sensor_clock(true, 8 * 10, dothin_device_id); // 8Mhz mclk
	if (ret < 0) {
		DDEBUG("dothin_set_sensor_clock ret=%d\n", ret);
	}
	ret = dothin_set_softpin_pullup(1, dothin_device_id);
	if (ret < 0) {
		DDEBUG("dothin_set_softpin_pullup ret=%d\n", ret);
	}
	usleep(1000*10);
	ret =dothin_set_sensor_i2c_rate(1, dothin_device_id); // 400Khz
	if (ret < 0) {
		DDEBUG("dothin_set_sensor_i2c_rate ret=%d\n", ret);
	}
	ret =dothin_sensor_enable(1, true, dothin_device_id);
	if (ret < 0) {
		DDEBUG("dothin_sensor_enable ret=%d\n", ret);
	}
	usleep(1000 * 20);
	ret = dothin_sensor_enable(3, true, dothin_device_id);
	if (ret < 0) {
		DDEBUG("dothin_sensor_enable ret=%d\n", ret);
	}
	usleep(1000 * 50);

    return ret;
#endif

    
}

int sensor_write_reg(uint16_t reg, uint8_t value)
{
    int rtn = i2c_reg_write(SENSOR_ADDR, reg, 2, value, 1);
    return rtn;
}

int sensor_read_reg(uint16_t reg, uint8_t *value)
{
    int rtn = i2c_reg_read(SENSOR_ADDR, reg, 2, value, 1);
    return rtn;
}

int temp_write_reg(uint8_t addr, uint8_t reg, uint16_t value)
{
    int rtn = i2c_reg_write(addr, reg, 1, value, 2);
    return rtn;
}

int temp_read_reg(uint8_t addr, uint8_t reg, uint16_t *value)
{
    int rtn = i2c_reg_read(addr, reg, 1, value, 2);
    return rtn;
}

int dac5574_write_reg(uint8_t reg, uint16_t value)
{
    int rtn = i2c_reg_write(DAC5574_ADDR, reg, 1, value, 2);
    return rtn;

}

int dac5574_read_reg(uint8_t reg, uint16_t *value)
{
    int rtn = i2c_reg_read(DAC5574_ADDR, reg, 1, value, 2);
    return rtn;

}

int eeprom_write_byte(uint16_t reg, uint8_t value)
{
    int rtn = i2c_reg_write(EEPROM_ADDR, reg, 2, value, 1);
    return rtn;

}

int eeprom_read_byte(uint16_t reg, uint8_t *value)
{
    int rtn = i2c_reg_read(EEPROM_ADDR, reg, 2, value, 1);
    return rtn;

}

int cxa4016_write_reg(uint16_t reg, uint8_t value)
{
	return i2c_reg_write(CXA4016_IIC_ADDR, reg, 2, value, 1);
}

int cxa4016_read_reg(uint16_t reg, uint8_t *value)
{
	return i2c_reg_read(CXA4016_IIC_ADDR, reg, 2, value, 1);
}

// 1-4 used vcsel num
int cxa4016_device_write_reg(uint8_t device, uint8_t reg, uint8_t value) // multi device write
{
	int rtn = 0;

	if (device >= 1 && device <= 4) {
		const static uint8_t spi_write_cs[4] = { 0x01, 0x03, 0x07, 0x0F };
		uint8_t devMask = spi_write_cs[device - 1];
		uint16_t reg_ = (devMask << 8) + reg;
	    rtn = cxa4016_write_reg(reg_, value);
		usleep(5000);
	}
	
	return rtn;
}

// 1-4 used vcsel num
int cxa4016_device_read_reg(uint8_t device, uint8_t reg, uint8_t *value) // single device read
{
	int rtn = 0;

	if (device >= 1 && device <= 4) {
		const static uint8_t spi_read_cs[4] = { 0x01, 0x02, 0x04, 0x08 };
		uint8_t devMask = spi_read_cs[device - 1];
		uint16_t reg_ = ((devMask + 0x10) << 8) + reg; // 0x10 means read
		rtn = cxa4016_write_reg(reg_, 0xFF); // dummy write before read, value is not care

		usleep(5000);
		rtn |= cxa4016_read_reg(reg_, value);
		usleep(5000);
	}

	return rtn;
}

// cxa4016 register list
#define WORK_MODE      0x00
#define IBIAS_APCL1    0x01
#define IBIAS_APCH1    0x02
#define ISW_APCL2      0x05
#define ISW_APCH2      0x06
#define IBIAX_FIX      0x07
#define ISW_FIX        0x08
#define IBIAS_BACK     0x09
#define AS_W           0x0F // bit[0:3]
#define AS_I           0x0F // bit[4:7]
#define AS_Tr          0x10 // bit[0:3]
#define GL_RST         0x25 
// user defined,
#define SET_APC_GATE_SIGNAL     0x50

#define GET_APC_GATE_SIGNAL     0x70
#define GET_TEMPERATUER         0x71

int cxa4016_init() // default using first vcsel
{
	int rtn = 0;
	rtn = cxa4016_device_write_reg(4, SET_APC_GATE_SIGNAL, 0x00); // turn off all the vcsel
	rtn |= cxa4016_device_write_reg(4, GL_RST, 0x01); // global reset all the cxa4016
	rtn |= cxa4016_device_write_reg(4, WORK_MODE, 0x0C); // fix current mode, error detect enable
	rtn |= cxa4016_device_write_reg(4, IBIAX_FIX, 0x00); // 0A
	rtn |= cxa4016_device_write_reg(4, ISW_FIX, 0xCC); // 3A
	rtn |= cxa4016_device_write_reg(4, SET_APC_GATE_SIGNAL, 0x01); // turn on four vcsel
	return rtn;
}

void vcsel_driver_detect()
{
	int rtn = 0;
    uint16_t value = 9;
    rtn = cxa4016_device_write_reg(1, WORK_MODE, 0x0C);
	rtn = cxa4016_device_read_reg(1, WORK_MODE, &value);
 
    DDEBUG("----->cxa4016_write_reg   = %d, value %x\n", rtn, value);

	if(value == 0x0C){
        vcsel_driver_type = QUAD_CXA4016;
        vcsel_number = 4;
		
	}
	else {
		rtn = dac5574_write_reg(0x10, 0x1234);
		if (rtn >= 0){
            vcsel_driver_type = DAC5574;
            vcsel_number = 1;
		}
		else{
			vcsel_driver_type = 0;
			vcsel_number = 0;
		}
	}
	//printf("vcsel_driver_type = %x\n", vcsel_driver_type);
}

int mlx75027_get_rx_temp(float *temperature)
{
    uint16_t value = 0;
    int rtn = temp_read_reg(TEMP_RX_ADDR, 0x00, &value);
    *temperature = (value >> 4)*0.0625f;
	//DDEBUG("mlx75027_get_rx_temp %f   value = %x", *temperature, value);
    return rtn;
}

int mlx75027_get_tx_temp(float *temperature)
{    
    if(vcsel_driver_type == DAC5574){
        uint16_t value = 0;
        int rtn = temp_read_reg(TEMP_TX_ADDR, 0x00, &value);
        *temperature = (value >> 4)*0.0625f;
        return rtn;
    }
    else if (vcsel_driver_type == QUAD_CXA4016) {
		return  -HW_ERR_NO_SUPPORT;
    }
	return  -HW_ERR_NO_SUPPORT;
}

int mlx75027_set_illum_power(uint8_t vcsel_num, uint8_t value_A, uint8_t value_B)
{
	int rtn = 0;
	if (vcsel_driver_type == DAC5574) {

		rtn = dac5574_write_reg(0x10, (value_A << 8)); // 0x10 channel A
		rtn = dac5574_write_reg(0x12, (value_B << 8)); // 0x12 channel B
	}
	else if (vcsel_driver_type == QUAD_CXA4016) {
		if (vcsel_num > 4 )
			return -HW_ERR_INVALID;
		if(vcsel_num == 0)
			rtn = cxa4016_device_write_reg(4, SET_APC_GATE_SIGNAL, 0x00); // turn off all the vcsel
		else{
			rtn = cxa4016_device_write_reg(4, SET_APC_GATE_SIGNAL, 0x00); // turn off all the vcsel first
			rtn = cxa4016_device_write_reg(vcsel_num, ISW_FIX, value_A);
			rtn = cxa4016_device_write_reg(vcsel_num, IBIAX_FIX, value_B);

			rtn = cxa4016_device_write_reg(vcsel_num, SET_APC_GATE_SIGNAL, 0x01); // turn on selected vcsel
		}
	}
	else
		return -HW_ERR_NO_SUPPORT;
    return rtn;
}

int mlx75027_get_illum_power(uint8_t *vcsel_num, uint8_t *value_A, uint8_t *value_B)
{
	int rtn = 0;
	if (vcsel_driver_type == DAC5574) {
		*vcsel_num = 1;
		
		uint16_t value;
		rtn = dac5574_read_reg(0x10, &value);
		*value_A = (value >> 8) & 0xff;

		rtn = dac5574_read_reg(0x12, &value);
		*value_B = (value >> 8) & 0xff;
	}
	else if (vcsel_driver_type == QUAD_CXA4016) {

		rtn = cxa4016_device_read_reg(1, GET_APC_GATE_SIGNAL, vcsel_num); // first param is not care
		rtn = cxa4016_device_read_reg(1, ISW_FIX, value_A); // always read first vcsel
		rtn = cxa4016_device_read_reg(1, IBIAX_FIX, value_B);
	}
    return rtn;
}

int mlx75027_hardware_trigger()
{/*
    //mdevices.gpio_control(TRIGGER_PIN, 1);
    mdevices.gpio_control(TRIGGER_PIN, 0); // active low
    mdevices.sleepms(0.001); // 1us, not accurate
    //mdevices.usleep(100);
    mdevices.gpio_control(TRIGGER_PIN, 1);
    */
    return 0;
}

int mlx75027_software_trigger()
{
    int rtn = sensor_write_reg(0x2100, 0x01);
    return rtn;
}

int mlx75027_illum_power_control(bool enable)
{
    return gpio_control(LD_ENABLE_PIN, enable); // active high

}

struct regList reglist[] = {
    /* Sensor standby */
    // Input clock setting(fixed)
    {0x1006, 0x08},
    {0x1007, 0x00},
    {0x1042, 0x01},
    {0x1046, 0x01},
    {0x104A, 0x01},
    // Input clock setting end

    {0x1000, 0x00}, // Change from Sensor Standby to Software Standby

    /* Sensor standby end*/

    /* Software standby */
    // fixed setting, total 306 registers
	{0x10D2, 0x00},
    {0x10D3, 0x10},			   				   
    {0x1448, 0x06},
    {0x1449, 0x40},
    {0x144A, 0x06},
    {0x144B, 0x40},
    {0x144C, 0x06},
    {0x144D, 0x40},
    {0x144E, 0x06},
    {0x144F, 0x40},
    {0x1450, 0x06},
    {0x1451, 0x40},
    {0x1452, 0x06},
    {0x1453, 0x40},
    {0x1454, 0x06},
    {0x1455, 0x40},
    {0x1456, 0x06},
    {0x1457, 0x40},
    {0x21C4, 0x00},
    {0x2202, 0x00},
    {0x2203, 0x1E},
    {0x2C08, 0x01},
	{0x2C0C, 0x51}, // set embedded data length
    {0x2C0D, 0x00},										   				   
    {0x3C2B, 0x1B},
    {0x400E, 0x01},
    {0x400F, 0x81},
    {0x40D1, 0x00},
    {0x40D2, 0x00},
    {0x40D3, 0x00},
    {0x40DB, 0x3F},
    {0x40DE, 0x40},
    {0x40DF, 0x01},
    {0x412C, 0x00},
    {0x4134, 0x04},
    {0x4135, 0x04},
    {0x4136, 0x04},
    {0x4137, 0x04},
    {0x4138, 0x04},
    {0x4139, 0x04},
    {0x413A, 0x04},
    {0x413B, 0x04},
    {0x413C, 0x04},
    {0x4146, 0x01},
    {0x4147, 0x01},
    {0x4148, 0x01},
    {0x4149, 0x01},
    {0x414A, 0x01},
    {0x414B, 0x01},
    {0x414C, 0x01},
    {0x414D, 0x01},
    {0x4158, 0x01},
    {0x4159, 0x01},
    {0x415A, 0x01},
    {0x415B, 0x01},
    {0x415C, 0x01},
    {0x415D, 0x01},
    {0x415E, 0x01},
    {0x415F, 0x01},
    {0x4590, 0x00},
    {0x4591, 0x2E},
    {0x4684, 0x00},
    {0x4685, 0xA0},
    {0x4686, 0x00},
    {0x4687, 0xA1},
    {0x471E, 0x07},
    {0x471F, 0xC9},
    {0x473A, 0x07},
    {0x473B, 0xC9},
    {0x4770, 0x00},
    {0x4771, 0x00},
    {0x4772, 0x1F},
    {0x4773, 0xFF},
    {0x4778, 0x06},
    {0x4779, 0xA4},
    {0x477A, 0x07},
    {0x477B, 0xAE},
    {0x477D, 0xD6},
    {0x4788, 0x06},
    {0x4789, 0xA4},
    {0x478C, 0x1F},
    {0x478D, 0xFF},
    {0x478E, 0x00},
    {0x478F, 0x00},
    {0x4792, 0x00},
    {0x4793, 0x00},
    {0x4796, 0x00},
    {0x4797, 0x00},
    {0x479A, 0x00},
    {0x479B, 0x00},
    {0x479C, 0x1F},
    {0x479D, 0xFF},
    {0x479E, 0x00},
    {0x479F, 0x00},
    {0x47A2, 0x00},
    {0x47A3, 0x00},
    {0x47A6, 0x00},
    {0x47A7, 0x00},
    {0x47AA, 0x00},
    {0x47AB, 0x00},
    {0x47AC, 0x1F},
    {0x47AD, 0xFF},
    {0x47AE, 0x00},
    {0x47AF, 0x00},
    {0x47B2, 0x00},
    {0x47B3, 0x00},
    {0x47B6, 0x00},
    {0x47B7, 0x00},
    {0x47BA, 0x00},
    {0x47BB, 0x00},
    {0x47BC, 0x1F},
    {0x47BD, 0xFF},
    {0x47BE, 0x00},
    {0x47BF, 0x00},
    {0x47C2, 0x00},
    {0x47C3, 0x00},
    {0x47C6, 0x00},
    {0x47C7, 0x00},
    {0x47CA, 0x00},
    {0x47CB, 0x00},
    {0x4834, 0x00},
    {0x4835, 0xA0},
    {0x4836, 0x00},
    {0x4837, 0xA1},
    {0x4878, 0x00},
    {0x4879, 0xA0},
    {0x487A, 0x00},
    {0x487B, 0xA1},
    {0x48BC, 0x00},
    {0x48BD, 0xA0},
    {0x48BE, 0x00},
    {0x48BF, 0xA1},
    {0x4954, 0x00},
    {0x4955, 0xA0},
    {0x4956, 0x00},
    {0x4957, 0xA1},
    {0x4984, 0x00},
    {0x4985, 0xA0},
    {0x4986, 0x00},
    {0x4987, 0xA1},
    {0x49B8, 0x00},
    {0x49B9, 0x78},
    {0x49C2, 0x00},
    {0x49C3, 0x3C},
    {0x49C8, 0x00},
    {0x49C9, 0x76},
    {0x49D2, 0x00},
    {0x49D3, 0x3F},
    {0x49DC, 0x00},
    {0x49DD, 0xA0},
    {0x49DE, 0x00},
    {0x49DF, 0xA1},
    {0x49EE, 0x00},
    {0x49EF, 0x78},
    {0x49F8, 0x00},
    {0x49F9, 0x3C},
    {0x49FE, 0x00},
    {0x49FF, 0x78},
    {0x4A04, 0x00},
    {0x4A05, 0x3C},
    {0x4A0A, 0x00},
    {0x4A0B, 0x76},
    {0x4A10, 0x00},
    {0x4A11, 0x3F},
    {0x4A1A, 0x00},
    {0x4A1B, 0xA0},
    {0x4A1C, 0x00},
    {0x4A1D, 0xA1},
    {0x4A1E, 0x00},
    {0x4A1F, 0x78},
    {0x4A28, 0x00},
    {0x4A29, 0x3C},
    {0x4A4A, 0x00},
    {0x4A4B, 0xA0},
    {0x4A4C, 0x00},
    {0x4A4D, 0xA1},
    {0x4A7A, 0x00},
    {0x4A7B, 0xA0},
    {0x4A7C, 0x00},
    {0x4A7D, 0xA1},
    {0x4AEE, 0x00},
    {0x4AEF, 0xA0},
    {0x4AF0, 0x00},
    {0x4AF1, 0xA1},
    {0x4B2E, 0x00},
    {0x4B2F, 0xA0},
    {0x4B30, 0x00},
    {0x4B31, 0xA1},
    {0x4B5A, 0x00},
    {0x4B5B, 0xA0},
    {0x4B5C, 0x00},
    {0x4B5D, 0xA1},
    {0x4B86, 0x00},
    {0x4B87, 0xA0},
    {0x4B88, 0x00},
    {0x4B89, 0xA1},
    {0x4B9E, 0x00},
    {0x4B9F, 0x1A},
    {0x4BAE, 0x00},
    {0x4BAF, 0x1A},
    {0x4BB6, 0x00},
    {0x4BB7, 0x1A},
    {0x4BC6, 0x00},
    {0x4BC7, 0x1A},
    {0x4BCE, 0x00},
    {0x4BCF, 0x1A},
    {0x4BEE, 0x00},
    {0x4BEF, 0xA0},
    {0x4BF0, 0x00},
    {0x4BF1, 0xA1},
    {0x4BF6, 0x00},
    {0x4BF7, 0x1A},
    {0x4C00, 0x00},
    {0x4C01, 0x1A},
    {0x4C58, 0x00},
    {0x4C59, 0xA0},
    {0x4C5A, 0x00},
    {0x4C5B, 0xA1},
    {0x4C6E, 0x00},
    {0x4C6F, 0xA0},
    {0x4C70, 0x00},
    {0x4C71, 0xA1},
    {0x4C7A, 0x01},
    {0x4C7B, 0x35},
    {0x4CF2, 0x07},
    {0x4CF3, 0xC9},
    {0x4CF8, 0x06},
    {0x4CF9, 0x9B},
    {0x4CFA, 0x07},
    {0x4CFB, 0xAE},
    {0x4CFE, 0x07},
    {0x4CFF, 0xC9},
    {0x4D04, 0x06},
    {0x4D05, 0x98},
    {0x4D06, 0x07},
    {0x4D07, 0xB1},
    {0x4D18, 0x06},
    {0x4D19, 0xA4},
    {0x4D1A, 0x07},
    {0x4D1B, 0x49},
    {0x4D1E, 0x07},
    {0x4D1F, 0xC9},
    {0x4D2A, 0x07},
    {0x4D2B, 0xC9},
    {0x4D4A, 0x07},
    {0x4D4B, 0xC9},
    {0x4D50, 0x06},
    {0x4D51, 0x9B},
    {0x4D52, 0x07},
    {0x4D53, 0xAE},
    {0x4D56, 0x07},
    {0x4D57, 0xC9},
    {0x4D5C, 0x06},
    {0x4D5D, 0x98},
    {0x4D5E, 0x07},
    {0x4D5F, 0xB1},
    {0x4D70, 0x06},
    {0x4D71, 0xA4},
    {0x4D72, 0x07},
    {0x4D73, 0x49},
    {0x4D78, 0x06},
    {0x4D79, 0xA4},
    {0x4D7A, 0x07},
    {0x4D7B, 0xAE},
    {0x4D7C, 0x1F},
    {0x4D7D, 0xFF},
    {0x4D7E, 0x1F},
    {0x4D7F, 0xFF},
    {0x4D80, 0x06},
    {0x4D81, 0xA4},
    {0x4D82, 0x07},
    {0x4D83, 0xAE},
    {0x4D84, 0x1F},
    {0x4D85, 0xFF},
    {0x4D86, 0x1F},
    {0x4D87, 0xFF},
    {0x4E39, 0x07},
    {0x4E7B, 0x64},
    {0x4E8E, 0x0E},
    {0x4E9A, 0x00},
    {0x4E9C, 0x01},
    {0x4EA0, 0x01},
    {0x4EA1, 0x03},
    {0x4EA5, 0x00},
    {0x4EA7, 0x00},
    {0x4F05, 0x04},
    {0x4F0D, 0x04},
    {0x4F15, 0x04},
    {0x4F19, 0x01},
    {0x4F20, 0x01},
    {0x4F66, 0x0F},
    {0x500F, 0x01},
    {0x5224, 0x00},
    {0x5225, 0x2F},
    {0x5226, 0x00},
    {0x5227, 0x1E},
    {0x5230, 0x00},
    {0x5231, 0x19},
    {0x5244, 0x00},
    {0x5245, 0x07},
    {0x5252, 0x07},
    {0x5253, 0x08},
    {0x5254, 0x07},
    {0x5255, 0xB4},
    {0x5271, 0x00},
    {0x5272, 0x04},
    {0x5273, 0x2E},
    {0x5285, 0x00},
    {0x5286, 0x00},
    {0x5287, 0x5D},
    // fixed setting end


    // custom setting
    // mipi data output confguration
    {0x1010, 0x03}, // 4 data lane
    {0x100C, 0x0F},
    {0x100D, 0x00}, // 960 Mbps
    {0x1C40, 0x01}, // clock-off during blank

    // trigger mode setting
    {0x2020, 0x00},
    {0x2100, 0x00},
    {0x2F05, 0x07},
    {0x2F06, 0x00},
    {0x2F07, 0x00},
    {0x3071, 0x03},

    {0x0828, 0x02}, // Mode A-B (12 bit signed data)

    // HMAX & Frame Read-Out Time
    {0x0800, 0x02}, // 1.54ms
    {0x0801, 0xB6},
	{0x1433, 0x00},

    // custom setting end



};

int mlx75027_sensor_initialize()
{
    int rtn = 0;
    for(int i = 0; i < sizeof(reglist)/sizeof(struct regList); i++){

        rtn |= sensor_write_reg(reglist[i].reg, reglist[i].val);
    }
    return rtn;
}

int mlx75027_shadow_register(bool enable)
{
    int rtn = 0;
    if(enable)
        rtn = sensor_write_reg(0x0102, 0x01);
    else
        rtn = sensor_write_reg(0x0102, 0x00);
    return rtn;
}

int mlx75027_get_sensor_id(uint16_t *id) // id should be 0x96
{
    int rtn = 0;
    uint8_t value = 0;
	static int isConfig = 0;
	if (!isConfig) {
		isConfig = 1;
		dothin_config();
	}
	
    rtn = sensor_read_reg(0x0001, &value);
	//printf("mlx75027_get_sensor_id: %d \r\n", value);
    if (rtn < 0) {
        *id = 0xFFFF;
    }
    else if (value == 0x45) {
        *id = mlx75027_sensor_id;
    }
    else {
        *id = 0xFFFF;
    }
        
    return rtn;
}

int mlx75027_set_user_ID(uint8_t value)
{
    return sensor_write_reg(0x0824, value);
}

int mlx75027_set_hmax(uint16_t hmax)
{
    int rtn = 0;
    // set HMAX register
    rtn = sensor_write_reg(0x0800, (hmax >>  8)&0xFF);
    rtn |= sensor_write_reg(0x0801, (hmax &0xFF));
/**/
    // set PLLSSETUP register
    uint8_t pllssetup = (uint8_t)ceil(503*120.0f/hmax + 8.0f);
    rtn |= sensor_write_reg(0x4010, pllssetup);

    // set PIXRST register
    uint16_t pixrst = (uint16_t)ceil(50*120.0f/hmax);
    rtn |= sensor_write_reg(0x4015, (pixrst >> 8)&0xFF);
    rtn |= sensor_write_reg(0x4016, (pixrst &0xFF));

    // set RANDNM0 register
    uint32_t randnm0 = (uint32_t)ceil(hmax*pixrst - 1070 - 2098);
    rtn |= sensor_write_reg(0x5265, (randnm0 >> 16)&0xFF);
    rtn |= sensor_write_reg(0x5266, (randnm0 >>  8)&0xFF);
    rtn |= sensor_write_reg(0x5267, (randnm0 &0xFF));

    return rtn;
}									
int mlx75027_set_trigger_mode(uint8_t mode) // should stop stream first
{
    int rtn = 0;
    if(mode == 0){ // hardware trigger mode
        rtn = sensor_write_reg(0x2020, 0x00);
        rtn |= sensor_write_reg(0x2100, 0x00);
        rtn |= sensor_write_reg(0x2F05, 0x07);
        rtn |= sensor_write_reg(0x2F06, 0x00);
        rtn |= sensor_write_reg(0x2F07, 0x00);
        rtn |= sensor_write_reg(0x3071, 0x03);
    }
    else{ // software trigger mode
        rtn = sensor_write_reg(0x2020, 0x01);
        rtn |= sensor_write_reg(0x2100, 0x01);
        rtn |= sensor_write_reg(0x2F05, 0x01);
        rtn |= sensor_write_reg(0x2F06, 0x09);
        rtn |= sensor_write_reg(0x2F07, 0x7A);
        rtn |= sensor_write_reg(0x3071, 0x00);
    }
    return rtn;
}

int mlx75027_set_stream_mode() // should stop stream first
{
    int rtn = 0;
    rtn = sensor_write_reg(0x2020, 0x01);
    rtn |= sensor_write_reg(0x2100, 0x08);
    rtn |= sensor_write_reg(0x2F05, 0x01);
    rtn |= sensor_write_reg(0x2F06, 0x09);
    rtn |= sensor_write_reg(0x2F07, 0x7A);
    rtn |= sensor_write_reg(0x3071, 0x00);
    return rtn;
}

int mlx75027_get_data_output_mode(uint8_t *mode)
{
    uint8_t value = 0;
    int rtn = sensor_read_reg(0x0828, &value);
    *mode = value&0xff;
    return rtn;
}

// be careful that the resolution of A&B mode will change to 1280*480
int mlx75027_set_data_output_mode(uint8_t mode) // should stop streaming before change data output mode
{
    // mode = 0: A-B, 1: A+B, 2: A, 3: B, 4: A&B
	if (mode > 4) {
		ALOGE("mode=%d", mode);
		return -HW_ERR_INVALID;
	}
    else if(mode == 4){
        HMAX = 0x0514;
        mlx75027_set_hmax(HMAX);
    }
    else if(mode < 4){
        HMAX = 0x02B6;
        mlx75027_set_hmax(HMAX);
    } 
    int rtn = sensor_write_reg(0x0828, mode);
	
    return rtn;
}

int mlx75027_get_modulation_frequency(uint16_t *modFreq)
{
    uint8_t divselpre = 0;
    uint8_t divsel = 0;
    int rtn = sensor_read_reg(0x21BE, &divselpre);
    rtn |= sensor_read_reg(0x21BF, &divsel);

    uint8_t fmod_H, fmod_L;
    rtn |= sensor_read_reg(0x1048, &fmod_H);
    rtn |= sensor_read_reg(0x1049, &fmod_L);
    uint16_t fmod = (fmod_H << 8) + fmod_L;

    *modFreq = (uint16_t)(fmod/pow(2, (divselpre + divsel)));

    return rtn;
}

int mlx75027_set_modulation_frequency(uint16_t modFreq)
{
    if(modFreq > 100 || modFreq < 4)
        return -HW_ERR_INVALID;

    int rtn = mlx75027_shadow_register(true);
    uint8_t divselpre = 0;
    uint8_t divsel = 0;
    if((modFreq <= 100 && modFreq >= 75) || (modFreq <= 50 && modFreq >= 38) || (modFreq <= 20 && modFreq >= 19)){
        divselpre = 0x00;
        rtn |= sensor_write_reg(0x21BE, divselpre);
    }
    else if((modFreq <= 74 && modFreq >= 51) || (modFreq <= 37 && modFreq >= 21) || (modFreq <= 18 && modFreq >= 10)){
        divselpre = 0x01;
        rtn |= sensor_write_reg(0x21BE, divselpre);
    }
    else if(modFreq <= 9 && modFreq >= 5){
        divselpre = 0x02;
        rtn |= sensor_write_reg(0x21BE, divselpre);
    }
    else if(modFreq == 4){
        divselpre = 0x03;
        rtn |= sensor_write_reg(0x21BE, divselpre);
    }

    if(modFreq <= 100 && modFreq >= 51){
        divsel = 0x00;
        rtn |= sensor_write_reg(0x21BF, divsel);
    }
    else if(modFreq <= 50 && modFreq >= 21){
        divsel = 0x01;
        rtn |= sensor_write_reg(0x21BF, divsel);
    }
    else if(modFreq <= 20 && modFreq >= 4){
        divsel = 0x02;
        rtn |= sensor_write_reg(0x21BF, divsel);
    }

    uint16_t fmod = (uint16_t)(pow(2, (divselpre + divsel))*modFreq);
    //qDebug() << "divselpre, divsel, fmod, modFreq" << divselpre << divsel << fmod << modFreq;
    rtn |= sensor_write_reg(0x1048, (fmod >> 8)&0xFF);
    rtn |= sensor_write_reg(0x1049, (fmod & 0xFF));

    if(fmod*8 < 900 && fmod*8 >= 500)
        rtn |= sensor_write_reg(0x104B, 0x02);
    else if(fmod*8 <= 1200 && fmod*8 >= 900)
        rtn |= sensor_write_reg(0x104B, 0x00);

    rtn |= mlx75027_shadow_register(false);

    return rtn;
}

// param uint: us
int mlx75027_get_frame_startup_time(uint16_t *startupTime)
{
    uint8_t byte0, byte1;
    int rtn = sensor_read_reg(0x21D4, &byte1);
    rtn |= sensor_read_reg(0x21D5, &byte0);
    uint16_t value = (byte1 << 8) + byte0;
    *startupTime = value*HMAX/120;

    return rtn;
}														  
int mlx75027_set_frame_startup_time(uint16_t startupTime)
{
    uint16_t value = (startupTime*120)/HMAX;
    int rtn = sensor_write_reg(0x21D4, (value >> 8)&0xFF);
    rtn |= sensor_write_reg(0x21D5, (value & 0xFF));

    return rtn;
}

// param uint: us
int mlx75027_set_frame_time(uint32_t frameTime)
{
    uint32_t value = (frameTime*120)/HMAX;
    int rtn = sensor_write_reg(0x2108, (value >> 24)&0xFF);
    rtn |= sensor_write_reg(0x2109, (value >> 16)&0xFF);
    rtn |= sensor_write_reg(0x210A, (value >> 8)&0xFF);
    rtn |= sensor_write_reg(0x210B, (value & 0xFF));

    return rtn;
}

int mlx75027_get_fps(uint8_t *fps)
{
	int rtn = 0;
	if (!use_trigger_mode) {
		uint8_t byte0, byte1, byte2, byte3;
		rtn = sensor_read_reg(0x2108, &byte3);
		rtn |= sensor_read_reg(0x2109, &byte2);
		rtn |= sensor_read_reg(0x210A, &byte1);
		rtn |= sensor_read_reg(0x210B, &byte0);

		uint32_t value = (byte3 << 24) + (byte2 << 16) + (byte1 << 8) + byte0;
		uint32_t frameTime = (value*HMAX) / 120 + 1; // +1 avoid zero
		*fps = (uint8_t)(1000000 / frameTime);
		//DDEBUG("mlx75027_get_fps %d   value = %x",*fps, value);
	}
	else {
		return -HW_ERR_NO_SUPPORT;
	}
    return rtn;
}

int mlx75027_set_fps(uint8_t fps)
{
	int rtn = 0;
	if(!use_trigger_mode){
		uint32_t frameTime = 1000000/fps; // us
		rtn = mlx75027_set_frame_time(frameTime);
	}
	else {
		return -HW_ERR_NO_SUPPORT;
	}
    return rtn;
}

int mlx75027_set_phase_count(uint8_t phaseCount)
{
    if(phaseCount > 8 || phaseCount < 1)
        return -HW_ERR_INVALID;
    int rtn = sensor_write_reg(0x21E8, phaseCount); // default is 4

    return rtn;
}

int mlx75027_set_phase_pretime(uint16_t preTime)
{
    if(preTime > 2000 || preTime < 1)  // value 0 is not allowed, 2000 is user defined
        return -HW_ERR_INVALID;
    int rtn = mlx75027_shadow_register(true);
    rtn |= sensor_write_reg(0x4015, (preTime >> 8)&0xFF);
    rtn |= sensor_write_reg(0x4016, (preTime & 0xFF));
    rtn |= mlx75027_shadow_register(false);

    return rtn;
}

int mlx75027_get_integration_time(uint16_t *integrationTime)
{
    uint8_t byte0, byte1, byte2, byte3;
    int rtn = sensor_read_reg(0x2120, &byte3);
    rtn |= sensor_read_reg(0x2121, &byte2);
    rtn |= sensor_read_reg(0x2122, &byte1);
    rtn |= sensor_read_reg(0x2123, &byte0);

    uint32_t value = (byte3 << 24) + (byte2 << 16) + (byte1 << 8) + byte0;
    uint8_t repeat = 0;
    rtn |= sensor_read_reg(0x21A0, &repeat);
    value = value*(repeat&0x0F);
    *integrationTime = (uint16_t)(value/120); // may slightly different from what we set because of floor function
	//printf("byte 3 2 1 0:  %x %x %x %x", byte3, byte2, byte1, byte0);
	//printf("repeat %d, ", repeat);
    return rtn;
}

int mlx75027_set_integration_time(uint16_t integrationTime)
{
    if(integrationTime > 2000)
        return -HW_ERR_INVALID;

	unsigned char repeat = 1;
    uint16_t max_int_time = 999;  // us
    uint16_t new_int_time = 0;
    while(integrationTime > max_int_time){
        integrationTime = integrationTime - max_int_time;
        repeat++;
    }
    new_int_time = integrationTime + max_int_time*(repeat - 1);
    int value = (int)(floor((new_int_time/repeat)*120.0f/HMAX)*HMAX);

    int rtn = mlx75027_shadow_register(true);
    for(uint8_t phase = 0; phase < 4; phase++){
        // set integration time
        rtn |= sensor_write_reg(0x2120 + phase*4, (value >> 24)&0xFF);
        rtn |= sensor_write_reg(0x2121 + phase*4, (value >> 16)&0xFF);
        rtn |= sensor_write_reg(0x2122 + phase*4, (value >>  8)&0xFF);
        rtn |= sensor_write_reg(0x2123 + phase*4, (value & 0xFF));

        // set interval time
        rtn |= sensor_write_reg(0x2161 + phase*4, 0);
        rtn |= sensor_write_reg(0x2162 + phase*4, 0);
        rtn |= sensor_write_reg(0x2163 + phase*4, 0);
    }
    repeat = (repeat << 4) + repeat;
    // set repeat count
    rtn |= sensor_write_reg(0x21A0, (repeat& 0xFF));
    rtn |= sensor_write_reg(0x21A1, (repeat & 0xFF));

    rtn |= mlx75027_shadow_register(false);

    return rtn;
}

int mlx75027_set_phase_idle_time(uint8_t value)
{
    if(value < 5)  // value out side [0x05 - 0xFF] are prohibited
        return -HW_ERR_INVALID;
    int rtn = mlx75027_shadow_register(true);
    for(uint8_t phase = 0; phase < 4; phase++){

        rtn |= sensor_write_reg(0x21C8 + phase, value);
    }
    rtn |= mlx75027_shadow_register(false);

    return rtn;
}

/*
mode = 0: no binning (= VGA resolution, 640x480 pixels)  // default mode
mode = 1: 2x2 binning (= QVGA resolution, 320x240 pixels)
mode = 2: 4x4 binning (= QQVGA resolution, 160x120 pixels)
mode = 3: 8x8 binning (= QQQVGA resolution, 80x60 pixels)
*/
int mlx75027_set_pixel_binning(uint8_t mode)
{
    return sensor_write_reg(0x14A5, mode);
}

int mlx75027_get_pixel_binning(uint8_t *mode)
{
	return sensor_read_reg(0x14A5, *mode);
}

int mlx75027_pixelROI(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
    uint16_t value[4];
    value[0] = x1;
    value[1] = x2 - x1 + 1;
    value[2] = (y1 - 1)/2;
    value[3] = y2/2 + 1;

    int rtn = 0;
    for(uint8_t i = 0; i < 4; i++){

        rtn |= sensor_write_reg(0x0804 + i*2, (value[i] >> 8)&0xFF);
        rtn |= sensor_write_reg(0x0805 + i*2, (value[i] & 0xFF));
    }

    return rtn;
}

int mlx75027_get_img_mirror_flip(uint8_t *mode)
{
    int rtn = 0;
    uint8_t h = 0, v = 0;
    rtn |= sensor_read_reg(0x080C, &h);
    rtn |= sensor_read_reg(0x080D, &v);
    if(h == 0){
        if(v == 0)
            *mode = 0;
        else
            *mode = 2;
    }
    else{
        if(v == 0)
            *mode = 1;
        else
            *mode = 3;
    }
    return rtn;
}

int mlx75027_set_img_mirror_flip(uint8_t mode)
{
    int rtn = 0;
    if(mode == 0){
        rtn |= sensor_write_reg(0x080C, 0x00); // default
        rtn |= sensor_write_reg(0x080D, 0x00); // default
    }
    else if(mode == 1){
        rtn |= sensor_write_reg(0x080C, 0x01);
        rtn |= sensor_write_reg(0x080D, 0x00);
    }
    else if(mode == 2){
        rtn |= sensor_write_reg(0x080C, 0x00);
        rtn |= sensor_write_reg(0x080D, 0x01);
    }
    else if(mode == 3){
        rtn |= sensor_write_reg(0x080C, 0x01);
        rtn |= sensor_write_reg(0x080D, 0x01);
    }
    return rtn;
}

int mlx75027_get_sensor_temperature(float *temp)
{
    uint8_t value;
    int rtn = sensor_read_reg(0x1403, &value);

    *temp = (float)(value&0xff) - 40;
    //qDebug() << "value temp " << value << *temp;
	//DDEBUG("   value = %x",  value);
    return rtn;
}

int mlx75027_pixel_statistics(bool enable, uint8_t mode)
{
    int rtn = 0;
    if(enable){
        rtn |= sensor_write_reg(0x1433, 0x01);
        if(mode)
            rtn |= sensor_write_reg(0x14BB, 0x01); // error code enabled, [11:0] = 0x800
        else
            rtn |= sensor_write_reg(0x14BB, 0x00); // error flag enabled, [11] = error flag, [10:0] = pixel data
    }
    else{
        rtn |= sensor_write_reg(0x1433, 0x00); // default. if disabled, mode selection is invalid
    }

    return rtn;
}

// The minimum threshold for each tap.
int mlx75027_set_pixel_lower_limit(uint16_t value)
{
    int rtn = 0;
    for(uint8_t phase = 0; phase < 4; phase++){

        rtn |= sensor_write_reg(0x1434 + phase*2, (value >>  8)&0xFF);
        rtn |= sensor_write_reg(0x1435 + phase*2, (value & 0xFF));
    }

    return rtn;
}

// The maximum threshold for each tap.
int mlx75027_set_pixel_upper_limit(uint16_t value)
{
    int rtn = 0;
    for(uint8_t phase = 0; phase < 4; phase++){

        rtn |= sensor_write_reg(0x1448 + phase*2, (value >>  8)&0xFF);
        rtn |= sensor_write_reg(0x1449 + phase*2, (value & 0xFF));
    }

    return rtn;
}

int mlx75027_get_pixel_error_count_low(uint8_t phase, uint32_t *value)
{
    uint8_t Byte1, Byte2, Byte3;

    int rtn = sensor_read_reg(0x145D + phase*4, &Byte3);
    rtn |= sensor_read_reg(0x145E + phase*4, &Byte2);
    rtn |= sensor_read_reg(0x145F + phase*4, &Byte1);

    *value = Byte1 + (Byte2 << 8) + (Byte3 <<16);

    return rtn;
}

int mlx75027_get_pixel_error_count_high(uint8_t phase, uint32_t *value)
{
    uint8_t Byte1, Byte2, Byte3;

    int rtn = sensor_read_reg(0x1481 + phase*4, &Byte3);
    rtn |= sensor_read_reg(0x1482 + phase*4, &Byte2);
    rtn |= sensor_read_reg(0x1483 + phase*4, &Byte1);

    *value = Byte1 + (Byte2 << 8) + (Byte3 <<16);

    return rtn;
}

int mlx75027_test_pattern(uint8_t mode)
{
    int rtn = 0;
    if(mode){
        rtn |= sensor_write_reg(0x1405, 0x00);
        rtn |= sensor_write_reg(0x1406, 0x04);
        rtn |= sensor_write_reg(0x1407, 0x01);
    }
    else{
        rtn |= sensor_write_reg(0x1407, 0x00); // default
    }

    return rtn;
}

int mlx75027_get_sensor_info(struct sensor_info_t *info) {
	info->embedded_data_size = 972 * 2;
	info->vcsel_num = vcsel_number;
	info->vcsel_driver_id = vcsel_driver_type;
	info->sensor_id = mlx75027_sensor_id;
	return 0;
}

int mlx75027_illum_duty_cycle_adjust(uint8_t mode, uint8_t value)
{
    int rtn = 0;
    if(mode == 0){ // no duty cycle correction (= disabled) default.
        rtn |= sensor_write_reg(0x4E9E, 0x00);
    }
    else if(mode == 1){ // time delay on the falling edge (= increased duty cycle)
        rtn |= sensor_write_reg(0x4E9E, 0x01);
        rtn |= sensor_write_reg(0x21B9, value);
    }
    else if(mode == 2){ // time delay on the rising edge (= decreased duty cycle)
        rtn |= sensor_write_reg(0x4E9E, 0x02);
        rtn |= sensor_write_reg(0x21B9, value);
    }

    return rtn;
}

// different modulation frequency have different adjustable duty cycle range
int mlx75027_get_illum_duty_cycle_list(uint8_t mod_freq, float *duty_cycle_list)
{
    float duty = 0;
    float cycle = 1000.0f/mod_freq; // period ns
    for(int step = -15; step <= 15; step++){
		if(vcsel_driver_type == DAC5574){
			duty = (0.5f + (step*0.5f - 2.0f)/cycle)*100; // 0.5ns/step, 2.0 is a const missing caused by IC-HG vcsel driver
		}
		else if(vcsel_driver_type == QUAD_CXA4016){
			duty = (0.5f + (step*0.5f) / cycle) * 100;
		}
		if(duty < 0){
            duty = 0;
        }
        else if((duty > 100)){
            duty = 100;
        }
        duty_cycle_list[step + 15] = duty;
    }
																				 
    return 0;
}

int mlx75027_get_illum_duty_cycle(uint16_t *duty)
{
    uint8_t mode = 0, value = 0;
    int rtn = sensor_read_reg(0x4E9E, &mode);
    if(mode == 0)
        *duty = 15; // 50%
    else if(mode == 1){
        rtn |= sensor_read_reg(0x21B9, &value);
        *duty = 15 + value; // > 50%
    }
    else if(mode == 2){
        rtn |= sensor_read_reg(0x21B9, &value);
        *duty = 15 - value; // < 50%
    }

    return rtn;
}

int mlx75027_set_illum_duty_cycle(uint16_t duty)
{
    int rtn = 0;
    uint8_t value = 0;
    if(duty == 15){
        rtn = mlx75027_illum_duty_cycle_adjust(0, 0);
    }
    else if(duty > 15){
        value = duty - 15;
        rtn = mlx75027_illum_duty_cycle_adjust(2, 0);
        rtn |= mlx75027_illum_duty_cycle_adjust(1, value);
    }
    else if(duty < 15){
        value = 15 - duty;
        rtn = mlx75027_illum_duty_cycle_adjust(1, 0);
        rtn |= mlx75027_illum_duty_cycle_adjust(2, value);
    }
    return rtn;
}

int mlx75027_illum_signal(uint8_t mode)
{
    int rtn = 0;
    if(mode == 0){ // subLVDS mode (LEDP positive, LEDN negative) default.
        rtn |= sensor_write_reg(0x10E2, 0x01);
    }
    else // CMOS mode (LEDP = LEDN)
        rtn |= sensor_write_reg(0x10E2, 0x00);

    return rtn;
}

int mlx75027_metadata_output(uint8_t mode)
{
    int rtn = 0;
    if(mode == 0){ // no metadata lines enabled
        rtn |= sensor_write_reg(0x3C18, 0x00);
    }
    else if(mode == 1){ // first metadata line (line #1) enabled
        rtn |= sensor_write_reg(0x3C18, 0x01);
    }
    else if(mode == 2){ // first & second metadata lines (line #1 and line #2) enabled default.
        rtn |= sensor_write_reg(0x3C18, 0x02);
    }

    return rtn;
}

int mlx75027_video_streaming(bool enable)
{
    int rtn = 0;
    if(enable)
        rtn |= sensor_write_reg(0x1001, 0x01);
    else
        rtn |= sensor_write_reg(0x1001, 0x00);

    return rtn;
}

int mlx75027_init()
{
    int rtn = 0;
	vcsel_driver_detect();
	if (vcsel_driver_type == QUAD_CXA4016) {
		rtn = cxa4016_init();
	}
	else if (vcsel_driver_type == DAC5574) {
		rtn |= mlx75027_illum_power_control(true);
		rtn |= mlx75027_set_illum_power(0, 160, 25);

	}
    
    rtn |= mlx75027_sensor_initialize();
	rtn |= mlx75027_set_data_output_mode(0);
    
    
	if (!use_trigger_mode) {
		rtn |= mlx75027_set_stream_mode();
		rtn |= mlx75027_set_fps(30);
	}
	else {
		//use  trigger_mode
		rtn |= mlx75027_set_trigger_mode(1);
	}
    rtn |= mlx75027_set_modulation_frequency(60); // MHz
    rtn |= mlx75027_set_integration_time(300); // us
    return rtn;
}

int mlx75027_func_init()
{
#if !DEBUG_MLX75027_IN_QT
	tof_sensor.init = mlx75027_init;
	tof_sensor.get_sensor_id = mlx75027_get_sensor_id;
	tof_sensor.hardware_trigger = mlx75027_hardware_trigger;
	tof_sensor.software_trigger = mlx75027_software_trigger;
	tof_sensor.video_streaming = mlx75027_video_streaming;
	tof_sensor.get_fps = mlx75027_get_fps;
	tof_sensor.set_fps = mlx75027_set_fps;
	tof_sensor.get_sensor_temperature = mlx75027_get_sensor_temperature;
	tof_sensor.get_rx_temp = mlx75027_get_rx_temp;
	tof_sensor.get_tx_temp = mlx75027_get_tx_temp;
	tof_sensor.set_illum_power = mlx75027_set_illum_power;
	tof_sensor.get_illum_power = mlx75027_get_illum_power;
	tof_sensor.illum_power_control = mlx75027_illum_power_control;
	tof_sensor.get_integration_time = mlx75027_get_integration_time;
	tof_sensor.set_integration_time = mlx75027_set_integration_time;
	tof_sensor.get_modulation_frequency = mlx75027_get_modulation_frequency;
	tof_sensor.set_modulation_frequency = mlx75027_set_modulation_frequency;
	tof_sensor.get_illum_duty_cycle = mlx75027_get_illum_duty_cycle;
	tof_sensor.set_illum_duty_cycle = mlx75027_set_illum_duty_cycle;
	tof_sensor.get_data_output_mode = mlx75027_get_data_output_mode;
	tof_sensor.set_data_output_mode = mlx75027_set_data_output_mode;
	tof_sensor.get_img_mirror_flip = mlx75027_get_img_mirror_flip;
	tof_sensor.set_img_mirror_flip = mlx75027_set_img_mirror_flip;
	tof_sensor.get_pixel_binning = mlx75027_get_pixel_binning;
	tof_sensor.set_pixel_binning = mlx75027_set_pixel_binning;
	tof_sensor.test_pattern = mlx75027_test_pattern;
	tof_sensor.get_sensor_info = mlx75027_get_sensor_info;
	tof_sensor.get_illum_duty_cycle_list = mlx75027_get_illum_duty_cycle_list;
#endif
	return 0;
}
