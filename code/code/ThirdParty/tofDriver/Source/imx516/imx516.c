#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "imx516.h"


#define DDEBUG(fmt, ...)   \
	printf("[DEBUG] [%s(%d)] : " fmt"\n",__FUNCTION__,__LINE__,##__VA_ARGS__)


// GPIO control
#define TRIGGER_PIN          2 // PO1
#define LD_ENABLE_PIN        1 // PO2
#define LD_ERROR_PIN         3

// IIC slave device
#define SENSOR_ADDR          (0x10 << 1)
#define EEPROM_ADDR          (0x50 << 1)


static uint16_t HMAX = 1440; // 0x05A0
static float CLK120MHz = 120; // result turned out to be us


struct regList {
	uint16_t reg;
	uint8_t val;
};	

#if !DEBUG_IMX516_IN_QT
#include <tof_sensors.h>
#include <obc_tee_funcs.h>

#ifdef __linux__
#include <unistd.h>
#endif



//#define ALOGE(...) tops.qsee_log(TEE_LOG_LEVEL_ERROR, __VA_ARGS__)
#define ALOGE(...)
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
#define dothin_pmu_set_voltage     tops_t.ap_ops.PmuSetVoltage

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


static int sensor_write_reg(uint32_t reg, uint32_t value)
{
    int rtn = i2c_reg_write(SENSOR_ADDR, reg, 2, value, 1);
    return rtn;
}

static int sensor_read_reg(uint32_t reg, uint32_t *value)
{
    int rtn = i2c_reg_read(SENSOR_ADDR, reg, 2, value, 1);
    return rtn;
}

static int eeprom_write_byte(uint32_t reg, uint32_t value)
{
    int rtn = i2c_reg_write(EEPROM_ADDR, reg, 2, value, 1);
    return rtn;

}

static int eeprom_read_byte(uint32_t reg, uint32_t *value)
{
    int rtn = i2c_reg_read(EEPROM_ADDR, reg, 2, value, 1);
    return rtn;

}

int imx516_get_rx_temp(float *temperature)
{
    uint32_t value;
    int rtn = sensor_read_reg(0x1403, &value);

    *temperature = (float) (value&0xff) - 40;
    //("sensor temp %f", temperature);
    //qDebug() << "value temp " << value << *temp;

    return rtn;
}

int imx516_get_tx_temp(float *temperature)
{
    uint32_t valueH = 0;
    uint32_t valueL = 0;
	uint32_t value = 0;
#if 0
    int rtn = sensor_read_reg(0x153d, &valueH);
    rtn |= sensor_read_reg(0x153e, &valueL);
    value = ((valueH & 0x03) << 8) + valueL;
	*temperature = 25 + value/5.4f;
    //printf("valueH %d, valueL %d\n", valueH, valueL);
#endif

    int rtn = 0;
    int times = 2;
    
    for (int i = 0; i < times; i++) {
        rtn = 0;
        rtn = sensor_read_reg(0x153d, &valueH);
        rtn |= sensor_read_reg(0x153e, &valueL);
        value = ((valueH & 0x03) << 8) + valueL;
        if (value != 0x2ff && value != 0x100) {
            break;
        }
        else {
            printf("--->value %d,  valueH %d, valueL %d    \n", value, valueH, valueL);
            
        }
    }
    *temperature = 25 + ((value - 296)) / 5.4f;
    //printf("value %d,  valueH %d, valueL %d    tx=%f  \n", value, valueH, valueL, *temperature);
    return rtn;
}

int imx516_set_illum_power(uint8_t vcsel_num, uint8_t value_A, uint8_t value_B)
{/*
    uint32_t value = volt << 8;
    uint8_t reg = 0x10 + channel*2;  // 0x10 channel A, 0x12 channel B
    int rtn = dac5574_write_reg(reg, value);

    return rtn;*/
    return 0;
}

int imx516_get_illum_power(uint8_t *vcsel_num, uint8_t *value_A, uint8_t *value_B)
{/*
    uint32_t value;
    uint8_t reg = 0x10 + channel*2;
    int rtn = dac5574_read_reg(reg, &value);
    *volt = (value >> 8)& 0xff;
    //qDebug() << "get illum value " << *volt;
    return rtn;*/
    return 0;
}

int imx516_hardware_trigger()
{/*
    //mdevices.gpio_control(TRIGGER_PIN, 1);
    mdevices.gpio_control(TRIGGER_PIN, 0); // active low
    mdevices.sleepms(0.001); // 1us, not accurate
    //mdevices.usleep(100);
    mdevices.gpio_control(TRIGGER_PIN, 1);
    */
    return 0;
}

int imx516_software_trigger()
{/*
    int rtn = sensor_write_reg(0x2100, 0x01);
    return rtn;*/
	return 0;
}

int imx516_illum_power_control(bool enable)
{
    //return gpio_control(LD_ENABLE_PIN, enable); // active high
	return 0;
}

static struct regList reglist[] = {

    //External clock frequency [MHz]
    {0x1006,  0x18},
    {0x1007,  0x00},
    {0x1000,  0x00},
    {0x2268,  0x40},
    {0x2269,  0x01},
    {0x226A,  0x42},
    {0x226B,  0x11},
    {0x400E,  0x03},
    {0x400F,  0x84},
    {0x402B,  0x02},
    {0x405C,  0x00},
    {0x4130,  0x08},
    {0x4131,  0x02},
    {0x4132,  0x02},
    {0x4133,  0x02},
    {0x4134,  0x00},
    {0x4135,  0x02},
    {0x4136,  0x00},
    {0x4137,  0x02},
    {0x4138,  0x00},
    {0x4139,  0x02},
    {0x413A,  0x00},
    {0x413B,  0x02},
    {0x413C,  0x00},
    {0x413D,  0x02},
    {0x443C,  0x00},
    {0x443D,  0x79},
    {0x443E,  0x00},
    {0x443F,  0x8A},
    {0x4596,  0x0B},
    {0x4597,  0x45},
    {0x45AE,  0x0B},
    {0x45AF,  0x45},
    {0x45B6,  0x0B},
    {0x45B7,  0xBD},
    {0x45BA,  0x0B},
    {0x45BB,  0x45},
    {0x45C6,  0x0B},
    {0x45C7,  0x45},
    {0x45CE,  0x0B},
    {0x45CF,  0xBD},
    {0x45F8,  0x1F},
    {0x45F9,  0xFF},
    {0x47B4,  0x00},
    {0x47B5,  0x56},
    {0x47BE,  0x00},
    {0x47BF,  0x3C},
    {0x47C4,  0x00},
    {0x47C5,  0x00},
    {0x47C6,  0x03},
    {0x47C7,  0xFF},
    {0x47D0,  0x00},
    {0x47D1,  0x54},
    {0x47DA,  0x00},
    {0x47DB,  0x3F},
    {0x47DC,  0x00},
    {0x47DD,  0x00},
    {0x47DE,  0x03},
    {0x47DF,  0xFF},
    {0x493C,  0x07},
    {0x494E,  0x02},
    {0x4998,  0x01},
    {0x4999,  0x01},
    {0x4A01,  0x01},
    {0x4A05,  0x01},
    {0x5852,  0x0A},
    {0x5853,  0xD5},
    {0x5854,  0x0A},
    {0x5855,  0xDB},
    {0x5856,  0x0B},
    {0x5857,  0x43},
    {0x586D,  0x00},
    {0x586E,  0x2E},
    {0x586F,  0xEB},
    {0x5881,  0x00},
    {0x5882,  0x3B},
    {0x5883,  0x28},
    {0x5885,  0x00},
    {0x5886,  0x05},
    {0x5887,  0x14},
    {0x5889,  0x00},
    {0x588A,  0x03},
    {0x588B,  0xAC},

    {0x1040,  0x00},
    {0x1041,  0x78},
    {0x1042,  0x02},
    {0x1048,  0x00},
    {0x1049,  0x64},
    {0x104a,  0x03},
    {0x104b,  0x02},

    {0x100c,  0x05},
    {0x100d,  0xA0},
    {0x100e,  0x00},
    {0x100f,  0x00},
    {0x1016,  0x04},
    {0x1017,  0x00},
    {0x1060,  0x01},
    {0x1070,  0x06},
    {0x1071,  0x0C},
    {0x020e,  0x02},
    {0x020f,  0x58},
    {0x1010,  0x01},

    {0x0800,  0x05},
    {0x0801,  0xA6},
    {0x2108,  0x02},
    {0x2109,  0x58},
    {0x4015,  0x00},
    {0x4016,  0x36},
    {0x4078,  0x00},
    {0x4079,  0x00},
    {0x407a,  0x00},
    {0x407b,  0x1A},
    {0x4080,  0x00},
    {0x4081,  0x00},
    {0x4082,  0x00},
    {0x4083,  0x36},


    {0x082C,  0x22},
    {0x082D,  0x22},
    {0x082E,  0x22},
    {0x082F,  0x22},
    {0x0830,  0x22},
    {0x0831,  0x22},
    {0x0832,  0x22},
    {0x0833,  0x22},
    {0x0834,  0x22},
    {0x0835,  0x22},
    {0x0836,  0x22},
    {0x0837,  0x22},
    {0x0838,  0x33},
    {0x0839,  0x33},
    {0x083a,  0x33},
    {0x083b,  0x33},
    {0x083c,  0x33},
    {0x083d,  0x33},
    {0x083e,  0x33},
    {0x083f,  0x33},
    {0x0840,  0x33},
    {0x0841,  0x33},
    {0x0842,  0x33},
    {0x0843,  0x33},
    {0x0844,  0x00},
    {0x0848,  0x00},
    {0x084c,  0xFF},


    {0x213c,  0x00},
    {0x213d,  0x00},
    {0x2140,  0x00},
    {0x2141,  0x00},
    {0x2144,  0x00},
    {0x2145,  0x00},


    {0x1433,  0x00}, // pixel statistic
    {0x149b,  0x00},
    {0x1434,  0x00},
    {0x1435,  0x00},
    {0x1436,  0x00},
    {0x1437,  0x00},
    {0x1438,  0x00},
    {0x1439,  0x00},
    {0x143c,  0x06},
    {0x143d,  0x40},
    {0x143e,  0x06},
    {0x143f,  0x40},
    {0x1440,  0x07},
    {0x1441,  0xFF},

    {0x2c08,  0x02},
    {0x2c09,  0x80},
    {0x3c18,  0x03}, // 0x03, 3 line embedded data
    {0x2c0c,  0x01},
    {0x0804,  0x00},
    {0x0805,  0x04},
    {0x0806,  0x02},
    {0x0807,  0x84},
    {0x0808,  0x00},
    {0x0809,  0x04},
    {0x080a,  0x01},
    {0x080b,  0xE3},
    {0x080c,  0x00},
    {0x080d,  0x00},
    {0x080e,  0x00},

    {0x0810,  0x00},
    {0x0811,  0x00},

    {0x2247,  0x18},
    {0x2248,  0x00},
    {0x2249,  0x00},
    {0x224a,  0x00},
    {0x2254,  0x04},
    {0x2255,  0x00},
    {0x2256,  0x00},

    {0x2124,  0x00},
    {0x2125,  0x01},
    {0x2126,  0xd4}, //0x19 },
    {0x2127,  0xc0}, //0x40 },
    {0x2128,  0x00},
    {0x2129,  0x01},
    {0x212a,  0xd4}, //0x19 },
    {0x212b,  0xc0}, //0x40 },
    {0x212c,  0x00},
    {0x212d,  0x00},
    {0x212e,  0x09},
    {0x212f,  0x60},


    {0x2118,  0x00},
    {0x2119,  0x00},
    {0x211a,  0x00},
    {0x211b,  0x00},
    {0x211c,  0x00},
    {0x211d,  0x00},
    {0x211e,  0x00},
    {0x211f,  0x00},
    {0x2120,  0x00},
    {0x2121,  0x00},
    {0x2122,  0x00},
    {0x2123,  0x00},

    {0x215c,  0x04},
    {0x215d,  0x04},
    {0x215e,  0x04},
    {0x214c,  0x00},
    {0x214d,  0x01},
    {0x214e,  0x00},
    {0x217c,  0x04},
    {0x217d,  0x0A},
    {0x217e,  0x04},

    {0x210c,  0x00},
    {0x210d,  0x00},
    {0x210e,  0x00},
    {0x210f,  0x00},
    {0x2110,  0x00},
    {0x2111,  0x00},
    {0x2112,  0x1E},
    {0x2113,  0xBA},
    {0x2114,  0x00},
    {0x2115,  0x00},
    {0x2116,  0x00},
    {0x2117,  0x00},
    {0x2100,  0x48},
    {0x0828,  0x01},
    {0x2164,  0x00},
    {0x2165,  0x00},
    {0x2168,  0x00},
    {0x2169,  0x00},
    {0x216c,  0x00},
    {0x216d,  0x00},


    {0x2184,  0x0D},
    {0x2188,  0x06},
    {0x2189,  0x08},
    {0x218a,  0x0A},
    {0x218b,  0x0C},
    {0x2190,  0x06},
    {0x2191,  0x0B},
    {0x2192,  0x10},
    {0x2193,  0x15},
    {0x2198,  0x06},
    {0x2199,  0x08},
    {0x219a,  0x0A},
    {0x219b,  0x0C},


    {0x2244,  0x00},
    {0x2245,  0x00},
    {0x2246,  0x00},
    {0x21b8,  0x04},
    {0x21b9,  0x07},
    {0x21ba,  0x02},


    {0x0145,  0x00},
    {0x0148,  0x01},

    {0x2020,  0x01},
    {0x3071,  0x03},
    {0x2f05,  0x01},
    {0x2f06,  0x00},
    {0x2f07,  0x07},
    {0x2001,  0x01},


    //Laser Driver (CXA4016)  ADC initialization
    {0x0403,  0x20},
    {0x0405,  0x00},
    {0x0450,  0x47},
    {0x0500,  0x02},
    {0x0501,  0x13},
    {0x0502,  0x23}, //0x27 },
    {0x0401,  0x01},
    {0x0400,  0x01},
    {0x0401,  0x00},

    //Laser Driver(CXA4016) reset
    {0x0403,  0x20},
    {0x0405,  0x00},
    {0x0450,  0x47},
    {0x0500,  0x02},
    {0x0501,  0x25},
    {0x0502,  0x01},
    {0x0401,  0x01},
    {0x0400,  0x01},
    {0x0401,  0x00},

    //Sony Laser Driver (CXA4016)  initial
    {0x0403,  0x20},
    {0x0405,  0x00},
    {0x0407,  0x03},
    {0x0503,  0x15},
    {0x0504,  0x00},
    {0x0505,  0x0c}, //0x1C },	//{0x04},	//APC
    {0x0506,  0xA0},
    {0x0507,  0xFF},
    {0x0508,  0x1A},
    {0x0509,  0x28},
    {0x050A,  0x69},
    {0x050B,  0x87},
    {0x050C,  0x85},	//IBIAS_FIX
    {0x050D,  0xB3},	//ISW_FIX
    {0x050E,  0x14},
    {0x050F,  0x05},
    {0x0510,  0xA4},
    {0x0511,  0x28},
    {0x0512,  0xc3},
    {0x0513,  0xc7},	//{0x0b},	//ISW_FIX threshold
    {0x0514,  0x72},
    {0x0515,  0x0F},
    {0x0516,  0x00},
    {0x0517,  0x00},
    {0x0518,  0x01},
    {0x0401,  0x01},
    {0x0400,  0x01},
    {0x0401,  0x00},
    {0x0403,  0x20},
    {0x0405,  0x00},
    {0x0407,  0x22},
    {0x0522,  0x02},
    {0x0523,  0x23},
    {0x0524,  0x00},
    {0x0401,  0x01},
    {0x0400,  0x01},
    {0x0401,  0x00},
    {0x0413,  0xA0},
    {0x0417,  0x19},
    {0x0423,  0xA0},
    {0x0425,  0x04},
    {0x0427,  0x1C},
    {0x042a,  0x01},
    {0x0519,  0x02},
    {0x051A,  0x13},
    {0x051B,  0x21},//0x25 },
    {0x051C,  0x02},
    {0x051D,  0xA3},
    {0x051E,  0x12},
    {0x051F,  0x94},
    {0x0520,  0x04},
    {0x0521,  0x80},
    {0x0522,  0x08},
    {0x0523,  0x85},
    {0x0524,  0x05},
    {0x0525,  0x8D},
    {0x0411,  0x01},
    {0x0421,  0x01}
    //{0x1001,  0x01},

};


int imx516_dothin_config()
{
#if !DEBUG_IMX516_IN_QT
    int ret = 0;

    DDEBUG("dothin_device_id=%d\n", dothin_device_id);

    ret = dothin_enable_softpin(true, dothin_device_id);
    if (ret < 0) {
        DDEBUG("dothin_enable_softpin ret=%d\n", ret);
    }
    ret = dothin_enable_gpio(true, dothin_device_id);
    if (ret < 0) {
        DDEBUG("dothin_enable_gpio ret=%d\n", ret);
    }
    ret = dothin_set_sensor_clock(true, 24 * 10, dothin_device_id); // 8Mhz mclk
    if (ret < 0) {
        DDEBUG("dothin_set_sensor_clock ret=%d\n", ret);
    }
    ret = dothin_set_softpin_pullup(1, dothin_device_id);
    if (ret < 0) {
        DDEBUG("dothin_set_softpin_pullup ret=%d\n", ret);
    }
    usleep(1000 * 10);
    ret = dothin_set_sensor_i2c_rate(1, dothin_device_id); // 400Khz
    if (ret < 0) {
        DDEBUG("dothin_set_sensor_i2c_rate ret=%d\n", ret);
    }
    ret = dothin_sensor_enable(1, true, dothin_device_id);
    if (ret < 0) {
        DDEBUG("dothin_sensor_enable ret=%d\n", ret);
    }
    usleep(1000 * 20);
    ret = dothin_sensor_enable(3, true, dothin_device_id);
    if (ret < 0) {
        DDEBUG("dothin_sensor_enable ret=%d\n", ret);
    }
    usleep(1000 * 50);

     ret = dothin_sensor_enable(3, true, dothin_device_id);
    if (ret < 0) {
        DDEBUG("dothin_sensor_enable ret=%d\n", ret);
    }
    usleep(1000 * 50);

    SENSOR_POWER sensor_power[] = { POWER_AVDD, POWER_DOVDD,POWER_DVDD ,POWER_AFVCC ,POWER_VPP };
    int           power_value[] = { 2800,       1800,      1260,        3300,       1200 };
    ret = dothin_pmu_set_voltage(sensor_power, power_value,5, dothin_device_id);
    if (ret < 0) {
        DDEBUG("dothin_sensor_enable ret=%d", ret);
    }
    usleep(1000 * 50);
    DDEBUG("end");
    return ret;
#endif


}

int imx516_sensor_initialize()
{
    int rtn = 0;
    for(int i = 0; i < sizeof(reglist)/sizeof(struct regList); i++){

        rtn |= sensor_write_reg(reglist[i].reg, reglist[i].val);
    }
    return rtn;
}

int imx516_shadow_register(bool enable)
{
    int rtn = 0;
    if(enable)
        rtn = sensor_write_reg(0x0102, 0x01);
    else
        rtn = sensor_write_reg(0x0102, 0x00);
    return rtn;
}

int imx516_get_sensor_id(uint16_t *id)
{
    int rtn = 0;
    uint32_t value = 0;
    static int isConfig = 0;
    if (!isConfig) {
        isConfig = 1;
        imx516_dothin_config();
    }

    rtn = sensor_read_reg(0x0000, &value);
    if(value == 0x8c)
        *id = imx516_sensor_id;
    return rtn;
}

int imx516_set_user_ID(uint8_t value)
{
    //return sensor_write_reg(0x0824, value);
	return 0;
}

int imx516_set_hmax(uint16_t hmax)
{/*
    int rtn = 0;
    // set HMAX register
    rtn = sensor_write_reg(0x0800, (hmax >>  8)&0xFF);
    rtn |= sensor_write_reg(0x0801, (hmax &0xFF));

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

    return rtn;*/
	return 0;
}

int imx516_set_trigger_mode(uint8_t mode) // should stop stream first
{/*
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
    return rtn;*/
	return 0;
}

int imx516_set_stream_mode() // should stop stream first
{/*
    int rtn = 0;
    rtn = sensor_write_reg(0x2020, 0x01);
    rtn |= sensor_write_reg(0x2100, 0x08);
    rtn |= sensor_write_reg(0x2F05, 0x01);
    rtn |= sensor_write_reg(0x2F06, 0x09);
    rtn |= sensor_write_reg(0x2F07, 0x7A);
    rtn |= sensor_write_reg(0x3071, 0x00);
    return rtn;*/
	return 0;
}

int imx516_get_data_output_mode(uint8_t *mode)
{
    uint32_t value = 0;
    int rtn = sensor_read_reg(0x082C, &value);
    *mode = value&0x0f;
    return rtn;
}

// be careful that the resolution of A&B mode will change to 1280*480
int imx516_set_data_output_mode(uint8_t mode) // should stop streaming before change data output mode
{
    // mode = 0: A-B, 1: A+B, 2: A, 3: B, 4: A&B
    if(mode > 4)
        return -1;
    int rtn = 0;
    uint8_t set_mode = (mode << 4) + mode;
    for(int i = 0; i < 8; i++){ // set only group A and group B here
        rtn = sensor_write_reg(0x082C + i, set_mode);
    }
    return rtn;
}

int imx516_get_modulation_frequency(uint16_t *modFreq)
{/*
    uint32_t divselpre = 0;
    uint32_t divsel = 0;
    int rtn = sensor_read_reg(0x21BE, &divselpre);
    rtn |= sensor_read_reg(0x21BF, &divsel);

    uint32_t fmod_H, fmod_L;
    rtn |= sensor_read_reg(0x1048, &fmod_H);
    rtn |= sensor_read_reg(0x1049, &fmod_L);
    uint16_t fmod = (fmod_H << 8) + fmod_L;

    *modFreq = fmod/pow(2, (divselpre + divsel));

    return rtn;*/
	return 0;
}

int imx516_set_modulation_frequency(uint16_t modFreq)
{/*
    if(modFreq > 100 || modFreq < 4)
        return -1;

    int rtn = imx516_shadow_register(true);
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

    uint16_t fmod = pow(2, (divselpre + divsel))*modFreq;
    //qDebug() << "divselpre, divsel, fmod, modFreq" << divselpre << divsel << fmod << modFreq;
    rtn |= sensor_write_reg(0x1048, (fmod >> 8)&0xFF);
    rtn |= sensor_write_reg(0x1049, (fmod & 0xFF));

    if(fmod*8 < 900 && fmod*8 >= 500)
        rtn |= sensor_write_reg(0x104B, 0x02);
    else if(fmod*8 <= 1200 && fmod*8 >= 900)
        rtn |= sensor_write_reg(0x104B, 0x00);

    rtn |= imx516_shadow_register(false);

    return rtn;*/
	return 0;
}

// param uint: us
int imx516_get_frame_startup_time(uint16_t *startupTime)
{
    uint32_t byte0, byte1;
    int rtn = sensor_read_reg(0x21D4, &byte1);
    rtn |= sensor_read_reg(0x21D5, &byte0);
    uint16_t value = (byte1 << 8) + byte0;
    *startupTime = value*HMAX/120;

    return rtn;
}														  
int imx516_set_frame_startup_time(uint16_t startupTime)
{
    uint16_t value = (startupTime*120)/HMAX;
    int rtn = sensor_write_reg(0x21D4, (value >> 8)&0xFF);
    rtn |= sensor_write_reg(0x21D5, (value & 0xFF));

    return rtn;
}

// param uint: us
int imx516_set_frame_time(uint32_t frameTime)
{
    uint32_t value = (frameTime*120)/HMAX;
    int rtn = sensor_write_reg(0x2108, (value >> 24)&0xFF);
    rtn |= sensor_write_reg(0x2109, (value >> 16)&0xFF);
    rtn |= sensor_write_reg(0x210A, (value >> 8)&0xFF);
    rtn |= sensor_write_reg(0x210B, (value & 0xFF));

    return rtn;
}

int imx516_get_fps(uint8_t *fps)
{/*
    uint8_t byte0, byte1, byte2, byte3;
    int rtn = sensor_read_reg(0x2108, &byte3);
    rtn |= sensor_read_reg(0x2109, &byte2);
    rtn |= sensor_read_reg(0x210A, &byte1);
    rtn |= sensor_read_reg(0x210B, &byte0);

    uint32_t value = (byte3 << 24) + (byte2 << 16) + (byte1 << 8) + byte0;
    uint32_t frameTime = (value*HMAX)/120;
    *fps = (uint8_t)(1000000/frameTime);
    printf("byte 3 2 1 0:  %x %x %x %x", byte3, byte2, byte1, byte0);
    printf("frameTime %d, fps %d", frameTime, *fps);
    return rtn;*/
	return 0;
}

int imx516_set_fps(uint8_t fps)
{/*
    uint32_t frameTime = 1000000/fps; // us
    int rtn = imx516_set_frame_time(frameTime);

    return rtn;*/
	return 0;
}

int imx516_set_phase_count(uint8_t phaseCount)
{
    if(phaseCount > 8 || phaseCount < 1)
        return -1;
    int rtn = sensor_write_reg(0x21E8, phaseCount); // default is 4

    return rtn;
}

int imx516_set_phase_pretime(uint16_t preTime)
{
    if(preTime > 2000 || preTime < 1)  // value 0 is not allowed, 2000 is user defined
        return -1;
    int rtn = imx516_shadow_register(true);
    rtn |= sensor_write_reg(0x4015, (preTime >> 8)&0xFF);
    rtn |= sensor_write_reg(0x4016, (preTime & 0xFF));
    rtn |= imx516_shadow_register(false);

    return rtn;
}

int imx516_get_integration_time(uint16_t *integrationTime)
{
    uint8_t byte0, byte1, byte2, byte3;
    int rtn = sensor_read_reg(0x2124, &byte3);
    rtn |= sensor_read_reg(0x2125, &byte2);
    rtn |= sensor_read_reg(0x2126, &byte1);
    rtn |= sensor_read_reg(0x2127, &byte0);

    uint32_t value = (byte3 << 24) + (byte2 << 16) + (byte1 << 8) + byte0;

    *integrationTime = (uint16_t)(value/CLK120MHz); // may slightly different from what we set because of floor function

    //printf("imx516_get_integration_time %d   ret=%d\n",*integrationTime,rtn);

    return rtn;
}

int imx516_set_integration_time(uint16_t integrationTime)
{
    if(integrationTime > 1000)
        return -1;

	unsigned int value = integrationTime*CLK120MHz;

    int rtn = imx516_shadow_register(true);
    for(int group = 0; group < 3; group++){
        // set integration time
        rtn |= sensor_write_reg(0x2124 + group*4, (value >> 24)&0xFF);
        rtn |= sensor_write_reg(0x2125 + group*4, (value >> 16)&0xFF);
        rtn |= sensor_write_reg(0x2126 + group*4, (value >>  8)&0xFF);
        rtn |= sensor_write_reg(0x2127 + group*4, (value & 0xFF));
    }

    rtn |= imx516_shadow_register(false);
    //printf("imx516_set_integration_time %d,  ret=%d\n", integrationTime,rtn);
    return rtn;
}

int imx516_set_phase_idle_time(uint8_t value)
{
    if(value < 5)  // value out side [0x05 - 0xFF] are prohibited
        return -1;
    int rtn = imx516_shadow_register(true);
    for(int phase = 0; phase < 4; phase++){

        rtn |= sensor_write_reg(0x21C8 + phase, value);
    }
    rtn |= imx516_shadow_register(false);

    return rtn;
}

int imx516_set_phase_led_enable_pulse(uint8_t phaseLedEn)
{
    return sensor_write_reg(0x21C4, phaseLedEn); // default is 0x00, disable 8 phase .
}

/*
mode = 0: no binning (= VGA resolution, 640x480 pixels)  // default mode
mode = 1: 2x2 binning (= QVGA resolution, 320x240 pixels)
mode = 2: 4x4 binning (= QQVGA resolution, 160x120 pixels)
mode = 3: 8x8 binning (= QQQVGA resolution, 80x60 pixels)
*/
int imx516_pixel_binning(uint8_t mode)
{
    return sensor_write_reg(0x14A5, mode);
}

int imx516_pixelROI(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
    uint16_t value[4];
    value[0] = x1;
    value[1] = x2 - x1 + 1;
    value[2] = (y1 - 1)/2;
    value[3] = y2/2 + 1;

    int rtn = 0;
    for(int i = 0; i < 4; i++){

        rtn |= sensor_write_reg(0x0804 + i*2, (value[i] >> 8)&0xFF);
        rtn |= sensor_write_reg(0x0805 + i*2, (value[i] & 0xFF));
    }

    return rtn;
}

int imx516_get_img_mirror_flip(uint8_t *mode)
{
    int rtn = 0;
    uint32_t h = 0, v = 0;
    rtn |= sensor_read_reg(0x0811, &h);
    rtn |= sensor_read_reg(0x0810, &v);
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

int imx516_set_img_mirror_flip(uint8_t mode)
{
    int rtn = 0;
    if(mode == 0){
        rtn |= sensor_write_reg(0x0811, 0x00); // default
        rtn |= sensor_write_reg(0x0810, 0x00); // default
    }
    else if(mode == 1){
        rtn |= sensor_write_reg(0x0811, 0x01);
        rtn |= sensor_write_reg(0x0810, 0x00);
    }
    else if(mode == 2){
        rtn |= sensor_write_reg(0x0811, 0x00);
        rtn |= sensor_write_reg(0x0810, 0x01);
    }
    else if(mode == 3){
        rtn |= sensor_write_reg(0x0811, 0x01);
        rtn |= sensor_write_reg(0x0810, 0x01);
    }
    return rtn;
}

int imx516_get_sensor_temperature(float *temp)
{
    uint32_t value;
    int rtn = sensor_read_reg(0x1403, &value);

    *temp = (float) (value&0xff) - 40;
    //printf("sensor temp %f",temp);
    //qDebug() << "value temp " << value << *temp;

    return rtn;
}

int imx516_pixel_statistics(bool enable, uint8_t mode)
{
    int rtn = 0;
    if(enable){
        rtn |= sensor_write_reg(0x1433, 0x01);
        if(mode)
            rtn |= sensor_write_reg(0x149B, 0x01); // error code enabled, [11:0] = 0x800
        else
            rtn |= sensor_write_reg(0x149B, 0x00); // error flag enabled, [11] = error flag, [10:0] = pixel data
    }
    else{
        rtn |= sensor_write_reg(0x1433, 0x00); // default. if disabled, mode selection is invalid
    }

    return rtn;
}

// The minimum threshold for each tap.
int imx516_set_pixel_lower_limit(uint16_t value)
{
    int rtn = 0;
    for(int phase = 0; phase < 4; phase++){

        rtn |= sensor_write_reg(0x1434 + phase*2, (value >>  8)&0xFF);
        rtn |= sensor_write_reg(0x1435 + phase*2, (value & 0xFF));
    }

    return rtn;
}

// The maximum threshold for each tap.
int imx516_set_pixel_upper_limit(uint16_t value)
{
    int rtn = 0;
    for(int phase = 0; phase < 4; phase++){

        rtn |= sensor_write_reg(0x1448 + phase*2, (value >>  8)&0xFF);
        rtn |= sensor_write_reg(0x1449 + phase*2, (value & 0xFF));
    }

    return rtn;
}

int imx516_get_pixel_error_count_low(uint8_t phase, uint32_t *value)
{
    uint32_t Byte1, Byte2, Byte3;

    int rtn = sensor_read_reg(0x145D + phase*4, &Byte3);
    rtn |= sensor_read_reg(0x145E + phase*4, &Byte2);
    rtn |= sensor_read_reg(0x145F + phase*4, &Byte1);

    *value = Byte1 + (Byte2 << 8) + (Byte3 <<16);

    return rtn;
}

int imx516_get_pixel_error_count_high(uint8_t phase, uint32_t *value)
{
    uint32_t Byte1, Byte2, Byte3;

    int rtn = sensor_read_reg(0x1481 + phase*4, &Byte3);
    rtn |= sensor_read_reg(0x1482 + phase*4, &Byte2);
    rtn |= sensor_read_reg(0x1483 + phase*4, &Byte1);

    *value = Byte1 + (Byte2 << 8) + (Byte3 <<16);

    return rtn;
}

int imx516_test_pattern(uint8_t mode)
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

int imx516_illum_duty_cycle_adjust(uint8_t mode, uint8_t value)
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
int imx516_get_illum_duty_cycle_list(uint8_t mod_freq, float *duty_cycle_list)
{
    float duty = 0;
    float cycle = 1000.0f/mod_freq; // period ns
    for(int step = -15; step <= 15; step++){
        duty = (0.5f + (step*0.5f - 2.0f)/cycle)*100; // 0.5ns/step, 2.0ns is a const missing caused by IC-HG vcsel driver
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

int imx516_illum_signal(uint8_t mode)
{
    int rtn = 0;
    if(mode == 0){ // subLVDS mode (LEDP positive, LEDN negative) default.
        rtn |= sensor_write_reg(0x10E2, 0x01);
    }
    else // CMOS mode (LEDP = LEDN)
        rtn |= sensor_write_reg(0x10E2, 0x00);

    return rtn;
}

int imx516_metadata_output(uint8_t mode)
{
    int rtn = 0;
    if(mode == 0){ // no metadata lines enabled
        rtn |= sensor_write_reg(0x3C18, 0x00);
    }
    else if(mode == 1){ // first metadata line (line #1) enabled
        rtn |= sensor_write_reg(0x3C18, 0x01);
    }
    else if(mode == 2){ // first & second metadata lines (line #1 and line #2) enabled default.
        rtn |= sensor_write_reg(0x3C18, 0x10);
    }

    return rtn;
}

int imx516_video_streaming(bool enable)
{
    int rtn = 0;
    if(enable)
        rtn |= sensor_write_reg(0x1001, 0x01);
    else
        rtn |= sensor_write_reg(0x1001, 0x00);

    return rtn;
}

int imx516_init()
{
    int rtn = 0;
    //rtn |= imx516_illum_power_control(true);
    //rtn |= imx516_set_illum_power(0, 150);
    //rtn |= imx516_set_illum_power(1, 25);
    rtn |= imx516_sensor_initialize();
    rtn |= imx516_set_data_output_mode(0);
/*
    if (0) {
        rtn |= imx516_set_stream_mode();
        rtn |= imx516_set_fps(30);
	}
	else {
		//use  trigger_mode
        rtn |= imx516_set_trigger_mode(1);
	}
    rtn |= imx516_set_modulation_frequency(60); // MHz
    rtn |= imx516_set_integration_time(300); // us
*/
    return rtn;
}

int imx516_func_init()
{
#if !DEBUG_IMX516_IN_QT
    tof_sensor.init = imx516_init;
    tof_sensor.get_sensor_id = imx516_get_sensor_id;
    tof_sensor.hardware_trigger = imx516_hardware_trigger;
    tof_sensor.software_trigger = imx516_software_trigger;
    tof_sensor.video_streaming = imx516_video_streaming;
    tof_sensor.get_fps = imx516_get_fps;
    tof_sensor.set_fps = imx516_set_fps;
    tof_sensor.get_sensor_temperature = imx516_get_sensor_temperature;
    tof_sensor.get_rx_temp = imx516_get_rx_temp;
    tof_sensor.get_tx_temp = imx516_get_tx_temp;
    tof_sensor.set_illum_power = imx516_set_illum_power;
    tof_sensor.get_illum_power = imx516_get_illum_power;
    tof_sensor.illum_power_control = imx516_illum_power_control;
    tof_sensor.get_integration_time = imx516_get_integration_time;
    tof_sensor.set_integration_time = imx516_set_integration_time;
    tof_sensor.get_modulation_frequency = imx516_get_modulation_frequency;
    tof_sensor.set_modulation_frequency = imx516_set_modulation_frequency;
    tof_sensor.get_data_output_mode = imx516_get_data_output_mode;
    tof_sensor.set_data_output_mode = imx516_set_data_output_mode;
    tof_sensor.get_img_mirror_flip = imx516_get_img_mirror_flip;
    tof_sensor.set_img_mirror_flip = imx516_set_img_mirror_flip;
    tof_sensor.test_pattern = imx516_test_pattern;
#endif
	return 0;
}
