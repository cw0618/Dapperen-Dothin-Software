#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "pleco.h"
#include "hw_obstatus.h"
#include "debug2log.h"
#include "hw_property.h"
#include "tofinfo.h"

//#define DDEBUG(fmt, ...)   \
//    printf("[DEBUG] [%s(%d)] : " fmt"\n",__FUNCTION__,__LINE__,##__VA_ARGS__)

//#define TEE_LOG_LEVEL_ERROR        8
//#define DDEBUG(fmt,...) tops_t.qsee_log(TEE_LOG_LEVEL_ERROR, "[DEBUG] [%s(%d)] : " fmt"\n",__FUNCTION__,__LINE__,##__VA_ARGS__)

//#define X210116                      3 //X210116项目：3使用泛光，2使用散斑。
//#define X210116_TX_TYPE              2 //3表示三节，2表示双节，1表示单节

#define GPIO_RESET_PIN               3
#define GPIO_LEVEL_HIGH              1
#define GPIO_LEVEL_LOW               0

#define GPIO_EXPANDER_RST_PIN_OB    3 // PO3, OB TB6309 board
#define CXA4046_XCLR                2 // PO1

#define PLECO_I2C_ADDR              0x20 // TOF sensor
#define TCA6408A_I2C_ADDR           0x40 // GPIO expander
#define AD7490_I2C_ADDR             0x98 // ADC
#define FIRST_AD5254_I2C_ADDR       0x5C // first digital potentiometers
#define SECOND_AD5254_I2C_ADDR      0x58 // second digital potentiometers
#define DAC5574_I2C_ADDR            0x98 // DAC
#define PCA9530_I2C_ADDR            0xC0//PCA9530
#define MAX77831_I2C_ADDR           (0x66 << 1)
#define GPIO_EXPANDER_IIC_ADDR	    (0x20 << 1) // TCA6408A

#define PCA9530_INPUT_REG_ADDR      0x00//input register
#define PCA9530_PSC0_REG_ADDR       0x01//frequency prescaler 0
#define PCA9530_PWM0_REG_ADDR       0x02//PWM register 0
#define PCA9530_PSC1_REG_ADDR       0x03//frequency prescaler 1
#define PCA9530_PWM1_REG_ADDR       0x04//PWM register 1
#define PCA9530_LS0_REG_ADDR        0x05//LED selector

// vcsel driver IC type
#define DRIVER_IC_CXA4016              4016
#define DRIVER_IC_CXA4046              4046
#define DRIVER_IC_PHX3D_3021_AA        5016
#define DRIVER_IC_PHX3D_3021_CB        5017
#define DRIVER_IC_DW9912               9912 // DongWoon
static uint16_t driver_ic_type = 0;
static uint8_t start_streaming_called = 0;

static ObcSensorInfo get_src_info;

#define DUAL_FREQ      1
#define SINGLE_FREQ    0
#define AF_FREQ        2

#define BINNING_NO      0
#define BINNING_2X2     1
#define BINNING_4X4     2
#define BINNING_AF      3

#define DUAL_FREQ_NO_BINNING_RESOLUTION         RESOLUTION_1920_2880
#define DUAL_FREQ_2X2_BINNING_RESOLUTION        RESOLUTION_960_1440
#define DUAL_FREQ_4X4_BINNING_RESOLUTION        0

#define SINGLE_FREQ_NO_BINNING_RESOLUTION       RESOLUTION_1920_1440
#define SINGLE_FREQ_2X2_BINNING_RESOLUTION      RESOLUTION_960_720
#define SINGLE_FREQ_4X4_BINNING_RESOLUTION      0

#define PIXEL_BIT       10
#define MIPI_PACK_BIT   10

#define FM24C128D_EEPROM_PAGE_SIZE  64
#define FM24C128D_CSP_CONFIG_REG    0x06CA
#define FM24C128D_CC_WREN_REG       0x3F35
#define FM24C128D_DATA_MEMORY       0xA0




struct regList {
    uint16_t reg;
    uint8_t val;
};
#include <tof_sensors.h>
#include <obc_tee_funcs.h>

#ifdef __linux__
#include <unistd.h>
#endif



//#define ALOGE(...) tops.qsee_log(TEE_LOG_LEVEL_ERROR, __VA_ARGS__)
//#define ALOGE(...)
#define TEE_LOG_LEVEL_ERROR        8
#define ALOGE(fmt,...) tops_t.qsee_log(TEE_LOG_LEVEL_ERROR, "[ERROR] [%s(%d)] : " fmt"\n",__FUNCTION__,__LINE__,##__VA_ARGS__)

#define malloc tops_t.qsee_malloc
#define free tops_t.qsee_free
#define usleep tops_t.tee_usleep
#define orbbec_i2c_writeread       tops_t.ops_writeread
//#define orbbec_spi_writeread       tops_t.ap_ops.SensorSpiRWEx
#define dothin_pmu_set_voltage     tops_t.ap_ops.PmuSetVoltage
#define dothin_pmu_set_onoff       tops_t.ap_ops.PmuSetOnOff
#define dothin_enable_softpin      tops_t.ap_ops.EnableSoftPin
#define dothin_enable_gpio         tops_t.ap_ops.EnableGpio
#define dothin_set_sensor_clock    tops_t.ap_ops.SetSensorClock
#define dothin_set_softpin_pullup  tops_t.ap_ops.SetSoftPinPullUp
#define dothin_set_sensor_i2c_rate tops_t.ap_ops.SetSensorI2cRate
#define dothin_sensor_enable       tops_t.ap_ops.SensorEnable

#define dothin_device_id             tops_t.ap_ops.device_id
#define dothin_set_gpio_level        tops_t.ap_ops.SetGpioPinLevel

#define I2C_M_RD     1
#define I2C_M_WT     0
/*
typedef struct i2c_msg {
    uint8_t slave_addr;
    uint8_t rw_mode;
    uint16_t reg;
    uint8_t  reg_size;
    uint32_t* data;
    uint16_t data_size;
}i2c_msg_t;
*/

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

static int max77831_write_reg(uint8_t reg, uint8_t value)
{
	int ret = i2c_reg_write(MAX77831_I2C_ADDR, reg, 1, value, 1);
	return ret;
}

static int max77831_read_reg(uint8_t reg, uint8_t *value)
{
	int ret = i2c_reg_read(MAX77831_I2C_ADDR, reg, 1, value, 1);
	return ret;
}

static int gpio_control(int pin, bool level)
{
    dothin_set_gpio_level(pin, level, dothin_device_id);
    return 0;
}

static int pleco_get_sensor_info(struct sensor_info_t *info) 
{
    memcpy(info, &get_src_info, sizeof(get_src_info));

    return 0;
}

int pleco_dothin_config()
{
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
    ret = dothin_set_sensor_clock(true, 24 * 10, dothin_device_id); // 24Mhz mclk
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
#ifdef X210116
    ret = dothin_sensor_enable(X210116, true, dothin_device_id);//X210116项目：3使用泛光，2使用散斑。
#else
    ret = dothin_sensor_enable(3, true, dothin_device_id);
#endif
    
    if (ret < 0) {
        DDEBUG("dothin_sensor_enable ret=%d\n", ret);
    }
    usleep(1000 * 50);

    SENSOR_POWER sensor_power[] = { POWER_AVDD, POWER_DOVDD,POWER_DVDD ,POWER_AFVCC ,POWER_VPP };
    int           power_value[] = { 2700,       1800,      1200,        1600,        0 };
    ret = dothin_pmu_set_voltage(sensor_power, power_value, 5, dothin_device_id);
    if (ret < 0) {
        DDEBUG("dothin_sensor_enable ret=%d", ret);
    }
    usleep(1000 * 50);

    return ret;
}

int pleco_get_sensor_id(uint16_t *id)
{
    int rtn = 0;
    uint8_t value = 0;
    static int isConfig = 0;

    if (!isConfig) {
        isConfig = 1;
        rtn = gpio_expander_init();// power enable
        rtn = pleco_dothin_config();
		// ST_Lion_Pleco_DEMO_V1.0 board
#ifndef X210116
		gpio_control(2, 1); // for TX power control
		gpio_control(3, 1);
		gpio_control(4, 1);
#endif
    }
    rtn = sensor_read_reg(7, &value); // TODO: use true sensor id
    if (value == 90)
        *id = pleco_sensor_id;
    //DDEBUG("id=%d, ret=%d",*id, rtn);
   
    return rtn;
}

static int sensor_write_reg(uint16_t reg, uint8_t value)
{
    int rtn = i2c_reg_write(PLECO_I2C_ADDR, reg, 2, value, 1);

    return rtn;
}

static int sensor_read_reg(uint16_t reg, uint8_t *value)
{
    int rtn = i2c_reg_read(PLECO_I2C_ADDR, reg, 2, value, 1);
    return rtn;
}

static int set_vrampst_volt(uint8_t value)
{
	int rtn = i2c_reg_write(FIRST_AD5254_I2C_ADDR, 0x01, 1, value, 1);
	uint8_t REG_EE = 0x20; // store to EEMEM
	rtn = i2c_reg_write(FIRST_AD5254_I2C_ADDR, (REG_EE | 0x01), 1, value, 1);
	return rtn;
}

static int set_vmghi_volt(uint8_t value)
{
    uint16_t VMG_HI = (value << 8);
    int rtn = i2c_reg_write(DAC5574_I2C_ADDR, 0x10, 1, VMG_HI, 2);
    //ALOGE("driver set_vmghi_volt:%d\r\n", value);
    return rtn;
}

static int get_vmghi_volt(uint8_t *value)
{
    uint16_t VMG_HI;
    int rtn = i2c_reg_read(DAC5574_I2C_ADDR, 0x10, 1, &VMG_HI, 2);
    *value = (VMG_HI >> 8) & 0xFF;
    //ALOGE("driver get_vmghi_volt:%d\r\n", *value);
    return rtn;
}

//IO1 -> A SPOT, IO2 -> B SPOT
static int set_expander_tx_gpio(uint8_t A_status_enable, uint8_t B_status_enable)
{
	int rtn = 0;
	uint8_t gpio_status = 0;
	if (A_status_enable && B_status_enable)
		return 0;
	if (B_status_enable) {
		rtn = vcsel_driver_write_reg(0x0D, 0x19); // current 0.39A
		gpio_status = 0x3F | (B_status_enable << 7); // set IO2 to high
		rtn = i2c_reg_write(GPIO_EXPANDER_IIC_ADDR, 0x01, 1, gpio_status, 1);
	}
	if (A_status_enable) {
		gpio_status = 0x3F | (A_status_enable << 6); // set IO1 to high
		rtn = i2c_reg_write(GPIO_EXPANDER_IIC_ADDR, 0x01, 1, gpio_status, 1);
		rtn = vcsel_driver_write_reg(0x0D, 0xDF); // current 3.5A
	}
	return rtn;
}

static int gpio_expander_init() // for power control
{
    int rtn = 0;

	rtn = gpio_control(GPIO_EXPANDER_RST_PIN_OB, 1); // pull gpio expander rst pin to high
	rtn = gpio_control(CXA4046_XCLR, 1); // pull cxa4046 rst pin to high
    rtn = i2c_reg_write(TCA6408A_I2C_ADDR, 0x03, 1, 0x00, 1); // set all gpio as output
    //rtn = i2c_reg_write(TCA6408A_I2C_ADDR, 0x01, 1, 111, 1); // set P0 - P3, P5 - P6 gpio to high
#ifdef X210116
    rtn = i2c_reg_write(TCA6408A_I2C_ADDR, 0x01, 1, 0x3f, 1); // pleco + ST_F201103_TOP_V1.0板子
#ifdef X210116_TX_TYPE
    if (X210116_TX_TYPE == 3)
    {
        rtn = max77831_write_reg(0x13, 0x55); // 6.25V 三节tx
    }
    else if (X210116_TX_TYPE == 2)
    {
        rtn = max77831_write_reg(0x13, 0x42); // 4.8V 双节tx
    }
    else
    {
        rtn = max77831_write_reg(0x13, 0x00); // 4.4V 单节tx
    }

#endif

#else
    rtn = i2c_reg_write(TCA6408A_I2C_ADDR, 0x01, 1, 0xEF, 1); // pleco新板子
    rtn = set_vmghi_volt(20); // 60 ->1.2V, 40 ->1.4V, 20 ->1.6V, 0 ->1.8V
#endif
	//rtn = set_vrampst_volt(162); // 162 -> 0.95V
	rtn = set_expander_tx_gpio(1, 0); // default use SPOT_A

	return rtn;
}

static int vcsel_driver_write_reg(uint8_t reg, uint8_t value)
{
    int rtn = 0;
#if 0
    //rtn = sensor_write_reg(354, 0x00); // send1 channel, start by register trig
    rtn = sensor_write_reg(356, 0x00); // send1 start pointer of Tx buffer
    rtn = sensor_write_reg(371, 0x02); // number of byte to be write
    rtn = sensor_write_reg(372, reg);  // address to be write to laser (write address = laser addr)
    rtn = sensor_write_reg(373, value);// value to be write to laser
    //rtn = sensor_write_reg(354, 0x01); // enable
    rtn = sensor_write_reg(354, 0x03); // trig
    rtn = sensor_write_reg(354, 0x00); // disable
#else
    rtn = sensor_write_reg(356, 0x10); // send1 start pointer of Tx buffer
    rtn = sensor_write_reg(387, 0x02); // number of byte to be write
    rtn = sensor_write_reg(388, reg);  // address to be write to laser (write address = laser addr)
    rtn = sensor_write_reg(389, value);// value to be write to laser

    rtn = sensor_write_reg(354, 0x03); // trig
    rtn = sensor_write_reg(354, 0x00); // disable
#endif
    return rtn;
}

static int vcsel_driver_read_reg(uint8_t reg, uint8_t *value)
{
    int rtn = 0;
    rtn = sensor_write_reg(349, 0x00); // general channel, start by register trig
    rtn = sensor_write_reg(351, 0x00); // start pointer of Tx buffer
    rtn = sensor_write_reg(352, 0x00); // start pointer of Rx buffer
    //rtn = sensor_write_reg(353, 0x01); // 1st byte is Tx, after Rx
    rtn = sensor_write_reg(371, 0x02); // number of byte to be write and read
    rtn = sensor_write_reg(372, reg + 0x80); // address to be read from laser (read address = laser addr + 0x80)
    //rtn = sensor_write_reg(349, 0x01); // enable
    rtn = sensor_write_reg(349, 0x03); // trig
    rtn = sensor_write_reg(349, 0x00); // disable

    rtn = sensor_read_reg(419, value); // first byte in Rx buffer

    return rtn;
}

struct regList pleco_reglist_2xbinning_dmfd[] = {
    {  16,   1 }, // group hold pull high

    {  21,  16 },
    {  31, 170 },
    {  32,  42 },
    {  38, 232 },
    {  39,   1 },
    { 702, 182 },
    { 703,	 5 },
    { 704, 119 },
    { 740, 160 },
    { 741,	 5 },
    { 784, 240 },
    { 785,	 0 },

#if (PIXEL_BIT == 10)
    { 735 , 0xB0 },// RAW10, EBD length 2400 byte
    { 736 , 0x04 },
    { 705 , 0x96 },// EBD pixel num = EBD length/8 -1 = 299
    { 706 , 0x04 },
#else
    { 735, 160 },//ebd_len
    { 736,	 5 },
    { 705, 179 },//ebd_pix_num 705，706[3]
    { 706,	 4 },//ebd_mode706[2:0]=EBD开几行
#endif

    {  54,  64 },
    {  55,	 1 },
    { 700,	 0 },
    { 701,	 1 },

    {  16,  00 }, // group hold pull low

};

struct regList pleco_reglist_2xbinning_smfd[] = {
    /**/
    {  16,  01 }, // group hold pull high

    {  21,  80 },
    {  31,  85 },
    {  32,  85 },
    {  38, 232 },
    {  39,   1 },
    { 702, 218 },
    { 703,   2 },
    { 704, 119 },
    { 740, 160 },
    { 741,   5 },
    { 784, 240 },
    { 785,   0 },

#if (PIXEL_BIT == 10)
    { 735 , 0xB0 },// RAW10, EBD length 2400 byte
    { 736 , 0x04 },
    { 705 , 0x96 },// EBD pixel num = EBD length/8 -1 = 299
    { 706 , 0x04 },
#else
    { 735, 160 },
    { 736,   5 },
    { 705, 179 },
    { 706,   4 },
#endif

    {  54,  64 },
    {  55,   1 },
    { 700,   0 },
    { 701,   1 },

    {  16,  00 }, // group hold pull low
};

struct regList pleco_reglist_nobinning_dmfd[] = {

    {  16,   1 }, // group hold pull high

    {  21,   0 },
    {  31, 170 },
    {  32,  42 },
    {  38, 228 },
    {  39,   1 },
    { 702,  86 },
    { 703,  11 },
    { 704, 239 },
    { 740, 160 },
    { 741,   5 },
    { 784, 224 },
    { 785,   1 },

#if (PIXEL_BIT == 10)
    { 735 , 96 },// RAW10, EBD length 2400 byte
    { 736 , 9 },
    { 705 , 43 },// EBD pixel num = EBD length/8 -1 = 299
    { 706 , 12 },
#else
    { 735,  64 },
    { 736,  11 },
    { 705, 104 },
    { 706,  12 },
#endif

    {  54, 128 },
    {  55,   2 },
    { 700,   0 },
    { 701,   1 },

    {  16,  00 }, // group hold pull low
};

struct regList pleco_reglist_nobinning_smfd[] = {
    /**/
    {  16,   1 }, // group hold pull high

    {  21,  64 },
    {  31,  85 },
    {  32,  85 },
    {  38, 228 },
    {  39,   1 },
    { 702, 170 },
    { 703,   5 },
    { 704, 239 },
    { 740, 160 },
    { 741,   5 },
    { 784, 224 },
    { 785,   1 },

#if (PIXEL_BIT == 10)
    { 735 , 96 },// RAW10, EBD length 2400 byte
    { 736 , 9 },
    { 705 , 43 },// EBD pixel num = EBD length/8 -1 = 299
    { 706 , 12 },
#else
    { 735,  64 },
    { 736,  11 },
    { 705, 104 },
    { 706,  12 },
#endif
    {  54, 128 },
    {  55,   2 },
    { 700,   0 },
    { 701,   1 },

    {  16, 00 }, // group hold pull low

};

struct regList pleco_reglist_af[] = {
    {  16,    1 },

    {  21,	192 },
    {  31,	170 },
    {  32,	 42 },
    {  38,	228 },
    {  39,	  1 },
    { 702,	 86 },
    { 703,	 11 },
    { 704,	239 },
    { 740,	160 },
    { 741,	  5 },
    { 784,	224 },
    { 785,	  1 },

#if (PIXEL_BIT == 10)
    { 735 , 96 },// RAW10, EBD length 2400 byte
    { 736 , 9 },
    { 705 , 43 },// EBD pixel num = EBD length/8 -1 = 299
    { 706 , 12 },
#else
    { 735,	 64 },
    { 736,	 11 },
    { 705,	104 },
    { 706,	 12 },
#endif

    {  54,	128 },
    {  55,	  2 },
    { 700,	  1 },
    { 701,	  1 },

    {  16,    0 },
};

struct regList pleco_reglist[] = {
    { 16, 0x01 }, // group hold pull high
    { 580 ,179 },// 179 -> DB test
    { 38 , 228 },// 484
    { 39 , 1 },
    { 740 , 160 }, // 1440
    { 741 , 5 },
    { 784 , 224 }, // 480
    { 785 , 1 },
    { 704 , 239 },
    { 702 , 226 },// 482
    { 703 , 1 },
#if (PIXEL_BIT == 10)
    { 735 , 96 },// RAW10, EBD length 2400 byte
    { 736 , 9 },
    { 705 , 43 },// EBD pixel num = EBD length/8 -1 = 299
    { 706 , 12 },
#else
    // EBD length 2880 byte
    { 735 , 64 },
    { 736 , 11 },
    { 705 , 0x68 },
    { 706 , 12 },
#endif
    { 67 , 0x44 },
    { 68 , 0x44 },

    { 74 ,100 },// ADC error
    { 75 ,17 },

    { 35 ,10 },// PGA gain
 
    { 593 ,169 },// analog test ramp_st,default 9
    //{ 594 ,0xE1 },// analog test ramp_st,

    { 585 ,43 },// vramp_st adjust value  // 0.94V
    { 586 ,41 },// vramp_st adjust value

    { 578 ,0xB2 }, // DB offset value
    { 579 ,0x6B }, // DB offset value
    { 224 ,0 },// test bias continue

    { 580 ,177 },// DB test pattern, default+2

    { 702 ,170 },// 1450, rd_line_max, 3 subframe
    { 703 , 5 },

	//{ 18 ,118 },// horizontal & vertical flip
	{ 18 ,86 },// vertical flip

    // DMFD
    { 784 ,224},// 480
    { 785 ,1 },
    { 702 ,86 },// 2902, rd_line_max, 6 subframe
    { 703 , 11 },
    { 704 , 239 },
    { 740 , 160 },//1440
    { 741 , 5 },
    { 31, 170 },// frame number 10922 in DMFD
    { 32, 42 },

#if (PIXEL_BIT == 10)
    { 62 ,180 },//// 200 -> 1.6Gbps, 180 -> 1.44Gbps
    { 66 ,0x2B },// data type RAW 10
    { 576, 0xAC },// ADC 10bit

    { 585, 43 }, // VRAMP_st=1.08V，Vref=1.26V
    { 586, 91 },
    { 587, 89 },
    // raw10 analog config
    { 592, 175 },
    { 168, 12 },
    { 170, 32 },
    { 172, 45 },
    { 174, 115 },
    { 175, 0 },
    { 180, 30 },
    { 182, 34 },
    { 184, 113 },
    { 185, 0 },
    { 186, 117 },
    { 187, 0 },
    { 192, 41 },
    { 188, 40 },
    { 190, 42 },
    { 196, 118 },
    { 197, 0 },
    { 198, 119 },
    { 199, 0 },
    { 176, 1 },
    { 178, 3 },
    { 144, 10 },
    { 146, 32 },
    { 148, 42 },
    { 150, 115 },
    { 151, 0 },
    { 160, 14 },
    { 162, 33 },
    { 164, 48 },
    { 166, 116 },
    { 167, 0 },
    { 152, 6 },
    { 154, 8 },
    { 156, 38 },
    { 158, 40 },
    { 578, 106 },
    { 579, 249 },
#endif

    { 632 ,13 },//VSG_M bg value adjust  0V
    { 633 ,0 }, // VTG_M bg value adjust
    { 634 ,160 },//VDRN_M bg value adjust
 
    { 602 ,19 },//VDRN LOW 0V   147 0V    179  -0.2V   211 -0.4V    243  -0.6V
    { 603 ,166 },//VDRN LOW

    //{583 ,130},// PGA current
    { 609 ,18 },// SG LO1   0V   best

    //{611 ,213},//SG L2
    { 609 ,26 },//SG L1
    { 608, 244 },// dark current utilize
    { 569, 1 },// disable antijamm

	{742, 184},// af_vld_line
	{743, 2},

	// power consumption utilize
	{ 0x154, 0x01},//340    0:50M 1:100M 2:200M 3:25M
	{ 0x257, 0x43},//599
	{ 0x258, 0x50},//600
//	{ 0x263, 0x5A},
//	{ 0x264, 0x0C},

    { 227,  200},

    { 539,  3 },

    { 16, 0x00 }, // group hold pull low
};

// PHX3D 3025 almost the same with PHX3D 3021 except XCLR mask control function
// (PHX3D 3025, register 0x2E bit 0 should set to 1, while PHX3D 3021 don't care this)
static struct regList phx3d_3021_cb_reglist[] = {

    // PHX3D Laser Driver reset
    { 356,  0x04 },
    { 375,  0x02 },
    { 376,  0x25 },
    { 377,  0x01 },
    { 354,  0x03 },
    { 354,  0x00 },

    // PHX3D Laser Driver initial
    { 356,  0x07 },
    { 378,  0x15 },
    { 379,  0x00 },
    { 380,  0x0C }, // 0x00
    { 381,  0x20 }, // 0x01
    { 382,  0x30 }, // 0x02
    { 383,  0x10 }, // 0x03
    { 384,  0x20 }, // 0x04
    { 385,  0x69 }, // 0x05
    { 386,  0xC3 }, // 0x06
    { 387,  0x00 }, // 0x07 IBIAS_FIX
    { 388,  0xC0 }, // 0x08 ISW_FIX
    { 389,  0x24 }, // 0x09
    { 390,  0x0D }, // 0x0A
    { 391,  0xFF }, // 0x0B
    { 392,  0xD9 }, // 0x0C
    { 393,  0xC2 }, // 0x0D
    { 394,  0x78 }, // 0x0E ISW_FIX threshold
    { 395,  0x01 }, // 0x0F
    { 396,  0xDF }, // 0x10
    { 397,  0x00 }, // 0x11
    { 398,  0x00 }, // 0x12
    { 399,  0x01 }, // 0x13
    { 354,  0x03 }, // send1 channel register trig
    { 354,  0x00 },

    { 356,  0x1D },
    { 400,  0x02 },
    { 401,  0x26 },
    { 402,  0x19 }, // 0x26  internal PD resistance
    { 403,  0x31 }, // 0x27
    { 404,  0xFF }, // 0x28
    { 354,  0x03 },
    { 354,  0x00 },

    { 356,  0x22 },
    { 405,  0x09 },
    { 406,  0x2B },
    { 407,  0xF1 }, // 0x2B
    { 408,  0xF8 }, // 0x2C
    { 409,  0x03 }, // 0x2D
    { 410,  0x13 }, // 0x2E
    { 411,  0x00 }, // 0x2F
    { 412,  0x00 }, // 0x30   duty increase
    { 413,  0x00 }, // 0x31   duty decrease
    { 414,  0xFF }, // 0x32   falling edge speed control
    { 354,  0x03 },
    { 354,  0x00 },

    // Laser Driver ADC auto start
    { 357,  0x05 }, // send2 channel start by internal sync signal
    { 359,  0x20 }, // send2 start pointer of Tx buffer
    { 403,  0x02 }, // send2 total write byte
    { 404,  0x13 }, // laser address 0x13
    { 405,  0x21 }, // value write to laser address 0x13
    { 365,  0x01 }, // internal sync enable for spi master write, start of phase
    { 366,  0x01 }, // internal sync timing for spi master write

    // Laser Driver temperature auto read
    { 344,  0x05 }, // thermo channel start by internal sync signal
    { 346,  0x23 }, // thermo start pointer of Tx buffer
    { 347,  0x0B }, // thermo start pointer of Rx buffer
    { 406,  0x13 }, // number of byte to be write and read, 1 tx byte, 17 rx byte
    { 407,  0x94 }, // temperature(TEMP_AD_DATA[9:8][7:0]) address to be read from laser (read address = laser addr 0x14 + 0x80)
    { 360,  0x01 }, // internal sync enable for spi master read, start of phase
    { 361,  0xFF }, // internal sync timing for spi master read
    { 362,  0x20 }, // internal sync timing for spi master read

};

struct regList cxa4016_reglist[] = {
    /*
    // Laser Driver (CXA4016)  ADC initialization
    { 356,  0x00 },
    { 371,  0x02 },
    { 372,  0x13 },
    { 373,  0x23 },
    { 354,  0x03 },
    { 354,  0x00 },*/

    // Laser Driver (CXA4016) reset
    { 356,  0x04 },
    { 375,  0x02 },
    { 376,  0x25 },
    { 377,  0x01 },
    { 354,  0x03 },
    { 354,  0x00 },
    // Sony Laser Driver (CXA4016) initial
    { 356,  0x07 },
    { 378,  0x15 },
    { 379,  0x00 },
    { 380,  0x0C }, // 0x00
    { 381,  0xA0 }, // 0x01
    { 382,  0xFF }, // 0x02
    { 383,  0x1A }, // 0x03
    { 384,  0x28 }, // 0x04
    { 385,  0x69 }, // 0x05
    { 386,  0x87 }, // 0x06
    { 387,  0x00 }, // 0x07 IBIAS_FIX
    { 388,  0xD3 }, // 0x08 ISW_FIX
    { 389,  0x14 }, // 0x09
    { 390,  0x05 }, // 0x0A
    { 391,  0xA4 }, // 0x0B
    { 392,  0x28 }, // 0x0C
    { 393,  0xC3 }, // 0x0D
    { 394,  0xC7 }, // 0x0E ISW_FIX threshold
    { 395,  0x72 }, // 0x0F
    { 396,  0x0F }, // 0x10
    { 397,  0x00 }, // 0x11
    { 398,  0x00 }, // 0x12
    { 399,  0x01 }, // 0x13
    { 354,  0x03 }, // send1 channel register trig
    { 354,  0x00 },
    // Laser Driver (CXA4016) ADC auto start
    {357,  0x05}, // send2 channel start by internal sync signal
    {359,  0x20}, // send2 start pointer of Tx buffer
    {403,  0x02}, // send2 total write byte
    {404,  0x13}, // laser address 0x13
    {405,  0x23}, // value write to laser address 0x13
    {365,  0x01}, // internal sync enable for spi master write, start of phase
    {366,  0x01}, // internal sync timing for spi master write

    // Laser Driver (CXA4016) temperature auto read
    {344,  0x05}, // thermo channel start by internal sync signal
    {346,  0x23}, // thermo start pointer of Tx buffer
    {347,  0x10}, // thermo start pointer of Rx buffer
    {406,  0x03}, // number of byte to be write and read
    {407,  0x94}, // temperature(TEMP_AD_DATA[9:8][7:0]) address to be read from laser (read address = laser addr 0x14 + 0x80)
    {360,  0x01}, // internal sync enable for spi master read, start of phase
    {361,  0xFF}, // internal sync timing for spi master read
    {362,  0x20}, // internal sync timing for spi master read
};

struct regList cxa4046_reglist[] = {
	// CXA4046 Laser Driver reset
	{ 356,  0x04 }, //0x04 -> buffer's 4 position
	{ 375,  0x02 },
	{ 376,  0x3C },
	{ 377,  0x01 },
	{ 354,  0x03 },
	{ 354,  0x00 },

	// PHX3D Laser Driver initial
	{ 356,  0x07 }, //0x07 -> buffer's 7 position
	{ 378,  0x20 },
	{ 379,  0x00 },
	{ 380,  0x00 }, // 0x00
	{ 381,  0x01 }, // 0x01
	{ 382,  0x19 }, // 0x02
	{ 383,  0x00 }, // 0x03
	{ 384,  0x40 }, // 0x04
	{ 385,  0x25 }, // 0x05
	{ 386,  0x1E }, // 0x06
	{ 387,  0x81 }, // 0x07 IBIAS_FIX
	{ 388,  0x42 }, // 0x08 ISW_FIX
#ifdef X210116
#if (X210116_TX_TYPE == 1)
    { 389,  0x00 }, // 0x09
#elif (X210116_TX_TYPE == 2)
    { 389,  0x0f }, // 0x09
#else
    { 389,  0x88 }, // 0x09
#endif
#else
	{ 389,  0x8f }, // 0x09
#endif
	{ 390,  0x00 }, // 0x0A
	{ 391,  0x00 }, // 0x0B
	{ 392,  0x5F }, // 0x0C
	{ 393,  0xDF }, // 0x0D
	{ 394,  0x00 }, // 0x0E ISW_FIX threshold
	{ 395,  0x00 }, // 0x0F
	{ 396,  0x00 }, // 0x10
	{ 397,  0x00 }, // 0x11
	{ 398,  0x00 }, // 0x12
	{ 399,  0x00 }, // 0x13
	{ 400,  0x00 }, // 0x14
	{ 401,  0x00 }, // 0x15
	{ 402,  0x00 }, // 0x16
	{ 403,  0x00 }, // 0x17
	{ 404,  0x00 }, // 0x18
	{ 405,  0xC0 }, // 0x19
	{ 406,  0xD0 }, // 0x1A
	{ 407,  0x78 }, // 0x1B
	{ 408,  0x21 }, // 0x1C
	{ 409,  0x3F }, // 0x1D
#ifdef X210116
#if (X210116_TX_TYPE == 1)
    { 410,  0x0B }, // 0x1E
#elif (X210116_TX_TYPE == 2)
    { 410,  0x06 }, // 0x1E
#else
    { 410,  0x05 }, // 0x1E
#endif
#else
    { 410,  0x00 }, // 0x1E
#endif
	{ 354,  0x03 }, // send1 channel register trig
	{ 354,  0x00 },

	// Laser Driver ADC auto start
	/*{ 357,  0x05 }, // send2 channel start by internal sync signal
	{ 359,  0x20 }, // send2 start pointer of Tx buffer
	{ 403,  0x02 }, // send2 total write byte
	{ 404,  0x07 }, // laser address 0x07
	{ 405,  0x81 }, // value write to laser address 0x07
	{ 365,  0x01 }, // internal sync enable for spi master write, start of phase
	{ 366,  0x01 }, // internal sync timing for spi master write*/

	// Laser Driver temperature auto read
    
	{ 344,  0x05 }, // thermo channel start by internal sync signal
	{ 346,  0x23 }, // thermo start pointer of Tx buffer
	{ 347,  0x0B }, // thermo start pointer of Rx buffer
	{ 406,  0x22 }, // number of byte to be write and read, 1 tx byte, 17 rx byte
	{ 407,  0xA2 }, // temperature(TEMP_AD_DATA[9:8][7:0]) address to be read from laser (read address = laser addr 0x22 + 0x80)
	{ 360,  0x01 }, // internal sync enable for spi master read, start of phase
	{ 361,  0xFF }, // internal sync timing for spi master read
	{ 362,  0x20 }, // internal sync timing for spi master read
    
};

static int cxa4016_cb_initialize()
{
    int rtn = 0;

    for (int i = 0; i < sizeof(cxa4016_reglist) / sizeof(struct regList); i++) {

        rtn = sensor_write_reg(cxa4016_reglist[i].reg, cxa4016_reglist[i].val);
    }
    return rtn;
}

static int cxa4046_cb_initialize()
{
	int rtn = 0;

	for (int i = 0; i < sizeof(cxa4046_reglist) / sizeof(struct regList); i++) {

		rtn = sensor_write_reg(cxa4046_reglist[i].reg, cxa4046_reglist[i].val);
	}
	return rtn;
}

static int phx3d_3021_cb_initialize()
{
    int rtn = 0;

    for (int i = 0; i < sizeof(phx3d_3021_cb_reglist) / sizeof(struct regList); i++) {

        rtn = sensor_write_reg(phx3d_3021_cb_reglist[i].reg, phx3d_3021_cb_reglist[i].val);
    }
    return rtn;
}

static int vcsel_driver_detect()
{
    int rtn = 0;
    uint32_t value1 = 0, value2 = 0;
    rtn = vcsel_driver_write_reg(0x25, 0x01); // reset
	rtn = vcsel_driver_write_reg(0x3C, 0x01); // reset 4046
    rtn = vcsel_driver_read_reg(0x0F, &value1);
	rtn = vcsel_driver_read_reg(0x09, &value2);

    if (value1 == 0x06) {
        uint8_t phx3d_value = 0;
        rtn = vcsel_driver_read_reg(0x2F, &phx3d_value);
        if (phx3d_value == 0x01) {
            driver_ic_type = DRIVER_IC_PHX3D_3021_CB;
            get_src_info.vcsel_driver_id = DRIVER_IC_PHX3D_3021_CB;
            phx3d_3021_cb_initialize();
        }
    }
    else if (value1 == 0x46)
    {
        driver_ic_type = DRIVER_IC_CXA4016;
        get_src_info.vcsel_driver_id = DRIVER_IC_CXA4016;
        cxa4016_cb_initialize();
    }
	else if (value1 == 0x00 && value2 == 0x88)
	{
		driver_ic_type = DRIVER_IC_CXA4046;
		get_src_info.vcsel_driver_id = DRIVER_IC_CXA4046;
		cxa4046_cb_initialize();
	}
    else {
        driver_ic_type = 0;
        get_src_info.vcsel_driver_id = 0;
    }

	ALOGE("vcsel_driver_detect, value1:0x%x, value2:0x%x, driver_ic_type: %d ", value1, value2, driver_ic_type);

    return rtn;
}

/**
* @brief  pleco_sensor_init 瀵勫瓨鍣ㄥ垵濮嬪寲
* @return int
*/
static int pleco_sensor_init()
{
    int rtn = 0;
    /**/
    for(int i = 0; i < sizeof(pleco_reglist)/sizeof(struct regList); i++){
        rtn |= sensor_write_reg(pleco_reglist[i].reg, pleco_reglist[i].val);
        if (rtn < 0)
            return rtn;
    }
    get_src_info.embedded_data_size = 1920 * 1.5 * 3;
    get_src_info.project_id = 0;
    get_src_info.sensor_id = pleco_sensor_id;
    get_src_info.vcsel_num = 1;

    rtn = vcsel_driver_detect();

    return rtn;
}

/**
* @brief  pleco_group_hold setup grouped parameter hold
* @return int
*/
int pleco_group_hold(bool enable)
{
    int rtn = 0;
    if(enable)
        rtn = sensor_write_reg(16, 0x01);
    else
        rtn = sensor_write_reg(16, 0x00);

    return rtn;
}

int pleco_hardware_trigger()
{
    return 0;
}

int pleco_software_trigger()
{
    int rtn = 0;
    rtn = sensor_write_reg(1, 0x01);
    rtn = sensor_write_reg(1, 0x00);
    usleep(100);

    return rtn;
}

/**
* @brief  pleco_video_streaming 寮€鍏虫暟鎹祦
* @param  [in] bool enable
* @return int
*/
int pleco_video_streaming(bool enable)
{
    int rtn = 0;
    if (enable) {
        rtn = sensor_write_reg(1, 0x01); // trigger
        rtn = sensor_write_reg(1, 0x00);
        start_streaming_called = 1;
    }
    else {
        rtn = sensor_write_reg(17, 252); // reset sequencer
        rtn = sensor_write_reg(17, 253);
        start_streaming_called = 0;
    }

    return rtn;
}

int pca9530_get_led0_freq_pwm(float *fps)
{
    int rtn;
    uint32_t psc0, pwm0, ls0;
    float duty;
    rtn = i2c_reg_read(PCA9530_I2C_ADDR, PCA9530_PSC0_REG_ADDR, 1, &psc0, 1);
    if (rtn < 0) {
        //printf("pca9530_read: PCA9530_PSC0_REG_ADDR: ret=%d\r\n", rtn);
        return rtn;
    }

    rtn = i2c_reg_read(PCA9530_I2C_ADDR, PCA9530_PWM0_REG_ADDR, 1, &pwm0, 1);
    if (rtn < 0) {
        //printf("pca9530_read: PCA9530_PWM0_REG_ADDR: ret=%d\r\n", rtn);
        return rtn;
    }

    rtn = i2c_reg_read(PCA9530_I2C_ADDR, PCA9530_LS0_REG_ADDR, 1, &ls0, 1);
    if (rtn < 0) {
        //printf("pca9530_read: PCA9530_LS0_REG_ADDR: ret=%d\r\n", rtn);
        return rtn;
    }

    *fps = 150 / ((float)psc0 + 1);
    duty = (float)pwm0 / 256;

    return rtn;
}

int pca9530_set_led0_freq_pwm(float fps, float duty)
{
    int rtn;
    uint32_t psc0, pwm0, LS0;
    //fps = fps / 1.1;
    psc0 = 152 / fps - 1;
    /*
    if (fps >= 20)
        psc0 = 152 / (fps);
    else
        psc0 = 152 / (fps)-1;
    */
    pwm0 = (1 - duty) * 256;
    LS0 = 0x02;// LED0 selected

    rtn = i2c_reg_write(PCA9530_I2C_ADDR, PCA9530_PSC0_REG_ADDR, 1, &psc0, 1);
    if (rtn < 0) {
        //printf("pca9530_write: PCA9530_PSC0_REG_ADDR: ret=%d\r\n", rtn);
        return rtn;
    }

    rtn = i2c_reg_write(PCA9530_I2C_ADDR, PCA9530_PWM0_REG_ADDR, 1, &pwm0, 1);
    if (rtn < 0) {
        //printf("pca9530_write: PCA9530_PWM0_REG_ADDR: ret=%d\r\n", rtn);
        return rtn;
    }

    rtn = i2c_reg_write(PCA9530_I2C_ADDR, PCA9530_PSC1_REG_ADDR, 1, &psc0, 1);
    if (rtn < 0) {
        //printf("pca9530_write: PCA9530_PSC1_REG_ADDR: ret=%d\r\n", rtn);
        return rtn;
    }

    rtn = i2c_reg_write(PCA9530_I2C_ADDR, PCA9530_PWM1_REG_ADDR, 1, &pwm0, 1);
    if (rtn < 0) {
        //printf("pca9530_write: PCA9530_PWM1_REG_ADDR: ret=%d\r\n", rtn);
        return rtn;
    }

    rtn = i2c_reg_write(PCA9530_I2C_ADDR, PCA9530_LS0_REG_ADDR, 1, &LS0, 1);
    if (rtn < 0) {
        //printf("pca9530_write: PCA9530_LS0_REG_ADDR: ret=%d\r\n", rtn);
        return rtn;
    }

    return rtn;
}

/**
* @brief  pleco_get_fps 甯х巼璁剧疆
* @param  [in] uint8_t fps
* @return int
*/
int pleco_get_fps(uint8_t *fps)
{
    int rtn = 0;
    uint8_t pixel_clock = 10; // uint: ns, 100MHz
    uint8_t valueL, valueH, valueHH;
    rtn = sensor_read_reg(26, &valueL);
    rtn = sensor_read_reg(27, &valueH);
    uint16_t line_length = (valueH << 8) + valueL;
    rtn = sensor_read_reg(28, &valueL);
    rtn = sensor_read_reg(29, &valueH);
	rtn = sensor_read_reg(30, &valueHH);
    uint32_t subframe_length = (valueHH << 16) + (valueH << 8) + valueL;
	uint8_t freq_mode, subframe_num;
	pleco_get_frequency_mode(&freq_mode);
	if (DUAL_FREQ == freq_mode || AF_FREQ == freq_mode)
		subframe_num = 6;
	else if (SINGLE_FREQ == freq_mode)
		subframe_num = 3;
	unsigned int frame_length = subframe_length * subframe_num; // 6 subframe in dual freq mode
    *fps = 1000000000 / (frame_length*line_length*pixel_clock);

    //pca9530_get_led0_freq_pwm(fps);

    return 0;
}

/**
* @brief  pleco_set_fps 鑾峰彇褰撳墠甯х巼
* @param  [out] uint8_t* fps
* @return int
*/
int pleco_set_fps(uint8_t fps)
{
    int rtn = 0;
	uint8_t freq_mode, subframe_num;
	rtn = pleco_get_frequency_mode(&freq_mode);
	if (DUAL_FREQ == freq_mode || AF_FREQ == freq_mode) {
		subframe_num = 6;
		if (fps == 0 || fps > 30)
			return -HW_ERR_INVALID;
	}
	else if (SINGLE_FREQ == freq_mode) {
		subframe_num = 3;
		if (fps == 0 || fps > 60)
			return -HW_ERR_INVALID;
	}
    
    // frame_time = line_length*frame_length*pixel_clock
    uint32_t frame_time = 1000000000 / fps; // uint: ns
    uint8_t pixel_clock = 10; // uint: ns, 100MHz
    uint8_t valueL, valueH;
    rtn = sensor_read_reg(26, &valueL);
    rtn = sensor_read_reg(27, &valueH);
    uint16_t line_length = (valueH << 8) + valueL;
    uint32_t frame_length = frame_time / (line_length * pixel_clock);
    uint32_t subframe_length = frame_length / subframe_num; // 6 subframe in dual freq mode
    //printf("subframe_length: %d", subframe_length);
    if (subframe_length < 2100)
        return -HW_ERR_INVALID;
	unsigned char valueHH = subframe_length / 65535;
	uint16_t valueHL = subframe_length % 65535;
    valueL = valueHL % 256;
    valueH = valueHL / 256;
    rtn = pleco_group_hold(true);
    rtn = sensor_write_reg(28, valueL);
    rtn = sensor_write_reg(29, valueH);
	rtn = sensor_write_reg(30, valueHH);
    rtn = pleco_group_hold(false);

    pca9530_set_led0_freq_pwm(fps, 0.1);

    return 0;
}

int pleco_get_rx_temp(float *temperature)
{
    int rtn = 0;
    uint8_t valueH, valueL;
    rtn = sensor_read_reg(451, &valueL);
    rtn = sensor_read_reg(452, &valueH);

    uint16_t value = (valueH << 8) | valueL;
    *temperature = value*0.16f - 272;
    //printf("IIC sensor_temp %d %d %d \r\n", valueL, valueH, *temperature);
    return rtn;
}

int pleco_get_tx_temp(float *temperature)
{
    int rtn = 0;
    uint8_t valueH, valueL;
    rtn = sensor_read_reg(419 + 0x10, &valueH); // 0x10's byte in Rx buffer
    rtn = sensor_read_reg(420 + 0x10, &valueL);

    uint32_t value = ((valueH & 0x03) << 8) + valueL;
    *temperature = 25 + ((value - 296)) / 5.4f;
    //printf("value %d,  valueH %d, valueL %d    tx=%f  \n", value, valueH, valueL, *temperature);
    return rtn;
}

int pleco_get_vcsel_pd(uint8_t *valueH, uint8_t *valueL) // driver ic pd value
{
    int rtn = 0;
    //uint32_t valueH, valueL;
    rtn = vcsel_driver_read_reg(0x1B, valueH); // APC2_CHECK_DATA[9:8]
    rtn = vcsel_driver_read_reg(0x22, valueL); // APC2_CHECK_DATA[7:0]
    //rtn = sensor_read_reg(0x0587, valueH); // 3 byte in Rx buffer
    //rtn = sensor_read_reg(0x058E, valueL); // 10 byte in Rx buffer
    //value = valueL + ((valueH >> 4)& 0x03)*256;

    //printf("value %d,  valueH %d, valueL %d  \n", value, valueH, valueL);
    return rtn;
}

int pleco_set_illum_power(uint8_t vcsel_num, uint8_t value_A, uint8_t value_B)
{
    int rtn = 0;

	if (driver_ic_type == DRIVER_IC_CXA4046)
	{
		rtn = vcsel_driver_write_reg(0x0C, value_B);// IBIAS
		rtn = vcsel_driver_write_reg(0x0D, value_A);// ISW
	}
	else
	{
		rtn = vcsel_driver_write_reg(0x07, value_B);// IBIAS
		rtn = vcsel_driver_write_reg(0x08, value_A);// ISW
	}

    return rtn;
}

int pleco_get_illum_power(uint8_t *vcsel_num, uint8_t *value_A, uint8_t *value_B)
{
    int rtn = 0;
    *vcsel_num = 1;

	if (driver_ic_type == DRIVER_IC_CXA4046)
	{
		rtn = vcsel_driver_read_reg(0x0C, value_B);// IBIAS
		rtn = vcsel_driver_read_reg(0x0D, value_A);// ISW
	}
	else
	{
		rtn = vcsel_driver_read_reg(0x07, value_B);
		rtn = vcsel_driver_read_reg(0x08, value_A);
	}

    return rtn;
}

int pleco_illum_power_control(bool enable)
{
    return 0;
}

/**
* @brief  pleco_get_anti_interference
* @param  [out] uint8_t status
* @return int
*/
int pleco_get_antijam(uint8_t *status)
{
    int rtn = 0;
    uint8_t value;
    rtn = sensor_read_reg(569, &value);
    *status = value & 0x02;

    return 0;
}

/**
* @brief  pleco_set_anti_interference
* @param  [in] uint8_t status
* @return int
*/
int pleco_set_antijam(uint8_t status)
{
    int rtn = 0;
    uint8_t value;
    rtn = pleco_group_hold(true);
    rtn = sensor_read_reg(569, &value);
    if (status)
        value |= (1 << 1);
    else
        value &= ~(1 << 1);
    rtn = sensor_write_reg(569, value);
    rtn = pleco_group_hold(false);
    return 0;
}

/**
* @brief  pleco_get_integration_time 鑾峰彇褰撳墠绉垎鏃堕棿
* @param  [out] uint16_t *integrationTime (us)
* @return int
*/
int pleco_get_integration_time(uint16_t *integrationTime)
{
    int rtn = 0;
    uint8_t valueL, valueH;
    rtn = sensor_read_reg(526, &valueL); // F0
    rtn = sensor_read_reg(527, &valueH);
    *integrationTime = (valueH << 8) + valueL;

	ALOGE("pleco_get_integration_time: %d \n", *integrationTime);

    return 0;
}

/**
* @brief  pleco_set_integration_time 绉垎鏃堕棿璁剧疆
* @param  [in] uint16_t integrationTime (us)
* @return int
*/
int pleco_set_integration_time(uint16_t integrationTime)
{
    int rtn = 0;
    if (integrationTime > 4095)
        return -HW_ERR_INVALID;
	unsigned char valueL, valueH;
    valueL = (uint8_t)(integrationTime % 256);
    valueH = (uint8_t)(integrationTime / 256);
    rtn = pleco_group_hold(true);
    rtn = sensor_write_reg(526, valueL); // F0
    rtn = sensor_write_reg(527, valueH);

    rtn = sensor_write_reg(528, valueL); // F1
    rtn = sensor_write_reg(529, valueH);
    rtn = pleco_group_hold(false);

	ALOGE("pleco_set_integration_time: %d \n", integrationTime);

    return 0;
}

int pleco_get_modulation_frequency(uint16_t *modFreq)
{
    int rtn = 0;
    uint8_t freq_0, freq_1, pll2_div_b;
    rtn = sensor_read_reg(567, &freq_0);
    rtn = sensor_read_reg(568, &freq_1);
    rtn = sensor_read_reg(560, &pll2_div_b);//PLL DIV
    if (pll2_div_b == 1) {
        *modFreq = ((freq_0 + 3) << 8) + (freq_1 + 3);
    } else if (pll2_div_b == 2) {
        *modFreq = (((freq_0 + 3) * 2) << 8) + (freq_1 + 3) * 2;
    }
	else if (pll2_div_b == 3)
	{
		*modFreq = ((freq_0 * 4 + 12) << 8) + (freq_1 * 4 + 12);
	}

    ALOGE("yzx pleco_get_modulation_frequency: %d, %d, 0x %x\r\n", freq_0, freq_1, *modFreq);

    return rtn;
}

int pleco_set_modulation_frequency(uint16_t modFreq)
{
    int rtn = 0;
    uint8_t freq_0, freq_1;
    freq_0 = (modFreq >> 8) & 0xFF;
    freq_1 = modFreq & 0xFF;

	ALOGE("pleco_set_modulation_frequency start:  %d  %d  ,modFreq: %x", freq_0, freq_1, modFreq);

    if (freq_0 >= 3 && freq_0 <= 10) {// && freq_1 >= 3 && freq_1 <= 10
        static uint16_t npulse[] = { 348, 264, 216, 180, 159, 141, 126 ,114 };// should be divided by 3
        uint8_t valueH, valueL;
        rtn = pleco_group_hold(true);

        rtn = sensor_write_reg(560, 1);//PLL DIV

        //uint16_t f0_npulse = (1000 / freq_0) + 15;// turn out to be 1us
        uint16_t f0_npulse = npulse[freq_0 - 3];
        valueL = f0_npulse & 0xFF;
        valueH = (f0_npulse >> 8) & 0xFF;
        rtn = sensor_write_reg(537, valueL);
        rtn = sensor_write_reg(538, valueH);

        //uint16_t f1_npulse = (1000 / freq_1) + 15;// turn out to be 1us
        uint16_t f1_npulse = npulse[freq_1 - 3];
        valueL = f1_npulse & 0xFF;
        valueH = (f1_npulse >> 8) & 0xFF;
        rtn = sensor_write_reg(541, valueL);
        rtn = sensor_write_reg(542, valueH);

        rtn = sensor_write_reg(567, (freq_0 - 3));
        rtn = sensor_write_reg(568, (freq_1 - 3));
        rtn = pleco_group_hold(false);
        //printf("f0_npulse: %d, f1_npulse: %d\r\n", f0_npulse, f1_npulse);
    }
    else if (freq_0 > 10 && freq_0 <= 20) {// || freq_1 > 10 && freq_1 <= 20
        static uint16_t npulse_div[] = { 114, 97, 85, 76, 70 ,64 };// should be divided by 3
        uint8_t valueH, valueL;
        rtn = pleco_group_hold(true);

        rtn = sensor_write_reg(560, 2);//PLL DIV

        if (freq_0 > 10 && freq_0 <= 20){
            //uint16_t f0_npulse = (1000 / freq_0) + 15;// turn out to be 1us
            uint16_t f0_npulse = npulse_div[freq_0/2 - 5];
            valueL = f0_npulse & 0xFF;
            valueH = (f0_npulse >> 8) & 0xFF;
            rtn = sensor_write_reg(537, valueL);
            rtn = sensor_write_reg(538, valueH);
            rtn = sensor_write_reg(567, (freq_0/2 - 3));
        }

        if (freq_1 > 10 && freq_1 <= 20) {
            //uint16_t f1_npulse = (1000 / freq_1) + 15;// turn out to be 1us
            uint16_t f1_npulse = npulse_div[freq_1/2 - 5];
            valueL = f1_npulse & 0xFF;
            valueH = (f1_npulse >> 8) & 0xFF;
            rtn = sensor_write_reg(541, valueL);
            rtn = sensor_write_reg(542, valueH);
            rtn = sensor_write_reg(568, (freq_1/2 - 3));
        }
        rtn = pleco_group_hold(false);
        //printf("f0_npulse: %d, f1_npulse: %d\r\n", f0_npulse, f1_npulse);
    }
	else if (freq_0 > 23 && freq_0 <= 40)  // only support: 24、28、32、36、40 (ns)
	{
		          //(freq_0/3-7):  0   1   2     3     4     5     6     7
		          //         set: 21  24  27    30    33    36    39    42
		          //         rel: 13  16  20    24    28    32    36    40
		          //         err:  8   8   7     6     5     4     3     2
		uint16_t npulse_div[] = { 99, 78, 60, 2100, 1800, 1578, 1404, 1266 };  // should be divided by 3

		//correction
		freq_0 += (12 - freq_0 / 4);
		freq_1 += (12 - freq_1 / 4);

		uint8_t valueH, valueL;
		rtn = pleco_group_hold(true);

		rtn = sensor_write_reg(560, 3);  //PLL DIV

		if (freq_0 > 23 && freq_0 <= 42)
		{
			//uint16_t f0_npulse = (1000 / freq_0) + 15;// turn out to be 1us
			uint16_t f0_npulse = npulse_div[freq_0 / 3 - 7];
			valueL = f0_npulse & 0xFF;
			valueH = (f0_npulse >> 8) & 0xFF;

			rtn = sensor_write_reg(537, valueL);
			rtn = sensor_write_reg(538, valueH);
			rtn = sensor_write_reg(567, (freq_0 / 3 - 7));
			ALOGE("f0_npulse: %d", f0_npulse);
		}

		if (freq_1 > 23 && freq_1 <= 42)
		{
			//uint16_t f1_npulse = (1000 / freq_1) + 15;// turn out to be 1us
			uint16_t f1_npulse = npulse_div[freq_1 / 3 - 7];
			valueL = f1_npulse & 0xFF;
			valueH = (f1_npulse >> 8) & 0xFF;

			rtn = sensor_write_reg(541, valueL);
			rtn = sensor_write_reg(542, valueH);
			rtn = sensor_write_reg(568, (freq_1 / 3 - 7));
		}
		rtn = pleco_group_hold(false);
		//printf("f0_npulse: %d, f1_npulse: %d\r\n", f0_npulse, f1_npulse);
	}
    else {
        return -HW_ERR_INVALID;
    }

	ALOGE("pleco_set_modulation_frequency end");

    return rtn;
}

int pleco_get_illum_duty_cycle(uint16_t *duty)
{
    int rtn = 0;
    uint8_t f0_duty = 0;
    uint8_t f1_duty = 0;
    uint8_t value = 0;
    rtn = sensor_read_reg(547, &value);
    if (value == 0) // no change
        f0_duty = 7;
    else if ((value >> 3) > 0) {
        f0_duty = 7 + (value >> 3); // increased duty
    }
    else if ((value & 0x07) > 0) {
        f0_duty = 7 - (value & 0x07); // decreased duty
    }
    value = 0;
    rtn = sensor_read_reg(551, &value);
    if (value == 0) // no change
        f1_duty = 7;
    else if ((value >> 3) > 0) {
        f1_duty = 7 + (value >> 3); // increased duty
    }
    else if ((value & 0x07) > 0) {
        f1_duty = 7 - (value & 0x07); // decreased duty
    }
    *duty = (f0_duty << 8) | f1_duty;
	
    return rtn;
}

// duty range 0 <- 7 -> 14, num 7 means no change, 0.2ns/step
int pleco_set_illum_duty_cycle(uint16_t duty)
{
	uint8_t f0_duty = (duty >> 8) & 0xFF;
	uint8_t f1_duty = duty & 0xFF;
	if (f0_duty > 14 || f1_duty > 14)
		return -HW_ERR_INVALID;

    int rtn = 0;
    uint8_t value = 0;
    rtn = pleco_group_hold(true);
    if (f0_duty == 7) { // no change
        rtn = sensor_write_reg(547, 0); // freq_0
    }
    else if (f0_duty > 7) { // increase duty
        value = f0_duty - 7;
        rtn = sensor_write_reg(547, (value << 3));
    }
    else if (f0_duty < 7) { // decrease duty
        value = 7 - f0_duty;
        rtn = sensor_write_reg(547, value);
    }
    value = 0;
    if (f1_duty == 7) { // no change
        rtn = sensor_write_reg(551, 0); // freq_1
    }
    else if (f1_duty > 7) { // increase duty
        value = f1_duty - 7;
        rtn = sensor_write_reg(551, (value << 3));
    }
    else if (f1_duty < 7) { // decrease duty
        value = 7 - f1_duty;
        rtn = sensor_write_reg(551, value);
    }
    rtn = pleco_group_hold(false);
    return rtn;
}

int pleco_get_illum_duty_cycle_list(uint8_t mod_freq, float *duty_cycle_list)
{
    float duty = 0;

    for (int step = -7; step <= 7; step++) {
        duty = step*0.2; // 0.2ns per step

        duty_cycle_list[step + 7] = duty;
    }

    return 0;
}

int pleco_get_master_slave_mode(uint8_t *mode)
{
    return 0;
}

int pleco_set_master_slave_mode(uint8_t mode)
{
    int rtn;
    uint8_t value;
    if (mode == 0x01) {//开启从模式（寄存器触发）
        rtn = sensor_write_reg(16, 1);
        rtn |= sensor_write_reg(0, 1);
        rtn |= sensor_write_reg(31, 1);
        rtn |= sensor_write_reg(32, 0);
        rtn |= sensor_write_reg(16, 0);
        ALOGD("pleco_test_pattern slave mode!");
    }
    else if (mode == 0x00) {//关闭从模式
        rtn = sensor_read_reg(21, &value);
        rtn |= sensor_write_reg(16, 1);
        rtn |= sensor_write_reg(0, 1);
        switch (value) {
        case 16:
            rtn |= sensor_write_reg(31, 170);
            rtn |= sensor_write_reg(32, 42);
            break;

        case 80:
        case 64:
            rtn |= sensor_write_reg(31, 85);
            rtn |= sensor_write_reg(32, 85);
            break;

        case 0:
            rtn |= sensor_write_reg(31, 170);
            rtn |= sensor_write_reg(32, 42);
            break;

        case 192:
            rtn |= sensor_write_reg(31, 170);
            rtn |= sensor_write_reg(32, 42);
            break;

        default:
            ;
        }
        rtn |= sensor_write_reg(16, 0);
        ALOGD("pleco_test_pattern master mode!");
    }
    return rtn;
}

int pleco_get_data_output_mode(uint8_t *mode)
{
    int rtn = 0;
    uint8_t value = 0;
    rtn = sensor_read_reg(21, &value);
    *mode = (value >> 6);

    return rtn;
}

int pleco_set_data_output_mode(uint8_t mode)
{
    int rtn = 0;
    if (mode > 3)
        return -HW_ERR_INVALID;
	unsigned char value = 0;
    rtn = sensor_read_reg(21, &value);
    value = (value & 0x3F) | (mode << 6);
    rtn = sensor_write_reg(21, value);
    ALOGE("pleco_set_data_output_mode: %d\r\n", mode);
    return rtn;
}

/**
* @brief  pleco_set_img_mirror_flip 璁剧疆sensor闀滃儚
* @param  [in] uint8_t mode
* @return int
*/
int pleco_set_img_mirror_flip(uint8_t mode)
{
    int rtn;
    uint8_t value;
    rtn = sensor_read_reg(18, &value);
	rtn = pleco_group_hold(true);
    switch (mode)
    {
    case IMAGE_NORMAL: // V = 1, H = 0
        value &= ~(3 << 5);
		value |= (2 << 5);
        rtn = sensor_write_reg(18, value);
        break;
    case IMAGE_H_MIRROR: // V = 1, H = 1
		value &= ~(3 << 5);
        value |= (3 << 5);
        rtn = sensor_write_reg(18, value);
        break;
    case IMAGE_V_MIRROR: // V = 0, H = 0
		value &= ~(3 << 5);
        rtn = sensor_write_reg(18, value);
        break;
    case IMAGE_HV_MIRROR: // V = 0, H = 1
		value &= ~(3 << 5);
        value |= (1 << 5);
        rtn = sensor_write_reg(18, value);
        break;
    default:
        rtn = -HW_ERR_INVALID;
        break;
    }
	rtn = pleco_group_hold(false);
	//ALOGE("pleco_set_img_mirror_flip: %d, mode: %d \r\n", value, mode);
    return rtn;
}

/**
* @brief  pleco_get_img_mirror_flip 鑾峰彇褰撳墠闀滃儚鐘舵€?
* @param  [out] uint8_t* mode
* @return int
*/
int pleco_get_img_mirror_flip(uint8_t *mode)
{
    int rtn;
    uint8_t value;
    rtn = sensor_read_reg(18, &value);
	uint8_t mode_mask[] = {2, 3, 0, 1};
    value = (value >> 5) & 0x03;
	*mode = mode_mask[value];

    return rtn;
}

/**
* @brief  pleco_test_pattern test pattern璁剧疆
* @param  [in] uint8_t mode
* @return int
*/
int pleco_test_pattern(uint8_t mode)
{
    int rtn;
    uint8_t value;
    rtn = sensor_read_reg(580, &value);
    if (mode == true)
    {
        value |= 0x02;
    }
    else
    {
        value &= 0xFD;
    }
    rtn = pleco_group_hold(true);
    rtn = sensor_write_reg(580, value+2);
    rtn = pleco_group_hold(false);
    return 0;
}

int pleco_get_pixel_binning(uint8_t *mode)
{

    return 0;
}

// 2x2 binning, resolution changed from 1280*960 to 640*480
int pleco_set_pixel_binning(uint8_t mode)
{

    return 0;
}

int pleco_set_binning_mode(uint8_t mode)
{
    int ret;
    uint8_t value, freq_mode;
    if (mode > 1) {
        return -HW_ERR_INVALID;
    }

    ret = sensor_read_reg(21, &value);
    if (value == 64 || value == 80) // SMFD
        freq_mode = SINGLE_FREQ;
    else if (value == 0 || value == 16) // DMFD
        freq_mode = DUAL_FREQ;
    else {
        return HW_ERR_NO_SUPPORT;//AF
    }

    if (start_streaming_called) 
    {
        ret = sensor_write_reg(17, 252); // 停流
        ret |= sensor_write_reg(17, 253);
    }
    if (ret < 0)
        return ret;

    switch (mode) {
        case BINNING_NO:
            if (freq_mode == DUAL_FREQ) {
                for (int i = 0; i < sizeof(pleco_reglist_nobinning_dmfd) / sizeof(struct regList); i++) {
                    ret = sensor_write_reg(pleco_reglist_nobinning_dmfd[i].reg, pleco_reglist_nobinning_dmfd[i].val);
                    if (ret < 0)
                        break;
                }
                //DEBUG2LOG("pleco_set_binning_mode:set to BINNING_NO, freq_mode=%d", freq_mode);
                get_src_info.embedded_data_size = 1920 * 1.5 * 12;
            }
            else {
                for (int i = 0; i < sizeof(pleco_reglist_nobinning_smfd) / sizeof(struct regList); i++) {
                    ret = sensor_write_reg(pleco_reglist_nobinning_smfd[i].reg, pleco_reglist_nobinning_smfd[i].val);
                    if (ret < 0)
                        break;
                }
                //DEBUG2LOG("pleco_set_binning_mode:set to BINNING_NO, freq_mode=%d", freq_mode);
                get_src_info.embedded_data_size = 1920 * 1.5 * 6;
            }
            break;

        case BINNING_2X2:
            if (freq_mode == DUAL_FREQ) {
                for (int i = 0; i < sizeof(pleco_reglist_2xbinning_dmfd) / sizeof(struct regList); i++) {
                    ret = sensor_write_reg(pleco_reglist_2xbinning_dmfd[i].reg, pleco_reglist_2xbinning_dmfd[i].val);
                    if (ret < 0)
                        break;
                }
                //DEBUG2LOG("pleco_set_binning_mode:set to BINNING_2X2, freq_mode=%d", freq_mode);
                get_src_info.embedded_data_size = 1920 / 2 * 1.5 * 12  ;
            }
            else {
                for (int i = 0; i < sizeof(pleco_reglist_2xbinning_smfd) / sizeof(struct regList); i++) {
                    ret = sensor_write_reg(pleco_reglist_2xbinning_smfd[i].reg, pleco_reglist_2xbinning_smfd[i].val);
                    if (ret < 0)
                        break;
                }
                //DEBUG2LOG("pleco_set_binning_mode:set to BINNING_2X2, freq_mode=%d", freq_mode);
            }
            get_src_info.embedded_data_size = 1920 / 2 * 1.5 * 6;
            break;

        default:
            ;
    }
    if (start_streaming_called) {
        ret |= sensor_write_reg(1, 0x01); // 开流
        ret |= sensor_write_reg(1, 0x00);
    }

    return ret;
}

int pleco_get_binning_mode(uint8_t *mode)
{
    uint8_t value;
    int rtn = sensor_read_reg(21, &value);
    if (value == 32 || value == 96) // DMFD 4xbinning mode // SMFD 4xbinning mode
        *mode = 2;
    else if (value == 16 || value == 80) // DMFD 2xbinning mode // SMFD 2xbinning mode
        *mode = 1;
    else if (value == 0 || value == 64) // DMFD nobinning mode // SMFD nobinning mode
        *mode = 0;
    else {
        *mode = 0xff;
        return -HW_ERR_INVALID;
    }

    return rtn;
}

int pleco_set_frequency_mode(uint8_t mode)
{
    uint8_t binnin_mode, value;
    int ret;

    if (mode > 2) {
        return -HW_ERR_INVALID;
    }
    ret = sensor_read_reg(21, &value);
    if (value == 16 || value == 80) // DMFD 2xbinning mode //SMFD 2xbinning mode
        binnin_mode = BINNING_2X2;
    else if (value == 0 || value == 64)// DMFD nobinning mode //SMFD nobinning mode
        binnin_mode = BINNING_NO;
    else if (value == 192)//AF
        binnin_mode = BINNING_AF;
    else {
        return -HW_ERR_INVALID;
    }

    if (start_streaming_called) {
        ret = sensor_write_reg(17, 252); // 停流
        ret |= sensor_write_reg(17, 253);
    }

    if (ret < 0)
        return ret;

    switch (mode) {
        case SINGLE_FREQ:
            if (binnin_mode == BINNING_NO || binnin_mode == BINNING_AF) {
                for (int i = 0; i < sizeof(pleco_reglist_nobinning_smfd) / sizeof(struct regList); i++) {
                    ret = sensor_write_reg(pleco_reglist_nobinning_smfd[i].reg, pleco_reglist_nobinning_smfd[i].val);
                    if (ret < 0)
                        break;
                }
                //DEBUG2LOG("pleco_set_frequency_mode:set to SINGLE_FREQ, binnin_mode=%d", binnin_mode);
                get_src_info.embedded_data_size = 1920 * 1.5 * 6;
            }
            else {
                for (int i = 0; i < sizeof(pleco_reglist_2xbinning_smfd) / sizeof(struct regList); i++) {
                    ret = sensor_write_reg(pleco_reglist_2xbinning_smfd[i].reg, pleco_reglist_2xbinning_smfd[i].val);
                    if (ret < 0)
                        break;
                }
                //DEBUG2LOG("pleco_set_frequency_mode:set to SINGLE_FREQ, binnin_mode=%d", binnin_mode);
                get_src_info.embedded_data_size = 1920 / 2 * 1.5 * 6;
            }
            break;

        case DUAL_FREQ:
            if (binnin_mode == BINNING_NO || binnin_mode == BINNING_AF) {
                for (int i = 0; i < sizeof(pleco_reglist_nobinning_dmfd) / sizeof(struct regList); i++) {
                    ret = sensor_write_reg(pleco_reglist_nobinning_dmfd[i].reg, pleco_reglist_nobinning_dmfd[i].val);
                    if (ret < 0)
                        break;
                }
                //DEBUG2LOG("pleco_set_frequency_mode:set to DUAL_FREQ, binnin_mode=%d", binnin_mode);
                get_src_info.embedded_data_size = 1920 * 1.5 * 12;
            }
            else {
                for (int i = 0; i < sizeof(pleco_reglist_2xbinning_dmfd) / sizeof(struct regList); i++) {
                    ret = sensor_write_reg(pleco_reglist_2xbinning_dmfd[i].reg, pleco_reglist_2xbinning_dmfd[i].val);
                    if (ret < 0)
                        break;
                }
                //DEBUG2LOG("pleco_set_frequency_mode:set to DUAL_FREQ, binnin_mode=%d", binnin_mode);
                get_src_info.embedded_data_size = 1920 / 2 * 1.5 * 12;
            }
            break;

        case AF_FREQ:
            for (int i = 0; i < sizeof(pleco_reglist_af) / sizeof(struct regList); i++) {
                ret = sensor_write_reg(pleco_reglist_af[i].reg, pleco_reglist_af[i].val);
                if (ret < 0)
                    break;
            }
            //DEBUG2LOG("pleco_set_frequency_mode:set to AF_FREQ, binnin_mode=%d", binnin_mode);
            get_src_info.embedded_data_size = 1920 * 1.5 * 3;
            break;

        default:
            ;
    }

    if (start_streaming_called) {
        ret |= sensor_write_reg(1, 0x01); // 开流
        ret |= sensor_write_reg(1, 0x00);
    }

    return ret;
}

int pleco_get_frequency_mode(uint8_t *mode)
{
    uint8_t value;
    int rtn = sensor_read_reg(21, &value);
    if (value == 64 || value == 80) // SMFD
        *mode = SINGLE_FREQ;
    else if(value == 0 || value == 16) // DMFD
        *mode = DUAL_FREQ;
    else if (value == 192) // AF
        *mode = AF_FREQ;
    return rtn;
}

int pleco_get_af_depth(uint16_t *value)
{
    int rtn = 0;
    rtn = sensor_write_reg(776, 0);
    rtn = sensor_write_reg(776, 1);

    uint8_t value_H = 0, value_L = 0;
    rtn = sensor_read_reg(844, &value_L);
    rtn = sensor_read_reg(845, &value_H);
    *value = value_L + (value_H * 256);
    *value &= 0x1fff;  // low 13 bit valid
    return rtn;
}

static int eeprom_write(uint8_t addr, uint16_t reg, uint16_t reg_size, uint32_t* data, uint16_t data_size)
{
    int ret = 0;
    i2c_msg_t msg;

    msg.slave_addr = addr;
    msg.rw_mode = I2C_M_WT;
    msg.reg = reg;
    msg.reg_size = reg_size;
    msg.data = data;
    msg.data_size = data_size;

    ret = orbbec_i2c_writeread(&msg, 1);

    usleep(6000);

    return ret;
}

static int eeprom_read(uint8_t addr, uint16_t reg, uint16_t reg_size, uint32_t* data, uint16_t data_size)
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


int rk1608_pleco_fm24c128_eeprom_write_protect(uint8_t en)
{
    int ret = 0;
    uint8_t data = 0x00;
    uint8_t p_write = 0x0D;

    ret = eeprom_write(0xB0, FM24C128D_CC_WREN_REG, 2, &data, 1);
    if (ret < 0) {
        return ret;
    }

    p_write |= (en << 1);

    ret = eeprom_write(0xB0, FM24C128D_CSP_CONFIG_REG, 2, &p_write, 1);
    if (ret < 0) {
        return ret;
    }

    return ret;
}

int rk1608_pleco_fm24c128d_eeprom_write(uint16_t offset, uint8_t *buf, uint16_t size)
{
    int ret = 0;

    uint8_t cnt = 0;
    uint8_t blocksize = FM24C128D_EEPROM_PAGE_SIZE;
    uint32_t addr = offset;
    uint8_t * buf_ptr_r = buf;
    uint8_t prog_time = size / blocksize;
    uint8_t lastsize = size % blocksize;
    uint32_t addr_offset = addr % blocksize;
    uint8_t  read_out_data[FM24C128D_EEPROM_PAGE_SIZE] = { 0 };

    ret = rk1608_pleco_fm24c128_eeprom_write_protect(0);
    if (ret < 0) {
        return ret;
    }

    if (0 == addr_offset)//write at page head 
    {
        for (cnt = 0; cnt < prog_time; cnt++)
        {
            ret = eeprom_write(FM24C128D_DATA_MEMORY, addr, 2, buf_ptr_r, blocksize);
            addr += blocksize;
            buf_ptr_r += blocksize;
        }

        if (lastsize != 0)
        {
            ret = eeprom_write(FM24C128D_DATA_MEMORY, addr, 2, buf_ptr_r, lastsize);
        }
    }
    else //write not at page head 
    {
        ret = rk1608_pleco_fm24c128d_eeprom_read((addr - addr_offset), read_out_data, addr_offset);//read the head data of a page
        if ((blocksize - addr_offset) > size) //write in a page
        {
            ret = eeprom_write(FM24C128D_DATA_MEMORY, (addr - addr_offset), 2, read_out_data, addr_offset);
            ret = eeprom_write(FM24C128D_DATA_MEMORY, addr, 2, buf_ptr_r, size);
        }
        else//write over a page
        {   //write not at head of page
            ret = eeprom_write(FM24C128D_DATA_MEMORY, (addr - addr_offset), 2, read_out_data, addr_offset);
            ret = eeprom_write(FM24C128D_DATA_MEMORY, addr, 2, buf_ptr_r, (blocksize - addr_offset));
            addr += (blocksize - addr_offset);
            buf_ptr_r += (blocksize - addr_offset);
            //write at head of page
            prog_time = (size - blocksize + addr_offset) / blocksize;
            lastsize = (size - blocksize + addr_offset) % blocksize;
            for (cnt = 0; cnt < prog_time; cnt++)
            {
                ret = eeprom_write(FM24C128D_DATA_MEMORY, addr, 2, buf_ptr_r, blocksize);
                addr += blocksize;
                buf_ptr_r += blocksize;
            }

            if (lastsize != 0)
            {
                ret = eeprom_write(FM24C128D_DATA_MEMORY, addr, 2, buf_ptr_r, lastsize);
            }
        }
    }

    ret = rk1608_pleco_fm24c128_eeprom_write_protect(1);
    if (ret < 0) {
        return ret;
    }

    return ret;
}

int rk1608_pleco_fm24c128d_eeprom_read(uint16_t offset, uint8_t *buf, uint16_t size)
{
    int ret = 0;

    uint8_t cnt;
    uint8_t blocksize = FM24C128D_EEPROM_PAGE_SIZE;
    uint8_t prog_time = size / blocksize;
    uint8_t lastsize = size % blocksize;
    uint8_t * buf_ptr_r = buf;
    uint32_t addr = offset;

    for (cnt = 0; cnt< prog_time; cnt++)
    {
        ret = eeprom_read(FM24C128D_DATA_MEMORY, addr, 2, buf_ptr_r, blocksize);
        addr += blocksize;
        buf_ptr_r += blocksize;
    }

    if (lastsize != 0)
    {
        ret = eeprom_read(FM24C128D_DATA_MEMORY, addr, 2, buf_ptr_r, lastsize);
    }

    return ret;
}

int pleco_get_tof_sensor_resolution(uint64_t *resolution)
{
    int ret;
    uint8_t mode = 0;
    ret = pleco_get_frequency_mode(&mode);
    *resolution = 0;
    if (ret == 0)
    {
        if (mode == DUAL_FREQ)
        {
            *resolution = DUAL_FREQ_4X4_BINNING_RESOLUTION << 16 | DUAL_FREQ_2X2_BINNING_RESOLUTION << 8 | DUAL_FREQ_NO_BINNING_RESOLUTION;
        }
        else
        {
            if (mode == SINGLE_FREQ)
            {
                *resolution = SINGLE_FREQ_4X4_BINNING_RESOLUTION << 16 | SINGLE_FREQ_2X2_BINNING_RESOLUTION << 8 | SINGLE_FREQ_NO_BINNING_RESOLUTION;
            }
        }
    }
    return ret;
}

int pleco_get_tof_sensor_pixel_bit(uint8_t *pixel_bit)
{
    *pixel_bit = PIXEL_BIT;
    return 0;
}

int pleco_get_mipi_pack_bit(uint8_t *mipi_pack_bit)
{
    *mipi_pack_bit = MIPI_PACK_BIT;
    return 0;
}

int pleco_init()
{
	int rtn = 0;

    pleco_sensor_init();

    return rtn;
}

int pleco_func_init()
{
    tof_sensor.init = pleco_init;
    tof_sensor.get_sensor_id = pleco_get_sensor_id;
    tof_sensor.hardware_trigger = pleco_hardware_trigger;
    tof_sensor.software_trigger = pleco_software_trigger;
    tof_sensor.video_streaming = pleco_video_streaming;
    tof_sensor.get_fps = pleco_get_fps;
    tof_sensor.set_fps = pleco_set_fps;
    tof_sensor.get_rx_temp = pleco_get_rx_temp;
    tof_sensor.get_tx_temp = pleco_get_tx_temp;
    tof_sensor.set_illum_power = pleco_set_illum_power;
    tof_sensor.get_illum_power = pleco_get_illum_power;
    tof_sensor.illum_power_control = pleco_illum_power_control;
    tof_sensor.get_integration_time = pleco_get_integration_time;
    tof_sensor.set_integration_time = pleco_set_integration_time;
    tof_sensor.get_modulation_frequency = pleco_get_modulation_frequency;
    tof_sensor.set_modulation_frequency = pleco_set_modulation_frequency;
    tof_sensor.get_illum_duty_cycle = pleco_get_illum_duty_cycle;
    tof_sensor.set_illum_duty_cycle = pleco_set_illum_duty_cycle;
    tof_sensor.get_master_slave_mode = pleco_get_master_slave_mode;
    tof_sensor.set_master_slave_mode = pleco_set_master_slave_mode;
    tof_sensor.get_data_output_mode = pleco_get_data_output_mode;
    tof_sensor.set_data_output_mode = pleco_set_data_output_mode;
    tof_sensor.get_img_mirror_flip = pleco_get_img_mirror_flip;
    tof_sensor.set_img_mirror_flip = pleco_set_img_mirror_flip;
    tof_sensor.get_pixel_binning = pleco_get_pixel_binning;
    tof_sensor.set_pixel_binning = pleco_set_pixel_binning;
    tof_sensor.set_binning_mode = pleco_set_binning_mode;
    tof_sensor.get_binning_mode = pleco_get_binning_mode;
    tof_sensor.test_pattern = pleco_test_pattern;
    tof_sensor.get_sensor_info = pleco_get_sensor_info;
    tof_sensor.get_illum_duty_cycle_list = pleco_get_illum_duty_cycle_list;
    tof_sensor.eeprom_write = rk1608_pleco_fm24c128d_eeprom_write;
    tof_sensor.eeprom_read = rk1608_pleco_fm24c128d_eeprom_read;
    tof_sensor.get_antijam = pleco_get_antijam;
    tof_sensor.set_antijam = pleco_set_antijam;
    tof_sensor.sensor_write_reg_8 = sensor_write_reg;
    tof_sensor.sensor_read_reg_8 = sensor_read_reg;
    tof_sensor.driver_ic_write_reg_8 = vcsel_driver_write_reg;
    tof_sensor.driver_ic_read_reg_8 = vcsel_driver_read_reg;
    tof_sensor.get_frequency_mode = pleco_get_frequency_mode;
    tof_sensor.set_frequency_mode = pleco_set_frequency_mode;
    tof_sensor.get_vmghi_voltage = get_vmghi_volt;
    tof_sensor.set_vmghi_voltage = set_vmghi_volt;
    tof_sensor.get_af_depth = pleco_get_af_depth;

    tof_sensor.get_tof_sensor_resolution = pleco_get_tof_sensor_resolution;
    tof_sensor.get_tof_sensor_pixel_bit = pleco_get_tof_sensor_pixel_bit;
    tof_sensor.get_mipi_pack_bit = pleco_get_mipi_pack_bit;

    return 0;
}

