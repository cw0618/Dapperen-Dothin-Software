#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <hw_obstatus.h>
#include "mlx75027_max96717_max96716a.h"

//#define DAC5574         5574
//#define QUAD_CXA4016    44016

static uint16_t HMAX = 694;
static uint16_t vcsel_driver_type = 5574;
static uint8_t vcsel_number = 1;

static uint8_t use_trigger_mode = 0;
static uint16_t g_u16ModFreqParam = 0; 
static uint16_t g_u16DutyParam = 0xffff;
static uint8_t mcu_trig_sensor_flag = 0;//1:双频模式，mcu定时触发sensor；此时要是需要和sensor通信应先关闭触发通信

static uint8_t module_type = 0;//1:理想模组 2：奥比模组

#define SINGLE_FREQ    0
#define DUAL_FREQ      1

struct regList_st {
	uint16_t reg;
	uint8_t val;
};	

typedef struct ic_hg {
    uint8_t D;
    uint16_t current;
};

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
//#define ALOGE(fmt,...) tops_t.qsee_log(TEE_LOG_LEVEL_ERROR, "[ERROR] [%s(%d)] : " fmt"\n",__FUNCTION__,__LINE__,##__VA_ARGS__)
#define ALOGE(fmt,...) printf("[DEBUG] [%s(%d)] : " fmt"\n",__FUNCTION__,__LINE__,##__VA_ARGS__)

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
#define dothin_pmu_set_voltage     tops_t.ap_ops.PmuSetVoltage
#define dothin_set_gpio_level      tops_t.ap_ops.SetGpioPinLevel

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

static struct ic_hg ic_hg_table[] =
{
#if 0    //4#单灯串联 调制频率：80MHz 占空比：100% 积分时间：500us 采集单频shuffle的第四个脉冲的平均电流(mA)

    { 0, 0 },
    { 23, 26 },
    { 24, 58 },
    { 25, 98 },
    { 26, 141 },
    { 27, 215 },
    { 28, 287 },
    { 29, 379 },
    { 30, 483 },
    { 31, 593 },
    { 32, 727 },
    { 33, 860 },
    { 34, 1005 },
    { 35, 1161 },
    { 36, 1322 },
    { 37, 1490 },
    { 38, 1666 },
    { 39, 1849 },
    { 40, 2037 },
    { 41, 2238 },
    { 42, 2435 },
    { 43, 2630 },
    { 44, 2841 },
    { 45, 2996 },
    { 46, 3181 },
    { 47, 3313 },
    { 48, 3429 },
    { 49, 3530 },
    { 50, 3591 },
    { 51, 3686 },
    { 52, 3749 },
    { 53, 3806 },
    { 54, 3855 },
    { 55, 3909 },
    { 56, 3963 },
    { 57, 4014 },
    { 58, 4049 },
    { 59, 4084 },
    { 60, 4119 },
    { 61, 4148 },
    { 62, 4176 },
    { 63, 4207 },
    { 64, 4240 },
    { 65, 4251 },
    { 66, 4272 },
    { 67, 4294 },
    { 68, 4314 },
    { 69, 4338 },
    { 70, 4355 },
    { 71, 4374 },
    { 72, 4386 },
    { 73, 4407 },
    { 74, 4422 },
    { 75, 4447 },
    { 76, 4472 },
    { 77, 4490 },
    { 78, 4501 },
    { 79, 4527 },
    { 80, 4539 },
    { 81, 4555 },
    { 82, 4567 },
    { 83, 4579 },
    { 84, 4582 },
    { 85, 4596 },
    { 86, 4606 },
    { 87, 4617 },
    { 88, 4628 },
    { 89, 4637 },
    { 90, 4644 },
    { 91, 4655 },
    { 92, 4667 },
    { 93, 4671 },
    { 94, 4680 },
    { 95, 4686 },
    { 96, 4698 },
    { 97, 4707 },
    { 98, 4713 },
    { 99, 4723 },
    { 100, 4728 },
    { 101, 4734 },
    { 102, 4740 },
    { 103, 4763 },
    { 104, 4775 },
    { 105, 4785 },
    { 106, 4796 },
    { 107, 4805 },
    { 108, 4814 },
    { 109, 4818 },
    { 110, 4835 },
    { 111, 4842 },
    { 112, 4846 },
    { 113, 4850 },
    { 114, 4854 },
    { 115, 4861 },
    { 116, 4865 },
    { 117, 4867 },
    { 118, 4874 },
    { 119, 4880 },
    { 120, 4881 },
    { 121, 4886 },
    { 122, 4894 },
    { 123, 4900 },
    { 124, 4903 },
    { 125, 4908 },
    { 126, 4916 },
    { 127, 4918 }
 
#else  //2#双灯串联 调制频率：80MHz 占空比：100% 积分时间：500us 采集单频shuffle的第四个脉冲的平均电流(mA)

    { 0, 0 },
    { 23, 7 },
    { 24, 33 },
    { 25, 82 },
    { 26, 136 },
    { 27, 199 },
    { 28, 273 },
    { 29, 366 },
    { 30, 466 },
    { 31, 580 },
    { 32, 703 },
    { 33, 841 },
    { 34, 984 },
    { 35, 1136 },
    { 36, 1297 },
    { 37, 1467 },
    { 38, 1636 },
    { 39, 1819 },
    { 40, 2010 },
    { 41, 2203 },
    { 42, 2407 },
    { 43, 2603 },
    { 44, 2814 },
    { 45, 3031 },
    { 46, 3242 },
    { 47, 3450 },
    { 48, 3704 },
    { 49, 3832 },
    { 50, 3994 },
    { 51, 4125 },
    { 52, 4217 },
    { 53, 4323 },
    { 54, 4373 },
    { 55, 4466 },
    { 56, 4519 },
    { 57, 4597 },
    { 58, 4644 },
    { 59, 4688 },
    { 60, 4725 },
    { 61, 4757 },
    { 62, 4795 },
    { 63, 4821 },
    { 64, 4850 },
    { 65, 4880 },
    { 66, 4901 },
    { 67, 4930 },
    { 68, 4950 },
    { 69, 4970 },
    { 70, 4991 },
    { 71, 5006 },
    { 72, 5027 },
    { 73, 5045 },
    { 74, 5057 },
    { 75, 5069 },
    { 76, 5090 },
    { 77, 5103 },
    { 78, 5118 },
    { 79, 5134 },
    { 80, 5142 },
    { 81, 5154 },
    { 82, 5168 },
    { 83, 5175 },
    { 84, 5188 },
    { 85, 5234 },
    { 86, 5241 },
    { 87, 5254 },
    { 88, 5264 },
    { 89, 5277 },
    { 90, 5284 },
    { 91, 5294 },
    { 92, 5304 },
    { 93, 5308 },
    { 94, 5320 },
    { 95, 5327 },
    { 96, 5332 },
    { 97, 5343 },
    { 98, 5353 },
    { 99, 5360 },
    { 100, 5363 },
    { 101, 5366 },
    { 102, 5376 },
    { 103, 5381 },
    { 104, 5393 },
    { 105, 5394 },
    { 106, 5403 },
    { 107, 5406 },
    { 108, 5414 },
    { 109, 5419 },
    { 110, 5422 },
    { 111, 5429 },
    { 112, 5427 },
    { 113, 5436 },
    { 114, 5441 },
    { 115, 5445 },
    { 116, 5450 },
    { 117, 5455 },
    { 118, 5460 },
    { 119, 5463 },
    { 120, 5466 },
    { 121, 5468 },
    { 122, 5474 },
    { 123, 5481 },
    { 124, 5482 },
    { 125, 5493 },
    { 126, 5495 },
    { 127, 5498 }
#endif
};

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

static int gpio_control(int pin, bool level)
{
    dothin_set_gpio_level(pin, level, dothin_device_id);
    return 0; // should wrap method by SDK
}

static int dothin_config()
{
#if !DEBUG_MLX75027_MAX96717_MAX96716A_IN_QT
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
	ret = dothin_set_sensor_clock(true, 25 * 10, dothin_device_id); // 25Mhz mclk
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

    SENSOR_POWER sensor_power[] = { POWER_AVDD, POWER_DOVDD, POWER_DVDD, POWER_AFVCC, POWER_VPP };
    int           power_value[] = { 1800, 1800, 1200, 0, 0 };
    ret = dothin_pmu_set_voltage(sensor_power, power_value, 5, dothin_device_id);
    if (ret < 0) {
        DDEBUG("dothin_sensor_enable ret=%d", ret);
    }
    usleep(1000 * 50);

    gpio_control(1, 1);//MCU_MODE 0：bootloader 1：normal
    usleep(500);
    gpio_control(2, 1);//MIPI_SEL 0：CHA->dothin 1：CHB->dothin
    usleep(500);

    return ret;
#endif

    
}

static int max96176a_write_reg(uint16_t reg, uint8_t value)
{
    if (reg == 0xffff)
    {
        usleep(value * 1000);
    }
    else
    {
        int rtn = i2c_reg_write(MAX96716A_ADDR, reg, 2, value, 1);
        return rtn;
    }
}

static int max96176a_read_reg(uint16_t reg, uint8_t *value)
{
    int rtn = i2c_reg_read(MAX96716A_ADDR, reg, 2, value, 1);
    return rtn;
}

static int max96717_write_reg(uint16_t reg, uint8_t value)
{
    if (reg == 0xffff)
    {
        usleep(value * 1000);
    }
    else
    {
        int rtn = i2c_reg_write(MAX96717_ADDR, reg, 2, value, 1);
        return rtn;
    }
}

static int max96717_read_reg(uint16_t reg, uint8_t *value)
{
    int rtn = i2c_reg_read(MAX96717_ADDR, reg, 2, value, 1);
    return rtn;
}

static int sensor_write_reg(uint16_t reg, uint8_t value)
{
    if (reg == 0xffff)
    {
        usleep(value * 1000);
    }
    else
    {
        int rtn = i2c_reg_write(SENSOR_ADDR, reg, 2, value, 1);
        return rtn;
    }
}

static int sensor_read_reg(uint16_t reg, uint8_t *value)
{
    int rtn = i2c_reg_read(SENSOR_ADDR, reg, 2, value, 1);
    return rtn;
}

static int mcu_write_reg(uint16_t cmd, uint16_t value)
{
    int rtn = -1, try_conut = 10;
    while (try_conut-- > 0 && rtn < 0)
    {
        rtn = i2c_reg_write(MCU_ADDR, cmd, 2, value, 2);
        usleep(50000);//wait print in mcu
    }
    return rtn;
}

static int mcu_read_reg(uint16_t cmd, uint16_t *value)
{
    int rtn = -1, try_conut = 10;

    if (cmd == OP_GET_DEVICE_ID || cmd == OP_GET_APP_VERSION)
    {
        while (try_conut-- > 0 && rtn < 0)
        {
            rtn = i2c_reg_read(MCU_ADDR, cmd, 2, value, 2);
        }
    }
    else
    {
        while (try_conut-- > 0 && rtn < 0)
        {
            rtn = i2c_reg_read(MCU_ADDR, cmd, 2, value, 1);
        }
    }

    return rtn;
}

uint16_t FineTab_Current(struct ic_hg *a, uint16_t TabLong, uint8_t d_data)
{
    uint16_t st, ed, m;
    uint16_t i;

    st = 0;
    ed = TabLong - 1;
    i = 0;

    if (d_data >= a[ed].D) return ed;
    else if (d_data <= a[st].D) return st;

    while (st < ed)
    {
        m = (st + ed) / 2;

        if (d_data == a[m].D) break;
        if (d_data < a[m + 1].D && d_data > a[m].D) break;


        if (d_data > a[m].D) st = m;  //ed = m ;
        else ed = m;//st = m ;

        if (i++ > TabLong) break;
    }

    if (st > ed) return 0;

    return a[m].current;
}


uint8_t FineTab_D(struct ic_hg *a, uint16_t TabLong, uint16_t current_data)
{
    uint16_t st, ed, m;
    uint16_t i;

    st = 0;
    ed = TabLong - 1;
    i = 0;

    if (current_data >= a[ed].current) return ed;
    else if (current_data <= a[st].current) return st;

    while (st < ed)
    {
        m = (st + ed) / 2;

        if (current_data == a[m].current) break;
        if (current_data < a[m + 1].current && current_data > a[m].current) break;


        if (current_data > a[m].current) st = m;  //ed = m ;
        else ed = m;//st = m ;

        if (i++ > TabLong) break;
    }
    if (st > ed) return 0;

    return a[m].D;
}

int mlx75027_max96717_max96716a_get_rx_temp(float *temperature)
{
    uint8_t value = 0;
    int rtn = sensor_read_reg(0x1403, &value);
    *temperature = value - 40;//°C

    return rtn;
}

int mlx75027_max96717_max96716a_get_tx_temp(float *temperature)
{    
    int rtn = 0;
    uint8_t reg = 0;
    uint16_t value = 0, valid_data = 0;
    if (module_type == 1) //理想模组没有tx温度传感器，直接赋值返回
    {
        *temperature = 0;
        return 0;
    }
    if (mcu_trig_sensor_flag == 1)
    {
        mcu_write_reg(OP_SET_START, 0);
    }
    rtn = i2c_reg_read(TMP112_ADDR, reg, 1, &value, 2);
    if (mcu_trig_sensor_flag == 1)
    {
        mcu_write_reg(OP_SET_START, 1);
    }
    valid_data = value >> 4;
    if (valid_data & 0x0800)//negative temperatures
    {
        *temperature = ((~(valid_data - 1)) & 0x07ff) * -0.0625f;
    }
    else//positive temperatures
    {
        *temperature = (valid_data & 0x07ff) * 0.0625f;
    }

	return  rtn;
}

int mlx75027_max96717_max96716a_set_illum_power(uint8_t vcsel_num, uint8_t value_A, uint8_t value_B)
{
	int rtn = 0;
    uint16_t led1_current = 0, led2_current = 0;
    uint8_t led1_D1 = 0, led2_D2 = 0;
    if (module_type == 1) 
    {
        if (value_A == 0 && value_B == 0)//关闭TX
        {
            //理想模组需要使能LEDEN信号，否则TX不亮
            rtn = sensor_write_reg(0x21C4, 0x00);
        }
        else
        {
            //理想模组需要使能LEDEN信号，否则TX不亮
            rtn = sensor_write_reg(0x21C4, 0xff);
        }
    }
    else
    {
        led1_current = value_A * 18;
        led2_current = value_B * 18;

        led1_D1 = FineTab_D(ic_hg_table, sizeof(ic_hg_table) / sizeof(ic_hg_table[0]), led1_current);
        led2_D2 = FineTab_D(ic_hg_table, sizeof(ic_hg_table) / sizeof(ic_hg_table[0]), led2_current);

        rtn = i2c_reg_write(LED1_RES_ADDR, LED_RES_COMMAND, 1, led1_D1, 1);
        rtn = i2c_reg_write(LED2_RES_ADDR, LED_RES_COMMAND, 1, led2_D2, 1);

        //    ALOGE("set power v_A:%d _B:%d C1:%d C2:%d D1:%d D2:%d !\r\n", value_A, value_B, led1_current, led2_current, led1_D1, led2_D2);
    }
    return rtn;
}

int mlx75027_max96717_max96716a_get_illum_power(uint8_t *vcsel_num, uint8_t *value_A, uint8_t *value_B)
{
	int rtn = 0;
    uint8_t res_A = 0, res_B = 0;
    uint16_t current_A = 0, current_B = 0;

    rtn = i2c_reg_read(LED1_RES_ADDR, LED_RES_COMMAND, 1, &res_A, 1);
    rtn |= i2c_reg_read(LED2_RES_ADDR, LED_RES_COMMAND, 1, &res_B, 1);
    current_A = FineTab_Current(ic_hg_table, sizeof(ic_hg_table) / sizeof(ic_hg_table[0]), res_A);
    current_B = FineTab_Current(ic_hg_table, sizeof(ic_hg_table) / sizeof(ic_hg_table[0]), res_B);

    *value_A = current_A / 18;
    *value_B = current_B / 18;

    return rtn;
}

int mlx75027_max96717_max96716a_hardware_trigger()
{/*
    //mdevices.gpio_control(TRIGGER_PIN, 1);
    mdevices.gpio_control(TRIGGER_PIN, 0); // active low
    mdevices.sleepms(0.001); // 1us, not accurate
    //mdevices.usleep(100);
    mdevices.gpio_control(TRIGGER_PIN, 1);
    */
    return 0;
}

int mlx75027_max96717_max96716a_software_trigger()
{
    int rtn = sensor_write_reg(0x2100, 0x01);
    return rtn;
}

int mlx75027_max96717_max96716a_illum_power_control(bool enable)
{
    return 0; 
}

static struct regList_st read_sensor_reglist[] = {
    { 0x1006, 0x08 },
    { 0x1007, 0x00 },
    { 0x1040, 0x00 },
    { 0x1041, 0x96 },
    { 0x1042, 0x01 },
    { 0x1043, 0x00 },
    { 0x1044, 0x00 },
    { 0x1046, 0x01 },
    { 0x104A, 0x01 },
    { 0x1000, 0x00 },
    { 0x10D3, 0x10 },
    { 0x1448, 0x06 },
    { 0x1449, 0x40 },
    { 0x144A, 0x06 },
    { 0x144B, 0x40 },
    { 0x144C, 0x06 },
    { 0x144D, 0x40 },
    { 0x144E, 0x06 },
    { 0x144F, 0x40 },
    { 0x1450, 0x06 },
    { 0x1451, 0x40 },
    { 0x1452, 0x06 },
    { 0x1453, 0x40 },
    { 0x1454, 0x06 },
    { 0x1455, 0x40 },
    { 0x1456, 0x06 },
    { 0x1457, 0x40 },
    { 0x2203, 0x1E },
    { 0x2C08, 0x01 },
    { 0x3C2B, 0x1B },
    { 0x400E, 0x01 },
    { 0x400F, 0x81 },
    { 0x40D1, 0x00 },
    { 0x40D2, 0x00 },
    { 0x40D3, 0x00 },
    { 0x40DB, 0x3F },
    { 0x40DE, 0x40 },
    { 0x40DF, 0x01 },
    { 0x4134, 0x04 },
    { 0x4135, 0x04 },
    { 0x4136, 0x04 },
    { 0x4137, 0x04 },
    { 0x4138, 0x04 },
    { 0x4139, 0x04 },
    { 0x413A, 0x04 },
    { 0x413B, 0x04 },
    { 0x413C, 0x04 },
    { 0x4146, 0x01 },
    { 0x4147, 0x01 },
    { 0x4148, 0x01 },
    { 0x4149, 0x01 },
    { 0x414A, 0x01 },
    { 0x414B, 0x01 },
    { 0x414C, 0x01 },
    { 0x414D, 0x01 },
    { 0x4158, 0x01 },
    { 0x4159, 0x01 },
    { 0x415A, 0x01 },
    { 0x415B, 0x01 },
    { 0x415C, 0x01 },
    { 0x415D, 0x01 },
    { 0x415E, 0x01 },
    { 0x415F, 0x01 },
    { 0x4590, 0x00 },
    { 0x4591, 0x2E },
    { 0x4684, 0x00 },
    { 0x4685, 0xA0 },
    { 0x4687, 0xA1 },
    { 0x471E, 0x07 },
    { 0x471F, 0xC9 },
    { 0x473A, 0x07 },
    { 0x473B, 0xC9 },
    { 0x4770, 0x00 },
    { 0x4771, 0x00 },
    { 0x4772, 0x1F },
    { 0x4773, 0xFF },
    { 0x4778, 0x06 },
    { 0x4779, 0xA4 },
    { 0x477A, 0x07 },
    { 0x477B, 0xAE },
    { 0x477D, 0xD6 },
    { 0x4788, 0x06 },
    { 0x4789, 0xA4 },
    { 0x478C, 0x1F },
    { 0x478D, 0xFF },
    { 0x478E, 0x00 },
    { 0x478F, 0x00 },
    { 0x4792, 0x00 },
    { 0x4793, 0x00 },
    { 0x4796, 0x00 },
    { 0x4797, 0x00 },
    { 0x479A, 0x00 },
    { 0x479B, 0x00 },
    { 0x479C, 0x1F },
    { 0x479D, 0xFF },
    { 0x479E, 0x00 },
    { 0x479F, 0x00 },
    { 0x47A2, 0x00 },
    { 0x47A3, 0x00 },
    { 0x47A6, 0x00 },
    { 0x47A7, 0x00 },
    { 0x47AA, 0x00 },
    { 0x47AB, 0x00 },
    { 0x47AC, 0x1F },
    { 0x47AD, 0xFF },
    { 0x47AE, 0x00 },
    { 0x47AF, 0x00 },
    { 0x47B2, 0x00 },
    { 0x47B3, 0x00 },
    { 0x47B6, 0x00 },
    { 0x47B7, 0x00 },
    { 0x47BA, 0x00 },
    { 0x47BB, 0x00 },
    { 0x47BC, 0x1F },
    { 0x47BD, 0xFF },
    { 0x47BE, 0x00 },
    { 0x47BF, 0x00 },
    { 0x47C2, 0x00 },
    { 0x47C3, 0x00 },
    { 0x47C6, 0x00 },
    { 0x47C7, 0x00 },
    { 0x47CA, 0x00 },
    { 0x47CB, 0x00 },
    { 0x4834, 0x00 },
    { 0x4835, 0xA0 },
    { 0x4837, 0xA1 },
    { 0x4878, 0x00 },
    { 0x4879, 0xA0 },
    { 0x487B, 0xA1 },
    { 0x48BC, 0x00 },
    { 0x48BD, 0xA0 },
    { 0x48BF, 0xA1 },
    { 0x4954, 0x00 },
    { 0x4955, 0xA0 },
    { 0x4957, 0xA1 },
    { 0x4984, 0x00 },
    { 0x4985, 0xA0 },
    { 0x4987, 0xA1 },
    { 0x49B9, 0x78 },
    { 0x49C3, 0x3C },
    { 0x49C9, 0x76 },
    { 0x49D3, 0x3F },
    { 0x49DC, 0x00 },
    { 0x49DD, 0xA0 },
    { 0x49DF, 0xA1 },
    { 0x49EF, 0x78 },
    { 0x49F9, 0x3C },
    { 0x49FF, 0x78 },
    { 0x4A05, 0x3C },
    { 0x4A0B, 0x76 },
    { 0x4A11, 0x3F },
    { 0x4A1A, 0x00 },
    { 0x4A1B, 0xA0 },
    { 0x4A1D, 0xA1 },
    { 0x4A1F, 0x78 },
    { 0x4A29, 0x3C },
    { 0x4A4A, 0x00 },
    { 0x4A4B, 0xA0 },
    { 0x4A4D, 0xA1 },
    { 0x4A7A, 0x00 },
    { 0x4A7B, 0xA0 },
    { 0x4A7D, 0xA1 },
    { 0x4AEE, 0x00 },
    { 0x4AEF, 0xA0 },
    { 0x4AF1, 0xA1 },
    { 0x4B2E, 0x00 },
    { 0x4B2F, 0xA0 },
    { 0x4B31, 0xA1 },
    { 0x4B5A, 0x00 },
    { 0x4B5B, 0xA0 },
    { 0x4B5D, 0xA1 },
    { 0x4B86, 0x00 },
    { 0x4B87, 0xA0 },
    { 0x4B89, 0xA1 },
    { 0x4B9F, 0x1A },
    { 0x4BAF, 0x1A },
    { 0x4BB7, 0x1A },
    { 0x4BC7, 0x1A },
    { 0x4BCF, 0x1A },
    { 0x4BEE, 0x00 },
    { 0x4BEF, 0xA0 },
    { 0x4BF1, 0xA1 },
    { 0x4BF7, 0x1A },
    { 0x4C01, 0x1A },
    { 0x4C58, 0x00 },
    { 0x4C59, 0xA0 },
    { 0x4C5B, 0xA1 },
    { 0x4C6E, 0x00 },
    { 0x4C6F, 0xA0 },
    { 0x4C71, 0xA1 },
    { 0x4C7A, 0x01 },
    { 0x4C7B, 0x35 },
    { 0x4CF2, 0x07 },
    { 0x4CF3, 0xC9 },
    { 0x4CF8, 0x06 },
    { 0x4CF9, 0x9B },
    { 0x4CFA, 0x07 },
    { 0x4CFB, 0xAE },
    { 0x4CFE, 0x07 },
    { 0x4CFF, 0xC9 },
    { 0x4D04, 0x06 },
    { 0x4D05, 0x98 },
    { 0x4D06, 0x07 },
    { 0x4D07, 0xB1 },
    { 0x4D18, 0x06 },
    { 0x4D19, 0xA4 },
    { 0x4D1A, 0x07 },
    { 0x4D1B, 0x49 },
    { 0x4D1E, 0x07 },
    { 0x4D1F, 0xC9 },
    { 0x4D2A, 0x07 },
    { 0x4D2B, 0xC9 },
    { 0x4D4A, 0x07 },
    { 0x4D4B, 0xC9 },
    { 0x4D50, 0x06 },
    { 0x4D51, 0x9B },
    { 0x4D52, 0x07 },
    { 0x4D53, 0xAE },
    { 0x4D56, 0x07 },
    { 0x4D57, 0xC9 },
    { 0x4D5C, 0x06 },
    { 0x4D5D, 0x98 },
    { 0x4D5E, 0x07 },
    { 0x4D5F, 0xB1 },
    { 0x4D70, 0x06 },
    { 0x4D71, 0xA4 },
    { 0x4D72, 0x07 },
    { 0x4D73, 0x49 },
    { 0x4D78, 0x06 },
    { 0x4D79, 0xA4 },
    { 0x4D7A, 0x07 },
    { 0x4D7B, 0xAE },
    { 0x4D7C, 0x1F },
    { 0x4D7D, 0xFF },
    { 0x4D7E, 0x1F },
    { 0x4D7F, 0xFF },
    { 0x4D80, 0x06 },
    { 0x4D81, 0xA4 },
    { 0x4D82, 0x07 },
    { 0x4D83, 0xAE },
    { 0x4D84, 0x1F },
    { 0x4D85, 0xFF },
    { 0x4D86, 0x1F },
    { 0x4D87, 0xFF },
    { 0x4E39, 0x07 },
    { 0x4E7B, 0x64 },
    { 0x4E8E, 0x0E },
    { 0x4E9C, 0x01 },
    { 0x4EA0, 0x01 },
    { 0x4EA1, 0x03 },
    { 0x4EA5, 0x00 },
    { 0x4EA7, 0x00 },
    { 0x4F05, 0x04 },
    { 0x4F0D, 0x04 },
    { 0x4F15, 0x04 },
    { 0x4F19, 0x01 },
    { 0x4F20, 0x01 },
    { 0x4F66, 0x0F },
    { 0x500F, 0x01 },
    { 0x5225, 0x2F },
    { 0x5227, 0x1E },
    { 0x5231, 0x19 },
    { 0x5245, 0x07 },
    { 0x5252, 0x07 },
    { 0x5253, 0x08 },
    { 0x5254, 0x07 },
    { 0x5255, 0xB4 },
    { 0x5272, 0x04 },
    { 0x5273, 0x2E },
    { 0x5282, 0x04 },
    { 0x5283, 0x2E },
    { 0x5286, 0x00 },
    { 0x5287, 0x5D },
    { 0x1C40, 0x00 },
    { 0x1010, 0x01 },
    { 0x100C, 0x07 },
    { 0x100D, 0x80 },
    { 0x100E, 0x00 },
    { 0x100F, 0x00 },
    { 0x1016, 0x03 },
    { 0x1017, 0x00 },
    { 0x1045, 0x78 },
    { 0x1047, 0x02 },
    { 0x1060, 0x00 },
    { 0x1071, 0x06 },
    { 0x10C2, 0x00 },
    { 0x10C3, 0x0A },
    { 0x10C4, 0x00 },
    { 0x10C5, 0x62 },
    { 0x10D0, 0x0A },
    { 0x10D4, 0x00 },
    { 0x10D5, 0xC5 },
    { 0x2020, 0x01 },
    { 0x2100, 0x00 },
    { 0x2F05, 0x01 },
    { 0x2F06, 0x09 },
    { 0x2F07, 0x7A },
    { 0x3071, 0x00 },
    { 0x0828, 0x00 },
    { 0x0800, 0x04 },
    { 0x0801, 0x9E },
    { 0x4010, 0x3C },
    { 0x4015, 0x00 },
    { 0x4016, 0x06 },
    { 0x5265, 0x00 },
    { 0x5266, 0x0F },
    { 0x5267, 0x54 },
    { 0x5281, 0x00 },
    { 0x5282, 0x04 },
    { 0x5283, 0x2E },
    { 0x21BE, 0x00 },
    { 0x21BF, 0x00 },
    { 0x1048, 0x00 },
    { 0x1049, 0x64 },
    { 0x104B, 0x02 },
    { 0x2108, 0x00 },
    { 0x2109, 0x00 },
    { 0x210A, 0x00 },
    { 0x210B, 0x00 },
    { 0x21E8, 0x04 },
    { 0x21C0, 0x00 },
    { 0x21C2, 0x00 },
    { 0x2120, 0x00 },
    { 0x2121, 0x01 },
    { 0x2122, 0xD4 },
    { 0x2123, 0xC0 },
    { 0x2124, 0x00 },
    { 0x2125, 0x01 },
    { 0x2126, 0xD4 },
    { 0x2127, 0xC0 },
    { 0x2128, 0x00 },
    { 0x2129, 0x01 },
    { 0x212A, 0xD4 },
    { 0x212B, 0xC0 },
    { 0x212C, 0x00 },
    { 0x212D, 0x01 },
    { 0x212E, 0xD4 },
    { 0x212F, 0xC0 },
    { 0x2130, 0x00 },
    { 0x2131, 0x01 },
    { 0x2132, 0xD4 },
    { 0x2133, 0xC0 },
    { 0x2134, 0x00 },
    { 0x2135, 0x01 },
    { 0x2136, 0xD4 },
    { 0x2137, 0xC0 },
    { 0x2138, 0x00 },
    { 0x2139, 0x01 },
    { 0x213A, 0xD4 },
    { 0x213B, 0xC0 },
    { 0x213C, 0x00 },
    { 0x213D, 0x01 },
    { 0x213E, 0xD4 },
    { 0x213F, 0xC0 },
    { 0x21B4, 0x20 },
    { 0x21B5, 0x64 },
    { 0x21B6, 0x00 },
    { 0x21B7, 0x00 },
    { 0x0804, 0x00 },
    { 0x0805, 0x01 },
    { 0x0806, 0x02 },
    { 0x0807, 0x80 },
    { 0x0808, 0x00 },
    { 0x0809, 0x00 },
    { 0x080A, 0x00 },
    { 0x080B, 0xF1 },
    { 0x080C, 0x01 },
    { 0x080D, 0x01 },
    { 0x14A5, 0x00 },
    { 0x1433, 0x00 },
    { 0x14BB, 0x01 },
    { 0x0824, 0x00 },
    { 0x2C0C, 0x10 },
    { 0x2C0D, 0x80 },
    { 0x3C18, 0x00 },
    { 0x2C0C, 0x10 },
    { 0x2C0D, 0x80 },
    { 0x0800, 0x04 },
    { 0x0801, 0x9E },
    { 0x4010, 0x3C },
    { 0x4015, 0x00 },
    { 0x4016, 0x06 },
    { 0x5265, 0x00 },
    { 0x5266, 0x0F },
    { 0x5267, 0x54 },
    { 0x2020, 0x01 },
    { 0x2100, 0x00 },
    { 0x2F05, 0x01 },
    { 0x2F06, 0x09 },
    { 0x2F07, 0x7A },
    { 0x3071, 0x00 },
    { 0x21BE, 0x00 },
    { 0x21BF, 0x00 },
    { 0x1048, 0x00 },
    { 0x1049, 0x64 },
    { 0x104B, 0x02 },
    { 0x21D4, 0x00 },
    { 0x21D5, 0x00 },
    { 0x2108, 0x00 },
    { 0x2109, 0x00 },
    { 0x210A, 0x00 },
    { 0x210B, 0x00 },
    { 0x21E8, 0x04 },
    { 0x4015, 0x00 },
    { 0x4016, 0x06 },
    { 0x2120, 0x00 },
    { 0x2121, 0x01 },
    { 0x2122, 0xD4 },
    { 0x2123, 0xC0 },
    { 0x2161, 0x01 },
    { 0x2162, 0xD4 },
    { 0x2163, 0xC0 },
    { 0x2124, 0x00 },
    { 0x2125, 0x01 },
    { 0x2126, 0xD4 },
    { 0x2127, 0xC0 },
    { 0x2164, 0x00 },
    { 0x2165, 0x01 },
    { 0x2166, 0xD4 },
    { 0x2128, 0x00 },
    { 0x2129, 0x01 },
    { 0x212A, 0xD4 },
    { 0x212B, 0xC0 },
    { 0x2167, 0xC0 },
    { 0x2168, 0x00 },
    { 0x2169, 0x01 },
    { 0x212C, 0x00 },
    { 0x212D, 0x01 },
    { 0x212E, 0xD4 },
    { 0x212F, 0xC0 },
    { 0x216A, 0xD4 },
    { 0x216B, 0xC0 },
    { 0x216C, 0x00 },
    { 0x21A0, 0x11 },
    { 0x21A1, 0x11 },
    { 0x21C8, 0x32 },
    { 0x21C9, 0x32 },
    { 0x21CA, 0x32 },
    { 0x21CB, 0x32 },
    { 0x14A5, 0x00 },
    { 0x0804, 0x00 },
    { 0x0805, 0x01 },
    { 0x0806, 0x02 },
    { 0x0807, 0x80 },
    { 0x0808, 0x00 },
    { 0x0809, 0x00 },
    { 0x080A, 0x00 },
    { 0x080B, 0xF1 },
    { 0x080C, 0x01 },
    { 0x080D, 0x01 },
    { 0x1433, 0x00 },
    { 0x4E9E, 0x00 },
    { 0x21B9, 0x00 },
    { 0x10E2, 0x01 },
};

static struct regList_st sensor_reglist[] = {
#if 0 //理想一代整机
    { 0x1006, 0x08 },
    { 0x1007, 0x00 },
    { 0x1040, 0x00 },
    { 0x1041, 0x96 },
    { 0x1042, 0x01 },
    { 0x1043, 0x00 },
    { 0x1044, 0x00 },
    { 0x1046, 0x01 },
    { 0x104A, 0x01 },
    { 0x1000, 0x00 },
    { 0x1c40, 0x00 },
    { 0x10e2, 0x01 },
    { 0x10D0, 0x0b },
    { 0X10D2, 0x00 },
    { 0x10D3, 0x10 },
    { 0x10D4, 0x00 },
    { 0x10D5, 0x9c },
    { 0x1448, 0x06 },
    { 0x1449, 0x40 },
    { 0x144A, 0x06 },
    { 0x144B, 0x40 },
    { 0x144C, 0x06 },
    { 0x144D, 0x40 },
    { 0x144E, 0x06 },
    { 0x144F, 0x40 },
    { 0x1450, 0x06 },
    { 0x1451, 0x40 },
    { 0x1452, 0x06 },
    { 0x1453, 0x40 },
    { 0x1454, 0x06 },
    { 0x1455, 0x40 },
    { 0x1456, 0x06 },
    { 0x1457, 0x40 },
    { 0x2202, 0x00 },
    { 0x2203, 0x1E },
    { 0x2c08, 0x00 },
    { 0x2C0D, 0x80 },
    { 0x400E, 0x01 },
    { 0x400F, 0x81 },
    { 0x40D1, 0x00 },
    { 0x40D2, 0x00 },
    { 0x40D3, 0x00 },
    { 0x40DB, 0x3F },
    { 0x40DE, 0x40 },
    { 0x40DF, 0x01 },
    { 0x412C, 0x00 },
    { 0x4134, 0x04 },
    { 0x4135, 0x04 },
    { 0x4136, 0x04 },
    { 0x4137, 0x04 },
    { 0x4138, 0x04 },
    { 0x4139, 0x04 },
    { 0x413A, 0x04 },
    { 0x413B, 0x04 },
    { 0x413C, 0x04 },
    { 0x4146, 0x01 },
    { 0x4147, 0x01 },
    { 0x4148, 0x01 },
    { 0x4149, 0x01 },
    { 0x414A, 0x01 },
    { 0x414B, 0x01 },
    { 0x414C, 0x01 },
    { 0x414D, 0x01 },
    { 0x4158, 0x01 },
    { 0x4159, 0x01 },
    { 0x415A, 0x01 },
    { 0x415B, 0x01 },
    { 0x415C, 0x01 },
    { 0x415D, 0x01 },
    { 0x415E, 0x01 },
    { 0x415F, 0x01 },
    { 0x4590, 0x00 },
    { 0x4591, 0x2E },
    { 0x4684, 0x00 },
    { 0x4685, 0xA0 },
    { 0x4686, 0x00 },
    { 0x4687, 0xA1 },
    { 0x471E, 0x07 },
    { 0x471F, 0xC9 },
    { 0x473A, 0x07 },
    { 0x473B, 0xC9 },
    { 0x4770, 0x00 },
    { 0x4771, 0x00 },
    { 0x4772, 0x1F },
    { 0x4773, 0xFF },
    { 0x4778, 0x06 },
    { 0x4779, 0xA4 },
    { 0x477A, 0x07 },
    { 0x477B, 0xAE },
    { 0x477D, 0xD6 },
    { 0x4788, 0x06 },
    { 0x4789, 0xA4 },
    { 0x478C, 0x1F },
    { 0x478D, 0xFF },
    { 0x478E, 0x00 },
    { 0x478F, 0x00 },
    { 0x4792, 0x00 },
    { 0x4793, 0x00 },
    { 0x4796, 0x00 },
    { 0x4797, 0x00 },
    { 0x479A, 0x00 },
    { 0x479B, 0x00 },
    { 0x479C, 0x1F },
    { 0x479D, 0xFF },
    { 0x479E, 0x00 },
    { 0x479F, 0x00 },
    { 0x47A2, 0x00 },
    { 0x47A3, 0x00 },
    { 0x47A6, 0x00 },
    { 0x47A7, 0x00 },
    { 0x47AA, 0x00 },
    { 0x47AB, 0x00 },
    { 0x47AC, 0x1F },
    { 0x47AD, 0xFF },
    { 0x47AE, 0x00 },
    { 0x47AF, 0x00 },
    { 0x47B2, 0x00 },
    { 0x47B3, 0x00 },
    { 0x47B6, 0x00 },
    { 0x47B7, 0x00 },
    { 0x47BA, 0x00 },
    { 0x47BB, 0x00 },
    { 0x47BC, 0x1F },
    { 0x47BD, 0xFF },
    { 0x47BE, 0x00 },
    { 0x47BF, 0x00 },
    { 0x47C2, 0x00 },
    { 0x47C3, 0x00 },
    { 0x47C6, 0x00 },
    { 0x47C7, 0x00 },
    { 0x47CA, 0x00 },
    { 0x47CB, 0x00 },
    { 0x4834, 0x00 },
    { 0x4835, 0xA0 },
    { 0x4836, 0x00 },
    { 0x4837, 0xA1 },
    { 0x4878, 0x00 },
    { 0x4879, 0xA0 },
    { 0x487A, 0x00 },
    { 0x487B, 0xA1 },
    { 0x48BC, 0x00 },
    { 0x48BD, 0xA0 },
    { 0x48BE, 0x00 },
    { 0x48BF, 0xA1 },
    { 0x4954, 0x00 },
    { 0x4955, 0xA0 },
    { 0x4956, 0x00 },
    { 0x4957, 0xA1 },
    { 0x4984, 0x00 },
    { 0x4985, 0xA0 },
    { 0x4986, 0x00 },
    { 0x4987, 0xA1 },
    { 0x49B8, 0x00 },
    { 0x49B9, 0x78 },
    { 0x49C2, 0x00 },
    { 0x49C3, 0x3C },
    { 0x49C8, 0x00 },
    { 0x49C9, 0x76 },
    { 0x49D2, 0x00 },
    { 0x49D3, 0x3F },
    { 0x49DC, 0x00 },
    { 0x49DD, 0xA0 },
    { 0x49DE, 0x00 },
    { 0x49DF, 0xA1 },
    { 0x49EE, 0x00 },
    { 0x49EF, 0x78 },
    { 0x49F8, 0x00 },
    { 0x49F9, 0x3C },
    { 0x49FE, 0x00 },
    { 0x49FF, 0x78 },
    { 0x4A04, 0x00 },
    { 0x4A05, 0x3C },
    { 0x4A0A, 0x00 },
    { 0x4A0B, 0x76 },
    { 0x4A10, 0x00 },
    { 0x4A11, 0x3F },
    { 0x4A1A, 0x00 },
    { 0x4A1B, 0xA0 },
    { 0x4A1C, 0x00 },
    { 0x4A1D, 0xA1 },
    { 0x4A1E, 0x00 },
    { 0x4A1F, 0x78 },
    { 0x4A28, 0x00 },
    { 0x4A29, 0x3C },
    { 0x4A4A, 0x00 },
    { 0x4A4B, 0xA0 },
    { 0x4A4C, 0x00 },
    { 0x4A4D, 0xA1 },
    { 0x4A7A, 0x00 },
    { 0x4A7B, 0xA0 },
    { 0x4A7C, 0x00 },
    { 0x4A7D, 0xA1 },
    { 0x4AEE, 0x00 },
    { 0x4AEF, 0xA0 },
    { 0x4AF0, 0x00 },
    { 0x4AF1, 0xA1 },
    { 0x4B2E, 0x00 },
    { 0x4B2F, 0xA0 },
    { 0x4B30, 0x00 },
    { 0x4B31, 0xA1 },
    { 0x4B5A, 0x00 },
    { 0x4B5B, 0xA0 },
    { 0x4B5C, 0x00 },
    { 0x4B5D, 0xA1 },
    { 0x4B86, 0x00 },
    { 0x4B87, 0xA0 },
    { 0x4B88, 0x00 },
    { 0x4B89, 0xA1 },
    { 0x4B9E, 0x00 },
    { 0x4B9F, 0x1A },
    { 0x4BAE, 0x00 },
    { 0x4BAF, 0x1A },
    { 0x4BB6, 0x00 },
    { 0x4BB7, 0x1A },
    { 0x4BC6, 0x00 },
    { 0x4BC7, 0x1A },
    { 0x4BCE, 0x00 },
    { 0x4BCF, 0x1A },
    { 0x4BEE, 0x00 },
    { 0x4BEF, 0xA0 },
    { 0x4BF0, 0x00 },
    { 0x4BF1, 0xA1 },
    { 0x4BF6, 0x00 },
    { 0x4BF7, 0x1A },
    { 0x4C00, 0x00 },
    { 0x4C01, 0x1A },
    { 0x4C58, 0x00 },
    { 0x4C59, 0xA0 },
    { 0x4C5A, 0x00 },
    { 0x4C5B, 0xA1 },
    { 0x4C6E, 0x00 },
    { 0x4C6F, 0xA0 },
    { 0x4C70, 0x00 },
    { 0x4C71, 0xA1 },
    { 0x4c7A, 0x01 },
    { 0x4c7B, 0x35 },
    { 0x4CF2, 0x07 },
    { 0x4CF3, 0xC9 },
    { 0x4CF8, 0x06 },
    { 0x4CF9, 0x9B },
    { 0x4CFA, 0x07 },
    { 0x4CFB, 0xAE },
    { 0x4CFE, 0x07 },
    { 0x4CFF, 0xC9 },
    { 0x4D04, 0x06 },
    { 0x4D05, 0x98 },
    { 0x4D06, 0x07 },
    { 0x4D07, 0xB1 },
    { 0x4D18, 0x06 },
    { 0x4D19, 0xA4 },
    { 0x4D1A, 0x07 },
    { 0x4D1B, 0x49 },
    { 0x4D1E, 0x07 },
    { 0x4D1F, 0xC9 },
    { 0x4D2A, 0x07 },
    { 0x4D2B, 0xC9 },
    { 0x4D4A, 0x07 },
    { 0x4D4B, 0xC9 },
    { 0x4D50, 0x06 },
    { 0x4D51, 0x9B },
    { 0x4D52, 0x07 },
    { 0x4D53, 0xAE },
    { 0x4D56, 0x07 },
    { 0x4D57, 0xC9 },
    { 0x4D5C, 0x06 },
    { 0x4D5D, 0x98 },
    { 0x4D5E, 0x07 },
    { 0x4D5F, 0xB1 },
    { 0x4D70, 0x06 },
    { 0x4D71, 0xA4 },
    { 0x4D72, 0x07 },
    { 0x4D73, 0x49 },
    { 0x4D78, 0x06 },
    { 0x4D79, 0xA4 },
    { 0x4D7A, 0x07 },
    { 0x4D7B, 0xAE },
    { 0x4D7C, 0x1F },
    { 0x4D7D, 0xFF },
    { 0x4D7E, 0x1F },
    { 0x4D7F, 0xFF },
    { 0x4D80, 0x06 },
    { 0x4D81, 0xA4 },
    { 0x4D82, 0x07 },
    { 0x4D83, 0xAE },
    { 0x4D84, 0x1F },
    { 0x4D85, 0xFF },
    { 0x4D86, 0x1F },
    { 0x4D87, 0xFF },
    { 0x4E39, 0x07 },
    { 0x4E7B, 0x64 },
    { 0x4E8E, 0x0E },
    { 0x4E9A, 0x00 },
    { 0x4E9C, 0x01 },
    { 0x4EA0, 0x00 },
    { 0x4EA1, 0x03 },
    { 0x4EA5, 0x00 },
    { 0x4EA7, 0x00 },
    { 0x4F05, 0x04 },
    { 0x4F0D, 0x04 },
    { 0x4F15, 0x04 },
    { 0x4F19, 0x01 },
    { 0x4F20, 0x01 },
    { 0x4F66, 0x0F },
    { 0x500F, 0x01 },
    { 0x5100, 0x84 },
    { 0x5103, 0x40 },
    { 0x5108, 0x72 },
    { 0x5109, 0x1C },
    { 0x5224, 0x00 },
    { 0x5225, 0x2F },
    { 0x5226, 0x00 },
    { 0x5227, 0x1E },
    { 0x5230, 0x00 },
    { 0x5231, 0x19 },
    { 0x5244, 0x00 },
    { 0x5245, 0x07 },
    { 0x5252, 0x07 },
    { 0x5253, 0x08 },
    { 0x5254, 0x07 },
    { 0x5255, 0xB4 },
    { 0x5271, 0x00 },
    { 0x5272, 0x04 },
    { 0x5273, 0x2E },
    { 0x5281, 0x00 },
    { 0x5282, 0x13 },
    { 0x5283, 0x9A },
    { 0x5285, 0x00 },
    { 0x5286, 0x00 },
    { 0x5287, 0x5D },
    { 0x1010, 0x01 },
    { 0x100C, 0x07 },
    { 0x100D, 0x80 },
    { 0x100E, 0x00 },
    { 0x100F, 0x00 },
    { 0x1016, 0x03 },
    { 0x1017, 0x00 },
    { 0x1045, 0x78 },
    { 0x1047, 0x02 },
    { 0x1060, 0x00 },
    { 0x1071, 0x06 },
    { 0x10C2, 0x00 },
    { 0x10C3, 0x0A },
    { 0x10C4, 0x00 },
    { 0x10C5, 0x62 },
    { 0x201C, 0x00 },
    { 0x201D, 0x00 },
    { 0x201E, 0x00 },
    { 0x2020, 0x01 },
    { 0x2100, 0x00 },
    { 0x3C18, 0x00 },
    { 0x2108, 0x00 },
    { 0x2109, 0x00 },
    { 0x210a, 0x00 },
    { 0x210b, 0x00 },
    { 0x2F05, 0x01 },
    { 0x2F06, 0x09 },
    { 0x2F07, 0x7A },
    { 0x3071, 0x00 },
    { 0x0828, 0x00 },
    { 0x0800, 0x07 },
    { 0x0801, 0x08 },
    { 0x4010, 0x34 },
    { 0x4015, 0x00 },
    { 0x4016, 0x28 },
    { 0x5265, 0x00 },
    { 0x5266, 0x05 },
    { 0x5267, 0x68 },
    { 0x21BE, 0x00 },
    { 0x21BF, 0x00 },
    { 0x21c4, 0x0f },
    { 0x21c8, 0x32 },
    { 0x21c9, 0x32 },
    { 0x21ca, 0x32 },
    { 0x21cb, 0x32 },
    { 0x21cc, 0x32 },
    { 0x21cd, 0x32 },
    { 0x21ce, 0x32 },
    { 0x21cf, 0x32 },
    { 0x1048, 0x00 },
    { 0x1049, 0x64 },
    { 0x104A, 0x01 },
    { 0x104B, 0x02 },
    { 0x10D0, 0x0b },
    { 0X10D2, 0x00 },
    { 0x10D3, 0x10 },
    { 0x10D4, 0x00 },
    { 0x10D5, 0x9c },
    { 0x2120, 0x00 },
    { 0x2121, 0x01 },
    { 0x2122, 0xd4 },
    { 0x2123, 0xc0 },
    { 0x2124, 0x00 },
    { 0x2125, 0x01 },
    { 0x2126, 0xd4 },
    { 0x2127, 0xc0 },
    { 0x2128, 0x00 },
    { 0x2129, 0x01 },
    { 0x212a, 0xd4 },
    { 0x212b, 0xc0 },
    { 0x212c, 0x00 },
    { 0x212d, 0x01 },
    { 0x212e, 0xd4 },
    { 0x212f, 0xc0 },
    { 0x21e8, 0x04 },
    { 0x21B4, 0x20 },
    { 0x21B5, 0x64 },
    { 0x21B6, 0x00 },
    { 0x21B7, 0x00 },
    { 0x0804, 0x00 },
    { 0x0805, 0x01 },
    { 0x0806, 0x02 },
    { 0x0807, 0x80 },
    { 0x0808, 0x00 },
    { 0x0809, 0x00 },
    { 0x080A, 0x00 },
    { 0x080B, 0xF1 },
    { 0x080C, 0x01 },
    { 0x080D, 0x01 },
    { 0x14A5, 0x00 },
    { 0x1433, 0x00 },
    { 0x1400, 0x00 },
    { 0x14bb, 0x00 },
    { 0x1407, 0x00 },
#else
    { 0x1006, 0x08 },
    { 0x1007, 0x00 },
    { 0x1040, 0x00 },
    { 0x1041, 0x96 },
    { 0x1042, 0x01 },
    { 0x1043, 0x00 },
    { 0x1044, 0x00 },
    { 0x1046, 0x01 },
    { 0x104A, 0x01 },
    { 0x1000, 0x00 },
    { 0x10D3, 0x10 },
    { 0x1448, 0x06 },
    { 0x1449, 0x40 },
    { 0x144A, 0x06 },
    { 0x144B, 0x40 },
    { 0x144C, 0x06 },
    { 0x144D, 0x40 },
    { 0x144E, 0x06 },
    { 0x144F, 0x40 },
    { 0x1450, 0x06 },
    { 0x1451, 0x40 },
    { 0x1452, 0x06 },
    { 0x1453, 0x40 },
    { 0x1454, 0x06 },
    { 0x1455, 0x40 },
    { 0x1456, 0x06 },
    { 0x1457, 0x40 },
    { 0x2203, 0x1E },
    { 0x2C08, 0x01 },
    { 0x3C2B, 0x1B },
    { 0x400E, 0x01 },
    { 0x400F, 0x81 },
    { 0x40D1, 0x00 },
    { 0x40D2, 0x00 },
    { 0x40D3, 0x00 },
    { 0x40DB, 0x3F },
    { 0x40DE, 0x40 },
    { 0x40DF, 0x01 },
    { 0x4134, 0x04 },
    { 0x4135, 0x04 },
    { 0x4136, 0x04 },
    { 0x4137, 0x04 },
    { 0x4138, 0x04 },
    { 0x4139, 0x04 },
    { 0x413A, 0x04 },
    { 0x413B, 0x04 },
    { 0x413C, 0x04 },
    { 0x4146, 0x01 },
    { 0x4147, 0x01 },
    { 0x4148, 0x01 },
    { 0x4149, 0x01 },
    { 0x414A, 0x01 },
    { 0x414B, 0x01 },
    { 0x414C, 0x01 },
    { 0x414D, 0x01 },
    { 0x4158, 0x01 },
    { 0x4159, 0x01 },
    { 0x415A, 0x01 },
    { 0x415B, 0x01 },
    { 0x415C, 0x01 },
    { 0x415D, 0x01 },
    { 0x415E, 0x01 },
    { 0x415F, 0x01 },
    { 0x4590, 0x00 },
    { 0x4591, 0x2E },
    { 0x4684, 0x00 },
    { 0x4685, 0xA0 },
    { 0x4687, 0xA1 },
    { 0x471E, 0x07 },
    { 0x471F, 0xC9 },
    { 0x473A, 0x07 },
    { 0x473B, 0xC9 },
    { 0x4770, 0x00 },
    { 0x4771, 0x00 },
    { 0x4772, 0x1F },
    { 0x4773, 0xFF },
    { 0x4778, 0x06 },
    { 0x4779, 0xA4 },
    { 0x477A, 0x07 },
    { 0x477B, 0xAE },
    { 0x477D, 0xD6 },
    { 0x4788, 0x06 },
    { 0x4789, 0xA4 },
    { 0x478C, 0x1F },
    { 0x478D, 0xFF },
    { 0x478E, 0x00 },
    { 0x478F, 0x00 },
    { 0x4792, 0x00 },
    { 0x4793, 0x00 },
    { 0x4796, 0x00 },
    { 0x4797, 0x00 },
    { 0x479A, 0x00 },
    { 0x479B, 0x00 },
    { 0x479C, 0x1F },
    { 0x479D, 0xFF },
    { 0x479E, 0x00 },
    { 0x479F, 0x00 },
    { 0x47A2, 0x00 },
    { 0x47A3, 0x00 },
    { 0x47A6, 0x00 },
    { 0x47A7, 0x00 },
    { 0x47AA, 0x00 },
    { 0x47AB, 0x00 },
    { 0x47AC, 0x1F },
    { 0x47AD, 0xFF },
    { 0x47AE, 0x00 },
    { 0x47AF, 0x00 },
    { 0x47B2, 0x00 },
    { 0x47B3, 0x00 },
    { 0x47B6, 0x00 },
    { 0x47B7, 0x00 },
    { 0x47BA, 0x00 },
    { 0x47BB, 0x00 },
    { 0x47BC, 0x1F },
    { 0x47BD, 0xFF },
    { 0x47BE, 0x00 },
    { 0x47BF, 0x00 },
    { 0x47C2, 0x00 },
    { 0x47C3, 0x00 },
    { 0x47C6, 0x00 },
    { 0x47C7, 0x00 },
    { 0x47CA, 0x00 },
    { 0x47CB, 0x00 },
    { 0x4834, 0x00 },
    { 0x4835, 0xA0 },
    { 0x4837, 0xA1 },
    { 0x4878, 0x00 },
    { 0x4879, 0xA0 },
    { 0x487B, 0xA1 },
    { 0x48BC, 0x00 },
    { 0x48BD, 0xA0 },
    { 0x48BF, 0xA1 },
    { 0x4954, 0x00 },
    { 0x4955, 0xA0 },
    { 0x4957, 0xA1 },
    { 0x4984, 0x00 },
    { 0x4985, 0xA0 },
    { 0x4987, 0xA1 },
    { 0x49B9, 0x78 },
    { 0x49C3, 0x3C },
    { 0x49C9, 0x76 },
    { 0x49D3, 0x3F },
    { 0x49DC, 0x00 },
    { 0x49DD, 0xA0 },
    { 0x49DF, 0xA1 },
    { 0x49EF, 0x78 },
    { 0x49F9, 0x3C },
    { 0x49FF, 0x78 },
    { 0x4A05, 0x3C },
    { 0x4A0B, 0x76 },
    { 0x4A11, 0x3F },
    { 0x4A1A, 0x00 },
    { 0x4A1B, 0xA0 },
    { 0x4A1D, 0xA1 },
    { 0x4A1F, 0x78 },
    { 0x4A29, 0x3C },
    { 0x4A4A, 0x00 },
    { 0x4A4B, 0xA0 },
    { 0x4A4D, 0xA1 },
    { 0x4A7A, 0x00 },
    { 0x4A7B, 0xA0 },
    { 0x4A7D, 0xA1 },
    { 0x4AEE, 0x00 },
    { 0x4AEF, 0xA0 },
    { 0x4AF1, 0xA1 },
    { 0x4B2E, 0x00 },
    { 0x4B2F, 0xA0 },
    { 0x4B31, 0xA1 },
    { 0x4B5A, 0x00 },
    { 0x4B5B, 0xA0 },
    { 0x4B5D, 0xA1 },
    { 0x4B86, 0x00 },
    { 0x4B87, 0xA0 },
    { 0x4B89, 0xA1 },
    { 0x4B9F, 0x1A },
    { 0x4BAF, 0x1A },
    { 0x4BB7, 0x1A },
    { 0x4BC7, 0x1A },
    { 0x4BCF, 0x1A },
    { 0x4BEE, 0x00 },
    { 0x4BEF, 0xA0 },
    { 0x4BF1, 0xA1 },
    { 0x4BF7, 0x1A },
    { 0x4C01, 0x1A },
    { 0x4C58, 0x00 },
    { 0x4C59, 0xA0 },
    { 0x4C5B, 0xA1 },
    { 0x4C6E, 0x00 },
    { 0x4C6F, 0xA0 },
    { 0x4C71, 0xA1 },
    { 0x4C7A, 0x01 },
    { 0x4C7B, 0x35 },
    { 0x4CF2, 0x07 },
    { 0x4CF3, 0xC9 },
    { 0x4CF8, 0x06 },
    { 0x4CF9, 0x9B },
    { 0x4CFA, 0x07 },
    { 0x4CFB, 0xAE },
    { 0x4CFE, 0x07 },
    { 0x4CFF, 0xC9 },
    { 0x4D04, 0x06 },
    { 0x4D05, 0x98 },
    { 0x4D06, 0x07 },
    { 0x4D07, 0xB1 },
    { 0x4D18, 0x06 },
    { 0x4D19, 0xA4 },
    { 0x4D1A, 0x07 },
    { 0x4D1B, 0x49 },
    { 0x4D1E, 0x07 },
    { 0x4D1F, 0xC9 },
    { 0x4D2A, 0x07 },
    { 0x4D2B, 0xC9 },
    { 0x4D4A, 0x07 },
    { 0x4D4B, 0xC9 },
    { 0x4D50, 0x06 },
    { 0x4D51, 0x9B },
    { 0x4D52, 0x07 },
    { 0x4D53, 0xAE },
    { 0x4D56, 0x07 },
    { 0x4D57, 0xC9 },
    { 0x4D5C, 0x06 },
    { 0x4D5D, 0x98 },
    { 0x4D5E, 0x07 },
    { 0x4D5F, 0xB1 },
    { 0x4D70, 0x06 },
    { 0x4D71, 0xA4 },
    { 0x4D72, 0x07 },
    { 0x4D73, 0x49 },
    { 0x4D78, 0x06 },
    { 0x4D79, 0xA4 },
    { 0x4D7A, 0x07 },
    { 0x4D7B, 0xAE },
    { 0x4D7C, 0x1F },
    { 0x4D7D, 0xFF },
    { 0x4D7E, 0x1F },
    { 0x4D7F, 0xFF },
    { 0x4D80, 0x06 },
    { 0x4D81, 0xA4 },
    { 0x4D82, 0x07 },
    { 0x4D83, 0xAE },
    { 0x4D84, 0x1F },
    { 0x4D85, 0xFF },
    { 0x4D86, 0x1F },
    { 0x4D87, 0xFF },
    { 0x4E39, 0x07 },
    { 0x4E7B, 0x64 },
    { 0x4E8E, 0x0E },
    { 0x4E9C, 0x01 },
    { 0x4EA0, 0x01 },
    { 0x4EA1, 0x03 },
    { 0x4EA5, 0x00 },
    { 0x4EA7, 0x00 },
    { 0x4F05, 0x04 },
    { 0x4F0D, 0x04 },
    { 0x4F15, 0x04 },
    { 0x4F19, 0x01 },
    { 0x4F20, 0x01 },
    { 0x4F66, 0x0F },
    { 0x500F, 0x01 },
    { 0x5225, 0x2F },
    { 0x5227, 0x1E },
    { 0x5231, 0x19 },
    { 0x5245, 0x07 },
    { 0x5252, 0x07 },
    { 0x5253, 0x08 },
    { 0x5254, 0x07 },
    { 0x5255, 0xB4 },
    { 0x5272, 0x04 },
    { 0x5273, 0x2E },
    { 0x5282, 0x04 },
    { 0x5283, 0x2E },
    { 0x5286, 0x00 },
    { 0x5287, 0x5D },
    { 0x1C40, 0x01 },
    { 0x1010, 0x03 },
    { 0x100C, 0x09 },
    { 0x100D, 0x60 },
    { 0x100E, 0x00 },
    { 0x100F, 0x00 },
    { 0x1016, 0x04 },
    { 0x1017, 0xCC },
    { 0x1045, 0x4B },
    { 0x1047, 0x02 },
    { 0x1060, 0x00 },
    { 0x1071, 0x06 },
    { 0x10C2, 0x00 },
    { 0x10C3, 0x0F },
    { 0x10C4, 0x00 },
    { 0x10C5, 0x9D },
    { 0x10D0, 0x0A },
    { 0x10D4, 0x00 },
    { 0x10D5, 0xC5 },
    { 0x201C, 0x00 },
    { 0x201D, 0x00 },
    { 0x201E, 0x00 },
    { 0x2020, 0x01 },
    { 0x2100, 0x08 },
    { 0x2F05, 0x01 },
    { 0x2F06, 0x09 },
    { 0x2F07, 0x7A },
    { 0x3071, 0x00 },
    { 0x0828, 0x01 },
    { 0x0800, 0x04 },
    { 0x0801, 0x44 },
    { 0x4010, 0x40 },
    { 0x4015, 0x00 },
    { 0x4016, 0x06 },
    { 0x5265, 0x00 },
    { 0x5266, 0x0D },
    { 0x5267, 0x38 },
    { 0x5281, 0x00 },
    { 0x5282, 0x04 },
    { 0x5283, 0x2E },
    { 0x21BE, 0x00 },
    { 0x21BF, 0x01 },
    { 0x1048, 0x00 },
    { 0x1049, 0x50 },
    { 0x104B, 0x02 },
    { 0x2108, 0x00 },
    { 0x2109, 0x00 },
    { 0x210A, 0x15 },
    { 0x210B, 0x77 },
    { 0x21E8, 0x04 },
    { 0x21C0, 0x00 },
    { 0x21C2, 0x00 },
    { 0x2120, 0x00 },
    { 0x2121, 0x01 },
    { 0x2122, 0xD5 },
    { 0x2123, 0x38 },
    { 0x2124, 0x00 },
    { 0x2125, 0x01 },
    { 0x2126, 0xD5 },
    { 0x2127, 0x38 },
    { 0x2128, 0x00 },
    { 0x2129, 0x01 },
    { 0x212A, 0xD5 },
    { 0x212B, 0x38 },
    { 0x212C, 0x00 },
    { 0x212D, 0x01 },
    { 0x212E, 0xD5 },
    { 0x212F, 0x38 },
    { 0x2130, 0x00 },
    { 0x2131, 0x01 },
    { 0x2132, 0xD5 },
    { 0x2133, 0x38 },
    { 0x2134, 0x00 },
    { 0x2135, 0x01 },
    { 0x2136, 0xD5 },
    { 0x2137, 0x38 },
    { 0x2138, 0x00 },
    { 0x2139, 0x01 },
    { 0x213A, 0xD5 },
    { 0x213B, 0x38 },
    { 0x213C, 0x00 },
    { 0x213D, 0x01 },
    { 0x213E, 0xD5 },
    { 0x213F, 0x38 },
    { 0x21B4, 0x20 },
    { 0x21B5, 0x64 },
    { 0x21B6, 0x00 },
    { 0x21B7, 0x00 },
    { 0x0804, 0x00 },
    { 0x0805, 0x01 },
    { 0x0806, 0x02 },
    { 0x0807, 0x80 },
    { 0x0808, 0x00 },
    { 0x0809, 0x00 },
    { 0x080A, 0x00 },
    { 0x080B, 0xF1 },
    { 0x080C, 0x00 },
    { 0x080D, 0x00 },
    { 0x14A5, 0x00 },
    { 0x1433, 0x00 },
    { 0x14BB, 0x01 },
    { 0x0824, 0x4E },
    
    //user add
//    { 0x1001, 0x01 },

    { 0x2C0C, 0xA0 },
    { 0x2C0D, 0x00 },
//    { 0x1001, 0x01 },
#endif
    { 0xffff, 0x10 },
};

static struct regList_st max96716a_reglist[] = {
    { 0x0001, 0x01 }, 		// 3Gbps
    { 0x0f00, 0x02 },		// just enable link B
    { 0x0323, 0x26 },		// Port B Set MIPI speed to be 600Mbps(UM330 max 1Gbps/lane)
    { 0x0313, 0x02 },		// Enable MIPICSI output
    { 0xffff, 0x64 },
};

static struct regList_st max96717_orbbec_reglist[] = {
    { 0x02C7, 0x00 },       // 0x18:GPIO3 = 1;	0x00:GPIO3 = 0; Rx板上的TX_PWR_CNTL 0:on 1:off
    { 0x0316, 0x6C },		// Apply filter RAW-12 on pipeline Y,RAW-12:6C ,YUV422 8bit：5E,YUV422 10bit:5F
    { 0x03F1, 0x09 },		// Enable INCK output on MFP4
    { 0x03F0, 0x69 },		// PLL output
    { 0x0570, 0x2C },		// increase MFP4 slew rate

    { 0x0383, 0x80 },       // EXT11 0x80: Select Tunnel mode

    { 0x02D3, 0x00 },		// 0x18:GPIO7 = 1; 	0x00:GPIO7 = 0; MLX75027复位,外部要上拉。
    { 0xffff, 0x64 },		// 100ms
    { 0x02D3, 0x18 },		// 0x18:GPIO7 = 1; 	0x00:GPIO7 = 0; MLX75027复位,外部要上拉。
    { 0xffff, 0x64 },
};

static struct regList_st max96717_lixiang_reglist[] = {
    { 0x02C7, 0x00 },          // 0x18:GPIO3 = 1; 0x00:GPIO3 = 0; LDEN,装上灯板后，此处要设为低，否则会烧灯
    { 0x02D0, 0x00 },          // 0x18:GPIO6 = 1;      0x00:GPIO6 = 0; EN1V2MIX,外部要上拉。
    { 0x0316, 0x6C },          // Apply filter RAW-12 on pipeline Y,RAW-12:6C ,YUV422 8bit：5E,YUV422 10bit:5F
    { 0x03F1, 0x09 },          // Enable INCK output on MFP4
    { 0x03F0, 0x69 },          // PLL output
    { 0x0570, 0x2C },          // increase MFP4 slew rate

    { 0x0383, 0x80 },

    { 0x02CD, 0x00 },		// 0x18:GPIO5 = 1; 	0x00:GPIO6 = 0; MLX75027复位,外部要上拉。
    { 0xffff, 0x64 },		// 100ms
    { 0x02CD, 0x18 },		// 0x18:GPIO5 = 1; 	0x00:GPIO6 = 0; MLX75027复位,外部要上拉。
    { 0xffff, 0x64 },
};

static struct regList_st max96717_setup[] = {
    { 0x02C7, 0x18 },   // 0x18:GPIO3 = 1;	0x00:GPIO3 = 0; LDEN,装上灯板后，此处要设为低，否则会烧灯
    { 0x02D0, 0x18 },		// 0x18:GPIO6 = 1; 	0x00:GPIO6 = 0; EN1V2MIX,外部要上拉。
    { 0xffff, 0xC8 },
};

int mlx75027_max96717_max96716a_sensor_initialize()
{
    int rtn = 0;
    for(int i = 0; i < sizeof(sensor_reglist)/sizeof(struct regList_st); i++){

        rtn |= sensor_write_reg(sensor_reglist[i].reg, sensor_reglist[i].val);
    }
    return rtn;
}

int mlx75027_max96717_max96716a_max96716a_initialize()
{
    int rtn = 0;
    for (int i = 0; i < sizeof(max96716a_reglist) / sizeof(struct regList_st); i++) {

        rtn |= max96176a_write_reg(max96716a_reglist[i].reg, max96716a_reglist[i].val);
    }
    return rtn;
}

int mlx75027_max96717_max96716a_orbbec_max96717_initialize()
{
    int rtn = 0;
    for (int i = 0; i < sizeof(max96717_orbbec_reglist) / sizeof(struct regList_st); i++) {

        rtn |= max96717_write_reg(max96717_orbbec_reglist[i].reg, max96717_orbbec_reglist[i].val);
    }
    return rtn;
}

int mlx75027_max96717_max96716a_lixiang_max96717_initialize()
{
    int rtn = 0;
    for (int i = 0; i < sizeof(max96717_lixiang_reglist) / sizeof(struct regList_st); i++) {

        rtn |= max96717_write_reg(max96717_lixiang_reglist[i].reg, max96717_lixiang_reglist[i].val);
    }
    return rtn;
}

int mlx75027_max96717_max96716a_max96717_setup()
{
    int rtn = 0;
    for (int i = 0; i < sizeof(max96717_setup) / sizeof(struct regList_st); i++) {

        rtn |= max96717_write_reg(max96717_setup[i].reg, max96717_setup[i].val);
    }
    return rtn;
}

int mlx75027_max96717_max96716a_shadow_register(bool enable)
{
    int rtn = 0;
    if(enable)
        rtn = sensor_write_reg(0x0102, 0x01);
    else
        rtn = sensor_write_reg(0x0102, 0x00);
    return rtn;
}

int mlx75027_max96717_max96716a_get_sensor_id(uint16_t *id)
{
    int rtn = 0;
    uint8_t max96176a_value = 0, sensor_value = 0, max96717_value = 0, tmp112_value = 0;
	static int isConfig = 0, sensor_id = 0;
    if (!isConfig) 
    {
        isConfig = 1;
        dothin_config();

        rtn = mlx75027_max96717_max96716a_max96716a_initialize();
        rtn = mlx75027_max96717_max96716a_lixiang_max96717_initialize();
        rtn = mlx75027_max96717_max96716a_orbbec_max96717_initialize();

        rtn |= i2c_reg_read(MAX96716A_ADDR, 0x00, 1, &max96176a_value, 1);
        rtn |= i2c_reg_read(SENSOR_ADDR, 0x0308, 2, &sensor_value, 1);
        sensor_value = (sensor_value >> 3);
        ALOGE("mlx75027_max96717_max96716a_get_sensor_id: %d \r\n", sensor_value);
        rtn |= i2c_reg_read(MAX96717_ADDR, 0x00, 1, &max96717_value, 1);

        if (rtn < 0) {
            sensor_id = 0xFFFF;
        }
        else if (sensor_value == 27) {
            sensor_id = mlx75027_max96717_max96716a_sensor_id;
        }
        else {
            sensor_id = 0xFFFF;
        }
//        mlx75027_sensor_id();
        rtn = i2c_reg_read(TMP112_ADDR, 0x00, 1, &tmp112_value, 2);
        if (rtn) {
            module_type = 1;
        }
        else {
            module_type = 2;
        }
        vcsel_driver_type = module_type;
        rtn = 0;
        ALOGE("get sensor_id:0x%x module_type:%d !\n", sensor_id, module_type);
        printf("get sensor_id:0x%x module_type:%d !\n", sensor_id, module_type);
    }
    *id = sensor_id;
    return rtn;
}

int mlx75027_max96717_max96716a_metadata_output(uint8_t mode)
{
    int rtn = 0;
    if (mode == 0) { // no metadata lines enabled
        rtn |= sensor_write_reg(0x3C18, 0x00);
    }
    else if (mode == 1) { // first metadata line (line #1) enabled
        rtn |= sensor_write_reg(0x3C18, 0x01);
    }
    else if (mode == 2) { // first & second metadata lines (line #1 and line #2) enabled default.
        rtn |= sensor_write_reg(0x3C18, 0x02);
    }

    return rtn;
}

int mlx75027_max96717_max96716a_metadata_length(uint16_t length)
{
    int rtn = 0;
    uint8_t metadata_length_h = (length >> 3) & 0x00ff;
    uint8_t metadata_length_l = (length << 5) & 0x00ff;
    rtn = sensor_write_reg(0x2C0C, metadata_length_h);
    rtn |= sensor_write_reg(0x2C0D, metadata_length_l);
    return rtn;
}

int mlx75027_max96717_max96716a_set_user_ID(uint8_t value)
{
    return sensor_write_reg(0x0824, value);
}

int mlx75027_max96717_max96716a_set_hmax(uint16_t hmax)
{
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

    return rtn;
}									
int mlx75027_max96717_max96716a_set_trigger_mode(uint8_t mode) // should stop stream first
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

int mlx75027_max96717_max96716a_set_stream_mode() // should stop stream first
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

int mlx75027_max96717_max96716a_get_data_output_mode(uint8_t *mode)
{
    uint8_t value = 0;
    int rtn = sensor_read_reg(0x0828, &value);
    *mode = value&0xff;
    return rtn;
}

// be careful that the resolution of A&B mode will change to 1280*480
int mlx75027_max96717_max96716a_set_data_output_mode(uint8_t mode) // should stop streaming before change data output mode
{
    // mode = 0: A-B, 1: A+B, 2: A, 3: B, 4: A&B
	if (mode > 4) {
		ALOGE("mode=%d", mode);
		return -HW_ERR_INVALID;
	}
    else if(mode == 4){
        HMAX = 0x750;// 0x0514; 0x744=1860
        mlx75027_max96717_max96716a_set_hmax(HMAX);
        mlx75027_max96717_max96716a_metadata_length(1280);
        mlx75027_max96717_max96716a_set_phase_idle_time(50);
    }
    else if(mode < 4){
        HMAX = 0x750;// 0x02B6;//694
        mlx75027_max96717_max96716a_set_hmax(HMAX);
        mlx75027_max96717_max96716a_metadata_length(640);
        mlx75027_max96717_max96716a_set_phase_idle_time(50);
    } 
    int rtn = sensor_write_reg(0x0828, mode);
	
    return rtn;
}

int mlx75027_max96717_max96716a_get_modulation_frequency(uint16_t *modFreq)
{
    int rtn = 0;

    if (use_trigger_mode == 0) 
    {
        uint8_t divselpre = 0;
        uint8_t divsel = 0;
        rtn = sensor_read_reg(0x21BE, &divselpre);
        rtn |= sensor_read_reg(0x21BF, &divsel);

        uint8_t fmod_H, fmod_L;
        rtn |= sensor_read_reg(0x1048, &fmod_H);
        rtn |= sensor_read_reg(0x1049, &fmod_L);
        uint16_t fmod = (fmod_H << 8) + fmod_L;

        *modFreq = (uint16_t)(fmod / pow(2, (divselpre + divsel)));
    }
    else if (use_trigger_mode == 1)
    {
        *modFreq = g_u16ModFreqParam;
        if (g_u16ModFreqParam == 0)
            rtn = -1;
    }

    return rtn;
}

int mlx75027_max96717_max96716a_set_modulation_frequency(uint16_t modFreqParam)
{
    int rtn = 0;
    uint16_t modFreq = 0;

    if (use_trigger_mode == 0)//single freq
    {
        modFreq = modFreqParam & 0xff;
        if (modFreq > 100 || modFreq < 4)
            return -HW_ERR_INVALID;

        rtn = mlx75027_max96717_max96716a_shadow_register(true);
        uint8_t divselpre = 0;
        uint8_t divsel = 0;
        if ((modFreq <= 100 && modFreq >= 75) || (modFreq <= 50 && modFreq >= 38) || (modFreq <= 20 && modFreq >= 19)) {
            divselpre = 0x00;
            rtn |= sensor_write_reg(0x21BE, divselpre);
        }
        else if ((modFreq <= 74 && modFreq >= 51) || (modFreq <= 37 && modFreq >= 21) || (modFreq <= 18 && modFreq >= 10)) {
            divselpre = 0x01;
            rtn |= sensor_write_reg(0x21BE, divselpre);
        }
        else if (modFreq <= 9 && modFreq >= 5) {
            divselpre = 0x02;
            rtn |= sensor_write_reg(0x21BE, divselpre);
        }
        else if (modFreq == 4) {
            divselpre = 0x03;
            rtn |= sensor_write_reg(0x21BE, divselpre);
        }

        if (modFreq <= 100 && modFreq >= 51) {
            divsel = 0x00;
            rtn |= sensor_write_reg(0x21BF, divsel);
        }
        else if (modFreq <= 50 && modFreq >= 21) {
            divsel = 0x01;
            rtn |= sensor_write_reg(0x21BF, divsel);
        }
        else if (modFreq <= 20 && modFreq >= 4) {
            divsel = 0x02;
            rtn |= sensor_write_reg(0x21BF, divsel);
        }

        uint16_t fmod = (uint16_t)(pow(2, (divselpre + divsel))*modFreq);
        //qDebug() << "divselpre, divsel, fmod, modFreq" << divselpre << divsel << fmod << modFreq;
        rtn |= sensor_write_reg(0x1048, (fmod >> 8) & 0xFF);
        rtn |= sensor_write_reg(0x1049, (fmod & 0xFF));

        if (fmod * 8 < 900 && fmod * 8 >= 500)
            rtn |= sensor_write_reg(0x104B, 0x02);
        else if (fmod * 8 <= 1200 && fmod * 8 >= 900)
            rtn |= sensor_write_reg(0x104B, 0x00);

        rtn |= mlx75027_max96717_max96716a_shadow_register(false);
    }
    else if (use_trigger_mode == 1) 
    {
        rtn |= mcu_write_reg(OP_SET_FREQ1, ((modFreqParam >> 8) & 0xFF));
        rtn |= mcu_write_reg(OP_SET_FREQ2, (modFreqParam & 0xFF));
        if (!rtn)
        {
            g_u16ModFreqParam = modFreqParam;
        }
    }
    return rtn;
}

// param uint: us
int mlx75027_max96717_max96716a_get_frame_startup_time(uint16_t *startupTime)
{
    uint8_t byte0, byte1;
    int rtn = sensor_read_reg(0x21D4, &byte1);
    rtn |= sensor_read_reg(0x21D5, &byte0);
    uint16_t value = (byte1 << 8) + byte0;
    *startupTime = value*HMAX/120;

    return rtn;
}														  
int mlx75027_max96717_max96716a_set_frame_startup_time(uint16_t startupTime)
{
    uint16_t value = (startupTime*120)/HMAX;
    int rtn = sensor_write_reg(0x21D4, (value >> 8)&0xFF);
    rtn |= sensor_write_reg(0x21D5, (value & 0xFF));

    return rtn;
}

// param uint: us
int mlx75027_max96717_max96716a_set_frame_time(uint32_t frameTime)
{
    uint32_t value = (frameTime*120)/HMAX;
    int rtn = sensor_write_reg(0x2108, (value >> 24)&0xFF);
    rtn |= sensor_write_reg(0x2109, (value >> 16)&0xFF);
    rtn |= sensor_write_reg(0x210A, (value >> 8)&0xFF);
    rtn |= sensor_write_reg(0x210B, (value & 0xFF));

    return rtn;
}

int mlx75027_max96717_max96716a_get_fps(uint8_t *fps)
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
		//DDEBUG("mlx75027_max96717_max96716a_get_fps %d   value = %x",*fps, value);
	}
	else {
		return -HW_ERR_NO_SUPPORT;
	}
    return rtn;
}

int mlx75027_max96717_max96716a_set_fps(uint8_t fps)
{
    uint32_t frameTime = fps ? 1000000/fps : fps; // us
    int rtn = mlx75027_max96717_max96716a_set_frame_time(frameTime);
    return rtn;
}

int mlx75027_max96717_max96716a_set_phase_count(uint8_t phaseCount)
{
    int rtn = 0;

    if (phaseCount == 2 || phaseCount == 4)// default is 4
    {
        rtn |= sensor_write_reg(0x21B4, 0x20);
        rtn |= sensor_write_reg(0x21B5, 0x64);
        rtn |= sensor_write_reg(0x21B6, 0x00);
        rtn |= sensor_write_reg(0x21B7, 0x00);
        rtn |= sensor_write_reg(0x0824, 0x70);
        rtn |= sensor_write_reg(0x21E8, phaseCount);
    }
    else if (phaseCount == 8)
    {
        rtn |= sensor_write_reg(0x21B4, 0x10);
        rtn |= sensor_write_reg(0x21B5, 0x32);
        rtn |= sensor_write_reg(0x21B6, 0x54);
        rtn |= sensor_write_reg(0x21B7, 0x76);
        rtn |= sensor_write_reg(0x0824, 0x80);
        rtn |= sensor_write_reg(0x21E8, phaseCount);
    }
    else
    {
        return -HW_ERR_INVALID;
    }

    return rtn;
}

int mlx75027_max96717_max96716a_get_phase_count(uint8_t *phaseCount)
{
    int rtn = 0;
    uint8_t byte0;

    rtn = sensor_read_reg(0x21E8, &byte0);
    *phaseCount = byte0;

    return rtn;
}

int mlx75027_sensor_id(uint64_t *chip_id)
{
    int rtn = 0;
    uint8_t byte[8] = {0};

    rtn |= sensor_read_reg(0x0000, &byte[6]);//SensorID[55:48]
    rtn |= sensor_read_reg(0x0001, &byte[5]);//SensorID[47:40]
    rtn |= sensor_read_reg(0x0002, &byte[4]);//SensorID[39:32]
    rtn |= sensor_read_reg(0x0003, &byte[3]);//SensorID[31:24]
    rtn |= sensor_read_reg(0x0016, &byte[2]);//SensorID[23:16]
    rtn |= sensor_read_reg(0x0017, &byte[1]);//SensorID[15:8]
    rtn |= sensor_read_reg(0x0018, &byte[0]);//SensorID[7:0]

    uint64_t SensorID = *(uint64_t *)byte;
    *chip_id = SensorID;

//    ALOGE("get_sensor_id:0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x !", byte[6], byte[5], byte[4], byte[3], byte[2], byte[1], byte[0]);
    ALOGE("get_sensor_id:0x%llX !", SensorID);
//    ALOGE("get_sensor_id:%s !", byte);
    return rtn;
}

int mlx75027_max96717_max96716a_set_phase_pretime(uint16_t preTime)
{
    if(preTime > 2000 || preTime < 1)  // value 0 is not allowed, 2000 is user defined
        return -HW_ERR_INVALID;
    int rtn = mlx75027_max96717_max96716a_shadow_register(true);
    rtn |= sensor_write_reg(0x4015, (preTime >> 8)&0xFF);
    rtn |= sensor_write_reg(0x4016, (preTime & 0xFF));
    rtn |= mlx75027_max96717_max96716a_shadow_register(false);

    return rtn;
}

int mlx75027_max96717_max96716a_get_integration_time(uint16_t *integrationTime)
{
    uint8_t byte0, byte1, byte2, byte3;
    int rtn = sensor_read_reg(0x2120, &byte3);
    rtn |= sensor_read_reg(0x2121, &byte2);
    rtn |= sensor_read_reg(0x2122, &byte1);
    rtn |= sensor_read_reg(0x2123, &byte0);

    uint32_t value = (byte3 << 24) + (byte2 << 16) + (byte1 << 8) + byte0;
    *integrationTime = (uint16_t)(value/120); // may slightly different from what we set because of floor function
	//printf("byte 3 2 1 0:  %x %x %x %x", byte3, byte2, byte1, byte0);
	//printf("repeat %d, ", repeat);
    return rtn;
}

int mlx75027_max96717_max96716a_set_integration_time(unsigned short integrationTime)
{
    if(integrationTime > 1000)
        return -HW_ERR_INVALID;

	unsigned short max_int_time = 1000;  // us
	unsigned short new_int_time = 0;
    if(integrationTime > max_int_time)
    {
        new_int_time = max_int_time;
    }
    else
    {
        new_int_time = integrationTime;
    }
    int value = new_int_time * 120;// (int)(floor((new_int_time / repeat)*120.0f / HMAX)*HMAX);

    int rtn = mlx75027_max96717_max96716a_shadow_register(true);
    for(uint8_t phase = 0; phase < 8; phase++){
        // set integration time
        rtn |= sensor_write_reg(0x2120 + phase*4, (value >> 24)&0xFF);
        rtn |= sensor_write_reg(0x2121 + phase*4, (value >> 16)&0xFF);
        rtn |= sensor_write_reg(0x2122 + phase*4, (value >>  8)&0xFF);
        rtn |= sensor_write_reg(0x2123 + phase*4, (value & 0xFF));
    }

    rtn |= mlx75027_max96717_max96716a_shadow_register(false);

    return rtn;
}

//unit:us
int mlx75027_max96717_max96716a_set_phase_idle_time(uint16_t time)
{
    uint8_t value = 0;

    value = (uint8_t)(time * 120 / HMAX);//HMAX:1860
    if (value < 5)  // value out side [0x05 - 0xFF] are prohibited
        return -HW_ERR_INVALID;
    int rtn = mlx75027_max96717_max96716a_shadow_register(true);
    for (uint8_t phase = 0; phase < 8; phase++) {

        rtn |= sensor_write_reg(0x21C8 + phase, value);
    }
    rtn |= mlx75027_max96717_max96716a_shadow_register(false);

    return rtn;
}

/*
mode = 0: no binning (= VGA resolution, 640x480 pixels)  // default mode
mode = 1: 2x2 binning (= QVGA resolution, 320x240 pixels)
mode = 2: 4x4 binning (= QQVGA resolution, 160x120 pixels)
mode = 3: 8x8 binning (= QQQVGA resolution, 80x60 pixels)
*/
int mlx75027_max96717_max96716a_set_pixel_binning(uint8_t mode)
{
    return sensor_write_reg(0x14A5, mode);
}

int mlx75027_max96717_max96716a_get_pixel_binning(uint8_t *mode)
{
	return sensor_read_reg(0x14A5, *mode);
}

int mlx75027_max96717_max96716a_pixelROI(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
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

int mlx75027_max96717_max96716a_get_img_mirror_flip(uint8_t *mode)
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

int mlx75027_max96717_max96716a_set_img_mirror_flip(uint8_t mode)
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

int mlx75027_max96717_max96716a_get_sensor_temperature(float *temp)
{
    uint8_t value;
    int rtn = sensor_read_reg(0x1403, &value);

    *temp = (float)(value&0xff) - 40;
    //qDebug() << "value temp " << value << *temp;
	//DDEBUG("   value = %x",  value);
    return rtn;
}

int mlx75027_max96717_max96716a_pixel_statistics(bool enable, uint8_t mode)
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
int mlx75027_max96717_max96716a_set_pixel_lower_limit(uint16_t value)
{
    int rtn = 0;
    for(uint8_t phase = 0; phase < 8; phase++){

        rtn |= sensor_write_reg(0x1434 + phase*2, (value >>  8)&0xFF);
        rtn |= sensor_write_reg(0x1435 + phase*2, (value & 0xFF));
    }

    return rtn;
}

// The maximum threshold for each tap.
int mlx75027_max96717_max96716a_set_pixel_upper_limit(uint16_t value)
{
    int rtn = 0;
    for(uint8_t phase = 0; phase < 8; phase++){

        rtn |= sensor_write_reg(0x1448 + phase*2, (value >>  8)&0xFF);
        rtn |= sensor_write_reg(0x1449 + phase*2, (value & 0xFF));
    }

    return rtn;
}

int mlx75027_max96717_max96716a_get_pixel_error_count_low(uint8_t phase, uint32_t *value)
{
    uint8_t Byte1, Byte2, Byte3;

    int rtn = sensor_read_reg(0x145D + phase*4, &Byte3);
    rtn |= sensor_read_reg(0x145E + phase*4, &Byte2);
    rtn |= sensor_read_reg(0x145F + phase*4, &Byte1);

    *value = Byte1 + (Byte2 << 8) + (Byte3 <<16);

    return rtn;
}

int mlx75027_max96717_max96716a_get_pixel_error_count_high(uint8_t phase, uint32_t *value)
{
    uint8_t Byte1, Byte2, Byte3;

    int rtn = sensor_read_reg(0x1481 + phase*4, &Byte3);
    rtn |= sensor_read_reg(0x1482 + phase*4, &Byte2);
    rtn |= sensor_read_reg(0x1483 + phase*4, &Byte1);

    *value = Byte1 + (Byte2 << 8) + (Byte3 <<16);

    return rtn;
}

int mlx75027_max96717_max96716a_test_pattern(uint8_t mode)
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

int mlx75027_max96717_max96716a_get_sensor_info(struct sensor_info_t *info) {
    info->embedded_data_size = 640 * 3;
	info->vcsel_num = vcsel_number;
	info->vcsel_driver_id = vcsel_driver_type;
	info->sensor_id = mlx75027_max96717_max96716a_sensor_id;
	return 0;
}

int mlx75027_max96717_max96716a_illum_duty_cycle_adjust(uint8_t mode, uint8_t value)
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
int mlx75027_max96717_max96716a_get_illum_duty_cycle_list(uint8_t mod_freq, float *duty_cycle_list)
{
    float duty = 0;
    float cycle = 1000.0f/mod_freq; // period ns
    for(int step = -15; step <= 15; step++){
/*		if(vcsel_driver_type == DAC5574){
			duty = (0.5f + (step*0.5f - 2.0f)/cycle)*100; // 0.5ns/step, 2.0 is a const missing caused by IC-HG vcsel driver
		}
		else if(vcsel_driver_type == QUAD_CXA4016){
			duty = (0.5f + (step*0.5f) / cycle) * 100;
		}*/
        duty = (0.5f + (step*0.5f) / cycle) * 100;
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

int mlx75027_max96717_max96716a_get_illum_duty_cycle(uint16_t *duty)
{
    int rtn = 0;
    uint8_t mode = 0, value = 0;

    if (use_trigger_mode == 0)
    {
        rtn = sensor_read_reg(0x4E9E, &mode);
        if (mode == 0)
            *duty = 15; // 50%
        else if (mode == 1) {
            rtn |= sensor_read_reg(0x21B9, &value);
            *duty = 15 + value; // > 50%
        }
        else if (mode == 2) {
            rtn |= sensor_read_reg(0x21B9, &value);
            *duty = 15 - value; // < 50%
        }
    }
    else if (use_trigger_mode == 1)
    {
        *duty = g_u16DutyParam;
        if (g_u16DutyParam == 0xffff)
            rtn = -1;
    }

    return rtn;
}

int mlx75027_max96717_max96716a_set_illum_duty_cycle(uint16_t dutyParam)
{
    int rtn = 0;
    uint8_t value = 0;
    uint32_t duty = 0;

    if (use_trigger_mode == 0) //single freq
    {
        duty = dutyParam & 0xff;
        if (duty == 15) {
            rtn = mlx75027_max96717_max96716a_illum_duty_cycle_adjust(0, 0);
        }
        else if (duty > 15) {
            value = duty - 15;
            rtn = mlx75027_max96717_max96716a_illum_duty_cycle_adjust(2, 0);
            rtn |= mlx75027_max96717_max96716a_illum_duty_cycle_adjust(1, value);
        }
        else if (duty < 15) {
            value = 15 - duty;
            rtn = mlx75027_max96717_max96716a_illum_duty_cycle_adjust(1, 0);
            rtn |= mlx75027_max96717_max96716a_illum_duty_cycle_adjust(2, value);
        }
    }
    else if (use_trigger_mode == 1) {
        rtn |= mcu_write_reg(OP_SET_DUTY1, ((dutyParam >> 8) & 0xFF));
        rtn |= mcu_write_reg(OP_SET_DUTY2, (dutyParam & 0xFF));
        if (!rtn)
            g_u16DutyParam = dutyParam;
    }

    return rtn;
}

int mlx75027_max96717_max96716a_illum_signal(uint8_t mode)
{
    int rtn = 0;
    if(mode == 0){ // subLVDS mode (LEDP positive, LEDN negative) default.
        rtn |= sensor_write_reg(0x10E2, 0x01);
    }
    else // CMOS mode (LEDP = LEDN)
        rtn |= sensor_write_reg(0x10E2, 0x00);

    return rtn;
}

int mlx75027_max96717_max96716a_video_streaming(bool enable)
{
    int rtn = 0;
    if (enable)
    {
//        rtn |= mlx75027_max96717_max96716a_set_sensor_analog_delay(2000);
        rtn |= sensor_write_reg(0x1001, 0x01);
#if 0
        int rtn = 0;
        uint8_t read_reg = 0;
        for (int i = 0; i < sizeof(read_sensor_reglist) / sizeof(struct regList_st); i++) {
            rtn |= sensor_read_reg(read_sensor_reglist[i].reg, &read_reg);
            printf("reg:0x%04X data:0x%02X\n", read_sensor_reglist[i].reg, read_reg);
        }
#endif
        if (use_trigger_mode) {
            rtn |= mcu_write_reg(OP_SET_START, 1);
            mcu_trig_sensor_flag = 1;
        }
    }
    else
    {
        rtn |= sensor_write_reg(0x1001, 0x00);
        if (use_trigger_mode) {
            rtn |= mcu_write_reg(OP_SET_STOP, 0);
            mcu_trig_sensor_flag = 0;
        }
    }

    return rtn;
}

int gt24c128e_eeprom_block_write(uint16_t offset, uint8_t *buf, uint16_t size)
{
    int ret = 0;
    uint16_t blocksize = GT24P128E_EEPROM_PAGE_SIZE;
    uint16_t cnt = 0;
    uint32_t addr = offset;
    uint8_t  *buf_ptr_r = buf;
    uint16_t prog_time = size / blocksize;
    uint16_t lastsize = size % blocksize;
    uint32_t addr_offset = addr % blocksize;
    uint8_t  read_out_data[GT24P128E_EEPROM_PAGE_SIZE] = { 0 };

    if (0 == addr_offset)  // write at page head
    {
        for (cnt = 0; cnt < prog_time; cnt++)
        {
            ret = eeprom_write(GT24P128E_EEPROM_I2C_ADDR, addr, 2, buf_ptr_r, blocksize);
            addr += blocksize;
            buf_ptr_r += blocksize;
        }

        if (lastsize != 0)
        {
            ret = eeprom_write(GT24P128E_EEPROM_I2C_ADDR, addr, 2, buf_ptr_r, lastsize);
        }
    }
    else  // write not at page head
    {
        ret = gt24p256b_eeprom_block_read((addr - addr_offset), read_out_data, addr_offset);  // read the head data of a page
        if ((blocksize - addr_offset) > size) {  // write in a page
            ret = eeprom_write(GT24P128E_EEPROM_I2C_ADDR, (addr - addr_offset), 2, read_out_data, addr_offset);
            ret = eeprom_write(GT24P128E_EEPROM_I2C_ADDR, addr, 2, buf_ptr_r, size);
        }
        else  // write over a page
        {   // write not at head of page
            ret = eeprom_write(GT24P128E_EEPROM_I2C_ADDR, (addr - addr_offset), 2, read_out_data, addr_offset);
            ret = eeprom_write(GT24P128E_EEPROM_I2C_ADDR, addr, 2, buf_ptr_r, (blocksize - addr_offset));
            addr += (blocksize - addr_offset);
            buf_ptr_r += (blocksize - addr_offset);
            // write at head of page
            prog_time = (size - blocksize + addr_offset) / blocksize;
            lastsize = (size - blocksize + addr_offset) % blocksize;
            for (cnt = 0; cnt < prog_time; cnt++)
            {
                ret = eeprom_write(GT24P128E_EEPROM_I2C_ADDR, addr, 2, buf_ptr_r, blocksize);
                addr += blocksize;
                buf_ptr_r += blocksize;
            }

            if (lastsize != 0)
            {
                ret = eeprom_write(GT24P128E_EEPROM_I2C_ADDR, addr, 2, buf_ptr_r, lastsize);
            }
        }
    }

    return ret;
}
int gt24c128e_eeprom_block_read(uint16_t offset, uint8_t *buf, uint32_t size)
{
    int ret = 0;
    uint16_t cnt;
    uint16_t blocksize = GT24P128E_EEPROM_PAGE_SIZE;
    uint16_t prog_time = size / blocksize;
    uint16_t lastsize = size % blocksize;
    uint8_t * buf_ptr_r = buf;
    uint32_t addr = offset;

    for (cnt = 0; cnt < prog_time; cnt++)
    {
        ret = eeprom_read(GT24P128E_EEPROM_I2C_ADDR, addr, 2, buf_ptr_r, blocksize);
        addr += blocksize;
        buf_ptr_r += blocksize;
    }

    if (lastsize != 0)
    {
        ret = eeprom_read(GT24P128E_EEPROM_I2C_ADDR, addr, 2, buf_ptr_r, lastsize);
    }

    return ret;
}

int mlx75027_max96717_max96716a_set_frequency_mode(uint8_t mode)
{
    int ret = 0;

    if (mode > 2) {
        return -HW_ERR_INVALID;
    }

    if (module_type == 1 && mode == 2) { //理想模组，不知道硬件上有没有连接trig，所以不支持双频
        return -HW_ERR_INVALID;
    }

    switch (mode) {
    case SINGLE_FREQ:
        ret |= mlx75027_max96717_max96716a_set_stream_mode();
        ret |= mlx75027_max96717_max96716a_set_fps(30);//PC端计算深度，帧率受限
        use_trigger_mode = 0;
        break;
    case DUAL_FREQ:
        ret |= mlx75027_max96717_max96716a_set_trigger_mode(0);
        ret |= mlx75027_max96717_max96716a_set_fps(0);//PC端计算深度，帧率受限
        uint8_t data_mode = 0;// data_mode = 0: A-B, 1: A+B, 2: A, 3: B, 4: A&B
        mlx75027_max96717_max96716a_get_data_output_mode(&data_mode);
        if (data_mode == 4) //A&B
        {
            ret |= mcu_write_reg(OP_SET_FPS, 10);
            ret |= mcu_write_reg(OP_SET_DELAY, 44000);
        }
        else
        {
            ret |= mcu_write_reg(OP_SET_FPS, 10);
            ret |= mcu_write_reg(OP_SET_DELAY, 44000);
        }
        use_trigger_mode = 1;
        break;

    default:
        break;
    }
    return ret;
}

int mlx75027_max96717_max96716a_get_frequency_mode(uint8_t *mode)
{
    if (use_trigger_mode == 0)
        *mode = SINGLE_FREQ;
    else if (use_trigger_mode == 1)
        *mode = DUAL_FREQ;
    return 0;
}

int mlx75027_max96717_max96716a_set_binning_mode(uint8_t mode)
{
    return 0;
}

int mlx75027_max96717_max96716a_get_binning_mode(uint8_t *mode)
{
   *mode = 0;

    return 0;
}
//unit:ps step:20ps
int mlx75027_max96717_max96716a_set_sensor_analog_delay(uint32_t time)
{
    int rtn = 0;
    uint16_t modFreq = 0;
    uint16_t freq1 = 0, freq2 = 0;
    uint32_t N = 0, adelay_coarse = 0, adelay_coarse_step = 0, adelay_fine = 0, adelay_sfine = 0;

    if (NULL == tof_sensor.get_modulation_frequency)
    {
        ALOGE("OBC_TOF_PROJECT_TEST set_sensor_analog_delay nullptr");
        return -HW_ERR_NULL;
    }
    rtn = tof_sensor.get_modulation_frequency(&modFreq);

    if (use_trigger_mode == 1)//dual freq
    {
        freq1 = (modFreq >> 8 ) & 0xff;
        freq2 = modFreq & 0xff;
        modFreq = (freq1 > freq2) ? freq2 : freq1;
        if (modFreq < 50)
        {
            ALOGE("dual freq(freq1:%dMHz freq2:%dMHz) set sensor delay(%dps) error !\r\n", freq1, freq2, time);
            return -1;
        }
    }

    if (modFreq >= 4 && modFreq <= 20)
    {
        N = 32;
    }
    else if (modFreq >= 21 && modFreq <= 50)
    {
        N = 16;
    }
    else if (modFreq >= 51 && modFreq <= 100)
    {
        N = 8;
    }
    else
    {
        ALOGE("set_sensor_analog_delay modulation_frequency error !\r\n");
        return -1;
    }

    if (modFreq * N != 0)
        adelay_coarse_step = 1000000 / (modFreq * N);//unit:ps
    ALOGE("adelay_coarse_step:%d ps !\r\n", adelay_coarse_step);
    adelay_coarse = time / adelay_coarse_step;
    if (adelay_coarse > (N - 1))
    {
        ALOGE("adelay_coarse error:%d !\r\n", adelay_coarse);
    }
    rtn |= sensor_write_reg(0x201C, adelay_coarse & 0xff);

    adelay_fine = (time % adelay_coarse_step) / 75;
    if (adelay_fine >= 69)
    {
        ALOGE("adelay_fine error:%d !\r\n", adelay_fine);
    }
    rtn |= sensor_write_reg(0x201D, adelay_fine & 0xff);

    adelay_sfine = ((time % adelay_coarse_step) % 75) / 20;
    if (adelay_sfine >= 4)
    {
        ALOGE("adelay_sfine error:%d !\r\n", adelay_sfine);
    }
    rtn |= sensor_write_reg(0x201E, adelay_sfine & 0xff);

    ALOGE("time: %d adelay_coarse:%d adelay_fine:%d adelay_sfine:%d !\r\n", time, adelay_coarse, adelay_fine, adelay_sfine);
    return rtn;
}

//unit:ps
int mlx75027_max96717_max96716a_get_sensor_analog_delay(uint32_t *time)
{
    int rtn = 0;
    uint16_t modFreq = 0;
    uint16_t freq1 = 0, freq2 = 0;
    uint32_t N = 0, adelay_coarse = 0, adelay_coarse_step = 0, adelay_fine = 0, adelay_sfine = 0;

    if (NULL == tof_sensor.get_modulation_frequency)
    {
        ALOGE("OBC_TOF_PROJECT_TEST set_sensor_analog_delay nullptr");
        return -HW_ERR_NULL;
    }
    rtn = tof_sensor.get_modulation_frequency(&modFreq);
    ALOGE("set_sensor_analog_delay get_modulation_frequency:%dMHz \r\n", modFreq);

    if (use_trigger_mode == 1)//dual freq
    {
        freq1 = (modFreq & 0xff00) >> 8;
        freq2 = modFreq & 0xff;
        modFreq = (freq1 > freq2) ? freq2 : freq1;
        if (modFreq < 50)
        {
            ALOGE("dual freq(freq1:%dMHz freq2:%dMHz) get sensor delay error !\r\n", freq1, freq2);
            return -1;
        }
    }

    if (modFreq >= 4 && modFreq <= 20)
    {
        N = 32;
    }
    else if (modFreq >= 21 && modFreq <= 50)
    {
        N = 16;
    }
    else if (modFreq >= 51 && modFreq <= 100)
    {
        N = 8;
    }
    else
    {
        ALOGE("set_sensor_analog_delay modulation_frequency error !\r\n");
        return -1;
    }

    if (modFreq * N != 0)
        adelay_coarse_step = 1000000 / (modFreq * N);//unit:ps
    ALOGE("adelay_coarse_step:%d ps !\r\n", adelay_coarse_step);

    rtn |= sensor_read_reg(0x201C, &adelay_coarse);
    rtn |= sensor_read_reg(0x201D, &adelay_fine);
    rtn |= sensor_read_reg(0x201E, &adelay_sfine);

    *time = adelay_coarse_step * adelay_coarse + adelay_fine * 75 + adelay_sfine * 20;
    ALOGE("time: %d adelay_coarse:%d adelay_fine:%d adelay_sfine:%d !\r\n", *time, adelay_coarse, adelay_fine, adelay_sfine);
    return rtn;
}

int mlx75027_max96717_max96716a_init()
{
    int rtn = 0;
    uint8_t test_addr = 0;
    uint16_t read_reg = 0;

    rtn = mlx75027_max96717_max96716a_max96716a_initialize();
    if (module_type == 1)
    {
        rtn = mlx75027_max96717_max96716a_lixiang_max96717_initialize;
    }
    else
    {
        rtn = mlx75027_max96717_max96716a_orbbec_max96717_initialize;
    }
    rtn = mlx75027_max96717_max96716a_sensor_initialize();
    //理想模组需要使能LEDEN信号，否则TX不亮
    if (module_type == 1)
    {
        rtn |= mlx75027_max96717_max96716a_max96717_setup();
        rtn |= i2c_reg_write(0x80, 0x02d3, 2, 0x80, 1);
        rtn |= sensor_write_reg(0x21C4, 0xff);
    }

#if 0
    for (test_addr = 0; test_addr < 0xff; test_addr++)
    {
        rtn = i2c_reg_read(test_addr, 0x00, 1, &read_reg, 1);
        if (rtn >= 0)
        {
            ALOGE("yzx test addr success:0x%x \n", test_addr);
        }
    }
    rtn = 0;
#endif

	rtn |= mlx75027_max96717_max96716a_set_data_output_mode(0);
    rtn |= mlx75027_max96717_max96716a_set_frequency_mode(SINGLE_FREQ);
    rtn |= mlx75027_max96717_max96716a_set_phase_count(4);
    rtn |= mlx75027_max96717_max96716a_set_modulation_frequency(80);

    return rtn;
}

int mlx75027_max96717_max96716a_func_init()
{
	tof_sensor.init = mlx75027_max96717_max96716a_init;
	tof_sensor.get_sensor_id = mlx75027_max96717_max96716a_get_sensor_id;
	tof_sensor.hardware_trigger = mlx75027_max96717_max96716a_hardware_trigger;
	tof_sensor.software_trigger = mlx75027_max96717_max96716a_software_trigger;
	tof_sensor.video_streaming = mlx75027_max96717_max96716a_video_streaming;
	tof_sensor.get_fps = mlx75027_max96717_max96716a_get_fps;
	tof_sensor.set_fps = mlx75027_max96717_max96716a_set_fps;
	tof_sensor.get_sensor_temperature = mlx75027_max96717_max96716a_get_sensor_temperature;
	tof_sensor.get_rx_temp = mlx75027_max96717_max96716a_get_rx_temp;
	tof_sensor.get_tx_temp = mlx75027_max96717_max96716a_get_tx_temp;
	tof_sensor.set_illum_power = mlx75027_max96717_max96716a_set_illum_power;
	tof_sensor.get_illum_power = mlx75027_max96717_max96716a_get_illum_power;
	tof_sensor.illum_power_control = mlx75027_max96717_max96716a_illum_power_control;
	tof_sensor.get_integration_time = mlx75027_max96717_max96716a_get_integration_time;
	tof_sensor.set_integration_time = mlx75027_max96717_max96716a_set_integration_time;
	tof_sensor.get_modulation_frequency = mlx75027_max96717_max96716a_get_modulation_frequency;
	tof_sensor.set_modulation_frequency = mlx75027_max96717_max96716a_set_modulation_frequency;
	tof_sensor.get_illum_duty_cycle = mlx75027_max96717_max96716a_get_illum_duty_cycle;
	tof_sensor.set_illum_duty_cycle = mlx75027_max96717_max96716a_set_illum_duty_cycle;
	tof_sensor.get_data_output_mode = mlx75027_max96717_max96716a_get_data_output_mode;
	tof_sensor.set_data_output_mode = mlx75027_max96717_max96716a_set_data_output_mode;
	tof_sensor.get_img_mirror_flip = mlx75027_max96717_max96716a_get_img_mirror_flip;
	tof_sensor.set_img_mirror_flip = mlx75027_max96717_max96716a_set_img_mirror_flip;
	tof_sensor.get_pixel_binning = mlx75027_max96717_max96716a_get_pixel_binning;
	tof_sensor.set_pixel_binning = mlx75027_max96717_max96716a_set_pixel_binning;
	tof_sensor.test_pattern = mlx75027_max96717_max96716a_test_pattern;
	tof_sensor.get_sensor_info = mlx75027_max96717_max96716a_get_sensor_info;
    tof_sensor.eeprom_write = gt24c128e_eeprom_block_write;
    tof_sensor.eeprom_read = gt24c128e_eeprom_block_read;
    tof_sensor.get_frequency_mode = mlx75027_max96717_max96716a_get_frequency_mode;
    tof_sensor.set_frequency_mode = mlx75027_max96717_max96716a_set_frequency_mode;
    tof_sensor.set_binning_mode = mlx75027_max96717_max96716a_set_binning_mode;
    tof_sensor.get_binning_mode = mlx75027_max96717_max96716a_get_binning_mode;
	tof_sensor.get_illum_duty_cycle_list = mlx75027_max96717_max96716a_get_illum_duty_cycle_list;
    tof_sensor.get_chip_id = mlx75027_sensor_id;

	return 0;
}
