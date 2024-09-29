#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <hw_obstatus.h>
#include "gaea.h"

static uint8_t vcsel_number = 2;
static uint16_t vcsel_driver_type = 3644;

struct regList {
    uint16_t reg;
    uint8_t val;
};

#if !DEBUG_GAEA_IN_QT
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

#define GAEA_PCLK           62500000 // pclk 62.5MHz
#define GAEA_PCLK_MHZ       62.5 // pclk 62.5MHz

uint8_t gaea_tca9548_val = 0;

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

int gaea_dothin_config()
{
#if !DEBUG_GAEA_IN_QT
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

int gaea_tca9548_write_reg(uint8_t reg, uint8_t value)
{
    int rtn = 0;
    uint8_t v1 = 0;

    if (gaea_tca9548_val != value)
    {
        rtn |= i2c_reg_write(TCA9548_ADDR, reg, 0, value, 1);
        rtn |= i2c_reg_read(TCA9548_ADDR, reg, 0, &v1, 1);

        if (rtn || (value - v1))
            DDEBUG("gaea_tca9548 write reg error, ret=%d\n", rtn);
        else
            gaea_tca9548_val = value;
    }

    return rtn;
}

int gaea_tca9548_read_reg(uint8_t reg, uint8_t *value)
{
    int rtn = 0;

    rtn = i2c_reg_read(TCA9548_ADDR, reg, 0, value, 1);

    if (! rtn)
        gaea_tca9548_val = *value;
    else
        DDEBUG("gaea_tca9548 read reg error, ret=%d\n", rtn);

    return rtn;
}

int gaea_write_reg(uint16_t reg, uint8_t value)
{
    gaea_tca9548_write_reg(0, 0x01);
    return i2c_reg_write(SENSOR_ADDR, reg, 2, value, 1);
}

int gaea_read_reg(uint16_t reg, uint8_t *value)
{
    gaea_tca9548_write_reg(0, 0x01);
    return i2c_reg_read(SENSOR_ADDR, reg, 2, value, 1);
}

int gaea_lm3644_write_reg(uint8_t reg, uint8_t value)
{
    gaea_tca9548_write_reg(0, 0x06);
    return i2c_reg_write(LM3644_ADDR, reg, 1, value, 1);
}

int gaea_lm3644_read_reg(uint8_t reg, uint8_t *value)
{
    gaea_tca9548_write_reg(0, 0x06);
    return i2c_reg_read(LM3644_ADDR, reg, 1, value, 1);
}

int gaea_lm3644_ldm_write_reg(uint8_t reg, uint8_t value)
{
    gaea_tca9548_write_reg(0, 0x04);
    return i2c_reg_write(LM3644_ADDR, reg, 1, value, 1);
}

int gaea_lm3644_ldm_read_reg(uint8_t reg, uint8_t *value)
{
    gaea_tca9548_write_reg(0, 0x04);
    return i2c_reg_read(LM3644_ADDR, reg, 1, value, 1);
}

int gaea_lm3644_led_write_reg(uint8_t reg, uint8_t value)
{
    gaea_tca9548_write_reg(0, 0x02);
    return i2c_reg_write(LM3644_ADDR, reg, 1, value, 1);
}

int gaea_lm3644_led_read_reg(uint8_t reg, uint8_t *value)
{
    gaea_tca9548_write_reg(0, 0x02);
    return i2c_reg_read(LM3644_ADDR, reg, 1, value, 1);
}

int gaea_set_debug(uint8_t type, uint8_t value)
{
    int rtn = 0;
    uint8_t val = 0, en1 = 0, en2 = 0;

    switch(type)
    {
        case DEBUG_TYPE_SBG_SELFTEST:
            // check sl4 mode
            rtn |= gaea_get_data_output_mode(&val);
            if (val == GAEA_WORKMODE_SL4)
            {
                rtn |= gaea_lm3644_ldm_read_reg(0x01, &en1);
                rtn |= gaea_lm3644_led_read_reg(0x01, &en2);

                if (value == 1)
                {
                    // set sl4 self test mode, background ldm
                    // ldm torch mode
                    rtn |= gaea_lm3644_ldm_write_reg(0x01, (en1 & 0x03) | 0x08);
                    rtn |= gaea_lm3644_led_write_reg(0x01, (en2 & 0x03) | 0x24);

                    // gpio map, set seq config, c0: strobe1->pad9, strobe1->pad8, c1: strobe0->pad9, strobe0->pad8,
                    rtn |= gaea_write_reg(0x0c01, 0x9f);
                    rtn |= gaea_write_reg(0x0c1e, 0x1c);
                    rtn |= gaea_write_reg(0x0c20, 0x1c);

                    DDEBUG("gaea debug set sbg test mode (backgroud ldm) \r\n");
                }
                else if (value == 2)
                {
                    // set sl4 self test mode, background led
                    // led torch mode
                    rtn |= gaea_lm3644_ldm_write_reg(0x01, (en1 & 0x03) | 0x24);
                    rtn |= gaea_lm3644_led_write_reg(0x01, (en2 & 0x03) | 0x08);

                    // gpio map, set seq config, c0: strobe1->pad9, strobe1->pad8, c1: strobe0->pad9, strobe0->pad8,
                    rtn |= gaea_write_reg(0x0c01, 0x9f);
                    rtn |= gaea_write_reg(0x0c1e, 0x1c);
                    rtn |= gaea_write_reg(0x0c20, 0x1c);

                    DDEBUG("gaea debug set sbg test mode (backgroud led) \r\n");
                }
                else
                {
                    // clear sl4 self test mode
                    // ldm/led ir mode
                    rtn |= gaea_lm3644_ldm_write_reg(0x01, (en1 & 0x03) | 0x24);
                    rtn |= gaea_lm3644_led_write_reg(0x01, (en2 & 0x03) | 0x24);
                    //rtn |= gaea_lm3644_write_reg(0x01, 0x25);

                    // gpio map, set user config
                    rtn |= gaea_write_reg(0x0c01, 0x1f);
                    rtn |= gaea_write_reg(0x0c04, 0x1c); // strobe0->pad9, strobe1->pad8

                    DDEBUG("gaea debug clear sbg test mode \r\n");
                }
            }

            break;

        case DEBUG_TYPE_LOAD_DEFCFG:
            gaea_sensor_load_defcfg();
            break;

        default:
            break;
    }

    return rtn;
}

int gaea_demo_write_reg(uint16_t reg, uint8_t value)
{
    int rtn = 0;
    uint8_t reg_h = reg >> 8;
    uint8_t reg_l = reg & 0xff;

    if (reg_h == LM3644_ADDR)
    {
        rtn = gaea_lm3644_write_reg(reg_l, value);
        DDEBUG("gaea_lm3644 write reg, reg 0x%x, val 0x%x \r\n", reg_l, value);
    }
    else if (reg_h == LM3644_ADDR + 0x02)
    {
        rtn = gaea_lm3644_led_write_reg(reg_l, value);
        DDEBUG("gaea_lm3644 led write reg, reg 0x%x, val 0x%x \r\n", reg_l, value);
    }
    else if (reg_h == LM3644_ADDR + 0x04)
    {
        rtn = gaea_lm3644_ldm_write_reg(reg_l, value);
        DDEBUG("gaea_lm3644 ldm write reg, reg 0x%x, val 0x%x \r\n", reg_l, value);
    }
    else if (reg_h == TCA9548_ADDR)
    {
        rtn = gaea_tca9548_write_reg(reg_l, value);
        DDEBUG("gaea_tca9548 write reg, reg 0x%x, val 0x%x \r\n", reg_l, value);
    }
    else if (reg_h == DEBUG_FLAG)
    {
        rtn = gaea_set_debug(reg, value);
        DDEBUG("gaea set debug, type 0x%x, val 0x%x \r\n", reg_l, value);
    }
    else
    {
        rtn = gaea_write_reg(reg, value);
        DDEBUG("gaea write reg, reg 0x%x, val 0x%x \r\n", reg, value);
    }

    return rtn;
}

int gaea_demo_read_reg(uint16_t reg, uint8_t *value)
{
    int rtn = 0;
    uint8_t reg_h = reg >> 8;
    uint8_t reg_l = reg & 0xff;

    if (reg_h == LM3644_ADDR)
    {
        rtn = gaea_lm3644_read_reg(reg, value);
        DDEBUG("gaea_lm3644 read reg, reg 0x%x, val 0x%x \r\n", reg_l, *value);
    }
    else if (reg_h == LM3644_ADDR + 0x02)
    {
        rtn = gaea_lm3644_led_read_reg(reg, value);
        DDEBUG("gaea_lm3644 led read reg, reg 0x%x, val 0x%x \r\n", reg_l, *value);
    }
    else if (reg_h == LM3644_ADDR + 0x04)
    {
        rtn = gaea_lm3644_ldm_read_reg(reg, value);
        DDEBUG("gaea_lm3644 ldm read reg, reg 0x%x, val 0x%x \r\n", reg_l, *value);
    }
    else if (reg_h == TCA9548_ADDR)
    {
        rtn = gaea_tca9548_read_reg(reg, value);
        DDEBUG("gaea_tca9548 read reg, reg 0x%x, val 0x%x \r\n", reg_l, *value);
    }
    else
    {
        rtn = gaea_read_reg(reg, value);
        DDEBUG("gaea read reg, reg 0x%x, val 0x%x \r\n", reg, *value);
    }

    return rtn;
}

int gaea_set_illum_power(uint8_t vcsel_num, uint8_t value_A, uint8_t value_B)
{
    int rtn = 0;

    // vcsel_num bit 0: ldm enable
    if (vcsel_num & 0x1)
        gaea_lm3644_ldm_write_reg(0x01, 0x25);
    else
        gaea_lm3644_ldm_write_reg(0x01, 0x24);

    // vcsel_num bit 1: led enable
    if (vcsel_num & 0x2)
        gaea_lm3644_led_write_reg(0x01, 0x25);
    else
        gaea_lm3644_led_write_reg(0x01, 0x24);


    // value A: LDM current
    if (value_A)
    {
        gaea_lm3644_ldm_write_reg(0x03, value_A & 0x7f);
    }

    // value B: LED current
    if (value_B)
    {
        gaea_lm3644_led_write_reg(0x03, value_B & 0x7f);
    }

    return rtn;
}

int gaea_get_illum_power(uint8_t *vcsel_num, uint8_t *value_A, uint8_t *value_B)
{
    int rtn = 0;
    uint8_t val = 0;

    *vcsel_num = 0;

    rtn |= gaea_lm3644_ldm_read_reg(0x01, &val);
    if(val & 0x01)
        *vcsel_num |= 0x01;

    rtn |= gaea_lm3644_led_read_reg(0x01, &val);
    if(val & 0x01)
        *vcsel_num |= 0x02;

    rtn |= gaea_lm3644_ldm_read_reg(0x03, value_A);

    rtn |= gaea_lm3644_led_read_reg(0x03, value_B);

    return rtn;
}

struct regList gaea_lm3644_reglist[] = {
    { 0x07, 0x89 }, // reset
    { 0x07, 0x09 }, // clear reset, set pass mode
    { 0x08, 0x10 }, // set torch ramp time(1ms) and flash timeout(40ms)
    { 0x01, 0x25 }, // enable strobe and LED1/LED2, set IR mode
    { 0x03, 0x64 }, // led1 flash current
    { 0x04, 0x00 }, // led2 flash current
    { 0x05, 0x7f }, // led1 torch current
    { 0x06, 0x00 }, // led2 torch current
};

struct regList gaea_reglist[] = {
    { 0x000E, 0x03 }, // PLL1 PLL2 always on
    { 0x0C06, 0xFF }, // 数字内部各种时钟不gateing
    { 0x0c12, 0x20 }, // sensor stop
    { 0x0c08, 0x01 }, // CHIP软复位
    { 0x0c07, 0x01 }, // MIPI软复位
    { 0x0c07, 0x00 },
    { 0x0c08, 0x00 },
};

struct regList gaea_defcfg_reglist[] = {
    { 0x000e, 0x03 }, // PLL1 PLL2 always on
    { 0x0c06, 0xff }, //数字内部各种时钟不gateing
    { 0x0c21, 0x01 }, // DPHY BG_RT的来源选择 1为从寄存器读取数据   0为从OTP读取数据  
    { 0x143e, 0x3f }, // 模拟信号配置来源 3F来自寄存器 0来自OTP
    { 0x143d, 0xff }, // 模拟信号配置来源 FF来自寄存器 0来自OTP
    { 0x0c23, 0x7e },
    { 0x0c22, 0x00 }, // DB offset = -100 -> 7F9C
    { 0x0c11, 0x02 }, // 2为无限帧， 0 为有限帧
    { 0x306b, 0x00 },
    { 0x306a, 0x05 }, // 有限帧率下的帧数，无限帧下配置无效

    { 0x0c07, 0x01 },
    { 0x0c07, 0x01 },
    { 0x0c07, 0x00 }, // MIPI软复位
    { 0x0c07, 0x00 }, // MIPI软复位
    { 0x1f19, 0xfb }, // 打开mipi，不包括ZQ
    { 0x1f19, 0xfb }, // 打开mipi，不包括ZQ

    { 0x204f, 0x80 }, // RAW 配置  0 为RAW12， 1 为RAW10. [7] enable embedded data
    { 0x2052, 0x2c }, // EMB Data 类型 raw10 0x2a, raw12 0x2c
    { 0x0c00, 0x01 }, // 驱动光源, pad7/8/9 output enable
    { 0x0c04, 0x1b }, // 驱动光源, strobe0->pad9, strobe0->pad8
    //{ 0x0c04, 0x23 }, // 驱动光源, strobe1->pad9, strobe0->pad8
    //{ 0x0c04, 0x1c }, // 驱动光源, strobe0->pad9, strobe1->pad8
    { 0x0009, 0x7d }, // 4lane 的配置 // 配置PLL2 输出时钟为750MHZ。

    { 0x000a, 0x03 }, // 配置PLL2 输出时钟为750MHZ。
    { 0x000a, 0x03 }, // 配置PLL2 输出时钟为750MHZ。
    { 0x1f01, 0x0c },
    { 0x1f01, 0x0c },

    { 0x2000, 0x80 }, //开窗与时序设置 //ISP使能打开
    { 0x2015, 0x80 }, //ROI开窗功能打开
    { 0x2016, 0x08 },
    { 0x2017, 0x00 }, //纵向起始列=8
    { 0x2018, 0xb0 },
    { 0x2019, 0x04 }, //纵向列数=1200
    { 0x3031, 0x80 },
    { 0x3032, 0x07 }, //读出行数=1920
    { 0x3033, 0x80 },
    { 0x3034, 0x07 }, //读出行数=1920
    { 0x3035, 0x80 },
    { 0x3036, 0x07 }, //读出行数=1920
    { 0x3049, 0x0b },
    { 0x304a, 0x00 }, //读出起始行=11
    { 0x304b, 0x0b },
    { 0x304c, 0x00 }, //读出起始行=11
    { 0x304d, 0x0b },
    { 0x304e, 0x00 }, //读出起始行=11

    { 0x300a, 0x64 }, // tline c0
    { 0x300b, 0x00 },
    { 0x300c, 0x64 }, // tline c1
    { 0x300d, 0x00 },
    { 0x300e, 0x64 }, // tline c2
    { 0x300f, 0x00 },

    { 0x3010, 0xf4 },
    { 0x3011, 0x01 }, //t_slot_c0=500*16ns, 8us
    { 0x3012, 0xf4 },
    { 0x3013, 0x01 }, //t_slot_c1=500*16ns, 8us
    { 0x3014, 0xf4 },
    { 0x3015, 0x01 }, //t_slot_c2=500*16ns, 8us
    { 0x3016, 0x00 },
    { 0x3017, 0x02 }, //exp_c0=512*t_slot, 4ms
    { 0x3018, 0x00 },
    { 0x3019, 0x02 }, //exp_c1=512*t_slot, 4ms
    { 0x301a, 0x00 },
    { 0x301b, 0x02 }, //exp_c2=512*t_slot, 4ms
    { 0x3022, 0x80 },
    { 0x3023, 0x00 }, //exp_rs_c0=512*t_bg_slot, 4ms (1 bgslot = 4 tslot)
    { 0x3024, 0x80 },
    { 0x3025, 0x00 }, //exp_rs_c1=512*t_bg_slot, 4ms (1 bgslot = 4 tslot)
    { 0x3026, 0x80 },
    { 0x3027, 0x00 }, //exp_rs_c2=512*t_bg_slot, 4ms (1 bgslot = 4 tslot)

    { 0x201a, 0x00 }, // dig gain, [3:1] c2:c0, [0] noise
    { 0x201d, 0x20 }, // c0 odd gain, 1/16 * val, 1/6 ~ 16
    { 0x201e, 0x20 }, // c0 even gain, 1/16 * val, 1/6 ~ 16
    { 0x201f, 0x20 }, // c1 odd gain, 1/16 * val, 1/6 ~ 16
    { 0x2020, 0x20 }, // c1 even gain, 1/16 * val, 1/6 ~ 16
    { 0x2021, 0x20 }, // c2 odd gain, 1/16 * val, 1/6 ~ 16
    { 0x2022, 0x20 }, // c2 even gain, 1/16 * val, 1/6 ~ 16

    { 0x3089, 0x08 }, // analog gain c0, 1 + 0.25 * val, 1x~4.75x
    { 0x308a, 0x08 }, // analog gain c1, 1 + 0.25 * val, 1x~4.75x
    { 0x308b, 0x08 }, // analog gain c2, 1 + 0.25 * val, 1x~4.75x

    { 0x1407, 0x29 }, //调节VREF电压值

    { 0x0005, 0x05 }, // lost
    { 0x0005, 0x05 },

    { 0x0006, 0x05 }, //调节CP输入频率
    { 0x31dc, 0x05 }, //调节FOT1时间倍数
    { 0x3075, 0x02 },
    { 0x309d, 0x00 },
    { 0x309c, 0x6d }, //pix_tim_val
    { 0x309f, 0x00 },
    { 0x309e, 0x6d }, //pix_tim_val
    { 0x30a1, 0x00 },
    { 0x30a0, 0x6d }, //pix_tim_val
    { 0x3100, 0x03 }, //调节PIX时间倍数
    { 0x1408, 0x29 }, //调节V_CLIP_RST
    { 0x315e, 0x81 }, //adc timing delay 200
    { 0x315f, 0x2c },
    { 0x3160, 0x00 },
    { 0x3161, 0x01 },
    { 0x3162, 0x21 },
    { 0x3163, 0x00 },
    { 0x3164, 0x01 },
    { 0x3165, 0x21 },
    { 0x3166, 0x00 },
    { 0x3167, 0x02 },
    { 0x3168, 0x40 },
    { 0x3169, 0x00 },
    { 0x316a, 0x28 },
    { 0x316b, 0x20 },
    { 0x316c, 0x00 },
    { 0x316d, 0x10 },
    { 0x316e, 0x68 },
    { 0x316f, 0x01 },
    { 0x3170, 0x80 },
    { 0x3171, 0x20 },
    { 0x3172, 0x00 },
    { 0x3173, 0x2a },
    { 0x3174, 0x28 },
    { 0x3175, 0x00 },
    { 0x3176, 0x80 },
    { 0x3177, 0x80 },
    { 0x3178, 0x00 },
    { 0x3179, 0x00 },
    { 0x317a, 0x21 },
    { 0x317b, 0x00 },
    { 0x317c, 0x00 },
    { 0x317d, 0x21 },
    { 0x317e, 0x00 },
    { 0x317f, 0x00 },
    { 0x3180, 0x24 },
    { 0x3181, 0x00 },
    { 0x3182, 0x04 },
    { 0x3183, 0x22 },
    { 0x3184, 0x00 },
    { 0x3185, 0x00 },
    { 0x3186, 0x24 },
    { 0x3187, 0x00 },
    { 0x3188, 0x48 },
    { 0x3189, 0x20 },
    { 0x318a, 0x00 },
    { 0x318b, 0x10 },
    { 0x318c, 0xa8 },
    { 0x318d, 0x01 },
    { 0x318e, 0x00 },
    { 0x318f, 0x00 },
    { 0x3190, 0x01 },
    { 0x3191, 0x80 },
    { 0x3192, 0x20 },
    { 0x3193, 0x00 },
    { 0x3194, 0x4c },
    { 0x3195, 0x28 },
    { 0x3196, 0x00 },
    { 0x3197, 0x80 },
    { 0x3198, 0x20 },
    { 0x3199, 0x00 },
    { 0x319a, 0x00 },
    { 0x319b, 0x30 },
    { 0x319c, 0x00 },
    { 0x319d, 0x00 },
    { 0x319e, 0x70 },
    { 0x319f, 0x00 },
    { 0x31a0, 0x00 },
    { 0x31a1, 0x02 },
    { 0x31a2, 0x00 },
    { 0x31a3, 0x00 },
    { 0x31a4, 0x00 },
    { 0x31a5, 0x00 },
    { 0x0c12, 0x20 }, // sensor stop
    //{ 0x0c12, 0x01 }, // sensor start
};

int gaea_sensor_initialize()
{
    int rtn = 0;

    for(int i = 0; i < sizeof(gaea_reglist)/sizeof(struct regList); i++){

        rtn |= gaea_write_reg(gaea_reglist[i].reg, gaea_reglist[i].val);
    }

    DDEBUG("gaea_sensor_initialize, return 0x%x \r\n", rtn);

    return rtn;
}

int gaea_sensor_load_defcfg()
{
    int rtn = 0;

    for (int i = 0; i < sizeof(gaea_defcfg_reglist) / sizeof(struct regList); i++) {

        rtn |= gaea_write_reg(gaea_defcfg_reglist[i].reg, gaea_defcfg_reglist[i].val);
    }

    DDEBUG("gaea_sensor_load_defcfg, return 0x%x \r\n", rtn);

    return rtn;
}

int gaea_get_sensor_id(uint16_t *id) // id should be 0x96
{
    int rtn = 0;
    uint8_t v1 = 0;
    uint8_t v2 = 0;

    static int isConfig = 0;
    if (!isConfig) {
        isConfig = 1;
        gaea_dothin_config();
        DDEBUG("config dothin \r\n");
    }

    rtn |= gaea_read_reg(0x0c30, &v1);
    rtn |= gaea_read_reg(0x0c31, &v2);
    if(! rtn)
        //*id = ((uint16_t)v1 << 8) | v2;
		*id = 0xe000;
    else
        *id = 0xffff;

    DDEBUG("gaea chip id: 0x%x \r\n", *id);

    rtn |= gaea_lm3644_read_reg(0x00, &v1);
    rtn |= gaea_lm3644_read_reg(0x0c, &v2);

    DDEBUG("aw3644 chip id: 0x%x, devce id: 0x%x \r\n", v1, v2);

    return rtn;
}

int gaea_video_streaming(bool enable)
{
    int rtn = 0;
#if 1
    rtn |= gaea_write_reg(0x0c12, 0x20);

    if (enable)
        rtn |= gaea_write_reg(0x0c12, 0x01);
    else
        rtn |= gaea_write_reg(0x0c12, 0x00);
#endif
    return rtn;
}

int gaea_get_data_output_mode(uint8_t *mode)
{
    int rtn = 0;
    uint8_t val = 0;

    rtn |= gaea_read_reg(0x0c11, &val);

    *mode = (val & 0x1c) >> 2;

    DDEBUG("gaea get output mode: 0x%x \r\n", *mode);

    return rtn;
}

int gaea_set_data_output_mode(uint8_t mode)
{
	int rtn = 0;
	uint8_t val = 0, en = 0;

	rtn |= gaea_video_streaming(0);

	rtn |= gaea_read_reg(0x0c11, &val);

	val = (val & ~0x1c) | ((mode << 2) & 0x1c);

	rtn |= gaea_write_reg(0x0c11, val);

	switch (mode)
	{
	case GAEA_WORKMODE_NORMAL:
		gaea_write_reg(0x3000, 0x00); // enable 1 context
		gaea_write_reg(0x203e, 0x00); // disable sbg
		gaea_write_reg(0x0c04, 0x1b); // strobe0->pad9, strobe0->pad8
		break;
	case GAEA_WORKMODE_SL0:
		gaea_write_reg(0x3000, 0x01); // enable 2 context
		gaea_write_reg(0x203e, 0x00); // disable sbg
		gaea_write_reg(0x0c04, 0x1c); // strobe0->pad9, strobe1->pad8
		break;
	case GAEA_WORKMODE_SL1:
		gaea_write_reg(0x3000, 0x01); // enable 2 context
		gaea_write_reg(0x203e, 0x00); // disable sbg
		gaea_write_reg(0x0c04, 0x1b); // strobe0->pad9, strobe0->pad8
		break;
	case GAEA_WORKMODE_SL2:
		gaea_write_reg(0x3000, 0x00); // enable 1 context
		gaea_write_reg(0x203e, 0x04); // enable sbg
		gaea_write_reg(0x0c04, 0x1b); // strobe0->pad9, strobe0->pad8
		break;
	case GAEA_WORKMODE_SL3:
		gaea_write_reg(0x3000, 0x02); // enable 3 context
		gaea_write_reg(0x203e, 0x00); // disable sbg
		gaea_write_reg(0x0c04, 0x1c); // strobe0->pad9, strobe0->pad8
		break;
	case GAEA_WORKMODE_SL4:
		gaea_write_reg(0x3000, 0x01); // enable 2 context
		gaea_write_reg(0x203e, 0x04); // enable sbg
		gaea_write_reg(0x0c04, 0x1c); // strobe0->pad9, strobe1->pad8
		break;
	}

	DDEBUG("gaea set output mode: 0x%x \r\n", mode);

	return rtn;
}

int gaea_set_window_originy(uint32_t originy)
{
	int rtn = 0;
	uint8_t val_h = 0;
	uint8_t val_l = 0;
	val_l = originy & 0xff;
	val_h = (originy >> 8) & 0xff;
	rtn |= gaea_write_reg(0x304a, val_h);
	rtn |= gaea_write_reg(0x3049, val_l);

	DDEBUG("gaea set window originy: 0x%x \r\n", originy);

	return rtn;
}

int gaea_set_window_originx(uint32_t originx)
{
	int rtn = 0;
	uint8_t val_h = 0;
	uint8_t val_l = 0;
	val_l = originx & 0xff;
	val_h = (originx >> 8) & 0xff;
	rtn |= gaea_write_reg(0x2000, 0x80);
	rtn |= gaea_write_reg(0x2015, 0x80);
	rtn |= gaea_write_reg(0x2016, val_l);
	rtn |= gaea_write_reg(0x2017, val_h);

	DDEBUG("gaea set window originx: 0x%x \r\n", originx);

	return rtn;
}

int gaea_set_window_height(uint32_t height)
{
	int rtn = 0;
	uint8_t val_h = 0;
	uint8_t val_l = 0;
	val_l = height & 0xff;
	val_h = (height >> 8) & 0xff;
	rtn |= gaea_write_reg(0x3032, val_h);
	rtn |= gaea_write_reg(0x3031, val_l);

	DDEBUG("gaea set window height: 0x%x \r\n", height);

	return rtn;
}

int gaea_get_window_height(uint32_t *height)
{
	int rtn = 0;
	uint32_t val_h = 0;
	uint32_t val_l = 0;
	rtn |= gaea_read_reg(0x3032, &val_h);
	rtn |= gaea_read_reg(0x3031, &val_l);
	*height = (val_h << 0x08) | val_l;
	DDEBUG("gaea get window height: 0x%x \r\n", *height);
	return rtn;
}

int gaea_set_window_width(uint32_t width)
{
	int rtn = 0;
	uint8_t val_h = 0;
	uint8_t val_l = 0;
	val_l = width & 0xff;
	val_h = (width >> 8) & 0xff;
	rtn |= gaea_write_reg(0x2000, 0x80);
	rtn |= gaea_write_reg(0x2015, 0x80);
	rtn |= gaea_write_reg(0x2018, val_l);
	rtn |= gaea_write_reg(0x2019, val_h);
	DDEBUG("gaea set window width: 0x%x \r\n", width);

	return rtn;
}

int gaea_get_window_width(uint32_t *width)
{
	int rtn = 0;
	uint32_t val_h = 0;
	uint32_t val_l = 0;
	rtn |= gaea_read_reg(0x2018, &val_l);
	rtn |= gaea_read_reg(0x2019, &val_h);
	*width = (val_h << 0x08) | val_l;
	DDEBUG("gaea get window width: 0x%x \r\n", *width);
	return rtn;
}

int gaea_get_fps(uint8_t *fps)
{
    return -HW_ERR_NO_SUPPORT;
}

int gaea_set_fps(uint8_t fps)
{
    return -HW_ERR_NO_SUPPORT;
}

int gaea_get_integration_time(uint16_t *integrationTime)
{
    int rtn = 0;
    uint8_t v1 = 0, v2 = 0;
    uint32_t tslot = 0, exp = 0;

    // get tslot
    rtn = gaea_read_reg(0x3010, &v1);
    rtn = gaea_read_reg(0x3011, &v2);
    tslot = ((uint32_t)v2 << 8) | v1;

    // get exp
    rtn = gaea_read_reg(0x3016, &v1);
    rtn = gaea_read_reg(0x3017, &v2);
    exp = ((uint32_t)v2 << 8) | v1;

    *integrationTime = (uint16_t)((float)exp * tslot / GAEA_PCLK_MHZ);

    DDEBUG("gaea_get_exp %d \r\n", *integrationTime);

    return rtn;
}

int gaea_set_integration_time(uint16_t integrationTime)
{
    int rtn = 0;
    uint8_t v1 = 0, v2 = 0;
    uint32_t tslot = 0, exp = 0;

    // get tslot
    rtn = gaea_read_reg(0x3010, &v1);
    rtn = gaea_read_reg(0x3011, &v2);
    tslot = ((uint32_t)v2 << 8) | v1;


    exp = (uint32_t)(integrationTime * GAEA_PCLK_MHZ / (float)tslot);

    v1 = exp & 0xff;
    v2 = (exp >> 8) & 0xff;

    // set gs exp
    rtn = gaea_write_reg(0x3016, v1);
    rtn = gaea_write_reg(0x3017, v2);
    rtn = gaea_write_reg(0x3018, v1);
    rtn = gaea_write_reg(0x3019, v2);
    rtn = gaea_write_reg(0x301a, v1);
    rtn = gaea_write_reg(0x301b, v2);
    // set rs exp
    exp /= 4;
    v1 = exp & 0xff;
    v2 = (exp >> 8) & 0xff;
    rtn = gaea_write_reg(0x3022, v1);
    rtn = gaea_write_reg(0x3023, v2);
    rtn = gaea_write_reg(0x3024, v1);
    rtn = gaea_write_reg(0x3025, v2);
    rtn = gaea_write_reg(0x3026, v1);
    rtn = gaea_write_reg(0x3027, v2);

    DDEBUG("gaea_set_exp %d, return 0x%x \r\n", integrationTime, rtn);

    return rtn;
}

int gaea_get_exp(uint32_t *exposure)
{
    int rtn = 0;
    uint8_t v1 = 0, v2 = 0;
    uint32_t tslot = 0, exp = 0;

    // get tslot
    rtn = gaea_read_reg(0x3010, &v1);
    rtn = gaea_read_reg(0x3011, &v2);
    tslot = ((uint32_t)v2 << 8) | v1;

    // get exp
    rtn = gaea_read_reg(0x3016, &v1);
    rtn = gaea_read_reg(0x3017, &v2);
    exp = ((uint32_t)v2 << 8) | v1;

    *exposure = (uint32_t)((float)exp * tslot / GAEA_PCLK_MHZ);

    DDEBUG("gaea_get_exp %d \r\n", *exposure);

    return rtn;
}

int gaea_set_exp(uint32_t exposure)
{
    int rtn = 0;
    uint8_t v1 = 0, v2 = 0;
    uint32_t tslot = 0, exp = 0;

    // get tslot
    rtn = gaea_read_reg(0x3010, &v1);
    rtn = gaea_read_reg(0x3011, &v2);
    tslot = ((uint32_t)v2 << 8) | v1;


    exp = (uint32_t)(exposure * GAEA_PCLK_MHZ / (float)tslot);

    v1 = exp & 0xff;
    v2 = (exp >> 8) & 0xff;

    // set gs exp
    rtn = gaea_write_reg(0x3016, v1);
    rtn = gaea_write_reg(0x3017, v2);
    rtn = gaea_write_reg(0x3018, v1);
    rtn = gaea_write_reg(0x3019, v2);
    rtn = gaea_write_reg(0x301a, v1);
    rtn = gaea_write_reg(0x301b, v2);
    // set rs exp
    exp /= 4;
    v1 = exp & 0xff;
    v2 = (exp >> 8) & 0xff;
    rtn = gaea_write_reg(0x3022, v1);
    rtn = gaea_write_reg(0x3023, v2);
    rtn = gaea_write_reg(0x3024, v1);
    rtn = gaea_write_reg(0x3025, v2);
    rtn = gaea_write_reg(0x3026, v1);
    rtn = gaea_write_reg(0x3027, v2);

    DDEBUG("gaea_set_exp %d, return 0x%x \r\n", exposure, rtn);

    return rtn;
}

int gaea_get_gain(uint32_t *gain)
{
    int rtn = 0;
    uint8_t val = 0;

    rtn = gaea_read_reg(0x3089, &val);

    *gain = (val & 0x0f) * 250 + 1000;

    DDEBUG("gaea_get_gain %d \r\n", *gain);

    return rtn;
}

int gaea_set_gain(uint32_t gain)
{
    int rtn = 0;
    uint8_t val = 0;

    val = (gain < 1000 ? 1000 : (gain > 4750 ? 4750 : gain)) / 250 - 4;

    rtn = gaea_write_reg(0x3089, val & 0x0f);
    rtn = gaea_write_reg(0x308a, val & 0x0f);
    rtn = gaea_write_reg(0x308b, val & 0x0f);

    DDEBUG("gaea_set_gain %d, return 0x%x \r\n", gain, rtn);

    return rtn;
}

int gaea_set_odd_dgain(uint32_t gain)
{
	int rtn = 0;

	rtn = gaea_write_reg(0x201a, 0x02);//开启数字增益
	rtn = gaea_write_reg(0x201e, gain);//调节奇数列数字增益
	DDEBUG("gaea_set_odd_dgain %d, return 0x%x \r\n", gain, rtn);

	return rtn;
}

int gaea_set_even_dgain(uint32_t gain)
{
	int rtn = 0;

	rtn = gaea_write_reg(0x201a, 0x02);//开启数字增益
	rtn = gaea_write_reg(0x201d, gain);//调节偶数列数字增益
	DDEBUG("gaea_set_odd_dgain %d, return 0x%x \r\n", gain, rtn);

	return rtn;
}

int gaea_set_sub_samp(uint8_t value)
{
	int rtn = 0;

	if(value == 0)
	{
		rtn = gaea_write_reg(0x204a, 0x00);//列抽样关闭
	}
	else
	{
		rtn = gaea_write_reg(0x2000, 0x80);
		rtn = gaea_write_reg(0x204a, 0x01);//列抽样开启
	}
	rtn = gaea_write_reg(0x204b, value);//列抽样开启
	DDEBUG("gaea_set_sub_samp %d, return 0x%x \r\n", value, rtn);

	return rtn;
}

int gaea_get_sub_samp(uint8_t *value)
{
	int rtn = 0;
	uint8_t val = 0;

	rtn = gaea_read_reg(0x204b, &val);

	*value = val;

	DDEBUG("gaea_get_sub_samp %d \r\n", *value);

	return rtn;
}

int gaea_get_ldm_en(uint32_t *en)
{
    int rtn = 0;
    uint8_t val = 0;

    rtn |= gaea_lm3644_ldm_read_reg(0x01, &val);
    if (val & 0x01)
        *en = 1;

    DDEBUG("gaea_get_ldm_en %d \r\n", *en);

    return rtn;
}

int gaea_set_ldm_en(uint32_t en)
{
    int rtn = 0;

    if (en)
        rtn |= gaea_lm3644_ldm_write_reg(0x01, 0x25);
    else
        rtn |= gaea_lm3644_ldm_write_reg(0x01, 0x24);

    DDEBUG("gaea_set_ldm_en %d \r\n", en);

    return rtn;
}

int gaea_get_led_en(uint32_t *en)
{
    int rtn = 0;
    uint8_t val = 0;

    rtn |= gaea_lm3644_led_read_reg(0x01, &val);
    if (val & 0x01)
        *en = 1;

    DDEBUG("gaea_get_led_en %d \r\n", *en);

    return rtn;
}

int gaea_set_led_en(uint32_t en)
{
    int rtn = 0;

    if (en)
        rtn |= gaea_lm3644_led_write_reg(0x01, 0x25);
    else
        rtn |= gaea_lm3644_led_write_reg(0x01, 0x24);

    DDEBUG("gaea_set_led_en %d \r\n", en);

    return rtn;
}
int gaea_get_ldm_current(uint32_t *current)
{
    int rtn = 0;
    uint8_t val = 0;


    gaea_lm3644_ldm_read_reg(0x03, &val);

    *current = (val & 0x7f) * 12;

    DDEBUG("gaea_get_ldm_current %d \r\n", *current);

    return rtn;
}

int gaea_set_ldm_current(uint32_t current)
{
    int rtn = 0;
    uint8_t val = 0;

    val = (current > 1524 ? 1524 : current) / 12;

    rtn |= gaea_lm3644_ldm_write_reg(0x03, val);

    DDEBUG("gaea_set_ldm_current %d, return 0x%x \r\n", current, rtn);

    return rtn;
}
int gaea_get_led_current(uint32_t *current)
{
    int rtn = 0;
    uint8_t val = 0;


    gaea_lm3644_led_read_reg(0x03, &val);

    *current = (val & 0x7f) * 12;

    DDEBUG("gaea_get_led_current %d \r\n", *current);

    return rtn;
}

int gaea_set_led_current(uint32_t current)
{
    int rtn = 0;
    uint8_t val = 0;

    val = (current > 1524 ? 1524 : current) / 12;

    rtn |= gaea_lm3644_led_write_reg(0x03, val);

    DDEBUG("gaea_set_led_current %d, return 0x%x \r\n", current, rtn);

    return rtn;
}

int gaea_set_pixel_binning(uint8_t mode)
{
	int rtn = 0;
	uint8_t val = 0;

	if (mode == 0){
		rtn = gaea_write_reg(0x2000, 0x80);
		rtn = gaea_write_reg(0x2049, 0x00);//1*1bining
	}
	else if (mode == 1){
		rtn = gaea_write_reg(0x2000, 0x80);
		rtn = gaea_write_reg(0x2049, 0x05);//2*2bining
	}
	else if (mode == 2){
		rtn = gaea_write_reg(0x2000, 0x80);
		rtn = gaea_write_reg(0x2049, 0x06);//3*3bining
	}

	DDEBUG("gaea set bining mode:%d, return 0x%x \r\n", mode, rtn);

	return rtn;
}

int gaea_get_pixel_binning(uint8_t *mode)
{
	int rtn = 0;
	uint8_t val = 0;

	rtn |= gaea_read_reg(0x2049, &val);
	*mode = val & 0x03;

	DDEBUG("gaea get bining mode: 0x%x \r\n", *mode);

	return rtn;
}

int gaea_get_img_mirror_flip(uint8_t *mode)
{
    return -HW_ERR_NO_SUPPORT;
}

int gaea_set_img_mirror_flip(uint8_t mode)
{
	int rtn = 0;
	uint8_t val = 0;

	switch (mode)
	{
	case 0:
		rtn = gaea_write_reg(0x3061, 0x00);
		rtn = gaea_write_reg(0x2014, 0x00);//翻转关闭
		break;
	case 1:
		rtn = gaea_write_reg(0x3061, 0x01);//垂直翻转开
		break;
	case 2:
		rtn = gaea_write_reg(0x2000, 0x80);
		rtn = gaea_write_reg(0x2014, 0x01);//水平翻转开
		break;
	}

	DDEBUG("gaea set flip mode:%d, return 0x%x \r\n", mode, rtn);

	return rtn;
}

int gaea_get_sensor_temperature(float *temp)
{
    return  -HW_ERR_NO_SUPPORT;
}

int gaea_test_pattern(uint8_t mode)
{
    int rtn = 0;

    return rtn;
}

int gaea_get_sensor_info(struct sensor_info_t *info) {
    info->embedded_data_size = IMAGE_WIDTH * 2;
    info->vcsel_num = vcsel_number;
    info->vcsel_driver_id = vcsel_driver_type;
    info->sensor_id = gaea_sensor_id;
    return 0;
}

int gaea_ldm_led_initialize()
{
    int rtn = 0;

    for (int i = 0; i < sizeof(gaea_lm3644_reglist) / sizeof(struct regList); i++) {

        rtn |= gaea_lm3644_write_reg(gaea_lm3644_reglist[i].reg & 0xff, gaea_lm3644_reglist[i].val);
    }

    DDEBUG("gaea_ldm_initialize, return 0x%x \r\n", rtn);

    return rtn;
}

int gaea_led_initialize()
{
    int rtn = 0;

    return rtn;
}

int gaea_init()
{
    int rtn = 0;

    // set LDM and LED driver
    rtn |= gaea_ldm_led_initialize();
    //rtn |= gaea_set_ldm_en(1);
    //rtn |= gaea_set_illum_power(3, 100, 100);


    // set gaea sensor
    rtn |= gaea_sensor_initialize();
    //rtn |= gaea_set_data_output_mode(0);

    return rtn;
}

int gaea_set_chip_reset(uint32_t reset_en)
{
    int rtn = 0;

    if(reset_en)
        rtn = gaea_init();

    return rtn;
}

int gaea_func_init()
{
#if !DEBUG_GAEA_IN_QT
    tof_sensor.init = gaea_init;
    tof_sensor.get_sensor_id = gaea_get_sensor_id;
    tof_sensor.video_streaming = gaea_video_streaming;
    tof_sensor.get_fps = gaea_get_fps;
    tof_sensor.set_fps = gaea_set_fps;
    tof_sensor.get_sensor_temperature = gaea_get_sensor_temperature;
    tof_sensor.set_illum_power = gaea_set_illum_power;
    tof_sensor.get_illum_power = gaea_get_illum_power;
    tof_sensor.get_integration_time = gaea_get_integration_time;
    tof_sensor.set_integration_time = gaea_set_integration_time;
    tof_sensor.get_data_output_mode = gaea_get_data_output_mode;
    tof_sensor.set_data_output_mode = gaea_set_data_output_mode;
	tof_sensor.set_window_originy = gaea_set_window_originy;
	tof_sensor.set_window_originx = gaea_set_window_originx;
	tof_sensor.set_window_height = gaea_set_window_height;
	tof_sensor.get_window_height = gaea_get_window_height;
	tof_sensor.set_window_width = gaea_set_window_width;
	tof_sensor.get_window_width = gaea_get_window_width;
    tof_sensor.get_img_mirror_flip = gaea_get_img_mirror_flip;
    tof_sensor.set_img_mirror_flip = gaea_set_img_mirror_flip;
    tof_sensor.get_pixel_binning = gaea_get_pixel_binning;
    tof_sensor.set_pixel_binning = gaea_set_pixel_binning;
    tof_sensor.test_pattern = gaea_test_pattern;
    tof_sensor.get_sensor_info = gaea_get_sensor_info;
    tof_sensor.sensor_write_reg_8 = gaea_demo_write_reg;
    tof_sensor.sensor_read_reg_8 = gaea_demo_read_reg;
    tof_sensor.get_exp = gaea_get_exp;
    tof_sensor.set_exp = gaea_set_exp;
    tof_sensor.get_gain = gaea_get_gain;
    tof_sensor.set_gain = gaea_set_gain;
	tof_sensor.set_odd_dgain = gaea_set_odd_dgain;
	tof_sensor.set_even_dgain = gaea_set_even_dgain;
	tof_sensor.set_sub_samp = gaea_set_sub_samp;
	tof_sensor.get_sub_samp = gaea_get_sub_samp;
    tof_sensor.get_ldm_en = gaea_get_ldm_en;
    tof_sensor.set_ldm_en = gaea_set_ldm_en;
    tof_sensor.get_led_en = gaea_get_led_en;
    tof_sensor.set_led_en = gaea_set_led_en;
    tof_sensor.get_ldm_current = gaea_get_ldm_current;
    tof_sensor.set_ldm_current = gaea_set_ldm_current;
    tof_sensor.get_led_current = gaea_get_led_current;
    tof_sensor.set_led_current = gaea_set_led_current;
    tof_sensor.set_chip_reset = gaea_set_chip_reset;
#endif
    return 0;
}
