#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "imx456.h"


#if  0
#define DDEBUG(fmt, ...)   	printf("[DEBUG] [%s(%d)] : " fmt"\n",__FUNCTION__,__LINE__,##__VA_ARGS__)
#else
#define DDEBUG(fmt, ...)   	
#endif

#define USED_IN_QC_DEVICE        1

// GPIO control
#define TRIGGER_PIN              2 // PO1
#define GPIO_EXPANDER_RST_PIN    1 // PO2
#define LD_ERROR_PIN             3

// IIC slave device
#define SENSOR_ADDR          (0x57 << 1)
#define EEPROM_ADDR          (0x50 << 1)
#define GPIO_EXPANDER_ADDR   (0x20 << 1) // TCA6408A

// vcsel driver IC type
#define DRIVER_IC_CXA4016              4016
#define DRIVER_IC_PHX3D_3021_AA        5016
#define DRIVER_IC_PHX3D_3021_CB        5017
#define DRIVER_IC_DW9912               9912 // DongWoon

//static uint16_t HMAX = 694; // 0x02B6   A-B mode
static uint16_t HMAX = 1300; // 0x02B6   A&B mode
static float CLK120MHz = 120.0f; // result turned out to be us
#if USED_IN_QC_DEVICE
static uint8_t use_trigger_mode = 0;// QC device should use auto streaming mode
#else
static uint8_t use_trigger_mode = 1;
#endif
static uint16_t driver_ic_type = 0;

struct regList {
	uint16_t reg;
	uint8_t val;
};	

#if !DEBUG_IMX456_IN_QT
#include <hw_obstatus.h>
#include <tof_sensors.h>
#include <obc_tee_funcs.h>

#ifdef __linux__
#include <unistd.h>
#endif



#define TEE_LOG_LEVEL_ERROR        8
#define ALOGE(fmt,...) tops_t.qsee_log(TEE_LOG_LEVEL_ERROR, "[ERROR] [%s(%d)] : " fmt"\n",__FUNCTION__,__LINE__,##__VA_ARGS__)

#define malloc tops_t.qsee_malloc
#define free tops_t.qsee_free
#define usleep tops_t.tee_usleep
#define orbbec_i2c_writeread tops_t.ops_writeread


#define dothin_device_id           tops_t.ap_ops.device_id

#define I2C_M_RD     1
#define I2C_M_WT     0


#if  (USE_WHICH_CONVERTER==kConverterIsCx3)
static char *test_name = "use_c2x3";

#define cx3_mipi_cfg            tops_t.ap_ops.SetMipiConfiguration
#define cx3_set_gpio_dir        tops_t.ap_ops.SetGpioPinDir
#define cx3_set_gpio_level      tops_t.ap_ops.SetGpioPinLevel

#define SENSOR_RESET_GPIO          0xFF
#define V3V3_EN                    57
#define VDDA_2V7_EN_GPIO           17
#define VDDIF_1V8_EN_GPIO          18
#define VDDD_1V2_EN_GPIO           19
#define VDDMIX_1V2_EN_GPIO         20
#define VDDLD_3V6_EN_GPIO          21
#define VDD_3V3_EN_GPIO            22


static MipiConfiguration imx456_cx3_mipi_cfg = {
	.data_format = 0x2C,
	.num_datalanes = 4,
	.pll_prd = 2,
	.pll_fbd = 119,
	.pll_frs = 1,
	.csi_rx_clk_div = 1,
	.par_clk_div = 1,
	.mclk_ctl = 0,
	.mclk_ref_div = 2,
	.hresolution = 1280,
	.fifo_delay = 50,
	.pll_clock = 384,
	.mclk = 8,
};

#elif  (USE_WHICH_CONVERTER==kConverterIsDuxin)
static char *test_name = "use_duxin";

#define dothin_enable_softpin      tops_t.ap_ops.EnableSoftPin
#define dothin_enable_gpio         tops_t.ap_ops.EnableGpio
#define dothin_set_sensor_clock    tops_t.ap_ops.SetSensorClock
#define dothin_set_softpin_pullup  tops_t.ap_ops.SetSoftPinPullUp
#define dothin_set_sensor_i2c_rate tops_t.ap_ops.SetSensorI2cRate
#define dothin_sensor_enable       tops_t.ap_ops.SensorEnable
#define dothin_pmu_set_voltage     tops_t.ap_ops.PmuSetVoltage
#define dothin_set_gpio_level      tops_t.ap_ops.SetGpioPinLevel
#define dothin_set_gpio_dir        tops_t.ap_ops.SetGpioPinDir
#endif

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
	dothin_set_gpio_level(pin, level, dothin_device_id);
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

static int sensor_write_reg(uint16_t reg, uint8_t value)
{
    int rtn = i2c_reg_write(SENSOR_ADDR, reg, 2, value, 1);
    return rtn;
}

static int sensor_read_reg(uint16_t reg, uint8_t *value)
{
    int rtn = i2c_reg_read(SENSOR_ADDR, reg, 2, value, 1);
    return rtn;
}

static int eeprom_write_byte(uint16_t reg, uint8_t value)
{
    int rtn = i2c_reg_write(EEPROM_ADDR, reg, 2, value, 1);
    return rtn;

}

static int eeprom_read_byte(uint16_t reg, uint8_t *value)
{
    int rtn = i2c_reg_read(EEPROM_ADDR, reg, 2, value, 1);
    return rtn;

}

static int gpio_expander_init() // power on
{
	int rtn = 0;
    rtn = gpio_control(GPIO_EXPANDER_RST_PIN, 1); // pull gpio expander rst pin to high
    rtn = i2c_reg_write(GPIO_EXPANDER_ADDR, 0x03, 1, 0x00, 1); // set all gpio as output
    rtn = i2c_reg_write(GPIO_EXPANDER_ADDR, 0x01, 1, 0xFF, 1); // set all gpio to high
    return rtn;
}

static int gpio_expander_deinit() // power off
{
	int rtn = 0;
	rtn = gpio_control(GPIO_EXPANDER_RST_PIN, 1); // pull gpio expander rst pin to high
	rtn = i2c_reg_write(GPIO_EXPANDER_ADDR, 0x03, 1, 0x00, 1); // set all gpio as output
	rtn = i2c_reg_write(GPIO_EXPANDER_ADDR, 0x01, 1, 0x00, 1); // set all gpio to low
	return rtn;
}

static int vcsel_driver_write_reg(uint8_t reg, uint8_t value)
{
    int rtn = 0;
    rtn = sensor_write_reg(0x0403, 0x20); // send1 channel, start by register trig
    rtn = sensor_write_reg(0x0407, 0x00); // send1 start pointer of Tx buffer
    rtn = sensor_write_reg(0x0500, 0x02); // number of byte to be write
    rtn = sensor_write_reg(0x0501, reg);  // address to be write to laser (write address = laser addr)
    rtn = sensor_write_reg(0x0502, value);// value to be write to laser
    rtn = sensor_write_reg(0x0401, 0x01); // enable
    rtn = sensor_write_reg(0x0400, 0x01); // trig
    rtn = sensor_write_reg(0x0401, 0x20); // disable
    return rtn;
}

static int vcsel_driver_read_reg(uint8_t reg, uint8_t *value)
{
    int rtn = 0;
    rtn = sensor_write_reg(0x0433, 0x20); // general channel, start by register trig
    rtn = sensor_write_reg(0x0436, 0x00); // start pointer of Rx buffer
    rtn = sensor_write_reg(0x0437, 0x00); // start pointer of Tx buffer
    rtn = sensor_write_reg(0x043A, 0x01); // 1st byte is Tx, after Rx
    rtn = sensor_write_reg(0x0500, 0x02); // number of byte to be write and read
    rtn = sensor_write_reg(0x0501, reg + 0x80); // address to be read from laser (read address = laser addr + 0x80)
    rtn = sensor_write_reg(0x0431, 0x01); // enable
    rtn = sensor_write_reg(0x0430, 0x01); // trig
    rtn = sensor_write_reg(0x0431, 0x00); // disable

    rtn = sensor_read_reg(0x0580, value); // first byte in Rx buffer

    return rtn;
}

int imx456_get_rx_temp(float *temperature)
{
    uint8_t value;
    int rtn = sensor_read_reg(0x1403, &value);
										
    *temperature = (float) (value&0xff) - 40;

    return rtn;
}

int imx456_get_tx_temp(float *temperature)
{
	int rtn = 0;/**/
    rtn = vcsel_driver_write_reg(0x13, 0x23); // start adc
    rtn = sensor_write_reg(0x0423, 0x20); // thermo channel, start by register trig
    rtn = sensor_write_reg(0x0426, 0x00); // start pointer of Rx buffer
    rtn = sensor_write_reg(0x0427, 0x00); // start pointer of Tx buffer
    rtn = sensor_write_reg(0x042A, 0x01); // 1st byte is Tx, after Rx
    rtn = sensor_write_reg(0x0500, 0x03); // number of byte to be write and read
    rtn = sensor_write_reg(0x0501, 0x14 + 0x80); // address to be read from laser (read address = laser temperature addr 0x14 + 0x80)
    rtn = sensor_write_reg(0x0421, 0x01); // enable
    rtn = sensor_write_reg(0x0420, 0x01); // trig
    rtn = sensor_write_reg(0x0421, 0x00); // disable

    uint8_t valueH, valueL;
    rtn = sensor_read_reg(0x0580, &valueH); // first byte in Rx buffer
    rtn = sensor_read_reg(0x0581, &valueL); // second byte in Rx buffer

    uint16_t value = ((valueH & 0x03) << 8) + valueL;

	*temperature = 25 + ((value - 296)) / 5.4f;

    //printf("value %d,  valueH %d, valueL %d    tx=%f  \n", value, valueH, valueL, *temperature);
				 
	return 0;
}

int imx456_get_vcsel_pd(uint32_t *value) // driver ic pd value
{
	int rtn = 0;
	uint8_t valueH, valueL;
	uint16_t pd_h2;
	if (driver_ic_type == DRIVER_IC_DW9912) {
		rtn = vcsel_driver_read_reg(0x1B, &valueH); // APC2_CHECK_DATA[9:8]
		rtn = vcsel_driver_read_reg(0x22, &valueL); // APC2_CHECK_DATA[7:0]
		pd_h2 = valueL + ((valueH >> 4) & 0x03) * 256;
	}
	else {
		rtn = vcsel_driver_read_reg(0x1B, &valueH); // PD_H2[9:8]
		rtn = vcsel_driver_read_reg(0x20, &valueL); // PD_H2[7:0]
		pd_h2 = valueL + (valueH & 0x03) * 256;
	}
	rtn = vcsel_driver_read_reg(0x1A, &valueH); // PD_BG[9:8]
	rtn = vcsel_driver_read_reg(0x1C, &valueL); // PD_BG[7:0]
	uint16_t pd_bg = valueL + (valueH & 0x03) * 256;
	//printf("value %d,  valueH %d, valueL %d  \n", value, valueH, valueL);
	*value = (pd_h2 << 16) + pd_bg;
	return rtn;
}

int imx456_set_illum_power(uint8_t vcsel_num, uint8_t value_A, uint8_t value_B)
{
	int rtn = 0;
    rtn = sensor_write_reg(0x0403, 0x20);
    rtn = sensor_write_reg(0x0407, 0x00);
    rtn = sensor_write_reg(0x0500, 0x03); // number of byte to be write
    rtn = sensor_write_reg(0x0501, 0x07);
    rtn = sensor_write_reg(0x0502, value_B); // IBIAS
    rtn = sensor_write_reg(0x0503, value_A); // ISW
    rtn = sensor_write_reg(0x0401, 0x01);
    rtn = sensor_write_reg(0x0400, 0x01);
    rtn = sensor_write_reg(0x0401, 0x20);
    return rtn;	
}

int imx456_get_illum_power(uint8_t *vcsel_num, uint8_t *value_A, uint8_t *value_B)
{
	int rtn = 0;
    *vcsel_num = 1;

    rtn = sensor_write_reg(0x0433, 0x20); // general channel, start by register trig
    rtn = sensor_write_reg(0x0436, 0x00); // start pointer of Rx buffer
    rtn = sensor_write_reg(0x0437, 0x00); // start pointer of Tx buffer
    rtn = sensor_write_reg(0x043A, 0x01); // 1st byte is Tx, after Rx
    rtn = sensor_write_reg(0x0500, 0x03); // number of byte to be write and read
    rtn = sensor_write_reg(0x0501, 0x07 + 0x80); // address to be read from laser (read address = laser addr 0x07 + 0x80)
    rtn = sensor_write_reg(0x0431, 0x01); // enable
    rtn = sensor_write_reg(0x0430, 0x01); // trig
    rtn = sensor_write_reg(0x0431, 0x00); // disable

    rtn = sensor_read_reg(0x0580, value_B); // first byte in Rx buffer
    rtn = sensor_read_reg(0x0581, value_A); // second byte in Rx buffer
    return rtn;
}

int imx456_illum_power_test_init()
{
	int rtn = 0;
	rtn = vcsel_driver_write_reg(0x00, 0x1C); // disable APC
	rtn = sensor_write_reg(0x21E8, 0x01); // 1 phase
	rtn = sensor_write_reg(0x21B0, 0xFF); // no pulse modulation(vcsel constant light when integration)
	rtn = imx456_set_stream_mode();
	rtn = imx456_set_fps(20);
		
	use_trigger_mode = 0;
	
	return rtn;
}

int imx456_illum_power_test(uint8_t mode)
{
	int rtn = 0;
	if (mode) { // illum power test mode
		rtn = vcsel_driver_write_reg(0x00, 0x1C); // disable APC
	}
	else { 
		rtn = vcsel_driver_write_reg(0x00, 0x0C); // enable APC
	}

	return rtn;
}

/*
int imx456_illum_power_test(uint8_t mode)
{
	int rtn = 0;
	rtn = imx456_video_streaming(0); // disable stream first
	if (mode) { // illum power test mode
		rtn = sensor_write_reg(0x21E8, 0x01); // 1 phase
		rtn = sensor_write_reg(0x21B0, 0xFF); // no pulse modulation(vcsel constant light when integration)
		rtn = imx456_set_stream_mode();
		rtn = imx456_set_fps(60);
		rtn = vcsel_driver_write_reg(0x00, 0x1C); // disable APC
		use_trigger_mode = 0;
	}
	else { // default depth mode
		rtn = sensor_write_reg(0x21E8, 0x04); // 4 phase
		rtn = sensor_write_reg(0x21B0, 0x00); // pulse modulation
		rtn = imx456_set_trigger_mode(1);
		rtn = vcsel_driver_write_reg(0x00, 0x0C); // enable APC
		use_trigger_mode = 1;
	}
	rtn = imx456_video_streaming(1); // enable stream

	return rtn;
}
*/
int imx456_hardware_trigger() // not used
{/*
    //mdevices.gpio_control(TRIGGER_PIN, 1);
    mdevices.gpio_control(TRIGGER_PIN, 0); // active low
    mdevices.sleepms(0.001); // 1us, not accurate
    //mdevices.usleep(100);
    mdevices.gpio_control(TRIGGER_PIN, 1);
    */
    return 0;
}

int imx456_software_trigger() // checked
{
	int rtn = 0;
	if(use_trigger_mode){
		rtn = sensor_write_reg(0x2100, 0x01);
	}
    return rtn;
}

int imx456_illum_power_control(bool enable)
{
	int rtn = 0;
#if  (USE_WHICH_CONVERTER==kConverterIsDuxin)
	if (enable) {
		rtn = gpio_expander_init();// power on
		if (rtn < 0)
			return rtn;

		rtn = imx456_dothin_config();
		uint8_t value;
		rtn = sensor_read_reg(0x0001, &value); // value should be 0x55
		if (rtn < 0)
			return rtn;

		rtn = imx456_init();
		rtn = imx456_video_streaming(1);
	}
	else {
		rtn = gpio_expander_deinit(); // power off
	}
#endif
	return rtn;
}


struct regList reglist_common[] = {
	{ 0x1006,0x08 },
	{ 0x1007,0x00 },
	{ 0x1000,0x00 },
	{ 0x1040,0x00 },
	{ 0x1041,0x96 },
	{ 0x1043,0x00 },
	{ 0x10D2,0x00 },
	{ 0x10D3,0x10 },
	{ 0x1448,0x06 },
	{ 0x1449,0x40 },
	{ 0x144A,0x06 },
	{ 0x144B,0x40 },
	{ 0x144C,0x06 },
	{ 0x144D,0x40 },
	{ 0x144E,0x06 },
	{ 0x144F,0x40 },
	{ 0x1450,0x06 },
	{ 0x1451,0x40 },
	{ 0x1452,0x06 },
	{ 0x1453,0x40 },
	{ 0x1454,0x06 },
	{ 0x1455,0x40 },
	{ 0x1456,0x06 },
	{ 0x1457,0x40 },
	{ 0x2202,0x00 },
	{ 0x2203,0x1E },
	{ 0x2C08,0x01 },
	{ 0x2C0C,0xA0 },
	{ 0x2C0D,0x00 },
	{ 0x3C2B,0x1B },
	{ 0x400E,0x01 },
	{ 0x400F,0x81 },
	{ 0x40D1,0x00 },
	{ 0x40D2,0x00 },
	{ 0x40D3,0x00 },
	{ 0x40DB,0x3F },
	{ 0x40DE,0x40 },
	{ 0x40DF,0x01 },
	{ 0x4134,0x04 },
	{ 0x4135,0x04 },
	{ 0x4136,0x04 },
	{ 0x4137,0x04 },
	{ 0x4138,0x04 },
	{ 0x4139,0x04 },
	{ 0x413A,0x04 },
	{ 0x413B,0x04 },
	{ 0x413C,0x04 },
	{ 0x4146,0x01 },
	{ 0x4147,0x01 },
	{ 0x4148,0x01 },
	{ 0x4149,0x01 },
	{ 0x414A,0x01 },
	{ 0x414B,0x01 },
	{ 0x414C,0x01 },
	{ 0x414D,0x01 },
	{ 0x4158,0x01 },
	{ 0x4159,0x01 },
	{ 0x415A,0x01 },
	{ 0x415B,0x01 },
	{ 0x415C,0x01 },
	{ 0x415D,0x01 },
	{ 0x415E,0x01 },
	{ 0x415F,0x01 },
	{ 0x4590,0x00 },
	{ 0x4591,0x2E },
	{ 0x4684,0x00 },
	{ 0x4685,0xA0 },
	{ 0x4686,0x00 },
	{ 0x4687,0xA1 },
	{ 0x471E,0x07 },
	{ 0x471F,0xC9 },
	{ 0x473A,0x07 },
	{ 0x473B,0xC9 },
	{ 0x4770,0x00 },
	{ 0x4771,0x00 },
	{ 0x4772,0x1F },
	{ 0x4773,0xFF },
	{ 0x4778,0x06 },
	{ 0x4779,0xA4 },
	{ 0x477A,0x07 },
	{ 0x477B,0xAE },
	{ 0x477C,0x0A },
	{ 0x477D,0xD6 },
	{ 0x4788,0x06 },
	{ 0x4789,0xA4 },
	{ 0x478C,0x1F },
	{ 0x478D,0xFF },
	{ 0x478E,0x00 },
	{ 0x478F,0x00 },
	{ 0x4792,0x00 },
	{ 0x4793,0x00 },
	{ 0x4796,0x00 },
	{ 0x4797,0x00 },
	{ 0x479A,0x00 },
	{ 0x479B,0x00 },
	{ 0x479C,0x1F },
	{ 0x479D,0xFF },
	{ 0x479E,0x00 },
	{ 0x479F,0x00 },
	{ 0x47A2,0x00 },
	{ 0x47A3,0x00 },
	{ 0x47A6,0x00 },
	{ 0x47A7,0x00 },
	{ 0x47AA,0x00 },
	{ 0x47AB,0x00 },
	{ 0x47AC,0x1F },
	{ 0x47AD,0xFF },
	{ 0x47AE,0x00 },
	{ 0x47AF,0x00 },
	{ 0x47B2,0x00 },
	{ 0x47B3,0x00 },
	{ 0x47B6,0x00 },
	{ 0x47B7,0x00 },
	{ 0x47BA,0x00 },
	{ 0x47BB,0x00 },
	{ 0x47BC,0x1F },
	{ 0x47BD,0xFF },
	{ 0x47BE,0x00 },
	{ 0x47BF,0x00 },
	{ 0x47C2,0x00 },
	{ 0x47C3,0x00 },
	{ 0x47C6,0x00 },
	{ 0x47C7,0x00 },
	{ 0x47CA,0x00 },
	{ 0x47CB,0x00 },
	{ 0x4834,0x00 },
	{ 0x4835,0xA0 },
	{ 0x4836,0x00 },
	{ 0x4837,0xA1 },
	{ 0x4878,0x00 },
	{ 0x4879,0xA0 },
	{ 0x487A,0x00 },
	{ 0x487B,0xA1 },
	{ 0x48BC,0x00 },
	{ 0x48BD,0xA0 },
	{ 0x48BE,0x00 },
	{ 0x48BF,0xA1 },
	{ 0x4954,0x00 },
	{ 0x4955,0xA0 },
	{ 0x4956,0x00 },
	{ 0x4957,0xA1 },
	{ 0x4984,0x00 },
	{ 0x4985,0xA0 },
	{ 0x4986,0x00 },
	{ 0x4987,0xA1 },
	{ 0x49B8,0x00 },
	{ 0x49B9,0x78 },
	{ 0x49C2,0x00 },
	{ 0x49C3,0x3C },
	{ 0x49C8,0x00 },
	{ 0x49C9,0x76 },
	{ 0x49D2,0x00 },
	{ 0x49D3,0x3F },
	{ 0x49DC,0x00 },
	{ 0x49DD,0xA0 },
	{ 0x49DE,0x00 },
	{ 0x49DF,0xA1 },
	{ 0x49EE,0x00 },
	{ 0x49EF,0x78 },
	{ 0x49F8,0x00 },
	{ 0x49F9,0x3C },
	{ 0x49FE,0x00 },
	{ 0x49FF,0x78 },
	{ 0x4A04,0x00 },
	{ 0x4A05,0x3C },
	{ 0x4A0A,0x00 },
	{ 0x4A0B,0x76 },
	{ 0x4A10,0x00 },
	{ 0x4A11,0x3F },
	{ 0x4A1A,0x00 },
	{ 0x4A1B,0xA0 },
	{ 0x4A1C,0x00 },
	{ 0x4A1D,0xA1 },
	{ 0x4A1E,0x00 },
	{ 0x4A1F,0x78 },
	{ 0x4A28,0x00 },
	{ 0x4A29,0x3C },
	{ 0x4A4A,0x00 },
	{ 0x4A4B,0xA0 },
	{ 0x4A4C,0x00 },
	{ 0x4A4D,0xA1 },
	{ 0x4A7A,0x00 },
	{ 0x4A7B,0xA0 },
	{ 0x4A7C,0x00 },
	{ 0x4A7D,0xA1 },
	{ 0x4AEE,0x00 },
	{ 0x4AEF,0xA0 },
	{ 0x4AF0,0x00 },
	{ 0x4AF1,0xA1 },
	{ 0x4B2E,0x00 },
	{ 0x4B2F,0xA0 },
	{ 0x4B30,0x00 },
	{ 0x4B31,0xA1 },
	{ 0x4B5A,0x00 },
	{ 0x4B5B,0xA0 },
	{ 0x4B5C,0x00 },
	{ 0x4B5D,0xA1 },
	{ 0x4B86,0x00 },
	{ 0x4B87,0xA0 },
	{ 0x4B88,0x00 },
	{ 0x4B89,0xA1 },
	{ 0x4B9E,0x00 },
	{ 0x4B9F,0x1A },
	{ 0x4BAE,0x00 },
	{ 0x4BAF,0x1A },
	{ 0x4BB6,0x00 },
	{ 0x4BB7,0x1A },
	{ 0x4BC6,0x00 },
	{ 0x4BC7,0x1A },
	{ 0x4BCE,0x00 },
	{ 0x4BCF,0x1A },
	{ 0x4BEE,0x00 },
	{ 0x4BEF,0xA0 },
	{ 0x4BF0,0x00 },
	{ 0x4BF1,0xA1 },
	{ 0x4BF6,0x00 },
	{ 0x4BF7,0x1A },
	{ 0x4C00,0x00 },
	{ 0x4C01,0x1A },
	{ 0x4C58,0x00 },
	{ 0x4C59,0xA0 },
	{ 0x4C5A,0x00 },
	{ 0x4C5B,0xA1 },
	{ 0x4C6E,0x00 },
	{ 0x4C6F,0xA0 },
	{ 0x4C70,0x00 },
	{ 0x4C71,0xA1 },
	{ 0x4C7A,0x01 },
	{ 0x4C7B,0x35 },
	{ 0x4CF2,0x07 },
	{ 0x4CF3,0xC9 },
	{ 0x4CF8,0x06 },
	{ 0x4CF9,0x9B },
	{ 0x4CFA,0x07 },
	{ 0x4CFB,0xAE },
	{ 0x4CFE,0x07 },
	{ 0x4CFF,0xC9 },
	{ 0x4D04,0x06 },
	{ 0x4D05,0x98 },
	{ 0x4D06,0x07 },
	{ 0x4D07,0xB1 },
	{ 0x4D18,0x06 },
	{ 0x4D19,0xA4 },
	{ 0x4D1A,0x07 },
	{ 0x4D1B,0x49 },
	{ 0x4D1E,0x07 },
	{ 0x4D1F,0xC9 },
	{ 0x4D2A,0x07 },
	{ 0x4D2B,0xC9 },
	{ 0x4D4A,0x07 },
	{ 0x4D4B,0xC9 },
	{ 0x4D50,0x06 },
	{ 0x4D51,0x9B },
	{ 0x4D52,0x07 },
	{ 0x4D53,0xAE },
	{ 0x4D56,0x07 },
	{ 0x4D57,0xC9 },
	{ 0x4D5C,0x06 },
	{ 0x4D5D,0x98 },
	{ 0x4D5E,0x07 },
	{ 0x4D5F,0xB1 },
	{ 0x4D70,0x06 },
	{ 0x4D71,0xA4 },
	{ 0x4D72,0x07 },
	{ 0x4D73,0x49 },
	{ 0x4D78,0x06 },
	{ 0x4D79,0xA4 },
	{ 0x4D7A,0x07 },
	{ 0x4D7B,0xAE },
	{ 0x4D7C,0x1F },
	{ 0x4D7D,0xFF },
	{ 0x4D7E,0x1F },
	{ 0x4D7F,0xFF },
	{ 0x4D80,0x06 },
	{ 0x4D81,0xA4 },
	{ 0x4D82,0x07 },
	{ 0x4D83,0xAE },
	{ 0x4D84,0x1F },
	{ 0x4D85,0xFF },
	{ 0x4D86,0x1F },
	{ 0x4D87,0xFF },
	{ 0x4E39,0x07 },
	{ 0x4E7B,0x64 },
	{ 0x4E8E,0x0E },
	{ 0x4E9C,0x01 },
	{ 0x4EA1,0x03 },
	{ 0x4EA5,0x00 },
	{ 0x4EA7,0x00 },
	{ 0x4F05,0x04 },
	{ 0x4F0D,0x04 },
	{ 0x4F15,0x04 },
	{ 0x4F19,0x01 },
	{ 0x4F20,0x01 },
	{ 0x500F,0x01 },
	{ 0x5224,0x00 },
	{ 0x5225,0x2F },
	{ 0x5226,0x00 },
	{ 0x5227,0x1E },
	{ 0x5230,0x00 },
	{ 0x5231,0x19 },
	{ 0x5244,0x00 },
	{ 0x5245,0x07 },
	{ 0x5252,0x07 },
	{ 0x5253,0x08 },
	{ 0x5254,0x07 },
	{ 0x5255,0xB4 },
	{ 0x5271,0x00 },
	{ 0x5272,0x04 },
	{ 0x5273,0x2E },
	{ 0x5285,0x00 },
	{ 0x5286,0x00 },
	{ 0x5287,0x5D },

	// 100MHZ
	{ 0x0800,0x05 },
	{ 0x0801,0x14 },
	{ 0x0804,0x00 },
	{ 0x0805,0x01 },
	{ 0x0806,0x02 },
	{ 0x0807,0x80 },
	{ 0x0808,0x00 },
	{ 0x0809,0x00 },
	{ 0x080A,0x00 },
	{ 0x080B,0xF1 },
	{ 0x080C,0x00 },
	{ 0x080D,0x00 },
	{ 0x080E,0x01 },
	{ 0x0828,0x04 },
	{ 0x100C,0x0F },
	{ 0x100D,0x00 },
	{ 0x100E,0x00 },
	{ 0x100F,0x00 },
	{ 0x1010,0x03 },
	{ 0x1016,0x03 },
	{ 0x1017,0x00 },
	{ 0x1042,0x01 },
	{ 0x1044,0x00 },
	{ 0x1045,0x78 },
	{ 0x1046,0x01 },
	{ 0x1047,0x00 },
	{ 0x1048,0x00 },
	{ 0x1049,0x64 },
	{ 0x104A,0x01 },
	{ 0x104B,0x02 },
	{ 0x1060,0x00 },
	{ 0x1071,0x06 },
	{ 0x10C2,0x00 },
	{ 0x10C3,0x0A },
	{ 0x10C4,0x00 },
	{ 0x10C5,0x62 },
	{ 0x10D0,0x0B },
	{ 0x10D4,0x00 },
	{ 0x10D5,0x9C },
	{ 0x1433,0x01 },
	{ 0x14A5,0x00 },
	{ 0x14BB,0x00 },
	{ 0x1C40,0x01 },
	{ 0x2020,0x01 },
	{ 0x2100,0x08 },
	{ 0x2108,0x00 },
	{ 0x2109,0x00 },
	{ 0x210A,0x0B },
	{ 0x210B,0xCC },
	{ 0x2120,0x00 },
	{ 0x2121,0x01 },
	{ 0x2122,0xD4 },
	{ 0x2123,0xC0 },
	{ 0x2124,0x00 },
	{ 0x2125,0x01 },
	{ 0x2126,0xD4 },
	{ 0x2127,0xC0 },
	{ 0x2128,0x00 },
	{ 0x2129,0x01 },
	{ 0x212A,0xD4 },
	{ 0x212B,0xC0 },
	{ 0x212C,0x00 },
	{ 0x212D,0x01 },
	{ 0x212E,0xD4 },
	{ 0x212F,0xC0 },
	{ 0x21B4,0x40 },
	{ 0x21B5,0x62 },
	{ 0x21BE,0x00 },
	{ 0x21BF,0x00 },
	{ 0x21C4,0xFF },
	{ 0x21E8,0x04 },
	{ 0x2F05,0x01 },
	{ 0x2F06,0x09 },
	{ 0x2F07,0x7A },
	{ 0x3071,0x00 },
	{ 0x4010,0x37 },
	{ 0x4015,0x00 },
	{ 0x4016,0x05 },
	{ 0x412C,0x00 },
	{ 0x4E9A,0x00 },
	{ 0x4EA0,0x00 },
	{ 0x5265,0x00 },
	{ 0x5266,0x0D },
	{ 0x5267,0x04 },
	{ 0x5281,0x00 },
	{ 0x5282,0x04 },
	{ 0x5283,0x2E },

	{ 0x1433,0x00 }, // no error flag
	{ 0x3C18,0x02 }, // 2 line EBD
	{ 0x2C0C,0x10 },
	{ 0x2C0D,0x80 },
	{ 0x0450,0x47 }, // 3-wire spi protocal setting
	{ 0x4016,0x15 }, // reset sequence lie time
};

static struct regList reglist[] = {

    //External clock frequency [MHz]
    {0x1006, 0x08},
    {0x1007, 0x00},
    {0x1000, 0x00},
    {0x0800, 0x05},
    {0x0801, 0x14},
    {0x0828, 0x04},
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
    {0x201C, 0x08},
    {0x201E, 0x06},
    {0x2020, 0x01},
    {0x2100, 0x08},
    {0x2202, 0x00},
    {0x2203, 0x1E},
    {0x2c08, 0x00},
    {0x2C0D, 0x80},
    {0x2F05, 0x01},
    {0x2F06, 0x09},
    {0x2F07, 0x7A},
    {0x3071, 0x00},
    {0x4010, 0x34},
    {0x400E, 0x01},
    {0x400F, 0x81},
    {0x4016, 0x05},
    {0x40D1, 0x00},
    {0x40D2, 0x00},
    {0x40D3, 0x00},
    {0x40DB, 0x3F},
    {0x40DF, 0x01},
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
    {0x4591, 0x2E},
    {0x4684, 0x00},
    {0x4685, 0xA0},
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
    {0x4837, 0xA1},
    {0x4878, 0x00},
    {0x4879, 0xA0},
    {0x487B, 0xA1},
    {0x48BC, 0x00},
    {0x48BD, 0xA0},
    {0x48BF, 0xA1},
    {0x4954, 0x00},
    {0x4955, 0xA0},
    {0x4957, 0xA1},
    {0x4984, 0x00},
    {0x4985, 0xA0},
    {0x4987, 0xA1},
    {0x49B9, 0x78},
    {0x49C3, 0x3C},
    {0x49C9, 0x76},
    {0x49D3, 0x3F},
    {0x49DC, 0x00},
    {0x49DD, 0xA0},
    {0x49DF, 0xA1},
    {0x49EF, 0x78},
    {0x49F9, 0x3C},
    {0x49FF, 0x78},
    {0x4A05, 0x3C},
    {0x4A0B, 0x76},
    {0x4A11, 0x3F},
    {0x4A1A, 0x00},
    {0x4A1B, 0xA0},
    {0x4A1D, 0xA1},
    {0x4A1F, 0x78},
    {0x4A29, 0x3C},
    {0x4A4A, 0x00},
    {0x4A4B, 0xA0},
    {0x4A4D, 0xA1},
    {0x4A7A, 0x00},
    {0x4A7B, 0xA0},
    {0x4A7D, 0xA1},
    {0x4AEE, 0x00},
    {0x4AEF, 0xA0},
    {0x4AF1, 0xA1},
    {0x4B2E, 0x00},
    {0x4B2F, 0xA0},
    {0x4B31, 0xA1},
    {0x4B5A, 0x00},
    {0x4B5B, 0xA0},
    {0x4B5D, 0xA1},
    {0x4B86, 0x00},
    {0x4B87, 0xA0},
    {0x4B89, 0xA1},
    {0x4B9F, 0x1A},
    {0x4BAF, 0x1A},
    {0x4BB7, 0x1A},
    {0x4BC7, 0x1A},
    {0x4BCF, 0x1A},
    {0x4BEE, 0x00},
    {0x4BEF, 0xA0},
    {0x4BF1, 0xA1},
    {0x4BF7, 0x1A},
    {0x4C01, 0x1A},
    {0x4C58, 0x00},
    {0x4C59, 0xA0},
    {0x4C5B, 0xA1},
    {0x4C6E, 0x00},
    {0x4C6F, 0xA0},
    {0x4C71, 0xA1},
    {0x4c7A, 0x01},
    {0x4c7B, 0x35},
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
    {0x4E9C, 0x01},
    {0x4EA0, 0x00},
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
    {0x5100, 0x84},
    {0x5101, 0x15},
    {0x5102, 0x18},
    {0x5103, 0x40},
    {0x5104, 0x11},
    {0x5105, 0xBD},
    {0x5106, 0x80},
    {0x5107, 0x1B},
    {0x5108, 0x72},
    {0x5109, 0x1C},
    {0x510A, 0x80},
    {0x510B, 0x00},
    {0x510C, 0x00},
    {0x510D, 0x00},
    {0x510E, 0x60},
    {0x510F, 0x60},
    {0x5110, 0x01},
    {0x5111, 0xE0},
    {0x5113, 0x0A},
    {0x5225, 0x2F},
    {0x5227, 0x1E},
    {0x5231, 0x19},
    {0x5252, 0x07},
    {0x5253, 0x08},
    {0x5254, 0x07},
    {0x5255, 0xB4},
    {0x5266, 0x05},
    {0x5267, 0x68},
    {0x5272, 0x04},
    {0x5273, 0x2E},
    // mode setting, 960Mbps_8MHz_60fps Fmod20MHZ
    {0x0800, 0x02},
    {0x0801, 0xB6},
    {0x0804, 0x00},
    {0x0805, 0x01},
    {0x0806, 0x02},
    {0x0807, 0x80},
    {0x0808, 0x00},
    {0x0809, 0x00},
    {0x080A, 0x00},
    {0x080B, 0xF1},
    {0x080C, 0x00},
    {0x080D, 0x00},
    {0x0828, 0x00},
    {0x1010, 0x03},
    {0x14A5, 0x00},
    {0x21e8, 0x04},
    {0x21B4, 0x40},
    {0x21B5, 0x62},
    {0x21B6, 0x00},
    {0x21B7, 0x00},
    {0x1042, 0x01},
    {0x1046, 0x01},
    {0x1048, 0x00},
    {0x1049, 0x50},
    {0x104A, 0x01},
    {0x104B, 0x02},
    {0x21BE, 0x00},
    {0x21BF, 0x02},
    {0x10D0, 0x09},
    {0x10D3, 0x10},
    {0x10D5, 0xC5},
    {0x100C, 0x0F},
    {0x100D, 0x00},
    {0x100E, 0x00},
    {0x100F, 0x00},
    {0x4010, 0x5F},
    {0x4015, 0x00},
    {0x4016, 0x28},
    {0x5281, 0x00},
    {0x5282, 0x5E},
    {0x5283, 0xD6},
    {0x5285, 0x00},
    {0x5286, 0x00},
    {0x5287, 0x5D},
    {0x2108, 0x00},
    {0x2109, 0x00},
    {0x210a, 0x0A},
    {0x210b, 0xE1},
    {0x14bb, 0x01},
    {0x2120, 0x00},
    {0x2121, 0x00},
    {0x2122, 0xBB},
    {0x2123, 0x80},
    {0x2124, 0x00},
    {0x2125, 0x00},
    {0x2126, 0xBB},
    {0x2127, 0x80},
    {0x2128, 0x00},
    {0x2129, 0x00},
    {0x212a, 0xBB},
    {0x212b, 0x80},
    {0x212c, 0x00},
    {0x212d, 0x00},
    {0x212e, 0xBB},
    {0x212f, 0x80},
    {0x1433, 0x00}, // no error flag
	{0x3c18, 0x02}, // 2 line EBD
    {0x21c4, 0xff}, // LED enable

    //start streaming
    //{0x1001,  0x01},

};

static struct regList cxa4016_reglist[] = {

	//Laser Driver (CXA4016)  ADC initialization
	{ 0x0403,  0x20 },
	{ 0x0405,  0x00 },
	{ 0x0450,  0x47 },
	{ 0x0500,  0x02 },
	{ 0x0501,  0x13 },
	{ 0x0502,  0x23 }, //0x27 },
	{ 0x0401,  0x01 },
	{ 0x0400,  0x01 },
	{ 0x0401,  0x00 },

	//Laser Driver(CXA4016) reset
	{ 0x0403,  0x20 },
	{ 0x0405,  0x00 },
	{ 0x0450,  0x47 },
	{ 0x0500,  0x02 },
	{ 0x0501,  0x25 },
	{ 0x0502,  0x01 },
	{ 0x0401,  0x01 },
	{ 0x0400,  0x01 },
	{ 0x0401,  0x00 },

	//Sony Laser Driver (CXA4016)  initial
	{ 0x0403,  0x20 },
	{ 0x0405,  0x00 },
	{ 0x0407,  0x03 },
	{ 0x0503,  0x15 },
	{ 0x0504,  0x00 },
	{ 0x0505,  0x0C }, // [driver IC addr : 0x00 APC_MODE] default 0x0C
	{ 0x0506,  0xA0 },
	{ 0x0507,  0xFF },
	{ 0x0508,  0x1A },
	{ 0x0509,  0x28 },
	{ 0x050A,  0x69 },
	{ 0x050B,  0x87 },
	{ 0x050C,  0x00 }, // [driver IC addr : 0x07 IBIAS_FIX]
	{ 0x050D,  0xEE }, // [driver IC addr : 0x08 ISW_FIX]   // default 0xFF
	{ 0x050E,  0x14 },
	{ 0x050F,  0x05 },
	{ 0x0510,  0xA4 },
	{ 0x0511,  0x28 },
	{ 0x0512,  0xc3 },
	{ 0x0513,  0xc7 }, // 0x0b ISW_FIX threshold
	{ 0x0514,  0xAA }, // [driver IC addr : 0x0F AS_I AS_W]
	{ 0x0515,  0x0F }, // [driver IC addr : 0x10 Tr_AS]
	{ 0x0516,  0x00 },
	{ 0x0517,  0x00 },
	{ 0x0518,  0x01 },
	{ 0x0401,  0x01 },
	{ 0x0400,  0x01 },
	{ 0x0401,  0x00 },

	{ 0x0403,  0x20 },
	{ 0x0405,  0x00 },
	{ 0x0407,  0x22 },
	{ 0x0522,  0x02 },
	{ 0x0523,  0x23 },
	{ 0x0524,  0x00 },// clear driver ic error
	{ 0x0401,  0x01 },
	{ 0x0400,  0x01 },
	{ 0x0401,  0x00 },

	/**/
	// Laser Driver (CXA4016) ADC auto start
	{ 0x0413,  0x20 }, // send2 start by internal sync signal   0xA0
	{ 0x0417,  0x19 }, // send2 start pointer of Tx buffer
	{ 0x0519,  0x02 }, // send2 total write byte
	{ 0x051A,  0x13 }, // laser adc address 0x13
	{ 0x051B,  0x23 }, // start adc
	//{0x0411,  0x01}, // send2 enable


					   // Laser Driver (CXA4016) temperature auto read
	{ 0x0423,  0x20 }, // thermo start by internal sync signal   0xA0
	{ 0x0426,  0x10 }, // thermo start pointer of Rx buffer
	{ 0x0427,  0x1C }, // thermo start pointer of Tx buffer
	{ 0x042A,  0x01 }, // thermo 1st byte is Tx, after Rx
	{ 0x051C,  0x03 }, // number of byte to be write and read
	{ 0x051D,  0x94 }, // temperature address to be read from laser (read address = laser addr 0x14 + 0x80)
	//{0x0421,  0x01}, // thermo enable

	/*
	{0x0423,  0xA0},
	{0x0425,  0x04},// transaction x5
	{0x0427,  0x1C},
	{0x042a,  0x01},




	{0x051C,  0x02},// write and read total 14 byte
	{0x051D,  0xA3},
	{0x051E,  0x12},
	{0x051F,  0x94},
	{0x0520,  0x04},
	{0x0521,  0x80},
	{0x0522,  0x08},
	{0x0523,  0x85},
	{0x0524,  0x05},
	{0x0525,  0x8D},*/



	//{0x0421,  0x01},

	//{0x407c,  0x01},// enable internal sync signal for thermo channel

};

static struct regList phx3d_3021_aa_reglist[] = {

	//Laser Driver(PHX3D) reset
	{ 0x0403,  0x20 },
	{ 0x0405,  0x00 },
	{ 0x0450,  0x47 },
	{ 0x0500,  0x02 },
	{ 0x0501,  0x25 },
	{ 0x0502,  0x01 }, // 0x25
	{ 0x0401,  0x01 },
	{ 0x0400,  0x01 },
	{ 0x0401,  0x00 },

	//PhotonIC Laser Driver (PHX3D)  initial
	{ 0x0403,  0x20 },
	{ 0x0405,  0x00 },
	{ 0x0407,  0x03 },
	{ 0x0503,  0x15 },
	{ 0x0504,  0x00 },
	{ 0x0505,  0x0C }, // 0x00
	{ 0x0506,  0xA0 }, // 0x01
	{ 0x0507,  0x80 }, // 0x02
	{ 0x0508,  0x0A }, // 0x03
	{ 0x0509,  0x28 }, // 0x04
	{ 0x050A,  0x49 }, // 0x05
	{ 0x050B,  0x87 }, // 0x06
	{ 0x050C,  0x00 }, // 0x07 IBIAS_FIX
	{ 0x050D,  0xD3 }, // 0x08 ISW_FIX
	{ 0x050E,  0x00 }, // 0x09
	{ 0x050F,  0x01 }, // 0x0A
	{ 0x0510,  0x01 }, // 0x0B
	{ 0x0511,  0x28 }, // 0x0C
	{ 0x0512,  0xC0 }, // 0x0D
	{ 0x0513,  0x78 }, // 0x0E ISW_FIX threshold
	{ 0x0514,  0x04 }, // 0x0F
	{ 0x0515,  0xDF }, // 0x10
	{ 0x0516,  0x06 }, // 0x11
	{ 0x0517,  0x00 }, // 0x12
	{ 0x0518,  0x01 }, // 0x13
	{ 0x0401,  0x01 },
	{ 0x0400,  0x01 },
	{ 0x0401,  0x00 },

	{ 0x0403,  0x20 },
	{ 0x0405,  0x00 },
	{ 0x0407,  0x22 },
	{ 0x0522,  0x04 },
	{ 0x0523,  0x26 },
	{ 0x0524,  0x19 }, // 0x26
	{ 0x0525,  0x31 }, // 0x27
	{ 0x0526,  0xFF }, // 0x28
	{ 0x0401,  0x01 },
	{ 0x0400,  0x01 },
	{ 0x0401,  0x00 },

	{ 0x0403,  0x20 },
	{ 0x0405,  0x00 },
	{ 0x0407,  0x27 },
	{ 0x0527,  0x04 },
	{ 0x0528,  0x2B },
	{ 0x0529,  0x00 }, // 0x2B
	{ 0x052A,  0x00 }, // 0x2C
	{ 0x052B,  0xFF }, // 0x2D // default 0x8E
	{ 0x0401,  0x01 },
	{ 0x0400,  0x01 },
	{ 0x0401,  0x00 },

};

static struct regList phx3d_3021_cb_reglist[] = {

	//Laser Driver(PHX3D) reset
	{ 0x0403,  0x20 },
	{ 0x0405,  0x00 },
	{ 0x0450,  0x47 },
	{ 0x0500,  0x02 },
	{ 0x0501,  0x25 },
	{ 0x0502,  0x01 }, // 0x25
	{ 0x0401,  0x01 },
	{ 0x0400,  0x01 },
	{ 0x0401,  0x00 },

	//PhotonIC Laser Driver (PHX3D)  initial
	{ 0x0403,  0x20 },
	{ 0x0405,  0x00 },
	{ 0x0407,  0x03 },
	{ 0x0503,  0x15 },
	{ 0x0504,  0x00 },
	{ 0x0505,  0x0C }, // 0x00
	{ 0x0506,  0x20 }, // 0x01
	{ 0x0507,  0x30 }, // 0x02
	{ 0x0508,  0x10 }, // 0x03
	{ 0x0509,  0x20 }, // 0x04
	{ 0x050A,  0x69 }, // 0x05
	{ 0x050B,  0xC3 }, // 0x06
	{ 0x050C,  0x00 }, // 0x07 IBIAS_FIX
	{ 0x050D,  0xD3 }, // 0x08 ISW_FIX
	{ 0x050E,  0x24 }, // 0x09
	{ 0x050F,  0x0D }, // 0x0A
	{ 0x0510,  0xFF }, // 0x0B
	{ 0x0511,  0xD9 }, // 0x0C
	{ 0x0512,  0x00 }, // 0x0D
	{ 0x0513,  0x78 }, // 0x0E ISW_FIX threshold
	{ 0x0514,  0x04 }, // 0x0F
	{ 0x0515,  0xDF }, // 0x10
	{ 0x0516,  0x00 }, // 0x11
	{ 0x0517,  0x00 }, // 0x12
	{ 0x0518,  0x01 }, // 0x13
	{ 0x0401,  0x01 },
	{ 0x0400,  0x01 },
	{ 0x0401,  0x00 },

	{ 0x0403,  0x20 },
	{ 0x0405,  0x00 },
	{ 0x0407,  0x19 },
	{ 0x0519,  0x02 },
	{ 0x051A,  0x26 },
	{ 0x051B,  0x19 }, // 0x26  internal PD resistance
	{ 0x0401,  0x01 },
	{ 0x0400,  0x01 },
	{ 0x0401,  0x00 },

	{ 0x0403,  0x20 },
	{ 0x0405,  0x00 },
	{ 0x0407,  0x22 },
	{ 0x0522,  0x09 },
	{ 0x0523,  0x2B },
	{ 0x0524,  0xF1 }, // 0x2B
	{ 0x0525,  0xF8 }, // 0x2C
	{ 0x0526,  0x03 }, // 0x2D
	{ 0x0527,  0x02 }, // 0x2E
	{ 0x0528,  0x00 }, // 0x2F
	{ 0x0529,  0x00 }, // 0x30   duty increase
	{ 0x052A,  0x00 }, // 0x31   duty decrease
	{ 0x052B,  0xFF }, // 0x32   falling edge speed control
	{ 0x0401,  0x01 },
	{ 0x0400,  0x01 },
	{ 0x0401,  0x00 },

};


static struct regList dw9912_reglist[] = {

	// Laser Driver reset
	{ 0x0403,  0x20 },
	{ 0x0405,  0x00 },
	{ 0x0450,  0x47 },
	{ 0x0500,  0x02 },
	{ 0x0501,  0x25 },
	{ 0x0502,  0x01 }, // 0x25
	{ 0x0401,  0x01 },
	{ 0x0400,  0x01 },
	{ 0x0401,  0x00 },

	// Laser Driver  initial
	{ 0x0407,  0x03 },
	{ 0x0503,  0x02 },
	{ 0x0504,  0x00 },
	{ 0x0505,  0x0C }, // 0x00
	{ 0x0401,  0x01 },
	{ 0x0400,  0x01 },
	{ 0x0401,  0x00 },

	{ 0x0407,  0x06 },
	{ 0x0506,  0x03 },
	{ 0x0507,  0x07 },
	{ 0x0508,  0x00 }, // 0x07
	{ 0x0509,  0xFF }, // 0x08
	{ 0x0401,  0x01 },
	{ 0x0400,  0x01 },
	{ 0x0401,  0x00 },

	{ 0x0407,  0x0A },
	{ 0x050A,  0x05 },
	{ 0x050B,  0x0D },
	{ 0x050C,  0xC3 }, // 0x0D
	{ 0x050D,  0x07 }, // 0x0E
	{ 0x050E,  0x65 }, // 0x0F
	{ 0x050F,  0x02 }, // 0x10
	{ 0x0401,  0x01 },
	{ 0x0400,  0x01 },
	{ 0x0401,  0x00 },

	{ 0x0407,  0x10 },
	{ 0x0510,  0x02 },
	{ 0x0511,  0x3B },
	{ 0x0512,  0x07 }, // 0x3B	
	{ 0x0401,  0x01 },
	{ 0x0400,  0x01 },
	{ 0x0401,  0x00 },

};

#if  (USE_WHICH_CONVERTER==kConverterIsDuxin)
int imx456_dothin_config()
{
    int ret = 0;
#if !DEBUG_IMX456_IN_QT   

    DDEBUG("dothin_device_id=%d\n", dothin_device_id);

    ret = dothin_enable_softpin(true, dothin_device_id);
    if (ret < 0) {
        DDEBUG("dothin_enable_softpin ret=%d\n", ret);
    }
    ret = dothin_enable_gpio(true, dothin_device_id);
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

    SENSOR_POWER sensor_power[] = { POWER_AVDD, POWER_DOVDD,POWER_DVDD ,POWER_AFVCC ,POWER_VPP };
    int           power_value[] = { 2800,       1800,      1200,        3300,       1200 };
    ret = dothin_pmu_set_voltage(sensor_power, power_value,5, dothin_device_id);
    if (ret < 0) {
        DDEBUG("dothin_sensor_enable ret=%d", ret);
    }
    usleep(1000 * 50);
    DDEBUG("end %d", ret);

#endif
    return ret;

}
#endif

#if  (USE_WHICH_CONVERTER==kConverterIsCx3)
int cx3_power_up()
{
	cx3_mipi_cfg(&imx456_cx3_mipi_cfg, sizeof(imx456_cx3_mipi_cfg), dothin_device_id);

	cx3_set_gpio_dir(V3V3_EN, 0, dothin_device_id);
	cx3_set_gpio_dir(VDDA_2V7_EN_GPIO, 0, dothin_device_id);
	cx3_set_gpio_dir(VDDIF_1V8_EN_GPIO, 0, dothin_device_id);
	cx3_set_gpio_dir(VDDD_1V2_EN_GPIO, 0, dothin_device_id);
	cx3_set_gpio_dir(VDDMIX_1V2_EN_GPIO, 0, dothin_device_id);
	cx3_set_gpio_dir(VDDLD_3V6_EN_GPIO, 0, dothin_device_id);
	cx3_set_gpio_dir(VDD_3V3_EN_GPIO, 0, dothin_device_id);

	cx3_set_gpio_level(V3V3_EN, 1, dothin_device_id);
	cx3_set_gpio_level(VDDA_2V7_EN_GPIO, 1, dothin_device_id);
	cx3_set_gpio_level(VDDIF_1V8_EN_GPIO, 1, dothin_device_id);
	cx3_set_gpio_level(VDDD_1V2_EN_GPIO, 1, dothin_device_id);
	cx3_set_gpio_level(VDDMIX_1V2_EN_GPIO, 1, dothin_device_id);
	cx3_set_gpio_level(VDDLD_3V6_EN_GPIO, 1, dothin_device_id);
	cx3_set_gpio_level(VDD_3V3_EN_GPIO, 1, dothin_device_id);

	cx3_set_gpio_level(SENSOR_RESET_GPIO, 0, dothin_device_id);
	usleep(10);
	cx3_set_gpio_level(SENSOR_RESET_GPIO, 1, dothin_device_id);
	
	return 0;
}

int cx3_power_down()
{
	cx3_set_gpio_level(SENSOR_RESET_GPIO, 0, dothin_device_id);

	cx3_set_gpio_level(V3V3_EN, 0, dothin_device_id);
	cx3_set_gpio_level(VDDA_2V7_EN_GPIO, 0, dothin_device_id);
	cx3_set_gpio_level(VDDIF_1V8_EN_GPIO, 0, dothin_device_id);
	cx3_set_gpio_level(VDDD_1V2_EN_GPIO, 0, dothin_device_id);
	cx3_set_gpio_level(VDDMIX_1V2_EN_GPIO, 0, dothin_device_id);
	cx3_set_gpio_level(VDDLD_3V6_EN_GPIO, 0, dothin_device_id);
	cx3_set_gpio_level(VDD_3V3_EN_GPIO, 0, dothin_device_id);

	return 0;
}
#endif

static int cxa4016_initialize()
{
	int rtn = 0;
	for (int i = 0; i < sizeof(cxa4016_reglist) / sizeof(struct regList); i++) {

		rtn |= sensor_write_reg(cxa4016_reglist[i].reg, cxa4016_reglist[i].val);
	}
	return rtn;
}

static int phx3d_3021_aa_initialize()
{
	int rtn = 0;
	for (int i = 0; i < sizeof(phx3d_3021_aa_reglist) / sizeof(struct regList); i++) {

		rtn |= sensor_write_reg(phx3d_3021_aa_reglist[i].reg, phx3d_3021_aa_reglist[i].val);
	}
	return rtn;
}

static int phx3d_3021_cb_initialize()
{
	int rtn = 0;
	for (int i = 0; i < sizeof(phx3d_3021_cb_reglist) / sizeof(struct regList); i++) {

		rtn |= sensor_write_reg(phx3d_3021_cb_reglist[i].reg, phx3d_3021_cb_reglist[i].val);
	}
	return rtn;
}

static int dw9912_initialize()
{
	int rtn = 0;
	for (int i = 0; i < sizeof(dw9912_reglist) / sizeof(struct regList); i++) {

		rtn = sensor_write_reg(dw9912_reglist[i].reg, dw9912_reglist[i].val);
	}
	return rtn;
}

static int vcsel_driver_detect()
{
	int rtn = 0;
	uint8_t value = 0;
	rtn = vcsel_driver_write_reg(0x25, 0x01); // reset
	rtn = vcsel_driver_read_reg(0x0F, &value);

	if (value == 0x06) {
		uint8_t phx3d_value = 0;
		rtn = vcsel_driver_read_reg(0x2F, &phx3d_value);
		if(phx3d_value == 0x00){
			driver_ic_type = DRIVER_IC_PHX3D_3021_AA;
			phx3d_3021_aa_initialize();
		}
		else if (phx3d_value == 0x01) {
			driver_ic_type = DRIVER_IC_PHX3D_3021_CB;
			phx3d_3021_cb_initialize();
		}
		
	}
	else if (value == 0x46) {
		driver_ic_type = DRIVER_IC_CXA4016;
		rtn = cxa4016_initialize();
	}
	else if (value == 0xA7) {
		driver_ic_type = DRIVER_IC_DW9912;
		rtn = dw9912_initialize();
	}
	else {
		driver_ic_type = 0;

	}
	printf("driver_ic_type: %d. value: %d.\r\n", driver_ic_type, value);
	return rtn;
}

int imx456_sensor_initialize()
{
    int rtn = 0;
    for(int i = 0; i < sizeof(reglist_common)/sizeof(struct regList); i++){

        rtn |= sensor_write_reg(reglist_common[i].reg, reglist_common[i].val);
    }
    return rtn;
}

int imx456_shadow_register(bool enable) // checked
{
    int rtn = 0;
    if(enable)
        rtn = sensor_write_reg(0x0102, 0x01);
    else
        rtn = sensor_write_reg(0x0102, 0x00);
    return rtn;
}

int imx456_get_sensor_id(uint16_t *id) // checked
{
    int rtn = 0;
    uint8_t value = 0;
	static int isConfig = 0;

#if  (USE_WHICH_CONVERTER==kConverterIsDuxin)
	rtn = gpio_expander_init();// power enable
	if (rtn < 0)
		return rtn;

    if (!isConfig) {
        isConfig = 1;
        imx456_dothin_config();
    }
#endif

    rtn = sensor_read_reg(0x0001, &value); // value should be 0x55
    //printf("imx456_get_sensor_id: %d \r\n", value);
    if (rtn < 0)
        *id = 0xFFFF;
    else
        *id = imx456_sensor_id;
    return rtn;
}

int imx456_set_user_ID(uint8_t value) // not support
{
    return 0;//sensor_write_reg(0x0824, value);
}

int imx456_set_hmax(uint16_t hmax) // half checked, slightly different
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

// should stop stream first
int imx456_set_trigger_mode(uint8_t mode) // checked
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

// should stop stream first
int imx456_set_stream_mode() // checked
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

int imx456_get_data_output_mode(uint8_t *mode) // checked
{
    uint8_t value = 0;
    int rtn = sensor_read_reg(0x0828, &value);
    *mode = value&0x0f;
    return rtn;
}

// be careful that the phase resolution of A&B mode will change to 1280*480
int imx456_set_data_output_mode(uint8_t mode) // should stop streaming before change data output mode  // checked
{/*
    // mode = 0: A-B, 1: A+B, 2: A, 3: B, 4: A&B
	if (mode > 4) {
		ALOGE("mode=%d", mode);
		return -HW_ERR_INVALID;
	}
	else if (mode == 4) {
		HMAX = 0x0514;
		imx456_set_hmax(HMAX);
	}
	else if (mode < 4) {
		HMAX = 0x02B6;
		imx456_set_hmax(HMAX);
	}
	int rtn = sensor_write_reg(0x0828, mode);
    return rtn;
	*/
	return 0;
}

int imx456_get_modulation_frequency(uint16_t *modFreq) // calculation method is not provided
{
    uint8_t divselpre = 0;
    uint8_t divsel = 0;
    int rtn = sensor_read_reg(0x21BE, &divselpre);
    rtn |= sensor_read_reg(0x21BF, &divsel);

    uint8_t fmod_H, fmod_L;
    rtn |= sensor_read_reg(0x1048, &fmod_H);
    rtn |= sensor_read_reg(0x1049, &fmod_L);
    uint16_t fmod = (fmod_H << 8) + fmod_L;

    *modFreq = fmod/pow(2, (divselpre + divsel));

    return rtn;
}

int imx456_set_modulation_frequency(uint16_t modFreq)
{
	int rtn = imx456_shadow_register(true);
	if (modFreq == 100) {
		rtn |= sensor_write_reg(0x1049, 0x64);
		rtn |= sensor_write_reg(0x104B, 0x02);
		rtn |= sensor_write_reg(0x10D0, 0x0B);
		rtn |= sensor_write_reg(0x10D5, 0x9C);
		rtn |= sensor_write_reg(0x21BE, 0x00);
		rtn |= sensor_write_reg(0x21BF, 0x00);
	}
	else if (modFreq == 60) {
		rtn |= sensor_write_reg(0x1049, 0x78);
		rtn |= sensor_write_reg(0x104B, 0x00);
		rtn |= sensor_write_reg(0x10D0, 0x0A);
		rtn |= sensor_write_reg(0x10D5, 0x81);
		rtn |= sensor_write_reg(0x21BE, 0x01);
		rtn |= sensor_write_reg(0x21BF, 0x00);
	}
	else if (modFreq == 20) {
		rtn |= sensor_write_reg(0x1049, 0x50);
		rtn |= sensor_write_reg(0x104B, 0x02);
		rtn |= sensor_write_reg(0x10D0, 0x09);
		rtn |= sensor_write_reg(0x10D5, 0xC5);
		rtn |= sensor_write_reg(0x21BE, 0x00);
		rtn |= sensor_write_reg(0x21BF, 0x02);
	}
	else {
		return -1;
	}
	rtn |= imx456_shadow_register(false);

	return rtn;
}

/*
int imx456_set_modulation_frequency(uint16_t modFreq) // checked
{
    if(modFreq > 100 || modFreq < 4)
        return -1;

    int rtn = imx456_shadow_register(true);
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

    rtn |= imx456_shadow_register(false);

    return rtn;
}
*/
// param uint: us
int imx456_get_frame_startup_time(uint16_t *startupTime)
{
    uint8_t byte0, byte1;
    int rtn = sensor_read_reg(0x21D4, &byte1);
    rtn |= sensor_read_reg(0x21D5, &byte0);
    uint16_t value = (byte1 << 8) + byte0;
    *startupTime = value*HMAX/120;

    return rtn;
}														  
int imx456_set_frame_startup_time(uint16_t startupTime)
{
    uint16_t value = (startupTime*120)/HMAX;
    int rtn = sensor_write_reg(0x21D4, (value >> 8)&0xFF);
    rtn |= sensor_write_reg(0x21D5, (value & 0xFF));

    return rtn;
}

// param uint: us
int imx456_set_frame_time(uint32_t frameTime) // checked
{
	uint32_t value = (frameTime * 120) / HMAX;
    int rtn = sensor_write_reg(0x2108, (value >> 24)&0xFF);
    rtn |= sensor_write_reg(0x2109, (value >> 16)&0xFF);
    rtn |= sensor_write_reg(0x210A, (value >> 8)&0xFF);
    rtn |= sensor_write_reg(0x210B, (value & 0xFF));

    return rtn;
}

int imx456_get_fps(uint8_t *fps) // checked
{
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
    return rtn;
}

int imx456_set_fps(uint8_t fps)
{
	if(fps == 0)
        return -1;															   
	unsigned int frameTime = 1000000 / fps; // us
    int rtn = imx456_set_frame_time(frameTime);

    return rtn;
}

int imx456_set_phase_count(uint8_t phaseCount) // checked
{
    if(phaseCount > 8 || phaseCount < 1)
        return -1;
    int rtn = sensor_write_reg(0x21E8, phaseCount); // default is 4

    return rtn;
}

int imx456_set_phase_pretime(uint16_t preTime)
{
    if(preTime > 2000 || preTime < 1)  // value 0 is not allowed, 2000 is user defined
        return -1;
    int rtn = imx456_shadow_register(true);
    rtn |= sensor_write_reg(0x4015, (preTime >> 8)&0xFF);
    rtn |= sensor_write_reg(0x4016, (preTime & 0xFF));
    rtn |= imx456_shadow_register(false);

    return rtn;
}

int imx456_get_integration_time(uint16_t *integrationTime) // checked
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
    *integrationTime = value/120; // may slightly different from what we set because of floor function

    return rtn;
}

int RoundUpBase10(int n) {
    return (n + 9) / 10 * 10;
}

int imx456_set_integration_time(uint16_t integrationTime) // checked
{
    integrationTime = RoundUpBase10(integrationTime);
    if(integrationTime > 2000)
        return -1;

	unsigned char repeat = 1;
    uint16_t max_int_time = 999;  // us
    uint16_t new_int_time = 0;
    while(integrationTime > max_int_time){
        integrationTime = integrationTime - max_int_time;
        repeat++;
    }
    new_int_time = integrationTime + max_int_time*(repeat - 1);
    int value = floor((new_int_time/repeat)*120.0f/HMAX)*HMAX;

    int rtn = imx456_shadow_register(true);
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

    rtn |= imx456_shadow_register(false);

    return rtn;
}

int imx456_set_phase_idle_time(uint8_t value)
{
    if(value < 5)  // value out side [0x05 - 0xFF] are prohibited
        return -1;
    int rtn = imx456_shadow_register(true);
    for(uint8_t phase = 0; phase < 4; phase++){

        rtn |= sensor_write_reg(0x21C8 + phase, value);
    }
    rtn |= imx456_shadow_register(false);

    return rtn;
}

/*
mode = 0: no binning (= VGA resolution, 640x480 pixels)  // default mode
mode = 1: 2x2 binning (= QVGA resolution, 320x240 pixels)
mode = 2: 4x4 binning (= QQVGA resolution, 160x120 pixels)
mode = 3: 8x8 binning (= QQQVGA resolution, 80x60 pixels)
*/
int imx456_set_pixel_binning(uint8_t mode) // checked
{
    return sensor_write_reg(0x14A5, mode);
}

int imx456_get_pixel_binning(uint8_t *mode) // checked
{
    return sensor_write_reg(0x14A5, *mode);
}

// haven't consider binning yet
int imx456_pixelROI(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2) // checked
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

int imx456_get_img_mirror_flip(uint8_t *mode) // checked
{
    int rtn = 0;
    uint8_t h = 0, v = 0;
    rtn |= sensor_read_reg(0x080d, &h);
    rtn |= sensor_read_reg(0x080c, &v);
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

int imx456_set_img_mirror_flip(uint8_t mode) // checked
{
    int rtn = 0;
    if(mode == 0){
        rtn |= sensor_write_reg(0x080d, 0x00); // default
        rtn |= sensor_write_reg(0x080c, 0x00); // default
    }
    else if(mode == 1){
        rtn |= sensor_write_reg(0x080d, 0x01);
        rtn |= sensor_write_reg(0x080c, 0x00);
    }
    else if(mode == 2){
        rtn |= sensor_write_reg(0x080d, 0x00);
        rtn |= sensor_write_reg(0x080c, 0x01);
    }
    else if(mode == 3){
        rtn |= sensor_write_reg(0x080d, 0x01);
        rtn |= sensor_write_reg(0x080c, 0x01);
    }
    return rtn;
}

int imx456_get_sensor_temperature(float *temp)
{
    uint8_t value;
    int rtn = sensor_read_reg(0x1403, &value);

    *temp = (float)(value&0xff) - 40;
    //qDebug() << "value temp " << value << *temp;

    return rtn;
}

int imx456_pixel_statistics(bool enable, uint8_t mode)
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
int imx456_set_pixel_lower_limit(uint16_t value)
{
    int rtn = 0;
    for(uint8_t phase = 0; phase < 4; phase++){

        rtn |= sensor_write_reg(0x1434 + phase*2, (value >>  8)&0xFF);
        rtn |= sensor_write_reg(0x1435 + phase*2, (value & 0xFF));
    }

    return rtn;
}

// The maximum threshold for each tap.
int imx456_set_pixel_upper_limit(uint16_t value)
{
    int rtn = 0;
    for(uint8_t phase = 0; phase < 4; phase++){

        rtn |= sensor_write_reg(0x1448 + phase*2, (value >>  8)&0xFF);
        rtn |= sensor_write_reg(0x1449 + phase*2, (value & 0xFF));
    }

    return rtn;
}

int imx456_get_pixel_error_count_low(uint8_t phase, uint32_t *value)
{
    uint8_t Byte1, Byte2, Byte3;

    int rtn = sensor_read_reg(0x145D + phase*4, &Byte3);
    rtn |= sensor_read_reg(0x145E + phase*4, &Byte2);
    rtn |= sensor_read_reg(0x145F + phase*4, &Byte1);

    *value = Byte1 + (Byte2 << 8) + (Byte3 <<16);

    return rtn;
}

int imx456_get_pixel_error_count_high(uint8_t phase, uint32_t *value)
{
    uint8_t Byte1, Byte2, Byte3;

    int rtn = sensor_read_reg(0x1481 + phase*4, &Byte3);
    rtn |= sensor_read_reg(0x1482 + phase*4, &Byte2);
    rtn |= sensor_read_reg(0x1483 + phase*4, &Byte1);

    *value = Byte1 + (Byte2 << 8) + (Byte3 <<16);

    return rtn;
}

int imx456_test_pattern(uint8_t mode) // not provided register yet
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

#if !DEBUG_IMX456_IN_QT
int imx456_get_sensor_info(struct sensor_info_t *info) 
{
    info->embedded_data_size = 200 * 2;
    info->vcsel_num = 1;
    info->vcsel_driver_id = driver_ic_type;
    info->sensor_id = imx456_sensor_id;

    return 0;
}

int imx456_get_sensor_default_param(struct sensor_default_param_t *param) 
{
	param->mod_freq1 = 60; // 60MHz
	param->mod_freq2 = 100; // 100MHz
	param->freq1_duty_index = 10; // duty cycle list index, correspond to 35% when mod_freq = 60MHz
	param->freq2_duty_index = 12; // duty cycle list index, correspond to 35% when mod_freq = 100MHz
	param->_illum_power.value_A = 255; // driver IC isw value
	param->_illum_power.value_B = 0; // driver IC ibias value
	param->integration_time = 1000; // us

	return 0;
}
#endif

int imx456_illum_duty_cycle_adjust(uint8_t mode, uint8_t value) // checked
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
int imx456_get_illum_duty_cycle_list(uint8_t mod_freq, float *duty_cycle_list) // checked
{
    float duty = 0;
    float cycle = 1000.0f/mod_freq; // period ns
    for(int step = -15; step <= 15; step++){
        duty = (0.5f + step*0.5f/cycle)*100;
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

int imx456_get_illum_duty_cycle(uint16_t *duty) // checked
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

int imx456_set_illum_duty_cycle(uint16_t duty) // checked
{
    int rtn = 0;
    uint8_t value = 0;

	rtn = imx456_shadow_register(true); // very important!! should use shadow register when multi registers are changed
    if(duty == 15){
        rtn = imx456_illum_duty_cycle_adjust(0, 0);
    }
    else if(duty > 15){
        value = duty - 15;
        rtn = imx456_illum_duty_cycle_adjust(2, 0);
        rtn |= imx456_illum_duty_cycle_adjust(1, value);
    }
    else if(duty < 15){
        value = 15 - duty;
        rtn = imx456_illum_duty_cycle_adjust(1, 0);
        rtn |= imx456_illum_duty_cycle_adjust(2, value);
    }
	rtn = imx456_shadow_register(false);

    return rtn;

}

int imx456_metadata_output(uint8_t mode, uint16_t length) // default length 192
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

	if(length <= 133)
        return -1;
    // EBD length = [0x2c0c]*12 + [0x2c0d bit7]*6 + [0x2c0d bit6]*3 + [0x2c0d bit7]*1

    if((length%12) != 0)
        return -1;
    else{
        uint8_t value = length/12;
        rtn |= sensor_write_reg(0x2C0C, value); // default value 0x10
        rtn |= sensor_write_reg(0x2C0D, 0);
    }										  
    return rtn;
}

int imx456_video_streaming(bool enable)
{
    int rtn = 0;
    if(enable)
        rtn |= sensor_write_reg(0x1001, 0x01);
    else
        rtn |= sensor_write_reg(0x1001, 0x00);

    return rtn;
}

int imx456_init()
{
    int rtn = 0;
   
    rtn = imx456_sensor_initialize();
	rtn = vcsel_driver_detect();
	//rtn |= imx456_set_illum_power(0, 150, 0);													
/**/
    if (!use_trigger_mode) {
        rtn = imx456_set_stream_mode();
        rtn = imx456_set_fps(30);
	}
	else {
		//use trigger_mode
        rtn = imx456_set_trigger_mode(1);
	}
    //rtn |= imx456_set_modulation_frequency(60); // MHz
    rtn = imx456_set_integration_time(1000); // us
	//rtn |= imx456_test_pattern(1);
	//imx456_metadata_output(0, 192);
#if USED_IN_QC_DEVICE
    // TODO 如果是测试光功率版本  需要把这条打开
	rtn = imx456_illum_power_test_init();
#endif
    return rtn;
}

int imx456_func_init()
{
#if !DEBUG_IMX456_IN_QT
#if  (USE_WHICH_CONVERTER==kConverterIsCx3)
	cx3_power_up();
#endif

    tof_sensor.init = imx456_init;
    tof_sensor.get_sensor_id = imx456_get_sensor_id;
    tof_sensor.hardware_trigger = imx456_hardware_trigger;
    tof_sensor.software_trigger = imx456_software_trigger;
    tof_sensor.video_streaming = imx456_video_streaming;
    tof_sensor.get_fps = imx456_get_fps;
    tof_sensor.set_fps = imx456_set_fps;
    tof_sensor.get_sensor_temperature = imx456_get_sensor_temperature;
    tof_sensor.get_rx_temp = imx456_get_rx_temp;
    tof_sensor.get_tx_temp = imx456_get_tx_temp;
    tof_sensor.set_illum_power = imx456_set_illum_power;
    tof_sensor.get_illum_power = imx456_get_illum_power;
    tof_sensor.illum_power_control = imx456_illum_power_control;
    tof_sensor.get_integration_time = imx456_get_integration_time;
    tof_sensor.set_integration_time = imx456_set_integration_time;
    tof_sensor.get_modulation_frequency = imx456_get_modulation_frequency;
    tof_sensor.set_modulation_frequency = imx456_set_modulation_frequency;
    tof_sensor.get_illum_duty_cycle = imx456_get_illum_duty_cycle;
    tof_sensor.set_illum_duty_cycle = imx456_set_illum_duty_cycle;
    tof_sensor.get_data_output_mode = imx456_get_data_output_mode;
    tof_sensor.set_data_output_mode = imx456_set_data_output_mode;
    tof_sensor.get_img_mirror_flip = imx456_get_img_mirror_flip;
    tof_sensor.set_img_mirror_flip = imx456_set_img_mirror_flip;
    tof_sensor.get_pixel_binning = imx456_get_pixel_binning;
    tof_sensor.set_pixel_binning = imx456_set_pixel_binning;
    tof_sensor.test_pattern = imx456_test_pattern;
    tof_sensor.get_sensor_info = imx456_get_sensor_info;
    tof_sensor.get_illum_duty_cycle_list = imx456_get_illum_duty_cycle_list;
	tof_sensor.get_vcsel_pd = imx456_get_vcsel_pd;
	tof_sensor.illum_power_test = imx456_illum_power_test;
	tof_sensor.get_sensor_default_param = imx456_get_sensor_default_param;
#endif
	return 0;
}
