#include "mx6xhal\hw_obstatus.h"
#include "tof_board.h"
#include "tof_sensors.h"
#include "mx6xhal\hw_modules.h"
#include "rk1608\rk1608.h"
#include"rk1608\debug2log.h"
#include "project_config.h"

#ifdef WIN32
#include <stdio.h>
#define __FUNCTION__ __FUNCTION__
#endif

//#define TEE_LOG_LEVEL_ERROR        8
//#define ALOGE(fmt,...)			   tops_t.qsee_log(TEE_LOG_LEVEL_ERROR, "[ERROR] [TOF_DLL] [%s(%d)] : " fmt"\n",__FUNCTION__,__LINE__,##__VA_ARGS__)

#define I2C_M_RD				   1
#define I2C_M_WT				   0

board_list_t board_list[] = {
	{ RK1608_BOARD_ID, rk1608_init },
	{ RK1608_PLECO_BOARD_ID, rk1608_pleco_init },
	{ RK1608_S5K33DXX_BOARD_ID, rk1608_S5K33dxx_init },
};

/*
@brief:  配置板载eeprom地址为0xA2，和模组内部eeprom地址区分 操作接口
@param[in] None
@return   MX6X_NO_ERR          成功
*/
static int gt24c64c_change_addr_to_A2()
{
	int ret = 0;
	uint32_t data_value = 0;
	i2c_msg_t msg;

	msg.slave_addr = RK1608_EEPROM_CSP_DEFAULT;
	msg.rw_mode = I2C_M_RD;
	msg.reg = 0x3CA;
	msg.reg_size = 2;
	msg.data = &data_value;
	msg.data_size = 2;

	ret = tops_t.ops_writeread(&msg, 1);
	if (ret < 0) {
		return ret;
	}
	tops_t.tee_usleep(1000);

	if (data_value == RK1608_EEPROM_ADDR_DEFAULT) {
		msg.slave_addr = 0xb0;
		msg.rw_mode = I2C_M_WT;
		msg.reg = 0xFF;
		msg.reg_size = 1;
		data_value = 0x35;
		msg.data = &data_value;
		msg.data_size = 1;
		ret = tops_t.ops_writeread(&msg, 1);
		if (ret < 0) {
			return ret;
		}
		tops_t.tee_usleep(1000);

		msg.slave_addr = RK1608_EEPROM_CSP_DEFAULT;
		msg.rw_mode = I2C_M_WT;
		msg.reg = 0x06CA;
		msg.reg_size = 2;
		data_value = 0x2D;
		msg.data = &data_value;
		msg.data_size = 1;
		ret = tops_t.ops_writeread(&msg, 1);
		if (ret < 0) {
			return ret;
		}
		tops_t.tee_usleep(1000);

		msg.slave_addr = 0xb2;
		msg.rw_mode = I2C_M_RD;
		msg.reg = 0x06CA;
		msg.reg_size = 2;
		data_value = 0x2D;
		msg.data = &data_value;
		msg.data_size = 1;
		ret = tops_t.ops_writeread(&msg, 1);
		if (ret < 0) {
			return ret;
		}
		if (data_value != 0x2D) {
			return -1;
		}
		tops_t.tee_usleep(1000);
	}

}

/*
@brief:  板载eeprom读取数据 操作接口
@param[in] addr：设备地址, reg：寄存器地址, reg_size：寄存器地址长度, data：读取的数据, data_size:读取的数据长度
@return   MX6X_NO_ERR          成功
*/
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

	ret = tops_t.ops_writeread(&msg, 1);

	return ret;
}

/*
@brief:  板载eeprom写数据 操作接口
@param[in] addr：设备地址, reg：寄存器地址, reg_size：寄存器地址长度, data：发送的数据, data_size:发送的数据长度
@return   MX6X_NO_ERR          成功
*/
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

	ret = tops_t.ops_writeread(&msg, 1);

	tops_t.tee_usleep(6000);

	return ret;
}

/*
@brief:  板载eeprom写保护 操作接口
@param[in] en：使能写保护参数
@return   MX6X_NO_ERR          成功
*/
int gt24c64c_eeprom_write_protect(uint8_t en)
{
	int ret = 0;
	uint8_t data = 0x35;
	uint8_t p_write = 0x2D;

	ret = eeprom_write(0xB2, 0xff, 1, &data, 1);
	if (ret < 0) {
		return ret;
	}

	p_write |= (en << 1);

	ret = eeprom_write(0xB2, 0x06CA, 2, &p_write, 1);
	if (ret < 0) {
		return ret;
	}

	return ret;
}

/*
@brief:  板载板载eeprom写数据 操作接口
@param[in] offset：写入数据地址, buf：发送的数据, size:发送的数据长度
@return   MX6X_NO_ERR          成功
*/
int gt24c64c_eeprom_write(uint16_t offset, uint8_t *buf, uint16_t size)
{
	int ret = 0;

	uint8_t cnt = 0;
	uint8_t blocksize = 64;
	uint32_t addr = offset;
	uint8_t * buf_ptr_r = buf;
	uint8_t prog_time = size / blocksize;
	uint8_t lastsize = size % blocksize;

	ret = gt24c64c_eeprom_write_protect(0);
	if (ret < 0) {
		return ret;
	}

	for (cnt = 0; cnt < prog_time; cnt++)
	{
		ret = eeprom_write(FM24C64_ADDR, addr, 2, buf_ptr_r, blocksize);
		addr += blocksize;
		buf_ptr_r += blocksize;
	}

	if (lastsize != 0)
	{
		ret = eeprom_write(FM24C64_ADDR, addr, 2, buf_ptr_r, lastsize);
	}

	ret = gt24c64c_eeprom_write_protect(1);
	if (ret < 0) {
		return ret;
	}

	return ret;
}

/*
@brief:  板载eeprom读取数据 操作接口
@param[in] offset：写入数据地址, buf：发送的数据, size:发送的数据长度
@return   MX6X_NO_ERR          成功
*/
int gt24c64c_eeprom_read(uint16_t offset, uint8_t *buf, uint16_t size)
{
	int ret = 0;

	uint8_t cnt;
	uint8_t blocksize = 64;
	uint8_t prog_time = size / blocksize;
	uint8_t lastsize = size % blocksize;
	uint8_t * buf_ptr_r = buf;
	uint32_t addr = offset;

	for (cnt = 0; cnt < prog_time; cnt++)
	{
		ret = eeprom_read(FM24C64_ADDR, addr, 2, buf_ptr_r, blocksize);
		addr += blocksize;
		buf_ptr_r += blocksize;
	}

	if (lastsize != 0)
	{
		ret = eeprom_read(FM24C64_ADDR, addr, 2, buf_ptr_r, lastsize);
	}

	return ret;
}

/*
@brief:  加载tof板载型号函数 操作接口
@param[in] type 返回板载型号ID
@return   MX6X_NO_ERR          成功
*/
int tof_sensor_board_type(int *type)
{
	int ret = 0;
	uint32_t data = 0;
	i2c_msg_t msg;

#if 0//为了区分板载eeprom和模组内eeprom,先取下模组，更改板载eeprom的地址。
        ret = gt24c64c_change_addr_to_A2();
        if (ret < 0) {
	        ALOGE("===========>gt24c64c_change_addr_to_A2 fail -> ret=%d", ret);
	        return ret;
        }
        DEBUG2LOG("gt24c64c_change_addr_to_A2 success!");
#endif

#if 0//向板载eeprom中固定地址写入相应的板载ID
        data = RK1608_PLECO_BOARD_ID;//RK1608_S5K33DXX_BOARD_ID;//RK1608_BOARD_ID  RK1608_PLECO_BOARD_ID 东海  RK1608_S5K33DXX_BOARD_ID 昆仑山
        ret = gt24c64c_eeprom_write(BOARD_REG, &data, 4);
        if (ret < 0) {
	        ALOGE("===========>gt24c64c_eeprom_write fail -> ret=%d", ret);
	        return ret;
        }
        DEBUG2LOG("gt24c64c_eeprom_write data=0x%x success!", data);
        data = 0;
#endif

	ret = gt24c64c_eeprom_read(BOARD_REG, type, 4);
	if (ret < 0) {
		//ALOGE("===========>gt24c64c_eeprom_read read fail-> ret=%d", ret);
	}

	return ret;
}

/*
*@brief:  加载TOF板子函数结构体
* 通过查询板子上eeprom设备信息，匹配合适的板载驱动
@return   MX6X_NO_ERR          成功
-MX6X_ERR_NULL       传入空指针
-MX6X_ERR_INVALID   无效参数
*/
int tof_board_init()
{
	int i;
	int ret = 0;
	int board_id = 0;//0x1608;
	uint16_t id = 0;

	rk1608_pleco_gpio_expander_init();//东海项目的pleco需要先上电，不然会拉低I2C电平。
	ret = tof_sensor_board_type(&board_id);
	if (ret < 0)
	{
		ALOGE("%s This board is not RK1608 and RK1608 + sensor", __FUNCTION__);
		return -HW_ERR_RW;
	}
	for (i = 0; i < (sizeof(board_list) / sizeof(board_list[0])); i++)
	{
		if (board_list[i].BoardId == board_id) {
			ret = board_list[i].board_init();
			if (ret < 0) {
				ALOGE("%s board init fail, ID:0x%x ", __FUNCTION__, board_id);
				return  -HW_ERR_RW;
			}
			else
			{
				return  0;
			}
		}
	}

	ALOGE("%s match board type fail, ID:0x%x ", __FUNCTION__, board_id);
	return  -HW_ERR_NO_MATCH;
}
