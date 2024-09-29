#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "hw_modules.h"
#include "mx6300.h"


mx6x_module_t MX6X_HMI ={
    .init_spi = orbbecdrv_init_spi,
    .set_property = orbbecdrv_set_property,
    .get_property = orbbecdrv_get_property,
    .deinit = orbbecdrv_deinit,
};


/**
 * @brief: 加载系统函数 操作接口
 * @params[in]: ops 系统函数操作接口
 * @return:  MX6X_NO_ERR          成功
             -MX6X_ERR_NULL       传入空指针
             -MX6X_ERR_INVALID    无效参数
 */
int orbbecdrv_init_spi(obc_ops_t* ops)
{
    return mx6x_spi_lib_init(ops);
}


/**
 * @brief: set orbbec 3d devices property
 * @params[in]: command structure/buffer pointer of the callback functions
 * @params[in]: data structure/buffer pointer of the callback functions
 * @return: MX6X_NO_ERR          成功
            -MX6X_ERR_INVALID    无效参数
 */
int orbbecdrv_set_property(mx6x_command_t command, command_data_t* pdata)
{
    int ret = MX6X_NO_ERR;

    switch(command)
    {
        case MODULE_NAME:
        //
        break;

        case MODULE_VERSION:
        //
        break;

        case LOAD_FW:
        if(pdata->len != sizeof(block_buf_t)){
            return -MX6X_ERR_INVALID;
        }
        block_buf_t* p_firmware = (block_buf_t*)pdata->data;
        ret = mx6x_load_firmware(p_firmware->data, p_firmware->len, p_firmware->block);
        break;

        case LOAD_REF:
        ret = load_irpattern_ref(pdata->data, pdata->len);
        break;

        case EEPROM_RW:
        if(pdata->len != sizeof(eeprom_data)){
            return -MX6X_ERR_INVALID;
        }
        eeprom_data* p_eeprom = (eeprom_data*)pdata->data;
        ret = mx6x_ir_sensor_eeprom_write(p_eeprom->addr, p_eeprom->len, &p_eeprom->data);
        break;

        case REG_RW:
        reg_map_t* p_reg = (reg_map_t*)pdata->data;
        ret = mx6x_reg_write(p_reg->addr, p_reg->value);
        break;

        case STREAM_MODE:
        if(pdata->len != sizeof(stream_mode_t)){
            return -MX6X_ERR_INVALID;
        }

        stream_mode_t* p_stream_mode = (stream_mode_t*)pdata->data;
        ret = mx6x_start_stream(p_stream_mode->width, p_stream_mode->height, p_stream_mode->type);
        break;

        case DATA_FMT:
        //
        break;

        case FPS:
        if(pdata->len != sizeof(uint32_t)){
            return -MX6X_ERR_INVALID;
        }
        ret = mx6x_ir_sensor_fps_set(*(uint32_t*)pdata->data);
        break;

        case AE_ENABLE:
        if(pdata->len != sizeof(uint8_t)){
            return -MX6X_ERR_INVALID;
        }
        ret = mx6x_ir_sensor_enableAE(*(uint8_t*)pdata->data);
        break;

        case IR_EXP:
        if(pdata->len != sizeof(uint32_t)){
            return -MX6X_ERR_INVALID;
        }
        ret = mx6x_ir_sensor_exp_set(*(uint32_t*)pdata->data);
        break;

        case IR_GAIN:
        ret = int mx6x_ir_set_gain(*(uint32_t*)pdata->data);
        break;

        case PULSE_WIDTH:
        if(pdata->len != sizeof(uint32_t)){
            return -MX6X_ERR_INVALID;
        }
        ret = mx6x_laser_pulsewidth_set(*(uint32_t*)pdata->data);
        break;

        case FLOOD_LED:
        if(pdata->len != sizeof(uint8_t)){
            return -MX6X_ERR_INVALID;
        }
        ret = mx6x_flood_led_onoff(*(uint8_t*)pdata->data);
        break;

        case LASER:
        if(pdata->len != sizeof(uint8_t)){
            return -MX6X_ERR_INVALID;
        }
        ret = mx6x_laser_onoff(*(uint8_t*)pdata->data);
        break;

        case IR_SN:
        //
        break;

        case LDMP_SN:
        //
        break;

        case LASER_CURRENT:
        ret = mx6x_laser_current_set(*(uint32_t*)pdata->data);
        break;

        case FLOOD_CURRENT:
        //
        break;

        case WATCH_DOG:
        if(pdata->len != sizeof(uint8_t)){
            return -MX6X_ERR_INVALID;
        }

        ret = mx6x_wdt_enable(*(uint8_t*)pdata->data);
        break;

        case SLEEP_WAKE:
        if(pdata->len != sizeof(int)){
            return -MX6X_ERR_INVALID;
        }

        ret = mx6x_sleep_wakeup(*(int*)pdata->data);
        break;

        case PACK_FMT:
        //
        break;

        case DISP_SUB_BITS:
        //
        break;

        case DEPTH_ROTATION_TYPE:
        //
        break;

        case DEPTH_MIRROR_TYPE:
        //
        break;

        case FW_VERSION:
        //
        break;

        case DEPTH_ENGINE:
        //
        break;

        default:
        ret = -MX6X_ERR_INVALID;
        break;
    }
    return ret;
}


/**
 * @brief: get orbbec 3d devices property
 * @params[in]: command structure/buffer pointer of the callback functions
 * @params: structure/buffer pointer of the callback functions
 * @return: MX6X_NO_ERR          成功
            -MX6X_ERR_INVALID    无效参数
 */
int orbbecdrv_get_property(mx6x_command_t command, command_data_t* pdata)
{
    int ret = MX6X_NO_ERR;

    switch(command)
    {
        case MODULE_NAME:
        //ret = mx6x_get_module_name((uint32_t*)pdata->data);
        break;

        case MODULE_VERSION:
        ret = mx6x_get_lib_version((uint32_t*)pdata->data);
        break;

        case LOAD_FW:
        //
        break;

        case LOAD_REF:
        //
        break;

        case EEPROM_RW:
        if(pdata->len != sizeof(eeprom_data)){
            return -MX6X_ERR_INVALID;
        }

        eeprom_data *p_eeprom = pdata->data;
        mx6x_ir_sensor_eeprom_read(p_eeprom->addr, p_eeprom->len, &p_eeprom->data);
        break;

        case REG_RW:
        if(pdata->len != sizeof(reg_map_t)){
            return -MX6X_ERR_INVALID;
        }
        reg_map_t* p_reg = (reg_map_t*)pdata->data;
        mx6x_reg_read(p_reg->addr, p_reg->value);
        break;

        case STREAM_MODE:
        //
        break;

        case DATA_FMT:
        ret = mx6x_pack_fmt_get((uint8_t*)pdata->data);
        break;

        case FPS:
        ret = mx6x_ir_sensor_fps_get((uint32_t*)pdata->data);
        break;

        case AE_ENABLE:
        ret = mx6x_ir_sensor_AE_status((uint32_t*)pdata->data);
        break;

        case IR_EXP:
        ret = mx6x_ir_sensor_exp_get((uint32_t*)pdata->data);
        break;

        case IR_GAIN:
        ret = mx6x_ir_get_gain((uint32_t*)pdata->data);
        break;

        case PULSE_WIDTH:
        ret = mx6x_laser_pulsewidth_get((uint32_t*)pdata->data);
        break;

        case FLOOD_LED:
        ret = mx6x_flood_led_status((uint32_t*)pdata->data);
        break;

        case LASER:
        //
        break;

        case IR_SN:
        //
        break;

        case LDMP_SN:
        ret = mx6x_ldmp_get_sn((uint16_t*)pdata->data, pdata->len);
        break;

        case LASER_CURRENT:
        ret = mx6x_laser_current_get((uint32_t*)pdata->data);
        break;

        case FLOOD_CURRENT:
        //
        break;

        case WATCH_DOG:
        //
        break;

        case SLEEP_WAKE:
        //
        break;

        case PACK_FMT:
        ret = mx6x_pack_fmt_get((uint8_t*)pdata->data);
        break;

        case DISP_SUB_BITS:
        ret = mx6x_disp_bits_get((uint8_t*)pdata->data);
        break;

        case DEPTH_ROTATION_TYPE:
        ret = mx6x_rotation_get((uint8_t*)pdata->data);
        break;

        case DEPTH_MIRROR_TYPE:
        ret = mx6x_mirror_get((uint8_t*)pdata->data);
        break;

        case FW_VERSION:
		ret = mx6x_get_fw_version((uint32_t*)pdata->data);
        break;

        case DEPTH_ENGINE:
        ret = mx6x_depth_engine_get((char*)pdata->data, pdata->len);
        break;

        default:
        ret = -MX6X_ERR_INVALID;
        break;
    }
    return ret;
}


/*
@brief:  释放系统函数 操作接口
@param    null
@return   MX6X_NO_ERR     成功
*/
int orbbecdrv_deinit()
{
    ALOGE("spi lib release.\n");

    retern MX6X_NO_ERR;
}




