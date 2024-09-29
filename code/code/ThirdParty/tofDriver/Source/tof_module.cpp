#include <string.h>
#include <thread>
#include <chrono>
#include "mx6xhal\hw_modules.h"
#include "mx6xhal\ob_resolution.h"
#include "mx6xhal\obc_tee_funcs.h"
#include "mx6xhal\hw_obstatus.h"
#include "include\typedef.h"
#include "tof_sensors.h"
#include "tof_board.h"
#include "imx456\imx456.h"
#include "s5k33dxx\s5k33dxx.h"
#include "delay_board\delay_board.h"
#include"rk1608\debug2log.h"

#ifdef WIN32
#define OBC_API_EXPORT __declspec(dllexport) 
#else
#define OBC_API_EXPORT 
#endif

#ifdef __cplusplus 
extern "C"{
#endif

obc_ops_t tops_t;
tof_sensor_func_t tof_sensor;
tof_board_func_t tof_board;

int is_init_ = 0;

int tof_slave_init(void* ops);
int tof_slave_deinit();
int tof_slave_set_property(hw_command_t command, command_data_t* pcmd);
int tof_slave_get_property(hw_command_t command, command_data_t* pdata);
int get_resolution_value(const int width, const int height);

OBC_API_EXPORT mx6x_module_t HAL_MODULE_SYM = {
    tof_slave_init,
    tof_slave_set_property,
    tof_slave_get_property,
    tof_slave_deinit
};

OBC_API_EXPORT support_protocol_type_t HAL_PROTOCOL_SYM = {
    I2C_PROTOCOL,
    "TofSensor",
    IR_SENSOR
};

hw_stream_type_t support_stream_[] = 
{
    STREAM_TOF_PHASE,
    STREAM_DEPTH,
};

stream_mode_t support_modes_mlx75027_[] = {
    {
        640,
        480,
        STREAM_TOF_PHASE
    }
};

stream_mode_t support_modes_s5k33d_[] = {
    {
        1280,
        960,
        STREAM_TOF_PHASE
    }
};
stream_mode_t support_modes_rk1608_s5k33d_[] = {
    {
        1280,
        960,
        STREAM_TOF_PHASE
    },
    {
        640,
        480,
        STREAM_DEPTH
    }
};
stream_mode_t support_modes_pleco_[] =
{
    {
        1920,
        480,
        STREAM_TOF_PHASE,
    },
    {
        960,
        240,
        STREAM_TOF_PHASE,
    },
};
stream_mode_t support_modes_pleco_rk1608_[] = 
{
    {
        1920,
        480,
        STREAM_TOF_PHASE,
    },
    {
        640,
        480,
        STREAM_DEPTH,
    }
};

static uint8_t illum_power_test_mode = 0;
int tof_slave_init(void* ops)
{
    //printf("------->mx6x init_spi ...\n");

    int ret = 0;
    if (ops == NULL) {
        return -HW_ERR_NULL;
    }
    ret = tof_sensor_lib_init(ops);
    if (ret < 0) 
    {
		ALOGE("=====>tof_sensor_lib_init fail, ret = %d", ret);
		return ret;
    }

    ret = tof_board_init();
	if (ret < 0)
    {
		ret = tof_sensor_init();
		if (ret < 0) 
        {
            ALOGE("=====>tof_init, fail, ret = %d", ret);
            return ret;
		}
	}
    ALOGE("=====>tof_init, success, ret = %d", ret);

    is_init_ = 1;
    illum_power_test_mode = 0;

    return ret;
}

int tof_slave_deinit()
{
    int ret = 0;
    is_init_ = 0;
    return ret;
}

int tof_slave_set_property(hw_command_t command, command_data_t* pcmd)
{
    int ret = 0;
    if (!is_init_) 
    {
        return -HW_ERR_NO_INIT;
    }
    if (nullptr == pcmd)
    {
        ALOGE("tof_slave_set_property input nullptr");
        return -HW_ERR_NULL;
    }
    switch (command) 
    {
    case STREAM_MODE: {
        stream_mode_t* mode = (stream_mode_t*)pcmd->data;
        if (NULL != tof_sensor.set_streaming_type) {
            tof_sensor.set_streaming_type(mode->type);
        }

        if (STREAM_DISABLED == mode->type) {
            //DDEBUG("tof_sensor.video_streaming(false)========");
            tof_sensor.video_streaming(false);
            return 0;
        }
        ret = tof_sensor.video_streaming(true);
        if (ret < 0) {
            return ret;
        }
        break;
    }
    case REG16_RW: {

        uint16_t sensor_id = 0;
        ret = tof_sensor.get_sensor_id(&sensor_id);
        reg16_map_t* reg = (reg16_map_t*)(pcmd->data);

        if (sensor_id == OBC_SENSOR_ID_PLECO || OBC_SENSOR_ID_IMX518 == sensor_id || sensor_id == OBC_SENSOR_ID_RK1608_PLECO || sensor_id == OBC_SENSOR_ID_IMX316 || sensor_id == OBC_SENSOR_ID_GAEA) {
            if (nullptr == tof_sensor.sensor_write_reg_8) {
                return -HW_ERR_NO_SUPPORT;
            }
            ret = tof_sensor.sensor_write_reg_8(reg->addr, (uint8_t)reg->value);
			if (sensor_id == OBC_SENSOR_ID_PLECO) { // group hold release
				tof_sensor.sensor_write_reg_8(16, 1);
				tof_sensor.sensor_write_reg_8(16, 0);
			}
        }
        else if (OBC_SENSOR_ID_S5K33D == sensor_id || OBC_SENSOR_ID_RK1608_S5K33D == sensor_id) {
            
            if (nullptr == tof_sensor.sensor_write_reg_16) {
                return -HW_ERR_NO_SUPPORT;
            }
           // printf("sensor_write_reg_16 reg->addr %x, reg->value %x\n", reg->addr, reg->value);
            ret = tof_sensor.sensor_write_reg_16(reg->addr, reg->value);
            
        }
        else {
            ret = -HW_ERR_NO_SUPPORT;
        }

        
        break;
    }
   case EEPROM_RW: {
       if (NULL != tof_sensor.eeprom_write) {
           eeprom_data_t * pbuf = (eeprom_data_t*)pcmd->data;
           ret = tof_sensor.eeprom_write(pbuf->addr, pbuf->data, pbuf->len);
       }
       else {
           ret = -HW_ERR_NO_SUPPORT;
           ALOGE("---->eeprom_write HW_ERR_NO_SUPPORT");
       }
       break;
    }
    case FPS: {
        if (NULL != tof_sensor.set_fps) {
            uint32_t *pvalue = (uint32_t*)pcmd->data;
            ret = tof_sensor.set_fps((uint8_t)*pvalue);
            ALOGE("-->set_fps  %d,ret=%d",*pvalue,ret);
        }
        else {
            ret = -HW_ERR_NO_SUPPORT;
            ALOGE("---->set_fps HW_ERR_NO_SUPPORT");
        }
        break;
    }
    case LOAD_REF: {
        //ALOGE("%s LOAD_REF", __FUNCTION__);
        //TDODO ---  support  ref
        if (NULL != tof_sensor.load_ref_buffer) {
            ref_buffer_data_t * pbuf = (ref_buffer_data_t*)pcmd->data;

            ret = tof_sensor.load_ref_buffer(pbuf->addr, pbuf->data, pbuf->len);
            ALOGE("---->load_ref_buffer  ret=%d", ret);
        }
        else {
            ret = -HW_ERR_NO_SUPPORT;
            ALOGE("-->load_ref_buffer HW_ERR_NO_SUPPORT");
        }
        break;
    }
    case MASTER_SLAVE_MODE: {
        if (NULL != tof_sensor.set_master_slave_mode) {
            uint32_t *pvalue = (uint32_t*)pcmd->data;
            ret = tof_sensor.set_master_slave_mode((uint8_t)*pvalue);
        }
        else {
            ret = -HW_ERR_NO_SUPPORT;
            ALOGE("---->set_master_slave_mode HW_ERR_NO_SUPPORT");
        }
        break;
    }
    case ILLUM_POWER: {
        if (NULL != tof_sensor.set_illum_power) {
            ObcIllumPower *pvalue = (ObcIllumPower*)pcmd->data;
            ret = tof_sensor.set_illum_power(pvalue->vcsel, pvalue->value_A, pvalue->value_B);
            ALOGE("---->set_illum_power vcsel=%d,value_A=%d,value_B=%d ret=%d", pvalue->vcsel, pvalue->value_A, pvalue->value_B, ret);
        }
        else {
            ret = -HW_ERR_NO_SUPPORT;
            ALOGE("---->set_illum_power HW_ERR_NO_SUPPORT");
        }
        break;
    }
    case ILLUM_POWER_CTL: {
        if (NULL != tof_sensor.illum_power_control) {
            uint32_t *pvalue = (uint32_t*)pcmd->data;
            ret = tof_sensor.illum_power_control((uint8_t)*pvalue);
            ALOGE("-->illum_power_control %d,ret=%d,*pvalue,ret");

        }
        else {
            ret = -HW_ERR_NO_SUPPORT;
            ALOGE("---->illum_power_control HW_ERR_NO_SUPPORT");
        }
        break;
    }
    case INTEGRATION_TIME: {
        if (NULL != tof_sensor.set_integration_time) {
            uint32_t *pvalue = (uint32_t*)pcmd->data;
            ret = tof_sensor.set_integration_time((uint16_t)*pvalue);
        }
        else {
            ret = -HW_ERR_NO_SUPPORT;
            ALOGE("---->set_integration_time HW_ERR_NO_SUPPORT");
        }
        break;
    }
    case IR_EXP: {
        if (NULL != tof_sensor.set_exp) {
            uint32_t *pvalue = (uint32_t*)pcmd->data;
            ret = tof_sensor.set_exp((uint32_t)*pvalue);
        }
        else {
            ret = -HW_ERR_NO_SUPPORT;
            ALOGE("---->set_exp HW_ERR_NO_SUPPORT");
        }
        break;
    }
    case IR_GAIN: {
        if (NULL != tof_sensor.set_gain) {
            uint32_t *pvalue = (uint32_t*)pcmd->data;
            ret = tof_sensor.set_gain((uint32_t)*pvalue);
        }
        else {
            ret = -HW_ERR_NO_SUPPORT;
            ALOGE("---->set_gain HW_ERR_NO_SUPPORT");
        }
        break;
    }
	case ODD_DGAIN: {
		if (NULL != tof_sensor.set_odd_dgain) {
			uint32_t *pvalue = (uint32_t*)pcmd->data;
			ret = tof_sensor.set_odd_dgain((uint32_t)*pvalue);
		}
		else {
			ret = -HW_ERR_NO_SUPPORT;
			ALOGE("---->set_odd_dgain HW_ERR_NO_SUPPORT");
		}
		break;
	}
	case EVEN_DGAIN: {
		if (NULL != tof_sensor.set_even_dgain) {
			uint32_t *pvalue = (uint32_t*)pcmd->data;
			ret = tof_sensor.set_even_dgain((uint32_t)*pvalue);
		}
		else {
			ret = -HW_ERR_NO_SUPPORT;
			ALOGE("---->set_even_dgain HW_ERR_NO_SUPPORT");
		}
		break;
	}
	case SUB_SAMP: {
		if (NULL != tof_sensor.set_sub_samp) {
			uint8_t *pvalue = (uint8_t*)pcmd->data;
			ret = tof_sensor.set_sub_samp((uint8_t)*pvalue);
		}
		else {
			ret = -HW_ERR_NO_SUPPORT;
			ALOGE("---->set_sub_samp HW_ERR_NO_SUPPORT");
		}
		break;
	}
	case SUB_SAMP_V: {
		if (NULL != tof_sensor.set_sub_sampv) {
			uint8_t *pvalue = (uint8_t*)pcmd->data;
			ret = tof_sensor.set_sub_sampv((uint8_t)*pvalue);
		}
		else {
			ret = -HW_ERR_NO_SUPPORT;
			ALOGE("---->set_sub_samp HW_ERR_NO_SUPPORT");
		}
		break;
	}
	case DEEPSLEEP_MODE: {
		if (NULL != tof_sensor.deepsleep_mode) {
			uint8_t *pvalue = (uint8_t*)pcmd->data;
			ret = tof_sensor.deepsleep_mode((uint8_t)*pvalue);
		}
		else {
			ret = -HW_ERR_NO_SUPPORT;
			ALOGE("---->deepsleep_mode HW_ERR_NO_SUPPORT");
		}
		break;
	}
	case HDR_ALGORITHM: {
		if (NULL != tof_sensor.hdr_algorithm) {
			uint8_t *pvalue = (uint8_t*)pcmd->data;
			ret = tof_sensor.hdr_algorithm((uint8_t)*pvalue);
		}
		else {
			ret = -HW_ERR_NO_SUPPORT;
			ALOGE("---->hdr_algorithm HW_ERR_NO_SUPPORT");
		}
		break;
	}
	case HIST_ALGORITHM: {
		if (NULL != tof_sensor.hist_algorithm) {
			uint8_t *pvalue = (uint8_t*)pcmd->data;
			ret = tof_sensor.hist_algorithm((uint8_t)*pvalue);
		}
		else {
			ret = -HW_ERR_NO_SUPPORT;
			ALOGE("---->hist_algorithm HW_ERR_NO_SUPPORT");
		}
		break;
	}
	case MEDIAN_ALGORITHM: {
		if (NULL != tof_sensor.median_algorithm) {
			uint8_t *pvalue = (uint8_t*)pcmd->data;
			ret = tof_sensor.median_algorithm((uint8_t)*pvalue);
		}
		else {
			ret = -HW_ERR_NO_SUPPORT;
			ALOGE("---->median_algorithm HW_ERR_NO_SUPPORT");
		}
		break;
	}
	case EBC_ALGORITHM: {
		if (NULL != tof_sensor.ebc_algorithm) {
			uint8_t *pvalue = (uint8_t*)pcmd->data;
			ret = tof_sensor.ebc_algorithm((uint8_t)*pvalue);
		}
		else {
			ret = -HW_ERR_NO_SUPPORT;
			ALOGE("---->ebc_algorithm HW_ERR_NO_SUPPORT");
		}
		break;
	}

	case LSC_ALGORITHM: {
		if (NULL != tof_sensor.lsc_algorithm) {
			uint8_t *pvalue = (uint8_t*)pcmd->data;
			ret = tof_sensor.lsc_algorithm((uint8_t)*pvalue);
		}
		else {
			ret = -HW_ERR_NO_SUPPORT;
			ALOGE("---->ebc_algorithm HW_ERR_NO_SUPPORT");
		}
		break;
	}

	case CORRECTION_ALGORITHM: {
		if (NULL != tof_sensor.correction_algorithm) {
			uint8_t *pvalue = (uint8_t*)pcmd->data;
			ret = tof_sensor.correction_algorithm((uint8_t)*pvalue);
		}
		else {
			ret = -HW_ERR_NO_SUPPORT;
			ALOGE("---->correction_algorithm HW_ERR_NO_SUPPORT");
		}
		break;
	}

    case LASER: {
        if (NULL != tof_sensor.set_ldm_en) {
            uint32_t *pvalue = (uint32_t*)pcmd->data;
            ret = tof_sensor.set_ldm_en((uint32_t)*pvalue);
        }
        else {
            ret = -HW_ERR_NO_SUPPORT;
            ALOGE("---->set_ldm_en HW_ERR_NO_SUPPORT");
        }
        break;
    }
    case FLOOD_LED: {
        if (NULL != tof_sensor.set_led_en) {
            uint32_t *pvalue = (uint32_t*)pcmd->data;
            ret = tof_sensor.set_led_en((uint32_t)*pvalue);
        }
        else {
            ret = -HW_ERR_NO_SUPPORT;
            ALOGE("---->set_led_en HW_ERR_NO_SUPPORT");
        }
        break;
    }
    case LASER_CURRENT: {
        if (NULL != tof_sensor.set_ldm_current) {
            uint32_t *pvalue = (uint32_t*)pcmd->data;
            ret = tof_sensor.set_ldm_current((uint32_t)*pvalue);
        }
        else {
            ret = -HW_ERR_NO_SUPPORT;
            ALOGE("---->set_ldm_current HW_ERR_NO_SUPPORT");
        }
        break;
    }
    case FLOOD_CURRENT: {
        if (NULL != tof_sensor.set_led_current) {
            uint32_t *pvalue = (uint32_t*)pcmd->data;
            ret = tof_sensor.set_led_current((uint32_t)*pvalue);
        }
        else {
            ret = -HW_ERR_NO_SUPPORT;
            ALOGE("---->set_led_current HW_ERR_NO_SUPPORT");
        }
        break;
    }
    case CHIP_RESET: {
        if (NULL != tof_sensor.set_chip_reset) {
            uint32_t *pvalue = (uint32_t*)pcmd->data;
            ret = tof_sensor.set_chip_reset((uint32_t)*pvalue);
        }
        else {
            ret = -HW_ERR_NO_SUPPORT;
            ALOGE("---->set_chip_reset HW_ERR_NO_SUPPORT");
        }
        break;
    }
    case MODULATION_FREQUENCY: {
        if (NULL != tof_sensor.set_modulation_frequency){
            uint32_t *pvalue = (uint32_t*)pcmd->data;
            ret = tof_sensor.set_modulation_frequency((uint16_t)*pvalue);
            //ALOGE("---->set_modulation_frequency  %x,ret=%d", *pvalue,ret);
        }
        else {
            ret = -HW_ERR_NO_SUPPORT;
            //ALOGE("---->set_modulation_frequency HW_ERR_NO_SUPPORT");
        }
        break;
    }
    case DATA_OUTPUT_MODE: {
        if (NULL != tof_sensor.set_data_output_mode) {
            uint32_t *pvalue = (uint32_t*)pcmd->data;
            tof_sensor.video_streaming(false);
            ret = tof_sensor.set_data_output_mode((uint8_t)*pvalue);
            tof_sensor.video_streaming(true);
        }
        else {
            ret = -HW_ERR_NO_SUPPORT;
            ALOGE("---->set_data_output_mode HW_ERR_NO_SUPPORT");
        }
        break;
    }

	case WINDOW_ORIGINY: {
		if (NULL != tof_sensor.set_window_originy) {
			uint32_t *pvalue = (uint32_t*)pcmd->data;
			ret = tof_sensor.set_window_originy((uint32_t)*pvalue);
		}
		else {
			ret = -HW_ERR_NO_SUPPORT;
			ALOGE("---->set_window_originy HW_ERR_NO_SUPPORT");
		}
		break;
	}
	case WINDOW_ORIGINX: {
		if (NULL != tof_sensor.set_window_originx) {
			uint32_t *pvalue = (uint32_t*)pcmd->data;
			ret = tof_sensor.set_window_originx((uint32_t)*pvalue);
		}
		else {
			ret = -HW_ERR_NO_SUPPORT;
			ALOGE("---->set_window_originx HW_ERR_NO_SUPPORT");
		}
		break;
	}
	case WINDOW_HEIGHT: {
		if (NULL != tof_sensor.set_window_height) {
			uint32_t *pvalue = (uint32_t*)pcmd->data;
			ret = tof_sensor.set_window_height((uint32_t)*pvalue);
		}
		else {
			ret = -HW_ERR_NO_SUPPORT;
			ALOGE("---->set_window_height HW_ERR_NO_SUPPORT");
		}
		break;
	}
	case WINDOW_WIDTH: {
		if (NULL != tof_sensor.set_window_width) {
			uint32_t *pvalue = (uint32_t*)pcmd->data;
			ret = tof_sensor.set_window_width((uint32_t)*pvalue);
		}
		else {
			ret = -HW_ERR_NO_SUPPORT;
			ALOGE("---->set_window_width HW_ERR_NO_SUPPORT");
		}
		break;
	}

    case DUTY_CYCLE: {
        if (NULL != tof_sensor.set_illum_duty_cycle) {
            uint32_t *pvalue = (uint32_t*)pcmd->data;
            ret = tof_sensor.set_illum_duty_cycle((uint16_t)*pvalue);
        }
        else {
            ret = -HW_ERR_NO_SUPPORT;
            ALOGE("---->set_illum_duty_cycle HW_ERR_NO_SUPPORT");
        }
        break;
    }
    case MIRROR_FLIP: {
        if (NULL != tof_sensor.set_img_mirror_flip) {
            uint32_t *pvalue = (uint32_t*)pcmd->data;
            ret = tof_sensor.set_img_mirror_flip((uint8_t)*pvalue);
        }
        else {
            ret = -HW_ERR_NO_SUPPORT;
            ALOGE("---->set_img_mirror_flip HW_ERR_NO_SUPPORT");
        }
        break;
    }
    case TEST_PATTERN: {
        uint32_t *pvalue = (uint32_t*)pcmd->data;
        ret = tof_sensor.test_pattern((uint8_t)*pvalue);
        break;
    }
    case SOFTWARE_TRIGGER: {
        if (nullptr != tof_sensor.software_trigger) {
            ret = tof_sensor.software_trigger();
        }
        break;
    }
    case HARDWARE_TRIGGER: {
        if (nullptr != tof_sensor.hardware_trigger) {
            ret = tof_sensor.hardware_trigger();
        }
        break;
    }
    case ILLUM_POWER_TEST: {
        uint32_t *pvalue = (uint32_t*)pcmd->data;
        if (NULL != tof_sensor.illum_power_test) 
        {
            ret = tof_sensor.illum_power_test((uint8_t)*pvalue);
            if (0 == ret) 
            {
                illum_power_test_mode = (uint8_t)*pvalue;
            }
            //ALOGE("----->illum_power_test set mode=%d,ret=%d",*pvalue,ret);
            {
                uint8_t vcsel_num = 0;
                uint8_t value_A = 0;
                uint8_t value_B = 0;
                ret = tof_sensor.get_illum_power(&vcsel_num, &value_A, &value_B);
                ALOGE("--->get_illum_power vcsel_nume=%d,value_A=%d,value_B=%d,ret=%d",
                    vcsel_num, value_A, value_B, ret);
            }
        }
        else 
        {
            ret = -HW_ERR_NO_SUPPORT;
            ALOGE("--->set_illum_power HW_ERR_NO_SUPPORT");

        }
        break;
    }
    case AE: {
        if (NULL != tof_sensor.AE) {
            uint32_t *pvalue = (uint32_t*)pcmd->data;
            ret = tof_sensor.AE((bool)*pvalue);
            ALOGE("-->set_AE %d,ret=%d",*pvalue,ret);
        }
        else {
            ret = -HW_ERR_NO_SUPPORT;
        }
        break;
    }
    case DELAY_CIRCUIT_TIME: {
        uint32_t *pvalue = (uint32_t*)pcmd->data;
        if (NULL == pvalue) {
            return  -HW_ERR_NULL;
        }
        if (*pvalue > 0xffff) {
            return  -HW_ERR_INVALID;
        }
        ret = nb6l295_write(*pvalue);
        //DDEBUG("DELAY_CIRCUIT_TIME  GET %d", *pvalue);
        break;
    }
    case TX_A_B_POWER:
        if (NULL != tof_sensor.set_tx_a_b_power) {
            tx_a_b_power_t *pvalue = (tx_a_b_power_t*)pcmd->data;
            if (NULL == pvalue) {
                return  -HW_ERR_NULL;
            }
            ret = tof_sensor.set_tx_a_b_power(pvalue->a_state, pvalue->b_state);
            ALOGE("-->set_tx_a_b_power  %d,%d    ret=%d\n", pvalue->a_state, pvalue->b_state, ret);
            //Sleep(40);
            std::this_thread::sleep_for(std::chrono::milliseconds(40));
        }
        else {
            ret = -HW_ERR_NO_SUPPORT;
        }
        break;
    case  SENSOR_FREQUENCY_DUTY_CONFIG:
        if (NULL != tof_sensor.set_frequency_and_duty) {
            ObcSensorFreqDutyConfig *pvalue = (ObcSensorFreqDutyConfig*)pcmd->data;
            if (NULL == pvalue) {
                return  -HW_ERR_NULL;
            }
            ret = tof_sensor.set_frequency_and_duty(pvalue->index);
            
        }
        else {
            ret = -HW_ERR_NO_SUPPORT;
        }
        break;
    case SHUFFLE:
        if (NULL != tof_sensor.set_shuffle_mode) {
            bool *pvalue = (bool*)pcmd->data;
            if (NULL == pvalue) {
                return  -HW_ERR_NULL;
            }
            ret = tof_sensor.set_shuffle_mode(*pvalue);
            //ALOGE("set_shuffle_mode %d,ret=%d", *pvalue,ret);
        }
        else {
            ret = -HW_ERR_NO_SUPPORT;
        }
        break;
    case BINNING_MODE:
		if (NULL != tof_sensor.set_pixel_binning) {
            uint8_t *pvalue = (uint8_t*)pcmd->data;
            if (NULL == pvalue) {
                return  -HW_ERR_NULL;
            }
			ret = tof_sensor.set_pixel_binning(*pvalue);
            ALOGE("set_pixel_binning %d,ret= %d", *pvalue,ret);
        }
        else {
            ret = -HW_ERR_NO_SUPPORT;
        }
        break;
    case DRIVER_IC_REG8_RW:
    {
        reg16_map_t* reg = (reg16_map_t*)(pcmd->data);
        if (nullptr == tof_sensor.driver_ic_write_reg_8) {
            return -HW_ERR_NO_SUPPORT;
        }
        ret = tof_sensor.driver_ic_write_reg_8(reg->addr, (uint8_t)reg->value);
       
        break;
    }
    case BURST_MODE:
        if (NULL != tof_sensor.set_burst_mode) {
            uint8_t *pvalue = (uint8_t*)pcmd->data;
            if (NULL == pvalue) {
                return  -HW_ERR_NULL;
            }
            ret = tof_sensor.set_burst_mode(*pvalue);
            //ALOGE("get_shuffle_mode %d,ret= %d", *pvalue,ret);
        }
        else {
            ret = -HW_ERR_NO_SUPPORT;
        }
        break;
    case FREQUENCY_MODE:
    {
        if (NULL != tof_sensor.set_frequency_mode) {
            uint8_t *pvalue = (uint8_t*)pcmd->data;
            if (NULL == pvalue) {
                return -HW_ERR_NULL;
            }
            ret = tof_sensor.set_frequency_mode(*pvalue);
            ALOGE("-->set_frequency_mode %d,ret= %d", *pvalue, ret);
        }
        else {
            ALOGE("HW_ERR_NO_SUPPORT");
            ret = -HW_ERR_NO_SUPPORT;
        }
        break;
    }
    case VMGHI_VOLTAGE:
    {
        if (NULL != tof_sensor.set_vmghi_voltage) {
            uint8_t *pvalue = (uint8_t*)pcmd->data;
            if (NULL == pvalue) {
                return -HW_ERR_NULL;
            }
            ret = tof_sensor.set_vmghi_voltage(*pvalue);
        }
        else {
            ret = -HW_ERR_NO_SUPPORT;
        }
        break;
    }
    case TOF_CONFIG:
    {
        if (NULL != tof_sensor.set_tof_config) {
            ObToFConfig *pvalue = (ObToFConfig *)pcmd->data;
            if (NULL == pvalue) {
                return -HW_ERR_NULL;
            }
            ret = tof_sensor.set_tof_config(0, pvalue);
        }
        else {
            ret = -HW_ERR_NO_SUPPORT;
        }
        break;
    }
    case LASER_DRIVER_VOLTAGE:
    {
        if (nullptr != tof_sensor.set_ld_vcc)
        {
            uint16_t *pvalue = (uint16_t*)pcmd->data;
            ret = tof_sensor.set_ld_vcc(*pvalue);
            ALOGD("tof_sensor.set_ld_vcc:   %d", *pvalue);
        }
        else
        {
            ret = -HW_ERR_NO_SUPPORT;
        }
        break;
    }
    case DELAY_BOARD_POWER:
    {
        uint8_t *pvalue = (uint8_t*)pcmd->data;
        ret = delay_board_set_power_on_off(*pvalue);
        break;
    }
    case DELAY_BOARD_DELAY_SWITCH:
    {
        uint8_t *pvalue = (uint8_t*)pcmd->data;
        ret = delay_board_set_delay_select(*pvalue);
        break;
    }
    case DELAY_BOARD_DELAY_TIME:
    {
        uint32_t *pvalue = (uint32_t*)pcmd->data;
        ret = delay_board_set_delay_time(*pvalue);
        break;
    }
    case OBC_TOF_PROJECT_TEST:
    {
        if (nullptr == tof_sensor.set_aa_qc_test)
        {
            ALOGD("OBC_TOF_PROJECT_TEST set_aa_qc_test nullptr");
            return -HW_ERR_NULL;
        }
        uint32_t *pvalue = (uint32_t*)pcmd->data;
        ret = tof_sensor.set_aa_qc_test(*pvalue);
        ALOGD("OBC_TOF_PROJECT_TEST index = %d, ret = %d", *pvalue, ret);
        break;
    }

    default:
        ret = -HW_ERR_NO_SUPPORT;
        break;
    }

    return ret;
}

int tof_slave_get_property(hw_command_t command, command_data_t* pdata)
{
    if (!is_init_) 
    {
        return -HW_ERR_NO_INIT;
    }
    if (nullptr == pdata)
    {
        return -HW_ERR_NULL;
    }

    int ret = HW_NO_ERR;

    switch (command)
    {
    case ENGINE_ID: {
        if (pdata->len < sizeof(uint64_t)) {
            return -HW_ERR_INVALID;
        }
        char *id = (char*)pdata->data;
        //ret = tof_slave_get_sensor_id((char*)pdata->data, pdata->len);
        uint16_t sensor_id = 0;
        ret = tof_sensor.get_sensor_id(&sensor_id);
        if (ret < 0 || sensor_id == 0) {
            DDEBUG("get_sensor_id %x,ret=%d\n", sensor_id, ret);
            return  -HW_ERR_INVALID;
        }
        //DDEBUG("--->ENGINE_ID %x\n", sensor_id);
        pdata->len = strlen(HAL_PROTOCOL_SYM.sensorname) + 1;
        //memcpy(id, HAL_PROTOCOL_SYM.sensorname, strlen(HAL_PROTOCOL_SYM.sensorname));
        strcpy(id, HAL_PROTOCOL_SYM.sensorname);
        id[pdata->len - 1] = '\0';
        break;
    }
    case MODULE_NAME: {
        if (pdata->len < sizeof(uint64_t)) {
            return -HW_ERR_INVALID;
        }
        char *id = (char*)pdata->data;
        memset(id, 0, pdata->len);
        uint16_t sensor_id = 0;
        ret = tof_sensor.get_sensor_id(&sensor_id);
        //DDEBUG("--->ENGINE_ID %x\n", sensor_id);
        sprintf(id, "%x", sensor_id);
        pdata->len = strlen(id);
        break;
    }
    case SENSOR_ID: {
        if (pdata->len != sizeof(uint32_t)) {
            return -HW_ERR_INVALID;
        }
        uint32_t *id = (uint32_t*)pdata->data;
        uint16_t sensor_id = 0;
        ret = tof_sensor.get_sensor_id(&sensor_id);
        //DDEBUG("--->ENGINE_ID %x\n", sensor_id);
        *id = sensor_id;
        break;
    }
    case MODULE_VERSION: {
        if (pdata->len < sizeof(uint32_t)) {
            return -HW_ERR_INVALID;
        }
        uint32_t *pversion = (uint32_t*)pdata->data;
        ret = tof_get_lib_version(pversion);
        pdata->len = sizeof(uint32_t);
        break;
    }
    case STREAMS_CAPACITY: {
        //memcpy(pdata->data, support_stream_, sizeof(support_stream_));
        pdata->data = support_stream_;
        pdata->len = sizeof(support_stream_) / sizeof(support_stream_[0]);
        break;
    }
    case REG16_RW: {
        uint16_t sensor_id = 0;
        ret = tof_sensor.get_sensor_id(&sensor_id);
        reg16_map_t* reg = (reg16_map_t*)(pdata->data);

        if (sensor_id == OBC_SENSOR_ID_PLECO || OBC_SENSOR_ID_IMX518 == sensor_id || sensor_id == OBC_SENSOR_ID_RK1608_PLECO || sensor_id == OBC_SENSOR_ID_IMX316 || sensor_id == OBC_SENSOR_ID_GAEA) {
            if (nullptr == tof_sensor.sensor_read_reg_8) {
                return -HW_ERR_NO_SUPPORT;
            }
            uint8_t value = 0;
            ret = tof_sensor.sensor_read_reg_8(reg->addr, &value);
            {
                //度信的接口  DT_ERROR_OK=1
                reg->value = value;
                ret = 0;
            }
        }
        else if (OBC_SENSOR_ID_S5K33D == sensor_id || OBC_SENSOR_ID_RK1608_S5K33D == sensor_id) {

            if (nullptr == tof_sensor.sensor_read_reg_16) {
                return -HW_ERR_NO_SUPPORT;
            }
            ret = tof_sensor.sensor_read_reg_16(reg->addr, &reg->value);

        }
        else {
            ret = -HW_ERR_NO_SUPPORT;
        }
        break;
    }

    case LOAD_REF: {
        ref_buffer_data_t* buffer_data = (ref_buffer_data_t*)pdata->data;
        if (nullptr == tof_sensor.get_ref_buffer_addr) {
            DDEBUG("nullptr == tof_sensor.get_ref_buffer_addr");
            return -HW_ERR_NO_SUPPORT;
        }
        buffer_data->addr = 0;
        DDEBUG("begin get_ref_buffer_addr %x\n", buffer_data->addr);
        ret = tof_sensor.get_ref_buffer_addr((uint32_t*)&buffer_data->addr);
        DDEBUG("end get_ref_buffer_addr %x\n", buffer_data->addr);
        if (ret == 1) {
            //度信的接口  DT_ERROR_OK=1
            ret = 0;
        }
        break;
    }
    case EEPROM_RW: {
        eeprom_data_t* eeprom = (eeprom_data_t*)pdata->data;
        ret = tof_sensor.eeprom_read(eeprom->addr, eeprom->data, eeprom->len);
        break;
    }
    case SUPPORT_VIDEO_MODES: {
        if (pdata->len < sizeof(support_modes_mlx75027_) / sizeof(support_modes_mlx75027_[0])) {
            return  -HW_ERR_NO_MEM;
        }
        uint16_t sensor_id = 0;
        ret = tof_sensor.get_sensor_id(&sensor_id);
        //printf("------->sensor_id %x\n", sensor_id);
        if (sensor_id == 0x5027 || sensor_id == 0x0516
            || sensor_id == 0x0518
            || sensor_id == imx456_sensor_id
            ) {
            memcpy(pdata->data, support_modes_mlx75027_, sizeof(support_modes_mlx75027_));
            pdata->len = sizeof(support_modes_mlx75027_) / sizeof(support_modes_mlx75027_[0]);
        }
        else if (sensor_id == OBC_SENSOR_ID_PLECO) {
            memcpy(pdata->data, support_modes_pleco_, sizeof(support_modes_pleco_));
            pdata->len = sizeof(support_modes_pleco_) / sizeof(support_modes_pleco_[0]);
        }
        else if (sensor_id == OBC_SENSOR_ID_RK1608_PLECO) {
            memcpy(pdata->data, support_modes_pleco_rk1608_, sizeof(support_modes_pleco_rk1608_));
            pdata->len = sizeof(support_modes_pleco_rk1608_) / sizeof(support_modes_pleco_rk1608_[0]);
        }
        else if (sensor_id == OBC_SENSOR_ID_RK1608_S5K33D) {
            memcpy(pdata->data, support_modes_rk1608_s5k33d_, sizeof(support_modes_rk1608_s5k33d_));
            pdata->len = sizeof(support_modes_rk1608_s5k33d_) / sizeof(support_modes_rk1608_s5k33d_[0]);
        }
        else {
            memcpy(pdata->data, support_modes_s5k33d_, sizeof(support_modes_s5k33d_));
            pdata->len = sizeof(support_modes_s5k33d_) / sizeof(support_modes_s5k33d_[0]);
        }
        ret = 0;
        break;
    }
    case FPS: {
        if (NULL != tof_sensor.get_fps) {
            uint32_t *pversion = (uint32_t*)pdata->data;
            uint8_t fs = 0;
            ret = tof_sensor.get_fps(&fs);
            *pversion = fs;
            //DDEBUG("FPS %d\n", fs);
        }
        else {
            ret = -HW_ERR_NO_SUPPORT;
            ALOGE("---->get_fps HW_ERR_NO_SUPPORT");
        }
        break;
    }
    case SENSOR_TEMP: {
        if (NULL != tof_sensor.get_sensor_temperature) {
            float *pversion = (float*)pdata->data;
            float value = 0;
            ret = tof_sensor.get_sensor_temperature(&value);
            *pversion = value;
            //DEBUG("SENSOR_TEMP %d\n", value);
        }
        else {
            ret = -HW_ERR_NO_SUPPORT;
            ALOGE("---->get_sensor_temperature HW_ERR_NO_SUPPORT");
        }
        break;
    }
    case RX_TEMP: {
        if (NULL != tof_sensor.get_rx_temp) {
            float *pvalue = (float*)pdata->data;
            ret = tof_sensor.get_rx_temp(pvalue);
            //DDEBUG("RX_TEMP %f\n", *pvalue);
        }
        else {
            ret = -HW_ERR_NO_SUPPORT;
            ALOGE("---->get_rx_temp HW_ERR_NO_SUPPORT");
        }
        break;
    }
    case TX_TEMP: {
        if (NULL != tof_sensor.get_tx_temp) {
            float *pvalue = (float*)pdata->data;
            ret = tof_sensor.get_tx_temp(pvalue);
            //DDEBUG("TX_TEMP %f\n", *pvalue);
        }
        else {
            ret = -HW_ERR_NO_SUPPORT;
            ALOGE("---->get_tx_temp HW_ERR_NO_SUPPORT");
        }
        break;
    }
    case ILLUM_POWER: {
        if (NULL != tof_sensor.get_illum_power) {
            ObcIllumPower *pvalue = (ObcIllumPower*)pdata->data;
            uint8_t vcsel_num = 0;
            uint8_t value_A = 0;
            uint8_t value_B = 0;
            ret = tof_sensor.get_illum_power(&vcsel_num, &value_A, &value_B);
            pvalue->vcsel = vcsel_num;
            pvalue->value_A = value_A;
            pvalue->value_B = value_B;
            ALOGE("---->get_illum_power vcsel=%d,value_A=%d,value_B=%d ret=%d", pvalue->vcsel, pvalue->value_A, pvalue->value_B, ret);

        }
        else {
            ret = -HW_ERR_NO_SUPPORT;
            ALOGE("---->get_illum_power HW_ERR_NO_SUPPORT");
        }
        break;
    }
#if 0
    case ILLUM_POWER_CTL: {
        uint32_t *pversion = (uint32_t*)pdata->data;
        uint8_t value = 0;
        ret = tof_sensor.illum_power_control(&value);
        *pversion = value;
        DDEBUG("ILLUM_POWER_CTL  %d", value);
        break;
    }
#endif

    case INTEGRATION_TIME: {
        if (NULL != tof_sensor.get_integration_time) {
            uint32_t *pvalue = (uint32_t*)pdata->data;
            uint16_t value = 0;
            ret = tof_sensor.get_integration_time(&value);
            *pvalue = value;
            //DDEBUG("INTEGRATION_TIME %d", value);
        }
        else {
            ret = -HW_ERR_NO_SUPPORT;
            ALOGE("---->get_integration_time HW_ERR_NO_SUPPORT");
        }
        break;
    }
    case IR_EXP: {
        if (NULL != tof_sensor.get_exp) {
            uint32_t *pvalue = (uint32_t*)pdata->data;
            uint32_t value = 0;
            ret = tof_sensor.get_exp(&value);
            *pvalue = value;
            //DDEBUG("EXP %d", value);
        }
        else {
            ret = -HW_ERR_NO_SUPPORT;
            ALOGE("---->get_exp HW_ERR_NO_SUPPORT");
        }
        break;
    }
    case IR_GAIN: {
        if (NULL != tof_sensor.get_gain) {
            uint32_t *pvalue = (uint32_t*)pdata->data;
            uint32_t value = 0;
            ret = tof_sensor.get_gain(&value);
            *pvalue = value;
            //DDEBUG("GAIN %d", value);
        }
        else {
            ret = -HW_ERR_NO_SUPPORT;
            ALOGE("---->get_gain HW_ERR_NO_SUPPORT");
        }
        break;
    }
	case SUB_SAMP: {
		if (NULL != tof_sensor.get_sub_samp) {
			uint8_t *pvalue = (uint8_t*)pdata->data;
			uint8_t value = 0;
			ret = tof_sensor.get_sub_samp(&value);
			*pvalue = value;
			//DDEBUG("GAIN %d", value);
		}
		else {
			ret = -HW_ERR_NO_SUPPORT;
			ALOGE("---->get_sub_samp HW_ERR_NO_SUPPORT");
		}
		break;
	}

	case SUB_SAMP_V: {
		if (NULL != tof_sensor.get_sub_sampv) {
			uint8_t *pvalue = (uint8_t*)pdata->data;
			uint8_t value = 0;
			ret = tof_sensor.get_sub_sampv(&value);
			*pvalue = value;
			//DDEBUG("GAIN %d", value);
		}
		else {
			ret = -HW_ERR_NO_SUPPORT;
			ALOGE("---->get_sub_sampv HW_ERR_NO_SUPPORT");
		}
		break;
	}
    case LASER: {
        if (NULL != tof_sensor.get_ldm_en) {
            uint32_t *pvalue = (uint32_t*)pdata->data;
            uint32_t value = 0;
            ret = tof_sensor.get_ldm_en(&value);
            *pvalue = value;
            //DDEBUG("GAIN %d", value);
        }
        else {
            ret = -HW_ERR_NO_SUPPORT;
            ALOGE("---->get_ldm_en HW_ERR_NO_SUPPORT");
        }
        break;
    }
    case FLOOD_LED: {
        if (NULL != tof_sensor.get_led_en) {
            uint32_t *pvalue = (uint32_t*)pdata->data;
            uint32_t value = 0;
            ret = tof_sensor.get_led_en(&value);
            *pvalue = value;
            //DDEBUG("GAIN %d", value);
        }
        else {
            ret = -HW_ERR_NO_SUPPORT;
            ALOGE("---->get_led_en HW_ERR_NO_SUPPORT");
        }
        break;
    }
    case LASER_CURRENT: {
        if (NULL != tof_sensor.get_ldm_current) {
            uint32_t *pvalue = (uint32_t*)pdata->data;
            uint32_t value = 0;
            ret = tof_sensor.get_ldm_current(&value);
            *pvalue = value;
            //DDEBUG("GAIN %d", value);
        }
        else {
            ret = -HW_ERR_NO_SUPPORT;
            ALOGE("---->get_ldm_current HW_ERR_NO_SUPPORT");
        }
        break;
    }
    case FLOOD_CURRENT: {
        if (NULL != tof_sensor.get_led_current) {
            uint32_t *pvalue = (uint32_t*)pdata->data;
            uint32_t value = 0;
            ret = tof_sensor.get_led_current(&value);
            *pvalue = value;
            //DDEBUG("GAIN %d", value);
        }
        else {
            ret = -HW_ERR_NO_SUPPORT;
            ALOGE("---->get_led_current HW_ERR_NO_SUPPORT");
        }
        break;
    }
    case MODULATION_FREQUENCY: {
        if (NULL != tof_sensor.get_modulation_frequency) {
            uint32_t *pvalue = (uint32_t*)pdata->data;
            uint16_t value = 0;
            ret = tof_sensor.get_modulation_frequency(&value);
            *pvalue = value;
            ALOGE("---->get_modulation_frequency  %x,ret=%d", value, ret);
        }
        else {
            ret = -HW_ERR_NO_SUPPORT;
            ALOGE("---->get_modulation_frequency HW_ERR_NO_SUPPORT");
        }
        break;
    }
    case DATA_OUTPUT_MODE: {
        if (NULL != tof_sensor.get_data_output_mode) {
            uint32_t *pvalue = (uint32_t*)pdata->data;
            uint8_t value = 0;
            ret = tof_sensor.get_data_output_mode(&value);
            *pvalue = value;
            //DDEBUG("DATA_OUTPUT_MODE %d", value);
        }
        else {
            ret = -HW_ERR_NO_SUPPORT;
            ALOGE("---->get_data_output_mode HW_ERR_NO_SUPPORT");
        }
        break;
    }
	case WINDOW_HEIGHT: {
		if (NULL != tof_sensor.get_window_height) {
			uint32_t *pvalue = (uint32_t*)pdata->data;
			uint32_t value = 0;
			ret = tof_sensor.get_window_height(&value);
			*pvalue = value;
		}
		else {
			ret = -HW_ERR_NO_SUPPORT;
			ALOGE("---->get_window_height HW_ERR_NO_SUPPORT");
		}
		break;
	}

	case WINDOW_WIDTH: {
		if (NULL != tof_sensor.get_window_width) {
			uint32_t *pvalue = (uint32_t*)pdata->data;
			uint32_t value = 0;
			ret = tof_sensor.get_window_width(&value);
			*pvalue = value;
		}
		else {
			ret = -HW_ERR_NO_SUPPORT;
			ALOGE("---->get_window_width HW_ERR_NO_SUPPORT");
		}
		break;
	}

    case DUTY_CYCLE: {
        if (NULL != tof_sensor.get_illum_duty_cycle) {
            uint32_t *pvalue = (uint32_t*)pdata->data;
            uint16_t value = 0;
            ret = tof_sensor.get_illum_duty_cycle(&value);
            *pvalue = value;
        }
        else {
            ret = -HW_ERR_NO_SUPPORT;
            ALOGE("---->get_illum_duty_cycle HW_ERR_NO_SUPPORT");
        }
        break;
    }
    case MIRROR_FLIP: {
        if (NULL != tof_sensor.get_img_mirror_flip) {
            uint32_t *pvalue = (uint32_t*)pdata->data;
            uint8_t value = 0;
            ret = tof_sensor.get_img_mirror_flip(&value);
            *pvalue = value;
        }
        else {
            ret = -HW_ERR_NO_SUPPORT;
            ALOGE("---->get_img_mirror_flip HW_ERR_NO_SUPPORT");
        }
        break;
    }
    case DISP_BITS: {
        //mipi  transmission pack type
        uint32_t *pvalue = (uint32_t*)pdata->data;

        uint16_t sensor_id = 0;
        ret = tof_sensor.get_sensor_id(&sensor_id);
        if (OBC_SENSOR_ID_MLX75027 == sensor_id || OBC_SENSOR_ID_IMX516 == sensor_id
            || OBC_SENSOR_ID_IMX518 == sensor_id || OBC_SENSOR_ID_IMX456 == sensor_id)
        {
            *pvalue = 12;
        }
        else if (OBC_SENSOR_ID_RK1608_S5K33D == sensor_id
            || OBC_SENSOR_ID_RK1608_PLECO == sensor_id)
        {
            *pvalue = 8;
        }
        else
        {
            *pvalue = 10;
        }
        //DDEBUG("DISP_BITS %d", *pvalue);
        break;
    }
    case EXT_PARAMS:
    {
        if (pdata->len != sizeof(hw_ext_msg_t))
        {
            ALOGE("pdata->len %d,sizeof(hw_ext_msg_t %d", pdata->len, sizeof(hw_ext_msg_t));
            ret = -HW_ERR_INVALID;
            break;
        }
        hw_ext_msg_t *ext_param = (hw_ext_msg_t*)pdata->data;
        if (ext_param->subcmd == GET_SUPPORT_RESOLUTION && (*(int*)(ext_param->msg) == STREAM_TOF_PHASE || *(int*)(ext_param->msg) == STREAM_DEPTH))
        {
            if (*(int*)(ext_param->msg) == STREAM_TOF_PHASE)
            {
                if (nullptr == tof_sensor.get_tof_sensor_resolution)
                {
                    ALOGE("tof_sensor.get_tof_sensor_resolution is nullptr");
                    ret = -HW_ERR_NULL;
                }
                uint64_t *pvalue = (uint64_t*)ext_param->p_data;
                ret = tof_sensor.get_tof_sensor_resolution(pvalue);
            }
            else if (*(int*)(ext_param->msg) == STREAM_DEPTH)
            {
                uint64_t *pvalue = (uint64_t*)ext_param->p_data;
                *pvalue = OB_TOF_RESOLUTION_320_240 << 8 | OB_TOF_RESOLUTION_640_480;//TODO 通过函数查询

                ret = HW_NO_ERR;
            }
        }
        else
        {
            ALOGE("ext_param is unsupported, subcmd = %d, msg = %d", ext_param->subcmd, *(int*)(ext_param->msg));
            ret = -HW_ERR_INVALID;
        }
        break;
    }
    case SENSOR_INFO: {
        if (pdata->len != sizeof(sensor_info))
        {
            ALOGE("pdata->len: %d, sizeof(sensor_info): %d", pdata->len, sizeof(sensor_info));
            ret = -HW_ERR_INVALID;
            break;
        }
        sensor_info* info = (sensor_info*)pdata->data;
        ret = tof_sensor.get_sensor_info(info);
        break;
    }
    case DUTY_CYCLE_LIST: {
        if (NULL != tof_sensor.get_illum_duty_cycle_list) {
            if (pdata->len != sizeof(duty_cycle_list)) {
                ret = -HW_ERR_INVALID;
                break;
            }
            duty_cycle_list* cycle_list = (duty_cycle_list*)pdata->data;
            ret = tof_sensor.get_illum_duty_cycle_list(cycle_list->frequency, cycle_list->duty_cycle_steps);
            //DDEBUG("DUTY_CYCLE_LIST frequency = %d, ret = %d", cycle_list->frequency, ret);
        }
        else {
            ret = -HW_ERR_NO_SUPPORT;
            ALOGE("---->get_illum_duty_cycle_list HW_ERR_NO_SUPPORT");
        }
        break;
    }
    case VCSEL_PD: {
        uint32_t *pvalue = (uint32_t*)pdata->data;
        uint32_t pd = 0;
        if (NULL != tof_sensor.get_vcsel_pd) {
            ret = tof_sensor.get_vcsel_pd(&pd);
            *pvalue = pd;
            ALOGE("---->get_vcsel_pd %d,ret=%d", pd, ret);
        }
        else {
            ret = -HW_ERR_NO_SUPPORT;
        }
        break;
    }
    case ILLUM_POWER_TEST: {
        if (NULL != tof_sensor.illum_power_test) {
            uint32_t *pvalue = (uint32_t*)pdata->data;
            *pvalue = illum_power_test_mode;
            ret = 0;
            //ALOGE("----->illum_power_test get mode=%d,ret=%d", illum_power_test_mode, ret);
        }
        else {
            ret = -HW_ERR_NO_SUPPORT;
            ALOGE("----->illum_power_test HW_ERR_NO_SUPPORT");
        }
        break;
    }
    case DEFAULT_PARAMS: {
        DriverTofRequestParam *pvalue = (DriverTofRequestParam*)pdata->data;
        uint16_t sensor_id = 0;
        ret = tof_sensor.get_sensor_id(&sensor_id);
        if (ret < 0) {
            DDEBUG("tof_sensor.get_sensor_id ret= %d", ret);
            break;
        }
        else {
            switch (sensor_id) {
            case OBC_SENSOR_ID_RK1608_PLECO:
            case OBC_SENSOR_ID_PLECO: {

                uint8_t  value8 = 0;
                uint16_t  integration_time = 0;
                //ret = tof_sensor.get_frequency_mode(&value8);
                //if (ret < 0) {
                //    DDEBUG("tof_sensor.get_frequency_mode ret= %d", ret);
                //}
                ret = tof_sensor.get_integration_time((uint16_t*)&integration_time);
                if (ret < 0) {
                    DDEBUG("tof_sensor.set_integration_time ret= %d", ret);
                }

                uint16_t duty_cycle = 0;
                ret = tof_sensor.get_illum_duty_cycle(&duty_cycle);
                if (ret < 0) {
                    DDEBUG("tof_sensor.get_illum_duty_cycle ret= %d", ret);
                }

                pvalue->frequency_mode = OBC_DUAL_FREQUENCY;
                pvalue->outmode = OB_TOF_OUT_MODE_3TAP;
                pvalue->param[1].duty_cycle = duty_cycle & 0xFF;
                pvalue->param[1].integration_time = integration_time;

                pvalue->param[0].duty_cycle = duty_cycle >> 8 & 0xff;
                pvalue->param[0].integration_time = integration_time;

                uint16_t freq = 0;
                ret = tof_sensor.get_modulation_frequency(&freq);
                if (ret == 0) {
                    pvalue->param[0].frequency = (freq >> 8) & 0x0F;
                    pvalue->param[1].frequency = freq & 0x0F;
                }
            }
                                  break;
            case 0x456: {
                pvalue->frequency_mode = OBC_DUAL_FREQUENCY;
                pvalue->outmode = OB_TOF_OUT_MODE_A_AND_B;
#if 0
                pvalue->param[0].duty_cycle = 8;
                pvalue->param[0].integration_time = 1000;
                pvalue->param[0].frequency = 60;

                pvalue->param[1].duty_cycle = 12;
                pvalue->param[1].integration_time = 1000;
                pvalue->param[1].frequency = 100;
#else
                pvalue->param[1].duty_cycle = 8;
                pvalue->param[1].integration_time = 1000;
                pvalue->param[1].frequency = 60;

                pvalue->param[0].duty_cycle = 12;
                pvalue->param[0].integration_time = 1000;
                pvalue->param[0].frequency = 100;
#endif
            }
                        break;
            case 0x303d: {
                uint8_t  freq_mode = 0;
                ret = tof_sensor.get_frequency_mode(&freq_mode);
                if (ret < 0) {
                    DDEBUG("tof_sensor.get_frequency_mode ret= %d", ret);
                }

                uint16_t  integration_time = 0;
                ret = tof_sensor.get_integration_time((uint16_t*)&integration_time);
                if (ret < 0) {
                    DDEBUG("tof_sensor.set_integration_time ret= %d", ret);
                }
                uint16_t duty_cycle = 0;
                ret = tof_sensor.get_illum_duty_cycle(&duty_cycle);
                if (ret < 0) {
                    DDEBUG("tof_sensor.get_illum_duty_cycle ret= %d", ret);
                }

                pvalue->frequency_mode = (ObcFrequencyMode)freq_mode;
                pvalue->outmode = OB_TOF_OUT_MODE_S5K_4TAP_DUAL_FREQ;
                pvalue->param[1].duty_cycle = duty_cycle & 0xFF;
                pvalue->param[1].integration_time = integration_time;
                pvalue->param[0].duty_cycle = duty_cycle >> 8 & 0xff;
                pvalue->param[0].integration_time = integration_time;

                uint16_t freq = 0;
                ret = tof_sensor.get_modulation_frequency(&freq);
                if (ret == 0) {
                    pvalue->param[0].frequency = (freq >> 8) & 0x0F;
                    pvalue->param[1].frequency = freq & 0x0F;
                }

#if 0
                //黄山
                pvalue->frequency_mode = OBC_DUAL_FREQUENCY;
                pvalue->outmode = OB_TOF_OUT_MODE_S5K_4TAP_DUAL_FREQ;
                pvalue->param[0].duty_cycle = 16;//16：50%     7; //37.16
                pvalue->param[0].integration_time = 600;
                pvalue->param[0].frequency = 100;

                pvalue->param[1].duty_cycle = 16;//16：50%，   5;    //34.2
                pvalue->param[1].integration_time = 600;
                pvalue->param[1].frequency = 80;
#endif
#if 0
                pvalue->frequency_mode = OBC_DUAL_FREQUENCY;
                pvalue->outmode = OB_TOF_OUT_MODE_S5K_4TAP_DUAL_FREQ;
                pvalue->param[1].duty_cycle = 7; //37.16
                pvalue->param[1].integration_time = 600;
                pvalue->param[1].frequency = 100;

                pvalue->param[0].duty_cycle = 5;    //34.2
                pvalue->param[0].integration_time = 600;
                pvalue->param[0].frequency = 80;

                //泰山
                pvalue->frequency_mode = OBC_DUAL_FREQUENCY;
                pvalue->outmode = OB_TOF_OUT_MODE_S5K_4TAP_DUAL_FREQ;
                pvalue->param[0].duty_cycle = 16;// 7; //37.16
                pvalue->param[0].integration_time = 600;
                pvalue->param[0].frequency = 100;

                pvalue->param[1].duty_cycle = 16;// 5;    //34.2
                pvalue->param[1].integration_time = 600;
                pvalue->param[1].frequency = 10;
#endif
            }
                         break;
            case OBC_SENSOR_ID_RK1608_S5K33D: {
                uint8_t  value8 = 0;
                uint16_t  integration_time = 0;
                ret = tof_sensor.get_frequency_mode(&value8);
                if (ret < 0) {
                    DDEBUG("tof_sensor.get_frequency_mode ret= %d", ret);
                }
                ret = tof_sensor.get_integration_time((uint16_t*)&integration_time);
                if (ret < 0) {
                    DDEBUG("tof_sensor.set_integration_time ret= %d", ret);
                }
                uint16_t modulation_frequency = 0;
                ret = tof_sensor.get_modulation_frequency(&modulation_frequency);
                if (ret < 0) {
                    DDEBUG("tof_sensor.get_modulation_frequency ret= %d", ret);
                }
                // rk1608
                pvalue->frequency_mode = (ObcFrequencyMode)value8;
                pvalue->outmode = OB_TOF_OUT_MODE_S5K_4TAP_DUAL_FREQ;
                pvalue->param[0].duty_cycle = 16;//16：50%     7; //37.16
                pvalue->param[0].integration_time = integration_time;
                pvalue->param[0].frequency = modulation_frequency >> 8;

                pvalue->param[1].duty_cycle = 16;//16：50%，   5;    //34.2
                pvalue->param[1].integration_time = integration_time;
                pvalue->param[1].frequency = modulation_frequency & 0xFF;
                break;
            }
            case OBC_SENSOR_ID_IMX316:
            case OBC_SENSOR_ID_IMX627: 
            {
                uint8_t  value8 = 0;
                uint16_t  integration_time = 0;
                //ret = tof_sensor.get_frequency_mode(&value8);
                //if (ret < 0) {
                //    DDEBUG("tof_sensor.get_frequency_mode ret= %d", ret);
                //}
                ret = tof_sensor.get_integration_time((uint16_t*)&integration_time);
                if (ret < 0) {
                    DDEBUG("tof_sensor.set_integration_time ret= %d", ret);
                }

                uint16_t duty_cycle = 0;
                ret = tof_sensor.get_illum_duty_cycle(&duty_cycle);
                if (ret < 0) {
                    DDEBUG("tof_sensor.get_illum_duty_cycle ret= %d", ret);
                }

                pvalue->frequency_mode = OBC_SINGLE_FREQUENCY;
                pvalue->outmode = OB_TOF_OUT_MODE_A_AND_B;
                pvalue->param[0].duty_cycle = duty_cycle >> 8 & 0xff;
                pvalue->param[0].integration_time = integration_time;
                pvalue->param[1].duty_cycle = duty_cycle & 0xFF;
                pvalue->param[1].integration_time = integration_time;

                uint16_t freq = 0;
                ret = tof_sensor.get_modulation_frequency(&freq);
                if (ret == 0) {
                    pvalue->param[0].frequency = (freq >> 8) & 0x0F;
                    pvalue->param[1].frequency = freq & 0x0F;
                }
                break;
            }
            case OBC_SENSOR_ID_MLX75027:
            {
                uint8_t  value8 = 0;
                uint16_t  integration_time = 0;

                ret = tof_sensor.get_integration_time((uint16_t*)&integration_time);
                if (ret < 0) {
                    DDEBUG("tof_sensor.set_integration_time ret= %d", ret);
                }

                uint16_t duty_cycle = 0;
                ret = tof_sensor.get_illum_duty_cycle(&duty_cycle);
                if (ret < 0) {
                    DDEBUG("tof_sensor.get_illum_duty_cycle ret= %d", ret);
                }

                pvalue->frequency_mode = OBC_SINGLE_FREQUENCY;
                pvalue->outmode = OB_TOF_OUT_MODE_A_AND_B;
                pvalue->param[0].duty_cycle = duty_cycle >> 8 & 0xff;
                pvalue->param[0].integration_time = integration_time;
                pvalue->param[1].duty_cycle = duty_cycle & 0xFF;
                pvalue->param[1].integration_time = integration_time;

                uint16_t freq = 0;
                ret = tof_sensor.get_modulation_frequency(&freq);
                if (ret == 0) {
                    pvalue->param[0].frequency = (freq >> 8) & 0x0F;
                    pvalue->param[1].frequency = freq & 0x0F;
                }
                break;
            }
            default:
                ret = -HW_ERR_NO_SUPPORT;
            }
        }
        break;
    }
    case OPS_PTR: {
        void **pvalue = (void**)pdata->data;
        *pvalue = &tops_t;
        ret = 0;
        break;
    }
    case DRIVER_IC_DETECT: {
        if (NULL != tof_sensor.driver_ic_detect) {
            uint32_t *pvalue = (uint32_t*)pdata->data;
            uint16_t ic_type = 0;
            ret = tof_sensor.driver_ic_detect(&ic_type);
            if (ret >= 0) {
                *pvalue = ic_type;
            }
            //ALOGE("----->driver_ic_detect ret=%d\n",  ret);
        }
        else {
            ret = -HW_ERR_NO_SUPPORT;
            ALOGE("----->driver_ic_detect HW_ERR_NO_SUPPORT");
        }
        break;
    }

    case DELAY_CIRCUIT_TIME: {
        uint32_t *pvalue = (uint32_t*)pdata->data;
        if (NULL == pvalue) {
            return  -HW_ERR_NULL;
        }
        uint16_t value = 0;
        ret = nb6l295_read(&value);
        *pvalue = value;
        //DDEBUG("DELAY_CIRCUIT_TIME  GET %d", *pvalue);
        break;
    }
    case DELAY_CIRCUIT_ID: {
        uint32_t *pvalue = (uint32_t*)pdata->data;
        if (NULL == pvalue) {
            return  -HW_ERR_NULL;
        }
        uint16_t value = 0;
        ret = nb6l295_get_id(&value);
        *pvalue = value;
        //DDEBUG("DELAY_CIRCUIT_ID  GET %d", *pvalue);
        break;
    }
    case TX_A_B_POWER:
        if (NULL != tof_sensor.get_tx_a_b_power) {
            tx_a_b_power_t *pvalue = (tx_a_b_power_t*)pdata->data;
            if (NULL == pvalue) {
                return  -HW_ERR_NULL;
            }
            uint8_t a = 0;
            uint8_t b = 0;
            ret = tof_sensor.get_tx_a_b_power(&a, &b);
            if (ret >= 0) {
                pvalue->a_state = a;
                pvalue->b_state = b;
            }
            ALOGE("-->get_tx_a_b_power %d,%d  ret=%d\n", pvalue->a_state, pvalue->b_state, ret);
        }
        else {
            ret = -HW_ERR_NO_SUPPORT;
            ALOGE("----->get_tx_a_b_power HW_ERR_NO_SUPPORT");
        }
        break;
    case  SENSOR_FREQUENCY_DUTY_CONFIG:
        if (NULL != tof_sensor.get_frequency_and_duty) {
            ObcSensorFreqDutyConfig *pvalue = (ObcSensorFreqDutyConfig*)pdata->data;
            if (NULL == pvalue) {
                return  -HW_ERR_NULL;
            }
            //ALOGD("get_frequency_and_duty %d,%d", pvalue->index, pvalue->freq_duty.mod_freq1);
            ret = tof_sensor.get_frequency_and_duty(&pvalue->index, &pvalue->index_max, &pvalue->freq_duty);
            ALOGD("--->get_frequency_and_duty %d,%d", pvalue->index, pvalue->freq_duty.mod_freq1);
        }
        else {
            ALOGD("do  not support SENSOR_FREQUENCY_DUTY_CONFIG ");
            ret = -HW_ERR_NO_SUPPORT;
        }
        break;
    case SHUFFLE:
        if (NULL != tof_sensor.get_shuffle_mode) {
            bool *pvalue = (bool*)pdata->data;
            if (NULL == pvalue) {
                return  -HW_ERR_NULL;
            }
            ret = tof_sensor.get_shuffle_mode(pvalue);
            //ALOGE("get_shuffle_mode %d,ret= %d", *pvalue,ret);
        }
        else {
            ret = -HW_ERR_NO_SUPPORT;
        }
        break;
    case CHIP_ID:
        if (NULL != tof_sensor.get_chip_id) {
            uint8_t *pvalue = (uint8_t*)pdata->data;
            if (NULL == pvalue) {
                return  -HW_ERR_NULL;
            }
            ret = tof_sensor.get_chip_id(pvalue);
            //ALOGE("get_shuffle_mode %d,ret= %d", *pvalue,ret);
        }
        else {
            ret = -HW_ERR_NO_SUPPORT;
        }
        break;
    case BINNING_MODE:
		if (NULL != tof_sensor.get_pixel_binning) {
            uint8_t *pvalue = (uint8_t*)pdata->data;
            if (NULL == pvalue) {
                return  -HW_ERR_NULL;
            }
			ret = tof_sensor.get_pixel_binning(pvalue);
            ALOGD("get_pixel_binning %d,ret= %d", *pvalue,ret);
        }
        else {
            ret = -HW_ERR_NO_SUPPORT;
        }
        break;
    case BURST_MODE:
        if (NULL != tof_sensor.get_burst_mode) {
            uint8_t *pvalue = (uint8_t*)pdata->data;
            if (NULL == pvalue) {
                return  -HW_ERR_NULL;
            }
            ret = tof_sensor.get_burst_mode(pvalue);
            //ALOGD("get_shuffle_mode %d,ret= %d", *pvalue,ret);
        }
        else {
            ret = -HW_ERR_NO_SUPPORT;
        }
        break;
    case ITO_EM:
        if (NULL != tof_sensor.get_vcsel_error) {
            uint16_t vcsel_error = 0;
            uint16_t *pvalue = (uint16_t*)pdata->data;
            if (NULL == pvalue) {
                return  -HW_ERR_NULL;
            }
            ret = tof_sensor.get_vcsel_error(&vcsel_error);
            *pvalue = (vcsel_error & 0x02) >> 1;
            //ALOGD("---->get_vcsel_error %d\n", *pvalue);
        }
        else {
            ret = -HW_ERR_NO_SUPPORT;
        }
        break;
    case DRIVER_IC_REG8_RW:
    {
        reg16_map_t* reg = (reg16_map_t*)(pdata->data);
        if (nullptr == tof_sensor.driver_ic_read_reg_8) {
            return -HW_ERR_NO_SUPPORT;
        }
        uint8_t value = 0;
        ret = tof_sensor.driver_ic_read_reg_8(reg->addr, &value);
        // ALOGD("---->driver_ic_read_reg_8 %d,%d,ret=%d\n", reg->addr, value,ret);
        reg->value = value;
        break;
    }
    case FREQUENCY_MODE:
        if (nullptr != tof_sensor.get_frequency_mode)
        {
            uint8_t *pvalue = (uint8_t*)pdata->data;
            ret = tof_sensor.get_frequency_mode(pvalue);
            ALOGD("tof_sensor.get frequency_mode:   %d", *pvalue);
        }
        else
        {
            ret = -HW_ERR_NO_SUPPORT;
        }
        break;
    case VMGHI_VOLTAGE:
        if (nullptr != tof_sensor.get_vmghi_voltage)
        {
            uint8_t *pvalue = (uint8_t*)pdata->data;
            ret = tof_sensor.get_vmghi_voltage(pvalue);
            //ALOGD("tof_sensor.get_vmghi_voltage:   %d", *pvalue);
        }
        else
        {
            ret = -HW_ERR_NO_SUPPORT;
        }
        break;
    case AF_DEPTH:
        if (nullptr != tof_sensor.get_af_depth)
        {
            uint16_t *pvalue = (uint16_t*)pdata->data;
            ret = tof_sensor.get_af_depth(pvalue);
            //ALOGD("tof_sensor.get_af_depth:   %d", *pvalue);
        }
        else
        {
            ret = -HW_ERR_NO_SUPPORT;
        }
        break;
    case LASER_DRIVER_VOLTAGE:
    {
        if (nullptr != tof_sensor.get_ld_vcc)
        {
            uint16_t *pvalue = (uint16_t*)pdata->data;
            ret = tof_sensor.get_ld_vcc(pvalue);
            ALOGD("tof_sensor.get_ld_vcc:   %d", *pvalue);
        }
        else
        {
            ret = -HW_ERR_NO_SUPPORT;
        }
        break;
    }
    case DELAY_BOARD_POWER:
    {
        uint8_t *pvalue = (uint8_t*)pdata->data;
        ret = delay_board_get_power_on_off(pvalue);
        break;
    }
    case DELAY_BOARD_FW_VERSION:
    {
        uint8_t *pvalue = (uint8_t*)pdata->data;
        ret = delay_board_get_firmware_version(pvalue);
        ALOGD("DELAY_BOARD_FW_VERSION %d", ret);
        break;
    }
    case DELAY_BOARD_DELAY_SWITCH:
    {
        uint8_t *pvalue = (uint8_t*)pdata->data;
        ret = delay_board_get_delay_selcet(pvalue);
        ALOGD("DELAY_BOARD_DELAY_SWITCH %d", ret);
        break;
    }
    case DELAY_BOARD_DELAY_TIME:
    {
        uint32_t *pvalue = (uint32_t*)pdata->data;
        ret = delay_board_get_delay_time(pvalue);
        ALOGD("DELAY_BOARD_DELAY_TIME %d", ret);
        break;
    }
    case DELAY_BOARD_TEMPERATURE:
    {
        ObcDelayBoardTemperature *pvalue = (ObcDelayBoardTemperature*)pdata->data;
        if (nullptr == pvalue->adc_tmpr || nullptr == pvalue->centigrade_tmpr)
        {
            return -HW_ERR_NULL;
        }
        ret = delay_board_get_all_temperature(pvalue->adc_tmpr, pvalue->centigrade_tmpr);
        ALOGD("DELAY_BOARD_DELAY_TIME %d", ret);
        break;
    }
    case OBC_TOF_PROJECT_TEST:
    {
        if (nullptr == tof_sensor.get_aa_qc_test)
        {
            ALOGD("OBC_TOF_PROJECT_TEST get_aa_qc_test nullptr");
            return -HW_ERR_NULL;
        }
        uint16_t *pvalue = (uint16_t*)pdata->data;
        ret = tof_sensor.get_aa_qc_test(pvalue);
        ALOGD("OBC_TOF_PROJECT_TEST index = %d, ret = %d", *pvalue, ret);
        break;
    }
    default:
        ret = -HW_ERR_NO_SUPPORT;
        break;
    }
    
    return ret;
}

#ifdef __cplusplus
}
#endif
