//
// Copyright 2020 ORBBEC Technology., Inc.
//
// This file belongs to ORBBEC Technology., Inc.
// It is considered a trade secret, and is not to be divulged or used by
// parties who have NOT received written authorization from the owner.
// see https:
//

/***********************************************************************************************
***           C O N F I D E N T I A L  ---  O R B B E C   Technology                        ***
***********************************************************************************************
*
*  @file      :    tofinfo.h
*  @brief     :    定义tof相关信息，定义类型适用于驱动、SDK及应用层
*  @version   :    0.0.0.1
*  @date      :    2020.06.16                                                                 *
*  @update    :    2021.04.14                                                                 *                                  
*                                                                                             *
*---------------------------------------------------------------------------------------------*
* Functions:  																				  *                                   
* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */
#ifndef _INCLUDE_TOFINFO_H_
#define _INCLUDE_TOFINFO_H_

#ifdef __cplusplus
extern "C" {
#endif

    typedef enum  {
        OBC_SENSOR_ID_UNKNOWN = -1,
        OBC_SENSOR_ID_S5K33D = 0x303d,
        OBC_SENSOR_ID_MLX75027 = 0x5027,
        OBC_SENSOR_ID_IMX278 = 0x0278,
        OBC_SENSOR_ID_IMX316 = 0x0316,
        OBC_SENSOR_ID_IMX456 = 0x0456,
        OBC_SENSOR_ID_IMX516 = 0x0516,
        OBC_SENSOR_ID_IMX518 = 0x0518,
        OBC_SENSOR_ID_PLECO = 0x0123,
        OBC_SENSOR_ID_RK1608_S5K33D = 0x313d,
        OBC_SENSOR_ID_RK1608_PLECO = 0x1123,
        OBC_SENSOR_ID_IMX627 = 0x0627,
    } ObcSensorId;
    typedef ObcSensorId driver_sensor_id_t;
    typedef ObcSensorId obc_sensor_id_t;

    typedef enum
    {
        OBC_PROJECT_CONFIG_DEFAULT = 0,
        OBC_PROJECT_CONFIG_AA_TX = 1,
        OBC_PROJECT_CONFIG_QC_TX = 2,
    }ObcProjectConfig;

    typedef enum {
        OB_FRAME_TYPE_PHASE = 0x00,
        OB_FRAME_TYPE_PHASE_INT16 = 1,    // 16bit 的相位帧数据
        OB_FRAME_TYPE_Q_90_270 = 2,
        OB_FRAME_TYPE_I_0_180 = 3,
        OB_FRAME_TYPE_AMP = 4,
        OB_FRAME_TYPE_DEPTH = 5,
        OB_FRAME_TYPE_IR = 6,
        OB_FRAME_TYPE_INVAILD = 7
    }ObcFrameType;

    typedef enum ObcTofOutMode_ {
        OB_TOF_OUT_MODE_A_B = 0,      //!< 单位，A-B
        OB_TOF_OUT_MODE_A_ADD_B = 1,  //!< 单位，A+B
        OB_TOF_OUT_MODE_A = 2,        //!< 单位，A
        OB_TOF_OUT_MODE_B = 3,        //!< 单位，B
        OB_TOF_OUT_MODE_A_AND_B = 4,  //!< 单位，A&B

        OB_TOF_OUT_MODE_S5K_4TAP_DUAL_FREQ = 10,  //!< 单位，4-tap, dual-freq., shuffle mode
        OB_TOF_OUT_MODE_S5K_4TAP_DUAL_FREQ_NO_SHUFFLE_4FRAME = 11,  //!< 单位，4-tap, dual-freq., no-shuffle mode  4帧
        OB_TOF_OUT_MODE_S5K_4TAP_DUAL_FREQ_NO_SHUFFLE = 12,  //!< 单位，4-tap, dual-freq., no-shuffle mode  2帧
        OB_TOF_OUT_MODE_S5K_4TAP_SINGLE_FREQ_SHUFFLE = 14,  //!< 单位，4-tap, dual-freq., shuffle mode   2帧
        OB_TOF_OUT_MODE_S5K_4TAP_SINGLE_FREQ_NO_SHUFFLE = 15,  //!< 单位，4-tap, dual-freq., no-shuffle mode 1帧

        OB_TOF_OUT_MODE_3TAP = 20,  //!< 3 tap, pleco, single_freq: 3 frames, dual_freq: 6 frames
    } ObcTofOutMode;

#define OB_DUTY_CYCLE_LIST_NUM  512
    typedef struct OBDutyCycleSteps_ {
        uint32_t frequency;
        float  duty_cycle_steps[OB_DUTY_CYCLE_LIST_NUM];
    }ObcDutyCycleSteps;
    typedef  ObcDutyCycleSteps OBDutyCycleSteps;

    typedef enum {
        PHASE_MAP_NON_SHUFFLE = 0x10,
        PHASE_MAP_SHUFFLE = 0x1B
    }ObcPhaseMap;

    typedef struct sensor_freq_duty_t {
        uint8_t mod_freq1;
        uint8_t mod_freq2;
        float duty1;
        float duty2;
        uint8_t out_mode;
    }ObcSensorFreqDuty;
    typedef  ObcSensorFreqDuty sensor_freq_duty;

    typedef struct SensorFreqDutyConfig_t
    {
        int   index;    //作为输入时，如果index为负数，则查询当前sensor使用配置，如果是合法值，则查询指定index的配置。
        int   index_max;
        sensor_freq_duty freq_duty;
    }ObcSensorFreqDutyConfig;

    typedef enum
    {
        OBC_SINGLE_FREQUENCY,
        OBC_DUAL_FREQUENCY,
        OBC_AF_FREQUENCY,
    } ObcFrequencyMode;

    typedef struct 
    {
        uint32_t frequency;         // 调制频率
        uint32_t duty_cycle;        // 占空比
        uint32_t integration_time;  // 积分时间
    } ObcFrequencyGroupParam;
    typedef ObcFrequencyGroupParam DriverFrequencyGroupParam;

    typedef struct 
    {
        ObcFrequencyGroupParam  param[2];
        ObcFrequencyMode frequency_mode;
        ObcTofOutMode outmode;
    }DriverTofRequestParam;

    // sesnor信息
    typedef struct sensor_info_t {
        uint32_t embedded_data_size;
        uint8_t  vcsel_num;
        uint16_t vcsel_driver_id;   //@obc_vcsel_driver_ic_type
        uint16_t sensor_id;
        uint16_t project_id;
    } ObcSensorInfo;
    typedef ObcSensorInfo sensor_info;
    
    // OBC_TX_A_B_POWER
    typedef struct _tx_a_b_power_t {
        uint16_t a_state;   //A区状态
        uint16_t b_state;
    } ObcTxABPower;
    typedef ObcTxABPower tx_a_b_power_t;

    typedef struct sensor_common_t {
        ObcSensorInfo sensor_info;
        uint8_t binning_mode;
    } ObcSensorCommonInfo;

    typedef struct
    {
        uint16_t *adc_tmpr; //延时电路板温度 adc数值
        double *centigrade_tmpr; //延时电路板温度 摄氏度
    }ObcDelayBoardTemperature;

	typedef enum {
		OBC_DRIVER_IC_CXA4016 = 4016,  //Taishan DVT1 Tx
		OBC_DRIVER_IC_CXA4026 = 4026,  // Polaris Tx
		OBC_DRIVER_IC_PHX3D_3021_AA = 5016,  //Taishan DVT2 Tx
		OBC_DRIVER_IC_PHX3D_3021_CB = 5017,  //Taishan DVT3 Tx
		OBC_DRIVER_IC_DW9912 = 9912,  //DongWoon
		OBC_DRIVER_IC_CXA4046 = 4046,  //CXA4046
		OBC_DRIVER_IC_PHX3D_3018 = 3018,  //F201201
    } ObcVcselDriverICType;
    typedef ObcVcselDriverICType obc_vcsel_driver_ic_type;



#ifdef __cplusplus
}
#endif

#endif  // _INCLUDE_TOFINFO_H_


