#ifndef __LIBTOF_TOF_SENSORS_H__
#define __LIBTOF_TOF_SENSORS_H__
#include <stdint.h>
//#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include"include\tofinfo.h"
#include"mx6xhal\obc_tee_funcs.h"
#ifdef __cplusplus
extern "C" {
#endif

#define OBTOF_DRIVER_VERSION_MAJOR 0
#define OBTOF_DRIVER_VERSION_MINOR 3           // interface changed plus 1
#define OBTOF_DRIVER_VERSION_PATCHLEVEL 14     //
#define OBTOF_DRIVER_VERSION_RELEASER 3        // release 1,othr test plus 1

extern obc_ops_t tops_t;

typedef int bool_;//与rk1608保持一致

typedef enum img_rotation_t{
    IMAGE_NORMAL = 0,
    IMAGE_H_MIRROR,
    IMAGE_V_MIRROR,
    IMAGE_HV_MIRROR
}img_rotation;

typedef enum output_mode_t{
    PIXEL_A_M_B = 0,  //A minus B
    PIXEL_A_P_B,  //A plus B
    PIXEL_A,
    PIXEL_B,
    PIXEL_AB,
    PIXEL_2TAP,
    PIXEL_4TAP,
    PIXEL_SHUFFLE_TAP,
}output_mode;

typedef enum test_pattern_t {
    IMAGE_NO_PATTERN = 0,
    IMAGE_GRADIENT_PATTERN,
    IMAGE_SOLID_PATTERN,
}test_pattern;

typedef struct {
    uint16_t SensorId;
    int(*sensor_init)();
} sensor_list_t;



/*TODO (qibing)
typedef struct sensor_info_t {//
    uint32_t embedded_data_size;
    uint8_t  vcsel_num;
    uint16_t vcsel_driver_id;
    uint16_t sensor_id;
    uint16_t project_id;
} sensor_info;

typedef struct ObcIllumPower {//
    uint8_t vcsel;          // select which vcsel to write
    uint32_t value_A;       // current A
    uint32_t value_B;        // current B
}ObcIllumPower;

// OBC_TX_A_B_POWER
typedef struct _tx_a_b_power_t {//
    uint16_t a_state;   //A区状态
    uint16_t b_state;
} tx_a_b_power_t;
*/



typedef struct sensor_default_param_t {
    uint8_t mod_freq1;
    uint8_t mod_freq2;
    uint8_t freq1_duty_index;
    uint8_t freq2_duty_index;
    ObcIllumPower _illum_power;
    uint16_t integration_time;
} sensor_default_param;

//FILTER_PARAM
typedef struct _MEDIAN_FILTER_PARAM
{
    // window size
    int win_size;
    // skip or not
    bool_ bypass;
}MedianFilterParam;

typedef struct _RAWPHASE_FILTER_PARAM
{
    // windows size
    int win_size;
    // skip or not
    bool_ bypass;
}RawphaseFilterParam;

typedef struct _TEMPORAL_FILTER_PARAM
{
    // weight
    float weight;
    // refresh maxdiff
    float maxdiff;
    // skip or not
    bool_ bypass;
}TemporalFilterParam;

typedef struct _NOISE_FILTER_PARAM
{
    // window size
    int win_size;
    // skip or not
    bool_ bypass;
}NoiseFilterParam;

typedef struct _FASTGUIDED_FILTER_PARAM
{
    // filter distance
    int win_size;
    // color sigma
    float eps;
    // space sigma
    float scale;
    // skip or not
    bool_ bypass;
}FastGuidedFilterParam;

typedef struct _GAUSSIAN_FILTER_PARAM
{
    // filter size
    int win_x;
    int win_y;
    // x sigma
    float sigma_x;
    // y sigma
    float sigma_y;
    // skip or not
    bool_ bypass;
}GaussianFilterParam;

typedef struct _FLYPOINT_FILTER_PARAM
{
    //thres for fly point filtering
    float thres;
    //noise coeff;
    float noise_coeff;
    //fill the hole after fly point filtering
    bool_ fill_hole;
    //	skip or not
    bool_ bypass;
}FlyPointFilterParam;

typedef struct _SCATTER_FILTER_PARAM
{
    //outdoor th
    float outdoor_th;
    //indoor distance th
    int indoor_disth;
    //outdoor distance th
    int outdoor_disth;
    // skip or not
    bool_ bypass;
}ScatterFilterParam;

typedef struct _CONFIDENCE_FILTER_PARAM
{
    float confidence_indoor_amp_thres;
    float confidence_outdoor_amp_thres;
    float sbr_thres;
    float intensity_thres;
    float usm_thres;
    float reflectivity_thres;
    int usm_win_size;
    bool_ usm_bypass;
    bool_ bypass;
}ConfidenceFilterParam;

typedef struct _POST_FILTER_PARAM
{
    //outdoor th
    float outdoor_th;

    //max peckle Size
    float maxSpeckleSize;

    //max difference
    float maxdiff;

    //outdoor long distance threshold
    float outdoor_long_dis;

    // window size
    int win_size;
    // skip or not
    bool_ bypass;
}PostFilterParam;

typedef struct _DEPTH_CONFIGURATION_PARAM
{
    float depth_uint;
    int max_depth;
    int min_depth;
}DepthConfigParam;

typedef struct _POINT_CLOUD_FILTER_PARAM
{
    float statistical_max_slope;
    float statistical_sigma;

    float cluster_ratio;
    /// 1 - statistical; 8 - cluster filter;
    int filter_type;
    bool_ flypoint_compens;

    bool_ bypass;

}PointCloudFilterParam;

typedef struct _OB_TOF_CONFIG
{
    RawphaseFilterParam rawphase_filter_cfg;
    TemporalFilterParam temporal_filter_cfg;
    NoiseFilterParam noise_filter_cfg;
    MedianFilterParam median_filter_cfg;
    FastGuidedFilterParam fastgiuded_filter_cfg;
    GaussianFilterParam gaussian_filter_cfg;
    FlyPointFilterParam flypoint_filter_cfg;
    ScatterFilterParam scatter_filter_cfg;
    PostFilterParam post_filter_cfg;
    ConfidenceFilterParam confidence_filter_cfg;
    DepthConfigParam depth_cfg;
    PointCloudFilterParam point_cloud_filter_cfg;
}ObToFConfig;

typedef struct _AE_PARAM
{
    float fov;
    int t_min;
    int t_max;
    float t_step_ratio_min;
    float t_step_ratio_max;
    int over_exposure_value;
    int over_dark_value;
    float ratio_thresh;
    float ir_thresh;
}AEParam;

typedef struct duty_cycles_list_t {
    uint32_t frequency;
    float  duty_cycle_steps[OB_DUTY_CYCLE_LIST_NUM];
}duty_cycle_list;

/* 定义SENSOR需要的电源类型 */
///定义SENSOR需要的电源类型。
typedef enum {
    /* A通道，或只有一个通道时 */
    POWER_AVDD = 0,            ///<AVDD
    POWER_DOVDD = 1,        ///<DOVDD
    POWER_DVDD = 2,            ///<DVDD
    POWER_AFVCC = 3,        ///<AFVCC
    POWER_VPP = 4,            ///<VPP

    /* B通道,(B通道电源定义，只有UH920使用) */
    POWER_AVDD_B = 5,        ///<B通道AVDD
    POWER_DOVDD_B = 6,        ///<B通道DOVDD
    POWER_DVDD_B = 7,        ///<B通道DVDD
    POWER_AFVCC_B = 8,        ///<B通道AFVCC
    POWER_VPP_B = 9,        ///<B通道VPP

    /* 新增加的电源通道定义 */
    POWER_OISVDD = 10,
    POWER_AVDD2 = 11,
    POWER_AUX1 = 12,
    POWER_AUX2 = 13,
    POWER_VPP2 = 14
}SENSOR_POWER;

#define  kConverterIsCx3    2
#define  kConverterIsDuxin  1


#if  (USE_WHICH_CONVERTER==kConverterIsCx3)
#pragma pack (push, 1)
typedef struct _MipiConfiguration
{
    uint8_t data_format;
    uint8_t num_datalanes;
    uint8_t pll_prd;
    uint16_t pll_fbd;
    uint8_t pll_frs;
    uint8_t csi_rx_clk_div;
    uint8_t par_clk_div;
    uint16_t mclk_ctl;
    uint8_t mclk_ref_div;
    uint16_t hresolution;
    uint16_t fifo_delay;
    uint16_t pll_clock;
    uint8_t mclk;
} MipiConfiguration, *PMipiConfiguration;
#pragma pack (pop) // Undo the pack change
#endif


typedef struct tof_sensor_func
{
    int(*init)();
    int(*get_chip_id)(uint8_t *id);
    int(*get_sensor_id)(uint16_t *id);
    int(*hardware_trigger)();
    int(*software_trigger)();
    int(*video_streaming)(bool enable);
	int(*deepsleep_mode)(bool enable);
	int(*hdr_algorithm)(bool enable);
	int(*hist_algorithm)(bool enable);
	int(*median_algorithm)(bool enable);
	int(*ebc_algorithm)(bool enable);
	int(*lsc_algorithm)(bool enable);
	int(*correction_algorithm)(bool enable);
    int(*get_fps)(uint8_t *fps);
    int(*set_fps)(uint8_t fps);
    int(*get_sensor_temperature)(float *temp);
    int(*get_rx_temp)(float *temperature);
    int(*get_tx_temp)(float *temperature);
    int(*set_illum_power)(uint8_t vcsel_num, uint8_t value_A, uint8_t value_B);
    int(*get_illum_power)(uint8_t *vcsel_num, uint8_t *value_A, uint8_t *value_B);
    int(*illum_power_control)(bool enable); 
    int(*get_integration_time)(uint16_t *integrationTime);
    int(*set_integration_time)(uint16_t integrationTime);
    int(*get_modulation_frequency)(uint16_t *modFreq);
    int(*set_modulation_frequency)(uint16_t modFreq);
    int(*get_illum_duty_cycle)(uint16_t *duty);
    int(*set_illum_duty_cycle)(uint16_t duty);
    int(*get_data_output_mode)(uint8_t *mode);
    int(*set_data_output_mode)(uint8_t mode);
	int(*set_window_originy)(uint32_t originy);
	int(*set_window_originx)(uint32_t originx);
	int(*set_window_height)(uint32_t height);
	int(*get_window_height)(uint32_t *height);
	int(*set_window_width)(uint32_t width);
	int(*get_window_width)(uint32_t *width);
    int(*get_img_mirror_flip)(uint8_t *mode);
    int(*set_img_mirror_flip)(uint8_t mode);
    int(*get_pixel_binning)(uint8_t *mode);
    int(*set_pixel_binning)(uint8_t mode);
    int(*test_pattern)(uint8_t mode);
    int(*get_sensor_info)(struct sensor_info_t *info);
    int(*get_illum_duty_cycle_list)(uint8_t mod_freq, float *duty_cycle_list);
    int(*get_vcsel_pd)(uint32_t *value);
    int(*get_vcsel_error)(uint16_t *value);
    int(*eeprom_write)(uint16_t offset, uint8_t* buf, uint16_t size);
    int(*eeprom_read)(uint16_t offset, uint8_t * buf, uint16_t size);
    int(*illum_power_test)(uint8_t mode);
    int(*get_sensor_default_param)(struct sensor_default_param_t *param);
    int(*AE)(bool enable);
    int(*driver_ic_detect)(uint16_t * driver_ic_type);
    int(*sensor_write_reg_16)(uint16_t reg, uint16_t value); // s5k33d used
    int(*sensor_read_reg_16)(uint16_t reg, uint16_t *value);
    int(*sensor_write_reg_8)(uint16_t reg, uint8_t value); // imx518 and pleco used
    int(*sensor_read_reg_8)(uint16_t reg, uint8_t *value);
    int(*driver_ic_write_reg_8)(uint8_t reg, uint8_t value);
    int(*driver_ic_read_reg_8)(uint8_t reg, uint8_t *value);
    int(*set_tx_a_b_power)(uint8_t a_enable, uint8_t b_enable);
    int(*get_tx_a_b_power)(uint8_t *A_status_enable, uint8_t *B_status_enable);
    int(*set_shuffle_mode)(bool enable);
    int(*get_shuffle_mode)(bool *enable);
    int(*set_binning_mode)(uint8_t mode);
    int(*get_binning_mode)(uint8_t *mode);
    int(*set_burst_mode)(uint8_t mode);
    int(*get_burst_mode)(uint8_t *mode);
    int(*get_frequency_and_duty)(int *index, int *index_max, struct sensor_freq_duty_t *freq_duty);
    int(*set_frequency_and_duty)(int index);
    int(*get_frequency_mode)(uint8_t *mode);
    int(*set_frequency_mode)(uint8_t mode);
    int(*get_master_slave_mode)(uint8_t *mode);
    int(*set_master_slave_mode)(uint8_t mode);
    int(*get_antijam)(uint8_t *enable);
    int(*set_antijam)(uint8_t enable);
    int(*get_vmghi_voltage)(uint8_t *value);
    int(*set_vmghi_voltage)(uint8_t value);
    int(*load_ref_buffer)(uint32_t rk1608_store_addr, uint32_t *ref_data_addr, uint32_t ref_data_len);
    int(*get_ref_buffer_addr)(uint32_t *ref_buffer_addr);
    int(*set_streaming_type)(int mode);
    int(*get_af_depth)(uint16_t *value);
    int(*set_tof_config)(int8_t cam_id, ObToFConfig *param);
    int(*get_tof_sensor_resolution)(uint64_t *resolution);
    int(*get_tof_sensor_pixel_bit)(uint8_t * pixel_bit);
    int(*get_mipi_pack_bit)(uint8_t * mipi_pack_bit);
    int(*get_ld_vcc)(uint16_t *value_mv_p);
    int(*set_ld_vcc)(uint16_t value_mv);
	int(*set_aa_qc_test)(uint16_t index);
	int(*get_aa_qc_test)(uint16_t *index_p);

    int(*get_exp)(uint32_t *exp);
    int(*set_exp)(uint32_t exp);
    int(*get_gain)(uint32_t *gain);
    int(*set_gain)(uint32_t gain);
	int(*set_odd_dgain)(uint32_t gain);
	int(*set_even_dgain)(uint32_t gain);
	int(*set_sub_samp)(uint8_t value);
	int(*get_sub_samp)(uint8_t *value);
	int(*set_sub_sampv)(uint8_t value);
	int(*get_sub_sampv)(uint8_t *value);
    int(*get_ldm_en)(uint32_t *en);
    int(*set_ldm_en)(uint32_t en);
    int(*get_led_en)(uint32_t *en);
    int(*set_led_en)(uint32_t en);
    int(*get_ldm_current)(uint32_t *current);
    int(*set_ldm_current)(uint32_t current);
    int(*get_led_current)(uint32_t *current);
    int(*set_led_current)(uint32_t current);
    int(*set_chip_reset)(uint32_t reset_en);

}tof_sensor_func_t;

extern tof_sensor_func_t tof_sensor;

int tof_sensor_lib_init(void * ops);
int tof_sensor_init();
int tof_get_lib_version(uint32_t * pversion);

#ifdef __cplusplus
}
#endif

#endif  // __LIBTOF_TOF_SENSORS_H__
