/*
 * Copyright (C) Fuzhou Rockchip Electronics Co.Ltd
 * Author: Tusson <dusong@rock-chips.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef __RK_PREISP_MSG_H__
#define __RK_PREISP_MSG_H__

#include "spi2apb.h"
#include "tof_sensors.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DSP_R_MSG_QUEUE_ADDR 0x60050000
#define DSP_S_MSG_QUEUE_ADDR 0x60050010

#define MSG_QUEUE_DEFAULT_SIZE 8*1024

enum sensor_stream_frame_type {
    SINGLE_FREQ_NONSHUFFLE,
    DUAL_FREQ_NONSHUFFLE,
    SINGLE_FREQ_SHUFFLE,
    DUAL_FREQ_SHUFFLE
};

enum sensor_stream_frame_mode {
    MASTER,
    SLAVE
};

enum depth_d2c_ctrl {
    D2C_DISABLE,
    D2C_ENABLE
};

enum preisp_stream_type {
    CAM_PHASE_STREAM,
    CAM_IR_STREAM,
    CAM_DEPTH_STREAM,
    CAM_PHASE_AND_DEPTH_STREAM,
    CAM_IR_AND_DEPTH_STREAM
};

typedef struct msg_queue {
    uint32_t* buf_head; //msg buffer head
    uint32_t* buf_tail; //msg buffer tail
    uint32_t* cur_send; //current msg send postition
    uint32_t* cur_recv; //current msg receive position
} msg_queue_t;

#define DSP_PMU_SYS_REG0            0x120000f0
#define DSP_MSG_QUEUE_OK_MASK       0xffff0001
#define DSP_MSG_QUEUE_OK_TAG        0x16080001

typedef struct dsp_msg_queue {
    uint32_t buf_head; //msg buffer head
    uint32_t buf_tail; //msg buffer tail
    uint32_t cur_send; //current msg send postition
    uint32_t cur_recv; //current msg receive position
} dsp_msg_queue_t;

typedef struct msg {
	uint32_t size; /* unit 4 bytes */
	uint16_t type;
	union {
		uint8_t  camera_id;
		uint8_t  core_id;
	}id;
	union {
		uint8_t  sync;
		uint8_t  log_level;
		int8_t	 err;
	}mux;
} msg_t;

typedef struct msg_ext {
    uint32_t size; /* unit 4 bytes */
    uint16_t type;
    union {
        uint8_t  camera_id;
        uint8_t  core_id;
    }id;
    union {
        uint8_t  sync;
        uint8_t  log_level;
        int8_t	 err;
        int8_t	file_id;
    }mux;
    union {
        uint32_t data;
        uint32_t file_buf;
    }ext;
} msg_ext_t;

typedef struct msg_init_sensor {
	uint32_t size; /* unit 4 bytes */
	uint16_t type;
	uint8_t  camera_id;
	uint8_t  sync;

	uint32_t i2c_bus;
	uint32_t i2c_clk;
	int8_t in_mipi_phy;
	int8_t out_mipi_phy;
	int8_t mipi_lane;
	int8_t bayer;
	uint8_t sensor_name[32];
	uint8_t i2c_slave_addr;
} msg_init_sensor_t;

struct preisp_vc_cfg {
	int8_t   data_type;
	int8_t   decode_format;
	int8_t   flag;
	int8_t   unused;
	uint16_t width;
	uint16_t height;
};

typedef struct msg_set_pleco_input_size {
	struct msg msg_head;
	struct preisp_vc_cfg channel_embed;	
	struct preisp_vc_cfg channel_raw;
} msg_set_pleco_input_size_t;

typedef struct msg_set_input_size {
	struct msg msg_head;
	struct preisp_vc_cfg channel;
} msg_set_input_size_t;

typedef struct msg_out_size_head {
	uint32_t size; /* unit 4 bytes */
	uint16_t type;
	uint8_t  camera_id;
	uint8_t  sync;

	uint16_t width;
	uint16_t height;
	uint32_t mipi_clk;
	uint16_t line_length_pclk;
	uint16_t frame_length_lines;
	uint16_t mipi_lane;
	uint16_t reserved;
} msg_out_size_head_t;

typedef struct msg_set_output_size {
	struct msg_out_size_head msg_head;
	struct preisp_vc_cfg channel;

} msg_set_output_size_t;

enum {
	/* AP -> DSP
	1 msg of sensor */
	id_msg_init_sensor_t = 0x0001,
	id_msg_set_input_size_t,
	id_msg_set_output_size_t,
	id_msg_set_stream_in_on_t,
	id_msg_set_stream_in_off_t,
	id_msg_set_stream_out_on_t,
	id_msg_set_stream_out_off_t,

	/* 0x0008 ~ 0x0010 id reserved. */
	id_msg_set_algo_nightshot_t = 0x0011,
	id_msg_set_algo_antishaking_t,
	id_msg_set_algo_hdr_t,
	id_msg_set_algo_null_t,
    /* add by orbbec */
    id_msg_set_algo_in_stream_mode,
    id_msg_set_algo_out_stream_type,
    id_msg_set_d2c_algo_enable,
    id_msg_set_algo_param_init,
    id_msg_get_system_time,
    id_msg_set_calc_mirror_enable,
    id_msg_set_ae_param,
	/* AP -> DSP
	4 msg of power manager */
    id_msg_set_sys_mode_bypass_t = 0x0200,
    id_msg_set_sys_mode_standby_t,

    id_msg_set_log_level_t = 0x0250,

    //dsp -> ap
    id_msg_do_i2c_t = 0x0390,
    //ap -> dsp
    id_msg_do_i2c_ret_t,

    id_msg_dsp_log_t = 0x0400,
};

typedef msg_t msg_set_sys_mode_standby_t;

enum {
    LOG_ERROR,
    LOG_WARN,
    LOG_INFO,
    LOG_DEBUG,
};

typedef struct {
    uint32_t size;
    uint16_t type;
    int8_t  core_id;
    int8_t  log_level;
} msg_dsp_log_t;

typedef msg_dsp_log_t msg_set_log_level_t;

//dsp -> ap
typedef struct {
    uint16_t addr;
    uint16_t flags;
    uint16_t len;
    uint8_t  buf[6];
} do_i2c_msg_t;

typedef struct {
    uint32_t size;
    uint16_t type;
    int16_t  nr;
    uint16_t scl_rate;
    uint16_t num_msg;
} msg_do_i2c_head_t;

#define AP_I2C_ONCE_MAX_NUM 20
typedef struct {
    msg_do_i2c_head_t head;
    do_i2c_msg_t msg[AP_I2C_ONCE_MAX_NUM];
} msg_do_i2c_t;

typedef struct msg_set_input_mode {
    struct msg msg_head;
    enum sensor_stream_frame_type frame_type;
    enum sensor_stream_frame_mode frame_mode;
    uint8_t sync_freq;
    uint8_t skip_frame;
} msg_set_input_mode_t;

typedef struct msg_set_out_stream_type {
    struct msg msg_head;
    enum preisp_stream_type stream_type;
} msg_set_out_stream_type_t;

typedef struct msg_set_tof_config {
    struct msg msg_head;
    ObToFConfig tof_config_params;
} msg_set_tof_config_t;

typedef struct msg_set_ae_param {
    msg_t msg_head;
    float fov;
    int t_min;
    int t_max;
    float t_step_ratio_min;
    float t_step_ratio_max;
    int over_exposure_value;
    int over_dark_value;
    float ratio_thresh;
    float ir_thresh;
} msg_set_ae_param_t;

//ap -> dsp
typedef struct {
    uint32_t size;
    uint16_t type;
    int16_t  nr;
    uint16_t addr;
    uint16_t len;
    uint8_t  buf[8];
} msg_do_i2c_ret_t;


#define MSG(TYPE, var) TYPE var; \
    var.size = sizeof(TYPE)/4;\
    var.type = id_ ## TYPE;

#define PREISP_IRQ_TYPE_MSG 0x12345678

/**
 * msq_init - Initialize msg queue
 *
 * @q: the msg queue to initialize
 * @size: size of msg queue buf
 *
 * It returns zero on success, else a negative error code.
 */
int msq_init(struct msg_queue *q, int size);

/**
 * msq_release - release msg queue buf
 *
 * @q: the msg queue to release
 */
void msq_release(struct msg_queue *q);

/**
 * msq_is_empty - tests whether a msg queue is empty
 *
 * @q: the msg queue to test
 *
 * It returns true on msg queue is empty, else false.
 */
int msq_is_empty(const struct msg_queue *q);

/**
 * msq_send_msg - send a msg to msg queue
 *
 * @q: msg queue
 * @m: a msg to queue
 *
 * It returns zero on success, else a negative error code.
 */
int msq_send_msg(struct msg_queue *q, const struct msg *m);

/**
 * msq_recv_msg - receive a msg from msg queue
 *
 * @q: msg queue
 * @m: a msg pointer buf [out]
 *
 * need call msq_free_received_msg to free msg after msg use done
 *
 * It returns zero on success, else a negative error code.
 */
int msq_recv_msg(struct msg_queue *q, struct msg **m);

/**
 * msq_free_received_msg - free a received msg
 *
 * @q: msg queue
 * @m: a msg
 *
 * It returns zero on success, else a negative error code.
 */
int msq_free_received_msg(struct msg_queue *q, const struct msg *m);

/**
 * dsp_msq_init - init AP <-> DSP msg queue
 *
 * @spi: spi device
 *
 * It returns zero on success, else a negative error code.
 */
int dsp_msq_init(struct spi_device *spi);

/**
 * dsp_msq_read_head - read dsp msg queue head
 *
 * @spi: spi device
 * @addr: msg queue head addr
 * @m: msg queue pointer
 *
 * It returns zero on success, else a negative error code.
 */
int dsp_msq_read_head(uint32_t addr, struct dsp_msg_queue *q);

/**
 * dsp_msq_send_msg - send a msg to AP -> DSP msg queue
 *
 * @spi: spi device
 * @m: a msg to send
 *
 * It returns zero on success, else a negative error code.
 */
int dsp_msq_send_msg(const struct msg *m);

/**
 * dsp_msq_recv_query - query next msg size from DSP -> AP msg queue
 *
 * @q: msg queue
 * @size: msg size buf [out]
 *
 * It returns zero on success, else a negative error code.
 */
int dsp_msq_recv_query(int32_t *size);

/**
 * dsp_msq_recv_msg - receive a msg from DSP -> AP msg queue
 *
 * @q: msg queue
 * @m: a msg pointer buf [out]
 *
 * need call dsp_msq_free_received_msg to free msg after msg use done
 *
 * It returns zero on success, else a negative error code.
 */
int dsp_msq_recv_msg(struct msg **m);

/**
 * dsp_msq_free_received_msg - free a received msg
 *
 * @q: msg queue
 * @m: a msg
 *
 * It returns zero on success, else a negative error code.
 */
int dsp_msq_free_received_msg(const struct msg *m);

int cmd_msg_init_sensor(int8_t cam_id, int8_t in_mipi, int8_t out_mipi, char* sensor_name, uint8_t i2c_slave_addr, uint8_t i2c_bus);
int cmd_msg_set_input_size(int8_t cam_id, uint16_t width, uint16_t height);
int cmd_msg_set_output_size(int8_t cam_id, uint16_t width, uint16_t height, uint16_t htotal, uint16_t vtotal, uint32_t mipi_clock);
int cmd_msg_set_stream_in_mode(int8_t cam_id, uint8_t mode, uint8_t type, uint8_t freq, uint8_t skip_frame);
int cmd_msg_set_stream_out_type(int8_t cam_id, uint8_t type);
int cmd_msg_set_param_init(int8_t cam_id, uint32_t init_param);
int cmd_msg_set_stream_in_on(int8_t cam_id);
int cmd_msg_set_stream_in_off(int8_t cam_id);
int cmd_msg_set_stream_out_on(int8_t cam_id);
int cmd_msg_set_stream_out_off(int8_t cam_id);

int cmd_msg_set_pleco_input_size(int8_t cam_id, uint16_t embed_width, uint16_t embed_height, uint16_t pixel_width, uint16_t pixel_height);


#ifdef __cplusplus
}
#endif
#endif
