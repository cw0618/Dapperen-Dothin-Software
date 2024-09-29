/*
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include "spi\inc\msg-queue.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "debug2log.h"
/**
 * msq_init - Initialize msg queue
 *
 * @q: the msg queue to initialize
 * @size: size of msg queue buf
 *
 * It returns zero on success, else a negative error code.
 */
int msq_init(struct msg_queue *q, int size)
{
    uint32_t *buf = malloc(size);

    q->buf_head = buf;
    q->buf_tail = buf + size / sizeof(uint32_t);
    q->cur_send = buf;
    q->cur_recv = buf;

    return 0;
}

/**
 * msq_release - release msg queue buf
 *
 * @q: the msg queue to release
 */
void msq_release(struct msg_queue *q)
{
    free(q->buf_head);
    q->buf_head = NULL;
    q->buf_tail = NULL;
    q->cur_send = NULL;
    q->cur_recv = NULL;
}

/**
 * msq_is_empty - tests whether a msg queue is empty
 *
 * @q: the msg queue to test
 *
 * It returns true on msg queue is empty, else false.
 */
int msq_is_empty(const struct msg_queue *q)
{
    return q->cur_send == q->cur_recv;
}

/**
 * msq_total_size - get msg queue buf total size
 *
 * @q: msg queue
 *
 * It returns size of msg queue buf, unit 4 bytes.
 */
uint32_t msq_total_size(const struct msg_queue *q)
{
    return q->buf_tail - q->buf_head;
}

/**
 * msq_tail_free_size - get msg queue tail unused buf size
 *
 * @q: msg queue
 *
 * It returns size of msg queue tail unused buf size, unit 4 bytes
 */
uint32_t msq_tail_free_size(const struct msg_queue *q)
{
    if (q->cur_send >= q->cur_recv) {
        return (q->buf_tail - q->cur_send);
    }

    return q->cur_recv - q->cur_send;
}

/**
 * msq_head_free_size - get msg queue head unused buf size
 *
 * @q: msg queue
 *
 * It returns size of msg queue head unused buf size, unit 4 bytes
 */
uint32_t msq_head_free_size(const struct msg_queue *q)
{
    if (q->cur_send >= q->cur_recv) {
        return (q->cur_recv - q->buf_head);
    }

    return 0;
}

/**
 * msq_send_msg - send a msg to msg queue
 *
 * @q: msg queue
 * @m: a msg to queue
 *
 * It returns zero on success, else a negative error code.
 */
int msq_send_msg(struct msg_queue *q, const struct msg *m)
{
    int ret = 0;
    if (msq_tail_free_size(q) > m->size) {
        uint32_t * next_send;

        memcpy(q->cur_send, m, m->size * sizeof(uint32_t));
        next_send = q->cur_send + m->size;
        if (next_send == q->buf_tail) {
            next_send = q->buf_head;
        }
        q->cur_send = next_send;
    } else if (msq_head_free_size(q) > m->size) {
        *q->cur_send = 0; //set size to 0 for skip to head mark
        memcpy(q->buf_head, m, m->size * sizeof(uint32_t));
        q->cur_send = q->buf_head + m->size;
    } else {
        ret = -1;
    }

    return ret;
}

/**
 * msq_recv_msg - receive a msg from msg queue
 *
 * @q: msg queue
 * @m: a msg pointer buf [out]
 *
 * need call msq_recv_msg_free to free msg after msg use done
 *
 * It returns zero on success, else a negative error code.
 */
int msq_recv_msg(struct msg_queue *q, struct msg **m)
{
    *m = NULL;
    if (msq_is_empty(q))
        return -1;

    //skip to head when size is 0
    if (*q->cur_recv == 0) {
        *m = (struct msg*)q->buf_head;
    } else {
        *m = (struct msg*)q->cur_recv;
    }

    return 0;
}

/**
 * msq_free_received_msg - free a received msg to msg queue
 *
 * @q: msg queue
 * @m: a msg
 *
 * It returns zero on success, else a negative error code.
 */
int msq_free_received_msg(struct msg_queue *q, const struct msg *m)
{
    //skip to head when size is 0
    if (*q->cur_recv == 0) {
        q->cur_recv = q->buf_head + m->size;
    } else {
        uint32_t *next_recv;

        next_recv = q->cur_recv + m->size;
        if (next_recv == q->buf_tail) {
            next_recv = q->buf_head;
        }
        q->cur_recv = next_recv;
    }

    return 0;
}

/**
 * dsp_msq_init - init AP <-> DSP msg queue
 *
 * @spi: spi device
 *
 * It returns zero on success, else a negative error code.
 */
int dsp_msq_init(struct spi_device *spi)
{
    int err = 0;
    struct dsp_msg_queue queue;
    queue.buf_head = (DSP_R_MSG_QUEUE_ADDR + sizeof(queue));
    queue.buf_tail = (DSP_R_MSG_QUEUE_ADDR + sizeof(queue) + 16*1024);
    queue.cur_recv = queue.buf_head;
    queue.cur_send = queue.buf_head;

    err = spi2apb_safe_write(DSP_R_MSG_QUEUE_ADDR,
            (int32_t*)&queue, sizeof(queue));

    queue.buf_head = DSP_S_MSG_QUEUE_ADDR + sizeof(queue);
    queue.buf_tail = DSP_S_MSG_QUEUE_ADDR + sizeof(queue) + 16*1024;
    queue.cur_recv = queue.buf_head;
    queue.cur_send = queue.buf_head;

    err = spi2apb_safe_write(DSP_S_MSG_QUEUE_ADDR,
            (int32_t*)&queue, sizeof(queue));
    return err;
}

/**
 * dsp_msq_is_empty - tests whether a msg queue is empty
 *
 * @q: the msg queue to test
 *
 * It returns true on msg queue is empty, else false.
 */
int dsp_msq_is_empty(const struct dsp_msg_queue *q)
{
    return q->cur_send == q->cur_recv;
}

/**
 * dsp_msq_total_size - get msg queue buf total size
 *
 * @q: msg queue
 *
 * It returns size of msg queue buf, unit byte.
 */
uint32_t dsp_msq_total_size(const struct dsp_msg_queue *q)
{
    return q->buf_tail - q->buf_head;
}

/**
 * dsp_msq_tail_free_size - get msg queue tail unused buf size
 *
 * @q: msg queue
 *
 * It returns size of msg queue tail unused buf size, unit byte
 */
uint32_t dsp_msq_tail_free_size(const struct dsp_msg_queue *q)
{
    if (q->cur_send >= q->cur_recv) {
        return (q->buf_tail - q->cur_send);
    }

    return q->cur_recv - q->cur_send;
}

/**
 * dsp_msq_head_free_size - get msg queue head unused buf size
 *
 * @q: msg queue
 *
 * It returns size of msg queue head unused buf size, unit byte
 */
uint32_t dsp_msq_head_free_size(const struct dsp_msg_queue *q)
{
    if (q->cur_send >= q->cur_recv) {
        return (q->cur_recv - q->buf_head);
    }

    return 0;
}

/**
 * dsp_msq_read_head - read dsp msg queue head
 *
 * @spi: spi device
 * @addr: msg queue head addr
 * @m: msg queue pointer
 *
 * It returns zero on success, else a negative error code.
 */
int dsp_msq_read_head(uint32_t addr, struct dsp_msg_queue *q)
{
    int err = 0;
    int32_t reg;

    err = spi2apb_safe_r32(DSP_PMU_SYS_REG0, &reg);

    if (err || ((reg & DSP_MSG_QUEUE_OK_MASK) != DSP_MSG_QUEUE_OK_TAG)) {
        //dev_warn(&spi->dev, "dsp msg queue head not init!\n");
        return -1;
    }

    err = spi2apb_safe_read(addr, (int32_t*)q, sizeof(*q));
    return err;
}

/**
 * dsp_msq_send_msg - send a msg to AP -> DSP msg queue
 *
 * @spi: spi device
 * @m: a msg to send
 *
 * It returns zero on success, else a negative error code.
 */
int dsp_msq_send_msg(const struct msg *m)
{
    int err = 0;
    struct dsp_msg_queue queue;
    struct dsp_msg_queue *q = &queue;
    uint32_t msg_size = m->size * sizeof(uint32_t);

    err = dsp_msq_read_head(DSP_R_MSG_QUEUE_ADDR, q);
    if (err)
        return err;

    if (dsp_msq_tail_free_size(q) > msg_size) {
        uint32_t next_send;

        err = spi2apb_safe_write(q->cur_send, (int32_t*)m, msg_size);
        next_send = q->cur_send + msg_size;
        if (next_send == q->buf_tail) {
            next_send = q->buf_head;
        }
        q->cur_send = next_send;
    } else if (dsp_msq_head_free_size(q) > msg_size) {
        //set size to 0 for skip to head mark
        err = spi2apb_safe_w32(q->cur_send, 0);
        if (err)
            return err;

        err = spi2apb_safe_write(q->buf_head, (int32_t*)m, msg_size);

        q->cur_send = q->buf_head + msg_size;
    } else {
        return -1;
    }

    if (err)
        return err;

    err = spi2apb_safe_w32(DSP_R_MSG_QUEUE_ADDR +
            (uint8_t*)&q->cur_send - (uint8_t*)q, q->cur_send);

    spi2apb_interrupt_request(PREISP_IRQ_TYPE_MSG);

    return err;
}

/**
 * dsp_msq_recv_query - query next msg size from DSP -> AP msg queue
 *
 * @q: msg queue
 * @size: msg size buf [out]
 *
 * It returns zero on success, else a negative error code.
 */
int dsp_msq_recv_query(int32_t *size)
{
    struct dsp_msg_queue queue;
    struct dsp_msg_queue *q = &queue;
    int err = 0;

    err = dsp_msq_read_head(DSP_S_MSG_QUEUE_ADDR, q);
    if (err)
        return err;

    *size = 0;
    if (dsp_msq_is_empty(q))
        return 0;

    //skip to head when size is 0
    err = spi2apb_safe_r32(q->cur_recv, size);
    if (err)
        return err;

    if (*size == 0) {
        err = spi2apb_safe_r32((int32_t)q->buf_head, size);
    }

    return err;
}

/**
 * dsp_msq_recv_msg - receive a msg from DSP -> AP msg queue
 *
 * @q: msg queue
 * @m: a msg pointer buf [out]
 *
 * need call dsp_msq_recv_msg_free to free msg after msg use done
 *
 * It returns zero on success, else a negative error code.
 */
int dsp_msq_recv_msg(struct msg **m)
{
    struct dsp_msg_queue queue;
    struct dsp_msg_queue *q = &queue;
    uint32_t size = 0, msg_size = 0;
    uint32_t recv_addr = 0;
    uint32_t next_recv_addr = 0;
    int err = 0;

    *m = NULL;

    err = dsp_msq_read_head(DSP_S_MSG_QUEUE_ADDR, q);
    if (err)
        return err;

    if (dsp_msq_is_empty(q))
        return -1;

    //skip to head when size is 0
    err = spi2apb_safe_r32((int32_t)q->cur_recv, (int32_t*)&size);
    if (err)
        return err;
    if (size == 0) {
        err = spi2apb_safe_r32((int32_t)q->buf_head,
                (int32_t*)&size);
        if (err)
            return err;

        msg_size = size * sizeof(uint32_t);
        recv_addr = q->buf_head;
        next_recv_addr = q->buf_head + msg_size;
    } else {
        msg_size = size * sizeof(uint32_t);
        recv_addr = q->cur_recv;
        next_recv_addr = q->cur_recv + msg_size;
        if (next_recv_addr == q->buf_tail) {
            next_recv_addr = q->buf_head;
        }
    }

    if (msg_size > dsp_msq_total_size(q))
        return -2;

    *m = (struct msg*)malloc(msg_size);
    err = spi2apb_safe_read(recv_addr, (int32_t*)*m, msg_size);
    if (err == 0) {
        err = spi2apb_safe_w32(DSP_S_MSG_QUEUE_ADDR +
            (uint8_t*)&q->cur_recv - (uint8_t*)q,next_recv_addr);
    }

    if (err) {
        free(*m);
    }

    return err;
}

/**
 * dsp_msq_free_received_msg - free a received msg
 *
 * @q: msg queue
 * @m: a msg
 *
 * It returns zero on success, else a negative error code.
 */
int dsp_msq_free_received_msg(const struct msg *m)
{
    if (m != NULL)
        free(m);

    return 0;
}

int cmd_msg_init_sensor(int8_t cam_id, int8_t in_mipi, int8_t out_mipi, char* sensor_name, uint8_t i2c_slave_addr, uint8_t i2c_bus)
{
	msg_init_sensor_t msg;

	memset(&msg, 0, sizeof(msg));
	msg.size = sizeof(msg);
	msg.type = id_msg_init_sensor_t;
	msg.camera_id = cam_id;
	msg.sync = 1;
	msg.in_mipi_phy = in_mipi;
	msg.out_mipi_phy = out_mipi;
	msg.mipi_lane = 2;
	msg.bayer = 0;
	memcpy((void *)msg.sensor_name, sensor_name, 32);
	msg.i2c_slave_addr = i2c_slave_addr;
	msg.i2c_bus = i2c_bus;

	return dsp_msq_send_msg(&msg);
}

int cmd_msg_set_pleco_input_size(int8_t cam_id, uint16_t embed_width, uint16_t embed_height, uint16_t pixel_width, uint16_t pixel_height)
{
	msg_set_pleco_input_size_t msg;

	memset(&msg, 0, sizeof(msg));
	msg.msg_head.size = sizeof(msg) / sizeof(int);
	msg.msg_head.type = id_msg_set_input_size_t;
	msg.msg_head.id.camera_id = cam_id;
	msg.msg_head.mux.sync = 1;

	msg.channel_embed.data_type = 0x12;
	msg.channel_embed.decode_format = 0x12;
	msg.channel_embed.width = embed_width;
	msg.channel_embed.height = embed_height;
	msg.channel_embed.flag = 0;

	msg.channel_raw.data_type = 0x2B;
	msg.channel_raw.decode_format = 0x2B;
	msg.channel_raw.width = pixel_width;
	msg.channel_raw.height = pixel_height;
	msg.channel_raw.flag = 1;

	return dsp_msq_send_msg(&msg);
}

int cmd_msg_set_output_size(int8_t cam_id, uint16_t width, uint16_t height, uint16_t htotal, uint16_t vtotal, uint32_t mipi_clock)
{
	msg_set_output_size_t msg;

	memset(&msg, 0, sizeof(msg));
	msg.msg_head.size = sizeof(msg) / sizeof(int);
	msg.msg_head.type = id_msg_set_output_size_t;
	msg.msg_head.camera_id = cam_id;
	msg.msg_head.sync = 1;
	msg.msg_head.mipi_lane = 4;
	msg.msg_head.mipi_clk = mipi_clock;
	msg.msg_head.line_length_pclk = htotal;
	msg.msg_head.frame_length_lines = vtotal;
	msg.msg_head.height = height;
	msg.msg_head.width = width;

	msg.channel.data_type = 0x2A;
	msg.channel.decode_format = 0x2A;
	msg.channel.width = width;
	msg.channel.height = height;
	msg.channel.flag = 1;

	return dsp_msq_send_msg(&msg);
}

int cmd_msg_set_input_size(int8_t cam_id, uint16_t width, uint16_t height)
{
	msg_set_input_size_t msg;

	memset(&msg, 0, sizeof(msg));
	msg.msg_head.size = sizeof(msg) / sizeof(int);
	msg.msg_head.type = id_msg_set_input_size_t;
	msg.msg_head.id.camera_id = cam_id;
	msg.msg_head.mux.sync = 1;
	msg.channel.data_type = 0x2b;
	msg.channel.decode_format = 0x2b;
	msg.channel.width = width;
	msg.channel.height = height;
	msg.channel.flag = 1;

	dsp_msq_send_msg(&msg);
}

int cmd_msg_set_stream_in_on(int8_t cam_id)
{
	msg_t msg;

	memset(&msg, 0, sizeof(msg));
	msg.size = sizeof(msg);
	msg.type = id_msg_set_stream_in_on_t;
	msg.id.camera_id = cam_id;
	msg.mux.sync = 1;

	return dsp_msq_send_msg(&msg);
}

int cmd_msg_set_stream_in_mode(int8_t cam_id, uint8_t mode, uint8_t type, uint8_t freq, uint8_t skip_frame)
{
    msg_set_input_mode_t msg;
    memset(&msg, 0, sizeof(msg));
    msg.msg_head.size = sizeof(msg) / sizeof(int);
    msg.msg_head.type = id_msg_set_algo_in_stream_mode;
    msg.msg_head.id.camera_id = cam_id;
    msg.msg_head.mux.sync = 1;
    msg.frame_mode = mode;
    msg.frame_type = type,
    msg.sync_freq = freq;
    msg.skip_frame = skip_frame;
    return dsp_msq_send_msg(&msg);
}

int cmd_msg_set_stream_out_type(int8_t cam_id, uint8_t type)
{
    msg_set_out_stream_type_t msg;
    memset(&msg, 0, sizeof(msg));
    msg.msg_head.size = sizeof(msg) / sizeof(int);
    msg.msg_head.type = id_msg_set_algo_out_stream_type;
    msg.msg_head.id.camera_id = cam_id;
    msg.msg_head.mux.sync = 1;
    msg.stream_type = type;
    return dsp_msq_send_msg(&msg);
}

int cmd_msg_set_param_init(int8_t cam_id, uint32_t init_param)
{
    msg_ext_t msg;

    memset(&msg, 0, sizeof(msg));
    msg.size = sizeof(msg);
    msg.type = id_msg_set_algo_param_init;
    msg.id.camera_id = cam_id;
    msg.mux.sync = 1;
    msg.ext.data = init_param;

    return dsp_msq_send_msg(&msg);
}

int cmd_msg_set_stream_in_off(int8_t cam_id)
{
	msg_t msg;

	memset(&msg, 0, sizeof(msg));
	msg.size = sizeof(msg);
	msg.type = id_msg_set_stream_in_off_t;
	msg.id.camera_id = cam_id;
	msg.mux.sync = 1;

	return dsp_msq_send_msg(&msg);
}

int cmd_msg_set_stream_out_on(int8_t cam_id)
{
	msg_t msg;

	memset(&msg, 0, sizeof(msg));
	msg.size = sizeof(msg);
	msg.type = id_msg_set_stream_out_on_t;
	msg.id.camera_id = cam_id;
	msg.mux.sync = 1;

	return dsp_msq_send_msg(&msg);
}

int cmd_msg_set_stream_out_off(int8_t cam_id)
{
	msg_t msg;

	memset(&msg, 0, sizeof(msg));
	msg.size = sizeof(msg);
	msg.type = id_msg_set_stream_out_off_t;
	msg.id.camera_id = cam_id;
	msg.mux.sync = 1;

    return dsp_msq_send_msg(&msg);
}

int cmd_msg_set_ae_param(int8_t cam_id, AEParam *ae_param)
{
    msg_set_ae_param_t msg;

    memset(&msg, 0, sizeof(msg));
    msg.msg_head.size = sizeof(msg) / sizeof(int);
    msg.msg_head.type = id_msg_set_ae_param;
    msg.msg_head.id.camera_id = cam_id;
    msg.msg_head.mux.sync = 0;

    msg.fov = ae_param->fov;
    msg.t_min = ae_param->t_min;
    msg.t_max = ae_param->t_max;
    msg.t_step_ratio_min = ae_param->t_step_ratio_min;
    msg.t_step_ratio_max = ae_param->t_step_ratio_max;
    msg.over_exposure_value = ae_param->over_exposure_value;
    msg.over_dark_value = ae_param->over_dark_value ;
    msg.ratio_thresh = ae_param->ratio_thresh;
    msg.ir_thresh = ae_param->ir_thresh;

    return dsp_msq_send_msg(&msg);
}

