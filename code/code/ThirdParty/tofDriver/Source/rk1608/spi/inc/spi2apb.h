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
#ifndef __RK_PREISP_SPI2APB_H__
#define __RK_PREISP_SPI2APB_H__
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define APB_CMD_WRITE                   0x00000011
#define APB_CMD_WRITE_REG0              0X00010011
#define APB_CMD_WRITE_REG1              0X00020011
#define APB_CMD_READ                    0x00000077
#define APB_CMD_READ_BEGIN              0x000000AA
#define APB_CMD_QUERY                   0x000000FF
#define APB_CMD_QUERY_REG2              0x000001FF

#define APB_OP_STATE_ID_MASK           (0xffff0000)
#define APB_OP_STATE_ID                (0X16080000)

#define APB_OP_STATE_MASK              (0x0000ffff)
#define APB_OP_STATE_WRITE_ERROR       (0x01 << 0)
#define APB_OP_STATE_WRITE_OVERFLOW    (0x01 << 1)
#define APB_OP_STATE_WRITE_UNFINISHED  (0x01 << 2)
#define APB_OP_STATE_READ_ERROR        (0x01 << 8)
#define APB_OP_STATE_READ_UNDERFLOW    (0x01 << 9)
#define APB_OP_STATE_PRE_READ_ERROR    (0x01 << 10)

#define APB_MAX_OP_BYTES               60000
#define DUXIN_MAX_OP_BYTES             4000 // dothin spi can transfer 4000 byte per time

#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif

/*
int spi_block_write_ex(uint8_t *data, uint32_t data_size);
int w25q64_get_jedec_id(uint8_t *id);
int w25q64_block_erase(uint8_t block_num);
int w25q64_page_program(uint32_t addr, uint8_t *data, uint32_t data_len);
int flash_download_fw(uint8_t *data, int data_len);
int w25q64_test();*/
/**
 * spi2apb_switch_to_msb - SPI2APB set Fist bit mode to MSB
 *
 * @spi: spi device
 * Context: can sleep
 *
 */
void spi2apb_switch_to_msb();

/**
 * spi2apb_write - SPI2APB synchronous write
 *
 * @spi: spi device
 * @addr: apb resource address
 * @data: data buffer
 * @data_len: data buffer size, in bytes
 * Context: can sleep
 *
 * It returns zero on success, else a negative error code.
 */
int spi2apb_write(int32_t addr, const int32_t *data, size_t data_len);

/**
 * spi2apb_w32 - SPI2APB synchronous 32-bit write
 *
 * @spi: spi device
 * @addr: apb resource address
 * @data: 32-bit data
 * Context: can sleep
 *
 * It returns zero on success, else a negative error code.
 */
int spi2apb_w32(int32_t addr, int32_t data);

/**
 * spi2apb_read - SPI2APB synchronous read
 *
 * @spi: spi device
 * @addr: apb resource address
 * @data: data buffer [out]
 * @data_len: data buffer size, in bytes
 * Context: can sleep
 *
 * It returns zero on success, else a negative error code.
 */
int spi2apb_read(int32_t addr, int32_t *data, size_t data_len);

/**
 * spi2apb_r32 - SPI2APB synchronous 32-bit read
 *
 * @spi: spi device
 * @addr: apb resource address
 * @data: 32-bit data buffer [out]
 * Context: can sleep
 *
 * It returns zero on success, else a negative error code.
 */
int spi2apb_r32(int32_t addr, int32_t *data);

/**
 * spi2apb_operation_query - SPI2APB last operation state query
 *
 * @spi: spi device
 * @state: last operation state [out]
 * Context: can sleep
 *
 * It returns zero on success, else a negative error code.
 */
int spi2apb_operation_query(int32_t *state);

/**
 * spi2apb_state_query - SPI2APB system state query
 *
 * @spi: spi device
 * @state: system state [out]
 * Context: can sleep
 *
 * It returns zero on success, else a negative error code.
 */
int spi2apb_state_query(int32_t *state);

/**
 * spi2apb_interrupt_request - SPI2APB request a dsp interrupt
 *
 * @spi: spi device
 * @interrupt_num: interrupt identification
 * Context: can sleep
 *
 * It returns zero on success, else a negative error code.
 */
int spi2apb_interrupt_request(int32_t interrupt_num);


#define APB_SAFE_OPERATION_TRY_MAX 3
#define APB_SAFE_OPERATION_TRY_DELAY_US 10

/**
 * spi2apb_safe_write - SPI2APB synchronous write with state check
 *
 * @spi: spi device
 * @addr: apb resource address
 * @data: data buffer
 * @data_len: data buffer size, in bytes
 * Context: can sleep
 *
 * It returns zero on success, else operation state code.
 */
int spi2apb_safe_write(int32_t addr, const int32_t *data, size_t data_len);

/**
 * spi2apb_safe_w32 - SPI2APB synchronous 32-bit write with state check
 *
 * @spi: spi device
 * @addr: apb resource address
 * @data: 32-bit data
 * Context: can sleep
 *
 * It returns zero on success, else operation state code.
 */
int spi2apb_safe_w32(int32_t addr, int32_t data);

/**
 * spi2apb_safe_read - SPI2APB synchronous read with state check
 *
 * @spi: spi device
 * @addr: apb resource address
 * @data: data buffer [out]
 * @data_len: data buffer size, in bytes
 * Context: can sleep
 *
 * It returns zero on success, else operation state code.
 */
int spi2apb_safe_read(int32_t addr, int32_t *data, size_t data_len);

/**
 * spi2apb_safe_r32 - SPI2APB synchronous 32-bit read with state check
 *
 * @spi: spi device
 * @addr: apb resource address
 * @data: 32-bit data buffer [out]
 * Context: can sleep
 *
 * It returns zero on success, else operation state code.
 */
int spi2apb_safe_r32(int32_t addr, int32_t *data);

#ifdef __cplusplus
}
#endif
#endif
