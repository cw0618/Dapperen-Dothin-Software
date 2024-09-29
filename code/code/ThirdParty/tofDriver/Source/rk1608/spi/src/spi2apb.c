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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <windows.h>
#include "spi\inc\spi2apb.h"
#include <tof_sensors.h>
#include "spi\inc\msg-interface.h"

#ifdef WIN32
#include <stdio.h>
#define __FUNCTION__ __FUNCTION__ 
#endif

#define TEE_LOG_LEVEL_ERROR        8
#define TEE_LOG_LEVEL_DEBUG        2
#define ALOGE(fmt,...)			   tops_t.qsee_log(TEE_LOG_LEVEL_ERROR, "[ERROR] [%s(%d)] : " fmt"\n",__FUNCTION__,__LINE__,##__VA_ARGS__)
#define ALOGD(fmt,...)			   tops_t.qsee_log(TEE_LOG_LEVEL_DEBUG, "[DEBUG] [%s(%d)] : " fmt"\n",__FUNCTION__,__LINE__,##__VA_ARGS__)
#define malloc					   tops_t.qsee_malloc

#define SPI_CTRL0				   0x11060000
#define SPI_ENR					   0x11060008

#define W25Q64_READ_STATUS_RIGSTER1          0x05
#define W25Q64_READ_DATA					 0x03
#define W25Q64_WRITE_ENABLE					 0x06
#define W25Q64_WRITE_DISABLE				 0x04
#define W25Q64_PAGE_PROGRAM					 0x02
#define W25Q64_CHIP_ERASE					 0xC7
#define W25Q64_JEDEC_ID						 0x9F
#define W25Q64_BLOCK_ERASE_64KB				 0xD8

#define W25Q64_PAGE_SIZE					 256 // byte
#define W25Q64_BLOCK_ERASE_SIZE				 (64*1024)  // byte

static int spi_block_write_ex(uint8_t *data, uint32_t data_size)
{
	return spi_block_rw(data, NULL, data_size, 0);
}

static int w25q64_write_cmd(uint8_t cmd)
{
	return spi_block_write_ex(&cmd, 1);
}

static int w25q64_get_jedec_id(uint8_t *id)
{
	
	uint32_t cmd = W25Q64_JEDEC_ID;

	return spi_block_rw(&cmd, id, 4, 3);
}

static int w25q64_chech_busy() // return 1 is busy
{
	uint16_t cmd = W25Q64_READ_STATUS_RIGSTER1;
	uint8_t status;
	spi_block_rw(&cmd, &status, 2, 1);
	Sleep(1);
	return (status & 0x01);
}

static int w25q64_block_erase(uint8_t block_num) // start from zero
{
	int ret;
	uint8_t tx_buf[4];
	for (int i = 0; i < block_num; i++) {
		
		uint32_t addr = W25Q64_BLOCK_ERASE_SIZE * i;
		tx_buf[0] = W25Q64_BLOCK_ERASE_64KB;
		tx_buf[1] = (addr & 0xFF0000) >> 16;
		tx_buf[2] = (addr & 0xFF00) >> 8;
		tx_buf[3] = (addr & 0xFF);
		ALOGD("w25q64 erase block : %d, addr : %d", i, addr);
		ret = w25q64_write_cmd(W25Q64_WRITE_ENABLE);
		ret = spi_block_write_ex(tx_buf, 4);
		Sleep(150); // TODO: chech busy status here
	}
	return ret;
}

static int w25q64_read_data(uint32_t addr, uint8_t *data, uint32_t data_len)
{
	uint8_t *local_buf = NULL;
	uint8_t cmd = W25Q64_READ_DATA;

	size_t cmd_len = 1;
	size_t addr_len = 3;
	local_buf = malloc(cmd_len + addr_len + data_len);
	if (!local_buf)
		return -1;

	local_buf[0] = cmd;
	local_buf[1] = (addr & 0xFF0000) >> 16;
	local_buf[2] = (addr & 0xFF00) >> 8;
	local_buf[3] = (addr & 0xFF);

	// dummy write data_len bytes when read
	int ret = spi_block_rw(local_buf, data, (4 + data_len), data_len);

	if (local_buf)
		free(local_buf);

	return ret;
}

static int w25q64_page_program(uint32_t addr, uint8_t *data, uint32_t data_len)
{
	if (addr % W25Q64_PAGE_SIZE != 0)
		return -2;
	unsigned char *local_buf = NULL;
	unsigned char cmd = W25Q64_PAGE_PROGRAM;

	size_t cmd_len = 1;
	size_t addr_len = 3;
	local_buf = malloc(cmd_len + addr_len + data_len);
	if (!local_buf)
		return -1;

	unsigned char addr1 = (addr & 0xFF0000) >> 16;
	unsigned char addr2 = (addr & 0xFF00) >> 8;
	unsigned char addr3 = (addr & 0xFF);
	memcpy(local_buf, &cmd, cmd_len);
	memcpy(local_buf + 1, &addr1, 1);
	memcpy(local_buf + 2, &addr2, 1);
	memcpy(local_buf + 3, &addr3, 1);
	memcpy(local_buf + cmd_len + addr_len, data, data_len);
	
	while (w25q64_chech_busy());
	int ret = w25q64_write_cmd(W25Q64_WRITE_ENABLE);
	ret = spi_block_write_ex(local_buf, (cmd_len + addr_len + data_len));
	if (local_buf)
		free(local_buf);

	return ret;
}

static int flash_download_fw(uint8_t *data, int data_len)
{
	int ret = 0;
	size_t max_op_size = W25Q64_PAGE_SIZE;
	uint32_t addr = 0x00000000;

	int erase_num = (data_len / W25Q64_BLOCK_ERASE_SIZE) + 1;
	ALOGD("total erase num: %d", erase_num);
	ret = w25q64_block_erase(erase_num);

	while (data_len > 0) {
		size_t slen = MIN(data_len, max_op_size);

		ret = w25q64_page_program(addr, data, slen);
		if (ret)
			break;

		data_len = data_len - slen;
		data = ((uint8_t*)data + slen);
		addr += slen;
	}
	ALOGD("download firmware to flash finished");
}

static int w25q64_test()
{
	//set_duxin_access_flash();

#if 0
	uint8_t id[3];
	w25q64_get_jedec_id(id);
	printf("w25q64 jedec id: %d  %d  %d", id[0], id[1], id[2]);
#endif

#if 1 
	#define size  100	/**/
	uint8_t buf_write[size];
	w25q64_block_erase(0);

	for (int i = 0; i < size; i++) {
		buf_write[i] = i;
	}
	flash_download_fw(buf_write, size);
	
	uint8_t buf_read[size];
	w25q64_read_data(0x00000000, buf_read, size);
	for (int i = 0; i < 100; i++) {
		//if (buf_write[i] != buf_read[i])
		ALOGD("w25q64 read addr: %d, data: %d", i, buf_read[i]);
	}
#endif
	return 0;
}

uint8_t int8_msb2lsb(uint8_t src)
{
    int i;
    uint8_t dst = 0;

    for (i = 0; i < 8; i++) {
        uint8_t t = 0;
        t = (((src >> i) & 0x01) << (7 - i));
        dst |= t;
    }

    return dst;
}

uint32_t int32_msb2lsb(uint32_t src)
{
    uint32_t dst = 0;
    int i;

    for (i = 0; i < 4; i++) {
        int shift = i * 8;
        dst = dst | (int8_msb2lsb((src >> shift) & 0xff) << shift);
    }

    return dst;
}

int spi2apb_lsb_w32(int32_t addr, int32_t data)
{
    int32_t write_cmd = APB_CMD_WRITE;

    write_cmd = int32_msb2lsb(write_cmd);
    addr = int32_msb2lsb(addr);
    data = int32_msb2lsb(data);

    int32_t tx_buff[3];
    tx_buff[0] = write_cmd;
    tx_buff[1] = addr;
    tx_buff[2] = data;
    return spi_block_write_ex((uint8_t *)tx_buff, sizeof(tx_buff));
}

/**
 * spi2apb_switch_to_msb - SPI2APB set Fist bit mode to MSB
 *
 * @spi: spi device
 * Context: can sleep
 *
 */
void spi2apb_switch_to_msb()
{
    spi2apb_lsb_w32(SPI_ENR, 0);
    spi2apb_lsb_w32(SPI_CTRL0, 0x108002);
}

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
int spi2apb_write(int32_t addr, const int32_t *data, size_t data_len)
{
    uint8_t *local_buf = NULL;
    int32_t write_cmd = APB_CMD_WRITE;

    size_t cmd_len = sizeof(write_cmd);
    size_t addr_len = sizeof(addr);
    local_buf = malloc(cmd_len + addr_len + data_len);
    if (!local_buf)
        return -1;
    memcpy(local_buf, &write_cmd, cmd_len);
    memcpy(local_buf + cmd_len, &addr, addr_len);
    memcpy(local_buf + cmd_len + addr_len, data, data_len);

    int ret = spi_block_write_ex(local_buf, (cmd_len + addr_len + data_len));

    if (local_buf)
        free(local_buf);

    return ret;
}


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
int spi2apb_w32(int32_t addr, int32_t data)
{
    return spi2apb_write(addr, &data, 4);
}

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
int spi2apb_read(int32_t addr, int32_t *data, size_t data_len)
{
    int32_t real_len = MIN(data_len, APB_MAX_OP_BYTES);
    int32_t read_cmd = APB_CMD_READ | (real_len << 14 & 0xffff0000);
    int32_t read_begin_cmd = APB_CMD_READ_BEGIN;
    int32_t dummy = 0;

    int32_t tx_buff[4];
    tx_buff[0] = read_cmd;
    tx_buff[1] = addr;
    tx_buff[2] = dummy;
    tx_buff[3] = read_begin_cmd;

    return spi_block_rw((uint8_t *)tx_buff, (uint8_t *)data, (sizeof(tx_buff) + data_len), data_len);
}

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
int spi2apb_r32(int32_t addr, int32_t *data)
{
    return spi2apb_read(addr, data, 4);
}

/**
 * spi2apb_operation_query - SPI2APB last operation state query
 *
 * @spi: device from which data will be read
 * @state: last operation state [out]
 * Context: can sleep
 *
 * It returns zero on success, else a negative error code.
 */
int spi2apb_operation_query(int32_t *state)
{
    int32_t query_cmd = APB_CMD_QUERY;
    spi_block_rw(&query_cmd, (uint8_t *)state, 4 + 4, 4);

    return ((*state & APB_OP_STATE_ID_MASK) == APB_OP_STATE_ID) ? 0 : -1;
}

/**
 * spi2apb_state_query - SPI2APB system state query
 *
 * @spi: spi device
 * @state: system state [out]
 * Context: can sleep
 *
 * It returns zero on success, else a negative error code.
 */
int spi2apb_state_query(int32_t *state)
{
    int ret = 0;
    int32_t query_cmd = APB_CMD_QUERY_REG2;

    return spi_block_rw(&query_cmd, (uint8_t *)state, 4 + 4, 4);
}

/**
 * spi2apb_interrupt_request - SPI2APB request a dsp interrupt
 *
 * @spi: spi device
 * @interrupt_num: interrupt identification
 * Context: can sleep
 *
 * It returns zero on success, else a negative error code.
 */
int spi2apb_interrupt_request(int32_t interrupt_num)
{
    int ret = 0;
    int32_t write_reg1_cmd = APB_CMD_WRITE_REG1;
    int32_t tx_buff[2];
    tx_buff[0] = write_reg1_cmd;
    tx_buff[1] = interrupt_num;

    return spi_block_write_ex(tx_buff, sizeof(tx_buff));
}

/**
 * _spi2apb_safe_write - SPI2APB synchronous write with state check
 *
 * @spi: spi device
 * @addr: apb resource address
 * @data: data buffer
 * @data_len: data buffer size, in bytes
 * Context: can sleep
 *
 * It returns zero on success, else operation state code.
 */
int _spi2apb_safe_write(int32_t addr, const int32_t *data, size_t data_len)
{
    int32_t state = 0;
    int32_t try_ = 0;

    do {
        int ret = 0;
        ret = spi2apb_write(addr, data, data_len);
        if (ret == 0)
            ret = spi2apb_operation_query(&state);

        if (ret != 0) {
            return ret;
        } else if ((state & APB_OP_STATE_MASK) == 0) {
            break;
        }

        if (try_++ == APB_SAFE_OPERATION_TRY_MAX)
            break;
        //udelay(APB_SAFE_OPERATION_TRY_DELAY_US);
        Sleep(1);
    } while (1);

    return (state & APB_OP_STATE_MASK);
}

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
int spi2apb_safe_write(int32_t addr, const int32_t *data, size_t data_len)
{
    int ret = 0;
    size_t max_op_size = MIN((size_t)APB_MAX_OP_BYTES, (size_t)DUXIN_MAX_OP_BYTES);

    while (data_len > 0) {
        size_t slen = MIN(data_len, max_op_size);

        ret = _spi2apb_safe_write(addr, data, slen);
        if (ret)
            break;

        data_len = data_len - slen;
        data = (int32_t*)((int8_t*)data + slen);
        addr += slen;
    }
    return ret;
}

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
int spi2apb_safe_w32(int32_t addr, int32_t data)
{
    return _spi2apb_safe_write(addr, &data, 4);
}

/**
 * _spi2apb_safe_read - SPI2APB synchronous read with state check
 *
 * @spi: spi device
 * @addr: apb resource address
 * @data: data buffer [out]
 * @data_len: data buffer size, in bytes
 * Context: can sleep
 *
 * It returns zero on success, else operation state code.
 */
int _spi2apb_safe_read(int32_t addr, int32_t *data, size_t data_len)
{
    int32_t state = 0;
    int32_t try_ = 0;

    do {
        int ret = 0;
        ret = spi2apb_read(addr, data, data_len);
        if (ret == 0)
            ret = spi2apb_operation_query(&state);

        if (ret != 0) {
            return ret;
        } else if ((state & APB_OP_STATE_MASK) == 0) {
            break;
        }

        if (try_++ == APB_SAFE_OPERATION_TRY_MAX)
            break;
        //udelay(APB_SAFE_OPERATION_TRY_DELAY_US);
        Sleep(1);
    } while (1);

    return (state & APB_OP_STATE_MASK);
}

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
int spi2apb_safe_read(int32_t addr, int32_t *data, size_t data_len)
{
    int ret = 0;
    size_t max_op_size = MIN((size_t)APB_MAX_OP_BYTES, (size_t)DUXIN_MAX_OP_BYTES);

    while (data_len > 0) {
        size_t slen = MIN(data_len, max_op_size);

        ret = _spi2apb_safe_read(addr, data, slen);
        if (ret)
            break;

        data_len = data_len - slen;
        data = (int32_t*)((int8_t*)data + slen);
        addr += slen;
    }
    return ret;
}

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
int spi2apb_safe_r32(int32_t addr, int32_t *data)
{
    return spi2apb_safe_read(addr, data, 4);
}
