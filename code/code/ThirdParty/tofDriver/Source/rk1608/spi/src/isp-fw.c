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
#include <windows.h>
#include "spi\inc\isp-fw.h"
#include "spi\inc\spi2apb.h"
#include <tof_sensors.h>

#ifdef WIN32
#include <stdio.h>
#define __FUNCTION__ __FUNCTION__ 
#endif

#define TEE_LOG_LEVEL_ERROR        8
#define TEE_LOG_LEVEL_DEBUG        2
#define ALOGE(fmt,...)			   tops_t.qsee_log(TEE_LOG_LEVEL_ERROR, "[ERROR] [%s(%d)] : " fmt"\n",__FUNCTION__,__LINE__,##__VA_ARGS__)
#define ALOGD(fmt,...)			   tops_t.qsee_log(TEE_LOG_LEVEL_DEBUG, "[DEBUG] [%s(%d)] : " fmt"\n",__FUNCTION__,__LINE__,##__VA_ARGS__)
#define free tops_t.qsee_free

static int spi_read_wait(const struct rkl_section *sec)
{
    int32_t value = 0;
    int try_ = 0;
    int ret = 0;

    do {
        ret = spi2apb_safe_r32(sec->wait_addr, &value);

        if (!ret && value == sec->wait_value)
            break;

        if (try_++ == sec->timeout) {
            ret = -1;
			ALOGE("read 0x%x is %x != %x timeout\n", sec->wait_addr, value, sec->wait_value);
            break;
        }
        Sleep(sec->wait_time);
    } while (1);

    return ret;
}

static int spi_boot_request(const struct rkl_section * sec)
{
    struct rkl_boot_request boot_req;
    int try_ = 0;
    int ret = 0;

    //send boot request to dsp for ddr init
    boot_req.flag = sec->flag;
    boot_req.loadAddr = sec->load_addr;
    boot_req.bootLen = sec->size;
    boot_req.status = 1;
    boot_req.cmd = 2;

    ret = spi2apb_safe_write(BOOT_REQUEST_ADDR,
            (int32_t*)&boot_req, sizeof(boot_req));
    if (ret)
        return ret;

    if (sec->flag & BOOT_FLAG_READ_WAIT) {
        //waitting for dsp init ddr done
        do {
            ret = spi2apb_safe_read(BOOT_REQUEST_ADDR,
                    (int32_t*)&boot_req, sizeof(boot_req));

            if (!ret && boot_req.status == 0)
                break;

            if (try_++ == sec->timeout) {
                ret = -1;
				ALOGE("boot_request timeout\n");
                break;
            }
            Sleep(sec->wait_time);
        } while (1);
    }

    return ret;
}

static int spi_download_section(const uint8_t *data, const struct rkl_section *sec)
{
    int ret = 0;

	ALOGD("offset/wait_value:0x%x,size:%d,addr:0x%x,"
		"wait_time:%d,timeout:%d,crc:0x%x,flag:0x%x,type:0x%x",
		sec->offset, sec->size, sec->load_addr, sec->wait_time,
		sec->timeout, sec->crc_16, sec->flag, sec->type);
    
    if (sec->size > 0) {
        ret = spi2apb_safe_write(sec->load_addr,
                (int32_t*)(data + sec->offset), sec->size);
        if (ret)
            return ret;
    }

    if (sec->flag & BOOT_FLAG_BOOT_REQUEST) {
        ret = spi_boot_request(sec);
    } else if (sec->flag & BOOT_FLAG_READ_WAIT) {
        ret = spi_read_wait(sec);
    }

    return ret;
}

/**
 * spi_download_fw: - rk preisp firmware download through spi
 *
 * @spi: spi device
 * @fw_name: name of firmware file, NULL for default firmware name
 * Context: can sleep
 *
 * It returns zero on success, else a negative error code.
 **/
int spi_download_fw(const uint8_t *fw_data)
{
    const struct rkl_header * head;

    int i = 0;
    int ret = 0;

    head = (const struct rkl_header *) fw_data;

	ALOGD("request firmware (version:%s header_size:%d) success!", head->version, head->header_size);

    for (i = 0; i < head->section_count; i++) {
        ret = spi_download_section(fw_data, &head->sections[i]);
        if (ret)
            break;
    }
	free(fw_data);
    return ret;
}
