#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include "mx6xhal\hw_modules.h"
#include "tof_sensors.h"
#include "mx6xhal\hw_obstatus.h"
#include "mx6xhal\depth_obstatus.h"

#include "mlx75027\mlx75027.h"
#include "mlx75027\mlx75027_max96717_max96716a.h"
#include "s5k33dxx\s5k33dxx.h"
#include "imx516\imx516.h"
#include "imx456\imx456.h"
#include "imx518\imx518.h"
#include "imx627\imx627.h"
#include "imx316\imx316.h"
#include "pleco\pleco.h"
#include "gaea\gaea.h"
#include "dapperen100hy\dapperen100hy.h"
#include "rk1608\debug2log.h"
#include "project_config.h"

#ifdef WIN32
#include <stdio.h>
#define __FUNCTION__ __FUNCTION__
#endif

#define max_cnt 8

//#define TEE_LOG_LEVEL_ERROR        8
//#define ALOGE(fmt,...) tops_t.qsee_log(TEE_LOG_LEVEL_ERROR, "[ERROR] [%s(%d)] : " fmt"\n",__FUNCTION__,__LINE__,##__VA_ARGS__)


sensor_list_t sensor_list[] = {
	//{ mlx75027_sensor_id, mlx75027_func_init 
	{ dapperen100hy_sensor_id, dapperen100hy_func_init },
    //{ gaea_sensor_id, gaea_func_init },
    //{ mlx75027_max96717_max96716a_sensor_id, mlx75027_max96717_max96716a_func_init },
	//{ s5k33dxx_sensor_id, s5k33dxx_func_init },
    //{ imx456_sensor_id,   imx456_func_init },
    //{ imx516_sensor_id,   imx516_func_init },
	//{ imx518_sensor_id,   imx518_func_init },
	//{ pleco_sensor_id,   pleco_func_init },
   // { imx627_sensor_id,   imx627_func_init },
    //{ imx316_sensor_id,   imx316_func_init },
};

/*
@brief:  加载系统函数 操作接口
@param[in] ops 系统函数操作接口
@return   MX6X_NO_ERR          成功
-MX6X_ERR_NULL       传入空指针
-MX6X_ERR_INVALID   无效参数
*/
int tof_sensor_lib_init(void * ops)
{

	//printf("master_i2c_lib_init ...\n");
	if (ops)
		memcpy(&tops_t, ops, sizeof(obc_ops_t));
	else
		return -HW_ERR_NULL;

	if ((!tops_t.qsee_log) || (!tops_t.qsee_malloc) || (!tops_t.qsee_free) || (!tops_t.ops_writeread) || (!tops_t.tee_usleep)) {
		printf("master_i2c_lib_init  ops members is qseelog = %p, qsee_malloc = %p, qsee_free = %p, ii_writeread = %p  usleep = %p\n",
			tops_t.qsee_log, tops_t.qsee_malloc, tops_t.qsee_free, tops_t.ops_writeread, tops_t.tee_usleep);
		return -HW_ERR_NULL;
	}
	//ALOGE("tof_sensor_lib_init OK!");

	return 0;
}

/*
 *@brief:  加载TOF sensor功能函数结构体
 * 通过查询设备信息，匹配合适的芯片驱动
@return   MX6X_NO_ERR          成功
-MX6X_ERR_NULL       传入空指针
-MX6X_ERR_INVALID   无效参数
*/
int tof_sensor_init()
{
	uint8_t i;
	uint16_t sensor_id = 0;
	int16_t ret;

	//memset(&tof_sensor, 0, sizeof(tof_sensor));

	for (i = 0; i < sizeof(sensor_list) / sizeof(sensor_list_t); i++)
	{
		if (sensor_list[i].sensor_init == NULL){
			break;
		}
        memset(&tof_sensor, 0, sizeof(tof_sensor));
		sensor_list[i].sensor_init();

		tof_sensor.get_sensor_id(&sensor_id);

		if (sensor_id == sensor_list[i].SensorId) {
			ret = tof_sensor.init();
			break;
		}
	}

	if (i < sizeof(sensor_list) / sizeof(sensor_list_t)) {
		return  ret;
	}

	return  -HW_ERR_NO_MATCH;
}

int tof_get_lib_version(uint32_t * pversion)
{
	if (NULL == pversion) {
		return -DEPTH_LIB_NULL_INPUT_PTR;
	}

	*pversion = OBTOF_DRIVER_VERSION_MAJOR << 24 | OBTOF_DRIVER_VERSION_MINOR << 16 | OBTOF_DRIVER_VERSION_PATCHLEVEL << 8 | OBTOF_DRIVER_VERSION_RELEASER;
	return 0;
}
