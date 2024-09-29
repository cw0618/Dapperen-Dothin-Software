#ifndef __PROJECT_CONFIG_H__
#define __PROJECT_CONFIG_H__

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include "mx6xhal\obc_tee_funcs.h"
#include "tof_sensors.h"
#include "rk1608\debug2log.h"

/********************************************* Project *************************************************/
/*IMX518 Project:*/
#define IMX518_REG_LIST_6272           1  // normal used setting
#define IMX518_REG_LIST_6130           0  // QC and AA used setting

/*S5K33D Project:*/
#define S5K33D_TAISHAN               0x0201  // Putuoshan and Polaris share the same project id with Taishan
#define S5K33D_HUANGSHAN             0x0301  // Huangshan
#define S5K33D_T200515               0x0401  // T200515
#define S5K33D_KUNLUNSHAN            0x0501  // RK1608 + S5K33D
#define S5K33D_F201201               0x0601  // F201201
#define S5K33D_X210116               0x0701  // X210116
/******************************************* End project ***********************************************/



/****************************************** Select project *********************************************/
#define IMX518_PROJECT_SELECT        IMX518_REG_LIST_6272
/**************************************** End select project *******************************************/

#endif  // __PROJECT_CONFIG_H__

