#ifndef __TOF_BOARD_H__
#define __TOF_BOARD_H__
#include <stdint.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif


#define  FM24C64_ADDR					 0xA2
#define  BOARD_REG						 0x00
#define  RK1608_EEPROM_CSP_DEFAULT       0xB0
#define  RK1608_EEPROM_ADDR_DEFAULT      0x0D

typedef struct {
	uint32_t BoardId;
	int(*board_init)();
} board_list_t;

typedef struct tof_board_func
{
		int(*init)();
}tof_board_func_t;

extern tof_board_func_t tof_board;

int tof_board_init();
int tof_sensor_board_type(int *type);

#ifdef __cplusplus
}
#endif

#endif