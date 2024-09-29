#ifndef DELAY_BOARD_H
#define DELAY_BOARD_H
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C"{
#endif

int nb6l295_write(uint16_t value); // delay unit: ps
int nb6l295_read(uint16_t *value);
int nb6l295_get_id(uint16_t *id);


int delay_board_set_power_on_off(uint8_t on_off);
int delay_board_get_power_on_off(uint8_t *value_p);

int delay_board_get_firmware_version(uint8_t *value_p);

int delay_board_set_delay_select(uint8_t state);
int delay_board_get_delay_selcet(uint8_t *state_p);

int delay_board_set_delay_time(uint32_t ps);
int delay_board_get_delay_time(uint32_t *ps_p);

int delay_board_get_all_temperature(uint16_t *adc_v_p, double *centi_p);

#ifdef __cplusplus
}
#endif


#endif
