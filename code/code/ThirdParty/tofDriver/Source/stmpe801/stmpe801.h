#ifndef STMPE801_H
#define STMPE801_H
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// IIC slave device
#define STMPE801_I2C_ADDR    0x88  
#define STMPE801_ID          0x0801

int stmpe801_read_id(uint16_t* id);
int stmpe801_init();
int stmpe801_set_io_state(uint8_t gpio, uint8_t level);
int stmpe801_get_io_state(uint8_t gpio, uint8_t* level);

#endif