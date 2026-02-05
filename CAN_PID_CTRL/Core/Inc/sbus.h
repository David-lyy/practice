#ifndef SBUS_H
#define SBUS_H

#include "struct_typedef.h"
extern uint16_t g_sbus_channels[16];
extern uint8_t sbus_data[25];
extern fp32 set_speed[4];
extern void sbus_decode(uint8_t *sbus_buf, uint16_t *channels);
extern void cul_rpm(uint16_t *channels,fp32 *set_speed);

#endif
