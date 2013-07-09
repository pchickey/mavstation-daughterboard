
#ifndef __MAVSTATION_FIRMWARE_GPIO_H__
#define __MAVSTATION_FIRMWARE_GPIO_H__

#include <stdbool.h>

void gpio_interface_init(void);
void gpio_interface_tick(void);
void gpio_interface_setled(int,bool);
bool gpio_interface_getbtn(int);

#endif // __MAVSTATION_FIRMWARE_GPIO_H__

