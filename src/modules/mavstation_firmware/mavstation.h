/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __MAVSTATION_FIRMWARE_MAVSTATION_H__
#define __MAVSTATION_FIRMWARE_MAVSTATION_H__

/**
 * @file mavstation.h
 *
 * General defines and structures for the mavstation module firmware.
 */

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>

#include <drivers/boards/mavstation/mavstation_internal.h>

#define MAVSTA_SERVO_COUNT		8

/*
 * Debug logging
 */

#ifdef DEBUG
# include <debug.h>
# define debug(fmt, args...)	lowsyslog(fmt "\n", ##args)
#else
# define debug(fmt, args...)	do {} while(0)
#endif

/*
 * GPIO handling.
 */
#define LED_BLUE(_s)		stm32_gpiowrite(GPIO_LED1, !(_s))
#define LED_AMBER(_s)		stm32_gpiowrite(GPIO_LED2, !(_s))
#define LED_SAFETY(_s)		stm32_gpiowrite(GPIO_LED3, !(_s))

/** global debug level for isr_debug() */
extern volatile uint8_t debug_level;

/** send a debug message to the console */
extern void	isr_debug(uint8_t level, const char *fmt, ...);

#endif // __MAVSTATION_FIRMWARE_MAVSTATION_H__
