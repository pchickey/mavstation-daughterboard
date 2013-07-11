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

/**
 * @file mavstation.c
 * Top-level logic for the mavstation module.
 */

#include <nuttx/config.h>

#include <stdio.h>	// required for task_create
#include <stdbool.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <poll.h>
#include <signal.h>

#include <drivers/drv_pwm_output.h>
#include <drivers/drv_hrt.h>

#include <systemlib/perf_counter.h>

#include <stm32_uart.h>

#define DEBUG
#include "mavstation.h"
#include "protocol.h"
#include "registers.h"
#include "sysstate.h"
#include "i2c.h"
#include "gpio.h"

__EXPORT int user_start(int argc, char *argv[]);

extern void up_cxxinitialize(void);

struct sys_state_s system_state;

static struct hrt_call serial_dma_call;

/*
 * a set of debug buffers to allow us to send debug information from ISRs
 */

static volatile uint32_t msg_counter;
static volatile uint32_t last_msg_counter;
static volatile uint8_t msg_next_out, msg_next_in;

/*
 * WARNING: too large buffers here consume the memory required
 * for mixer handling. Do not allocate more than 80 bytes for
 * output.
 */
#define NUM_MSG 2
static char msg[NUM_MSG][40];

/*
 * add a debug message to be printed on the console
 */
void
isr_debug(uint8_t level, const char *fmt, ...)
{
	if (level > r_page_setup[PX4IO_P_SETUP_SET_DEBUG]) {
		return;
	}
	va_list ap;
	va_start(ap, fmt);
	vsnprintf(msg[msg_next_in], sizeof(msg[0]), fmt, ap);
	va_end(ap);
	msg_next_in = (msg_next_in+1) % NUM_MSG;
	msg_counter++;
}

/*
 * show all pending debug messages
 */
static void
show_debug_messages(void)
{
	if (msg_counter != last_msg_counter) {
		uint32_t n = msg_counter - last_msg_counter;
		if (n > NUM_MSG) n = NUM_MSG;
		last_msg_counter = msg_counter;
		while (n--) {
			debug("%s", msg[msg_next_out]);
			msg_next_out = (msg_next_out+1) % NUM_MSG;
		}
	}
}

int
user_start(int argc, char *argv[])
{
	/* run C++ ctors before we go any further */
	up_cxxinitialize();

	/* reset all to zero */
	memset(&system_state, 0, sizeof(system_state));

	/* configure the high-resolution time/callout interface */
	hrt_init();

	/*
	 * Poll at 1ms intervals for received bytes that have not triggered
	 * a DMA event.
	 */
#ifdef CONFIG_ARCH_DMA
	hrt_call_every(&serial_dma_call, 1000, 1000, (hrt_callout)stm32_serial_dma_poll, NULL);
#endif

	/* print some startup info */
	lowsyslog("\nmavstation: starting\n");

	/* default all the LEDs to off while we start */
	LED_AMBER(false);
	LED_BLUE(false);
	LED_SAFETY(false);

	/* configure the first 8 PWM outputs (i.e. all of them) */
	up_pwm_servo_init(0xff);

	/* start the i2c slave interface */
	i2c_interface_init();

	/* start gpio interface */
	gpio_interface_init();
	/* pass usart2 to raspberry pi by default */
	gpio_interface_setusart2mux(true);

	/* add a performance counter for the interface */
	perf_counter_t interface_perf = perf_alloc(PC_ELAPSED, "interface");

	/* and one for measuring the loop rate */
	perf_counter_t loop_perf = perf_alloc(PC_INTERVAL, "loop");

	struct mallinfo minfo = mallinfo();
	lowsyslog("MEM: free %u, largest %u\n", minfo.mxordblk, minfo.fordblks);

	/*
	 * Run everything in a tight loop.
	 */

	uint64_t last_debug_time = 0;
	for (;;) {

		/* track the rate at which the loop is running */
		perf_count(loop_perf);

		/* kick the interface */
		perf_begin(interface_perf);
		i2c_interface_tick();
		gpio_interface_tick();

#ifdef DEBUG_GPIOS
		for (int i = 0; i < 5; i++) {
			gpio_interface_setled((i%3), gpio_interface_getbtn(i));
		}
#endif

		perf_end(interface_perf);

		/* check for debug activity */
		show_debug_messages();

		/* post debug state at ~1Hz */
		if (hrt_absolute_time() - last_debug_time > (1000 * 1000)) {

			struct mallinfo minfoloop = mallinfo();

			isr_debug(1, "d:%u s=0x%x f=0x%x m=%u", 
				  (unsigned)r_page_setup[PX4IO_P_SETUP_SET_DEBUG],
				  (unsigned)r_status_flags,
				  (unsigned)r_setup_features,
				  (unsigned)minfoloop.mxordblk);
			last_debug_time = hrt_absolute_time();
		}
	}
}

