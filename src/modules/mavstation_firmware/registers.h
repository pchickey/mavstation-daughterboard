
#ifndef __MAVSTATION_FIRMWARE_REGISTERS_H__
#define __MAVSTATION_FIRMWARE_REGISTERS_H__

#include <stdint.h>

#include "protocol.h"

/**
 * Register space
 */
void registers_set(uint8_t page, uint8_t offset, const uint16_t *values, unsigned num_values);
int  registers_get(uint8_t page, uint8_t offset, volatile uint16_t **values, unsigned *num_values);

/*
 * Registers.
 */
extern uint16_t			r_page_status[];	/* PX4IO_PAGE_STATUS */
extern uint16_t			r_page_servos[];	/* PX4IO_PAGE_SERVOS */
extern uint16_t			r_page_adc[];		/* PX4IO_PAGE_RAW_ADC_INPUT */

extern volatile uint16_t	r_page_setup[];		/* PX4IO_PAGE_SETUP */

/*
 * Register aliases.
 *
 * Handy aliases for registers that are widely used.
 */
#define r_status_flags		r_page_status[PX4IO_P_STATUS_FLAGS]
#define r_status_alarms		r_page_status[PX4IO_P_STATUS_ALARMS]

#define r_setup_features	r_page_setup[PX4IO_P_SETUP_FEATURES]
#define r_setup_arming		r_page_setup[PX4IO_P_SETUP_ARMING]
#define r_setup_pwm_rates	r_page_setup[PX4IO_P_SETUP_PWM_RATES]
#define r_setup_pwm_defaultrate	r_page_setup[PX4IO_P_SETUP_PWM_DEFAULTRATE]
#define r_setup_pwm_altrate	r_page_setup[PX4IO_P_SETUP_PWM_ALTRATE]
#define r_setup_relays		r_page_setup[PX4IO_P_SETUP_RELAYS]

#define r_control_values	(&r_page_controls[0])

#endif // __MAVSTATION_FIRMWARE_REGISTERS_H__
