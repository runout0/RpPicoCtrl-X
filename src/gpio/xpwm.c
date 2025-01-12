/*****************************************************************************/
/**
* @file xpwm.c
*
* Ctrl-X PWM/Servo driver.
*
* GNU GENERAL PUBLIC LICENSE:
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program. If not, see <http://www.gnu.org/licenses/>.
*
* Errors and commissions should be reported to runout@gmx.de
*
* <pre>
* MODIFICATION HISTORY:
*
* Ver   Who  Date        Changes
* ----- ---  --------    -----------------------------------------------
* 1.00  tt   31/07/2023  First release
*
* </pre>
******************************************************************************/

#include "pico/stdlib.h"
#include "xpwm.h"
#include "xgpio.h"

/* Calculation see https://cocode.se/linux/raspberry/pwm.html
 * 
 * fsys = 125MHz; 1/fsys = 8ns
 * Divider = 255 -> 490.196kHz; 1/fsysdiv255 = 2,04µs
 * counter 2exp16-1 = 65535; 2,04µs * 65535 = 0,1336914s; 1/t = 7,48Hz
 * Wrap 20ms = 20ms/2,04µs = 9.803,92 Val-Counter
 * Limit_min = -90° = 1ms = 5% Fullrange = 9804*0,05 = 490,2
 * Limit_max = +90° = 2ms = 10% Fullrange = 9804*0,1 = 980,4
 * Neutral = 735,3
 **/

static uint16_t pwmset;

void on_pwm_wrap() {

	// Clear the interrupt flag that brought us here
	pwm_clear_irq(pwm_gpio_to_slice_num(XGPIO_PWM));

	// Note this range matches with the wrap value
	pwm_set_gpio_level(XGPIO_PWM, pwmset);
}

xpwm_error_t XPWM_Init(xpwm_t *xpwm, uint8_t gpio)
{
	pwmset = XPWM_NEUTRAL;
	xpwm->set_val = pwmset;

	gpio_set_function(XGPIO_PWM, GPIO_FUNC_PWM);
	xpwm->pwm_slice = pwm_gpio_to_slice_num(gpio);

	// Mask our slice's IRQ output into the PWM block's single interrupt line,
	// and register our interrupt handler
	pwm_clear_irq(xpwm->pwm_slice);
	pwm_set_irq_enabled(xpwm->pwm_slice, true);
	irq_set_exclusive_handler(PWM_IRQ_WRAP, on_pwm_wrap);
	irq_set_enabled(PWM_IRQ_WRAP, true);

	// counter is allowed to wrap over its maximum range (0 to 2**16-1)
	pwm_config config = pwm_get_default_config();
	pwm_config_set_clkdiv(&config, XPWM_DIV);
	pwm_config_set_wrap(&config, XPWM_WRAP);
	xpwm->IsInitialized = true;
	
	// Load the configuration into our PWM slice, and set it running.
	pwm_init(xpwm->pwm_slice, &config, true);
	xpwm->IsEnabled = true;
	return XPWM_NO_ERROR;
}

xpwm_error_t XPWM_SetVal(xpwm_t *xpwm, uint16_t min, uint16_t max, uint16_t set)
{
	pwmset = set;
	xpwm->set_val = set;
	return XPWM_NO_ERROR;
}
