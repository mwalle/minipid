/*
 * minipid - attiny-based heating controller
 * Copyright (c) 2018, Michael Walle <michael@walle.cc>
 *
 * SPDX-License-Identifier: GPL-2.0
 */

#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/atomic.h>
#include <util/delay.h>

#include "usi.h"
#include "adc.h"
#include "pt1000.h"
#include "pid.h"
#include "config.h"

#define VERSION "0.1"
#ifndef FORCE_CLI_ENABLED
#define FORCE_CLI_ENABLED 0
#endif

static volatile uint8_t __pwm_steps;
static void pwm_set(uint8_t steps)
{
	__pwm_steps = steps;
}

static void pwm_init(void)
{
	__pwm_steps = 0;
	PORTB &= _BV(PB4);
	DDRB |= _BV(PB4);
}

static void pwm_off(void)
{
	__pwm_steps = 0;
	PORTB &= ~_BV(PB4);
}

static volatile uint16_t __uptime;
static volatile uint32_t __millis;

uint16_t uptime(void)
{
	uint16_t _uptime;
	ATOMIC_BLOCK(ATOMIC_FORCEON) {
		_uptime = __uptime;
	}
	return _uptime;
}

uint32_t millis(void)
{
	uint32_t _millis;
	ATOMIC_BLOCK(ATOMIC_FORCEON) {
		_millis = __millis;
	}
	return _millis;
}

ISR(TIM1_COMPA_vect)
{
	static uint8_t cycle = 0;

	/* only set the pwm output of the beginning of the cycle */
	if (__pwm_steps && cycle == 0)
		PORTB |= _BV(PB4);
	else if (cycle >= __pwm_steps)
		PORTB &= ~_BV(PB4);

	__millis += 10;

	if (cycle == 99 || cycle == 199)
		__uptime++;

	if (++cycle >= 200)
		cycle = 0;
}

static void timer1_init(void)
{
	/*
	 * Setup Timer1 to 10ms (actually 9.98ms).
	 *
	 * The system clock is 8 MHz, use the 512 prescaler and a counter
	 * value of 156.
	 *
	 */
	/* CTC mode, prescaler to 512 */
	TCCR1 = _BV(CTC1) | _BV(CS13) | _BV(CS11);
	OCR1A = (F_CPU / 512 / 100);
	OCR1C = (F_CPU / 512 / 100);

	/* enable timer interrupt */
	TIMSK |= _BV(OCIE1A);
}

static void timer1_stop(void)
{
	TCCR1 = 0;
}

enum {
	ERROR_INVALID_SDA_SCL_STATE = 1,
	ERROR_SENSOR_UNCONNECTED = 2,
	ERROR_SENSOR_SHORTED = 3,
	ERROR_EMERGENCY_OFF = 4,
};

static void reset(void) __attribute__((noreturn));
static void reset(void)
{
	WDTCR = _BV(WDE);
	while (true);
}

static void configuration_mode(void)
{
	const char *buf;
	char c;

	/* turn output off */
	pwm_off();

	uart_puts_P(PSTR("Configuration mode, output disabled.\n"));

	while (true) {
again:
		uart_get_buf();
		uart_puts_P(PSTR("config> "));
		while (true) {
			uart_poll();
			c = uart_getc();

			if (c == '\r') {
				uart_putc('\n');
				break;
			}

			if (c == '\b' || c == 127) {
				uart_puts("Backspace not supported. Try again.\n");
				goto again;
			}

			/* echo back */
			uart_putc(c);
		}

		buf = uart_get_buf();
		if (buf[0] == '.') {
			config_scan_input(buf + 1);
			config_dump();
		} else if (buf[0] == '\r') {
			continue;
		} else if (!strncmp_P(buf, PSTR("reset"), 5)) {
			reset();
		} else if (!strncmp_P(buf, PSTR("dump"), 4)) {
			config_dump();
		} else if (!strncmp_P(buf, PSTR("save"), 4)) {
			uart_puts_P(PSTR("Saving config to EEPROM.\n"));
			config_save();
		} else if (!strncmp_P(buf, PSTR("exit"), 4)) {
			uart_puts_P(PSTR("Exitting config mode.\n"));
			return;
		} else if (!strncmp_P(buf, PSTR("help"), 4)) {
			uart_puts_P(PSTR(
						"Available commands:\n"
						".<P> <V> - set parameter <P> to value <V>\n"
						"dump  - show config\n"
						"save  - show config\n"
						"reset - reset controller\n"
						"exit  - show config\n"));
		} else {
			uart_puts_P(PSTR("Unknown command, try 'help'.\n"));
		}
	}
}

static void blink_errorcode(uint8_t code) __attribute__((noreturn));
static void blink_errorcode(uint8_t code)
{
	char buf[8];
	int i;

	/* turn output off */
	timer1_stop();
	PORTB &= ~_BV(PB4);

	while (true) {
		if (uart_getc() == '\r')
			configuration_mode();

		/* output error on UART */
		uart_puts_P(PSTR("ERR"));
		uart_puts(itoa(code, buf, 10));
		uart_puts_P(PSTR("\n"));

		/* two short blinks */
		for (i = 0; i < 2; i++) {
			PORTB |= _BV(PB4);
			_delay_ms(50);
			PORTB &= ~_BV(PB4);
			_delay_ms(300);
		}

		_delay_ms(500);
		/* error code */
		for (i = 0; i < code; i++) {
			PORTB |= _BV(PB4);
			_delay_ms(100);
			PORTB &= ~_BV(PB4);
			_delay_ms(500);
		}

		/* 10s pause */
		_delay_ms(10000);
	}
}

static void display_init(void)
{
	uint8_t buf;

	/* dummy transfer */
	buf = 0x00;
	twi_transfer(&buf, 1);

	/* display on, max brightness */
	buf = 0xf1;
	twi_transfer(&buf, 1);
}

static uint8_t display_digit(uint8_t i)
{
	static const uint8_t digits[] PROGMEM = {
		0b11111100,    // 0
		0b01100000,    // 1
		0b11011010,    // 2
		0b11110010,    // 3
		0b01100110,    // 4
		0b10110110,    // 5
		0b10111110,    // 6
		0b11100000,    // 7
		0b11111110,    // 8
		0b11110110,    // 9
		0b11101110,    // A
		0b00111110,    // b
		0b10011100,    // C
		0b01111010,    // d
		0b10011110,    // E
		0b10001110,    // F
		0b00000000,    // all off
		0b11000110,    // deg
	};

	return pgm_read_byte(&(digits[i]));
}

static void display_errorcode(uint8_t code)
{
	static uint8_t buf[5];

	buf[0] = 0x02;
	twi_transfer(buf, 1);

	buf[0] = '\x03';
	buf[1] = display_digit(14);
	buf[2] = display_digit(0);
	buf[3] = display_digit(0);
	buf[4] = display_digit(code % 10);

	twi_transfer(buf, 5);
}

static void display_degc(uint8_t val)
{
	static uint8_t buf[5];

	buf[0] = 0x02;
	twi_transfer(buf, 1);

	buf[0] = '\x03';
	if (val < 100)
		buf[1] = display_digit(16);
	else
		buf[1] = display_digit((val / 100) % 10);
	if (val < 10)
		buf[2] = display_digit(16);
	else
		buf[2] = display_digit((val / 10) % 10);
	buf[3] = display_digit(val % 10);
	buf[4] = display_digit(17);

	twi_transfer(buf, 5);
}

static void print_statusline(int16_t degc, int16_t pwm, bool auto_off)
{
	char buf[8];
	uart_puts(itoa(uptime(), buf, 10));
	uart_puts_P(PSTR(" "));
	uart_puts(itoa(degc / 10, buf, 10));
	uart_puts_P(PSTR("."));
	uart_puts(itoa(degc % 10, buf, 10));
	uart_puts_P(PSTR(" "));
	uart_puts(itoa(pwm, buf, 10));
	uart_puts_P(PSTR(" "));
	if (auto_off)
		uart_puts_P(PSTR("auto-off "));
	uart_puts_P(PSTR("\n"));
}

int main(void)
{
	bool cli_enabled = false;
	bool display_enabled = false;
	bool auto_off = false;

	uint16_t _uptime;
	uint32_t _millis;
	uint16_t last_uptime = UINT16_MAX;
	uint32_t next_sample_time = 0;
	uint16_t adc_val;
	int16_t degc;
	int16_t error;
	int16_t out;
	uint16_t emergency_off_adc_val;
	uint8_t errorcode;

	/* clear watchdog */
	MCUSR = 0;
	WDTCR |= _BV(WDCE) | _BV(WDE);
	WDTCR = 0;

	config_init();
	emergency_off_adc_val = degc2adc(config->emergency_off);

	/*
	 * We have three cases for PB2 (SCL) and PB0 (SDA/UART_DI).
	 *
	 * If I2C is connected, there will be a pull-up on both lines. If UART is
	 * connected the UART_DI will be high because it is the default line state.
	 *
	 * | PB2  | PB0  | Description                                                |
	 * +------+------+------------------------------------------------------------+
	 * | low  | low  | Nothin is connected. Disable display and cli are disabled. |
	 * | high | high | I2C is connected. Enable display.                          |
	 * | low  | high | UART is connected. Enable cli.                             |
	 * | high | low  | Invalid state. Turn off, display error.                    |
	 */

	switch (PINB & (_BV(PB2) | _BV(PB0))) {
	case 0:
		break;
	case _BV(PB2):
		blink_errorcode(ERROR_INVALID_SDA_SCL_STATE);
		break;
	case _BV(PB0):
		cli_enabled = true;
		break;
	case _BV(PB2) | _BV(PB0):
		display_enabled = true;
		break;
	}

	if (FORCE_CLI_ENABLED)
		cli_enabled = true;

	if (cli_enabled)
		uart_rx_tx_init();
	else
		uart_tx_twi_init();

	adc_init();
	if (display_enabled)
		display_init();
	pwm_init();
	timer1_init();
	pid_init();

	sei();

	uart_puts_P(PSTR("minipid v" VERSION "\n"));

	if (cli_enabled) {
		uart_puts_P(PSTR("UART detected. Press return to enter configuration mode.\n"));
	}

	while (true) {
		adc_val = adc_get();
		if (adc_val == ADC_OPEN) {
			errorcode = ERROR_SENSOR_UNCONNECTED;
		} else if (adc_val == ADC_SHORT) {
			errorcode = ERROR_SENSOR_SHORTED;
		} else if (adc_val >= emergency_off_adc_val) {
			errorcode = ERROR_EMERGENCY_OFF;
		} else {
			errorcode = 0;
		}

		if (errorcode) {
			if (display_enabled)
				display_errorcode(errorcode);
			blink_errorcode(errorcode);
		}
		degc = adc2degc(adc_val);

		_uptime = uptime();
		if (_uptime != last_uptime) {
			last_uptime = _uptime;

			if (display_enabled) {
				display_degc(degc / 10);
			}
			print_statusline(degc, out, auto_off);
		}

		if (uart_getc() == '\r') {
			configuration_mode();
			auto_off = false;
			emergency_off_adc_val = degc2adc(config->emergency_off);
			pid_init();
		}

		_millis = millis();
		if (_millis > next_sample_time) {
			next_sample_time = _millis + config->sample_time_ms;

			error = config->set_point - degc;
			out = pid_update(error, degc);
			if (out > 200) {
				out = 200;
			} else if (out < 0) {
				out = 0;
			}
			if (auto_off)
				out = 0;
			pwm_set(out);
		}

		if (config->auto_off_time && _uptime >= config->auto_off_time)
			auto_off = true;
	}

	return 0;
}
