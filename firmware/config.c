/*
 * minipid - attiny-based heating controller
 * Copyright (c) 2018, Michael Walle <michael@walle.cc>
 *
 * SPDX-License-Identifier: GPL-2.0
 */

#include <ctype.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>

#include "pid.h"
#include "usi.h"
#include "config.h"

static struct config eep_config EEMEM;
struct config _config, *config = &_config;

static void dump_param_decpoint(char param, int16_t val_fp)
{
	char buf[10];
	int16_t val;
	int32_t t;
	memset(buf, 0, sizeof(buf));

	t = ((int32_t)val_fp * 100) >> 10;
	val = t;

	buf[0] = param;
	itoa(val, buf + 1, 10);
	if (val >= 10000) {
		memmove(buf+5, buf+4, 2);
		buf[4] = '.';
	} else if (val >= 1000) {
		memmove(buf+4, buf+3, 2);
		buf[3] = '.';
	} else if (val >= 100) {
		memmove(buf+3, buf+2, 2);
		buf[2] = '.';
	} else {
		if (val < 10) {
			memmove(buf+4, buf+1, 2);
			buf[3] = '0';
		} else {
			memmove(buf+3, buf+1, 2);
		}
		buf[1] = '0';
		buf[2] = '.';
	}

	uart_puts(buf);
}

static void dump_param(char param, int16_t val)
{
	char buf[8];

	memset(buf, 0, sizeof(buf));
	buf[0] = param;
	itoa(val, buf + 1, 10);
	uart_puts(buf);
}

static void config_init_default(void)
{
	config->version = 1;
	config->kp = 0.75 * (1 << PID_SCALING_SHIFT);
	config->ki = 0.00 * (1 << PID_SCALING_SHIFT);
	config->kd = 0.00 * (1 << PID_SCALING_SHIFT);
	config->i_min = 0;
	config->i_max = 10.0 * (1 << PID_SCALING_SHIFT);
	config->set_point = 1000;
	config->band = 100;
	config->sample_time_ms = 100;
	config->emergency_off = 1600;
	config->auto_off_time = 0;
}

void config_save(void)
{
	eeprom_update_block(config, &eep_config, sizeof(*config));
}

void config_init(void)
{
	eeprom_read_block(config, &eep_config, sizeof(*config));
	if (config->version != 1) {
		config_init_default();
		config_save();
	}
}

void config_dump(void)
{
	dump_param_decpoint('P', config->kp);
	dump_param_decpoint('I', config->ki);
	dump_param_decpoint('D', config->kd);
	dump_param_decpoint('m', config->i_min);
	dump_param_decpoint('M', config->i_max);
	dump_param('S', config->set_point);
	dump_param('B', config->band);
	dump_param('T', config->sample_time_ms);
	dump_param('E', config->emergency_off);
	dump_param('O', config->auto_off_time);
	dump_param('F', config->flags);
	uart_putc('\n');
}

void config_scan_input(const char *str)
{
	uint32_t t;
	int16_t val_fp, val;
	char param = *(str++);

	/* value before decimal point */
	val = atoi(str);
	val_fp = val * 100;

	/* now parse up to two digits after decimal point */
	while (isdigit(*(++str)));
	if (*(str++) == '.') {
		if (isdigit(*str)) {
			val_fp += (*(str++) - '0') * 10;
			if (isdigit(*str))
				val_fp += (*(str++) - '0');
		}
	}

	/* convert to our pid gain values */
	t = (uint32_t)val_fp << PID_SCALING_SHIFT;
	t /= 100;
	val_fp = t;

	switch (param) {
	case 'P':
		/* Kp */
		config->kp = val_fp;
		break;
	case 'I':
		/* Ki */
		config->ki = val_fp;
		break;
	case 'D':
		/* Kd */
		config->kd = val_fp;
		break;
	case 'm':
		/* Imin */
		config->i_min = val_fp;
		break;
	case 'M':
		/* Imax */
		config->i_max = val_fp;
		break;
	case 'S':
		/* set point */
		config->set_point = val;
		break;
	case 'B':
		/* band */
		config->band = val;
		break;
	case 'T':
		/* sample time */
		config->sample_time_ms = val;
		break;
	case 'E':
		/* emergency off temperature threshold */
		config->emergency_off = val;
		break;
	case 'O':
		/* auto off time */
		config->auto_off_time = val;
		break;
	case 'F':
		/* flags */
		config->flags = val;
		break;
	default:
		break;
	}
}
