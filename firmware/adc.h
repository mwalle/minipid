/*
 * minipid - attiny-based heating controller
 * Copyright (c) 2018, Michael Walle <michael@walle.cc>
 *
 * SPDX-License-Identifier: GPL-2.0
 */

#ifndef __ADC_H
#define __ADC_H

void adc_init(void);
uint16_t adc_get(void);
int16_t adc2degc(uint16_t val);

#define ADC_SHORT (0)
#define ADC_OPEN  ((0x3FFUL * 64) >> 3)

#endif /* __ADC_H */
