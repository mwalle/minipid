/*
 * minipid - attiny-based heating controller
 * Copyright (c) 2018, Michael Walle <michael@walle.cc>
 *
 * SPDX-License-Identifier: GPL-2.0
 */

#ifndef __PT1000_H
#define __PT1000_H

int16_t adc2degc(uint16_t val);
uint16_t degc2adc(int16_t val);

#endif /* __PT1000_H */
