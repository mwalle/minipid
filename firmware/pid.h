/*
 * minipid - attiny-based heating controller
 * Copyright (c) 2018, Michael Walle <michael@walle.cc>
 *
 * SPDX-License-Identifier: GPL-2.0
 */

#ifndef __PID_H
#define __PID_H

#define PID_SCALING_SHIFT 10

void pid_init(void);
int16_t pid_update(int16_t error, int16_t position);
void pid_reset(void);

#endif /* __PID_H */
