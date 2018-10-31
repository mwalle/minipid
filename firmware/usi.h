/*
 * minipid - attiny-based heating controller
 * Copyright (c) 2018, Michael Walle <michael@walle.cc>
 *
 * SPDX-License-Identifier: GPL-2.0
 */

#ifndef __USI_H
#define __USI_H

void twi_transfer(uint8_t *data, uint8_t len);
void uart_putc(const char c);
void uart_puts(const char *s);
void uart_puts_P(const char *s);
char uart_poll(void);
char uart_getc(void);
const char *uart_get_buf(void);
void uart_tx_twi_init(void);
void uart_rx_tx_init(void);

#endif /* __USI_H */
