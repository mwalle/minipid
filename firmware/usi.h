#ifndef __USI_H
#define __USI_H

void twi_transfer(uint8_t *data, uint8_t len);
void uart_putc(const char c);
void uart_puts(const char *s);
void uart_puts_P(const char *s);
char uart_getc(void);
void uart_tx_twi_init(void);
void uart_rx_tx_init(void);

#endif /* __USI_H */
