#ifndef __USI_H
#define __USI_H

void twi_transfer(uint8_t *data, uint8_t len);
void uart_putc(char c);
char uart_getc(void);
void uart_tx_twi_init(void);
void uart_rx_tx_init(void);

#endif /* __USI_H */
