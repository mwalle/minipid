#ifndef __MOCKUP_H
#define __MOCKUP_H

#include <stdint.h>

char *itoa(int16_t val, char *s, int radix);
void uart_putc(const char c);
void uart_puts(const char *s);

void test_buf_clear(void);
char *test_buf_get(void);

#endif /* __MOCKUP_H */
