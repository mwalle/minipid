#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include "mockup.h"

static char buf[64];
static char *bufptr = buf;

char *itoa(int16_t val, char *s, int radix)
{
	sprintf(s, "%d", val);
	return s;
}

void uart_putc(const char c)
{
	*(bufptr++) = c;
}

void uart_puts(const char *s)
{
	bufptr += sprintf(bufptr, "%s", s);
}

void test_buf_clear(void)
{
	memset(buf, 0, sizeof(buf));
	bufptr = buf;
}

char *test_buf_get(void)
{
	return buf;
}
