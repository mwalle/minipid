#include <stdio.h>
#include <stdbool.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

#include "usi.h"
#include "adc.h"
#include "config.h"

#define VERSION "0.1"

static volatile uint8_t pwm_steps;
static void pwm_set(uint8_t steps)
{
	pwm_steps = steps;
}

static void pwm_init(void)
{
	pwm_steps = 0;
	PORTA &= _BV(PA3);
	DDRA |= _BV(PA3);
}

static uint16_t uptime;
ISR(TIM1_COMPA_vect)
{
	static uint8_t cycle = 0;

	/* only set the pwm output of the beginning of the cycle */
	if (pwm_steps && cycle == 0)
		PORTA |= _BV(PA3);
	else if (cycle >= pwm_steps)
		PORTA &= ~_BV(PA3);

	if (cycle == 100 || cycle == 200)
		uptime++;

	if (++cycle >= 200)
		cycle = 0;
}

static void timer1_init(void)
{
	/*
	 * Setup Timer1 to 10ms.
	 *
	 * The system clock is 8 MHz, use the 64 prescaler and a counter
	 * value of 1250.
	 *
	 */
	/* CTC mode, prescaler to 64 */
	TCCR1A = 0;
	TCCR1B = _BV(WGM12) | _BV(CS11) | _BV(CS10);
	OCR1AH = (F_CPU / 64 / 100) >> 8;
	OCR1AL = (F_CPU / 64 / 100) & 0xff;

	/* enable timer interrupt */
	TIMSK1 = _BV(OCIE1A);
}

enum {
	ERROR_INVALID_SDA_SCL_STATE = 1,
};

static void blink_errorcode(uint8_t code) __attribute__((noreturn));
static void blink_errorcode(uint8_t code)
{
	int i;
	/* turn output off */
	PORTB &= ~_BV(PB2);

	while (true) {
		/* two short blinks */
		PORTB |= _BV(PB2);
		_delay_ms(25);
		PORTB &= ~_BV(PB2);
		_delay_ms(500);

		/* error code */
		for (i=0; i<code; i++) {
			PORTB |= _BV(PB2);
			_delay_ms(25);
			PORTB &= ~_BV(PB2);
			_delay_ms(200);
		}

		/* 5s pause */
		_delay_ms(5000);
	}
}

static FILE uart_stdout = FDEV_SETUP_STREAM(uart_putc, NULL,
		_FDEV_SETUP_WRITE);

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

int main(void)
{
	bool cli_enabled = false;
	bool display_enabled = false;

	uint16_t last_uptime = UINT16_MAX;
	uint16_t degc;
	int16_t error;
	uint16_t out;

	/* XXX remove me probe pin */
	DDRB |= _BV(PB2);
	PORTB &= ~_BV(PB2);

	/*
	 * We have three cases for PA4 (SCL) and PA6 (SDA/UART_DI).
	 *
	 * If I2C is connected, there will be a pull-up on both lines. If UART is
	 * connected the UART_DI will be high because it is the default line state.
	 *
	 * | PA4  | PA6  | Description                                                |
	 * +------+------+------------------------------------------------------------+
	 * | low  | low  | Nothin is connected. Disable display and cli are disabled. |
	 * | high | high | I2C is connected. Enable display.                          |
	 * | low  | high | UART is connected. Enable cli.                             |
	 * | high | low  | Invalid state. Turn off, display error.                    |
	 */

	switch (PINA & (_BV(PA4) | _BV(PA6))) {
	case 0:
		break;
	case _BV(PA4):
		blink_errorcode(ERROR_INVALID_SDA_SCL_STATE);
		break;
	case _BV(PA6):
		cli_enabled = true;
		break;
	case _BV(PA4) | _BV(PA6):
		display_enabled = true;
		break;
	}

	if (cli_enabled)
		uart_rx_tx_init();
	else
		uart_tx_twi_init();

	display_init();
	pwm_init();
	timer1_init();

	stdout = &uart_stdout;
	sei();

	uart_puts_P(PSTR("minipid v" VERSION "\n"));

	while (true) {
		if (uptime != last_uptime) {
			last_uptime = uptime;
			//printf_P(PSTR("Hello World (%02x) (%d/%d)\r\n"), uart_getc(), cli_enabled, display_enabled);
			display_degc((degc + 5) / 10);
			config_dump();
		}

		degc = adc2degc(adc_get());
	}

	error = config->set_point - degc;
	out = pid_update(error, degc);
	if (out > 200) {
		out = 200;
	} else if (out < 0) {
		out = 0;
	}
	pwm_set(out);

	return 0;
}
