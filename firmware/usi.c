#include <stdbool.h>
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <util/atomic.h>

#define BAUD 9600
#define MIN(a, b) (((a) < (b)) ? (a) : (b))

enum {
	USI_IDLE,
	USI_UART_TX1,
	USI_UART_TX2,
	USI_UART_RX,
	USI_TWI,
};

static volatile uint8_t __usi_state;
static volatile uint8_t __uart_tx_char;
static volatile uint8_t __uart_rx_buf[12];
static volatile uint8_t __uart_rx_idx;
static volatile uint8_t __usi_event;
static char uart_rx_dbl_buf[sizeof(__uart_rx_buf)];

static uint8_t swapb(uint8_t b)
{
	b = ((b >> 1) & 0x55) | ((b << 1) & 0xaa);
	b = ((b >> 2) & 0x33) | ((b << 2) & 0xcc);
	b = ((b >> 4) & 0x0f) | ((b << 4) & 0xf0);
	return b;
}

static void usi_aquire(uint8_t new_state)
{
	while (true) {
		ATOMIC_BLOCK(ATOMIC_FORCEON) {
			if (__usi_state == USI_IDLE) {
				__usi_state = new_state;
				return;
			}
		}
	};
}

static void usi_release(void)
{
	__usi_state = USI_IDLE;
}

static void usi_stop(void)
{
	USICR = 0; /* stop USI */
}

#define TIMER0_BIT_CNT ((uint8_t)((F_CPU / 8 / BAUD) + 0.5))

ISR(TIM0_OVF_vect)
{
	TCNT0 += 256 - TIMER0_BIT_CNT;
	PINB |= _BV(PB2);
}

static inline void timer0_start(uint8_t cnt)
{
	TIFR = _BV(TOV0); /* clear pending interrupt */
	TCNT0 = 256 - cnt;
	GTCCR |= _BV(PSR0);
	TCCR0A = 0;
	TCCR0B = _BV(CS01); /* start Timer0, clk/8 prescaler */
}

static inline void timer0_stop(void)
{
	TIFR = _BV(TOV0);
	TCCR0A = 0;
	TCCR0B = 0; /* stop Timer0 */
}

#define USICNT(x) (16 - x)

ISR(PCINT0_vect)
{
	/* If a transmission is in progress, there is nothing we can do. Just
	 * ignore the interrupt. */
	if (__usi_state != USI_IDLE)
		return;

	/* uart start bit detected */
	if (!(PINB & _BV(PB0))) {

		/* Start the counter to hit the middle of the start bit */
		timer0_start(TIMER0_BIT_CNT / 2 - 5);

		/* enable USI in three wire mode and timer0 as clock source */
		USICR = _BV(USIOIE) | _BV(USIWM0) | _BV(USICS0);

		/* Shift in 9 bits, the start bit and 8 data bits, the start bit will
		 * be shifted out again, leaving us with just the 8 data bits. */
		USISR = _BV(USIOIF) | USICNT(9);

		/* mask any further pin change interrupts */
		PCMSK &= ~_BV(PCINT0);

		__usi_state = USI_UART_RX;
	}
}

ISR(USI_OVF_vect)
{
	switch (__usi_state) {
	case USI_UART_TX1:
		/* clear flags and cause overflow after 5 clocks */
		USISR = _BV(USIOIF) | USICNT(5);
		/* second half of the data: 5 data bits, stop bit, two idle bits */
		USIDR = 0x07 | __uart_tx_char << 3;
		__usi_state = USI_UART_TX2;
		break;
	case USI_UART_RX:
		/* last byte of uart_rx_buf is always zero to ensure the string is
		 * always zero terminated */
		__uart_rx_idx = MIN(__uart_rx_idx + 1, sizeof(__uart_rx_buf) - 2);
		__uart_rx_buf[__uart_rx_idx] = swapb(USIDR);
		__usi_event = 1;
		/* reenable pin change interrupt again */
		PCMSK |= _BV(PCINT0);
		/* fall through */
	case USI_UART_TX2:
		/* put DO pin into input mode again */
		PORTB |= _BV(PB1);
		DDRB &= ~_BV(PB1);
		timer0_stop();
		usi_stop();
		usi_release();
		break;
	}
}

void uart_putc(const char c)
{
	if (c == '\n')
		uart_putc('\r');

	usi_aquire(USI_UART_TX1);

	__uart_tx_char = swapb(c);

	/* clear flags and cause overflow after 5 clocks */
	USISR = _BV(USIOIF) | USICNT(5);

	/* enable counter overflow interrupt, three wire mode and timer0
	 * overflow as clock source */
	USICR = _BV(USIOIE) | _BV(USIWM0) | _BV(USICS0);

	/* load first half of the byte to be transmitted */
	USIDR = 0x80 | __uart_tx_char >> 2; /* idle bit, start bit, 6 data bits */

	/* turn output on, should be high, because the msb in USIDR is 1 */
	DDRB |= _BV(PB1);

	/* now start the timer, everything else will be interrupt driven */
	timer0_start(TIMER0_BIT_CNT);
}

void uart_puts(const char *s)
{
	while (*s)
		uart_putc(*(s++));
}

void uart_puts_P(const char *s)
{
	char c;
	while ((c = pgm_read_byte(s++)))
		uart_putc(c);
}

const char *uart_get_buf(void)
{
	ATOMIC_BLOCK(ATOMIC_FORCEON) {
		memcpy(uart_rx_dbl_buf, (void*)__uart_rx_buf, sizeof(uart_rx_dbl_buf));
		__uart_rx_idx = 0;
	}
	return uart_rx_dbl_buf;
}

char uart_getc(void)
{
	char c;

	ATOMIC_BLOCK(ATOMIC_FORCEON) {
		__usi_event = 0;
		c = __uart_rx_buf[__uart_rx_idx];
	}

	return c;
}

void uart_poll(void)
{
	while (!__usi_event);
}

void twi_transfer(uint8_t *data, uint8_t len)
{
	usi_aquire(USI_TWI);

	/* two wire mode, enable counter interrupt, no clock */
	USICR = _BV(USIWM1) | _BV(USICS1);

	/* clear interrupt flags and clear counter */
	USISR = _BV(USIOIF);

	/* make sure USIDR doesn't pull SDA low */
	USIDR = 0xff;

	/* enable ports */
	DDRB |= _BV(PB2); /* SCL */
	DDRB |= _BV(PB0); /* SDA */

	/* start, scl will now held low automatically */
	PORTB &= ~_BV(PB0);

	while (len--) {
		uint8_t n = 7;

		/* first 8 bit of data */
		USIDR = *(data++);

		/* first clock cycle */
		PORTB &= ~_BV(PB2);  /* pull scl low */
		PORTB |= _BV(PB0);   /* release sda */
		USISR = _BV(USISIF); /* release automatic scl lock */
		_delay_us(4);

		PORTB |= _BV(PB2);   /* release scl */
		_delay_us(4);

		while (n--) {
			PORTB &= ~_BV(PB2);
			PINB |= _BV(PB2);
			_delay_us(4);
			PORTB |= _BV(PB2);
			PINB |= _BV(PB2);
			_delay_us(4);
		}

		PORTB &= ~_BV(PB2);
		_delay_us(4);

		/* make sure USIDR doesn't pull SDA low, during ack cycle */
		USIDR = 0xff;

		PORTB |= _BV(PB2);
		_delay_us(4);

		PORTB &= ~_BV(PB2);
		_delay_us(4);
	}

	/* stop */
	PORTB &= ~_BV(PB0);
	_delay_us(4);
	PORTB |= _BV(PB2);
	_delay_us(4);
	PORTB |= _BV(PB0);

	usi_stop();
	usi_release();
}

/*
 * Initialize TX UART and I2C bus
 *
 * This is the default mode for normal operation, where we print the status
 * line on UART and show the temperature on display.
 */
void uart_tx_twi_init(void)
{
	/* PB2 is SCL, PB0 is SDA and PB1 is UART_DO */

	/* DO is input with pull-up if transmitter is disabled.
	 * This is because the UART uses the three wire mode of the USE and we may
	 * shift out low pulses if receiving data. */
	PORTB |= _BV(PB1);

	/* enable overflow interrupt */
	TIMSK |= _BV(TOIE0);

	__usi_state = USI_IDLE;
}

/*
 * Initialize RX and TX UART
 *
 * This mode is for configure the PI controller. The user will be prompted with
 * a tiny CLI.
 */
void uart_rx_tx_init(void)
{
	/* PB0 is UART_DI and PB1 is UART_DO */

	/* DO is input with pull-up if transmitter is disabled.
	 * This is because the UART uses the three wire mode of the USI and we may
	 * shift out low pulses if receiving data. */
	PORTB |= _BV(PB1);

	/* DI is input with pull-up */
	PORTB |= _BV(PB0);

	/* uart rx pin change interrupt */
	PCMSK |= _BV(PCINT0);
	GIMSK |= _BV(PCIE);

	/* enable overflow interrupt */
	TIMSK |= _BV(TOIE0);

	__uart_rx_idx = 0;
	memset((uint8_t*)__uart_rx_buf, 0, sizeof(__uart_rx_buf));
	memset(uart_rx_dbl_buf, 0, sizeof(uart_rx_dbl_buf));

	__usi_event = 0;
	__usi_state = USI_IDLE;
}
