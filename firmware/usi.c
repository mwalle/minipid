#include <stdbool.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <util/atomic.h>

#define BAUD 9600

enum {
	USI_IDLE,
	USI_UART_TX1,
	USI_UART_TX2,
	USI_UART_RX,
	USI_TWI,
};

static volatile uint8_t usi_state;
static volatile uint8_t uart_tx_char;
static volatile uint8_t uart_rx_char;

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
			if (usi_state == USI_IDLE) {
				usi_state = new_state;
				return;
			}
		}
	};
}

static void usi_release(void)
{
	usi_state = USI_IDLE;
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
	TIFR0 = _BV(TOV0); /* clear pending interrupt */
	TCNT0 = 256 - cnt;
	GTCCR |= _BV(PSR10);
	TCCR0A = 0;
	TCCR0B = _BV(CS01); /* start Timer0, clk/8 prescaler */
}

static inline void timer0_stop(void)
{
	TIFR0 = _BV(TOV0);
	TCCR0A = 0;
	TCCR0B = 0; /* stop Timer0 */
}

#define USICNT(x) (16 - x)

ISR(PCINT0_vect)
{
	/* If a transmission is in progress, there is nothing we can do. Just
	 * ignore the interrupt. */
	if (usi_state != USI_IDLE)
		return;

	/* uart start bit detected */
	if (!(PINA & _BV(PA6))) {

		/* Start the counter to hit the middle of the start bit */
		timer0_start(TIMER0_BIT_CNT / 2 - 5);

		PORTB |= _BV(PB2); /* XXX remove me */

		/* enable USI in three wire mode and timer0 as clock source */
		USICR = _BV(USIOIE) | _BV(USIWM0) | _BV(USICS0);

		/* Shift in 9 bits, the start bit and 8 data bits, the start bit will
		 * be shifted out again, leaving us with just the 8 data bits. */
		USISR = _BV(USIOIF) | USICNT(9);

		/* mask any further pin change interrupts */
		PCMSK0 &= ~_BV(PCINT6);

		usi_state = USI_UART_RX;
	}
}

ISR(USI_OVF_vect)
{
	switch (usi_state) {
	case USI_UART_TX1:
		/* clear flags and cause overflow after 5 clocks */
		USISR = _BV(USIOIF) | USICNT(5);
		/* second half of the data: 5 data bits, stop bit, two idle bits */
		USIDR = 0x07 | uart_tx_char << 3;
		usi_state = USI_UART_TX2;
		break;
	case USI_UART_RX:
		uart_rx_char = swapb(USIDR);
		/* reenable pin change interrupt again */
		PCMSK0 |= _BV(PCINT6);
		/* fall through */
	case USI_UART_TX2:
		/* put DO pin into input mode again */
		PORTA |= _BV(PA5);
		DDRA &= ~_BV(PA5);
		timer0_stop();
		usi_stop();
		usi_release();
		PORTB &= ~_BV(PB2); /* XXX remove me */
		break;
	}
}

void uart_putc(const char c)
{
	if (c == '\n')
		uart_putc('\r');

	usi_aquire(USI_UART_TX1);

	uart_tx_char = swapb(c);

	/* clear flags and cause overflow after 5 clocks */
	USISR = _BV(USIOIF) | USICNT(5);

	/* enable counter overflow interrupt, three wire mode and timer0
	 * overflow as clock source */
	USICR = _BV(USIOIE) | _BV(USIWM0) | _BV(USICS0);

	/* load first half of the byte to be transmitted */
	USIDR = 0x80 | uart_tx_char >> 2; /* idle bit, start bit, 6 data bits */

	/* turn output on, should be high, because the msb in USIDR is 1 */
	DDRA |= _BV(PA5);

	PORTB &= ~_BV(PB2); /* XXX remove me */

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

char uart_getc(void)
{
	return uart_rx_char;
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
	DDRA |= _BV(PA4); /* SCL */
	DDRA |= _BV(PA6); /* SDA */

	/* start, scl will now held low automatically */
	PORTA &= ~_BV(PA6);

	while (len--) {
		uint8_t n = 7;

		/* first 8 bit of data */
		USIDR = *(data++);

		/* first clock cycle */
		PORTA &= ~_BV(PA4);  /* pull scl low */
		PORTA |= _BV(PA6);   /* release sda */
		USISR = _BV(USISIF); /* release automatic scl lock */
		_delay_us(4);

		PORTA |= _BV(PA4);   /* release scl */
		_delay_us(4);

		while (n--) {
			PORTA &= ~_BV(PA4);
			PINB |= _BV(PB2);
			_delay_us(4);
			PORTA |= _BV(PA4);
			PINB |= _BV(PB2);
			_delay_us(4);
		}

		PORTA &= ~_BV(PA4);
		_delay_us(4);

		/* make sure USIDR doesn't pull SDA low, during ack cycle */
		USIDR = 0xff;

		PORTA |= _BV(PA4);
		_delay_us(4);

		PORTA &= ~_BV(PA4);
		_delay_us(4);
	}

	/* stop */
	PORTA &= ~_BV(PA6);
	_delay_us(4);
	PORTA |= _BV(PA4);
	_delay_us(4);
	PORTA |= _BV(PA6);

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
	/* PA4 is SCL, PA6 is SDA and PA5 is UART_DO */

	/* DO is input with pull-up if transmitter is disabled.
	 * This is because the UART uses the three wire mode of the USE and we may
	 * shift out low pulses if receiving data. */
	PORTA |= _BV(PA5);

	/* enable overflow interrupt */
	TIMSK0 |= _BV(TOIE0);
	usi_state = USI_IDLE;
}

/*
 * Initialize RX and TX UART
 *
 * This mode is for configure the PI controller. The user will be prompted with
 * a tiny CLI.
 */
void uart_rx_tx_init(void)
{
	/* PA6 is UART_DI and PA5 is UART_DO */

	/* DO is input with pull-up if transmitter is disabled.
	 * This is because the UART uses the three wire mode of the USI and we may
	 * shift out low pulses if receiving data. */
	PORTA |= _BV(PA5);

	/* DI is input with pull-up */
	PORTA |= _BV(PA6);

	/* uart rx pin change interrupt */
	PCMSK0 |= _BV(PCINT6);
	GIMSK |= _BV(PCIE0);

	/* enable overflow interrupt */
	TIMSK0 |= _BV(TOIE0);
	usi_state = USI_IDLE;
}
