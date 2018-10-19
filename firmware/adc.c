#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

#include "adc.h"

static volatile uint16_t __adc_val;
ISR(ADC_vect)
{
	uint16_t val;
	static uint16_t sum = 0;
	static uint8_t count = 64;

	val = ADCL;
	val |= ADCH << 8;

	/* oversampling by 64, thus we have an 13-bit ADC value */
	if (count--) {
		sum += val;
	} else {
		__adc_val = sum >> 3;
		sum = 0;
		count = 64;
	}
}

uint16_t adc_get(void)
{
	uint16_t _adc_val;
	ATOMIC_BLOCK(ATOMIC_FORCEON)
	{
		_adc_val = __adc_val;
	}
	return _adc_val;
}

void adc_init(void)
{
	/*
	 * ADC setup
	 *
	 * - Aref is AVcc
	 * - AIN is ADC3
	 * - 128 prescaler ^= 62.5 kHz
	 * */
	ADMUX = 3;
	DIDR0 = _BV(ADC3D);
	ADCSRA = _BV(ADEN) | _BV(ADATE) | _BV(ADIE) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);

	/* start ADC */
	ADCSRA |= _BV(ADSC);
}
