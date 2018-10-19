#include <avr/io.h>
#include <avr/pgmspace.h>

#include "pt1000.h"

#define ADC_VAL(x) (0x2000UL * (x) / ((x) + 1000))
static const struct {
	const int16_t t;
	const uint16_t v;
} pt1000_tab[] PROGMEM = {
	{0,    ADC_VAL(1000)}, // 2048  -> 0
	{100,  ADC_VAL(1039)},
	{200,  ADC_VAL(1078)},
	{300,  ADC_VAL(1117)},
	{400,  ADC_VAL(1155)},
	{500,  ADC_VAL(1194)},
	{600,  ADC_VAL(1232)},
	{700,  ADC_VAL(1271)},
	{800,  ADC_VAL(1309)},
	{900,  ADC_VAL(1347)},
	{1000, ADC_VAL(1385)},
	{1100, ADC_VAL(1423)},
	{1200, ADC_VAL(1461)},
	{1300, ADC_VAL(1498)},
	{1400, ADC_VAL(1536)},
	{1500, ADC_VAL(1573)},
	{1600, ADC_VAL(1611)},
	{1700, ADC_VAL(1648)}, // 2549 -> 501
};

#define ARRAY_SIZE(a) (sizeof(a)/sizeof(a[0]))
#define PT1000_VMIN (pt1000_tab[0].v)
#define PT1000_VMAX (pt1000_tab[ARRAY_SIZE(pt1000_tab)-1].v)
#define PT1000_TMIN (pt1000_tab[0].t)
#define PT1000_TMAX (pt1000_tab[ARRAY_SIZE(pt1000_tab)-1].t)

int16_t adc2degc(uint16_t val)
{
	uint8_t i = 0;
	uint32_t temp;
	uint16_t t1, t2, v1, v2;

	if (val <= PT1000_VMIN)
		return PT1000_TMIN;
	if (val >= PT1000_VMAX)
		return PT1000_TMAX;

	while ((v2 = pgm_read_word(&(pt1000_tab[i].v))) <= val)
		i++;

	v1 = pgm_read_word(&(pt1000_tab[i-1].v));
	t1 = pgm_read_word(&(pt1000_tab[i-1].t));
	t2 = pgm_read_word(&(pt1000_tab[i].t));

	/* temp = t1 + (t2 - t1) / (v2 - v1) * (val - v1) */
	temp = (t2 - t1);
	temp <<= 16;
	temp /= (v2 - v1);
	temp *= (val - v1);
	temp >>= 16;
	temp += t1;

	return temp;
}
