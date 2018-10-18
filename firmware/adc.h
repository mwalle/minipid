#ifndef __ADC_H
#define __ADC_H

void adc_init(void);
uint16_t adc_get(void);
int16_t adc2degc(uint16_t val);

#endif /* __ADC_H */
