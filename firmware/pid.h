#ifndef __PID_H
#define __PID_H

#define PID_SCALING_SHIFT 10

void pid_init(int16_t p_gain, int16_t i_gain, int16_t d_gain,
		int32_t i_min, int32_t i_max, int16_t band);
int16_t pid_update(int16_t error, int16_t position);
void pid_reset(void);

#endif /* __PID_H */
