#ifndef __PID_H
#define __PID_H

struct pid {
	int16_t p_gain;
	int16_t i_gain;
	int16_t d_gain;
	int16_t band;

	int32_t i_sum;
	int16_t d_last;
	int16_t kt;

	int32_t i_min;
	int32_t i_max;
	int16_t max_error;

	int16_t out_min;
	int16_t out_max;
	int16_t out_last;
};

extern struct pid *pid;

void pid_init(float p_gain, float i_gain, float d_gain,
		float i_min, float i_max, int16_t band);
int16_t pid_update(int16_t error, int16_t position);
void pid_reset(void);

#endif /* __PID_H */
