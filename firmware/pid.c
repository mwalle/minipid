#include <stdio.h>
#include <stdint.h>
#include "pid.h"

struct pid {
	int16_t p_gain;
	int16_t i_gain;
	int16_t band;
	//int16_t max_error;

	int16_t i_sum;
	int16_t i_min;
	int16_t i_max;

	//int16_t kt;
	//int16_t out_min;
	//int16_t out_max;
	//int16_t out_last;

#ifdef PID_D
	int16_t d_gain;
	int16_t d_last;
#endif
};

struct pid _pid, *pid = &_pid;

void pid_init(int16_t p_gain, int16_t i_gain, int16_t d_gain,
		int32_t i_min, int32_t i_max, int16_t band)
{
	pid->p_gain = p_gain;
	pid->band = band;
	//pid->max_error = INT16_MAX / (pid->p_gain + 1);

	pid->i_gain = i_gain;
	pid->i_min = i_min;
	pid->i_max = i_max;

	//pid->kt = 5;
	//pid->out_min = 0;
	//pid->out_max = 200;

#ifdef PID_D
	pid->d_gain = d_gain;
#endif

	pid_reset();
}

void pid_reset(void)
{
	pid->i_sum = 0;
	//pid->out_last = 0;
#ifdef PID_D
	pid->d_last = INT16_MIN;
#endif
}

int16_t pid_update(int16_t error, int16_t position)
{
	int32_t ret, p, i, d;

	if (error < -pid->band)
		return INT16_MIN;
	else if (error > pid->band)
		return INT16_MAX;

#if 0
	/* TBD limit error to prevent overflow */
	if (error > pid->max_error)
		error = pid->max_error;
	else if (error < -pid->max_error)
		error = -pid->max_error;
#endif

	p = (int32_t)pid->p_gain * error;

	/* account error within limits */
	i = (int32_t)pid->i_sum + error;
#if 0
	if (pid->out_last > pid->out_max) {
		//printf("remove %d..", pid->i_sum);
		pid->i_sum -= (pid->out_last - pid->out_max) * pid->kt;
		//printf("%d\n", pid->i_sum);
	} else if (pid->out_last < pid->out_min) {
		//printf("add %d..", pid->i_sum);
		pid->i_sum += (pid->out_min - pid->out_last) * pid->kt;
		//printf("%d\n", pid->i_sum);
	}
#else
	if (i > pid->i_max)
		pid->i_sum = pid->i_max;
	else if (i < pid->i_min)
		pid->i_sum = pid->i_min;
	else
		pid->i_sum = i;
#endif
	i = pid->i_gain * pid->i_sum;

#if PID_D
	if (pid->d_last != INT16_MIN)
		d = pid->d_gain * (pid->d_last - position);
	else
		d = 0;
	pid->d_last = position;
#else
	d = 0;
#endif

	ret = (p + i + d) >> PID_SCALING_SHIFT;

	//printf("PID %d %f %f %f ret=%ld\r\n", error, (float)p/32, (float)i/32, (float)d/32, ret);
	if (ret > INT16_MAX)
		ret = INT16_MAX;
	else if (ret < INT16_MIN)
		ret = INT16_MIN;

	//pid->out_last = ret;
	return ret;
}
