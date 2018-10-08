#include <stdio.h>
#include <stdint.h>
#include "pid.h"

#define PID_SCALING_FACTOR 32

struct pid _pid, *pid = &_pid;

void pid_init(float p_gain, float i_gain, float d_gain,
		float i_min, float i_max, int16_t band)
{
	pid->p_gain = p_gain * PID_SCALING_FACTOR;
	pid->i_gain = i_gain * PID_SCALING_FACTOR;
	pid->d_gain = d_gain * PID_SCALING_FACTOR;
	pid->band = band;

	pid->i_sum = 0;
	pid->d_last = INT16_MIN;

	pid->i_min = i_min;
	pid->i_max = i_max;
	pid->kt = 5;

	pid->max_error = INT16_MAX / (pid->p_gain + 1);
	pid->out_last = 0;
	pid->out_min = 0;
	pid->out_max = 200;
}

void pid_reset(void)
{
	pid->i_sum = 0;
	pid->d_last = INT16_MIN;
	pid->out_last = 0;
}

int16_t pid_update(int16_t error, int16_t position)
{
	int16_t p, d;
	int32_t ret, i;

	if (error < -pid->band)
		return INT16_MIN;
	else if (error > pid->band)
		return INT16_MAX;

	/* TBD limit error to prevent overflow */
	if (error > pid->max_error)
		error = pid->max_error;
	else if (error < -pid->max_error)
		error = -pid->max_error;

	p = pid->p_gain * error;

	/* account error within limits */
	pid->i_sum += error;
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
	if (pid->i_sum > pid->i_max)
		pid->i_sum = pid->i_max;
	else if (pid->i_sum < pid->i_min)
		pid->i_sum = pid->i_min;
#endif

	i = pid->i_gain * pid->i_sum;

	if (pid->d_last != INT16_MIN)
		d = pid->d_gain * (pid->d_last - position);
	else
		d = 0;
	pid->d_last = position;

	ret = (p + i + d) / PID_SCALING_FACTOR;

	//printf("PID %d %f %f %f ret=%ld\r\n", error, (float)p/32, (float)i/32, (float)d/32, ret);
	if (ret > INT16_MAX)
		ret = INT16_MAX;
	else if (ret < INT16_MIN)
		ret = INT16_MIN;

	pid->out_last = ret;
	return ret;
}
