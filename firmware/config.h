#ifndef __CONFIG_H
#define __CONFIG_H

struct config {
	uint8_t version;
	int16_t kp;
	int16_t ki;
	int16_t kd;
	int16_t i_min;
	int16_t i_max;
	int16_t set_point;
	int16_t band;
	uint16_t sample_time_ms;
	uint16_t auto_off_time;
} __attribute__((packed));

extern struct config *config;

void config_dump(void);
void config_scan_input(const char *str);

#endif /* __CONFIG_H */