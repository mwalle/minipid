#ifndef __CONFIG_H
#define __CONFIG_H

#define FLAGS_DEBUG_PID _BV(0)

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
	int16_t emergency_off;
	uint16_t auto_off_time;
	uint8_t flags;
} __attribute__((packed));

extern struct config *config;

void config_init(void);
void config_save(void);
void config_dump(void);
void config_scan_input(const char *str);

#endif /* __CONFIG_H */
