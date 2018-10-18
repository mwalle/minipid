#include <stdio.h>
#include "mockup.h"
#include "../config.c"
#include <glib.h>

#define k (1 << PID_SCALING_SHIFT)

void test_config_dump(void)
{
	test_buf_clear();

	config->kp = 10 * k;
	config->ki = 0.75 * k;
	config->kd = 0.15 * k;
	config->i_min = 10;
	config->i_max = 100;
	config->set_point = 1050;
	config->band = 200;
	config->sample_time_ms = 100;
	config->auto_off_time = 2 * 60 * 60;

	config_dump();
	g_assert_cmpstr(test_buf_get(), ==, "P10.00I0.75D0.14m10M100S1050B200T100O7200\n");
}

void test_config_scan_input_fp(void)
{
	config_scan_input("P15.12");
	g_assert_cmpint(config->kp, ==, 15.12 * k);

	config_scan_input("P15.12\n");
	g_assert_cmpint(config->kp, ==, 15.12 * k);
}

void test_config_scan_input_int(void)
{
	config_scan_input("M10");
	g_assert_cmpint(config->i_max, ==, 10);

	config_scan_input("M15\n");
	g_assert_cmpint(config->i_max, ==, 15);

	config_scan_input("M15.12");
	g_assert_cmpint(config->i_max, ==, 15);
}

int main(int argc, char **argv)
{
	g_test_init (&argc, &argv, NULL);
	g_test_add_func("/config/dump", test_config_dump);
	g_test_add_func("/config/scan_input/fp", test_config_scan_input_fp);
	g_test_add_func("/config/scan_input/int", test_config_scan_input_int);

	return g_test_run();
}
