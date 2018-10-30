#include <stdio.h>
#include "mockup.h"
#include "../pt1000.c"
#include <glib.h>

void test_adc2degc(void)
{
	g_assert_cmpint(adc2degc(0), ==, 0);
	g_assert_cmpint(adc2degc(UINT16_MAX), ==, 1700);
	g_assert_cmpint(adc2degc(4521), ==, 600);
	g_assert_cmpint(adc2degc(4553), ==, 650);
	g_assert_cmpint(adc2degc(4584), ==, 700);
}

void test_degc2adc(void)
{
	g_assert_cmpint(degc2adc(INT16_MIN), ==, 4096);
	g_assert_cmpint(degc2adc(INT16_MAX), ==, 5098);
	g_assert_cmpint(degc2adc(600), ==, 4521);
	g_assert_cmpint(degc2adc(650), ==, 4552);
	g_assert_cmpint(degc2adc(700), ==, 4584);
}

int main(int argc, char **argv)
{
	g_test_init (&argc, &argv, NULL);
	g_test_add_func("/pt1000/adc2degc", test_adc2degc);
	g_test_add_func("/pt1000/degc2adc", test_degc2adc);

	return g_test_run();
}
