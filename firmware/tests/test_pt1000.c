#include <stdio.h>
#include "mockup.h"
#include "../pt1000.c"
#include <glib.h>

void test_adc2degc(void)
{
	g_assert_cmpint(adc2degc(0), ==, 0);
	g_assert_cmpint(adc2degc(0xffff), ==, 1700);
	g_assert_cmpint(adc2degc(4521), ==, 600);
	g_assert_cmpint(adc2degc(4553), ==, 650);
	g_assert_cmpint(adc2degc(4584), ==, 700);
}

int main(int argc, char **argv)
{
	g_test_init (&argc, &argv, NULL);
	g_test_add_func("/pt1000/adc2degc", test_adc2degc);

	return g_test_run();
}
