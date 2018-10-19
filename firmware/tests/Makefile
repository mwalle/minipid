
test: test-config test-pt1000
test-%: test_%
	gtester --verbose $<

test_config: test_config.c mockup.c
	gcc -Iinclude/ $(shell pkg-config --libs --cflags glib-2.0) -o $@ $^

test_pt1000: test_pt1000.c mockup.c
	gcc -Iinclude/ $(shell pkg-config --libs --cflags glib-2.0) -o $@ $^

clean:
	rm -f test_config
