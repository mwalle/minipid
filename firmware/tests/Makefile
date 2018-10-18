
test: test-config
test-config: test_config
	gtester --verbose $<

test_config: test_config.c mockup.c
	gcc -Iinclude/ $(shell pkg-config --libs --cflags glib-2.0) -o $@ $^

clean:
	rm -f test_config
