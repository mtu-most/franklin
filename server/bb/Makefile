all: setpin

%: %.c
	gcc $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -Wall -Wextra $< -o $@

clean:
	rm -f setpin

.PHONY: all clean
