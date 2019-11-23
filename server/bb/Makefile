all: setpin

%: %.c
	gcc $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -Wall -Wextra $< -o $@

install: setpin
	install -m 644 -t ${DESTDIR}/usr/lib/franklin/bb/ -D avrdude.conf
	install -t ${DESTDIR}/usr/lib/franklin/bb/ -D flash-bb-0 flash-bb-4
	install -s -t ${DESTDIR}/usr/lib/franklin/bb/ -D setpin

clean:
	rm -f setpin

.PHONY: all clean
