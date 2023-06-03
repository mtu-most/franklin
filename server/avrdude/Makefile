all: setpin

%: %.c
	gcc $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -Wall -Wextra $< -o $@

install: setpin
	install -m 644 -t ${DESTDIR}/usr/lib/franklin/avrdude/ -D avrdude.conf
	install -t ${DESTDIR}/usr/lib/franklin/avrdude/ -D flash-bb-0 flash-bb-4 flash-opi
	install -s -t ${DESTDIR}/usr/lib/franklin/avrdude/ -D setpin

clean:
	rm -f setpin

.PHONY: all install clean
