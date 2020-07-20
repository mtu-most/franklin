# Support {foo,bar} notation.
SHELL=/bin/bash

all:
	$(MAKE) -C cdriver
	$(MAKE) -C type
	$(MAKE) -C firmware
	$(MAKE) -C bb
	$(MAKE) -C opi
	rm cdriver.cpython* || :
	ln -s cdriver/build/lib*/cdriver.cpython* .

install:
	$(MAKE) -C firmware install
	$(MAKE) -C type install
	$(MAKE) -C html install
	$(MAKE) -C bb install
	$(MAKE) -C opi install
	install -t ${DESTDIR}/usr/lib/franklin/ -D cdriver/franklin-cdriver
	install -t ${DESTDIR}/usr/lib/franklin/ -D {server,driver,control}.py
	install -m 644 -t ${DESTDIR}/usr/lib/franklin/ -D protocol.py
	install -t ${DESTDIR}/usr/lib/franklin/ -D control-wrap
	install -t ${DESTDIR}/usr/lib/franklin/ -D cdriver/build/lib.*/cdriver.cpython*
	install -m 644 -t ${DESTDIR}/etc/apache2/conf-available -D franklin.conf

clean:
	$(MAKE) -C cdriver clean
	$(MAKE) -C type clean
	$(MAKE) -C html clean
	$(MAKE) -C firmware clean
	$(MAKE) -C bb clean
	$(MAKE) -C opi clean
	rm cdriver.cpython* || :

.PHONY: all clean
