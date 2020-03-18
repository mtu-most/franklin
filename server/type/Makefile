modules := $(shell cat types.txt)
targets := $(shell sed -E 's/^\s*(.*?)\s*$$/\1\/\1.so/' < types.txt)

all: ${targets} html

html:
	$(MAKE) -C ../html

install:
	install -m 644 -t ${DESTDIR}/usr/lib/franklin/type/ -D types.txt
	for m in ${modules} ; do install -t ${DESTDIR}/usr/lib/franklin/type/$$m/ -D $$m/$$m.so ; done
	for m in ${modules} ; do install -m 644 -t ${DESTDIR}/usr/lib/franklin/type/$$m/ -D $$m/$$m.ini ; done
	for m in ${modules} ; do install -m 644 -t ${DESTDIR}/usr/lib/franklin/html/type/ -D $$m/$$m.js ; done

clean:
	rm -f ${targets}

%.so: %.o Makefile ../cdriver/cdriver.h ../cdriver/franklin-module.h ../cdriver/arch-avr.h ../cdriver/module.h
	g++ -ggdb3 -Wall -Wextra -shared -fPIC $< -o $@
	mkdir -p ../html/type
	ln -sf ../../type/$(patsubst %.o,%.js,$<) ../html/type/

%.o: %.cpp Makefile
	g++ -c -Wall -Wextra -ggdb3 -I../cdriver -DARCH_INCLUDE='"arch-avr.h"' -fPIC $< -o $@
