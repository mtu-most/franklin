modules := $(shell tr '[:upper:]' '[:lower:]' < types.txt | tr -c '[:alnum:]\n' '[_*]')
targets := $(patsubst %,%/module.so,${modules})

all: ${targets} html

html:
	$(MAKE) -C ../html

install:
	install -m 644 -t ${DESTDIR}/usr/lib/franklin/type/ -D types.txt
	for m in ${modules} ; do install -t ${DESTDIR}/usr/lib/franklin/type/$$m/ -D $$m/$$m.so ; done
	for m in ${modules} ; do install -m 644 -t ${DESTDIR}/usr/lib/franklin/type/$$m/ -D $$m/$$m.ini ; done
	for m in ${modules} ; do install -m 644 -t ${DESTDIR}/usr/lib/franklin/html/type/ -D $$m/$$m.js ; done

clean:
	rm -f ${targets} ../html/type/*.js

%/module.so: js = $(shell echo '$<' | sed -E 's#(.*)/module.o#\1/\1.js#g')
%/module.so: %/module.o Makefile ../cdriver/cdriver.h ../cdriver/franklin-module.h ../cdriver/arch-avr.h ../cdriver/module.h
	g++ -ggdb3 -Wall -Wextra -shared -fPIC $< -o $@
	mkdir -p ../html/type
	test ! -f '${js}' || ln -sf '../../type/${js}' ../html/type/

%/module.o: %/module.cpp Makefile
	g++ -c -Wall -Wextra -ggdb3 -I../cdriver -DARCH_INCLUDE='"arch-avr.h"' -fPIC $< -o $@
