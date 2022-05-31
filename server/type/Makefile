TARGET_ARCH ?= avr
modules := $(shell tr '[:upper:]' '[:lower:]' < types.txt | tr -c '[:alnum:]\n' '[_*]')
targets := $(patsubst %,%/module.so,${modules})

all: ${targets} html

html:
	$(MAKE) -C ../html

install:
	install -m 644 -t ${DESTDIR}/usr/lib/franklin/type/ -D types.txt
	for m in ${modules} ; do install -t ${DESTDIR}/usr/lib/franklin/type/$$m/ -D $$m/module.so ; done
	for m in ${modules} ; do install -m 644 -t ${DESTDIR}/usr/lib/franklin/type/$$m/ -D $$m/$$m.ini ; done
	for m in ${modules} ; do if [ -f $$m/$$m.js ] ; then install -m 644 -t ${DESTDIR}/usr/lib/franklin/html/type/ -D $$m/$$m.js ; fi ; done

clean:
	rm -f ${targets} ../html/type/*.js

%/module.so: js = $(shell echo '$<' | sed -E 's#(.*)/module.o#\1/\1.js#g')
%/module.so: %/module.o Makefile ../cdriver/cdriver.h ../cdriver/franklin-module.h ../../arch/${TARGET_ARCH}/arch-host.h ../cdriver/module/module.h
	g++ -ggdb3 -Wall -Wextra -shared -fPIC -flinker-output=dyn $< -o $@
	mkdir -p ../html/type
	test ! -f '${js}' || ln -sf '../../type/${js}' ../html/type/

%/module.o: %/module.cpp Makefile
	g++ -c -Wall -Wextra -ggdb3 -I../cdriver -I../../arch/${TARGET_ARCH} -fPIC $< -o $@
