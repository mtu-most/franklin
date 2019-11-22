sources = $(wildcard */*.cpp)
targets = $(patsubst %.cpp,%.so,${sources})

all: ${targets} html

html:
	$(MAKE) -C ../html

clean:
	rm -f ${targets}

%.so: %.o Makefile ../cdriver/cdriver.h ../cdriver/franklin-module.h
	g++ -Wall -Wextra -shared -fPIC $< -o $@
	mkdir -p ../html/type
	ln -sf ../../type/$(patsubst %.o,%.js,$<) ../html/type/

%.o: %.cpp Makefile
	g++ -c -Wall -Wextra -I../cdriver -DARCH_INCLUDE='"arch-avr.h"' -fPIC $< -o $@
