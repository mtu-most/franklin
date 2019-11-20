sources = $(wildcard */*.cpp)
targets = $(patsubst %.cpp,%.so,${sources})

all: ${targets} html

html:
	$(MAKE) -C ../html

clean:
	rm -f ${targets}

%.so: %.o Makefile
	g++ -Wall -Wextra -shared -fPIC $< -o $@
	ln -sf ../../type/$(patsubst %.o,%.js,$<) ../html/type/

%.o: %.cpp Makefile
	g++ -c -Wall -Wextra -I../cdriver -DARCH_INCLUDE='"arch-avr.h"' -fPIC $< -o $@
