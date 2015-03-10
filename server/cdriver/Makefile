ifeq (${TARGET}, bbb)
ARCH_HEADER = arch-bbb.h
else
ARCH_HEADER = arch-avr.h
endif

CPPFLAGS = -g -Wformat -Werror=format-security -D_FORTIFY_SOURCE=2 -Wshadow

SOURCES = \
	base.cpp \
	debug.cpp \
	globals.cpp \
	gpio.cpp \
	hostserial.cpp \
	move.cpp \
	packet.cpp \
	run.cpp \
	serial.cpp \
	setup.cpp \
	space.cpp \
	storage.cpp \
	temp.cpp \
	type-cartesian.cpp \
	type-delta.cpp

HEADERS = \
	configuration.h \
	cdriver.h \
	${ARCH_HEADER}

CPPFLAGS += -DARCH_INCLUDE=\"${ARCH_HEADER}\"

OBJECTS = $(patsubst %.cpp,%.o,$(SOURCES))

cdriver: $(OBJECTS) Makefile
	g++ $(LDFLAGS) $(OBJECTS) -o $@

%.o: %.cpp $(HEADERS) Makefile
	g++ $(CPPFLAGS) $(CXXFLAGS) -c $< -o $@

clean:
	rm -rf $(OBJECTS) cdriver
