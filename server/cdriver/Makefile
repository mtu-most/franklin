#TARGET ?= bbb

PROFILE = #-pg

CPPFLAGS ?= -g -Wformat -Werror=format-security -D_FORTIFY_SOURCE=2 -Wshadow $(PROFILE)
LDFLAGS ?= $(PROFILE)

all: cdriver

ifeq (${TARGET}, bbb)
ARCH_HEADER = arch-bbb.h
#LIBS += -lprussdrv

#%.bin: %.asm Makefile
#	pasm -V2 -b $<

#cdriver: bbb_pru.bin
else
ARCH_HEADER = arch-avr.h
endif

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
	type-delta.cpp \
	type-polar.cpp

HEADERS = \
	configuration.h \
	cdriver.h \
	${ARCH_HEADER}

CPPFLAGS += -DARCH_INCLUDE=\"${ARCH_HEADER}\"

OBJECTS = $(addprefix build/,$(patsubst %.cpp,%.o,$(SOURCES)))

cdriver: $(OBJECTS) Makefile
	g++ $(LDFLAGS) $(OBJECTS) $(LIBS) -o $@

build/stamp:
	mkdir -p build
	touch $@

build/%.o: %.cpp $(HEADERS) build/stamp Makefile
	g++ $(CPPFLAGS) $(CXXFLAGS) -c $< -o $@

clean:
	rm -rf $(OBJECTS) build cdriver $(DTBO)
