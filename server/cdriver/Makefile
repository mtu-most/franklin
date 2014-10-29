CPPFLAGS = -g -Wformat -Werror=format-security -D_FORTIFY_SOURCE=2 -Wshadow
SOURCES = debug.cpp globals.cpp gpio.cpp move.cpp packet.cpp serial.cpp setup.cpp space.cpp storage.cpp temp.cpp type-carthesian.cpp type-delta.cpp base.cpp
HEADERS = configuration.h cdriver.h

ifeq (${TARGET}, bbb)
HEADERS += arch-bbb.h
CPPFLAGS += -DARCH_INCLUDE=\"arch-bbb.h\"
else
# Assume avr.
HEADERS += arch-avr.h
CPPFLAGS += -DARCH_INCLUDE=\"arch-avr.h\"
endif

OBJECTS = $(patsubst %.cpp,%.o,$(SOURCES))
cdriver: $(OBJECTS) Makefile
	g++ $(LDFLAGS) $(OBJECTS) -o $@
%.o: %.cpp $(HEADERS) Makefile
	g++ $(CPPFLAGS) $(CXXFLAGS) -c $< -o $@

clean:
	rm -rf build-* $(OBJECTS) cdriver
