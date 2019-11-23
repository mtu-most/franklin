all:
	$(MAKE) -C ../../firmware TARGET=atmega328p
	$(MAKE) -C ../../firmware TARGET=atmega1284p
	$(MAKE) -C ../../firmware TARGET=atmega1280
	$(MAKE) -C ../../firmware TARGET=atmega2560

install:
	install -m 644 -D -t ${DESTDIR}/usr/lib/franklin/firmware/ ../../firmware/build-mega/atmega1280.hex
	install -m 644 -D -t ${DESTDIR}/usr/lib/franklin/firmware/ ../../firmware/build-mighty_opt/atmega1284p.hex
	install -m 644 -D -t ${DESTDIR}/usr/lib/franklin/firmware/ ../../firmware/build-mega2560/atmega2560.hex
	install -m 644 -D -t ${DESTDIR}/usr/lib/franklin/firmware/ ../../firmware/build-uno/atmega328p.hex

clean:
	$(MAKE) -C ../../firmware TARGET=atmega328p clean
	$(MAKE) -C ../../firmware TARGET=atmega1284p clean
	$(MAKE) -C ../../firmware TARGET=atmega1280 clean
	$(MAKE) -C ../../firmware TARGET=atmega2560 clean

.PHONY: all clean
