all:
	$(MAKE) -C ../../firmware TARGET=atmega328p
	$(MAKE) -C ../../firmware TARGET=atmega1284p
	$(MAKE) -C ../../firmware TARGET=atmega1284p-12MHz
	$(MAKE) -C ../../firmware TARGET=atmega1280
	#$(MAKE) -C ../../firmware TARGET=atmega2560

install:
	install -m 644 -D -t ${DESTDIR}/usr/lib/franklin/firmware/ ../../firmware/build-atmega328p/atmega328p.hex
	install -m 644 -D -t ${DESTDIR}/usr/lib/franklin/firmware/ ../../firmware/build-atmega2560/atmega2560.hex
	install -m 644 -D -t ${DESTDIR}/usr/lib/franklin/firmware/ ../../firmware/build-atmega1284p/atmega1284p.hex
	install -m 644 -D -t ${DESTDIR}/usr/lib/franklin/firmware/ ../../firmware/build-atmega1284p-12MHz/atmega1284p.hex
	#install -m 644 -D -t ${DESTDIR}/usr/lib/franklin/firmware/ ../../firmware/build-mega2560/atmega2560.hex

clean:
	$(MAKE) -C ../../firmware TARGET=atmega328p clean
	$(MAKE) -C ../../firmware TARGET=atmega1284p clean
	$(MAKE) -C ../../firmware TARGET=atmega1284p-12MHz clean
	$(MAKE) -C ../../firmware TARGET=atmega1280 clean
	#$(MAKE) -C ../../firmware TARGET=atmega2560 clean

.PHONY: all install clean
