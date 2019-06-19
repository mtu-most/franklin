all:
	$(MAKE) -C ../../firmware TARGET=atmega328p
	$(MAKE) -C ../../firmware TARGET=atmega1284p
	$(MAKE) -C ../../firmware TARGET=atmega1280
	$(MAKE) -C ../../firmware TARGET=atmega2560

clean:
	$(MAKE) -C ../../firmware TARGET=atmega328p clean
	$(MAKE) -C ../../firmware TARGET=atmega1284p clean
	$(MAKE) -C ../../firmware TARGET=atmega1280 clean
	$(MAKE) -C ../../firmware TARGET=atmega2560 clean

.PHONY: all clean
