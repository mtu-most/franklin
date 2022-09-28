MCUS = atmega1284p
FREQS = 16000000 12000000

all:
	for mcu in ${MCUS} ; do \
		for freq in ${FREQS} ; do \
			$(MAKE) -C ../../firmware MCU=$$mcu F_CPU=$$freq ; \
		done \
	done

install:
	for mcu in ${MCUS} ; do \
		for freq in ${FREQS} ; do \
			install -m 644 -D -t ${DESTDIR}/usr/lib/franklin/firmware/ ../../firmware/build/franklin_$$mcu_$$freq.hex ; \
		done \
	done

clean:
	$(MAKE) -C ../../firmware clean

.PHONY: all install clean
