MCUS = atmega1284p atmega328p atmega168

all:
	for mcu in ${MCUS} ; do \
		$(MAKE) -C ../../firmware MCU=$$mcu ; \
		ln -s ../../firmware/franklin_$${mcu}_16000000.hex . || : ; \
	done
	$(MAKE) -C ../../firmware MCU=atmega1284p F_CPU=12000000 ; \
	ln -s ../../firmware/franklin_atmega1284p_12000000.hex . || : ; \

install:
	for mcu in ${MCUS} ; do \
		for freq in ${FREQS} ; do \
			install -m 644 -D -t ${DESTDIR}/usr/lib/franklin/firmware/ ../../firmware/build/franklin_$$mcu_$$freq.hex ; \
		done \
	done

clean:
	$(MAKE) -C ../../firmware clean
	rm *.hex || :

.PHONY: all install clean
