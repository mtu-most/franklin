all: joystick.py

%.py: %.elf
	./$< > $@

%.elf: %.c
	gcc -Wall -Wextra -Werror $< -o $@
