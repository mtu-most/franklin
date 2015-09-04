all: joystick.py c457a-ui.gui

%.py: %.elf
	./$< > $@

%.elf: %.c
	gcc -Wall -Wextra -Werror $< -o $@

%.gui: %.gui.in
	xmlgen <$< >$@

clean:
	rm -rf __pycache__
