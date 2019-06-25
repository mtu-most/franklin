all:
	$(MAKE) -C cdriver
	$(MAKE) -C html
	$(MAKE) -C firmware
	$(MAKE) -C bb
	rm cdriver.cpython* || :
	ln -s cdriver/build/lib*/cdriver.cpython* .

clean:
	$(MAKE) -C cdriver clean
	$(MAKE) -C html clean
	$(MAKE) -C firmware clean
	$(MAKE) -C bb clean
	rm cdriver.cpython* || :

.PHONY: all clean
