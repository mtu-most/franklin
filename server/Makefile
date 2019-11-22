all:
	$(MAKE) -C cdriver
	$(MAKE) -C type
	$(MAKE) -C firmware
	$(MAKE) -C bb
	rm cdriver.cpython* || :
	ln -s cdriver/build/lib*/cdriver.cpython* .

clean:
	$(MAKE) -C cdriver clean
	$(MAKE) -C type clean
	$(MAKE) -C firmware clean
	$(MAKE) -C bb clean
	rm cdriver.cpython* || :

.PHONY: all clean
