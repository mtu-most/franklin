check:

expressions.js: index.html make_check
	./make_check < $< > $@

check: expressions.js
	gjslint --disable 5,110 server.js main.js
	gjslint --disable 1,5,110 expressions.js

clean:
	rm -f expressions.js
