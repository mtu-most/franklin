all: setpin

# From dpkg-buildflags.
CFLAGS=-g -O2 -fstack-protector-strong -Wformat -Werror=format-security
CPPFLAGS=-Wdate-time -D_FORTIFY_SOURCE=2
LDFLAGS=-Wl,-z,relro

%: %.c
	gcc $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -Wall -Wextra $< -o $@
