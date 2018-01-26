CC=gcc
CFLAGS=-Wall -O2

ifndef DESTDIR
DESTDIR=/usr/local
endif

all: admin.c
	$(CC) $(CFLAGS) admin.c -o pwm-admin
.PHONY: all

clean :
	rm -rf *.o pwm-admin
.PHONY: clean

install: all
	mkdir -p $(DESTDIR)/bin/
	install pwm-admin $(DESTDIR)/bin/
	chown root:root $(DESTDIR)/bin/pwm-admin
	chmod +s $(DESTDIR)/bin/pwm-admin
.PHONY: install

uninstall:
	rm -f $(DESTDIR)/bin/pwm-admin
.PHONY: uninstall
