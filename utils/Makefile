TARGET = powerpc

XCROOT = ../linux/.xcomp.root/$(TARGET)
CC = $(XCROOT)/bin/$(TARGET)-linux-gcc
CFLAGS += -I../posix/timer -I../posix/serial
LDFLAGS += -L../posix/timer -ltimer -L../posix/serial -lserial

PREFIX = /usr/local

OBJS = robox_serial

all: $(OBJS)

install: $(OBJS)
	install -c -m 755 $(OBJS) $(PREFIX)/bin

clean:
	rm -f *.o $(OBJS)
