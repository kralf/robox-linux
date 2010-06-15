# Comment/uncomment the following line to enable/disable debugging
DEBUG = n

# Change it here or specify it on the "make" commandline
INCLUDEDIR = /usr/src/linux/include

ifeq ($(DEBUG),y)
  DEBFLAGS = -O -g -DDEBUG 
else
  DEBFLAGS = -O2
endif

CFLAGS = -D__KERNEL__ -DMODULE $(DEBFLAGS)

CFLAGS += -Wall -I.. -I../../h 


CFLAGS += -I$(INCLUDEDIR)

OBJS = icp_drv_gen.o 

%.o : %.c
	$(CC) -c $(CFLAGS) $< -o ../bin/$@

all: $(OBJS)

clean:
	rm -f *.o *.ver *~ ../bin/*.o ../bin/*.ver ../bin/*~

# set MODVERSIONS if the kernel uses it
VERSUSED = $(shell grep 'define CONFIG_MODVERSIONS' \
                   $(INCLUDEDIR)/linux/autoconf.h | wc -l | sed 's/ //g')
ifeq ($(VERSUSED),1)
  MODVERSIONS = y
endif

# and then use it
ifdef MODVERSIONS
export.o import.o: export.ver
endif

export.ver: export.c
	$(CC) -I$(INCLUDEDIR) -E -D__GENKSYMS__ $^ | genksyms > $@

