PWD := $(shell pwd)
KERNEL_SRC ?= /usr/src/linux-source-`uname -r`
IPAC_SRC ?= $(PWD)/../cpci100a-non-free

CFLAGS = -I$(IPAC_SRC)

obj-m := tip866.o

all: tip866.c
	@$(MAKE) -C $(KERNEL_SRC) SUBDIRS=$(PWD) CFLAGS=$(CFLAGS) M=$(PWD) modules

clean:
	rm -rf *.o *.ko *.mod.c .*.cmd .tmp_versions *~
