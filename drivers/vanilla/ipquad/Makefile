KERNEL_SRC ?= /usr/src/linux-source-`uname -r`
IPAC_SRC ?= $(PWD)/../cpci100a-non-free

CFLAGS = -I$(IPAC_SRC)

obj-m := ipquad.o

all: ipquad.c
	@$(MAKE) -C $(KERNEL_SRC) SUBDIRS=$(PWD) M=$(PWD) modules

clean:
	rm -rf *.o *.ko *.mod.c .*.cmd .tmp_versions *~
