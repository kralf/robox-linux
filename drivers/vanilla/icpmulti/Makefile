PWD := $(shell pwd)
KERNEL_SRC ?= /usr/src/linux-source-`uname -r`

obj-m := icpmulti.o

all: icpmulti.c
	@$(MAKE) -C $(KERNEL_SRC) SUBDIRS=$(PWD) M=$(PWD) modules

clean:
	rm -rf *.o *.ko *.mod.c .*.cmd .tmp_versions *~
