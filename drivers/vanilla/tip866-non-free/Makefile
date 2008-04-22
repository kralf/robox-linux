KERNEL_SRC ?= /usr/src/linux-source-`uname -r`

obj-m := ipac.o cpci100a.o tip866.o

all: ipac.c cpci100a.c tip866.c
	@$(MAKE) -C $(KERNEL_SRC) SUBDIRS=$(PWD) M=$(PWD) modules

clean:
	rm -rf *.o *.ko *~
