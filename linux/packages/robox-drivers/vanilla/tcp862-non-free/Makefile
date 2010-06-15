PWD := $(shell pwd)
KERNEL_VER := $(shell uname -r)
KERNEL_INC ?= /usr/src/linux-headers-$(KERNEL_VER)

obj-m := tcp862.o

all: tcp862.c
	@$(MAKE) -C $(KERNEL_INC) SUBDIRS=$(PWD) CFLAGS=$(CFLAGS) M=$(PWD) modules

install: tcp862.ko
	cp -f tcp862.ko /lib/modules/$(KERNEL_VER)/kernel/drivers/serial
	depmod -ae
	install -c -o root -g root -m 644 udev.rules \
	/etc/udev/rules.d/42-tcp862.rules

clean:
	rm -rf *.o *.ko *.mod.c .*.cmd .tmp_versions *~
