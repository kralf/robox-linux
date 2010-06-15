################################################################
#
# TIP866 Device Driver Makefile
#
# Copyright (c) 2006 by TEWS-TECHNOLOGIES GmbH
#
################################################################

MOD_PREFIX = tip866

#
#
#

VER := $(shell uname -r)
LINUX_2_6_x := 2.6.
KVERSION_AE_2_6 := $(findstring $(strip $(LINUX_2_6_x)),$(VER))
INCLUDEDIR = /lib/modules/$(VER)/build/include
INSTALL_DIR = /lib/modules/$(VER)/misc

TARGET = $(MOD_PREFIX)drv
DRV_C = $(MOD_PREFIX).c
DRV_PRIV_H = $(MOD_PREFIX)def.h
DRV_PUB_H = 
DI_MODS_SRC = include/*.c include/*.h
#DI_MODS_SRC = include/*.h


ifeq ($(KVERSION_AE_2_6),)
###########################
# Compiling for old kernels
###########################

# Use this for older Kernel Versions before 2.4.4
#INCLUDEDIR = /usr/include

CFLAGS = -D__KERNEL__ -I$(INCLUDEDIR) -Wall -Wstrict-prototypes -O2 -fomit-frame-pointer \
          -pipe -fno-strength-reduce -DMODULE 

OBJS = $(TARGET).o

all:	$(OBJS)

clean:
	rm -f *.o

$(TARGET).o: $(DRV_C) $(DRV_PRIV_H) $(DRV_PUB_H) $(DI_MODS_SRC) Makefile 
	@echo "Compiling for kernels older than 2.6.x..."
	$(CC) $(CFLAGS) -c $< -o $@

install:	$(TARGET).o
	install -d $(INSTALL_DIR)
	install -c $(TARGET).o $(INSTALL_DIR)

uninstall:
	rm -f $(INSTALL_DIR)/$(TARGET).o



else
#############################
# Compiling for 2.6.x kernels
#############################

ifneq ($(KERNELRELEASE),)
obj-m += $(TARGET).o
$(TARGET)-objs := $(MOD_PREFIX).o

else

KDIR:= /lib/modules/$(shell uname -r)/build
PWD:= $(shell pwd)

clean:
	rm -f *.ko
	rm -f *.o
	rm -f *.mod.c
	rm -f .tp*
	rm -f .$(MOD_PREFIX)*
	rm -fr .tmp_versions

install:	$(DRV_C) $(DRV_PRIV_H) $(DRV_PUB_H) $(DI_MODS_SRC) Makefile
	@echo "Compiling for a 2.6.x+ kernel..."
	$(MAKE) -C $(KDIR) SUBDIRS=$(PWD) modules
	install -d $(INSTALL_DIR)
	install -c $(TARGET).ko $(INSTALL_DIR)

uninstall:
	rm -f $(INSTALL_DIR)/$(TARGET).ko

endif

endif

