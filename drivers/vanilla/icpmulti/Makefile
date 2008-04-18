KERNEL_SRC ?= /usr/src/linux-source-`uname -r`

obj-m := icpmulti.o

all: icpmulti.c
	@$(MAKE) -C $(KERNEL_SRC) SUBDIRS=$(PWD) M=$(PWD) modules

clean:
	rm -rf *.o *.ko *~

#-------------------------------------------------------------
# Linux on RoboX project - ASL EPFL
#-------------------------------------------------------------
#
# Makefile for icpmulti linux drivers
#
#-------------------------------------------------------------
# $Id: Makefile,v 1.3 2004/02/03 10:15:16 fred Exp $
#-------------------------------------------------------------


# KERNEL_INC=-I/lib/modules/2.4.27-2-386/build/include/
# CC=gcc-3.3
# KERNEL_CC_FLAGS=-D__KERNEL__ -Wall -Wstrict-prototypes -Wno-trigraphs -O2 -fomit-frame-pointer -fno-strict-aliasing -fno-common -fsigned-char -pipe -Wno-uninitialized -DMODULE -DUSE_LASER
#
# 2.4:
# 	${CC} ${KERNEL_CC_FLAGS} ${KERNEL_INC} -DKBUILD_BASENAME=icpmulti  -DEXPORT_SYMTAB -c -o icpmulti.o icpmulti.c
#
# install:
# 	cp icpmulti.o ${MODULE_INSTALL_DIR}
#
#
# clean:
# 	rm -rf *.o *~




