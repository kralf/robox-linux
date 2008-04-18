#-------------------------------------------------------------
# Linux on RoboX project - ASL EPFL
#-------------------------------------------------------------
#
# Makefile for icpmulti linux drivers
# 
#-------------------------------------------------------------
# $Id: Makefile,v 1.3 2004/02/03 10:15:16 fred Exp $
#-------------------------------------------------------------



include ../../Makefile.inc

2.4:
	${CC} ${KERNEL_CC_FLAGS} ${KERNEL_INC} -DKBUILD_BASENAME=icpmulti  -DEXPORT_SYMTAB -c -o icpmulti.o icpmulti.c

install:
	cp icpmulti.o ${MODULE_INSTALL_DIR}


clean:
	rm -rf *.o *~




