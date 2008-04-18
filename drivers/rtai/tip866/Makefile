#-------------------------------------------------------------
# Linux on RoboX project - ASL EPFL
#-------------------------------------------------------------
#
# Makefile for tip866 linux drivers
# 
#-------------------------------------------------------------
# $Id: Makefile,v 1.12 2004/02/03 10:15:39 fred Exp $
#-------------------------------------------------------------



include ../../Makefile.inc

2.4:
	${CC} ${KERNEL_CC_FLAGS} ${KERNEL_INC} -DKBUILD_BASENAME=tip866  -DEXPORT_SYMTAB -c -o tip866.o tip866.c

install:
	cp tip866.o ${MODULE_INSTALL_DIR}


clean:
	rm -rf *.o *~





