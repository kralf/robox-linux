#-------------------------------------------------------------
# Linux on RoboX project - ASL EPFL
#-------------------------------------------------------------
#
# Makefile for ipquad linux drivers
# 
#-------------------------------------------------------------
# $Id: Makefile,v 1.5 2004/02/03 10:15:25 fred Exp $
#-------------------------------------------------------------




include ../../Makefile.inc

2.4:
	${CC} ${KERNEL_CC_FLAGS} ${KERNEL_INC} -DKBUILD_BASENAME=ipquad  -DEXPORT_SYMTAB -c -o ipquad.o ipquad.c

install:
	cp ipquad.o ${MODULE_INSTALL_DIR}


clean:
	rm -rf *.o *~




