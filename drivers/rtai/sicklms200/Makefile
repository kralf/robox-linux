#-------------------------------------------------------------
# Linux on RoboX project - ASL EPFL
#-------------------------------------------------------------
#
# Makefile for sicklms200 linux drivers
# 
#-------------------------------------------------------------
# $Id: Makefile,v 1.4 2004/02/03 10:15:32 fred Exp $
#-------------------------------------------------------------


include ../../Makefile.inc

2.4:
	${CC} ${KERNEL_CC_FLAGS} ${KERNEL_INC} -DKBUILD_BASENAME=sicklms200  -DEXPORT_SYMTAB -c -o sicklms200.o sicklms200.c

install:
	cp sicklms200.o ${MODULE_INSTALL_DIR}


clean:
	rm -rf *.o *~


