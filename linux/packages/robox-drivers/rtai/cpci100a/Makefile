#-------------------------------------------------------------
# Linux on RoboX project - ASL EPFL
#-------------------------------------------------------------
#
# Makefile for cpci100a linux drivers
# 
#-------------------------------------------------------------
# $Id: Makefile,v 1.2 2003/10/14 07:19:39 fred Exp $
#-------------------------------------------------------------


obj-m		           += cpci100a.o 


modules:
	@cd ${PRJROOT}/kernel/linux-ppc-2.5 && make modules


modules_install:
	        @cd ${PRJROOT}/kernel/linux-ppc-2.5 && make modules_install


clean:
	@find . \( -name '*.[oas]' -o -name '*.ko' -o -name '.*.cmd' -o -name '*~' \
		-o -name '.*.d' -o -name '.*.tmp' -o -name '*.mod.c' \) \
		-type f -print | xargs rm -f

