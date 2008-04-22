##---------------------------------------------------------------------------
## Inova Computers GmbH. Sudetenstr 5, 87600 Kaufbeuren
##
## (c) Copyright 1998-2000. All Rights Reserved
##
## Date: Oct.02.2000
##
## Operating System: Linux
##
## Compiler : Gnu C/C++ Compiler 
##
##---------------------------------------------------------------------------
## Project: ICP-MULTI Linux Driver
## Title:   Makefile
##---------------------------------------------------------------------------

##---------------------------------------------------------------------------
## DESCRIPTION
##---------------------------------------------------------------------------
 
##---------------------------------------------------------------------------
## MODIFICATION HISTORY
##-------------------------------------------------------------------------*/
## 000,28sep00,bt   created

##---------------------------------------------------------------------------
## Variables
##-------------------------------------------------------------------------*/

# Comment/uncomment the following line to enable/disable debugging
DEBUG_EEPROM = n
DEBUG_AI = n
DEBUG_AO = n
DEBUG_DI = n
DEBUG_DO = n
DEBUG_CTR = n
DEBUG_INTR = n
DEBUG_MIO = n
DEBUG_PCI = n

# Change it here or specify it on the "make" commandline
INCLUDEDIR = /usr/src/linux/include

ifeq ($(DEBUG),y)
  DEBFLAGS = -O -g -DDEBUG
else
  DEBFLAGS = -O2
endif

ifeq ($(DEBUG_EEPROM),y)
  DEBFLAGS += -DDEBUG_EEPROM
endif

ifeq ($(DEBUG_AI),y)
  DEBFLAGS += -DDEBUG_AI
endif

ifeq ($(DEBUG_AO),y)
  DEBFLAGS += -DDEBUG_AO
endif

ifeq ($(DEBUG_DI),y)
  DEBFLAGS += -DDEBUG_DI
endif

ifeq ($(DEBUG_DO),y)
  DEBFLAGS += -DDEBUG_DO
endif

ifeq ($(DEBUG_CTR),y)
  DEBFLAGS += -DDEBUG_CTR
endif

ifeq ($(DEBUG_INTR),y)
  DEBFLAGS += -DDEBUG_INTR
endif

ifeq ($(DEBUG_PCI),y)
  DEBFLAGS += -DDEBUG_PCI
endif

ifeq ($(DEBUG_MIO),y)
  DEBFLAGS += -DDEBUG_MIO
endif

# Temporary object files directory
OBJDIR= ../obj

# binary output directory
BINDIR= ../bin
SPACE= 

CFLAGS  = -D__KERNEL__ -DMODULE $(DEBFLAGS)
CFLAGS += -Wall -I../../h
CFLAGS += -I$(INCLUDEDIR)

# Name of the output
OBJS = icp_multi.o

##---------------------------------------------------------------------------
## Rules
##-------------------------------------------------------------------------*/
# 
# dependencies for the output (list of the modules build up the output module)
#
icp_multi.o: icp_mio_drv.o \
	     icp_mio.o     \
	     mio_pci.o     \
	     eeprom.o      \
	     interrupt.o   \
	     analog_in.o   \
	     ai_adc.o      \
	     ai_ch.o       \
	     analog_out.o  \
	     ao_ch.o       \
	     digital_in.o  \
	     digital_out.o \
	     counter.o
	     $(LD) -r $(addprefix $(OBJDIR)/,$^) -o $(BINDIR)/$@

# 
# implicit rule fo C source files
#
%.o : %.c
	$(CC) $(CFLAGS) -c $< -o $(OBJDIR)/$@

##---------------------------------------------------------------------------
## Targets
##-------------------------------------------------------------------------*/
all: $(OBJS)

clean:
	rm -f *.o *.ver *~ $(OBJDIR)/*.o $(OBJDIR)/*.ver $(OBJDIR)/*~ \
	                   $(BINDIR)/*.o $(BINDIR)/*.ver $(BINDIR)/*~


# set MODVERSIONS if the kernel uses it
VERSUSED = $(shell grep 'define CONFIG_MODVERSIONS' \
                   $(INCLUDEDIR)/linux/autoconf.h | wc -l | sed 's/ //g')
ifeq ($(VERSUSED),1)
  MODVERSIONS = y
endif

# and then use it
ifdef MODVERSIONS
export.o import.o: export.ver
endif

export.ver: export.c
	$(CC) -I$(INCLUDEDIR) -E -D__GENKSYMS__ $^ | genksyms > $@

