include $(CARMEN_HOME)/src/Makefile.conf

SUBDIRS = 

ifeq ($(ROBOT_TYPES),robox) 
	SUBDIRS += robox
	LFLAGS += -L../../posix/config -lconfig -L../../posix/timer -ltimer \
		-L../../posix/thread -lthread -Lroboxlib -lrobox
endif

CFLAGS += 
IFLAGS += -I../robot -I../laser
LFLAGS += -lparam_interface -lglobal -lcarmenserial -lgeometry -lipc
	
MODULE_NAME = BASE
MODULE_COMMENT = Generic interface to all robot bases

SOURCES = 
PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES = 

TARGETS = 

TARGETS += $(ROBOT_TYPES)
PUBLIC_BINARIES += $(ROBOT_TYPES)

# mains for each robot
robox: base_main.o roboxlib/librobox.a

.PHONY: roboxlib/librobox.a

# libraries for each robot
roboxlib/librobox.a: 
	$(MAKE) -C roboxlib 

include $(CARMEN_HOME)/src/Makefile.rules
