NAME=wiimote
SYM=wiimote

# If you want to use a customized Pd, then define a $PD_PATH variable.
# Otherwise, the Pd must be installed on the system
PD_PATH=/usr/lib/pd

# if AudioScape is installed, include the following file:
#include ../../../makefile.include

# specify the path to CWiid:
#CWIID_PATH = $(ASCAPE_PATH)/usr/lib
CWIID_PATH = ../libcwiid


######################################################
# You shouldn't need to change anything beyond here! #
######################################################


ifdef PD_PATH
PD_INCLUDE := -I$(PD_PATH)/src
PD_EXTRA_PATH := $(PD_PATH)/extra
PD_DOC_PATH := $(PD_PATH)/doc/5.reference
else
PD_INCLUDE := -I/usr/local/include
PD_EXTRA_PATH := /usr/local/lib/pd/extra
PD_DOC_PATH := /usr/local/lib/pd/doc/5.reference
endif

		
LIBS = $(CWIID_PATH)/libcwiid.a -lcwiid -lbluetooth -lpthread

current: pd_linux

##### LINUX:

pd_linux: $(NAME).pd_linux

.SUFFIXES: .pd_linux

LINUXCFLAGS = -DPD -O2 -funroll-loops -fomit-frame-pointer \
    -W -Wshadow -Wstrict-prototypes -fPIC \
    -Wno-unused -Wno-parentheses -Wno-switch

.c.pd_linux:
	cc $(LINUXCFLAGS) $(PD_INCLUDE) -o $*.o -c $*.c
	ld -export_dynamic -shared -o $*.pd_linux $*.o $(LIBS) -lc -lm
	strip --strip-unneeded $*.pd_linux 
	rm -f $*.o
 
install:

ifdef ASCAPE_INSTALLED
	-cp doc/*help*.pd $(ASCAPE_PATH)/ss_engine/pd/help/.
ifeq ($(findstring Linux,$(ASCAPE_OS)),Linux)
	-cp *.pd_linux $(ASCAPE_PATH)/ss_engine/pd/externs/$(ASCAPE_OS)$(ASCAPE_ARCH)/.
endif
ifeq ($(findstring Darwin,$(SS_OS)),Darwin)
	-cp *.pd_darwin $(ASCAPE_PATH)/ss_engine/pd/externs/$(ASCAPE_OS)$(ASCAPE_ARCH)/.
endif
endif
	
	-cp *.pd_linux $(PD_EXTRA_PATH)/.
	-cp doc/*help*.pd $(PD_DOC_PATH)/.

clean:
	-rm -f *.o *.pd_* so_locations
