#!/usr/sbin/make

#
#  Homan Igehy
#
#  Modified by: Brian Curless
#  
#  Computer Graphics Laboratory
#  Stanford University
#  
#  ---------------------------------------------------------------------
#  
#  Copyright (1997-2001) The Board of Trustees of the Leland Stanford
#  Junior University. Except for commercial resale, lease, license or
#  other commercial transactions, permission is hereby given to use,
#  copy, modify this software for academic purposes only.  No part of
#  this software or any derivatives thereof may be used in the
#  production of computer models for resale or for use in a commercial
#  product. STANFORD MAKES NO REPRESENTATIONS OR WARRANTIES OF ANY
#  KIND CONCERNING THIS SOFTWARE.  No support is implied or provided.
#  

ifndef OPTIMIZER
	OPTIMIZER = -O
endif

ROOT = ../..

# Get OS uname
UNAME := $(shell uname)

ifeq ($(UNAME), IRIX)

        # Set optimizer flags to allow it to run on Indys...
        ifeq ($(OPTIMIZER), -O) 
	    OPTIMIZER = -O -mips3
        endif  #OPTIMIZER

	CPP = CC

	CCFLAGS = -woff3262

	INCLS = -I. -I$(ROOT)/include

	LIBPATHS = -L. -L$(ROOT)/lib

	LIBS =	-lply -llinear -ltclrl -lreadline -ltermcap \
		-ltk -ltcl -lsl -lX11 -lc -lm

	ASFLAGS = -mips2 -O2

else

	# Set special optimizer flags...
	ifeq ($(OPTIMIZER),-O) 
		OPTIMIZER = -O6 -march=i686 -malign-double -fno-for-scope \
			    -funroll-loops -fomit-frame-pointer -ffast-math
	endif

	CPP = g++

	CCFLAGS = -DLINUX -Wno-unused 

	INCLS = -I. -I$(ROOT)/include

	LIBPATHS = -L. -L$(ROOT)/lib

endif

CCFLAGS := $(INCLS) -DFUNCPROTO -DHAVE_UNISTD_H $(CCFLAGS)
CPP := $(CPP) $(OPTIMIZER)

SHELL = /bin/sh
AS = as
AR = ar
AWK = awk
TR2CL = trunc_to_ceil.awk

HACK =	-DInterpolate_$$Interp -DConstant_$$Const \
	-DSampleCalc_$$Calc    -DFrameBuffer_$$Frame \


TRI_OBJS = \
	sl_triangle_ZI_None_Dda_Intensity.o 

TRI_FILES = makefile sl_triangle.C sl_common.H sl_common.I sl_hack.H \
	sl_vertex.H sl_val.H sl_framebuffer.H sl_framebuffer.I \
	sl_texture.H sl_texture.I

SL_OBJS = $(TRI_OBJS)
SL_LIB = libsl.a

default: $(SL_LIB)

all: $(SL_LIB)

install: clobber depend all

$(SL_LIB): makefile $(SL_OBJS)
	$(AR) ru $@ $(SL_OBJS)

$(TRI_OBJS): $(TRI_FILES)
	Interp=`echo $@|$(AWK) -F_ '{print $$3}'`;\
	Const=`echo $@|$(AWK) -F_ '{print $$4}'`;\
	Calc=`echo $@|$(AWK) -F_ '{print $$5}'`;\
	Frame=`echo $@|$(AWK) -F_ '{print $$6}'|$(AWK) -F. '{print $$1}'`;\
	File=`echo $@ | $(AWK) -F_ '{print $$1"_"$$2}'`;\
	$(CPP) $(HACK) $(CCFLAGS) -c $$File.C -o $@

clean:
	-/bin/rm -f  *~ *.o *.s *.S core 

clobber:
	-/bin/rm -f *~ *.o *.s *.S makedep core curtest core Makedepend \
	mon.out libsl.a

depend:
	echo > makedep
	gccmakedep -fmakedep -- $(CFLAGS) -- $(CSRCS)
	-rm makedep.bak

-include makedep

# DO NOT DELETE
