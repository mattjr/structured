#
#  Brian Curless
#  
#  Computer Graphics Laboratory
#  Stanford University
#  
#  ---------------------------------------------------------------------
#  
#  Copyright (1997) The Board of Trustees of the Leland Stanford Junior
#  University. Except for commercial resale, lease, license or other
#  commercial transactions, permission is hereby given to use, copy,
#  modify this software for academic purposes only.  No part of this
#  software or any derivatives thereof may be used in the production of
#  computer models for resale or for use in a commercial
#  product. STANFORD MAKES NO REPRESENTATIONS OR WARRANTIES OF ANY KIND
#  CONCERNING THIS SOFTWARE.  No support is implied or provided.
#  

ifndef OPTIMIZER
	OPTIMIZER = -O
endif

CSRCS = \

CXXSRCS = \
	plycullmaxx.cc \
	plymerge.cc


TARGETS = \
	plycullmaxx \
	plymerge

# Targets to be built on SGI only (need SGI libraries)...
SGICXXSRCS = matrix2quat.cc
SGITARGETS = matrix2quat

ROOT = ../..

# Get OS uname
UNAME := $(shell uname)

ifeq ($(UNAME), IRIX)

# Set optimizer flags to allow it to run on Indys...
ifeq ($(OPTIMIZER), -O) 
	OPTIMIZER = -O -mips3
endif  #OPTIMIZER

	CXXSRCS := $(CXXSRCS) $(SGICXXSRCS)

	TARGETS := $(TARGETS) $(SGITARGETS)

        CC = cc
        CXX = CC

	CFLAGS = -woff3262 -woff1174 -woff1356
	CXXFLAGS = -woff3262 -woff1174 -woff1356

	INCLS = -I. -I$(ROOT)/include

	LIBPATHS = -L. -L$(ROOT)/lib

	LIBS = -lm -lply

else

	# Set special optimizer flags...
	ifeq ($(OPTIMIZER), -O) 
		OPTIMIZER = -O6 -march=i686 -malign-double -fno-for-scope \
			    -funroll-loops -fomit-frame-pointer -ffast-math
	endif

	CC = gcc
	CXX = g++

	CFLAGS = -DLINUX -Wall -Wno-unused -Wmissing-prototypes \
		-Wmissing-declarations
	CXXFLAGS = -DLINUX -Wall -Wno-unused 

	INCLS = -I. -I$(ROOT)/include

	LIBPATHS = -L. -L$(ROOT)/lib

	LIBS = -lm -lply

endif


CFLAGS := $(INCLS) -DFUNCPROTO -DHAVE_UNISTD_H $(CFLAGS)
CXXFLAGS := $(INCLS) -DFUNCPROTO -DHAVE_UNISTD_H $(CXXFLAGS)
CC := $(CC) $(OPTIMIZER)
CXX := $(CXX) $(OPTIMIZER)



default: all

all:  $(TARGETS)

install: clobber depend all

clobber:
	-/bin/rm -f *.o *~ core makedep $(TARGETS)

matrix2quat: matrix2quat.o
	$(CXX) -o $@ matrix2quat.o $(LIBPATHS) $(LIBS) -lInventor 

plycullmaxx: plycullmaxx.o
	$(CXX) -o $@ plycullmaxx.o $(LIBPATHS) $(LIBS) 

plymerge: plymerge.o
	$(CXX) -o $@ plymerge.o $(LIBPATHS) $(LIBS) 

clean:
	-/bin/rm -f *.o *~ core

depend:
	echo > makedep
	gccmakedep -fmakedep -- $(CFLAGS) -- $(CSRCS) $(CXXSRCS)
	-rm makedep.bak

-include makedep
