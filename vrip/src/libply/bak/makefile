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

# PLY polygon file routines

ifndef OPTIMIZER
	OPTIMIZER = -O
endif

CSRCS = plyfile.c plytest.c

TARGETS = libply.a plytest

ROOT = ../..

# Get OS uname
UNAME := $(shell uname)

ifeq ($(UNAME), IRIX)

        # Set optimizer flags to allow it to run on Indys...

        ifeq ($(OPTIMIZER), -O) 
	    OPTIMIZER = -O -mips3
        endif  #OPTIMIZER

	CC = cc
	CXX = CC

	CFLAGS = -woff3262 -woff1174
	CXXFLAGS = -woff3262 -woff1174

	INCLS = -I. 

	LIBPATHS = -L.

else

	# Set special optimizer flags...
	ifeq ($(OPTIMIZER),-O) 
		OPTIMIZER = -O6 -march=i686 -malign-double \
			    -funroll-loops -fomit-frame-pointer -ffast-math
	endif

	CC = gcc
	CXX = g++

	CFLAGS = -DLINUX -Wno-unused
	CXXFLAGS = -DLINUX -Wno-unused

	INCLS = -I.

	LIBPATHS = -L.

endif

CFLAGS := $(INCLS) -DFUNCPROTO -DHAVE_UNISTD_H $(CFLAGS)
CC := $(CC) $(OPTIMIZER)
CXX := $(CXX) $(OPTIMIZER)
default: all

all: $(TARGETS)

install: clobber depend all

libply.a: plyfile.o
	ar -r libply.a plyfile.o

plytest: plytest.o
	$(CC) -o $@ plytest.o $(LIBPATHS) -lply

clean: 
	-/bin/rm -f *~ *.o core

clobber:
	-/bin/rm -f *~ *.o core makedep $(TARGETS)

depend:
	echo > makedep
	gccmakedep -fmakedep -- $(CFLAGS) -- $(CSRCS)
	rm makedep.bak

-include makedep
