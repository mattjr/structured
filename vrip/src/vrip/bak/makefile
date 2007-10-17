#! gmake

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

CSRCS = 

CXXSRCS = vripInit.cc vripMain.cc vripMiscCmds.cc OccGrid.cc \
	OccGridRLE.cc plyio.cc DepthMap.cc Mesh.cc softRenderGeom.cc \
	ChunkAllocator.cc ChunkLink.cc DepthMapAux.cc \
	vripFillCmds.cc vripFileCmds.cc vripGridCmds.cc vripRangeCmds.cc \
	vripGUICmds.cc vripGlobals.cc vripAux.cc BBox3f.cc rangePly.cc \
	linePersp.cc perspective.cc scan.cc scanRLE.cc scanLinePerspRLE.cc \
	configure.cc occFunc.cc resample.cc OccGridNormRLE.cc scanNormRLE.cc \
	vripPlyCmds.cc scanPerspRLE.cc

TARGETS = vrip vriphead

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

	CFLAGS = -woff3262 -woff1174 -woff1356
	CXXFLAGS = -woff3262 -woff1174 -woff1356

	INCLS = -I. -I$(ROOT)/include -I$(ROOT)/include/tcl8.3

	LIBPATHS = -L. -L$(ROOT)/lib -L$(ROOT)/lib/irix

	LIBS =	-lply -llinear  -ltermcap -ltk8.3 -ltcl8.3 \
		-lsl -lX11 -lc -lm
# Lucas: removed -ltclrl -lreadline 2-4-00

else

	# Set special optimizer flags...
	ifeq ($(OPTIMIZER), -O) 
		OPTIMIZER = -O6 -march=i686 -malign-double -fno-for-scope \
			    -funroll-loops -fomit-frame-pointer -ffast-math
	endif

	CC = gcc
	CXX = g++

	CFLAGS = -DLINUX -Wno-unused
	CXXFLAGS = -DLINUX -Wno-unused

	INCLS = -I. -I$(ROOT)/include

	LIBPATHS = -L. -L$(ROOT)/lib -L$(ROOT)/lib/linux -L/usr/X11R6/lib

	LIBS = -lply -llinear -ltk -ltcl -ldl \
		-lsl -lXext -lX11 -lc -lm 

endif

SRCS = $(CXXSRCS) $(CSRCS)

OBJS = $(CXXSRCS:.cc=.o) $(CSRCS:.c=.o)

CFLAGS := $(INCLS) -DFUNCPROTO -DHAVE_UNISTD_H $(CFLAGS)
CXXFLAGS := $(INCLS) -DFUNCPROTO -DHAVE_UNISTD_H $(CXXFLAGS)
CC := $(CC) $(OPTIMIZER)
CXX := $(CXX) $(OPTIMIZER)

default: all

all:	$(TARGETS)

install: clobber depend all

vrip: $(OBJS)
	$(CXX) -o $@ $(OBJS) $(LIBPATHS) $(LIBS)

vriphead: vriphead.o
	$(CXX) -o $@ vriphead.o $(LIBPATHS) $(LIBS)

cysiltoply: cysiltoply.o strings.o
	$(CXX) -o $@ cysiltoply.o strings.o $(LIBPATHS) -limg -limage $(LIBS)

gltest: gltest.o
	$(CXX) -o $@ gltest.o -lgl

gltest2: gltest2.o
	$(CXX) -o $@ gltest2.o -lgl

clean:
	-/bin/rm -f *.o *~ core

clobber:
	-/bin/rm -f *.o *~ core makedep $(TARGETS)

depend:
	echo > makedep
	gccmakedep -fmakedep -- $(CFLAGS) -- $(SRCS)
	-/bin/rm makedep.bak

-include makedep

# DO NOT DELETE
