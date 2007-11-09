# Skeletal GNUmakefile (derived from Palomino) that autogenerates dependencies
# using makedepend and uses separate source and object directories.

#
OS = $(shell uname)

ifeq ($(strip $(OS)),Linux) 
  PLATFORM=linux-gpp
endif

ifeq ($(strip $(OS)),CYGWIN_NT-5.1) 
  PLATFORM=cygwin-gpp
endif

ifeq ($(strip $(OS)),Darwin)
  PLATFORM=darwin-gpp
endif

DEBUG_FLAGS=1
AR = ar
ARLIST = ranlib

ifeq ($(strip $(PLATFORM)),darwin-gpp)
INC_FLAGS = -I/sw/include
LIBS= -L/sw/lib  -framework osg  -framework osgDB  -framework OpenThreads -framework osgUtil -framework vecLib -L/usr/local/lib/ -lkeypoint -framework AGL -framework OpenGL -framework osgGA -framework Carbon 

else
INC_FLAGS= -I/usr/local/include/
LIBS=-losg -losgDB -losgGA -losgViewer -losgText -lOpenThreads -lGL -lGLU  -losgUtil
endif



INC_FLAGS   += `pkg-config --cflags opencv` `pkg-config --cflags libpng` `pkg-config --cflags glib-2.0` -I/usr/local/include/libflounder -I/usr/local/include/libplankton -I/usr/local/include/libadt  -I/usr/local/include/libsnapper  -I/usr/local/include/libpolyp

LIBS  += `pkg-config --libs opencv`  `pkg-config --libs libpng` `pkg-config --libs glib-2.0`   -lgts -l3ds -l3dObjects -lplankton -lsnapper -L. -ladt -lboost_thread -lpolyp


CC_SRCS     =	OSGExport.cpp\
		PixelBufferCarbon.cpp\
		GraphicsWindowCarbon.cpp\
		mcd.cpp


HH_SRCS = 	

OUT_DIR     = objs
SRC_DIR     = .
SRC_EXT     = .cpp

CC          = g++
#CC_FLAGS    = -O9 -mtune=i686 -march=i686 -fexpensive-optimizations  $(CYGWINFLAGS) -Wall -Wno-missing-braces
CC_FLAGS    = -g

LINK	=	g++
#LFLAGS	=  -O9 -mtune=i686 -march=i686 -fexpensive-optimizations
LFLAGS	=  -g

ifdef DEBUG_FLAGS
CC_FLAGS    = -g  $(CYGWINFLAGS) -Wall -Wno-missing-braces #-D_MCD_CHECK
LFLAGS	=  -g
endif
TARGET = threadedStereo

# Order dependent.
CC_OBJS         := $(addprefix $(OUT_DIR)/,$(CC_SRCS:$(SRC_EXT)=.o))
CC_SRCS         := $(addprefix $(SRC_DIR)/,$(CC_SRCS))
HH_SRCS         := $(addprefix $(SRC_DIR)/,$(HH_SRCS))
LIB = libquickstereo.a
SUBDIRS = vrip
# myvrip
# EStereo

all:  threadedStereo genTex subdirs

.PHONY: subdirs $(SUBDIRS)

subdirs: $(SUBDIRS)

$(SUBDIRS):
	$(MAKE) -C $@



$(LIB): $(SUBDIRS) $(CC_OBJS)
	$(AR) rvu $@ $(CC_OBJS)
	$(ARLIST) $@




clean:
	rm -f $(TARGET) $(OUT_DIR)/*.o $(OUT_DIR)/deps *~ matlab/*~ $(LIB)

# Build an object file from every source file.
$(OUT_DIR)/%.o: %$(SRC_EXT)
	$(CC)  -c -o  $@  $(CC_FLAGS) $(INC_FLAGS)  $(subst $(OUT_DIR)/,$(SRC_DIR)/,$<)



threadedStereo: $(LIB) $(OUT_DIR)/threadedStereo.o
	$(LINK) $(LFLAGS) -o threadedStereo $(OUT_DIR)/threadedStereo.o $(LIB) $(LIBS)
genTex:  $(LIB) $(OUT_DIR)/genTex.o
	$(LINK) $(LFLAGS) -o genTex $(OUT_DIR)/genTex.o $(LIB) $(LIBS)

lodgen:  $(LIB) $(OUT_DIR)/lodgen.o
	$(LINK) $(LFLAGS) -o lodgen $(OUT_DIR)/lodgen.o $(LIB) $(LIBS)
# Skip depend rule if clean (-include would otherwise make it).
# Pass -I flags (INC_FLAGS) to makedepend or it will omit those directories.
# -include depends on $(OUT_DIR)/deps which make will make.
ifeq (,$(findstring clean,$(MAKECMDGOALS)))
depend $(OUT_DIR)/deps:  $(CC_SRCS)  $(HH_SRCS)
	@mkdir -p $(OUT_DIR)
	makedepend $(INC_FLAGS) -Y -f- $(CC_SRCS) 2>/dev/null | sed 's/$(SRC_DIR)\//$(OUT_DIR)\//' > $(OUT_DIR)/deps
-include $(OUT_DIR)/deps
endif
