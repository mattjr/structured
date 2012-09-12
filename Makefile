#
# The Makefile calls cmake and make to perform an out-of-source build in the 
# directory specified by the BUILD_DIR variable. If the directory does not
# exist it is created.
#

BUILD_DIR = build

ifndef OSTYPE
   OSTYPE = $(shell uname)
endif


# Get the number of processors
ifeq ("$(OSTYPE)","Darwin")
   CPU_COUNT=2
else
   CPU_COUNT := $(shell cat /proc/cpuinfo | grep -P "processor\t" | wc -l)
   ifeq ($(CPU_COUNT),0)
      CPU_COUNT=1
   endif
endif

# Set the number of build threads to the number of processors
ARGS=-j ${CPU_COUNT}


# Normal build
all: setup
	@make ${ARGS} -C $(BUILD_DIR) all

# Other targets. This works for 'make clean' and 'make install'
%: setup
	@make ${ARGS} -C $(BUILD_DIR) $@


debug: setup
	@cd $(BUILD_DIR); \
	cmake .. -DCMAKE_BUILD_TYPE=Debug
	make ${ARGS}

release: setup
	@cd $(BUILD_DIR); \
	cmake .. -DCMAKE_BUILD_TYPE=Release
	make ${ARGS} 

relwithdebinfo: setup
	@cd $(BUILD_DIR); \
	cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
	make ${ARGS} 

minsizerel: setup
	@cd $(BUILD_DIR); \
	cmake .. -DCMAKE_BUILD_TYPE=MinSizeRel
	make ${ARGS} 


test: setup
	@cd $(BUILD_DIR); \
	ctest . 

doc: setup
	doxygen doc/doxygen.cfg




setup:
        # Clean up if someone has run cmake . for an in-source build
	rm -f CMakeCache.txt  

        # Make sure the directory for an out-of source build exists
        # If the build dir exists but the makefile doesn't, rerun cmake
	@if test -d $(BUILD_DIR) ; \
	then \
           if ! test -f $(BUILD_DIR)/Makefile  ; then \
	      cd $(BUILD_DIR); \
	      cmake ..; \
           fi \
	else \
	   mkdir -p $(BUILD_DIR); \
	   cd $(BUILD_DIR); \
	   cmake ..; \
	fi 


