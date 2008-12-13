#
# The Makefile calls cmake and make to perform an out-of-source build in the 
# directory specified by the BUILD_DIR variable. If the directory does not
# exist it is created.
#

BUILD_DIR = build

all: setup
	@make -C $(BUILD_DIR) all

install: setup
	@make -C $(BUILD_DIR) install

clean: setup      
	@make -C $(BUILD_DIR) clean


debug: setup
	@cd $(BUILD_DIR); \
	cmake .. -DCMAKE_BUILD_TYPE=Debug
	make

release: setup
	@cd $(BUILD_DIR); \
	cmake .. -DCMAKE_BUILD_TYPE=Release
	make 

relwithdebinfo: setup
	@cd $(BUILD_DIR); \
	cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
	make 

minsizerel: setup
	@cd $(BUILD_DIR); \
	cmake .. -DCMAKE_BUILD_TYPE=MinSizeRel
	make 


test: setup
	@cd $(BUILD_DIR); \
	ctest -D Experimental 

doc: setup
	doxygen doc/doxygen.cfg


# Catch all for other targets - will be passed to BUILD_DIR
%: setup
	@make -C $(BUILD_DIR) $@

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


