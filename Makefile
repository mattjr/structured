#
# The Makefile calls cmake and make to perform an out-of-source build in the 
# directory specified by the BUILD_DIR variable. If the directory does not
# exist it is created.
#

BUILD_DIR = build

all: subdirs setup
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

test: setup
	@cd $(BUILD_DIR); \
	ctest -D Experimental 

# Catch all for other targets - will be passed to BUILD_DIR
%: setup
	@make -C $(BUILD_DIR) $@

setup:
        # Clean up if someone has run cmake . for an in-source build
	rm -f CMakeCache.txt  

        # Make a directory for an out-of-source build
	@if test -e $(BUILD_DIR) ; \
	then \
	   echo "$(BUILD_DIR) already exists"; \
	else \
	   echo "$(BUILD_DIR) does not exist - creating it now"; \
	   mkdir -p $(BUILD_DIR); \
	   cd $(BUILD_DIR); \
	   cmake ..; \
	fi

SUBDIRS =  vrip poisson

.PHONY: subdirs $(SUBDIRS)

subdirs: $(SUBDIRS)

$(SUBDIRS):
	$(MAKE) -C $@

