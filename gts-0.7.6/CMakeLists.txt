#include(ConfigureChecks.cmake)


#configure_file(${CMAKE_CURRENT_SOURCE_DIR}/config.h.cmake ${CMAKE_CURRENT_BINARY_DIR}/config.h)

include(CheckIncludeFile)

check_function_exists (getopt_long  HAVE_GETOPT_LONG)
check_function_exists (getpagesize  HAVE_GETPAGESIZE)
check_function_exists (gettimeofday HAVE_GETTIMEOFDAY)
check_function_exists (mmap         HAVE_MMAP)
check_include_file ("dlfcn.h"       HAVE_DLFCN_H)
check_include_file ("floatingpoint.h"    HAVE_FLOATINGPOINT_H)

check_include_file ("ieeefp.h"       HAVE_IEEEFP_H)
check_include_file ("inttypes.h"     HAVE_INTTYPES_H)
check_include_file ("memory.h"       HAVE_MEMORY_H)
check_include_file ("stdint.h"       HAVE_STDINT_H)
check_include_file ("stdlib.h"       HAVE_STDLIB_H)
check_include_file ("strings.h"      HAVE_STRINGS_H)
check_include_file ("string.h"       HAVE_STRING_H)
check_include_file ("sys/stat.h"     HAVE_SYS_STAT_H)
check_include_file ("sys/types.h"    HAVE_SYS_TYPES_H)
check_include_file ("unistd.h"       HAVE_UNISTD_H)




set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DHAVE_CONFIG_H")

# -----------------------------------------------
# configure section
# -----------------------------------------------

configure_file (${CMAKE_CURRENT_SOURCE_DIR}/config.in.cmake
    ${CMAKE_CURRENT_SOURCE_DIR}/config.h)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
ADD_CUSTOM_TARGET(predicates_init
    COMMAND ${CMAKE_CXX_COMPILER} -I${CMAKE_CURRENT_SOURCE_DIR}/ ${CMAKE_CURRENT_SOURCE_DIR}/src/predicates_init.c -o ${CMAKE_BINARY_DIR}/predicates_init    
   # DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/gts-0.7.6/src/predicates_init.c
    COMMENT "Generating precompiled headers"
    )
add_custom_target (predicates_init_app DEPENDS predicates_init  ${CMAKE_CURRENT_SOURCE_DIR}/config.h)

########### Generate predicates_init.h #########################################


#ADD_EXECUTABLE(predicates_init ${CMAKE_CURRENT_SOURCE_DIR}/src//predicates_init.c )
#add_custom_target(gts ALL 
#  DEPENDS predicates_init.h)
#ADD_CUSTOM_COMMAND(OUTPUT predicates_init.h
#                   COMMAND predicates_init  ${CMAKE_CURRENT_SOURCE_DIR}/src/predicates_init.h
#                   WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
#		   DEPENDS predicates_init#
#		   VERBATIM)

add_subdirectory(src)
#add_subdirectory(tools)
#add_subdirectory(examples)
#add_subdirectory(doc)
#add_subdirectory(test)

