# 
# poisson

PROJECT(poisson)


#--------------------------------#
# Enforce an out-of-source build #
#--------------------------------#

STRING(COMPARE EQUAL "${PROJECT_SOURCE_DIR}" "${PROJECT_BINARY_DIR}" INSOURCE)
IF(INSOURCE)
  MESSAGE(FATAL_ERROR "This project requires an out of source build. This is handled by the Makefile. Please type 'make'.")
ENDIF(INSOURCE)


#--------------------------------#
# Build Options                  #
#--------------------------------#

# Options can be set from the command line when running cmake.
#   eg. cmake . -DCMAKE_BUILD_TYPE=Release -DDEBUG_PROFILING=on
    

# Build types are: Release, Debug, RelWithDebInfo, MinSizeRel
IF( NOT CMAKE_BUILD_TYPE )
 SET( CMAKE_BUILD_TYPE Release )  
ENDIF( NOT CMAKE_BUILD_TYPE )

 
IF( NOT LIBPOLYP_PREFIX )
   SET( LIBPOLYP_PREFIX /usr/local )
ENDIF( NOT LIBPOLYP_PREFIX )

#-----------------------#
# Installation Options  #
#-----------------------#

IF( NOT CMAKE_INSTALL_PREFIX )
   SET( CMAKE_INSTALL_PREFIX "/usr/local" )
ENDIF( NOT CMAKE_INSTALL_PREFIX )



#-----------------------#
# Compiler setup        #
#-----------------------#
# -O3 -DRELEASE -funroll-loops -ffast-math
ADD_DEFINITIONS( "-fpermissive" )

#INCLUDE_DIRECTORIES( 
	#	     )



#-----------------------#
# Print Some Info       #
#-----------------------#

MESSAGE( "" )
MESSAGE( "CONFIGURATION SUMMARY" )
MESSAGE( "---------------------" )
MESSAGE( "" )
MESSAGE( "CMAKE_BUILD_TYPE = '${CMAKE_BUILD_TYPE}'" )
MESSAGE( "" )
MESSAGE( "CMAKE_INSTALL_PREFIX = '${CMAKE_INSTALL_PREFIX}'" )
MESSAGE( "" )


IF( CMAKE_BUILD_TYPE STREQUAL Debug )
   MESSAGE( "CMAKE_CXX_FLAGS_DEBUG = '${CMAKE_CXX_FLAGS_DEBUG}'" )
ENDIF( CMAKE_BUILD_TYPE STREQUAL Debug )

IF( CMAKE_BUILD_TYPE STREQUAL RelWithDebInfo )
   MESSAGE( "CMAKE_CXX_FLAGS_RELWITHDEBINFO = '${CMAKE_CXX_FLAGS_RELWITHDEBINFO}'" )
ENDIF( CMAKE_BUILD_TYPE STREQUAL RelWithDebInfo )

IF( CMAKE_BUILD_TYPE STREQUAL Release )
   MESSAGE( "CMAKE_CXX_FLAGS_RELEASE = '${CMAKE_CXX_FLAGS_RELEASE}'" )
ENDIF( CMAKE_BUILD_TYPE STREQUAL Release )

IF( CMAKE_BUILD_TYPE STREQUAL MinSizeRel )
   MESSAGE( "CMAKE_CXX_FLAGS_MINSIZEREL = '${CMAKE_CXX_FLAGS_MINSIZEREL}'" )
ENDIF( CMAKE_BUILD_TYPE STREQUAL MinSizeRel )

MESSAGE( "" )


# libpolyp 
SET( LIBPOLYP_INC ${LIBPOLYP_PREFIX}/include/libpolyp )
SET( LIBPOLYP_LIBRARY polyp )


INCLUDE_DIRECTORIES(
		     ${LIBPOLYP_INC})
 

LINK_LIBRARIES(	
		${LIBPOLYP_LIBRARY})

#-----------------------#
# Build Instructions    #
#-----------------------#


SET( EXECUTABLE_OUTPUT_PATH  ${PROJECT_SOURCE_DIR}/ )
#needed to supress rdynamic which causes segfault
####
SET(CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS "")
###

SUBDIRS(src)





ADD_EXECUTABLE( PoissonRecon src/MultiGridOctest.cpp ${SRCS} )
ADD_EXECUTABLE( dumpnormpts dumpnormpts.cc )
TARGET_LINK_LIBRARIES( PoissonRecon poisson )

