# 
# libpolyp
#
# Dependencies:
#  * boost
#  * lapack
#  * OpenCV
#  * OpenGL
#  * ulapack 
#  * libplankton 
#  * libsnapper
#  * libpolyp
#  * gts
PROJECT(libpolyp)

INCLUDE( ${CMAKE_ROOT}/Modules/CheckIncludeFileCXX.cmake )
INCLUDE( ${CMAKE_ROOT}/Modules/CheckLibraryExists.cmake )
INCLUDE( ${CMAKE_ROOT}/Modules/UsePkgConfig.cmake )

INCLUDE( ${PROJECT_SOURCE_DIR}/../CMakeUtils/FindOSG.cmake )
INCLUDE( ${PROJECT_SOURCE_DIR}/../CMakeUtils/FindOpenCV.cmake ) 
INCLUDE( ${PROJECT_SOURCE_DIR}/../CMakeUtils/FindLapack.cmake ) 

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

OPTION( USE_LIBKEYPOINT
        "Use the libkeypoint feature extraction library."
        ON )
 
IF( NOT LIBADT_PREFIX )
   SET( LIBADT_PREFIX /usr/local )
ENDIF( NOT LIBADT_PREFIX )

IF( NOT LIBPOLYP_PREFIX )
   SET( LIBPOLYP_PREFIX /usr/local )
ENDIF( NOT LIBPOLYP_PREFIX )

IF( NOT ULAPACK_PREFIX )
   SET( ULAPACK_PREFIX /usr/local/ )
ENDIF( NOT ULAPACK_PREFIX )

IF( NOT LIBPLANKTON_PREFIX )
   SET( LIBPLANKTON_PREFIX /usr/local/ )
ENDIF( NOT LIBPLANKTON_PREFIX )

IF( NOT LIBSNAPPER_PREFIX )
   SET( LIBSNAPPER_PREFIX /usr/local/ )
ENDIF( NOT LIBSNAPPER_PREFIX )

IF( NOT LIBKEYPOINT_PREFIX )
   SET( LIBKEYPOINT_PREFIX /usr/local/ )
ENDIF( NOT LIBKEYPOINT_PREFIX )
#-----------------------#
# Installation Options  #
#-----------------------#

IF( NOT CMAKE_INSTALL_PREFIX )
   SET( CMAKE_INSTALL_PREFIX "/usr/local" )
ENDIF( NOT CMAKE_INSTALL_PREFIX )


#-----------------------#
# Dependencies          #
#-----------------------#

# Boost 
FIND_PACKAGE(Boost)
IF( NOT Boost_FOUND )
   MESSAGE( FATAL_ERROR  "ERROR - Can't find boost" )
ENDIF( NOT Boost_FOUND )      
SET( CMAKE_REQUIRED_INCLUDES ${Boost_INCLUDE_DIR} )
# boost threads
CHECK_INCLUDE_FILE_CXX( "boost/thread.hpp" HAVE_BOOST_THREAD )
IF( NOT HAVE_BOOST_THREAD )
   MESSAGE( FATAL_ERROR  "ERROR - Can't find boost threads" )
ENDIF( NOT HAVE_BOOST_THREAD )    

IF(APPLE)
SET( BOOST_THREAD_LIBRARY boost_thread-mt )
else(APPLE)
SET( BOOST_THREAD_LIBRARY boost_thread )
ENDIF(APPLE)


# OpenGL and GLU
FIND_PACKAGE( OpenGL )
IF( NOT OPENGL_FOUND )
   MESSAGE( FATAL_ERROR  "ERROR - Can't find OpenGL" )
ENDIF( NOT OPENGL_FOUND )
IF( NOT OPENGL_GLU_FOUND )
   MESSAGE( FATAL_ERROR  "ERROR - Can't find GLU" )
ENDIF( NOT OPENGL_GLU_FOUND )


IF( NOT OSG_FOUND )
  MESSAGE( FATAL_ERROR  "ERROR - Can't find OSG" )
ENDIF( NOT OSG_FOUND )

# lapack
IF( NOT LAPACK_FOUND )
   MESSAGE( FATAL_ERROR  "ERROR - Can't find lapack" )
ENDIF( NOT LAPACK_FOUND )

  
# Gts
PKGCONFIG("gts" GTS_INCLUDE_DIR GTS_LINK_DIR GTS_LINK_FLAGS GTS_CFLAGS )
IF( NOT GTS_INCLUDE_DIR )
   MESSAGE( FATAL_ERROR  "ERROR - Can't find Gts" )
ENDIF( NOT GTS_INCLUDE_DIR )      
MESSAGE( ${GTS_INCLUDE_DIR} )


# FIXME: the --export-dynamic in the GTS linker flags is causing threadedStereo
#        to segfault. I don't know why this isn't working. For now I'm just
#        reproducing the linker flags without the --export-dynamic.
IF( NOT APPLE )
MESSAGE( "WARNING - Performing dodgy update of GTKS cflags to prevent --export-dynamic" )
MESSAGE( "Original GTS_LINK_FLAGS = ${GTS_LINK_FLAGS}" )
SET( GTS_LINK_FLAGS "-pthread -lgts -lgthread-2.0 -lrt -lgmodule-2.0 -ldl -lglib-2.0" )
MESSAGE( "Modified GTS_LINK_FLAGS = ${GTS_LINK_FLAGS}" )
ENDIF ( NOT APPLE )


# ulapack 
SET( ULAPACK_INCLUDE_DIR ${ULAPACK_PREFIX}/include/ulapack )
CHECK_INCLUDE_FILE_CXX( "${ULAPACK_INCLUDE_DIR}/matrix_types.hpp" HAVE_ULAPACK )
IF( NOT HAVE_ULAPACK )
   MESSAGE( FATAL_ERROR  "ERROR - Can't find ulapack" )
ENDIF( NOT HAVE_ULAPACK )   


# libplankton 
SET( LIBPLANKTON_INCLUDE_DIR ${LIBPLANKTON_PREFIX}/include/libplankton )
SET( LIBPLANKTON_LIB plankton )
CHECK_INCLUDE_FILE_CXX( "auv_matrix.hpp" HAVE_LIBPLANKTON "-I ${LIBPLANKTON_INCLUDE_DIR}" )
IF( NOT HAVE_LIBPLANKTON )
   MESSAGE( FATAL_ERROR  "ERROR - Can't find libplankton" )
ENDIF( NOT HAVE_LIBPLANKTON )      


# libkeypoint
IF( USE_LIBKEYPOINT )
   SET( LIBKEYPOINT_INCLUDE_DIR ${LIBKEYPOINT_PREFIX}/include/libkeypoint )
   SET( LIBKEYPOINT_LIB keypoint surf )
   ADD_DEFINITIONS( "-DHAVE_LIBKEYPOINT" )
   
   SET( CMAKE_REQUIRED_FLAGS "-I ${OPENCV_INCLUDE_DIR} ${OPENCV_LINK_FLAGS}" )
   CHECK_INCLUDE_FILE_CXX( "keypoint.hpp" HAVE_LIBKEYPOINT "-I ${LIBKEYPOINT_INCLUDE_DIR}" )
   IF( NOT HAVE_LIBKEYPOINT )
#      MESSAGE( FATAL_ERROR  "ERROR - Can't find libkeypoint" )
   ENDIF( NOT HAVE_LIBKEYPOINT )      
ENDIF( USE_LIBKEYPOINT )

# libsnapper 
SET( LIBSNAPPER_INC ${LIBSNAPPER_PREFIX}/include/libsnapper )
SET( LIBSNAPPER_LIBRARY snapper )

# libpolyp 
SET( LIBPOLYP_INC ${LIBPOLYP_PREFIX}/include/libpolyp )
SET( LIBPOLYP_LIBRARY polyp )

# AUV data library 
SET( LIBADT_INCLUDE_DIR ${LIBADT_PREFIX}/include/libadt )
SET( LIBADT_LIBRARY adt )

#-----------------------#
# Compiler setup        #
#-----------------------#
ADD_DEFINITIONS( ${GTS_CFLAGS} )
ADD_DEFINITIONS( "-Wall " )

INCLUDE_DIRECTORIES( ${PROJECT_SOURCE_DIR}/include
                     ${OPENCV_INCLUDE_DIR}
		     ${OPENGL_INCLUDE_DIR}
 		     ${GTS_INCLUDE_DIR}
                     ${ULAPACK_INCLUDE_DIR}
		     ${LIBADT_INCLUDE_DIR}
  		     ${LIBSNAPPER_INC}
		     ${LIBPOLYP_INC}
                     ${LIBPLANKTON_INCLUDE_DIR}
                     ${LIBKEYPOINT_INCLUDE_DIR}
		     ${OSG_INCLUDE_DIR}
		     )

LINK_DIRECTORIES( ${OPENCV_LINK_DIR} ${GTS_LINK_DIR})

LINK_LIBRARIES(	${OSG_LIBRARY}
		${OSGDB_LIBRARY}
		${OSGUTIL_LIBRARY}
		${OSGVIEWER_LIBRARY}
		${OSGTEXT_LIBRARY}
		${LIBADT_LIBRARY}
		${OPENCV_LIBRARY}
		${BOOST_THREAD_LIBRARY}
		${LIBPLANKTON_LIBRARY}	
		${LIBSNAPPER_LIBRARY}
		${GTS_LINK_FLAGS}
		${LAPACK_LIBRARY}
		-l3ds    		
		${LIBPOLYP_LIBRARY}
	)


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
MESSAGE( "USE_LIBKEYPOINT = '${USE_LIBKEYPOINT}'" )
MESSAGE( "" )
MESSAGE( "ULAPACK_PREFIX     = '${ULAPACK_PREFIX}'" )
MESSAGE( "LIBPLANKTON_PREFIX = '${LIBPLANKTON_PREFIX}'" )
MESSAGE( "LIBKEYPOINT_PREFIX = '${LIBKEYPOINT_PREFIX}'" )

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

#-----------------------#
# Build Instructions    #
#-----------------------#

# Locations for the library and binary files
SET( LIBRARY_OUTPUT_PATH     ${PROJECT_SOURCE_DIR}/lib )
SET( EXECUTABLE_OUTPUT_PATH  ${PROJECT_SOURCE_DIR}/ )
#needed to supress rdynamic which causes segfault
####
SET(CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS "")
###


#SUBDIRS(src include tools )
SUBDIRS(retex)
SET( SRCS OSGExport.cpp
		auv_clipped_texture_atlas.cpp
		Depth.cpp
		auv_3ds.cpp
		novelty.cpp
		stereo_cells.cpp
		)

ADD_DEFINITIONS( "-g")
ADD_LIBRARY( ts STATIC ${SRCS} )
ADD_EXECUTABLE( threadedStereo threadedStereo.cpp ShellCmd.cpp )
ADD_EXECUTABLE( genTex genTex.cpp )
ADD_EXECUTABLE( lodgen lodgen.cpp )
TARGET_LINK_LIBRARIES( threadedStereo ts plankton )
TARGET_LINK_LIBRARIES( genTex ts plankton)
TARGET_LINK_LIBRARIES( lodgen ts )
