# 
# libplankton build system
#

PROJECT(ann-merge)
CMAKE_MINIMUM_REQUIRED(VERSION 2.4)

find_package( GLUT ) 
INCLUDE( ${CMAKE_ROOT}/Modules/CheckIncludeFileCXX.cmake )
INCLUDE( ${PROJECT_SOURCE_DIR}/../../CMakeUtils/FindOSG.cmake )
INCLUDE( ${PROJECT_SOURCE_DIR}/../../CMakeUtils/FindOpenCV.cmake ) 
FIND_PACKAGE( OpenGL )
IF( NOT OPENGL_FOUND )
   MESSAGE( FATAL_ERROR  "ERROR - Can't find OpenGL" )
ENDIF( NOT OPENGL_FOUND )
IF( NOT LIBPOLYP_PREFIX )
   SET( LIBPOLYP_PREFIX /usr/local )
ENDIF( NOT LIBPOLYP_PREFIX )
# Gts
pkg_check_modules( GTS "gts" )
IF( NOT GTS_FOUND )
   MESSAGE( FATAL_ERROR  "ERROR - Can't find Gts" )
ENDIF( NOT GTS_FOUND )   

#--------------------------------#
# Enforce an out-of-source build #
#--------------------------------#

STRING(COMPARE EQUAL "${PROJECT_SOURCE_DIR}" "${PROJECT_BINARY_DIR}" INSOURCE)
IF(INSOURCE)
  MESSAGE(FATAL_ERROR "This project requires an out of source build. This is handled by the Makefile. Please type 'make'.")
ENDIF(INSOURCE)
 if(COMMAND cmake_policy)
      cmake_policy(SET CMP0003 NEW)
    endif(COMMAND cmake_policy)

#--------------------------------#
# Build Options                  #
#--------------------------------#

# Options can be set from the command line when running cmake.
#   eg. cmake . -DCMAKE_BUILD_TYPE=Release -DDEBUG_PROFILING=on
    
# Build types are: Release, Debug, RelWithDebInfo, MinSizeRel
IF( NOT CMAKE_BUILD_TYPE )
 SET( CMAKE_BUILD_TYPE Release )  
ENDIF( NOT CMAKE_BUILD_TYPE )


#-----------------------#
# Installation Options  #
#-----------------------#

IF( NOT CMAKE_INSTALL_PREFIX )
   SET( CMAKE_INSTALL_PREFIX "/usr/local" )
ENDIF( NOT CMAKE_INSTALL_PREFIX )

IF( NOT LIBPOLYP_PREFIX )
   SET( LIBPOLYP_PREFIX /usr/local )
ENDIF( NOT LIBPOLYP_PREFIX )

IF( NOT LIBPLANKTON_PREFIX )
   SET( LIBPLANKTON_PREFIX /usr/local/ )
ENDIF( NOT LIBPLANKTON_PREFIX )
# libplankton 
SET( LIBPLANKTON_INCLUDE_DIR ${LIBPLANKTON_PREFIX}/include/libplankton )
SET( LIBPLANKTON_LIB plankton )
#CHECK_INCLUDE_FILE_CXX( "auv_matrix.hpp" HAVE_LIBPLANKTON "-I ${LIBPLANKTON_INCLUDE_DIR}" )
#IF( NOT HAVE_LIBPLANKTON )
#   MESSAGE( FATAL_ERROR  "ERROR - Can't find libplankton" )
#ENDIF( NOT HAVE_LIBPLANKTON )    
#-----------------------#
# Compiler setup        #
#-----------------------#
IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    MESSAGE("Darwin build dyn lib")
    ADD_DEFINITIONS( "-Wall -fno-common" )
ELSE(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    ADD_DEFINITIONS( "-Wall" )
ENDIF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

# libpolyp 
SET( LIBPOLYP_INC ${LIBPOLYP_PREFIX}/include/libpolyp )
SET( LIBPOLYP_LIBRARY polyp )
# Boost 
FIND_PACKAGE(Boost)
IF( NOT Boost_FOUND )
   MESSAGE( FATAL_ERROR  "ERROR - Can't find boost" )
ENDIF( NOT Boost_FOUND )      
SET( CMAKE_REQUIRED_INCLUDES ${Boost_INCLUDE_DIR} )
# libpolyp 
SET( LIBPOLYP_INC ${LIBPOLYP_PREFIX}/include/libpolyp )
SET( LIBPOLYP_LIBRARY polyp )
INCLUDE_DIRECTORIES( ${PROJECT_SOURCE_DIR}/include  
  ${LIBPOLYP_INC} ${Boost_INCLUDE_DIR} ${LIBPOLYP_INC} 
  ${GTS_INCLUDE_DIRS} 		     
  ${OSG_INCLUDE_DIR}
    ${OPENCV_INCLUDE_DIRS} 
  ${LIBPLANKTON_INCLUDE_DIR})


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

#-----------------------#
# Build Instructions    #
#-----------------------#
INCLUDE_DIRECTORIES( 


		     )

# Locations for the library and binary files
SET( LIBRARY_OUTPUT_PATH     ${PROJECT_SOURCE_DIR}/lib )
SET( EXECUTABLE_OUTPUT_PATH  ${PROJECT_SOURCE_DIR}/bin )
LINK_DIRECTORIES( ${PROJECT_SOURCE_DIR}/nn/ )
LINK_LIBRARIES(	${GLUT_LIBRARY}  ${OSG_LIBRARY} ${OSGVIEWER_LIBRARY} 	${LIBPOLYP_LIBRARY} nn ${OPENGL_gl_LIBRARY} ${LIBPLANKTON_LIB} netcdf)

#ADD_EXECUTABLE( quadmerge quadmerge.cpp envelope.cpp raster.cpp )
#ADD_EXECUTABLE( quadtest quadtest.cpp envelope.cpp raster.cpp geometry.cpp uquadtree.cpp )
ADD_EXECUTABLE( quadmerge quadmerge.cpp envelope.cpp raster.cpp geometry.cpp uquadtree.cpp fileio.cpp)
# Build libraries
#SUBDIRS(src) #tests

#-----------------------#
# Testing Instructions  #
#-----------------------#

#ENABLE_TESTING( )
#ADD_TEST( cross_prod_mat_test ${EXECUTABLE_OUTPUT_PATH}/cross_prod_mat_test  )
#ADD_TEST( pose_test           ${EXECUTABLE_OUTPUT_PATH}/pose_test            )
#ADD_TEST( rot_mat_test        ${EXECUTABLE_OUTPUT_PATH}/rot_mat_test         )
#ADD_TEST( rot_mat_test        ${EXECUTABLE_OUTPUT_PATH}/coord_trans_jac_test )
#INCLUDE( Dart )
