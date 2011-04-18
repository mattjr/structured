# 
# poisson

PROJECT(tridecimator)


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


INCLUDE_DIRECTORIES( "."
		     "vcg"
".."
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


SET( EXECUTABLE_OUTPUT_PATH  ${PROJECT_SOURCE_DIR}/ )
#needed to supress rdynamic which causes segfault
####
SET(CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS "")
###

SET( SRCS ./wrap/ply/plylib.cpp   quadric_tex_simp.cpp   )





FIND_PACKAGE( GLUT )
IF( NOT GLUT_FOUND )
   MESSAGE( FATAL_ERROR  "ERROR - Can't find GLUT" )
ENDIF( NOT GLUT_FOUND )
## Compiler flags
if(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "-O3")        ## Optimize
endif()
ADD_EXECUTABLE( tridecimator  tridecimator.cpp  ${SRCS} )
ADD_EXECUTABLE( triGap triGap.cpp ${SRCS} )

ADD_EXECUTABLE( mergeMesh mergeMesh.cpp ${SRCS} )
ADD_EXECUTABLE( texturedDecimator  texturedDecimator.cpp ${SRCS} )
TARGET_LINK_LIBRARIES(texturedDecimator ts)
TARGET_LINK_LIBRARIES( triGap ts spatialindex )

ADD_EXECUTABLE( shadevis shadevis/shadevis.cpp ./wrap/ply/plylib.cpp wrap/gui/trackball.cpp wrap/gui/trackmode.cpp   )
IF( APPLE )
SET(GLEW_LIB glew)
ELSE(APPLE)
SET(GLEW_LIB GLEW)
ENDIF(APPLE)
TARGET_LINK_LIBRARIES(shadevis  ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} ${GLEW_LIB})
INSTALL_TARGETS( /bin texturedDecimator tridecimator)
INSTALL_TARGETS( /bin shadevis )
SET( EXECUTABLE_OUTPUT_PATH  ${PROJECT_SOURCE_DIR}/bin )




