#
#
#
# Dependencies:
#  * OpenCV
#  * OpenGL
# * GLEW
# * TCL/TK
# * vips
# *glib2
PROJECT(threadedStereo)

CMAKE_MINIMUM_REQUIRED(VERSION 2.4)

# This can removed if the required cmake version is increased to 2.6
IF(COMMAND CMAKE_POLICY)
  CMAKE_POLICY(SET CMP0003 NEW)
ENDIF(COMMAND CMAKE_POLICY)


#-----------------------#
# Includes              #
#-----------------------#

find_package(PkgConfig)
find_package(OpenMP)
include(ExternalProject)
FIND_PACKAGE( OpenCV REQUIRED )
pkg_check_modules(GLIB2 REQUIRED glib-2.0)
set(OSG_COMPONENTS osg osgUtil osgViewer osgGA osgManipulator osgText osgDB)
find_package(OpenSceneGraph REQUIRED COMPONENTS ${OSG_COMPONENTS})
#GDAL
INCLUDE( ${PROJECT_SOURCE_DIR}/VirtualPlanetBuilder/CMakeModules/FindGDAL.cmake )

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


#-----------------------#
# Installation Options  #
#-----------------------#

IF( NOT CMAKE_INSTALL_PREFIX )
   SET( CMAKE_INSTALL_PREFIX "/usr/local" )
ENDIF( NOT CMAKE_INSTALL_PREFIX )

ExternalProject_Add(vrip
  SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/vrip"
  BUILD_IN_SOURCE 1 
  BUILD_COMMAND $(MAKE)   
  DOWNLOAD_COMMAND "" 
  UPDATE_COMMAND "" 
  CONFIGURE_COMMAND "" 
  INSTALL_COMMAND "" 
)

#-----------------------#
# Dependencies          #
#-----------------------#


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

pkg_check_modules( VIPS vips-7.24 )
# UF
SET( UF_LIBRARY ufGeographicConversions ufGPSParser )

#
# find OpenMP
#

find_package( OpenMP )

IF (OPENMP_FOUND)
    MESSAGE(STATUS "found OpenMP, compiling with flags: " ${OpenMP_CXX_FLAGS} )
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    MESSAGE(STATUS "flags are now: " ${CMAKE_CXX_FLAGS})
ENDIF(OPENMP_FOUND)





#-----------------------#
# Compiler setup        #
#-----------------------#
ADD_DEFINITIONS( "-Wall" )
cmake_policy(SET CMP0005 OLD)
#spatial lib defs
add_definitions("-DHAVE_FCNTL_H=1 -DHAVE_UNISTD_H=1 -DHAVE_SYS_TYPES_H=1 -DHAVE_SYS_STAT_H=1 -DHAVE_PTHREAD_H=1 -DHAVE_SYS_RESOURCE_H=1 -DHAVE_SYS_TIME_H=1 -DHAVE_STDINT_H=1 -DHAVE_GETTIMEOFDAY=1 -DHAVE_BZERO=1 -DHAVE_MEMSET=1 -DHAVE_MEMCPY=1 -DHAVE_BCOPY=1")
INCLUDE_DIRECTORIES( ${PROJECT_SOURCE_DIR}/include
                     ${OPENCV_INCLUDE_DIRS}
		     ${OPENGL_INCLUDE_DIR}
                     ${VIPS_INCLUDE_DIRS}
                    spatialindex-src-1.6.1/src spatialindex-src-1.6.1/include
                     "/opt/local/include"
"VirtualPlanetBuilder/include"
                     ${GDAL_INCLUDE_DIR}
		     ${OSG_INCLUDE_DIR}
                   ${GLIB2_INCLUDE_DIRS}  )

LINK_DIRECTORIES( ${OPENCV_LIBRARY_DIRS}
                  ${UF_LIBRARY_DIRS}
		          ${PROJECT_SOURCE_DIR}
${GLIB2_LIBRARY_DIRS}
                )
add_definitions(${GLIB2_CFLAGS})
LINK_LIBRARIES(
               ${OSG_LIBRARY}
 ${OSGDB_LIBRARY}
               ${OSGUTIL_LIBRARY}
     ${OSGTERRAIN_LIBRARY_DEBUG}
${GDAL_LIBRARY}
osgTerrain
osgFX
osgViewer
osgSim
        ${OpenCV_LIBS}
                ${OSGSHADOW_LIBRARY}
                ${OSGTEXT_LIBRARY_DEBUG}
                ${OSGGA_LIBRARY_DEBUG}
                ${OPENTHREADS_LIBRARY}
         ${GLIB2_LIBRARIES}
          ${OPENGL_LIBRARY})


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

# Locations for the library and binary files
SET( LIBRARY_OUTPUT_PATH     ${PROJECT_SOURCE_DIR}/lib )
SET( EXECUTABLE_OUTPUT_PATH  ${PROJECT_SOURCE_DIR}/ )
#needed to supress rdynamic which causes segfault
####
SET(CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS "")
###


#SUBDIRS(src include tools )
SUBDIRS(VirtualPlanetBuilder vcgapps spatialindex-src-1.6.1/src swrender )
SET( SRCS
                Extents.cpp
calibFile.cpp
StereoEngine.cpp
plyfile.cpp
TexturedSource.cpp
Clipper.cpp
VPBInterface.cpp
PLYWriterNodeVisitor.cc
vertexData.cpp
TexPyrAtlas.cpp
Semaphore.cpp
TexturingQuery.cpp
configFile.cpp
SeaBedIO.cpp
SpatialReference.cpp
		stereo_cells.cpp
GLImaging.cpp
                )
ADD_DEFINITIONS( "-g")
ADD_LIBRARY( ts STATIC ${SRCS} ${SPINDEX_SRC})
ADD_EXECUTABLE( threadedStereo threadedStereo.cpp ShellCmd.cpp )
ADD_EXECUTABLE( engine engine.cpp ShellCmd.cpp )

ADD_EXECUTABLE( stereo_mesh_gen stereo_mesh_gen.cpp )

ADD_EXECUTABLE( osgdem  VirtualPlanetBuilder/applications/osgdem/osgdem.cpp )

ADD_EXECUTABLE( treeBBClip treeBBClip.cpp  )
ADD_EXECUTABLE( calcTexCoord calcTexCoord.cpp )
ADD_EXECUTABLE( testImageModel testImageModel.cpp )
ADD_EXECUTABLE( dicedImage dicedImage.cpp )
ADD_EXECUTABLE(deepzoomVIPS DeepZoom.cpp)
ADD_EXECUTABLE(singleImageTex singleImageTex.cpp)
ADD_EXECUTABLE(createSem createSem.cpp)
ADD_EXECUTABLE(vertCheck vertCheck.cpp)
#ADD_EXECUTABLE(temp temp.cpp)

LINK_LIBRARIES(osgTerrain  osgFX osg  spindex vips vipsCC)
TARGET_LINK_LIBRARIES( threadedStereo ts  vips vipsCC vpb spindex)
TARGET_LINK_LIBRARIES( engine ts  vips vipsCC vpb spindex)
TARGET_LINK_LIBRARIES( osgdem vpb  )

TARGET_LINK_LIBRARIES( testImageModel ts  )
TARGET_LINK_LIBRARIES( deepzoomVIPS  vips vipsCC)
TARGET_LINK_LIBRARIES( testImageModel ts  )

TARGET_LINK_LIBRARIES( stereo_mesh_gen ts  )

TARGET_LINK_LIBRARIES( treeBBClip ts spindex )
TARGET_LINK_LIBRARIES( calcTexCoord ts spindex vpb vips vipsCC)
TARGET_LINK_LIBRARIES( singleImageTex ts spindex vpb vips vipsCC)

TARGET_LINK_LIBRARIES( dicedImage vips vipsCC ts z)

#TARGET_LINK_LIBRARIES( threshold ts )

# Disabled by DJLM. 
# Threaded stereo expects access to a large bunch of other executables
# and scripts and is expecting to in the same directory as these other
# tools. To do this  properly you need to know all of the external utilities
# that T.S. calls and these all need to appear in /usr/local/bin
# INSTALL_TARGETS( /bin threadedStereo )
