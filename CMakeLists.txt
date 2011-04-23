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
PROJECT(threadedStereo)

CMAKE_MINIMUM_REQUIRED(VERSION 2.4)

# This can removed if the required cmake version is increased to 2.6
IF(COMMAND CMAKE_POLICY)
  CMAKE_POLICY(SET CMP0003 NEW)
ENDIF(COMMAND CMAKE_POLICY)


#-----------------------#
# Includes              #
#-----------------------#

INCLUDE( ${CMAKE_ROOT}/Modules/CheckIncludeFileCXX.cmake )
INCLUDE( ${CMAKE_ROOT}/Modules/CheckLibraryExists.cmake )
INCLUDE( ${CMAKE_ROOT}/Modules/FindPkgConfig.cmake )
INCLUDE( ${PROJECT_SOURCE_DIR}/../CMakeUtils/FindOpenMP.cmake ) 

INCLUDE( ${PROJECT_SOURCE_DIR}/../CMakeUtils/FindUF.cmake ) 
INCLUDE( ${PROJECT_SOURCE_DIR}/../CMakeUtils/FindOSG.cmake )
INCLUDE( ${PROJECT_SOURCE_DIR}/../CMakeUtils/FindOpenCV.cmake ) 
INCLUDE( ${PROJECT_SOURCE_DIR}/../CMakeUtils/FindLapack.cmake ) 
INCLUDE( ${PROJECT_SOURCE_DIR}/../CMakeUtils/FindGDAL.cmake )

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

IF( NOT LIBSEABEDCOMMON_PREFIX )
   SET( LIBSEABEDCOMMON_PREFIX /usr/local/ )
ENDIF( NOT LIBSEABEDCOMMON_PREFIX )

IF( NOT GMT_PREFIX )
   SET( GMT_PREFIX /usr/include/ )
ENDIF( NOT GMT_PREFIX )

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
FIND_PACKAGE(Boost COMPONENTS thread)
IF( NOT Boost_FOUND )
   MESSAGE( FATAL_ERROR  "ERROR - Can't find boost" )
ENDIF( NOT Boost_FOUND )     
SET( CMAKE_REQUIRED_INCLUDES ${Boost_INCLUDE_DIR} )

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
pkg_check_modules( GTS "gts" )
IF( NOT GTS_FOUND )
   MESSAGE( FATAL_ERROR  "ERROR - Can't find Gts" )
ENDIF( NOT GTS_FOUND )   

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
   SET( LIBKEYPOINT_LIB keypoint  )
   ADD_DEFINITIONS( "-DHAVE_LIBKEYPOINT" )
   
   SET( CMAKE_REQUIRED_FLAGS "-I ${OPENCV_INCLUDE_DIRS} ${OPENCV_LIBRARIES}" )
   CHECK_INCLUDE_FILE_CXX( "keypoint.hpp" HAVE_LIBKEYPOINT "-I ${LIBKEYPOINT_INCLUDE_DIR}" )
   IF( NOT HAVE_LIBKEYPOINT )
#      MESSAGE( FATAL_ERROR  "ERROR - Can't find libkeypoint" )
   ENDIF( NOT HAVE_LIBKEYPOINT )      
ENDIF( USE_LIBKEYPOINT )

# libseabedcommon
SET( LIBSEABEDCOMMON_INCLUDE_DIR ${LIBSEABEDCOMMON_PREFIX}/include/libseabedcommon )
SET( LIBSEABEDCOMMON_LIB seabedcommon  )


SET( CMAKE_REQUIRED_FLAGS "-I ${OPENCV_INCLUDE_DIRS} ${OPENCV_LIBRARIES}" )
CHECK_INCLUDE_FILE_CXX( "seabed_slam_file_io.hpp" HAVE_LIBSEABEDCOMMON "-I ${LIBSEABEDCOMMON_INCLUDE_DIR}" )
IF( NOT HAVE_LIBSEABEDCOMMON )
  #      MESSAGE( FATAL_ERROR  "ERROR - Can't find libseabedcommon" )
ENDIF( NOT HAVE_LIBSEABEDCOMMON )      



# gmt
SET( GMT_INC ${GMT_PREFIX}/gmt/ )
SET( GMT_LIBRARY gmt netcdf )
SET( GMT_DIR /usr/lib/gmt/lib )

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
ADD_DEFINITIONS( "-Wall -msse4.1" )

INCLUDE_DIRECTORIES( ${PROJECT_SOURCE_DIR}/include
                     ${OPENCV_INCLUDE_DIRS}
		     ${OPENGL_INCLUDE_DIR}
 		     ${GTS_INCLUDE_DIRS}
                     "${PROJECT_SOURCE_DIR}/mergeMesh/"
		     "tridecimator/"
		     "mesh2hmap/"
		     ${GDAL_INCLUDE_DIR}
                    spatialindex-src-1.6.1/src spatialindex-src-1.6.1/include
		     "/opt/local/include"
                     ${ULAPACK_INCLUDE_DIR}
		     ${LIBADT_INCLUDE_DIR}
  		     ${LIBSNAPPER_INC}
		     ${LIBPOLYP_INC}
                     ${LIBPLANKTON_INCLUDE_DIR}
                     ${LIBKEYPOINT_INCLUDE_DIR}
                     ${LIBSEABEDCOMMON_INCLUDE_DIR}
		     ${OSG_INCLUDE_DIR}
      ${UF_INCLUDE_DIRS}
${GMT_INC}
"FastDXT/"

		     )

LINK_DIRECTORIES( ${OPENCV_LIBRARY_DIRS}
                  ${GTS_LIBRARY_DIRS}
                  ${UF_LIBRARY_DIRS}
		          ${GMT_DIR}
		          ${PROJECT_SOURCE_DIR}
                  /quadmerge/nn
                )

LINK_LIBRARIES(
 #       /usr/local/lib64/libosgd.so
  #              /usr/local/lib64/libosgDBd.so
   #               /usr/local/lib64/libosgUtild.so
#${OSG_LIBRARY_DEBUG}
               ${OSG_LIBRARY_DEBUG}
 ${OSGDB_LIBRARY_DEBUG}
               ${OSGUTIL_LIBRARY_DEBUG}
     ${OSGTERRAIN_LIBRARY_DEBUG}
#/usr/lib//libgdal1.6.0.so.1
${GDAL_LIBRARY}
osgTerrain
osgFX
osgViewer
#/usr/local/lib64/libosgViewerd.so
osgSim
#-L/usr/local/lib/static nvtt nvcore nvmath  nvimage -fopenmp -L/usr/local/cuda/lib64 cudart cuda /home/mattjr/exp/nvidia-texture-tools-read-only/build-Debug/src/nvtt/squish/libsquish.a
                ${OSGSHADOW_LIBRARY}
                ${OSGVIEWER_LIBRARY_DEBUG}
                ${OSGTEXT_LIBRARY_DEBUG}
                ${OSGGA_LIBRARY_DEBUG}
                ${OPENTHREADS_LIB}
                ${LIBADT_LIBRARY}
		${OPENCV_LIBRARIES}
		${LIBPLANKTON_LIBRARY}	
		${LIBSNAPPER_LIBRARY}
		${GTS_LIBRARIES}
		${LAPACK_LIBRARY}
		${Boost_LIBRARIES}
        ${LIBKEYPOINT_LIB}
        ${LIBSEABEDCOMMON_LIB}
		-l3ds    		
		${LIBPOLYP_LIBRARY}
		${UF_LIBRARY}
fastdxt
 
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
SUBDIRS( FastDXT spatialindex-src-1.6.1/src texture_image retex mbosg texturedDecimator mesh2hmap mesh2xy grd2ply  quadmerge single_mesh_gen)
SET( SRCS OSGExport.cpp
		auv_clipped_texture_atlas.cpp
		Depth.cpp
                Extents.cpp
		auv_3ds.cpp
                #Clean.cpp
		novelty.cpp
PosterPrinter.cpp
imageNode.cpp
sha2.cpp
plyfile.cpp
TexturedSource.cpp
Clipper.cpp
VPBInterface.cpp
PLYWriterNodeVisitor.cc
vertexData.cpp
TexPyrAtlas.cpp
TexturingQuery.cpp
		stereo_cells.cpp
                tridecimator/plylib.cpp
		quadmerge/envelope.cpp
		quadmerge/fileio.cpp
		)

ADD_DEFINITIONS( "-g")
ADD_LIBRARY( ts STATIC ${SRCS} ${SPINDEX_SRC})
ADD_EXECUTABLE( threadedStereo threadedStereo.cpp ShellCmd.cpp )
ADD_EXECUTABLE( genTex genTex.cpp )

ADD_EXECUTABLE( lodgen lodgen.cpp )
ADD_EXECUTABLE( treeBBClip treeBBClip.cpp  )
ADD_EXECUTABLE( simplifier simplifier.cpp)
ADD_EXECUTABLE( calcTexCoord calcTexCoord.cpp )
ADD_EXECUTABLE( monoDriver monoDriver.cpp ShellCmd.cpp)
ADD_EXECUTABLE( testImageModel testImageModel.cpp )
ADD_EXECUTABLE( loadCamRender loadCamRender.cpp )
ADD_EXECUTABLE( loadCamRender loadCamRender.cpp )
ADD_EXECUTABLE( screencap screencap.cpp )
ADD_EXECUTABLE( remapReimaged remapReimaged.cpp )
ADD_EXECUTABLE( dicedImage dicedImage.cpp )
ADD_EXECUTABLE(deepzoomVIPS DeepZoom.cpp)
ADD_EXECUTABLE(singleImageTex singleImageTex.cpp)

#ADD_EXECUTABLE( bin/mergeMesh mergeMesh/mergeMesh.cpp mergeMesh/plylib.cpp)
#ADD_EXECUTABLE( bin/texturedDecimator mergeMesh/texturedDecimator.cpp mergeMesh/plylib.cpp mergeMesh/quadric_simp.cpp mergeMesh/quadric_tex_simp.cpp)

LINK_LIBRARIES(osgTerrain vpb osgFX osg  spindex vips vipsCC)
TARGET_LINK_LIBRARIES( threadedStereo ts plankton hmap gmt netcdf vips vipsCC)
TARGET_LINK_LIBRARIES( testImageModel ts  )
TARGET_LINK_LIBRARIES( genTex ts plankton )
TARGET_LINK_LIBRARIES( remapReimaged ts  vips vipsCC vpb)
TARGET_LINK_LIBRARIES( deepzoomVIPS  vips vipsCC)

TARGET_LINK_LIBRARIES( monoDriver ts plankton vpb  hmap gmt netcdf)

TARGET_LINK_LIBRARIES( lodgen ts )

TARGET_LINK_LIBRARIES( treeBBClip ts spindex )
TARGET_LINK_LIBRARIES( calcTexCoord ts spindex vpb vips vipsCC)
TARGET_LINK_LIBRARIES( singleImageTex ts spindex vpb vips vipsCC)

TARGET_LINK_LIBRARIES( loadCamRender ts spindex vpb vips vipsCC )
TARGET_LINK_LIBRARIES( screencap vips vipsCC)
TARGET_LINK_LIBRARIES( dicedImage vips vipsCC)

#TARGET_LINK_LIBRARIES( threshold ts )

# Disabled by DJLM. 
# Threaded stereo expects access to a large bunch of other executables
# and scripts and is expecting to in the same directory as these other
# tools. To do this  properly you need to know all of the external utilities
# that T.S. calls and these all need to appear in /usr/local/bin
# INSTALL_TARGETS( /bin threadedStereo )
