##################################################
# Copyright (c) INRIA (France) 2011, 2012, 2013
# 
# This file is part of inria-mvs. You can redistribute it and/or
# modify it under the terms of the GNU General Public License.
# 
# This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
# WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
# 
# Author: Jean-Paul CHIEZE <jean-paul.chieze@inria.fr>
# 
##################################################

# Created by the script cgal_create_cmake_script
# This is the CMake script for compiling a CGAL application.
# cmake -DCGAL_DIR="$HOME/cgal4.1/lib/CGAL" -DCMAKE_MODULE_PATH=$HOME/cgal4.1/lib/CGAL ..
# cmake -DCMAKE_BUILD_TYPE=Debug -DCGAL_DONT_OVERRIDE_CMAKE_FLAGS=TRUE -DCGAL_DIR="/local/chieze/cgal4.1/lib/CGAL" -DCMAKE_MODULE_PATH=/local/chieze/cgal4.1/lib/CGAL ..
# rioc:

# cmake -DCMAKE_CXX_COMPILER=/cm/shared/apps/gcc/4.4.6/bin/c++ -DBOOST_LIBRARYDIR=$HOME/boost/lib -DCGAL_DIR="$HOME/cgal4.1/lib/CGAL" -DCMAKE_MODULE_PATH=$HOME/cgal4.1/lib/CGAL ..
project( Triangulation )
cmake_minimum_required(VERSION 2.6.2)
if("${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}" VERSION_GREATER 2.6)
  if("${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}.${CMAKE_PATCH_VERSION}" VERSION_GREATER 2.8.3)
    cmake_policy(VERSION 2.8.4)
  else()
    cmake_policy(VERSION 2.6)
  endif()
endif()
if("${CMAKE_BUILD_TYPE}" STREQUAL  "Debug")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -frounding-math -DCGAL_USE_GMP -DCGAL_USE_MPFR")
endif()
if(NOT ("${BOOST_LIBRARYDIR}" STREQUAL ""))
  link_directories(${BOOST_LIBRARYDIR})
endif()
FIND_PACKAGE( OpenMP )

find_package(CGAL COMPONENTS Qt4 )

# find qt4
set( QT_USE_QTXML    TRUE )
set( QT_USE_QTMAIN   TRUE )
set( QT_USE_QTSCRIPT  TRUE )
set( QT_USE_QTOPENGL  TRUE )
find_package(Qt4)
find_package(OpenGL)
if(QT4_FOUND)
#  message("XX ${QT_USE_FILE}")
  include(${QT_USE_FILE})
  include(FindQGLViewer.cmake)

#  find_package(QGLViewer )
endif(QT4_FOUND)

if ( CGAL_FOUND )
  INCLUDE_DIRECTORIES(./maxflow)
  ADD_SUBDIRECTORY(maxflow)
  include( ${CGAL_USE_FILE} )

  include( CGAL_CreateSingleSourceCGALProgram )

#  include_directories (BEFORE "../../include")
  add_executable(triangclean triangclean.cpp config.cpp extract.cpp smooth.cpp gviewer.cpp delaunay_io.cpp)
  target_link_libraries(triangclean maxflow ${CGAL_LIBRARIES}  ${CGAL_3RD_PARTY_LIBRARIES})

  add_executable(delaunay delaunay.cpp config.cpp delaunay_io.cpp addcells.cpp intersect.cpp gviewer.cpp )
  target_link_libraries(delaunay ${CGAL_LIBRARIES}  ${CGAL_3RD_PARTY_LIBRARIES})
  set_target_properties(delaunay PROPERTIES COMPILE_FLAGS  "${OpenMP_CXX_FLAGS}" LINK_FLAGS "${OpenMP_CXX_FLAGS} ")

  add_executable(delaunayb delaunay.cpp config.cpp delaunay_io.cpp read_bunbler.cpp addcells.cpp intersect.cpp gviewer.cpp )
  target_link_libraries(delaunayb ${CGAL_LIBRARIES}  ${CGAL_3RD_PARTY_LIBRARIES})
  set_target_properties(delaunayb PROPERTIES COMPILE_FLAGS  "-DUSE_BUNDLER ${OpenMP_CXX_FLAGS}" LINK_FLAGS "${OpenMP_CXX_FLAGS} ")

  add_executable(delaunay1 delaunay.cpp config.cpp delaunay_io.cpp addcells.cpp intersect.cpp gviewer.cpp )
  target_link_libraries(delaunay1 ${CGAL_LIBRARIES}  ${CGAL_3RD_PARTY_LIBRARIES})

  add_executable(hist hist.cpp config.cpp delaunay_io.cpp)
  target_link_libraries(hist ${CGAL_LIBRARIES}  ${CGAL_3RD_PARTY_LIBRARIES})

  add_executable(triangclean.d triangclean.cpp config.cpp extract.cpp smooth.cpp gviewer.cpp delaunay_io.cpp)
  target_link_libraries(triangclean.d maxflow ${CGAL_LIBRARIES}  ${CGAL_3RD_PARTY_LIBRARIES})
  set_target_properties(triangclean.d PROPERTIES COMPILE_FLAGS "-DTRIANGDEBUG")
  message("QGL ${QGLVIEWER_LIBRARIES}")

  if(NOT ("${INST_BIN}" STREQUAL ""))
    install(TARGETS delaunay delaunayb delaunay1 triangclean hist RUNTIME DESTINATION ${INST_BIN})
  endif()
  if(NOT ("${QGLVIEWER_LIBRARIES}" STREQUAL ""))
    add_executable(cmpcgal cmpcgal.cpp config.cpp delaunay_io.cpp)
    target_link_libraries(cmpcgal ${QT_LIBRARIES} ${CGAL_LIBRARIES}  ${CGAL_3RD_PARTY_LIBRARIES} ${QGLVIEWER_LIBRARIES} ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY} )

    add_executable(camtest0 camtest0.cpp config.cpp qviewer.cpp delaunay_io.cpp)
    target_link_libraries(camtest0 ${QT_LIBRARIES} ${CGAL_LIBRARIES}  ${CGAL_3RD_PARTY_LIBRARIES} ${QGLVIEWER_LIBRARIES} ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY} )

    add_executable(camtest camtest.cpp config.cpp qviewer.cpp delaunay_io.cpp)
    target_link_libraries(camtest ${QT_LIBRARIES} ${CGAL_LIBRARIES}  ${CGAL_3RD_PARTY_LIBRARIES} ${QGLVIEWER_LIBRARIES} ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY} )

    if(NOT ("${INST_BIN}" STREQUAL ""))
      install(TARGETS camtest camtest0 cmpcgal RUNTIME DESTINATION ${INST_BIN})
    endif()

  endif()

else()
  
    message(STATUS "CGAL library not found : triangulation programs will not be compiled.")
  
endif()

