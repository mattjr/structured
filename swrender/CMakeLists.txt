
cmake_minimum_required (VERSION 2.6) 
project (render)

find_package(PkgConfig)

pkg_check_modules( VIPS vips-7.24 )

set(OSG_COMPONENTS osg osgUtil osgViewer osgGA osgManipulator osgText osgDB)
find_package(OpenSceneGraph REQUIRED COMPONENTS ${OSG_COMPONENTS})

   include_directories( ../ ${OSG_INCLUDE_DIR} ${VIPS_INCLUDE_DIRS})
add_library(swrender mipmap_ipl.cpp VertexShaders.cpp  FragmentShaders.cpp render_utils.cpp ../vertexData.cpp  ClippedTriangle.cpp Raster.cpp VipsSampler.cpp ../plyfile.c renderer/geometry_processor.cpp ../MemUtils.cpp ar.cpp)
add_executable (nonmem . nonmem.cpp )

#add_executable (calcTexCoordOutCore  calcTexCoordOutCore.cpp render_utils.cpp vertexData.cpp plyfile.c IMG_savepng.c renderer/geometry_processor.cpp )
add_executable (rangeimg rangeimg.cpp)

add_executable (blendincore  blendincore.cpp)
add_executable (meshvar  meshvar.cpp)
add_executable (depthmap  depthmap.cpp)

#add_executable (rendertest example2.cpp render_utils.cpp vertexData.cpp plyfile.c IMG_savepng.c renderer/geometry_processor.cpp)
   target_link_libraries( nonmem  swrender ${OSG_LIBRARY} ${OpenCV_LIBS}
 ${OSGDB_LIBRARY}
               ${OSGUTIL_LIBRARY} ${VIPS_LIBRARIES} vips vipsCC)
   target_link_libraries( rangeimg  swrender ${OSG_LIBRARY} ${OpenCV_LIBS} ts
 ${OSGDB_LIBRARY}
               ${OSGUTIL_LIBRARY} ${VIPS_LIBRARIES} vips vipsCC)
   target_link_libraries( blendincore swrender ${OSG_LIBRARY} ${OpenCV_LIBS}
 ${OSGDB_LIBRARY}
               ${OSGUTIL_LIBRARY})
   target_link_libraries( meshvar swrender ${OSG_LIBRARY} ${OpenCV_LIBS}
 ${OSGDB_LIBRARY}
               ${OSGUTIL_LIBRARY})
   target_link_libraries( depthmap swrender ${OSG_LIBRARY} ${OpenCV_LIBS} ts
 ${OSGDB_LIBRARY}
               ${OSGUTIL_LIBRARY} ${VIPS_LIBRARIES} vips vipsCC)
 #  target_link_libraries( calcTexCoordOutCore  ${SDL_LIBRARY} SDL_image ${OSG_LIBRARY} png cv highgui ${OSGDB_LIBRARY}               ${OSGUTIL_LIBRARY})
SET(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR})
