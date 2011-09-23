
cmake_minimum_required (VERSION 2.6) 
project (render)
IF (CMAKE_BUILD_TYPE STREQUAL "Release")

add_definitions(-Wall -O3 -DNDEBUG -pipe -ffast-math -funroll-loops -ftree-vectorize -fomit-frame-pointer -pipe -mfpmath=sse -mmmx -msse -mtune=core2 -march=core2 -msse2 -msse3 -mssse3 -msse4)
ENDIF()
find_package(PkgConfig)

pkg_check_modules( VIPS vips-7.24 )

set(OSG_COMPONENTS osg osgUtil osgViewer osgGA osgManipulator osgText osgDB)
find_package(OpenSceneGraph REQUIRED COMPONENTS ${OSG_COMPONENTS})

   include_directories( ${OSG_INCLUDE_DIR} ${VIPS_INCLUDE_DIRS})
add_executable (nonmem ../MemUtils.cpp nonmem.cpp render_utils.cpp vertexData.cpp plyfile.cpp renderer/geometry_processor.cpp )

#add_executable (calcTexCoordOutCore  calcTexCoordOutCore.cpp render_utils.cpp vertexData.cpp plyfile.cpp IMG_savepng.c renderer/geometry_processor.cpp )

add_executable (blendincore ../MemUtils.cpp blendincore.cpp render_utils.cpp vertexData.cpp plyfile.cpp renderer/geometry_processor.cpp)
#add_executable (rendertest example2.cpp render_utils.cpp vertexData.cpp plyfile.cpp IMG_savepng.c renderer/geometry_processor.cpp)
   target_link_libraries( nonmem ${SDL_LIBRARY} SDL_image ${OSG_LIBRARY} png cv highgui
 ${OSGDB_LIBRARY}
               ${OSGUTIL_LIBRARY} ${VIPS_LIBRARIES} vips vipsCC)

   target_link_libraries( blendincore ${SDL_LIBRARY} SDL_image ${OSG_LIBRARY} png cv highgui
 ${OSGDB_LIBRARY}
               ${OSGUTIL_LIBRARY})

 #  target_link_libraries( calcTexCoordOutCore  ${SDL_LIBRARY} SDL_image ${OSG_LIBRARY} png cv highgui ${OSGDB_LIBRARY}               ${OSGUTIL_LIBRARY})
SET(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR})
