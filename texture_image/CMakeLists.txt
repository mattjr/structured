#--------------------------------------------------------------------------------
# This is the cmake project file for EWS, derived from the code from
# http://www.itk.org/Wiki/BundleUtilitiesExample
#
# To have CMake pick a Qt installation of your choice that won't be found
# automatically, set the CMAKE_PREFIX_PATH environment variable.
# For example: "export CMAKE_PREFIX_PATH=/usr/local/trolltech/qt4.3.5"
#  

project(texture_image)
cmake_minimum_required(VERSION 2.8)

set(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_BINARY_DIR})


#set(BSVT_MODULES app/widget app/drawable app/model physics geometry util test)
add_definitions("-g")
#--------------------------------------------------------------------------------
# This is the list of source files that make up the physics engine
#--------------------------------------------------------------------------------
file(GLOB SRCS *.cpp)
include_directories(../ .)
set(OSG_COMPONENTS osg osgUtil osgViewer osgGA osgManipulator osgText osgDB osgWidget osgShadow)
find_package(OpenSceneGraph REQUIRED COMPONENTS ${OSG_COMPONENTS})
#-------------------------------------------------------------------------------


add_executable(texture_image ${SRCS} )
#--------------------------------------------------------------------------------


##message("libs:  ${ALL_QT_OSG_LIBS}")

target_link_libraries(texture_image ${OPENSCENEGRAPH_LIBRARIES} cv highgui )
SET( EXECUTABLE_OUTPUT_PATH  ${PROJECT_SOURCE_DIR} )
