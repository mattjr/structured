cmake_minimum_required(VERSION 2.4.6)
    if(COMMAND cmake_policy)
      cmake_policy(SET CMP0003 NEW)
    endif(COMMAND cmake_policy)
# add zlib
include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)
#pkg_check_modules(ZLIB REQUIRED zlib)
#include_directories(${ZLIB_INCLUDE_DIRS})
#link_directories(${ZLIB_LIBRARY_DIRS})

find_package(OpenCV REQUIRED)

# build 3rd party libraries

#include_directories(src/3rdparty/DUtils)
#include_directories(src/3rdparty/DUtilsCV)
#include_directories(src/3rdparty/DVision)
include_directories(include/re_vision)
#include_directories(src/3rdparty/RapidXml)
#include_directories(src/3rdparty/epnp)

# DUtils
add_library(DUtils
  src/3rdparty/DUtils/BinaryFile.cpp     src/3rdparty/DUtils/LineFile.cpp  
  src/3rdparty/DUtils/Random.cpp
  src/3rdparty/DUtils/FileFunctions.cpp  src/3rdparty/DUtils/Profiler.cpp  
  src/3rdparty/DUtils/Timestamp.cpp      src/3rdparty/DUtils/StringFunctions.cpp
)

# DUtilsCV
add_library(DUtilsCV
  src/3rdparty/DUtilsCV/Drawing.cpp   src/3rdparty/DUtilsCV/GUI.cpp  
  src/3rdparty/DUtilsCV/Transformations.cpp
  src/3rdparty/DUtilsCV/Geometry.cpp  src/3rdparty/DUtilsCV/IO.cpp
)

# DVision
add_library(DVision
  src/3rdparty/DVision/PatchFile.cpp       
  src/3rdparty/DVision/PMVSCamera.cpp
  src/3rdparty/DVision/BundleCamera.cpp  src/3rdparty/DVision/PixelPointFile.cpp  
  src/3rdparty/DVision/SurfSet.cpp
  src/3rdparty/DVision/Matches.cpp       src/3rdparty/DVision/PLYFile.cpp
)



include_directories(${CMAKE_CURRENT_BINARY_DIR}
#	src/g2o/EXTERNAL/eigen3
#	src/g2o
#	src/g2o/build
)





#link_directories(${PROJECT_SOURCE_DIR}/src/g2o/lib
#)

# PointCloudModelCreator (several apps)

add_executable(changeReferenceSystem
  src/changeReferenceSystem.cpp
)
target_link_libraries(changeReferenceSystem DVision DUtilsCV DUtils #g2o_core g2o_solver_cholmod g2o_stuff
${OpenCV_LIBRARIES})






