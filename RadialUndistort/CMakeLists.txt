ADD_EXECUTABLE(RadialUndistortBin RadialUndistort.cpp LoadJPEG.cpp)
TARGET_LINK_LIBRARIES(RadialUndistort jpeg)
include_directories(imagelib matrix)
subdirs(matrix imagelib)
