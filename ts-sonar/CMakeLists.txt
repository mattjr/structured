file(GLOB implem_files *.cpp)
file(GLOB header_files *.h)

set(LIB_NAME ${THREADED_STEREO_LIB}-sonar)

include_directories (${EIGEN3_INCLUDE_DIRS})
add_library (${LIB_NAME} SHARED ${implem_files})
