file(GLOB SRCS *.cc)
ADD_EXECUTABLE(volfill ${SRCS})
SET(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/vrip/bin)
target_link_libraries(volfill ${OPENGL_LIBRARY})
