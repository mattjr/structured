file(GLOB SRCS *.cc)
ADD_EXECUTABLE(volfill ${SRCS})
target_link_libraries(volfill ${OPENGL_LIBRARY})