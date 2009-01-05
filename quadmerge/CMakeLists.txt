ADD_EXECUTABLE( quadmerge quadmerge.cpp envelope.cpp raster.cpp geometry.cpp uquadtree.cpp fileio.cpp resample.cpp)

TARGET_LINK_LIBRARIES( quadmerge ${GMT_LIBRARY} ${OPENGL_gl_LIBRARY} nn ${LIBPLANKTON_LIB} )

SET( EXECUTABLE_OUTPUT_PATH  ${PROJECT_SOURCE_DIR}/bin )
