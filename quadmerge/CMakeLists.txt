SET(NN_SRC nn/delaunay.c
nn/hash.c
nn/istack.c
nn/lpi.c
nn/minell.c
nn/nnai.c
nn/nncommon.c
nn/nncommon-vulnerable.c
nn/nnpi.c
nn/preader.c
nn/triangle.c
)
ADD_DEFINITIONS("-DTRILIBRARY")
ADD_LIBRARY( nn  ${NN_SRC})
ADD_EXECUTABLE( quadmerge quadmerge.cpp envelope.cpp raster.cpp geometry.cpp uquadtree.cpp fileio.cpp resample.cpp shadowmap.cpp sample.cpp)

TARGET_LINK_LIBRARIES( quadmerge ${GMT_LIBRARY} ${OPENGL_gl_LIBRARY} nn ${LIBPLANKTON_LIB} )
INSTALL_TARGETS( /bin quadmerge )
SET( EXECUTABLE_OUTPUT_PATH  ${PROJECT_SOURCE_DIR}/bin )
