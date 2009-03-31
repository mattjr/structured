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

SET(ROBUST_SRC robust/RobustStats.cpp robust/Exception.cpp)
#ADD_LIBRARY( robust ${ROBUST_SRC} )
ADD_LIBRARY( quadm envelope.cpp raster.cpp geometry.cpp uquadtree.cpp fileio.cpp resample.cpp shadowmap.cpp sample.cpp vpRobust.cpp ${ROBUST_SRC} )
ADD_DEFINITIONS("-DTRILIBRARY")
ADD_LIBRARY( nn  ${NN_SRC})
ADD_EXECUTABLE( nnbathy nn/nnbathy.c)
ADD_EXECUTABLE( oldmerge oldmerge.cpp envelope.cpp raster.cpp )
ADD_EXECUTABLE( quadmerge quadmerge.cpp  )

TARGET_LINK_LIBRARIES( quadmerge quadm ${GMT_LIBRARY} ${OPENGL_gl_LIBRARY} nn ${LIBPLANKTON_LIB} hmap )
TARGET_LINK_LIBRARIES( oldmerge quadm  ${GMT_LIBRARY} nn  ${LIBPLANKTON_LIB} ${OPENGL_gl_LIBRARY})

TARGET_LINK_LIBRARIES( nnbathy nn )
INSTALL_TARGETS( /bin quadmerge )
SET( EXECUTABLE_OUTPUT_PATH  ${PROJECT_SOURCE_DIR}/bin )
