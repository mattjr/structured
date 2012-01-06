# 
# poisson

PROJECT(tridecimator)



INCLUDE_DIRECTORIES(vcglib .. )
SET( SRCS vcglib/wrap/ply/plylib.cpp  quadric_tex_simp.cpp   )




FIND_PACKAGE( GLUT )
IF( NOT GLUT_FOUND )
   MESSAGE( FATAL_ERROR  "ERROR - Can't find GLUT" )
ENDIF( NOT GLUT_FOUND )
ADD_EXECUTABLE( tridecimator  tridecimator.cpp  ${SRCS} )
ADD_EXECUTABLE( mergeMesh mergeMesh.cpp ${SRCS} )
ADD_EXECUTABLE( texturedDecimator  texturedDecimator.cpp ${SRCS} )
TARGET_LINK_LIBRARIES(texturedDecimator ts)
ADD_EXECUTABLE( shadevis shadevis.cpp vcglib/wrap/ply/plylib.cpp vcglib/wrap/gui/trackball.cpp vcglib/wrap/gui/trackmode.cpp  ../swrender/render_utils.cpp  )
IF( APPLE )
SET(GLEW_LIB glew)
ELSE(APPLE)
SET(GLEW_LIB GLEW)
ENDIF(APPLE)
TARGET_LINK_LIBRARIES(shadevis  ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} ${GLEW_LIB})
INSTALL_TARGETS( /bin texturedDecimator tridecimator)
INSTALL_TARGETS( /bin shadevis )
SET( EXECUTABLE_OUTPUT_PATH  ${PROJECT_SOURCE_DIR}/bin )




