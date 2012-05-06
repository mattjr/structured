# 
# poisson

PROJECT(tridecimator)

find_package(CGAL COMPONENTS Core REQUIRED)
IF(CGAL_FOUND)
  INCLUDE( ${CGAL_USE_FILE})


ELSE(CGAL_FOUND)
MESSAGE(FATAL_ERROR "CGAL not found. Please set CGAL_DIR.")
ENDIF(CGAL_FOUND)

INCLUDE_DIRECTORIES(vcglib .. ${CGAL_INCLUDE_DIRS} )
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
ADD_EXECUTABLE( sw-shadevis sw-shadevis vcglib/wrap/ply/plylib.cpp vcglib/wrap/gui/trackball.cpp vcglib/wrap/gui/trackmode.cpp  ../swrender/render_utils.cpp ../swrender/generic_fragment_shader.cpp ../swrender/generic_vertex_shader.cpp)
ADD_EXECUTABLE( reorder  reorderOutOfCoreImage.cpp  GenParam.cpp vcglib/wrap/ply/plylib.cpp vcglib/wrap/gui/trackball.cpp vcglib/wrap/gui/trackmode.cpp  ../swrender/render_utils.cpp ../swrender/generic_fragment_shader.cpp ../swrender/generic_vertex_shader.cpp)

IF( APPLE )
SET(GLEW_LIB glew)
ELSE(APPLE)
SET(GLEW_LIB GLEW)
ENDIF(APPLE)
TARGET_LINK_LIBRARIES(shadevis  ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} ${GLEW_LIB})
TARGET_LINK_LIBRARIES(sw-shadevis  ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} ${GLEW_LIB} swrender)
TARGET_LINK_LIBRARIES(reorder  ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} ${GLEW_LIB} swrender ${CGAL_3RD_PARTY_LIBRARIES} ${CGAL_LIBRARIES}  )
INSTALL_TARGETS( /bin texturedDecimator tridecimator)
INSTALL_TARGETS( /bin shadevis )
SET( EXECUTABLE_OUTPUT_PATH  ${PROJECT_SOURCE_DIR}/bin )




