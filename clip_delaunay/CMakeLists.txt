ADD_EXECUTABLE( clip_delaunay clip_delaunay.cpp ../tridecimator/plylib.cpp )
SET( EXECUTABLE_OUTPUT_PATH  ${PROJECT_SOURCE_DIR}/bin )
# Link test to libsnapper library 
TARGET_LINK_LIBRARIES(  clip_delaunay  ${OPENGL_LIBRARIES} ${OSG_LIBRARY} 	${OSGGA_LIBRARY} ${OSGVIEWER_LIBRARY} ${OSGTEXT_LIBRARY} ${OSGSIM_LIBRARY} ${OSGFX_LIBRARY} tiff geotiff )
INSTALL_TARGETS( /bin clip_delaunay )
