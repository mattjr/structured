ADD_EXECUTABLE( osgretex osgretex.cpp SnapshotCallback.cpp Path.cpp ar.cpp )

# Link test to libsnapper library 
TARGET_LINK_LIBRARIES(  osgretex 	${OSG_LIBRARY} 	${OSGGA_LIBRARY} ${OSGVIEWER_LIBRARY} ${OSGTEXT_LIBRARY} ${OSGSIM_LIBRARY} ${OSGFX_LIBRARY} tiff geotiff )
INSTALL_TARGETS( /bin osgretex )
