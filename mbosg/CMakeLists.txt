ADD_EXECUTABLE( mb2osg mb2osg.cpp )

# Link test to libsnapper library 
TARGET_LINK_LIBRARIES(  mb2osg 	${OSG_LIBRARY} 	${OSGGA_LIBRARY} ${OSGVIEWER_LIBRARY} ${OSGTEXT_LIBRARY} ${OSGSIM_LIBRARY} ${OSGFX_LIBRARY} tiff geotiff ts)
INSTALL_TARGETS( /bin mb2osg )
