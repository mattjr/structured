ADD_EXECUTABLE( grd2ply grd2ply.cpp )

SET( EXECUTABLE_OUTPUT_PATH  ${PROJECT_SOURCE_DIR}/bin )
# Link test to libsnapper library 
TARGET_LINK_LIBRARIES(  grd2ply ${GMT_LIBRARY} plankton 	)
INSTALL_TARGETS( /bin grd2ply )
