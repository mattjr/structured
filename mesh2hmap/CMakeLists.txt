ADD_LIBRARY( hmap matrix.cpp mesh2hmap.cpp output.cpp parser.cpp )
ADD_EXECUTABLE( mesh2hmap main.cpp )

# Link test to libsnapper library 
TARGET_LINK_LIBRARIES(  mesh2hmap ${GMT_LIBRARY} plankton  hmap	)
SET( EXECUTABLE_OUTPUT_PATH  ${PROJECT_SOURCE_DIR}/bin )

INSTALL_TARGETS( /bin mesh2hmap )
