ADD_EXECUTABLE( meshtohmap main.cpp matrix.cpp mesh2hmap.cpp output.cpp parser.cpp )

# Link test to libsnapper library 
#TARGET_LINK_LIBRARIES(  mb2osg 	)
INSTALL_TARGETS( /bin meshtohmap )
