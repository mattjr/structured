ADD_EXECUTABLE( meshtohmap main.cpp matrix.cpp mesh2hmap.cpp output.cpp parser.cpp )

# Link test to libsnapper library 
TARGET_LINK_LIBRARIES(  meshtohmap ${GMT_LIBRARY} plankton 	)
INSTALL_TARGETS( /bin meshtohmap )
