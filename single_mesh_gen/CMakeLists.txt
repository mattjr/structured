
ADD_EXECUTABLE( single_mesh_gen single_mesh_gen.cpp )

# Link test to libsnapper library 
TARGET_LINK_LIBRARIES( single_mesh_gen snapper polyp plankton)

INSTALL_TARGETS( /bin single_mesh_gen )
SET( EXECUTABLE_OUTPUT_PATH  ${PROJECT_SOURCE_DIR}/bin )
