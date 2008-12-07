
ADD_EXECUTABLE( meshtoxy meshtoxy.cpp)

SET( EXECUTABLE_OUTPUT_PATH  ${PROJECT_SOURCE_DIR}/bin )

# Link to seabedcommon library 
TARGET_LINK_LIBRARIES( meshtoxy seabedcommon gts plankton)
