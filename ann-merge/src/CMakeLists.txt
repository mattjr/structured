# sources which should be compiled into the library
SET( SRCS ann_merge.cpp )

# Build the library on 'make'
ADD_EXECUTABLE( ann_merge ${SRCS} )

# Install the library on 'make install'
INSTALL_TARGETS( /bin ann_merge )

