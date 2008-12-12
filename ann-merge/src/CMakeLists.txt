# sources which should be compiled into the library
SET( SRCS ann_merge.cpp )
SET( ANN_SRCS 
ann_1.1.1/src/ANN.cpp
ann_1.1.1/src/bd_fix_rad_search.cpp
ann_1.1.1/src/bd_pr_search.cpp
ann_1.1.1/src/bd_search.cpp
ann_1.1.1/src/bd_tree.cpp
ann_1.1.1/src/brute.cpp
ann_1.1.1/src/kd_dump.cpp
ann_1.1.1/src/kd_fix_rad_search.cpp
ann_1.1.1/src/kd_pr_search.cpp
ann_1.1.1/src/kd_search.cpp
ann_1.1.1/src/kd_split.cpp
ann_1.1.1/src/kd_tree.cpp
ann_1.1.1/src/kd_util.cpp
ann_1.1.1/src/perf.cpp

  )

INCLUDE_DIRECTORIES(ann_1.1.1/include/)
ADD_LIBRARY(ANN ${ANN_SRCS} )
# Build the library on 'make'
ADD_EXECUTABLE( ann_merge ${SRCS})

# Install the library on 'make install'
INSTALL_TARGETS( /bin ann_merge )

