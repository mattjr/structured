
SET(PBA_CPU_SRCS
    ConfigBA.h
    ConfigBA.cpp
    DataInterface.h
    interface.h
    pba.h
    pba.cpp
    SparseBundleCPU.h
    SparseBundleCPU.cpp
    util.h)
add_definitions(-fpermissive)
add_library(pbaCPU SHARED ${PBA_CPU_SRCS})
SET_TARGET_PROPERTIES(pbaCPU PROPERTIES COMPILE_FLAGS "-DPBA_NO_GPU" LIBRARY_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/../"   ARCHIVE_OUTPUT_DIRECTORY  "${PROJECT_SOURCE_DIR}/../")

if (CUDA_FOUND) 
  SET(PBA_GPU_SRCS
      ConfigBA.h
      ConfigBA.cpp
      DataInterface.h
      interface.h
      pba.h
      pba.cpp
      SparseBundleCU.h
      SparseBundleCU.cpp
      SparseBundleCPU.h
      SparseBundleCPU.cpp
      CuTexImage.h
      CuTexImage.cpp
      ProgramCU.h
      ProgramCU.cu
      util.h)

  CUDA_ADD_LIBRARY(pbaGPU SHARED ${PBA_GPU_SRCS})
    add_custom_command(
    TARGET pbaGPU POST_BUILD
    COMMAND ln ARGS -s -f ${PROJECT_SOURCE_DIR}/../libpbaGPU.so ${PROJECT_SOURCE_DIR}/../libpba.so
    )
else()
    add_custom_command(
    TARGET pbaCPU POST_BUILD
    COMMAND ln ARGS -s -f ${PROJECT_SOURCE_DIR}/../libpbaCPU.so ${PROJECT_SOURCE_DIR}/../libpba.so
    )
endif(CUDA_FOUND) 

set_target_properties(pbaGPU PROPERTIES
       ARCHIVE_OUTPUT_DIRECTORY  "${PROJECT_SOURCE_DIR}/../" LIBRARY_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/../")
