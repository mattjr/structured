PROJECT(IcpRegisterSurface)
    cmake_minimum_required(VERSION 2.4)

INCLUDE (${CMAKE_ROOT}/Modules/FindVTK.cmake)
 FIND_PACKAGE(VTK)
 IF(VTK_FOUND)
   INCLUDE (${USE_VTK_FILE})
   LINK_LIBRARIES( vtkHybrid vtkRendering  vtkGraphics vtkImaging vtkIO vtkFiltering vtkCommon )
   IF (${VTK_MAJOR_VERSION} GREATER 4)
      LINK_LIBRARIES( vtkWidgets )
   ENDIF (${VTK_MAJOR_VERSION} GREATER 4)
 ENDIF(VTK_FOUND)

INCLUDE_DIRECTORIES( ${IcpRegisterSurface_SOURCE_DIR}
                     ${IcpRegisterSurface_BINARY_DIR} )
ADD_EXECUTABLE(IcpRegisterSurface TestHyperOctreeIO.cxx )
TARGET_LINK_LIBRARIES(IcpRegisterSurface vtkIO vtkHybrid vtkFiltering vtkCommon)
