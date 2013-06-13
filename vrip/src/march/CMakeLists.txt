INCLUDE_DIRECTORIES(vrip/includes/)
ADD_EXECUTABLE( vripmarch main.cc getinfo.cc init.cc edge.cc cube.cc quit.cc fatal.cc slices.cc OccGridRLE.cc SectionRLE.cc  ChunkAllocator.cc ChunkLink.cc OccGrid.cc)
TARGET_LINK_LIBRARIES(vripmarch ply)
