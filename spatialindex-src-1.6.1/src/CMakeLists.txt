#############################################################
# spatial indexing library

# add path prefix to every specified file
MACRO(PATH_PREFIX OUTPUT PREFIX)
  FOREACH(F ${ARGN})
    SET(${OUTPUT} ${${OUTPUT}} ${PREFIX}/${F})
  ENDFOREACH(F)
ENDMACRO(PATH_PREFIX)

# tools library
PATH_PREFIX(TOOLS_SRC tools   Tools.cc)
#PATH_PREFIX(GEOM_SRC geometry LineSegment.cc Point.cc Region.cc)

# spatial index library
PATH_PREFIX(SPINDEX_SRC spatialindex Point.cc Region.cc LineSegment.cc MovingPoint.cc MovingRegion.cc TimePoint.cc TimeRegion.cc SpatialIndexImpl.cc SpatialIndexImpl.h)
PATH_PREFIX(STMAN_SRC storagemanager Buffer.h Buffer.cc DiskStorageManager.cc MemoryStorageManager.cc RandomEvictionsBuffer.cc DiskStorageManager.h MemoryStorageManager.h RandomEvictionsBuffer.h
)
PATH_PREFIX(RTREE_SRC rtree BulkLoader.cc Index.cc Leaf.cc Node.cc RTree.cc Statistics.cc)
PATH_PREFIX(TPRTREE_SRC tprtree Index.cc Leaf.cc Node.cc TPRTree.cc Statistics.cc)
PATH_PREFIX(MVRTREE_SRC mvrtree Index.cc Leaf.cc Node.cc MVRTree.cc Statistics.cc)
SET(SPINDEX_SRC ${TOOLS_SRC} ${GEOM_SRC} ${SPINDEX_SRC} ${STMAN_SRC} ${RTREE_SRC} ${TPRTREE_SRC} ${MVRTREE_SRC})
add_definitions("-DHAVE_FCNTL_H=1 -DHAVE_UNISTD_H=1 -DHAVE_SYS_TYPES_H=1 -DHAVE_SYS_STAT_H=1 -DHAVE_PTHREAD_H=1 -DHAVE_SYS_RESOURCE_H=1 -DHAVE_SYS_TIME_H=1 -DHAVE_STDINT_H=1 -DHAVE_GETTIMEOFDAY=1 -DHAVE_BZERO=1 -DHAVE_MEMSET=1 -DHAVE_MEMCPY=1 -DHAVE_BCOPY=1")
ADD_LIBRARY( spindex STATIC ${SPINDEX_SRC} )

PATH_PREFIX(INDEX_SRC spatialindex ${SPINDEX_SRC})
