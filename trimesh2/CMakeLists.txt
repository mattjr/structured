# Sparse libtrimesh for ply and icp
SET( TRIMESH_SRCS TriMesh_bounding.cc 
                 TriMesh_connectivity.cc 
                 TriMesh_curvature.cc 
                 TriMesh_io.cc 
                 TriMesh_grid.cc 
                 TriMesh_normals.cc 
                 TriMesh_pointareas.cc 
                 TriMesh_stats.cc 
                 TriMesh_tstrips.cc 
                 GLCamera.cc 
                 ICP.cc 
                 KDtree.cc 
                 diffuse.cc 
                 edgeflip.cc 
                 faceflip.cc 
                 filter.cc 
                 lmsmooth.cc 
                 remove.cc 
                 reorder_verts.cc 
                 subdiv.cc
 )
ADD_LIBRARY( trimesh2 STATIC ${TRIMESH_SRCS} )
