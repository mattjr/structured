SET( SRCS CmdLineParser.cpp
		Factor.cpp
		Geometry.cpp
		MarchingCubes.cpp
		Time.cpp
		ply.cpp
		plyfile.cpp
		)
ADD_LIBRARY( poisson STATIC ${SRCS} )
