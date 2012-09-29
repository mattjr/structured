// This code is in the public domain -- castanyo@yahoo.es

#ifndef NV_MESH_RASTER_H
#define NV_MESH_RASTER_H

/** @file Raster.h
 * @brief Rasterization library.
 *
 * This is just a standard scanline rasterizer that I took from one of my old
 * projects. The perspective correction wasn't necessary so I just removed it.
**/
/*
#include <nvmesh/nvmesh.h>
#include <nvmath/Vector.h>
*/
#define RASTER_ANTIALIAS true
#define RASTER_NOAA false
#include <osg/Vec2>
#include <osg/Vec3>

	namespace Raster 
	{
		/// A callback to sample the environment.
    typedef void ( * SamplingCallback)(void * param, int x, int y, const osg::Vec3& bar, const osg::Vec3& dx, const osg::Vec3& dy, float coverage);

		// Process the given triangle.
         bool drawTriangle(bool antialias, const osg::Vec2 & extents, const osg::Vec2 v[3], SamplingCallback cb, void * param);

		// Process the given quad.
        //NVMESH_API bool drawQuad(bool antialias, Vector2::Arg extents, const Vector2 vertex[4], SamplingCallback, void * param);

	}


#endif // NV_MESH_RASTER_H
