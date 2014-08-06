// This code is in the public domain -- castanyo@yahoo.es

/** @file Raster.cpp
 * @brief Triangle rasterization library using affine interpolation. Not
 * specially optimized, but enough for my purposes.
**/

/*#include <nvcore/nvcore.h> // swap
#include <nvcore/Containers.h> // min, max
#include <math.h> // round
#include <nvmesh/raster/ClippedTriangle.h>
#include <nvimage/HoleFilling.h>
#include <nvimage/FloatImage.h>
*/
#include "ClippedTriangle.h"

#include "Raster.h"
#include <stdlib.h>
#include <algorithm>
#include <osg/io_utils>
#define RA_EPSILON		0.00001f

//using namespace nv;
//using namespace nv::Raster;
using namespace Raster;
#define Vector2 osg::Vec2
#define Vector3 osg::Vec3

namespace Raster
{
	static inline float delta(float bot, float top, float ih)
	{
		return (bot - top) * ih;
	}

    static inline Vector2 delta(const Vector2& bot, const Vector2& top, float ih)
	{
		return (bot - top) * ih;
	}

    static inline Vector3 delta(const Vector3& bot, const Vector3& top, float ih)
	{
		return (bot - top) * ih;
	}

	static inline int iround(float f)
	{
		// @@ Optimize this.
		return int(floorf(f+0.5f));
		//return int(round(f));
		//return int(f);
	}
	
    static inline int min(int a, int b, int c)
    {
        return std::min(std::min(a, b), c);
    }

    static inline int max(int a, int b, int c)
    {
        return std::max(std::max(a, b), c);
    }

    static inline float min(float a, float b, float c)
    {
        return std::min(std::min(a, b), c);
    }

    static inline float max(float a, float b, float c)
    {
        return std::max(std::max(a, b), c);
    }

    static inline float max(float a, float b)
    {
        return std::max(a, b);
    }

    static inline float min(float a, float b)
    {
        return std::min(a, b);
    }


	/// A triangle vertex. 
	struct Vertex
	{
		Vector2 pos;	// Position.
		Vector3 tex;	// Texcoord. (Barycentric coordinate)
	};


	/// A triangle for rasterization.
	struct Triangle
	{
        Triangle(const Vector2& v0, const Vector2& v1, const Vector2& v2, const Vector3& t0, const Vector3& t1, const Vector3& t2);

		bool computeDeltas();

		void draw(const Vector2 & extents, SamplingCallback cb, void * param);
		void drawAA(const Vector2 & extents, SamplingCallback cb, void * param);
		void flipBackface();
		void computeUnitInwardNormals();
	
		// Vertices.	
		Vector2 v1, v2, v3;
		Vector2 n1, n2, n3; // unit inward normals
		Vector3 t1, t2, t3;

		// Deltas.
		Vector3 dx, dy;
		
		float sign;
		bool valid;
	};


	/// Triangle ctor.
    Triangle::Triangle(const Vector2& v0, const Vector2& v1, const Vector2& v2,
        const Vector3& t0, const Vector3& t1, const Vector3& t2)
	{
		// Init vertices.
		this->v1 = v0;
		this->v2 = v2;
		this->v3 = v1;
	
		// Set barycentric coordinates.
		this->t1 = t0;
		this->t2 = t2;
		this->t3 = t1;

		// make sure every triangle is front facing.
		flipBackface();
	
		// Compute deltas.
		valid = computeDeltas();

		computeUnitInwardNormals();
	}


	/// Compute texture space deltas.
	/// This method takes two edge vectors that form a basis, determines the 
	/// coordinates of the canonic vectors in that basis, and computes the 
	/// texture gradient that corresponds to those vectors.
	bool Triangle::computeDeltas()
	{
		Vector2 e0 = v3 - v1;
		Vector2 e1 = v2 - v1;
		
		Vector3 de0 = t3 - t1;
		Vector3 de1 = t2 - t1;
        float d = (e0.y() * e1.x() - e1.y() * e0.x());
        if(d == 0.0)
            return  false;

        float denom = 1.0f / d;
        if (!std::isfinite(denom)) {
			return false;
		}
		
		float lambda1 = - e1.y() * denom;
		float lambda2 = e0.y() * denom;
		float lambda3 = e1.x() * denom;
		float lambda4 = - e0.x() * denom;
		
		dx = de0 * lambda1 + de1 * lambda2;
		dy = de0 * lambda3 + de1 * lambda4;
		
		return true;

	}

	// compute unit inward normals for each edge.
	void Triangle::computeUnitInwardNormals()
	{
        float d;
        n1 = v1 - v2; n1 = Vector2(-n1.y(), n1.x()); d = sqrtf(n1.x()*n1.x() + n1.y()*n1.y()); n1 = n1 * ((d != 0.0) ? (1.0f/d) : 0.0);
        n2 = v2 - v3; n2 = Vector2(-n2.y(), n2.x()); d = sqrtf(n2.x()*n2.x() + n2.y()*n2.y()); n2 = n2 * ((d != 0.0) ? (1.0f/d) : 0.0);
        n3 = v3 - v1; n3 = Vector2(-n3.y(), n3.x()); d = sqrtf(n3.x()*n3.x() + n3.y()*n3.y()); n3 = n3 * ((d != 0.0) ? (1.0f/d) : 0.0);
	}

	// From cbloom's galaxy:
	/*float Triangle::pixelIntersection(const Vector2 & pixelPos) const
	{
		Poly2 poly;
		poly.addVertex( Vector2(pixelPos.x() - 0.5f, pixelPos.y() - 0.5f) );
		poly.addVertex( Vector2(pixelPos.x() + 0.5f, pixelPos.y() - 0.5f) );
		poly.addVertex( Vector2(pixelPos.x() + 0.5f, pixelPos.y() + 0.5f) );
		poly.addVertex( Vector2(pixelPos.x() - 0.5f, pixelPos.y() + 0.5f) );
		nvCheck( equal(poly.area(), 1.0f) );

		Poly2 poly2;
		if (!Poly2::clipPoly(poly, &poly2, edgePlane[0])) {
			return 0.0f;
		}
		if (!Poly2::clipPoly(poly2, &poly, edgePlane[1])) {
			return 0.0f;
		}
		if (!Poly2::clipPoly(poly, &poly2, edgePlane[2])) {
			return 0.0f;
		}
		
		float ret = poly2.area();
		//nvCheck(fiszerotoone(ret));
		
		return clamp(ret, 0.0f, 1.0f);
	}*/
	
	void Triangle::flipBackface()
	{
		// check if triangle is backfacing, if so, swap two vertices
		if ( ((v3.x()-v1.x())*(v2.y()-v1.y()) - (v3.y()-v1.y())*(v2.x()-v1.x())) < 0 ) {
			Vector2 hv=v1; v1=v2; v2=hv; // swap pos
			Vector3 ht=t1; t1=t2; t2=ht; // swap tex
		}
	}
#if 1
	void Triangle::draw(const Vector2 & extents, SamplingCallback cb, void * param)
	{
		// 28.4 fixed-point coordinates
		const int Y1 = iround(16.0f * v1.y());
		const int Y2 = iround(16.0f * v2.y());
		const int Y3 = iround(16.0f * v3.y());
	
		const int X1 = iround(16.0f * v1.x());
		const int X2 = iround(16.0f * v2.x());
		const int X3 = iround(16.0f * v3.x());
	
		// Deltas
		const int DX12 = X1 - X2;
		const int DX23 = X2 - X3;
		const int DX31 = X3 - X1;
	
		const int DY12 = Y1 - Y2;
		const int DY23 = Y2 - Y3;
		const int DY31 = Y3 - Y1;
	
		// Fixed-point deltas
		const int FDX12 = DX12 << 4;
		const int FDX23 = DX23 << 4;
		const int FDX31 = DX31 << 4;
	
		const int FDY12 = DY12 << 4;
		const int FDY23 = DY23 << 4;
		const int FDY31 = DY31 << 4;

		int frustumX0 =  0 << 4;
		int frustumY0 =  0 << 4;
		int frustumX1 =  (int)extents.x() << 4;
		int frustumY1 =  (int)extents.y() << 4;
		
		// Bounding rectangle
        int minx = (std::max(min(X1, X2, X3), frustumX0) + 0xF) >> 4;
        int miny = (std::max(min(Y1, Y2, Y3), frustumY0) + 0xF) >> 4;
        int maxx = (std::min(max(X1, X2, X3), frustumX1) + 0xF) >> 4;
        int maxy = (std::min(max(Y1, Y2, Y3), frustumY1) + 0xF) >> 4;
	
		// Block size, standard 8x8 (must be power of two)
		const int q = 8;
	
		// Start in corner of 8x8 block
		minx &= ~(q - 1);
		miny &= ~(q - 1);
	
		// Half-edge constants
		int C1 = DY12 * X1 - DX12 * Y1;
		int C2 = DY23 * X2 - DX23 * Y2;
		int C3 = DY31 * X3 - DX31 * Y3;
	
		// Correct for fill convention
		if(DY12 < 0 || (DY12 == 0 && DX12 > 0)) C1++;
		if(DY23 < 0 || (DY23 == 0 && DX23 > 0)) C2++;
		if(DY31 < 0 || (DY31 == 0 && DX31 > 0)) C3++;
		
		// Loop through blocks
		for(int y = miny; y < maxy; y += q)
		{
			for(int x = minx; x < maxx; x += q)
			{
				// Corners of block
				int x0 = x << 4;
                float x0_upscaled=(x0/16.0f)+0.5;

				int x1 = (x + q - 1) << 4;
				int y0 = y << 4;
                float y0_upscaled=(y0/16.0f)+0.5;
				int y1 = (y + q - 1) << 4;
	
				// Evaluate half-space functions
				bool a00 = C1 + DX12 * y0 - DY12 * x0 > 0;
				bool a10 = C1 + DX12 * y0 - DY12 * x1 > 0;
				bool a01 = C1 + DX12 * y1 - DY12 * x0 > 0;
				bool a11 = C1 + DX12 * y1 - DY12 * x1 > 0;
				int a = (a00 << 0) | (a10 << 1) | (a01 << 2) | (a11 << 3);
		
				bool b00 = C2 + DX23 * y0 - DY23 * x0 > 0;
				bool b10 = C2 + DX23 * y0 - DY23 * x1 > 0;
				bool b01 = C2 + DX23 * y1 - DY23 * x0 > 0;
				bool b11 = C2 + DX23 * y1 - DY23 * x1 > 0;
				int b = (b00 << 0) | (b10 << 1) | (b01 << 2) | (b11 << 3);
		
				bool c00 = C3 + DX31 * y0 - DY31 * x0 > 0;
				bool c10 = C3 + DX31 * y0 - DY31 * x1 > 0;
				bool c01 = C3 + DX31 * y1 - DY31 * x0 > 0;
				bool c11 = C3 + DX31 * y1 - DY31 * x1 > 0;
				int c = (c00 << 0) | (c10 << 1) | (c01 << 2) | (c11 << 3);
	
				// Skip block when outside an edge
				if(a == 0x0 || b == 0x0 || c == 0x0) continue;		
		
				// Accept whole block when totally covered
				if(a == 0xF && b == 0xF && c == 0xF)
				{
                    //Vector3 texRow = t1 + dy*(y0 - v1.y()) + dx*(x0 - v1.x());
                    Vector3 texRow = t1 + dy*(y0_upscaled - v1.y()) + dx*(x0_upscaled - v1.x());
					for(int iy = y; iy < y + q; iy++)
					{
						Vector3 tex = texRow;
						for(int ix = x; ix < x + q; ix++)
						{
                            //Vector3 tex = t1 + dx * (ix - v1.x()) + dy * (iy - v1.y());
							cb(param, ix, iy, tex, dx, dy, 1.0);
							tex += dx;
						}
						texRow += dy;
					}
				}
				else // Partially covered block
				{
					int CY1 = C1 + DX12 * y0 - DY12 * x0;
					int CY2 = C2 + DX23 * y0 - DY23 * x0;
					int CY3 = C3 + DX31 * y0 - DY31 * x0;
                    //Vector3 texRow = t1 + dy*(y0 - v1.y()) + dx*(x0 - v1.x());
                    Vector3 texRow = t1 + dy*(y0_upscaled - v1.y()) + dx*(x0_upscaled - v1.x());

					for(int iy = y; iy < y + q; iy++)
					{
						int CX1 = CY1;
						int CX2 = CY2;
						int CX3 = CY3;
						Vector3 tex = texRow;
	                  	
						for(int ix = x; ix < x + q; ix++)
						{
							if(CX1 > 0 && CX2 > 0 && CX3 > 0)
							{
								cb(param, ix, iy, tex, dx, dy, 1.0);
							}
	
							CX1 -= FDY12;
							CX2 -= FDY23;
							CX3 -= FDY31;
							tex += dx;
						}
	
						CY1 += FDX12;
						CY2 += FDX23;
						CY3 += FDX31;
						texRow += dy;
					}
				}
			}
		}
	}

#endif
#define PX_INSIDE    1.0f/sqrt(2.0f)
#define PX_OUTSIDE  -1.0f/sqrt(2.0f)

#define BK_SIZE 8
#define BK_INSIDE   sqrt(BK_SIZE*BK_SIZE/2.0f)
#define BK_OUTSIDE -sqrt(BK_SIZE*BK_SIZE/2.0f)

	void Triangle::drawAA(const Vector2 & extents, SamplingCallback cb, void * param)
		// extents has to be multiple of BK_SIZE!!
	{
		// Bounding rectangle
		float minx = floorf(max(min(v1.x(), v2.x(), v3.x()), 0.0f));
		float miny = floorf(max(min(v1.y(), v2.y(), v3.y()), 0.0f));
		float maxx = ceilf( min(max(v1.x(), v2.x(), v3.x()), extents.x()-1.0f));
		float maxy = ceilf( min(max(v1.y(), v2.y(), v3.y()), extents.y()-1.0f));

		minx = (float)(((int)minx) & (~((int)BK_SIZE - 1))); // align to blocksize (we don't need to worry about blocks partially out of
		miny = (float)(((int)miny) & (~((int)BK_SIZE - 1))); // frustum
	
		minx += 0.5; miny +=0.5;  // sampling at texel centers!
		maxx += 0.5; maxy +=0.5; 

		// Half-edge constants
		float C1 = n1.x() * (-v1.x()) + n1.y() * (-v1.y());
		float C2 = n2.x() * (-v2.x()) + n2.y() * (-v2.y());
		float C3 = n3.x() * (-v3.x()) + n3.y() * (-v3.y());
	
		// Loop through blocks
		for(float y0 = miny; y0 <= maxy; y0 += BK_SIZE)
		{
			for(float x0 = minx; x0 <= maxx; x0 += BK_SIZE)
			{
				// Corners of block
				float xc = (x0 + (BK_SIZE-1)/2.0f);
				float yc = (y0 + (BK_SIZE-1)/2.0f);
	
				// Evaluate half-space functions
				float aC = C1 + n1.x() * xc + n1.y() * yc;
				float bC = C2 + n2.x() * xc + n2.y() * yc;
				float cC = C3 + n3.x() * xc + n3.y() * yc;

				// Skip block when outside an edge
				if( (aC <= BK_OUTSIDE) || (bC <= BK_OUTSIDE) || (cC <= BK_OUTSIDE) ) continue;
				
				// Accept whole block when totally covered
				if( (aC >= BK_INSIDE) && (bC >= BK_INSIDE) && (cC >= BK_INSIDE) )
				{
					Vector3 texRow = t1 + dy*(y0 - v1.y()) + dx*(x0 - v1.x());

					for (float y = y0; y < y0 + BK_SIZE; y++)
					{
						Vector3 tex = texRow;
						for(float x = x0; x < x0 + BK_SIZE; x++)
						{
							cb(param, (int)x, (int)y, tex, dx, dy, 1.0f);
							tex += dx;
						}
						texRow += dy;
					}
				}
				else // Partially covered block
				{
					float CY1 = C1 + n1.x() * x0 + n1.y() * y0;
					float CY2 = C2 + n2.x() * x0 + n2.y() * y0;
					float CY3 = C3 + n3.x() * x0 + n3.y() * y0;
					Vector3 texRow = t1 + dy*(y0 - v1.y()) + dx*(x0 - v1.x());	                  	
	
					for(float y = y0; y < y0 + BK_SIZE; y++)
					{
						float CX1 = CY1;
						float CX2 = CY2;
						float CX3 = CY3;
						Vector3 tex = texRow;

						for (float x = x0; x < x0 + BK_SIZE; x++)
						{
							if (CX1 >= PX_INSIDE && CX2 >= PX_INSIDE && CX3 >= PX_INSIDE) 
							{
								// pixel completely covered
								Vector3 tex = t1 + dx * (x - v1.x()) + dy * (y - v1.y());
								cb(param, (int)x, (int)y, tex, dx, dy, 1.0f);
							}
							else if ((CX1 >= PX_OUTSIDE) && (CX2 >= PX_OUTSIDE) && (CX3 >= PX_OUTSIDE))
							{
								// triangle partially covers pixel. do clipping.
								ClippedTriangle ct(v1-Vector2(x,y), v2-Vector2(x,y), v3-Vector2(x,y));
								ct.clipAABox(-0.5, -0.5, 0.5, 0.5);
								Vector2 centroid = ct.centroid();
								float area = ct.area();
								if (area > 0.0f)
								{
									//Vector3 texCent = tex + dx*centroid.x() + dy*centroid.y();
									Vector3 texCent = t1 + dx * (x - v1.x()) + dy * (y - v1.y());
									cb(param, (int)x, (int)y, texCent, dx, dy, area); 
								}
							}
	
							CX1 += n1.x();
							CX2 += n2.x();
							CX3 += n3.x();
							tex += dx;
						}
	
						CY1 += n1.y();
						CY2 += n2.y();
						CY3 += n3.y();
						texRow += dy;
					}
				}
			}
		}
	}

} // namespace


/// Process the given triangle.
bool Raster::drawTriangle(bool antialias, const Vector2& extents, const Vector2 v[3],const Vector2 t[3], SamplingCallback cb, void * param)
{
    Triangle tri(v[0], v[1], v[2], Vector3(t[0].x(), t[0].y(), 0), Vector3(t[1].x(), t[1].y(), 0), Vector3(t[2].x(), t[2].y(), 0));
	if (tri.valid) {
        if (antialias) {
			tri.drawAA(extents, cb, param);		
        } else {
            tri.draw(extents, cb, param);
        }
		return true;
	}
	return false;
}

inline static float triangleArea(Vector2& v1, Vector2& v2, Vector2& v3)
{
	return 0.5f * (v3.x() * v1.y() + v1.x() * v2.y() + v2.x() * v3.y() - v2.x() * v1.y() - v3.x() * v2.y() - v1.x() * v3.y());
}




