/*  
    vertexData.h
    Copyright (c) 2007, Tobias Wolf <twolf@access.unizh.ch>
    All rights reserved.  
    
    Header file of the VertexData class.
*/

/** note, derived from Equalizer LGPL source.*/


#ifndef MESH_VERTEXDATA_H
#define MESH_VERTEXDATA_H


#include <osg/Node>
#include <osg/PrimitiveSet>
#include <osg/BoundingBox>
#include <vector>
#include <osg/Geometry>
#include "Clipper.h"
///////////////////////////////////////////////////////////////////////////////
//!
//! \class VertexData
//! \brief helps to read ply file and converts in to osg::Node format
//!
///////////////////////////////////////////////////////////////////////////////

// defined elsewhere
struct PlyFile;

namespace ply
{
    typedef struct _tri_t{
          int idx[3];
          int pos;
          int tri_idx;
      }tri_t;
    /*  Holds the flat data and offers routines to read, scale and sort it.  */
    class VertexData
    {
    public:
        // Default constructor
        VertexData();
        
        // Reads ply file and convert in to osg::Node and returns the same
        osg::Node* readPlyFile( const char* file, const bool ignoreColors = false,osg::BoundingBox *bbox=NULL,OverlapMode mode=DUP ,bool readQual=false);
        
        // to set the flag for using inverted face
        void useInvertedFaces() { _invertFaces = true; }
        osg::ref_ptr<osg::Vec4Array>   _texIds;
        std::vector<osg::ref_ptr<osg::Vec3Array> >   _texCoord;
        void calcNorms( osg::Vec3Array *norms );

    private:
        // Function which reads all the vertices and colors if color info is
        // given and also if the user wants that information
        void readVertices( PlyFile* file, const int nVertices, 
                           const bool readColors );

        // Reads the triangle indices from the ply file
        void readTriangles( PlyFile* file, const int nFaces, bool multTex ,bool tex);

        // Calculates the normals according to passed flag
        // if vertexNormals is true then computes normal per vertices
        // otherwise per triangle means per face
    protected:
        void _calculateNormals( const bool vertexNormals = true );

        bool        _invertFaces;
        bool qual;
        osg::ref_ptr<osg::Geode>   _geode;
        osg::ref_ptr<osg::Geometry>   _geom;
    public:
        // Vertex array in osg format
        osg::ref_ptr<osg::Vec3Array>   _vertices;
        std::vector<osg::Vec3> _tmp_verts;
        std::vector<osg::Vec4> _tmp_colors;
        std::map<int,std::vector<tri_t> > _img2tri;

        // Color array in osg format
        osg::ref_ptr<osg::Vec4Array>   _colors;
        // Color array in osg format
        osg::BoundingBox * _bbox;
        // Normals in osg format
        osg::ref_ptr<osg::Vec3Array> _normals;
        // The indices of the faces in premitive set
        osg::ref_ptr<osg::DrawElementsUInt> _triangles;
        std::map<int,int> outbboxVert;
        bool tex;
        OverlapMode _mode;
        osg::ref_ptr<osg::Vec2Array> _qualArray;

    };
    class VertexDataMosaic: public VertexData{

    public:

        osg::Node* readPlyFile( const char* file );
        void readTriangles( PlyFile* file, const int nFaces, bool multTex ,bool tex,bool qual);
        void readVertices( PlyFile* file, const int nVertices,
                                       const bool readColors );

    };
}


bool checkIsEmptyPly(const char *filename);

#endif // MESH_VERTEXDATA_H
