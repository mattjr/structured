/* -*-c++-*- OpenSceneGraph - Copyright (C) 1998-2006 Robert Osfield 
 *
 * This library is open source and may be redistributed and/or modified under  
 * the terms of the OpenSceneGraph Public License (OSGPL) version 0.0 or 
 * (at your option) any later version.  The full license is in LICENSE file
 * included with this distribution, and on the openscenegraph.org website.
 * 
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the 
 * OpenSceneGraph Public License for more details.
*/

#ifndef OSGUTIL_SIMPLIFIER
#define OSGUTIL_SIMPLIFIER 1

#include <osg/NodeVisitor>
#include <osg/Geode>
#include <osg/Geometry>

#include <osgUtil/Export>
#include <osg/KdTree>

#include <stdio.h>
////////////////////////////////////////////////////////////////////////////////
//
// IntersectKdTree
//
const int maxNumTC=4;
typedef std::vector<osg::ref_ptr<osg::Vec3Array> > TexBlendCoord;
struct IntersectKdTreeBbox
{
    IntersectKdTreeBbox(const osg::Vec3Array& vertices,
                        const osg::KdTree::KdNodeList& nodes,
                        const osg::KdTree::TriangleList& triangles,bool multTex):
    _vertices(vertices),
    _kdNodes(nodes),
    _triangles(triangles),
    _new_texid(NULL)
    {
        _new_triangles =  new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES, 0);
        _new_vertices = new osg::Vec3Array;
        int texCoordSize= multTex ? 4:1;
        _new_texcoords.resize(texCoordSize);
        for(int f=0; f< texCoordSize; f++){
            _new_texcoords[f]=new osg::Vec3Array;
        }
        if(multTex)
            _new_texid = new osg::Vec4Array;
        _gapPts=new osg::Vec3Array;

    }
    /* ~IntersectKdTreeBbox(){
        delete _new_triangles;
        delete _new_vertices;
        delete _new_texcoords;
        delete _new_texid;
    }*/

    enum OverlapMode{
        GAP,
        DUP,
        CUT,
        DUMP

    };

    void intersect(const osg::KdTree::KdNode& node, const osg::BoundingBox clipbox,const OverlapMode &mode) const;
   // void intersect(const osg::KdTree::KdNode& node, osg::Vec4Array* _texid,osg::Vec2Array *__texcoords, const osg::BoundingBox clipbox,const OverlapMode &mode) const;
    void intersect(const osg::KdTree::KdNode& node, osg::Vec4Array* _texid,const TexBlendCoord  &_texcoords, const osg::BoundingBox clipbox,const OverlapMode &mode) const;

    //bool intersectAndClip(osg::Vec3& s, osg::Vec3& e, const osg::BoundingBox& bb) const;
    const osg::Vec3Array&               _vertices;
    const osg::KdTree::KdNodeList&           _kdNodes;
    const osg::KdTree::TriangleList&         _triangles;
    osg::DrawElementsUInt * _new_triangles;
    osg::Vec3Array *   _new_vertices;
    TexBlendCoord  _new_texcoords;
    osg::Vec4Array *   _new_texid;
    osg::Vec3Array *_gapPts;





protected:

    IntersectKdTreeBbox& operator = (const IntersectKdTreeBbox&) { return *this; }
};
class KdTreeBbox : public osg::KdTree {


public:
    KdTreeBbox(const KdTree& rhs) : KdTree(rhs){}
    osg::ref_ptr<osg::Node> intersect(const osg::BoundingBox bbox,const IntersectKdTreeBbox::OverlapMode &overlapmode,osg::Vec3Array *&dumpPts,bool multTex) const;

    osg::ref_ptr<osg::Node> intersect(const osg::BoundingBox bbox,osg::Vec4Array *ids, const TexBlendCoord &texcoord,TexBlendCoord  &new_texcoord,
                                      osg::ref_ptr<osg::Vec4Array> &new_ids,
                                      const IntersectKdTreeBbox::OverlapMode &overlapmode) const;

};


#endif
