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


////////////////////////////////////////////////////////////////////////////////
//
// IntersectKdTree
//
struct IntersectKdTreeBbox
{
    IntersectKdTreeBbox(const osg::Vec3Array& vertices,
                    const osg::KdTree::KdNodeList& nodes,
                    const osg::KdTree::TriangleList& triangles):
                        _vertices(vertices),
                        _kdNodes(nodes),
                        _triangles(triangles)
    {
      _new_triangles =  new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES, 0);
      _new_vertices = new osg::Vec3Array;
    }
    enum OverlapMode{
        GAP,
        DUP,
        CUT

    };

    void intersect(const osg::KdTree::KdNode& node, const osg::BoundingBox clipbox,const OverlapMode &mode) const;
    //bool intersectAndClip(osg::Vec3& s, osg::Vec3& e, const osg::BoundingBox& bb) const;
    const osg::Vec3Array&               _vertices;
    const osg::KdTree::KdNodeList&           _kdNodes;
    const osg::KdTree::TriangleList&         _triangles;
    osg::DrawElementsUInt * _new_triangles;
    osg::Vec3Array *   _new_vertices;





protected:

    IntersectKdTreeBbox& operator = (const IntersectKdTreeBbox&) { return *this; }
};
class KdTreeBbox : public osg::KdTree {


public:
    KdTreeBbox(const KdTree& rhs) : KdTree(rhs){}
    osg::ref_ptr<osg::Node> intersect(const osg::BoundingBox bbox,const IntersectKdTreeBbox::OverlapMode &overlapmode) const
    {
        if (_kdNodes.empty())
        {
            OSG_NOTICE<<"Warning: _kdTree is empty"<<std::endl;
            return false;
        }



        IntersectKdTreeBbox intersector(*_vertices,
                                    _kdNodes,
                                    _triangles
                                    );
        osg::ref_ptr<osg::Geode> newGeode=new osg::Geode;
        osg::Geometry *new_geom=new osg::Geometry;
        newGeode->addDrawable(new_geom);
        intersector.intersect(getNode(0), bbox,overlapmode);
        new_geom->addPrimitiveSet(intersector._new_triangles);
        new_geom->setVertexArray(intersector._new_vertices);

        return newGeode;
    }
};


#endif
