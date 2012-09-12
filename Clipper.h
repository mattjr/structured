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
class geom_elems_dst{
public:
    geom_elems_dst(int numTex){
        colors=new osg::Vec4Array();
        faces = new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES, 0);

        texcoords.resize(numTex);
        if(numTex > 0)
            texcoords[0]=new osg::Vec3Array;
        if(numTex >1){
            texcoords[1]=new osg::Vec3Array;
            texcoords[2]=new osg::Vec3Array;
            texcoords[3]=new osg::Vec3Array;
        }

        if(numTex > 0)
            texid=new osg::Vec4Array;
        else
            texid=NULL;
        vertices= new osg::Vec3Array;
        faces = new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES, 0);
    }
    osg::Vec3Array *vertices;
    osg::Vec4Array *colors;
    osg::DrawElementsUInt *faces;
     osg::ref_ptr<osg::Vec4Array>  texid;
    TexBlendCoord  texcoords;
};

typedef struct _geom_elems_src{
    osg::Vec4Array *colors;
    osg::Vec4Array *texid;
    TexBlendCoord  texcoords;
}geom_elems_src;

struct IntersectKdTreeBbox
{
    IntersectKdTreeBbox(const geom_elems_src src,
                        const osg::ref_ptr<osg::Vec3Array>  vertices,
                        const osg::KdTree::KdNodeList& nodes,
                        const osg::KdTree::TriangleList& triangles
                        ):
        _vertices(vertices),
        _colors(src.colors),
        _texid(src.texid),
        _texcoords(src.texcoords),
        _kdNodes(nodes),
        _triangles(triangles)


    {



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

    void intersect(const osg::KdTree::KdNode& node, const geom_elems_dst dst,const osg::BoundingBox clipbox,const OverlapMode &mode) const;

    //bool intersectAndClip(osg::Vec3& s, osg::Vec3& e, const osg::BoundingBox& bb) const;
    const osg::Vec3Array *               _vertices;
    const osg::Vec4Array *               _colors;
    const osg::Vec4Array *   _texid;
    const TexBlendCoord  _texcoords;

    const osg::KdTree::KdNodeList&           _kdNodes;
    const osg::KdTree::TriangleList&         _triangles;
    const osg::Vec3Array *_gapPts;





protected:

    IntersectKdTreeBbox& operator = (const IntersectKdTreeBbox&) { return *this; }
};
class KdTreeBbox : public osg::KdTree {


public:
    KdTreeBbox(const KdTree& rhs,const geom_elems_src src) : KdTree(rhs),_src(src), intersector(src,_vertices,
            _kdNodes,
            _triangles) {


        if (_kdNodes.empty())
        {
            osg::notify(osg::NOTICE)<<"Warning: _kdTree is empty"<<std::endl;
            return;
        }




    }
    osg::ref_ptr<osg::Node> intersect(const osg::BoundingBox bbox,  geom_elems_dst &dst,
                                      const IntersectKdTreeBbox::OverlapMode &overlapmode);

    const geom_elems_src _src;
    IntersectKdTreeBbox intersector;
};
KdTreeBbox *setupKdTree(osg::ref_ptr<osg::Node> model);

bool cut_model(KdTreeBbox *kdtreeBBox,std::string outfilename,osg::BoundingBox bbox,const IntersectKdTreeBbox::OverlapMode &mode);


#endif
