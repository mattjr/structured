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
enum OverlapMode{
    GAP,
    DUP,
    CUT,
    DUMP,
    TWOBOX,

};

const int maxNumTC=4;
typedef std::vector<osg::ref_ptr<osg::Vec3Array> > TexBlendCoord;
class geom_elems_dst{
public:
    geom_elems_dst(int numTex,bool vt){
        colors=new osg::Vec4Array();
        faces = new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES, 0);
        marginFace=new std::vector<bool>();

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
        if(vt)
            texAndAux=new osg::Vec4Array;
else
            texAndAux=NULL;
        faces = new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES, 0);
    }
    osg::Vec3Array *vertices;
    osg::Vec4Array *colors;
    osg::DrawElementsUInt *faces;
     osg::ref_ptr<osg::Vec4Array>  texid;
     osg::Vec4Array *texAndAux;
     std::vector<bool> *marginFace;

    TexBlendCoord  texcoords;
};

typedef struct _geom_elems_src{
    osg::Vec4Array *colors;
    osg::Vec4Array *texid;
    TexBlendCoord  texcoords;
    osg::Vec4Array *texAndAux;
}geom_elems_src;


struct IntersectKdTreeBboxFaces
{
    IntersectKdTreeBboxFaces(osg::ref_ptr<osg::Geometry> geo,
                        const osg::KdTree::KdNodeList& nodes,
                        const osg::KdTree::TriangleList& triangles
                             )   :   _geo(geo),  _kdNodes(nodes),
    _triangles(triangles)

    {
        _vertices= (osg::Vec3Array*)_geo->getVertexArray();
        _texCoord0= (osg::Vec2Array*)_geo->getTexCoordArray(0);
        _auxData= (osg::Vec2Array*)_geo->getTexCoordArray(1);



    }


    enum OverlapMode{
        GAP,
        DUP,
        CUT,
        DUMP

    };

    void intersectFaceOnly(const osg::KdTree::KdNode& node,  osg::DrawElementsUInt *dst_tri,const osg::BoundingBox clipbox,const OverlapMode &mode) ;
    void finish(osg::DrawElementsUInt *dst_tri,osg::Vec3Array *verts,osg::Vec2Array *texCoord, osg::Vec2Array* auxData);

    //bool intersectAndClip(osg::Vec3& s, osg::Vec3& e, const osg::BoundingBox& bb) const;
    const osg::Geometry *               _geo;

    const osg::Vec3Array *               _vertices;
    const osg::Vec2Array *               _texCoord0;
    const osg::Vec2Array *               _auxData;



    const osg::KdTree::KdNodeList&           _kdNodes;
    const osg::KdTree::TriangleList&         _triangles;





protected:

    IntersectKdTreeBboxFaces& operator = (const IntersectKdTreeBboxFaces&) { return *this; }
};


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
        _texAndAux(src.texAndAux),
        _texcoords(src.texcoords),
        _kdNodes(nodes),
        _triangles(triangles)


    {
        usedFace.resize(_triangles.size(),false);


    }
    /* ~IntersectKdTreeBbox(){
        delete _new_triangles;
        delete _new_vertices;
        delete _new_texcoords;
        delete _new_texid;
    }*/



    void intersect(const osg::KdTree::KdNode& node, const geom_elems_dst dst,const osg::BoundingBox clipbox,const OverlapMode &mode,const osg::BoundingBox *bbox_margin);

    //bool intersectAndClip(osg::Vec3& s, osg::Vec3& e, const osg::BoundingBox& bb) const;
    const osg::Vec3Array *               _vertices;
    const osg::Vec4Array *               _colors;
    const osg::Vec4Array *   _texid;
    const osg::Vec4Array *   _texAndAux;

    const TexBlendCoord  _texcoords;

    const osg::KdTree::KdNodeList&           _kdNodes;
    const osg::KdTree::TriangleList&         _triangles;
    const osg::Vec3Array *_gapPts;


    std::vector<bool> usedFace;



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
                                      const OverlapMode &overlapmode,const osg::BoundingBox *bbox_margin=NULL);

    const geom_elems_src _src;
    IntersectKdTreeBbox intersector;
};



class KdTreeBboxFaces : public osg::KdTree {


public:
    KdTreeBboxFaces(const KdTree& rhs,osg::ref_ptr<osg::Geometry> geo) : KdTree(rhs), intersector(geo,
            _kdNodes,
            _triangles) {


        if (_kdNodes.empty())
        {
            osg::notify(osg::NOTICE)<<"Warning: _kdTree is empty"<<std::endl;
            return;
        }




    }
    osg::ref_ptr<osg::Node> intersect(const osg::BoundingBox bbox,
                                      const IntersectKdTreeBboxFaces::OverlapMode &overlapmode);

    IntersectKdTreeBboxFaces intersector;
};
KdTreeBbox *setupKdTree(osg::ref_ptr<osg::Node> model);
KdTreeBbox *createKdTreeForUnbuilt(osg::ref_ptr<osg::Node> model);
bool cut_model(KdTreeBbox *kdtreeBBox,std::string outfilename,osg::BoundingBox bbox,const OverlapMode &mode,osg::BoundingBox *bbox_margin=NULL);
struct CheckKdTreeBbox
{
    CheckKdTreeBbox(    const osg::ref_ptr<osg::Vec3Array>  vertices,
                        const osg::KdTree::KdNodeList& nodes,
                        const osg::KdTree::TriangleList& triangles
                        ):
        _vertices(vertices),
        _kdNodes(nodes),
        _triangles(triangles)


    {



    }



    bool check(const osg::KdTree::KdNode& node,const osg::BoundingBox clipbox) const;

    //bool intersectAndClip(osg::Vec3& s, osg::Vec3& e, const osg::BoundingBox& bb) const;
    const osg::Vec3Array *               _vertices;

    const osg::KdTree::KdNodeList&           _kdNodes;
    const osg::KdTree::TriangleList&         _triangles;





protected:

    CheckKdTreeBbox& operator = (const CheckKdTreeBbox&) { return *this; }
};
class KdTreeChecker : public osg::KdTree {


public:
    KdTreeChecker(const KdTree& rhs) : KdTree(rhs), checker(_vertices,
            _kdNodes,
            _triangles) {


        if (_kdNodes.empty())
        {
            osg::notify(osg::NOTICE)<<"Warning: _kdTree is empty"<<std::endl;
            return;
        }




    }
    bool check(const osg::BoundingBox bbox);

    CheckKdTreeBbox checker;
};

#endif
