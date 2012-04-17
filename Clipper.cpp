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

#include <osg/TriangleIndexFunctor>

#include "Clipper.h"

#include <set>
#include <list>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <osg/io_utils>
#include <assert.h>
#include <stdio.h>
#include <osg/TriangleFunctor>
#include <osgUtil/Tessellator> // to tessellate multiple contours
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include "PLYWriterNodeVisitor.h"
#include <osgUtil/SmoothingVisitor>
using namespace osg;
using namespace std;
bool ClipTriangle(osg::Vec3 (&poly)[10], unsigned int &polySize, const osg::BoundingBox &bounds)
{
    // I think allocating space for 7 vertices should be fine, but just in case
    // I'm being stupid, let's make it 10 to be safe.

    // Note: We use arrays instead of std::vector since this results in about a
    // 2x speedup in the build time since std::vector needs to allocate memory
    // on the heap which is very expensive.

    osg::Vec3 out[10];
    unsigned int outSize=0;

    const osg::Vec3 a = poly[0];
    const osg::Vec3 b = poly[1];
    const osg::Vec3 c = poly[2];

    for (int d=0; d<3; d++) {
        if (bounds._min[d] == bounds._max[d]) {
            // slabs is planar -- we're either in it, or fully out
            if (a[d] == b[d] && a[d] == c[d] && a[d] == bounds._min[d])
                continue; // no need to try clipping; we know it's all inside.
            else {
                // we're fully out.  Nothing to clip.
                return false;
            }
        }
        else
            for (int side = 0; side < 2; side++) {
                for (unsigned int edge = 0;edge < polySize; edge++) {
                    unsigned int v0 = edge;
                    unsigned int v1 = (edge+1);
                    if (v1 >= polySize)
                        v1 = v1-polySize; // cheaper than using mod as in: (edge+1)%polySize

                    const bool v0in
                            = (side==0)
                            ? poly[v0][d] >= bounds._min[d]
                            : poly[v0][d] <= bounds._max[d];
                    const bool v1in
                            = (side==0)
                            ? poly[v1][d] >= bounds._min[d]
                            : poly[v1][d] <= bounds._max[d];

                    if (v0in && v1in) {
                        // v0 was already added in the last step, then
                        out[outSize++] = poly[v1];
                    } else if (!v0in && !v1in) {
                        // do nothing, both are out
                    } else {
                        osg::Vec3 boundsside;
                        if(side == 0)
                            boundsside=bounds._min;
                        else
                            boundsside=bounds._max;

                        const float f
                                = (boundsside[d] - poly[v0][d])
                                / (    poly[v1][d] - poly[v0][d]);
                        osg::Vec3 newVtx=(poly[v1]-poly[v0]);
                        newVtx[0]*=f;
                        newVtx[1]*=f;
                        newVtx[2]*=f;
                        newVtx = (poly[v0] + newVtx) ;//(1-f)*poly[v0] + f*poly[v1];
                        newVtx[d] = boundsside[d]; // make sure it's exactly _on_ the plane

                        if (v0in) {
                            // v0 was already pushed
                            if (newVtx != poly[v0])
                                out[outSize++] = newVtx;
                        } else {
                            if (newVtx != poly[v0] && newVtx != poly[v1])
                                out[outSize++] = newVtx;
                            out[outSize++] = poly[v1];
                        }
                    }
                }
                if (outSize < 3)
                    return false;

                for (unsigned int i=0; i < outSize; ++i){
                    poly[i] = out[i];
                }

                polySize = outSize;
                outSize = 0;
            }
    }
    return true;
}
class StoreTri
{
public:
    void operator() (const osg::Vec3& v1,const osg::Vec3& v2,const osg::Vec3& v3, bool)
    {
        int size=idx.size();
        v.push_back(v1);
        v.push_back(v2);
        v.push_back(v3);
        idx.push_back(size);
        idx.push_back(size+1);
        idx.push_back(size+2);
        //std::cout << "\t("<<v1<<") ("<<v2<<") ("<<v3<<") "<<") " <<std::endl;

    }
    std::vector<osg::Vec3> v;
    std::vector<int> idx;

};

/*void IntersectKdTreeBbox::intersect(const KdTree::KdNode& node, const osg::BoundingBox clipbox,const OverlapMode &mode) const
{
    if (node.first<0)
    {
        // treat as a leaf

        //OSG_NOTICE<<"KdTree::intersect("<<&leaf<<")"<<std::endl;
        int istart = -node.first-1;
        int iend = istart + node.second;

        for(int i=istart; i<iend; ++i)
        {
            //const Triangle& tri = _triangles[_primitiveIndices[i]];
            const KdTree::Triangle& tri = _triangles[i];
            // OSG_NOTICE<<"   tri("<<tri.p1<<","<<tri.p2<<","<<tri.p3<<")"<<std::endl;

            osg::Vec3 v0 = _vertices[tri.p0];
            osg::Vec3 v1 = _vertices[tri.p1];
            osg::Vec3 v2 = _vertices[tri.p2];
            osg::Vec4 c0,c1,c2;
            if(_colors){
                c0 = (*_colors)[tri.p0];
                c1 = (*_colors)[tri.p1];
                c2 = (*_colors)[tri.p2];
            }
            int contains=0;
            contains+=clipbox.contains(v0);
            contains+=clipbox.contains(v1);
            contains+=clipbox.contains(v2);
            //No inside
            if(contains == 0)
                continue;
            else if(contains <3){

                //Some inside
                if(mode==GAP)
                    continue;
                else if(mode == CUT){
                    // clipping
                    osg::Vec3 poly[10]={v0,v1,v2};
                    unsigned int polySize=3;
                    if(ClipTriangle(poly,polySize,clipbox)){

                        // create Geometry object to store all the vertices and lines primitive.
                        osg::ref_ptr<osg::Geometry> polyGeom = new osg::Geometry();

                        // this time we'll use C arrays to initialize the vertices.
                        // note, anticlockwise ordering.
                        // note II, OpenGL polygons must be convex, planar polygons, otherwise
                        // undefined results will occur.  If you have concave polygons or ones
                        // that cross over themselves then use the osgUtil::Tessellator to fix
                        // the polygons into a set of valid polygons.


                        osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array(polySize,poly);

                        // pass the created vertex array to the points geometry object.
                        polyGeom->setVertexArray(vertices);

                        // This time we simply use primitive, and hardwire the number of coords to use
                        // since we know up front,
                        polyGeom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POLYGON,0,polySize));

                        //printTriangles("Polygon",*polyGeom);
                        osg::TriangleFunctor<StoreTri> tf;
                        polyGeom->accept(tf);

                        // add the points geometry to the geode.
                        int offset= _new_vertices->size();
                        for(int p=0; p < (int) tf.v.size(); p++)
                            _new_vertices->push_back(tf.v[p]);
                        for(int p=0; p < (int) tf.idx.size(); p++)
                            _new_triangles->push_back(tf.idx[p]+offset);

                        continue;
                    }
                }else if(mode == DUMP){
                    if(clipbox.contains(v0))
                        _gapPts->push_back(v0);
                    if(clipbox.contains(v1))
                        _gapPts->push_back(v1);
                    if(clipbox.contains(v2))
                        _gapPts->push_back(v2);
                    continue;
                }//else DUP INCLUDE THESE FACES IN BOTH BOXES

            }


            int counter=_new_triangles->size();
            _new_vertices->push_back(v0);
            _new_vertices->push_back(v1);
            _new_vertices->push_back(v2);
            if(_new_colors){
                _new_colors->push_back(c0);
                _new_colors->push_back(c1);
                _new_colors->push_back(c2);
            }
            _new_triangles->push_back(counter);
            _new_triangles->push_back(counter+1);
            _new_triangles->push_back(counter+2);

        }
    }
    else
    {
        if (node.first>0)
        {
            //osg::BoundingBox clipbox2(b);
            //  if (intersectAndClip(clipbox2, _kdNodes[node.first].bb))
            if(clipbox.intersects(_kdNodes[node.first].bb))
            {
                intersect(_kdNodes[node.first], clipbox,mode);
            }
        }
        if (node.second>0)
        {
            //osg::BoundingBox clipbox2(b);
            // if (intersectAndClip(clipbox2,_kdNodes[node.second].bb))
            if(clipbox.intersects(_kdNodes[node.second].bb))
            {
                intersect(_kdNodes[node.second], clipbox,mode);
            }
        }
    }
}
*/


void IntersectKdTreeBbox::intersect(const osg::KdTree::KdNode& node, const geom_elems_dst dst,const osg::BoundingBox clipbox,const OverlapMode &mode) const
{
    if (node.first<0)
    {
        // treat as a leaf

        //OSG_NOTICE<<"KdTree::intersect("<<&leaf<<")"<<std::endl;
        int istart = -node.first-1;
        int iend = istart + node.second;
        int numTC=_texcoords.size();
        //printf("%d Num TC\n",numTC);
        for(int i=istart; i<iend; ++i)
        {
            //const Triangle& tri = _triangles[_primitiveIndices[i]];
            const KdTree::Triangle& tri = _triangles[i];
           // cout <<"   tri("<<tri.p0<<","<<tri.p1<<","<<tri.p2<<")"<<std::endl;
           // cout << (*_vertices)[tri.p0] << " ";
          //  cout <<clipbox._min<< " "<< clipbox._max<<endl;
            osg::Vec4 c0,c1,c2;
            if(_colors){
                c0 = (*_colors)[tri.p0];
                c1 = (*_colors)[tri.p1];
                c2 = (*_colors)[tri.p2];
            }

            osg::Vec3 v0 = (*_vertices)[tri.p0];
            osg::Vec3 v1 = (*_vertices)[tri.p1];
            osg::Vec3 v2 = (*_vertices)[tri.p2];
            osg::Vec3 t0[numTC],t1[numTC],t2[numTC];

            if(_texcoords.size() && _texcoords[0]){
                for(int f=0; f< (int)_texcoords.size(); f++){
                    t0[f] = (*_texcoords[f])[tri.p0];
                    t1[f] = (*_texcoords[f])[tri.p1];
                    t2[f] = (*_texcoords[f])[tri.p2];
                }
            }
            osg::Vec4 id0,id1,id2;

            if(_texid){
                if(_texcoords.size() > 1){
                    id0 = (*_texid)[tri.p0];
                    id1 = (*_texid)[tri.p1];
                    id2 = (*_texid)[tri.p2];
                    assert(id0[0]  == id1[0] && id0[0] == id2[0]);
                }
            }
            //printf("%f\n",id0[0]);
            int contains=0;
            contains+=clipbox.contains(v0);
            contains+=clipbox.contains(v1);
            contains+=clipbox.contains(v2);
            //No inside
            if(contains == 0)
                continue;
            else if(contains <3){

                //Some inside
                if(mode==GAP)
                    continue;
                else if(mode == CUT){
                    // clipping
                    osg::Vec3 poly[10]={v0,v1,v2};
                    unsigned int polySize=3;
                    if(ClipTriangle(poly,polySize,clipbox)){

                        // create Geometry object to store all the vertices and lines primitive.
                        osg::ref_ptr<osg::Geometry> polyGeom = new osg::Geometry();

                        // this time we'll use C arrays to initialize the vertices.
                        // note, anticlockwise ordering.
                        // note II, OpenGL polygons must be convex, planar polygons, otherwise
                        // undefined results will occur.  If you have concave polygons or ones
                        // that cross over themselves then use the osgUtil::Tessellator to fix
                        // the polygons into a set of valid polygons.


                        osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array(polySize,poly);

                        // pass the created vertex array to the points geometry object.
                        polyGeom->setVertexArray(vertices);

                        // This time we simply use primitive, and hardwire the number of coords to use
                        // since we know up front,
                        polyGeom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POLYGON,0,polySize));

                        //printTriangles("Polygon",*polyGeom);
                        osg::TriangleFunctor<StoreTri> tf;
                        polyGeom->accept(tf);

                        // add the points geometry to the geode.
                        int offset= dst.vertices->size();
                        for(int p=0; p < (int) tf.v.size(); p++)
                            dst.vertices->push_back(tf.v[p]);
                        for(int p=0; p < (int) tf.idx.size(); p++)
                            dst.faces->push_back(tf.idx[p]+offset);

                        continue;
                    }
                }/*else if(mode == DUMP){
                    if(clipbox.contains(v0))
                        _gapPts->push_back(v0);
                    if(clipbox.contains(v1))
                        _gapPts->push_back(v1);
                    if(clipbox.contains(v2))
                        _gapPts->push_back(v2);
                    continue;
                }*///else DUP INCLUDE THESE FACES IN BOTH BOXES

            }


            int counter=dst.faces->size();
            dst.vertices->push_back(v0);
            dst.vertices->push_back(v1);
            dst.vertices->push_back(v2);
            if(dst.colors){
                dst.colors->push_back(c0);
                dst.colors->push_back(c1);
                dst.colors->push_back(c2);
            }
            dst.faces->push_back(counter);
            dst.faces->push_back(counter+1);
            dst.faces->push_back(counter+2);
            if(_texcoords.size() && _texcoords[0]){
                for(int f=0; f< numTC; f++){
                    dst.texcoords[f]->push_back(t0[f]);
                    dst.texcoords[f]->push_back(t1[f]);
                    dst.texcoords[f]->push_back(t2[f]);
                }
            }

            if(numTC > 1 && _texid){

                dst.texid->push_back(id0);
                dst.texid->push_back(id1);
                dst.texid->push_back(id2);
                assert(dst.texcoords[0]->size() ==  dst.texid->size());
            }

        }
    }
    else
    {
        if (node.first>0)
        {
            //osg::BoundingBox clipbox2(b);
            //  if (intersectAndClip(clipbox2, _kdNodes[node.first].bb))
            if(clipbox.intersects(_kdNodes[node.first].bb))
            {
                intersect(_kdNodes[node.first],dst, clipbox,mode);
            }
        }
        if (node.second>0)
        {
            //osg::BoundingBox clipbox2(b);
            // if (intersectAndClip(clipbox2,_kdNodes[node.second].bb))
            if(clipbox.intersects(_kdNodes[node.second].bb))
            {
                intersect(_kdNodes[node.second],dst, clipbox,mode);
            }
        }
    }
}
/*osg::ref_ptr<osg::Node> KdTreeBbox::intersect(const osg::BoundingBox bbox,osg::Vec4Array *colors,const IntersectKdTreeBbox::OverlapMode &overlapmode,osg::Vec3Array *&dumpPts,bool multTex) const

{
    if (_kdNodes.empty())
    {
        osg::notify(osg::NOTICE)<<"Warning: _kdTree is empty"<<std::endl;
        return NULL;
    }



    IntersectKdTreeBbox intersector(*_vertices,
                                    colors,
                                    _kdNodes,
                                    _triangles,multTex
                                    );
    osg::ref_ptr<osg::Geode> newGeode=new osg::Geode;
    osg::Geometry *new_geom=new osg::Geometry;
    newGeode->addDrawable(new_geom);
    intersector.intersect(getNode(0), bbox,overlapmode);
    new_geom->addPrimitiveSet(intersector._new_triangles);
    new_geom->setVertexArray(intersector._new_vertices);
    new_geom->setColorArray(intersector._new_colors);

    dumpPts=intersector._gapPts;

    return newGeode;
}*/
osg::ref_ptr<osg::Node> KdTreeBbox::intersect(const osg::BoundingBox bbox,  geom_elems_dst &dst,
                                              const IntersectKdTreeBbox::OverlapMode &overlapmode)

{
    //cout << "CRA"<<getNode(0).bb._min << "  "<< getNode(0).bb._max<<endl;
   /* if(getNode(0).bb < 0){
        fprintf(stderr,"Null tree\n");
        return NULL;
    }*/
    osg::ref_ptr<osg::Geode> newGeode=new osg::Geode;
    osg::Geometry *new_geom=new osg::Geometry;
    newGeode->addDrawable(new_geom);
    intersector.intersect(getNode(0),dst,bbox,overlapmode);
    //printf("Size of output %d %d\n",dst.vertices->size(),_vertices->size());
    new_geom->addPrimitiveSet(dst.faces);
    new_geom->setVertexArray(dst.vertices);
    new_geom->setColorArray(dst.colors);
    return newGeode;
}
KdTreeBbox *setupKdTree(osg::ref_ptr<osg::Node> model){
    if(model.valid()){
        osg::Geode *geode= dynamic_cast<osg::Geode*>(model.get());
        if(!geode)
            geode=model->asGroup()->getChild(0)->asGeode();
        if(geode && geode->getNumDrawables()){
            osg::Drawable *drawable = geode->getDrawable(0);
            osg::KdTree *kdTree = dynamic_cast<osg::KdTree*>(drawable->getShape());
            if(kdTree){
                osg::Geometry *geom = dynamic_cast< osg::Geometry*>(drawable);
                geom_elems_src *srcGeom=new geom_elems_src ;
                srcGeom->colors=(osg::Vec4Array*)geom->getColorArray();
                //    srcGeom.texcoords;
                srcGeom->texid=NULL;
                KdTreeBbox *kdtreeBbox=new KdTreeBbox(*kdTree,*srcGeom);
                return kdtreeBbox;
            }

        }else{
            osg::notify(osg::ALWAYS) << "Model can't be converted to geode\n";
            return false;
        }

    }else{
        osg::notify(osg::ALWAYS)  << "Model can't be loaded\n";
        return false;
    }

    return NULL;
}

KdTreeBbox *createKdTreeForUnbuilt(osg::ref_ptr<osg::Node> model){
    if(model.valid()){
        osg::Geode *geode= dynamic_cast<osg::Geode*>(model.get());
        if(!geode)
            geode=model->asGroup()->getChild(0)->asGeode();
        if(geode && geode->getNumDrawables()){
             osg::ref_ptr<osg::KdTreeBuilder> kdTreeBuilder = osgDB::Registry::instance()->getKdTreeBuilder()->clone();
             geode->accept(*kdTreeBuilder);
             osg::Drawable *drawable = geode->getDrawable(0);
             osg::KdTree *kdTree = dynamic_cast<osg::KdTree*>(drawable->getShape());
             if(kdTree){
                 osg::Geometry *geom = dynamic_cast< osg::Geometry*>(drawable);
                 geom_elems_src *srcGeom=new geom_elems_src ;
                 srcGeom->colors=(osg::Vec4Array*)geom->getColorArray();
                 //    srcGeom.texcoords;
                 srcGeom->texid=NULL;
                 KdTreeBbox *kdtreeBbox=new KdTreeBbox(*kdTree,*srcGeom);
                 return kdtreeBbox;
             }else{
                fprintf(stderr,"Can't be converted to kdtree\n");
                return false;
            }

        }else{
            osg::notify(osg::ALWAYS) << "Model can't be converted to geode\n";
            return false;
        }

    }else{
        osg::notify(osg::ALWAYS)  << "Model can't be loaded\n";
        return false;
    }

    return NULL;
}

bool cut_model(KdTreeBbox *kdtreeBBox,std::string outfilename,osg::BoundingBox bbox,const IntersectKdTreeBbox::OverlapMode &mode){
    if(!kdtreeBBox){
        fprintf(stderr,"Failed to load kdtree\n");
        return false;
    }
    osg::ref_ptr<osg::Node> root;
    int numTex=kdtreeBBox->_src.texcoords.size();
    geom_elems_dst dstGeom(numTex);
    root=kdtreeBBox->intersect(bbox,dstGeom,mode);
    if(dstGeom.faces->size()){
        if(osgDB::getFileExtension(outfilename) == "ply"){
            osgUtil::SmoothingVisitor sv;
            root->accept(sv);
            std::ofstream f(outfilename.c_str());
            PLYWriterNodeVisitor nv(f);
            root->accept(nv);
        }else{
            osgUtil::SmoothingVisitor sv;
            root->accept(sv);
            osgDB::writeNodeFile(*root,outfilename);
        }
        return true;
    }

    return false;
}

bool CheckKdTreeBbox::check(const osg::KdTree::KdNode& node,const osg::BoundingBox clipbox) const
{
    if (node.first<0)
    {
        // treat as a leaf

        //OSG_NOTICE<<"KdTree::intersect("<<&leaf<<")"<<std::endl;
        int istart = -node.first-1;
        int iend = istart + node.second;
        for(int i=istart; i<iend; ++i)
        {
            const KdTree::Triangle& tri = _triangles[i];


            osg::Vec3 v0 = (*_vertices)[tri.p0];
            osg::Vec3 v1 = (*_vertices)[tri.p1];
            osg::Vec3 v2 = (*_vertices)[tri.p2];


            //printf("%f\n",id0[0]);
            int contains=0;
            contains+=clipbox.contains(v0);
            contains+=clipbox.contains(v1);
            contains+=clipbox.contains(v2);
            //No inside
            if(contains > 0)
                return true;
        }
        return false;
    }
    else
    {
        return (node.first>0 && clipbox.intersects(_kdNodes[node.first].bb) && check(_kdNodes[node.first], clipbox)) ||
                ((node.second>0) && clipbox.intersects(_kdNodes[node.second].bb) &&check(_kdNodes[node.second], clipbox));
    }
}
bool  KdTreeChecker::check(const osg::BoundingBox bbox)

{


    return checker.check(getNode(0),bbox);

}
