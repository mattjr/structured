/* * structured - Tools for the Generation and Visualization of Large-scale
 * Three-dimensional Reconstructions from Image Data. This software includes
 * source code from other projects, which is subject to different licensing,
 * see COPYING for details. If this project is used for research see COPYING
 * for making the appropriate citations.
 * Copyright (C) 2013 Matthew Johnson-Roberson <mattkjr@gmail.com>
 *
 * This file is part of structured.
 *
 * structured is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * structured is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with structured.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef TEXTUREDSOURCE_H
#define TEXTUREDSOURCE_H
#include <vpb/System>

#include "SpatialIndex.h"
#include <osg/Geode>
#include <osg/BoundingBox>
#include <osg/Geometry>
#include "Clipper.h"
#include <osg/io_utils>
class MyDataStream;

class TexturedSource : public vpb::Source
{
    friend class TexturingQuery;
public:
    TexturedSource(Type type, const std::string& filename,const std::string &bbox_file,bool use_tex=true,bool use_texaux=false,
                   float expandByX=0.0, float expandByY=0.0, float expandByZ=0.0);
    TexturedSource(Type type, const std::string& filename);
    ~TexturedSource();

    class ProjectionCamera{
    public:
        osg::Matrixf m;
        std::string filename;
        long id;
        osg::BoundingBox bb;
    };
    typedef std::map<SpatialIndex::id_type,ProjectionCamera>  CameraVector;
    SpatialIndex::ISpatialIndex* tree;
    void intersectsWithQuery(const SpatialIndex::IShape& query, SpatialIndex::IVisitor& v);
    osg::BoundingBox _bb;
    osg::ref_ptr<osg::KdTree> _kdTree;
    TexturedSource::CameraVector _cameras;
    osg::Vec4Array *ids;
    osg::Vec4Array *colors;
    osg::Vec4Array *texAndAux;

    TexBlendCoord  tex;

protected:
    SpatialIndex::IStorageManager* memstore;
    SpatialIndex::IStorageManager* manager;
    double utilization;
    int capacity;
    MyDataStream *stream;
    std::string _bbox_file;
    OpenThreads::Mutex _treeMutex;



};


class CountVisitor : public SpatialIndex::IVisitor
{
private:
    uint32_t m_indexIO;
    uint32_t m_leafIO;
    uint32_t nResults;

public:
    CountVisitor(): nResults(0) {}

    ~CountVisitor() {}

    void visitNode(const SpatialIndex::INode& n)
    {
        if (n.isLeaf()) m_leafIO++;
        else m_indexIO++;
    }

    void visitData(const SpatialIndex::IData& d)
    {
        nResults += 1;

    }

    void visitData(std::vector<const SpatialIndex::IData*>& v)
    {
        // std::cout << v[0]->getIdentifier() << " " << v[1]->getIdentifier() << std::endl;
    }




    uint32_t GetResultCount() const { return nResults; }


};
// example of a Visitor pattern.
// findes the index and leaf IO for answering the query and prints
// the resulting data IDs to stdout.

class ObjVisitor : public SpatialIndex::IVisitor
{
private:
    uint32_t m_indexIO;
    uint32_t m_leafIO;
    std::vector<SpatialIndex::id_type> m_vector;
    uint32_t nResults;
    const SpatialIndex::IShape& m_center;
    unsigned int _maxCount;
    static const bool limitadd=false;
    double minDist;


public:
    ObjVisitor(const SpatialIndex::IShape& center,unsigned int maxCount): nResults(0),m_center(center),_maxCount(maxCount),minDist(DBL_MAX) {}

    ~ObjVisitor() {}

    void visitNode(const SpatialIndex::INode& n)
    {
        if (n.isLeaf()) m_leafIO++;
        else m_indexIO++;
    }

    void visitData(const SpatialIndex::IData& d)
    {
        nResults += 1;
        if(!limitadd)
            m_vector.push_back(d.getIdentifier());
        else{
            SpatialIndex::IShape *s;
            d.getShape(&s);
            SpatialIndex::Point pt;
            if(s){
                s->getCenter(pt);
                double dist=s->getMinimumDistance(m_center);
                bool insert= (m_vector.size() < _maxCount);
                if(!insert && minDist > dist){
                    minDist=dist;
                    insert=true;
                }
                if(insert)
                    m_vector.push_back(d.getIdentifier());

            }
        }
    }

    void visitData(std::vector<const SpatialIndex::IData*>& v)
    {
        // std::cout << v[0]->getIdentifier() << " " << v[1]->getIdentifier() << std::endl;
    }




    uint32_t GetResultCount() const { return nResults; }
    std::vector<SpatialIndex::id_type>& GetResults()  { return m_vector; }


};


class CollectVisitor : public SpatialIndex::IVisitor
{
private:
    uint32_t m_indexIO;
    uint32_t m_leafIO;
    std::vector<SpatialIndex::id_type> m_vector;
    uint32_t nResults;


public:
    CollectVisitor(): nResults(0) {}

    ~CollectVisitor() {}

    void visitNode(const SpatialIndex::INode& n)
    {
        if (n.isLeaf()) m_leafIO++;
        else m_indexIO++;
    }

    void visitData(const SpatialIndex::IData& d)
    {
        nResults += 1;

        m_vector.push_back(d.getIdentifier());

    }

    void visitData(std::vector<const SpatialIndex::IData*>& v)
    {
        // std::cout << v[0]->getIdentifier() << " " << v[1]->getIdentifier() << std::endl;
    }




    uint32_t GetResultCount() const { return nResults; }
    std::vector<SpatialIndex::id_type>& GetResults()  { return m_vector; }


};
class MyDataStream : public SpatialIndex::IDataStream
{
public:
    MyDataStream(std::string inputFile,TexturedSource::CameraVector &cam,
                 float expandByX=0.0, float expandByY=0.0, float expandByZ=0.0)
        : m_pNext(0),m_camVec(cam),_expandByX(expandByX),_expandByY(expandByY),_expandByZ(expandByZ)
    {
        m_fin.open(inputFile.c_str());

        if (! m_fin){
            fprintf(stderr,"Can't open %s\n",inputFile.c_str());
            throw Tools::IllegalArgumentException("Input file not found.");
        }
        readNextEntry();
    }

    virtual ~MyDataStream()
    {
        if (m_pNext != 0) delete m_pNext;
    }

    virtual SpatialIndex::IData* getNext()
    {
        if (m_pNext == 0) return 0;

        SpatialIndex::RTree::Data* ret = m_pNext;
        m_pNext = 0;
        readNextEntry();
        return ret;
    }

    virtual bool hasNext()
    {
        return (m_pNext != 0);
    }

    virtual uint32_t size()
    {
        throw Tools::NotSupportedException("Operation not supported.");
    }

    virtual void rewind()
    {
        if (m_pNext != 0)
        {
            delete m_pNext;
            m_pNext = 0;
        }

        m_fin.seekg(0, std::ios::beg);
        readNextEntry();
    }

    void readNextEntry()
    {
        double low[3], high[3];
        TexturedSource::ProjectionCamera cam;
        osg::Matrix m;
        m_fin >> cam.id >> cam.filename >> low[0] >> low[1] >> low[2] >> high[0] >> high[1] >> high[2]
                >> m(0,0) >>m(0,1)>>m(0,2) >>m(0,3)
                >> m(1,0) >>m(1,1)>>m(1,2) >>m(1,3)
                >> m(2,0) >>m(2,1)>>m(2,2) >>m(2,3)
                >> m(3,0) >>m(3,1)>>m(3,2) >>m(3,3);/*
        >> m(0,0) >>m(1,0)>>m(2,0) >>m(3,0)
        >> m(0,1) >>m(1,1)>>m(2,1) >>m(3,1)
        >> m(0,2) >>m(1,2)>>m(2,2) >>m(3,2)
        >> m(0,3) >>m(1,3)>>m(2,3) >>m(3,3);*/
        if (m_fin.good())
        {
            cam.m=m;//osg::Matrix::inverse(m);
            //std::cout << m<<std::endl;
            if(_expandByX>0.0){
                double mod0=(high[0]-low[0])*_expandByX;
                high[0]+=mod0;
                low[0]-=mod0;
            }

            if(_expandByY>0.0){
                double mod1=(high[1]-low[1])*_expandByY;
                high[1]+=mod1;
                low[1]-=mod1;
            }

            if(_expandByZ>0.0){
                double mod2=(high[2]-low[2])*_expandByZ;
                high[2]+=mod2;
                low[2]-=mod2;
            }

            cam.bb = osg::BoundingBox(low[0],low[1],low[2],high[0],high[1],high[2]);
            // std::cout << cam.bb._min<<" "<< cam.bb._max<<std::endl;
            m_camVec[cam.id]=cam;
            /*if (op != INSERT)
                                throw Tools::IllegalArgumentException(
                                        "The data input should contain insertions only."
                                );*/

            SpatialIndex::Region r(low, high, 3);
            m_pNext = new SpatialIndex::RTree::Data(sizeof(double), reinterpret_cast<byte*>(low), r, cam.id);
            // Associate a bogus data array with every entry for testing purposes.
            // Once the data array is given to RTRee:Data a local copy will be created.
            // Hence, the input data array can be deleted after this operation if not
            // needed anymore.
        }
    }

    std::ifstream m_fin;
    SpatialIndex::RTree::Data* m_pNext;
    TexturedSource::CameraVector &m_camVec;
    float _expandByX;
    float _expandByY;
    float _expandByZ;

};

#endif // TEXTUREDSOURCE_H
