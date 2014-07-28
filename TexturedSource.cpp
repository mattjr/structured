//
// structured - Tools for the Generation and Visualization of Large-scale
// Three-dimensional Reconstructions from Image Data. This software includes
// source code from other projects, which is subject to different licensing,
// see COPYING for details. If this project is used for research see COPYING
// for making the appropriate citations.
// Copyright (C) 2013 Matthew Johnson-Roberson <mattkjr@gmail.com>
//
// This file is part of structured.
//
// structured is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// structured is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with structured.  If not, see <http://www.gnu.org/licenses/>.
//

#include "TexturedSource.h"
using namespace SpatialIndex;
using namespace std;
TexturedSource::TexturedSource(Type type, const std::string& filename,const std::string &bbox_file,bool use_tex,bool use_texaux,
                               float expandByX, float expandByY, float expandByZ): Source(type,filename)
{
    std::string mf=filename;
    _bbox_file=bbox_file;

    utilization=0.7;
    capacity=4;

    memstore = StorageManager::createNewMemoryStorageManager();
    // Create a new storage manager with the provided base name and a 4K page size.

    manager = StorageManager::createNewRandomEvictionsBuffer(*memstore, 10, false);
    // applies a main memory random buffer on top of the persistent storage manager
    // (LRU buffer, etc can be created the same way).
    stream = new MyDataStream(_bbox_file,_cameras,expandByX,expandByY,expandByZ);

    // Create and bulk load a new RTree with dimensionality 3, using "file" as
    // the StorageManager and the RSTAR splitting policy.
    id_type indexIdentifier;
    tree = RTree::createAndBulkLoadNewRTree(
            RTree::BLM_STR, *stream, *manager, utilization, capacity,capacity, 3, SpatialIndex::RTree::RV_RSTAR, indexIdentifier);

    // std::cerr << *tree;
    //std::cerr << "Buffer hits: " << file->getHits() << std::endl;
    //std::cerr << "Index ID: " << indexIdentifier << std::endl;

    bool ret = tree->isIndexValid();
    if (ret == false){
        osg::notify(osg::FATAL) << "ERROR: Structure is invalid!" << std::endl;
    }else {
           osg::notify(osg::INFO) << "The stucture seems O.K." << std::endl;
    }

    texAndAux=new osg::Vec4Array;

    if(use_tex){
    ids=new osg::Vec4Array;


    tex.resize(4);
    // for(int f=0; tex.size(); f++)
    tex[0]=new osg::Vec3Array;
    tex[1]=new osg::Vec3Array;

    tex[2]=new osg::Vec3Array;
    tex[3]=new osg::Vec3Array;
    }
}
TexturedSource::TexturedSource(Type type, const std::string& filename): Source(type,filename)
{

}

TexturedSource::~TexturedSource(){
    if(_bbox_file.size()){
        delete tree;
        delete manager;
        delete memstore;
        delete stream;
    }
}
void TexturedSource::intersectsWithQuery(const SpatialIndex::IShape& query, SpatialIndex::IVisitor& v){
    OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_treeMutex);
    tree->intersectsWithQuery(query,v);

}
