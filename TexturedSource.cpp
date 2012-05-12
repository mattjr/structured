#include "TexturedSource.h"
using namespace SpatialIndex;
using namespace std;
TexturedSource::TexturedSource(Type type, const std::string& filename,const std::string &bbox_file,bool use_tex,bool use_texaux): Source(type,filename)
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
    stream = new MyDataStream(_bbox_file,_cameras);

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
