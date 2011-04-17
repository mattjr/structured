
/* -*-c++-*- VirtualPlanetBuilder - Copyright (C) 1998-2009 Robert Osfield
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


#include <osg/Texture2D>
#include <osg/ComputeBoundsVisitor>
#include <osg/io_utils>
#include <osg/Texture2DArray>
#include <osg/GLU>
#include <osgUtil/SmoothingVisitor>

#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgDB/FileNameUtils>

#include <osgFX/MultiTextureControl>

#include <osgViewer/GraphicsWindow>
#include <osgViewer/Viewer>
#include <osgViewer/Version>
#include "Clipper.h"
#include "Extents.h"
#include <vpb/DatabaseBuilder>
#include <vpb/TaskManager>
#include <vpb/System>
#include <vpb/FileUtils>
#include <vpb/FilePathManager>
#include <vpb/TextureUtils>
#include <vpb/ShapeFilePlacer>
#include "PosterPrinter.h"
// GDAL includes
#include <gdal_priv.h>
#include <ogr_spatialref.h>

// standard library includes
#include <sstream>
#include <iostream>
#include <algorithm>
#include <vips/vips.h>
#include "VPBInterface.hpp"
bool checkInBounds(osg::Vec3 tc);

using namespace vpb;
using vpb::log;
using namespace std;
bool readMatrixToScreen(std::string fname,osg::Matrix &viewProj){
    std::fstream file(fname.c_str(), std::ios::binary|std::ios::in);
    if(!file.good()){
        fprintf(stderr,"Can't open %s\n",fname.c_str());
        return false;
    }
    osg::Matrixd view,proj;

    for(int i=0; i<4; i++)
        for(int j=0; j<4; j++)
            file.read(reinterpret_cast<char*>(&(view(i,j))), sizeof(double));
    for(int i=0; i<4; i++)
        for(int j=0; j<4; j++)
            file.read(reinterpret_cast<char*>(&(proj(i,j))), sizeof(double));
    viewProj=osg::Matrix( view * proj);
    return true;
}

bool readMatrix(std::string fname,osg::Matrix &viewProj){
    std::fstream file(fname.c_str(), std::ios::binary|std::ios::in);
    if(!file.good()){
        fprintf(stderr,"Can't open %s\n",fname.c_str());
        return false;
    }

    for(int i=0; i<4; i++)
        for(int j=0; j<4; j++)
            file.read(reinterpret_cast<char*>(&(viewProj(i,j))), sizeof(double));

    return true;
}
MyDataSet::MyDataSet(const Camera_Calib &calib,string basePath,bool useTextureArray,bool useReImage,bool useVirtualTex): _calib(calib),_basePath(basePath),_useTextureArray(useTextureArray),_useReImage(useReImage),_useVirtualTex(useVirtualTex)
{
    in=NULL;
    init();
}

void MyDataSet::init()
{
    assert(!(_useReImage && _useVirtualTex));

    // make sure GDAL etc. are initialized
    System::instance();
    _C1 = 0;
    _R1 = 0;

    _numTextureLevels = 1;

    _newDestinationGraph = false;
    _useAtlas=false;
    _useDisplayLists=true;
    _useBlending=true;
    _useVBO=true;
    _useTextureArray=true;
    _file.open("/tmp/scope.txt");
    readMatrix("rot.mat",rotMat);
    readMatrixToScreen("view.mat",viewProj);

    if(_useReImage){
        if(osgDB::fileExists("subtile.v"))
            in = new vips::VImage("subtile.v");
        else
            std::cerr << "Can't open subtile on reimaging run\n";

    }

}
class CollectClusterCullingCallbacks : public osg::NodeVisitor
{
public:


    struct Triple
    {
        Triple():
                _object(0),
                _callback(0) {}

        Triple(osg::NodePath nodePath, osg::Object* object, osg::ClusterCullingCallback* callback):
                _nodePath(nodePath),
                _object(object),
                _callback(callback) {}

        Triple(const Triple& t):
                _nodePath(t._nodePath),
                _object(t._object),
                _callback(t._callback) {}

        Triple& operator = (const Triple& t)
                           {
            _nodePath = t._nodePath;
            _object = t._object;
            _callback = t._callback;
            return *this;
        }

        osg::NodePath                   _nodePath;
        osg::Object*                    _object;
        osg::ClusterCullingCallback*    _callback;
    };

    typedef std::vector<Triple> ClusterCullingCallbackList;

    CollectClusterCullingCallbacks():
            osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN) {}

    virtual void apply(osg::Group& group)
    {
        osgTerrain::Terrain* terrain = dynamic_cast<osgTerrain::Terrain*>(&group);
        if (terrain)
        {
            osg::ClusterCullingCallback* callback = dynamic_cast<osg::ClusterCullingCallback*>(terrain->getCullCallback());
            if (callback)
            {
                _callbackList.push_back(Triple(getNodePath(),terrain,callback));
            }
            return;
        }

        osgTerrain::TerrainTile* terrainTile = dynamic_cast<osgTerrain::TerrainTile*>(&group);
        if (terrainTile)
        {
            osg::ClusterCullingCallback* callback = dynamic_cast<osg::ClusterCullingCallback*>(terrainTile->getCullCallback());
            if (callback)
            {
                _callbackList.push_back(Triple(getNodePath(),terrain,callback));
            }
            return;
        }
        else
        {
            osg::NodeVisitor::apply(group);
        }
    }

    virtual void apply(osg::Geode& geode)
    {
        for(unsigned int i=0; i<geode.getNumDrawables();++i)
        {
            osg::ClusterCullingCallback* callback = dynamic_cast<osg::ClusterCullingCallback*>(geode.getDrawable(i)->getCullCallback());
            if (callback)
            {
                _callbackList.push_back(Triple(getNodePath(),geode.getDrawable(i),callback));
            }
        }
    }

    ClusterCullingCallbackList _callbackList;

};
std::vector<osg::ref_ptr<osg::Image> >MyDestinationTile::getRemappedImages(idmap_t allIds,int sizeIdx){
    osg::Timer_t before_computeMax = osg::Timer::instance()->tick();

    std::vector<osg::ref_ptr<osg::Image> > images(allIds.size());
    idmap_t::const_iterator end = allIds.end();
    for (idmap_t::const_iterator it = allIds.begin(); it != end; ++it)
    {
        images[it->second]=_atlasGen->getImage(it->second,sizeIdx);
    }

    osg::Timer_t after_computeMax = osg::Timer::instance()->tick();

    OSG_NOTICE << "Time for loadTex = " << osg::Timer::instance()->delta_s(before_computeMax, after_computeMax) <<endl;

    return images;
}

float clamp( const float& x, const float& min, const float& max )
{
    return std::min( std::max( min, x ), max );
};
void MyDestinationTile::remapArrayForTexturing(osg::Vec4Array *v,const TexBlendCoord &texCoordsArray,idmap_t allIds){
    //Remap
    for(int i=0; i< (int)v->size(); i++){
        for(int j=0; j< 4; j++){
            int id=(int)((*v)[i][j]);
            if(id >= 0){
                if(_mydataSet->_useAtlas){
                    (*v)[i][j]=_atlasGen->getAtlasId(id);
                    osg::Matrix matrix=_atlasGen->getTextureMatrixByID(id);
                    osg::Vec3 tc=texCoordsArray[j]->at(i);
                    if(checkInBounds(tc)){
                        double r=(osg::Vec2(tc[0],tc[1])-osg::Vec2(0.5,0.5)).length();
                        texCoordsArray[j]->at(i)=osg::Vec3(tc[0]*matrix(0,0) + tc[1]*matrix(1,0) + matrix(3,0),
                                                           tc[0]*matrix(0,1) + tc[1]*matrix(1,1) + matrix(3,1),r);
                    }else{
                        //    texCoordsArray[j]->at(i)=osg::Vec3(-1,-1,-1);
                    }
                }else{
                    if( allIds.count(id)){
                        (*v)[i][j]=allIds[id];
                        if(checkInBounds(texCoordsArray[j]->at(i))){
                            texCoordsArray[j]->at(i)[2]=(*v)[i][j];
                        }else{

                            texCoordsArray[j]->at(i)=osg::Vec3(-1,-1,-1);
                        }
                    }
                }
            }else{
                texCoordsArray[j]->at(i)=osg::Vec3(-1,-1,-1);
            }
        }
    }

}

osg::StateSet *MyDestinationTile::generateStateAndArray2DRemap( osg::Vec4Array *v,  const TexBlendCoord &texCoordsArray,int texSizeIdx){
    if(!v)
        return NULL;

    osg::StateSet *stateset= new osg::StateSet;

    int tex_size=  _atlasGen->getMaximumAtlasHeight();
    idmap_t allIds=_atlasGen->_allIDs;
    // idmap_t allIds=calcAllIds(v);
    //  idmap_t allIds=calcAllIds(v);

    std::vector<osg::ref_ptr<osg::Image> > texture_images;
    /*  if(_mydataSet->_useAtlas){
        if(_atlasGen->getNumAtlases() == 0)
            return NULL;
        texture_images.push_back(_atlasGen->getAtlasByNumber(0));

    }else{*/
    texture_images = _atlasGen->getImages(); //getRemappedImages(allIds,texSizeIdx);
    //}
    unsigned int a=v->size();
    unsigned int b=texCoordsArray[0]->size();

    assert(a == b);
    assert(texture_images.size() > 0);
    remapArrayForTexturing(v,texCoordsArray,allIds);

    OSG_ALWAYS << "\tImage Size: " <<texture_images[0]->s() <<endl;

    OSG_ALWAYS << "\tArray Size: " <<texture_images.size()<<endl;
    OSG_ALWAYS << "\tNumber of Sources: " <<_atlasGen->getNumSources()<<endl;
    OSG_ALWAYS << "\tNumber of Atlas: " <<_atlasGen->getNumAtlases()<<endl;

    //  texture_images[0]->setFileName(_dataSet->getSubtileName(_level,_tileX,_tileY));

    osg::ref_ptr<osg::Texture2DArray> textureArray;
    osg::ref_ptr<osg::Texture2D> texture;

    // if(!_mydataSet->_useAtlas){
    textureArray=new osg::Texture2DArray;
    textureArray->setTextureSize(texture_images[0]->s(),texture_images[0]->t(),texture_images.size());
    for(int i=0; i < (int)texture_images.size(); i++){
        cout << " Size " << texture_images[0]->s() << " "<<texture_images[0]->t() << "\n";
        assert(texture_images[i].valid() &&textureArray->getTextureWidth() == texture_images[i]->s() && textureArray->getTextureWidth() == texture_images[i]->t());
        textureArray->setImage(i,texture_images[i]);
    }
    stateset->setTextureAttribute(TEXUNIT_ARRAY, textureArray.get());

    //}
#if 0
    else{
        texture=new osg::Texture2D(texture_images[0]);
        /* texture->setWrap(osg::Texture::WRAP_S,osg::Texture::CLAMP_TO_BORDER);
        texture->setWrap(osg::Texture::WRAP_T,osg::Texture::CLAMP_TO_BORDER);
        texture->setFilter(osg::Texture2D::MIN_FILTER,osg::Texture2D::LINEAR_MIPMAP_LINEAR);
        texture->setFilter(osg::Texture2D::MAG_FILTER,osg::Texture2D::LINEAR);*/
        int layerNum=0;
        osg::Texture::InternalFormatMode internalFormatMode = osg::Texture::USE_IMAGE_DATA_FORMAT;
        /*  switch(getImageOptions(layerNum)->getTextureType())
        {
        case(BuildOptions::RGB_S3TC_DXT1): internalFormatMode = osg::Texture::USE_S3TC_DXT1_COMPRESSION; break;
        case(BuildOptions::RGBA_S3TC_DXT1): internalFormatMode = osg::Texture::USE_S3TC_DXT1_COMPRESSION; break;
        case(BuildOptions::RGBA_S3TC_DXT3): internalFormatMode = osg::Texture::USE_S3TC_DXT3_COMPRESSION; break;
        case(BuildOptions::RGBA_S3TC_DXT5): internalFormatMode = osg::Texture::USE_S3TC_DXT5_COMPRESSION; break;
        case(BuildOptions::ARB_COMPRESSED): internalFormatMode = osg::Texture::USE_ARB_COMPRESSION; break;
        case(BuildOptions::COMPRESSED_TEXTURE): internalFormatMode = osg::Texture::USE_S3TC_DXT1_COMPRESSION; break;
        case(BuildOptions::COMPRESSED_RGBA_TEXTURE): internalFormatMode = osg::Texture::USE_S3TC_DXT3_COMPRESSION; break;
        default: break;
        }*/
        //internalFormatMode = osg::Texture::USE_S3TC_DXT1_COMPRESSION;
        bool inlineImageFile = _dataSet->getDestinationTileExtension()==".ive" || _dataSet->getDestinationTileExtension()==".osgb" ;
        bool compressedImageSupported = inlineImageFile;
        bool compressedImageRequired = (internalFormatMode != osg::Texture::USE_IMAGE_DATA_FORMAT);
        //  image->s()>=minumCompressedTextureSize && image->t()>=minumCompressedTextureSize &&

        if (compressedImageSupported && compressedImageRequired )
        {
            log(osg::NOTICE,"Compressed image");

            bool generateMiMap = getImageOptions(layerNum)->getMipMappingMode()==DataSet::MIP_MAPPING_IMAGERY;
            bool resizePowerOfTwo = getImageOptions(layerNum)->getPowerOfTwoImages();
            vpb::compress(*_dataSet->getState(),*texture,internalFormatMode,generateMiMap,resizePowerOfTwo,_dataSet->getCompressionMethod(),_dataSet->getCompressionQuality());

            log(osg::INFO,">>>>>>>>>>>>>>>compressed image.<<<<<<<<<<<<<<");

        }

        stateset->setTextureAttribute(TEXUNIT_ARRAY, texture.get());
    }
#endif

    stateset->addUniform( new osg::Uniform("theTexture", TEXUNIT_ARRAY) );

    osg::Program* program = new osg::Program;
    program->setName( "projective_tex" );
    osg::ref_ptr<osg::Shader> lerpF=new osg::Shader( osg::Shader::FRAGMENT);
    osg::ref_ptr<osg::Shader> lerpV=new osg::Shader( osg::Shader::VERTEX);
    if(_mydataSet->_useBlending){
        loadShaderSource( lerpV, _mydataSet->_basePath+"/blend.vert" );

        // if(_mydataSet->_useAtlas){
        //   loadShaderSource( lerpF, _mydataSet->_basePath+"/blendAtlas.frag" );
        // }else
        {
            loadShaderSource( lerpF, _mydataSet->_basePath+"/blend.frag" );
        }

    }else{
        loadShaderSource( lerpF, _mydataSet->_basePath+"/pass.frag" );
        loadShaderSource( lerpV, _mydataSet->_basePath+"/blend.vert" );
    }
    program->addShader(  lerpF );
    program->addShader(  lerpV );
    program->addBindAttribLocation(_texCoordsAlias1.second,_texCoordsAlias1.first);
    program->addBindAttribLocation(_texCoordsAlias2.second,_texCoordsAlias2.first);
    program->addBindAttribLocation(_texCoordsAlias3.second,_texCoordsAlias3.first);
    program->addBindAttribLocation(_texCoordsAlias4.second,_texCoordsAlias4.first);





    stateset->setAttributeAndModes( program, osg::StateAttribute::ON );

    stateset->setDataVariance(osg::Object::STATIC);

    return stateset;
}
void MyCompositeDestination::unrefSubTileData()
{
    for(MyCompositeDestination::ChildList::iterator citr=_children.begin();
    citr!=_children.end();
    ++citr)
    {
        ((MyCompositeDestination*)((*citr).get()))->unrefLocalData();
    }
}
void MyCompositeDestination::unrefLocalData()
{
    for(CompositeDestination::TileList::iterator titr=_tiles.begin();
    titr!=_tiles.end();
    ++titr)
    {
        MyDestinationTile* tile = dynamic_cast<MyDestinationTile*>(titr->get());
        log(osg::INFO,"   unref tile level=%d X=%d Y=%d",tile->_level, tile->_tileX, tile->_tileY);
        tile->unrefData();
    }
}
void MyDestinationTile::unrefData()
{
    _imageLayerSet.clear();
    _terrain = 0;
    _models = 0;
    _atlasGen=0;
    _createdScene = 0;
    _stateset = 0;
}

void MyDestinationTile::generateStateAndSplitDrawables(vector<osg::Geometry*> &geoms,osg::Vec4Array *v, const osg::PrimitiveSet& prset,
                                                       const TexBlendCoord  &texCoordsArray,
                                                       const osg::Vec3Array &verts,int tex_size){
    if(!v)
        return;
    int numIdx=prset.getNumIndices();
    // printf("Num idx %d\n",numIdx);
    printf("Num idx %d %d\n",numIdx,v->size());

    idmap_t allIds=calcAllIds(v);
    // std::vector<osg::ref_ptr<osg::Image> > texture_images= _atlasGen->getImages();
    unsigned int a=v->size();
    unsigned int b=texCoordsArray[0]->size();
    //printf("%d != %d\n",a,(int)verts.size());
    if(a != 0)
        assert(a == verts.size());
    assert(a == b);
    std::vector<osg::ref_ptr<osg::Image> > texture_images=_atlasGen->getImages();
    remapArrayForTexturing(v,texCoordsArray,allIds);
    int numberOfGeoms=texture_images.size();
    std::vector<osg::DrawElementsUInt *>  primsets(numberOfGeoms+1);;
    std::vector<osg::Vec3Array *>  vertSplit(numberOfGeoms+1);
    std::vector<TexBlendCoord> texSplit(numberOfGeoms+1);
    geoms.resize(numberOfGeoms+1);
    int untexidx=(geoms.size()-1);
    for(int i=0; i< (int)geoms.size(); i++){
        geoms[i]= new osg::Geometry;
        vertSplit[i]=new osg::Vec3Array;
        geoms[i]->setVertexArray(vertSplit[i]);
        primsets[i] = new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES,0);
        geoms[i]->addPrimitiveSet(primsets[i]);
        //No texture in last set
        if(i == untexidx)
            continue;
        texSplit[i].resize(4);
        texSplit[i][0]=new osg::Vec3Array;
        texSplit[i][1]=new osg::Vec3Array;
        texSplit[i][2]=new osg::Vec3Array;
        texSplit[i][3]=new osg::Vec3Array;
        if(_mydataSet->_useBlending){
            setVertexAttrib(*geoms[i],_texCoordsAlias1,texSplit[i][0],false,osg::Geometry::BIND_PER_VERTEX);
            setVertexAttrib(*geoms[i],_texCoordsAlias2,texSplit[i][1],false,osg::Geometry::BIND_PER_VERTEX);
            setVertexAttrib(*geoms[i],_texCoordsAlias3,texSplit[i][2],false,osg::Geometry::BIND_PER_VERTEX);
            setVertexAttrib(*geoms[i],_texCoordsAlias4,texSplit[i][3],false,osg::Geometry::BIND_PER_VERTEX);
        }else{
            geoms[i]->setTexCoordArray(TEX_UNIT,texSplit[i][0]);
        }
        //geoms[i]->setUseDisplayList(false);
        osg::StateSet *stateset=geoms[i]->getOrCreateStateSet();
        osg::Image *tex=texture_images[i];
        char tmpf[255];
        sprintf(tmpf,"%d",rand());
        tex->setFileName(tmpf);

        osg::ref_ptr<osg::Texture2D> texture=new osg::Texture2D(tex);
        //  texture->setWrap(osg::Texture::WRAP_S,osg::Texture::CLAMP_TO_BORDER);
        //texture->setWrap(osg::Texture::WRAP_T,osg::Texture::CLAMP_TO_BORDER);
        //texture->setFilter(osg::Texture2D::MIN_FILTER,osg::Texture2D::LINEAR_MIPMAP_LINEAR);
        // texture->setFilter(osg::Texture2D::MAG_FILTER,osg::Texture2D::LINEAR);
        bool inlineImageFile = _dataSet->getDestinationTileExtension()==".ive" || _dataSet->getDestinationTileExtension()==".osgb" ;
        bool compressedImageSupported = inlineImageFile;
        //bool mipmapImageSupported = compressedImageSupported; // inlineImageFile;
        int layerNum=0;
        osg::Texture::InternalFormatMode internalFormatMode = osg::Texture::USE_IMAGE_DATA_FORMAT;
        switch(getImageOptions(layerNum)->getTextureType())
        {
        case(BuildOptions::RGB_S3TC_DXT1): internalFormatMode = osg::Texture::USE_S3TC_DXT1_COMPRESSION; break;
        case(BuildOptions::RGBA_S3TC_DXT1): internalFormatMode = osg::Texture::USE_S3TC_DXT1_COMPRESSION; break;
        case(BuildOptions::RGBA_S3TC_DXT3): internalFormatMode = osg::Texture::USE_S3TC_DXT3_COMPRESSION; break;
        case(BuildOptions::RGBA_S3TC_DXT5): internalFormatMode = osg::Texture::USE_S3TC_DXT5_COMPRESSION; break;
        case(BuildOptions::ARB_COMPRESSED): internalFormatMode = osg::Texture::USE_ARB_COMPRESSION; break;
        case(BuildOptions::COMPRESSED_TEXTURE): internalFormatMode = osg::Texture::USE_S3TC_DXT1_COMPRESSION; break;
        case(BuildOptions::COMPRESSED_RGBA_TEXTURE): internalFormatMode = osg::Texture::USE_S3TC_DXT3_COMPRESSION; break;
        default: break;
        }

        bool compressedImageRequired = (internalFormatMode != osg::Texture::USE_IMAGE_DATA_FORMAT);
        //  image->s()>=minumCompressedTextureSize && image->t()>=minumCompressedTextureSize &&

        if (0&&compressedImageSupported && compressedImageRequired )
        {
            log(osg::NOTICE,"Compressed image");

            bool generateMiMap = getImageOptions(layerNum)->getMipMappingMode()==DataSet::MIP_MAPPING_IMAGERY;
            bool resizePowerOfTwo = getImageOptions(layerNum)->getPowerOfTwoImages();
            vpb::compress(*_dataSet->getState(),*texture,internalFormatMode,generateMiMap,resizePowerOfTwo,_dataSet->getCompressionMethod(),_dataSet->getCompressionQuality());

            log(osg::INFO,">>>>>>>>>>>>>>>compressed image.<<<<<<<<<<<<<<");

        }


        stateset->setTextureAttributeAndModes(TEX_UNIT,texture,osg::StateAttribute::ON);
        stateset->setMode( GL_LIGHTING, osg::StateAttribute::PROTECTED | osg::StateAttribute::OFF );

        osg::Program* program = new osg::Program;
        program->setName( "projective_tex" );
        osg::ref_ptr<osg::Shader> lerpF=new osg::Shader( osg::Shader::FRAGMENT);
        osg::ref_ptr<osg::Shader> lerpV=new osg::Shader( osg::Shader::VERTEX);
        if(_mydataSet->_useBlending){
            loadShaderSource( lerpV, _mydataSet->_basePath+"/blend.vert" );

            if(_mydataSet->_useAtlas){
                loadShaderSource( lerpF,  _mydataSet->_basePath+"/blendAtlas.frag" );
            }else{
                loadShaderSource( lerpF,  _mydataSet->_basePath+"/blend.frag" );
            }

        }else{
            loadShaderSource( lerpF,  _mydataSet->_basePath+"/pass.frag" );
            loadShaderSource( lerpV,  _mydataSet->_basePath+"/blend.vert" );
        }
        program->addShader(  lerpF );
        program->addShader(  lerpV );
        program->addBindAttribLocation(_texCoordsAlias1.second,_texCoordsAlias1.first);
        program->addBindAttribLocation(_texCoordsAlias2.second,_texCoordsAlias2.first);
        program->addBindAttribLocation(_texCoordsAlias3.second,_texCoordsAlias3.first);
        program->addBindAttribLocation(_texCoordsAlias4.second,_texCoordsAlias4.first);
        if(_mydataSet->_useBlending)
            stateset->setAttributeAndModes( program, osg::StateAttribute::ON );

        stateset->setDataVariance(osg::Object::STATIC);
    }
    numIdx=prset.getNumIndices();
    int numIV=v->size();
    // printf("Num idx %d\n",numIdx);
    printf("Num idx %d %d\n",numIdx,v->size());

    assert(numIdx ==(int) v->size());
    for(int i=0; i<numIdx-2; i+=3){
        vector<osg::Vec3> vP;
        vector<osg::Vec3> tP[4];
        vector<unsigned int> iP;
        vector<SpatialIndex::id_type> idP;

        SpatialIndex::id_type id=-1;//int atlas=-1;
        bool untex=false;
        for(int k=0; k <3; k++){
            id = (v &&  v->size() && prset.index(i+k) < v->size()   )? (int)((*v)[prset.index(i+k)][0]) : -1;
            if(id < 0){
                untex=true;
            }else{
                //atlas=_atlasGen->getAtlasId(id);
                // if(atlas < 0 || atlas >= untexidx){
                //    OSG_ALWAYS << "Atlas mapping incorrect id: " << id << " index: " << prset.index(i+k) << endl;
                //   exit(-1);
                //}


                osg::Vec3 tc[4];
                for(int t=0; t <4; t++){
                    tc[t]= texCoordsArray[t]->at(prset.index(i+k));

                    //  osg::Matrix matrix=_atlasGen->getTextureMatrixByID(id);
                    tP[t].push_back(tc[t]);//osg::Vec2(tc[0]*matrix(0,0) + tc[1]*matrix(1,0) + matrix(3,0),
                }                     // tc[0]*matrix(0,1) + tc[1]*matrix(1,1) + matrix(3,1)));
                iP.push_back(vertSplit[id]->size()+k);

            }
            //  idP.push_back(atlas);
            vP.push_back((verts)[prset.index(i+k)]);
        }
        // assert(idP[0]== idP[1] == idP[2]);
        // if(idP[0]!= idP[1] || idP[1]!= idP[2] || idP[0]!= idP[2])
        //   untex=true;
        //assert(vP.size() == 3 && iP.size() == 3);
        int untexSize=vertSplit[untexidx]->size();
        for(int k=0; k <3; k++){
            if(!untex){
                vertSplit[id]->push_back(vP[k]);
                for(int t=0; t<4; t++)
                    texSplit[id][t]->push_back(tP[t][k]);
                primsets[id]->push_back(iP[k]);
            }else{
                vertSplit[untexidx]->push_back(vP[k]);
                primsets[untexidx]->push_back(untexSize+k);
            }
        }
    }

    if(!vertSplit[geoms.size()-1]->size()){
        geoms.resize(geoms.size()-1);
    }

}
osg::Node* MyCompositeDestination::createPagedLODScene()
{
    if (_children.empty() && _tiles.empty()) return 0;

    if (_children.empty() && _tiles.size()==1) { DestinationTile *t= _tiles.front();

        MyDestinationTile *myt=dynamic_cast<MyDestinationTile*>(t);

        osg::Node *node= myt?myt->createScene(): 0;

        if(node)return convertModel(node->asGroup());
        return NULL;

    }
    if (_tiles.empty() && _children.size()==1)  {
        CompositeDestination *c=_children.front();
        MyCompositeDestination *child=dynamic_cast<MyCompositeDestination*>(c);
        if(child){
            return convertModel(child->createPagedLODScene()->asGroup());
        }else return 0;
    }

    if (_type==GROUP)
    {
        osg::Group* group = new osg::Group;
        for(TileList::iterator titr=_tiles.begin();
        titr!=_tiles.end();
        ++titr)
        {
            DestinationTile *t=*titr;

            MyDestinationTile *myt=dynamic_cast<MyDestinationTile*>(t);

            osg::Node* node = myt?myt->createScene(): 0;

            if (node) group->addChild(node);
        }

        // handle chilren
        for(ChildList::iterator citr=_children.begin();
        citr!=_children.end();
        ++citr)
        {
            CompositeDestination *c=*citr;
            MyCompositeDestination *child=dynamic_cast<MyCompositeDestination*>(c);
            osg::Node* node = child ? child->createScene() : 0;
            if (node) group->addChild(node);
        }
        return group;
    }

    // must be either a LOD or a PagedLOD

    typedef std::vector<osg::Node*>  NodeList;

    // collect all the local tiles
    NodeList tileNodes;
    for(TileList::iterator titr=_tiles.begin();
    titr!=_tiles.end();
    ++titr)
    {
        DestinationTile *t=*titr;

        MyDestinationTile *myt=dynamic_cast<MyDestinationTile*>(t);

        osg::Node* node = myt?myt->createScene(): 0;
        if (node) tileNodes.push_back(node);
    }

    float cutOffDistance = -FLT_MAX;
    for(ChildList::iterator citr=_children.begin();
    citr!=_children.end();
    ++citr)
    {
        cutOffDistance = osg::maximum(cutOffDistance,(*citr)->_maxVisibleDistance);
    }


    osg::PagedLOD* pagedLOD = new osg::PagedLOD;

    float farDistance = _dataSet->getMaximumVisibleDistanceOfTopLevel();
    if (tileNodes.size()==1)
    {
        //   if(_useReImage){
        osg::Group *retex=convertModel(tileNodes.front()->asGroup());
        pagedLOD->addChild(retex);
        /*  }else{
            pagedLOD->addChild(tileNodes.front());
        }*/
    }
    else if (tileNodes.size()>1)
    {
        osg::Group* group = new osg::Group;
        for(NodeList::iterator itr=tileNodes.begin();
        itr != tileNodes.end();
        ++itr)
        {
            group->addChild(*itr);
        }
        // if(_useReImage){
        osg::Group *retex=convertModel(group);
        pagedLOD->addChild(retex);
        // }else{
        //   pagedLOD->addChild(group);
        // }
    }


    // find cluster culling callbacks on drawables and move them to the PagedLOD level.
    {
        CollectClusterCullingCallbacks collect;
        pagedLOD->accept(collect);

        if (!collect._callbackList.empty())
        {
            if (collect._callbackList.size()==1)
            {
                CollectClusterCullingCallbacks::Triple& triple = collect._callbackList.front();

                osg::Matrixd matrix = osg::computeLocalToWorld(triple._nodePath);

                triple._callback->transform(matrix);

                log(osg::INFO,"cluster culling matrix %f\t%f\t%f\t%f",matrix(0,0),matrix(0,1),matrix(0,2),matrix(0,3));
                log(osg::INFO,"                       %f\t%f\t%f\t%f",matrix(1,0),matrix(1,1),matrix(1,2),matrix(1,3));
                log(osg::INFO,"                       %f\t%f\t%f\t%f",matrix(2,0),matrix(2,1),matrix(2,2),matrix(2,3));
                log(osg::INFO,"                       %f\t%f\t%f\t%f",matrix(3,0),matrix(3,1),matrix(3,2),matrix(3,3));

                // moving cluster culling callback pagedLOD node.
                pagedLOD->setCullCallback(triple._callback);

                osg::Drawable* drawable = dynamic_cast<osg::Drawable*>(triple._object);
                if (drawable) drawable->setCullCallback(0);

                osg::Node* node = dynamic_cast<osg::Node*>(triple._object);
                if (node) node->setCullCallback(0);
            }
        }
    }

    cutOffDistance =(pagedLOD->getBound().radius()*_dataSet->getRadiusToMaxVisibleDistanceRatio());// osg::maximum(cutOffDistance,(float)(pagedLOD->getBound().radius()*_dataSet->getRadiusToMaxVisibleDistanceRatio()));
printf("AP Level %d Cutoff distance %f Radius %f \n",_level,cutOffDistance,pagedLOD->getBound().radius());
cout << "AP Center " <<pagedLOD->getBound().center()<<std::endl;
    pagedLOD->setRange(0,cutOffDistance,farDistance);

    pagedLOD->setFileName(1,getExternalSubTileName());
    pagedLOD->setRange(1,0,cutOffDistance);

    if (pagedLOD->getNumChildren()>0)
        pagedLOD->setCenter(pagedLOD->getBound().center());
    if(pagedLOD->getNumChildren() ==0)
        return NULL;
    return pagedLOD;
}
void MyDataSet::_buildDestination(bool writeToDisk)
{
    if (!_state) _state = new osg::State;

    osg::ref_ptr<osgDB::ReaderWriter::Options> previous_options = osgDB::Registry::instance()->getOptions();
    if(previous_options.get())
    {
        log(osg::NOTICE, "vpb: adding optionstring %s",previous_options->getOptionString().c_str());
        osgDB::Registry::instance()->setOptions(new osgDB::ReaderWriter::Options(std::string("precision 16") + std::string(" ") + previous_options->getOptionString()) );
    }
    else
    {
        osgDB::Registry::instance()->setOptions(new osgDB::ReaderWriter::Options("precision 16"));
    }

    if (!_archive && !_archiveName.empty())
    {
        unsigned int indexBlockSizeHint=4096;
        _archive = osgDB::openArchive(_archiveName, osgDB::Archive::CREATE, indexBlockSizeHint);
    }

    if (_destinationGraph.valid())
    {
#ifdef NEW_NAMING
        std::string filename = _destinationGraph->getTileFileName();
#else
        std::string filename = _directory+_tileBasename+_tileExtension;
#endif

        if (_archive.valid())
        {
            log(osg::NOTICE, "started MyDataSet::writeDestination(%s)",_archiveName.c_str());
            log(osg::NOTICE, "        archive file = %s",_archiveName.c_str());
            log(osg::NOTICE, "        archive master file = %s",filename.c_str());
        }
        else
        {
            log(osg::NOTICE, "started MyDataSet::writeDestination(%s)",filename.c_str());
        }

        /* if (_databaseType==LOD_DATABASE)
        {
            populateDestinationGraphFromSources();
            _rootNode = _destinationGraph->createScene();

            if (_decorateWithMultiTextureControl)
            {
                _rootNode = decorateWithMultiTextureControl(_rootNode.get());
            }

            if (getGeometryType()==TERRAIN)
            {
                _rootNode = decorateWithTerrain(_rootNode.get());
            }
            else if (_decorateWithCoordinateSystemNode)
            {
                _rootNode = decorateWithCoordinateSystemNode(_rootNode.get());
            }

            if (!_comment.empty())
            {
                _rootNode->addDescription(_comment);
            }

            if (writeToDisk)
            {
                _writeNodeFileAndImages(*_rootNode,filename);
            }
        }
        else  // _databaseType==PagedLOD_DATABASE*/
        {

            // for each level build read and write the rows.
            for(QuadMap::iterator qitr=_quadMap.begin();
                qitr!=_quadMap.end();
                ++qitr)
            {
                Level& level = qitr->second;

                // skip is level is empty.
                if (level.empty()) continue;

                // skip lower levels if we are generating subtiles
                if (getGenerateSubtile() && qitr->first<=getSubtileLevel()) continue;

                if (getRecordSubtileFileNamesOnLeafTile() && qitr->first>=getMaximumNumOfLevels()) continue;

                log(osg::NOTICE, "New level");

                Level::iterator prev_itr = level.begin();
                _readRow(prev_itr->second);
                Level::iterator curr_itr = prev_itr;
                ++curr_itr;
                for(;
                    curr_itr!=level.end();
                    ++curr_itr)
                {
                    _readRow(curr_itr->second);

                    _equalizeRow(prev_itr->second);
                    if (writeToDisk) _writeRow(prev_itr->second);

                    prev_itr = curr_itr;
                }

                _equalizeRow(prev_itr->second);

                if (writeToDisk)
                {
                    if (writeToDisk) _writeRow(prev_itr->second);
                }

#if 0
                if (_writeThreadPool.valid()) _writeThreadPool->waitForCompletion();
#endif
            }

        }

        if (_archive.valid())
        {
            log(osg::NOTICE, "completed MyDataSet::writeDestination(%s)",_archiveName.c_str());
            log(osg::NOTICE, "          archive file = %s",_archiveName.c_str());
            log(osg::NOTICE, "          archive master file = %s",filename.c_str());
        }
        else
        {
            log(osg::NOTICE, "completed MyDataSet::writeDestination(%s)",filename.c_str());
        }

        if (_writeThreadPool.valid()) _writeThreadPool->waitForCompletion();

    }
    else
    {
        log(osg::WARN, "Error: no scene graph to output, no file written.");
    }

    if (_archive.valid()) _archive->close();

    osgDB::Registry::instance()->setOptions(previous_options.get());

   /* for(QuadMap::iterator qitr = _quadMap.begin();
        qitr != _quadMap.end();
        ++qitr)
    {
        QuadMap::iterator temp_itr = qitr;
        ++temp_itr;
        if (temp_itr==_quadMap.end()) continue;

        Level& level = qitr->second;
        for(Level::iterator litr = level.begin();
            litr != level.end();
            ++litr)
        {
            Row& row = litr->second;
            for(Row::iterator ritr = row.begin();
                ritr != row.end();
                ++ritr)
            {
                MyCompositeDestination* cd = dynamic_cast<MyCompositeDestination*>(ritr->second);
                if(cd)
                     cd->unref();
            }
        }
    }*/

}
void MyDataSet::_equalizeRow(Row& row)
{
    log(osg::INFO, "_equalizeRow %d",row.size());
    for(Row::iterator citr=row.begin();
    citr!=row.end();
    ++citr)
    {
        CompositeDestination* cd = citr->second;
        //if(!cd)
        //     continue;
        for(CompositeDestination::TileList::iterator titr=cd->_tiles.begin();
        titr!=cd->_tiles.end();
        ++titr)
        {
            DestinationTile* tile = titr->get();
            log(osg::INFO, "   equalizing tile level=%u X=%u Y=%u",tile->_level,tile->_tileX,tile->_tileY);
            // tile->equalizeBoundaries();
            tile->setTileComplete(true);
        }
    }
}
void MyDestinationTile::setVertexAttrib(osg::Geometry& geom, const AttributeAlias& alias, osg::Array* array, bool normalize, osg::Geometry::AttributeBinding binding)
{
    if(!array)
        return;
    unsigned int index = alias.first;
    const std::string& name = alias.second;
    array->setName(name);
    geom.setVertexAttribArray(index, array);
    geom.setVertexAttribNormalize(index, normalize);
    geom.setVertexAttribBinding(index, binding);

    osg::notify(osg::NOTICE)<<"   vertex attrib("<<name<<", index="<<index<<", normalize="<<normalize<<" binding="<<binding<<")"<<std::endl;
}
void fillPrimTexCoordFromVert(const osg::PrimitiveSet& prset,const TexBlendCoord &texCoord,osg::Vec4Array *ids,TexBlendCoord &newTexCoord,osg::Vec4Array *newIds){
    newIds->resize(prset.getNumIndices());
    newTexCoord.resize(4);
    newTexCoord[0]=new osg::Vec3Array(prset.getNumIndices());
    newTexCoord[1]=new osg::Vec3Array(prset.getNumIndices());
    newTexCoord[2]=new osg::Vec3Array(prset.getNumIndices());
    newTexCoord[3]=new osg::Vec3Array(prset.getNumIndices());

    for(unsigned int i2=0; i2<prset.getNumIndices()-2; i2+=3)
    {
        for(int k=0; k <3; k++){
            newIds->at(i2+k)= ids->at(prset.index(i2+k));
            for(int t=0;t <4; t++)
                newTexCoord[t]->at(i2+k)= texCoord[t]->at(prset.index(i2+k));

        }
    }
}
osg::StateSet* createSS()
{
    osg::StateSet* stateset = new osg::StateSet();

    // osg::Image* image = osgDB::readImageFile( "/Users/julian/Desktop/test.rgb" );
    unsigned char bla [8][8] = {{1,2,3,4,5,6,7,8},{1,2,3,4,5,6,7,8},{1,2,3,4,5,6,7,8},{1,2,3,4,5,6,7,8},{1,2,3,4,5,6,7,8},{1,2,3,4,5,6,7,8},{1,2,3,4,5,6,7,8},{1,2,3,4,5,6,7,8}};
    osg::Image* ti = new osg::Image;
    ti->setImage(8, 8, 1,  GL_RGB, GL_RGB, GL_UNSIGNED_BYTE, (unsigned char *) &bla, osg::Image::NO_DELETE, 1);

    if (ti)
    {
        osg::Texture2D* texture = new osg::Texture2D;
        texture->setImage(ti);
        texture->setFilter(osg::Texture::MIN_FILTER, osg::Texture::LINEAR);
        stateset->setTextureAttributeAndModes(0,texture, osg::StateAttribute::ON);
    }

    stateset->setMode(GL_LIGHTING, osg::StateAttribute::ON);

    return stateset;
}
osg::Node* MyDestinationTile::createScene()
{
    if (_createdScene.valid()) return _createdScene.get();
    /*
    if (_dataSet->getGeometryType()==DataSet::HEIGHT_FIELD)
    {
        _createdScene = createHeightField();
    }
    else if (_dataSet->getGeometryType()==DataSet::TERRAIN)
    {
        _createdScene = createTerrainTile();
    }
    else
    {*/
    // _createdScene = createPolygonal();
    //}

    if (_models.valid())
    {
        /*  if (_dataSet->getModelPlacer())
        {
            for(ModelList::iterator itr = _models->_models.begin();
                itr != _models->_models.end();
                ++itr)
            {
                _dataSet->getModelPlacer()->place(*this, itr->get());
            }
        }
        else*/
        {
            int tex_size;

            if(!_mydataSet->_useVirtualTex){

                int numtex=_atlasGen->_totalImageList.size();
                double memsize;

                /*for( tex_size=16; tex_size <= 2048; tex_size*=2){
                memsize=(((tex_size)*(tex_size)*numtex*4)/1024.0)/1024.0;
                if(memsize >1.0)
                    break;
            }
            //Got the size that was over target get size under target
            tex_size=max(16,tex_size/2);*/
                int start_pow=9;
                tex_size=1024;//log2 = 10
                int leveloffset=(_hintNumLevels-_level);
                //printf("level offset %d level %d\n",leveloffset,_level);
                tex_size=pow(2,start_pow-leveloffset);
                tex_size=max(tex_size,8);
                memsize=(((tex_size)*(tex_size)*numtex*4)/1024.0)/1024.0;

                //  int texSizeIdx=levelToTextureLevel[_level];
                _atlasGen->loadTextureFiles(tex_size);
            }

            //  log(osg::NOTICE, "   dst: level=%u X=%u Y=%u size=%dx%d images=%d MemSize:%.2f MB",_level,_tileX,_tileY,tex_size,tex_size,numtex,memsize);
            //   printf("   dst: level=%u X=%u Y=%u size=%dx%d images=%d MemSize:%.2f MB\n",_level,_tileX,_tileY,tex_size,tex_size,numtex,memsize);

            //printf("tile Level %d texure level size %d\n",_level,_atlasGen->getDownsampleSize(levelToTextureLevel[_level]));
            int cnt=0;
            osg::ref_ptr<osg::Vec4Array> v=new osg::Vec4Array;
            TexBlendCoord texCoords;
            if(!_mydataSet->_useVirtualTex){
                texCoords.resize(4);
                texCoords[0]=new osg::Vec3Array;
                texCoords[1]=new osg::Vec3Array;
                texCoords[2]=new osg::Vec3Array;
                texCoords[3]=new osg::Vec3Array;
            }else{
                texCoords.resize(1);
                texCoords[0]=new osg::Vec3Array;
            }

            for(ModelList::iterator itr = _models->_models.begin();
            itr != _models->_models.end();
            ++itr,++cnt)
            {
                if(_atlasGen->_totalImageList.size()> 0 ||texCoordsPerModel.size()){
                    if(texCoordsPerModel.count(*itr) == 0 ){
                        OSG_ALWAYS << "Not correct number of texCoordsPerModel in createScene() "<< cnt << " "<<texCoordsPerModel.size() <<endl;
                    }else{
                        const TexBlendCoord &tmp2=texCoordsPerModel[*itr];
                        //  cout << tmp->size() << " ASS " << tmp2[0]->size() << endl;
                        if(!_mydataSet->_useVirtualTex){
                            osg::Vec4Array *tmp=texCoordIDIndexPerModel[*itr];
                            if(tmp && tmp->size()){
                                for(int i=0; i<(int)tmp->size(); i++)
                                    (*v).push_back(tmp->at(i));

                            }else{
                                OSG_FATAL << "Null Ptr texCoordIDIndexPerModel" <<endl;
                            }

                        }
                        for(int f=0; f<(int)tmp2.size(); f++){
                            //printf("%d %d %d\n",tmp2.size(),tmp2[f]->size(),tmp->size());

                            for(int i=0; i<(int)tmp2[f]->size(); i++){
                                osg::Vec3 a=tmp2[f]->at(i);
                                texCoords[f]->push_back(a);
                            }
                        }

                    }
                }
                addNodeToScene(itr->get());
            }
            /* idmap_t allIds=calcAllIds(v);
            numtex=allIds.size();
            memsize=(((tex_size)*(tex_size)*numtex*4)/1024.0)/1024.0;
            //printf("   dst2: level=%u X=%u Y=%u size=%dx%d images=%d MemSize:%.2f MB\n",_level,_tileX,_tileY,tex_size,tex_size,numtex,memsize);
*/
            OSG_INFO<< "Number of coords "<< texCoords[0]->size() << endl;
            if(_createdScene){
                osgUtil::Optimizer::MergeGeodesVisitor visitor;

                _createdScene->accept(visitor);
                osgUtil::Optimizer::MergeGeometryVisitor mgv;
                mgv.setTargetMaximumNumberOfVertices(INT_MAX);
                _createdScene->accept(mgv);

                osgUtil::GeometryCollector gc(NULL, osgUtil::Optimizer::DEFAULT_OPTIMIZATIONS);
                _createdScene->accept(gc);
                osgUtil::GeometryCollector::GeometryList geomList = gc.getGeometryList();
                if(geomList.size() > 1){
                    OSG_ALWAYS << "Number of collected geometies " << geomList.size() << "problem "<<endl;
                    //   OSG_FATAL << "Number of collected geometies " << geomList.size() << "problem "<<endl;
                    assert(0);

                }
                if(!_mydataSet->_useVirtualTex){
                    if(geomList.size() && _atlasGen->_totalImageList.size()> 0 ){
                        osg::Geometry *geom=*geomList.begin();

                        if(_mydataSet->_useTextureArray && !_mydataSet->_useAtlas){
                            osg::StateSet *stateset=generateStateAndArray2DRemap(v,texCoords,0);
                            //setVertexAttrib(*geom,_projCoordAlias,v,false,osg::Geometry::BIND_PER_VERTEX);
                            setVertexAttrib(*geom,_texCoordsAlias1,texCoords[0],false,osg::Geometry::BIND_PER_VERTEX);
                            setVertexAttrib(*geom,_texCoordsAlias2,texCoords[1],false,osg::Geometry::BIND_PER_VERTEX);
                            setVertexAttrib(*geom,_texCoordsAlias3,texCoords[2],false,osg::Geometry::BIND_PER_VERTEX);
                            setVertexAttrib(*geom,_texCoordsAlias4,texCoords[3],false,osg::Geometry::BIND_PER_VERTEX);
                            geom->setUseDisplayList(_mydataSet->_useDisplayLists);
                            geom->setStateSet(stateset);
                        }else{
                            vector<osg::Geometry*> geoms;
                            osg::Group *group=dynamic_cast<osg::Group*>(_createdScene.get());
                            osg::ref_ptr<osg::Geode> geode = dynamic_cast<osg::Geode*> (group->getChild(0));
                            osg::Vec3Array *verts=static_cast<const osg::Vec3Array*>(geom->getVertexArray());
                            osg::Geometry::PrimitiveSetList& primitiveSets = geom->getPrimitiveSetList();
                            int s=primitiveSets[0]->getNumIndices();
                            osg::PrimitiveSet &primset=*(primitiveSets.begin()->get());
                            int s2=verts->size();
                            int s3=texCoords[0]->size();
                            TexBlendCoord newTexCoord;
                            osg::Vec4Array *newIds=new osg::Vec4Array;
                            fillPrimTexCoordFromVert(primset,texCoords,v,newTexCoord,newIds);

                            // toVert(geode,newTexCoord,newIds,texCoords,v);
                            s=primitiveSets[0]->getNumIndices();
                            s2=v->size();
                            s3=texCoords[0]->size();
                            generateStateAndSplitDrawables(geoms,v,primset,texCoords,*verts,tex_size);
                            geode->removeDrawables(0);
                            for(int i=0; i < (int)geoms.size(); i++)
                                geode->addDrawable(geoms[i]);
                        }

                    }
                }else{
                    osg::Geometry *geom=*geomList.begin();
                    osg::StateSet *stateset=createSS();
                    geom->setUseDisplayList(_mydataSet->_useDisplayLists);
                    geom->setUseVertexBufferObjects(_mydataSet->_useVBO);
                    geom->setStateSet(stateset);
                    geom->setTexCoordArray(0,texCoords[0]);
                    int t=texCoords[0]->size();

                    //std::cout << "Drops : "<<texCoords[0]->size() <<"\n";
                }
            }
        }

        /*  if (_dataSet->getShapeFilePlacer())
        {
            for(ModelList::iterator itr = _models->_shapeFiles.begin();
                itr != _models->_shapeFiles.end();
                ++itr)
            {
                _dataSet->getShapeFilePlacer()->place(*this, itr->get());
            }
        }
        else
        {
            for(ModelList::iterator itr = _models->_shapeFiles.begin();
                itr != _models->_shapeFiles.end();
                ++itr)
            {
                addNodeToScene(itr->get());
            }
        }*/
    }else {
        //if (!_createdScene) _createdScene = new osg::Group;
    }

    return _createdScene.get();
}

MyCompositeDestination* MyDataSet::createDestinationTile(int currentLevel, int currentX, int currentY)
{
    CompositeDestination* parent = 0;
    GeospatialExtents extents;

    if (currentLevel==0)
    {
        extents = _destinationExtents;
    }
    else
    {
        // compute the extents
        double destination_xRange = _destinationExtents.xMax()-_destinationExtents.xMin();
        double destination_yRange = _destinationExtents.yMax()-_destinationExtents.yMin();

        int Ck = int(pow(2.0, double(currentLevel-1))) * _C1;
        int Rk = int(pow(2.0, double(currentLevel-1))) * _R1;

        extents.xMin() = _destinationExtents.xMin() + (double(currentX)/double(Ck)) * destination_xRange;
        extents.xMax() = _destinationExtents.xMin() + (double(currentX+1)/double(Ck)) * destination_xRange;

        extents.yMin() = _destinationExtents.yMin() + (double(currentY)/double(Rk)) * destination_yRange;
        extents.yMax() = _destinationExtents.yMin() + (double(currentY+1)/double(Rk)) * destination_yRange;

        // compute the parent
        if (currentLevel == 1)
        {
            parent = _destinationGraph.get();
        }
        else
        {
            parent = getComposite(currentLevel-1,currentX/2,currentY/2);

            if (!parent)
            {
                log(osg::NOTICE,"Warning: getComposite(%i,%i,%i) return 0",currentLevel-1,currentX/2,currentY/2);
            }
        }

    }

    MyCompositeDestination* destinationGraph = new MyCompositeDestination(_intermediateCoordinateSystem.get(),extents,_useReImage,_useVBO,_useDisplayLists,_file,_fileMutex);

    if (currentLevel==0) _destinationGraph = destinationGraph;

    /*if (mapLatLongsToXYZ())
    {
        // we need to project the extents into world coords to get the appropriate size to use for control max visible distance
        float max_range = osg::maximum(extents.xMax()-extents.xMin(),extents.yMax()-extents.yMin());
        float projected_radius =  osg::DegreesToRadians(max_range) * getEllipsoidModel()->getRadiusEquator();
        float center_offset = (max_range/360.0f) * getEllipsoidModel()->getRadiusEquator();
        destinationGraph->_maxVisibleDistance = projected_radius * getRadiusToMaxVisibleDistanceRatio() + center_offset;
    }
    else*/
    {
        destinationGraph->_maxVisibleDistance = extents.radius()*getRadiusToMaxVisibleDistanceRatio();
    }

    // first create the topmost tile

    // create the name
    std::ostringstream os;
    os << _tileBasename << "_L"<<currentLevel<<"_X"<<currentX<<"_Y"<<currentY;
    //std::cout << os.str() << "extents " << extents._min  << " " << extents._max<<std::endl;
    destinationGraph->_parent = parent;
    destinationGraph->_name = os.str();
    destinationGraph->_level = currentLevel;
    destinationGraph->_tileX = currentX;
    destinationGraph->_tileY = currentY;
    destinationGraph->_dataSet = this;

    MyDestinationTile* tile = new MyDestinationTile(_cachedDirs);
    tile->_name = destinationGraph->_name;
    tile->_level = currentLevel;
    tile->_tileX = currentX;
    tile->_tileY = currentY;
    tile->_dataSet = this;
    tile->_mydataSet=this;
    tile->_cs = destinationGraph->_cs;
    tile->_extents = extents;
    tile->_parent = destinationGraph;
    tile->_atlasGen->_useTextureArray = this->_useTextureArray;
    tile->_atlasGen->_useAtlas = this->_useAtlas;

    // set to NONE as the tile is a mix of RASTER and VECTOR
    // that way the default of RASTER for image and VECTOR for height is maintained
    tile->_dataType = SpatialProperties::NONE;

    tile->setMaximumTerrainSize(_maximumTileTerrainSize,_maximumTileTerrainSize);

    destinationGraph->_tiles.push_back(tile);

    if (parent)
    {
        parent->_type = LOD;
        parent->addChild(destinationGraph);
    }


    insertTileToQuadMap(destinationGraph);

    return destinationGraph;
}

void MyDataSet::createNewDestinationGraph(
        const GeospatialExtents& extents,
        unsigned int maxImageSize,
        unsigned int maxTerrainSize,
        unsigned int maxNumLevels)
{


    //int highestLevelFound = 0;
    _destinationExtents=extents;
    computeOptimumTileSystemDimensions(_C1,_R1);

    // first populate the destination graph from imagery and DEM sources extents/resolution
    //for(CompositeSource::source_iterator itr(_sourceGraph.get());itr.valid();++itr)
    {
        /*  Source* source = (*itr).get();



        SourceData* sd = (*itr)->getSourceData();
        if (!sd)
        {
            log(osg::NOTICE,"Skipping source %s as no data loaded from it.",source->getFileName().c_str());
            continue;
        }

        const SpatialProperties& sp = sd->computeSpatialProperties(cs);

        if (!sp._extents.intersects(extents))
        {
            // skip this source since it doesn't overlap this tile.
            log(osg::NOTICE,"Skipping source %s as its extents don't overlap destination extents.",source->getFileName().c_str());
            continue;
        }


        if (source->getType()!=Source::IMAGE && source->getType()!=Source::HEIGHT_FIELD)
        {
            continue;
        }

        int k = 0;
        if (!computeOptimumLevel(source, maxNumLevels-1, k)) continue;


        if (k>highestLevelFound) highestLevelFound = k;
*/
        int k=maxNumLevels;
        int startLevel = 0; // getGenerateSubtile() ? getSubtileLevel() : 0;

        for(int l=startLevel; l<=k; l++)
        {
            int i_min, i_max, j_min, j_max;
            if (computeCoverage(extents, l, i_min, j_min, i_max, j_max))
            {
                printf("     level=%i i_min=%i i_max=%i j_min=%i j_max=%i\n",l, i_min, i_max, j_min, j_max);

                if (getGenerateSubtile())
                {
                    int i_lower, i_upper, j_lower, j_upper;

                    if (l<static_cast<int>(getSubtileLevel()))
                    {
                        // divide by 2 to the power of ((getSubtileLevel()-l);
                        int delta = getSubtileLevel()-l;
                        i_lower = getSubtileX() >> delta;
                        j_lower = getSubtileY() >> delta;
                        i_upper = i_lower + 1;
                        j_upper = j_lower + 1;
                    }
                    else
                    {
                        // multiply 2 to the power of ((l-getSubtileLevel());
                        int f = 1 << (l-getSubtileLevel());
                        i_lower = getSubtileX() * f;
                        j_lower = getSubtileY() * f;
                        i_upper = i_lower + f;
                        j_upper = j_lower + f;
                    }

                    if (i_min<i_lower) i_min = i_lower;
                    if (i_max>i_upper) i_max = i_upper;
                    if (j_min<j_lower) j_min = j_lower;
                    if (j_max>j_upper) j_max = j_upper;
                }

                for(int j=j_min; j<j_max;++j)
                {
                    for(int i=i_min; i<i_max;++i)
                    {
                        CompositeDestination* cd = getComposite(l,i,j);
                        if (!cd)
                        {
                            cd = createDestinationTile(l,i,j);
                        }
                    }
                }
            }
        }
    }
    // printf("Total Size %d\n",(int)_quadMap.size());
    // now extend the sources upwards where required.
    for(QuadMap::iterator qitr = _quadMap.begin();
    qitr != _quadMap.end();
    ++qitr)
    {
        QuadMap::iterator temp_itr = qitr;
        ++temp_itr;
        if (temp_itr==_quadMap.end()) continue;

        int l = qitr->first;

        Level& level = qitr->second;
        for(Level::iterator litr = level.begin();
        litr != level.end();
        ++litr)
        {
            Row& row = litr->second;
            for(Row::iterator ritr = row.begin();
            ritr != row.end();
            ++ritr)
            {
                CompositeDestination* cd = ritr->second;

                int numChildren = cd->_children.size();
                int numChildrenExpected = (l==0) ? (_C1*_R1) : 4;
                if (numChildren!=0 && numChildren!=numChildrenExpected)
                {
#if 0
                    printf("  tile (%i,%i,%i) numTiles=%i numChildren=%i\n",
                           cd->_level, cd->_tileX, cd->_tileY, cd->_tiles.size(), cd->_children.size());
#endif
                    int i_min = (l==0) ? 0   : (cd->_tileX * 2);
                    int j_min = (l==0) ? 0   : (cd->_tileY * 2);
                    int i_max = (l==0) ? _C1 : i_min + 2;
                    int j_max = (l==0) ? _R1 : j_min + 2;
                    int new_l = l+1;

                    if (getGenerateSubtile())
                    {
                        int i_lower, i_upper, j_lower, j_upper;


                        if (l<static_cast<int>(getSubtileLevel()))
                        {
                            // divide by 2 to the power of ((getSubtileLevel()-new_l);
                            int delta = getSubtileLevel()-new_l;
                            i_lower = getSubtileX() >> delta;
                            j_lower = getSubtileY() >> delta;
                            i_upper = i_lower + 1;
                            j_upper = j_lower + 1;
                        }
                        else
                        {
                            // multiply 2 to the power of ((new_l-getSubtileLevel());
                            int f = 1 << (new_l-getSubtileLevel());
                            i_lower = getSubtileX() * f;
                            j_lower = getSubtileY() * f;
                            i_upper = i_lower + f;
                            j_upper = j_lower + f;
                        }

                        if (i_min<i_lower) i_min = i_lower;
                        if (i_max>i_upper) i_max = i_upper;
                        if (j_min<j_lower) j_min = j_lower;
                        if (j_max>j_upper) j_max = j_upper;
                    }

                    for(int j=j_min; j<j_max;++j)
                    {
                        for(int i=i_min; i<i_max;++i)
                        {
                            CompositeDestination* cd = getComposite(new_l,i,j);
                            if (!cd)
                            {
                                cd = createDestinationTile(new_l,i,j);
                            }
                        }
                    }

                }
            }
        }
    }

    // now insert the sources into the destination graph
    for(CompositeSource::source_iterator itr(_sourceGraph.get());itr.valid();++itr)
    {
        Source* source = (*itr).get();

        if (source->getMinLevel()>maxNumLevels)
        {
            log(osg::ALWAYS,"Skipping source %s as its min level excees destination max level.",source->getFileName().c_str());
            continue;
        }

        if (getGenerateSubtile() && source->getMaxLevel()<getSubtileLevel())
        {
            log(osg::ALWAYS,"Skipping source %s as its max level is lower than the subtile level.",source->getFileName().c_str());
            continue;
        }

        SourceData* sd = (*itr)->getSourceData();
        if (!sd)
        {
            log(osg::ALWAYS,"Skipping source %s as no data loaded from it.",source->getFileName().c_str());
            continue;
        }
        osg::CoordinateSystemNode* cs = _intermediateCoordinateSystem.get();

        const SpatialProperties& sp = sd->computeSpatialProperties(cs);
        //std::cout << sp._extents._min << " " << sp._extents._max << " " << source->getFileName() <<std::endl;
        if (!sp._extents.intersects(extents))
        {
            // skip this source since it doesn't overlap this tile.
            log(osg::ALWAYS,"Skipping source %s as its extents don't overlap destination extents.",source->getFileName().c_str());
            continue;
        }

        int k = maxNumLevels;

        /* if (source->getType()==Source::IMAGE || source->getType()==Source::HEIGHT_FIELD)
        {
            if (!computeOptimumLevel(source, maxNumLevels-1, k)) continue;
        }
        else
        {
            k = highestLevelFound;
        }
*/

        // log(osg::ALWAYS,"     opt level = %i",k);

        int startLevel = 0; // getGenerateSubtile() ? getSubtileLevel() : 0;

        for(int l=startLevel; l<=k; l++)
        {
            int i_min, i_max, j_min, j_max;
            if (computeCoverage(extents, l, i_min, j_min, i_max, j_max))
            {
                // log(osg::NOTICE,"     level=%i i_min=%i i_max=%i j_min=%i j_max=%i",l, i_min, i_max, j_min, j_max);

                if (getGenerateSubtile())
                {
                    int i_lower, i_upper, j_lower, j_upper;

                    if (l<static_cast<int>(getSubtileLevel()))
                    {
                        // divide by 2 to the power of ((getSubtileLevel()-l);
                        int delta = getSubtileLevel()-l;
                        i_lower = getSubtileX() >> delta;
                        j_lower = getSubtileY() >> delta;
                        i_upper = i_lower + 1;
                        j_upper = j_lower + 1;
                    }
                    else
                    {
                        // multiply 2 to the power of ((l-getSubtileLevel());
                        int f = 1 << (l-getSubtileLevel());
                        i_lower = getSubtileX() * f;
                        j_lower = getSubtileY() * f;
                        i_upper = i_lower + f;
                        j_upper = j_lower + f;
                    }

                    if (i_min<i_lower) i_min = i_lower;
                    if (i_max>i_upper) i_max = i_upper;
                    if (j_min<j_lower) j_min = j_lower;
                    if (j_max>j_upper) j_max = j_upper;
                }

                for(int j=j_min; j<j_max;++j)
                {
                    for(int i=i_min; i<i_max;++i)
                    {
                        CompositeDestination* cd = getComposite(l,i,j);
                        if (!cd || !cd->intersects(sp) || (l <(int)source->getMinLevel() || l > (int)source->getMaxLevel())) continue;

                        // printf("Tile %d %d_%d %s\n",l ,i,j,source->getFileName().c_str());
                        /*  if (l==k)
                        {
                            cd->addSource(source);
                        }
                        else*/
                        dynamic_cast<MyCompositeDestination*>(cd)->_numLevels=maxNumLevels;

                        {
                            for(CompositeDestination::TileList::iterator titr = cd->_tiles.begin();
                            titr != cd->_tiles.end();
                            ++titr)
                            {
                                MyDestinationTile* tile = dynamic_cast<MyDestinationTile*>(titr->get());
                                TexturedSource* texsource = dynamic_cast<TexturedSource*>(source);

                                tile->addSourceWithHint(texsource,extents);
                                tile->setHintNumLevels(maxNumLevels);
                            }
                        }
                    }
                }
            }
        }
    }

    /*   int startLevel = 0; // getGenerateSubtile() ? getSubtileLevel() : 0;
  int k = maxNumLevels;
    for(int l=startLevel; l<=k; l++)
    {
        int i_min, i_max, j_min, j_max;
        if (computeCoverage(extents, l, i_min, j_min, i_max, j_max))
        {
            // log(osg::NOTICE,"     level=%i i_min=%i i_max=%i j_min=%i j_max=%i",l, i_min, i_max, j_min, j_max);

            if (getGenerateSubtile())
            {
                int i_lower, i_upper, j_lower, j_upper;

                if (l<static_cast<int>(getSubtileLevel()))
                {
                    // divide by 2 to the power of ((getSubtileLevel()-l);
                    int delta = getSubtileLevel()-l;
                    i_lower = getSubtileX() >> delta;
                    j_lower = getSubtileY() >> delta;
                    i_upper = i_lower + 1;
                    j_upper = j_lower + 1;
                }
                else
                {
                    // multiply 2 to the power of ((l-getSubtileLevel());
                    int f = 1 << (l-getSubtileLevel());
                    i_lower = getSubtileX() * f;
                    j_lower = getSubtileY() * f;
                    i_upper = i_lower + f;
                    j_upper = j_lower + f;
                }

                if (i_min<i_lower) i_min = i_lower;
                if (i_max>i_upper) i_max = i_upper;
                if (j_min<j_lower) j_min = j_lower;
                if (j_max>j_upper) j_max = j_upper;
            }

            for(int j=j_min; j<j_max;++j)
            {
                for(int i=i_min; i<i_max;++i)
                {
                    CompositeDestination* cd = getComposite(l,i,j);


                        for(CompositeDestination::TileList::iterator titr = cd->_tiles.begin();
                        titr != cd->_tiles.end();
                        ++titr)
                        {
                            MyDestinationTile* tile = dynamic_cast<MyDestinationTile*>(titr->get());
                            if(tile->_sources.size() == 0)
                                 _quadMap[tile->_level][tile->_tileY][tile->_tileX]=NULL;
                        }

                }
            }
        }
    }
*/
    osg::Timer_t before_computeMax = osg::Timer::instance()->tick();

    if (_destinationGraph.valid()) _destinationGraph->computeMaximumSourceResolution();

    osg::Timer_t after_computeMax = osg::Timer::instance()->tick();

    log(osg::INFO,"Time for _destinationGraph->computeMaximumSourceResolution() = %f", osg::Timer::instance()->delta_s(before_computeMax, after_computeMax));
}

void MyDataSet::processTile(MyDestinationTile *tile,TexturedSource *src){
    if(tile->_sources.size() == 0)
        return;
    log(osg::INFO,"   source:%s",src->getFileName().c_str());
    osg::BoundingBox ext_bbox(osg::Vec3d(tile->_extents._min.x(),
                                         tile->_extents._min.y(),
                                         -DBL_MAX),osg::Vec3d(tile->_extents._max.x(),
                                                              tile->_extents._max.y(),
                                                              DBL_MAX));
    osg::ref_ptr<osg::Node> root;


    osg::Vec4Array *ids= _useVirtualTex ? NULL :src->ids;
    TexBlendCoord  *texCoords=(&src->tex);
    osg::ref_ptr<osg::Vec4Array> new_ids;
    TexBlendCoord  new_texCoords;
    bool projectSucess=!_useReImage;//!_reimagePass;//false;//(ids!=NULL && !_reimagePass);
    if(src->_kdTree){
        osg::ref_ptr<KdTreeBbox> kdtreeBbox=new KdTreeBbox(*src->_kdTree);
        //  if(projectSucess){
        root=kdtreeBbox->intersect(ext_bbox, ids,*texCoords,new_texCoords,new_ids,IntersectKdTreeBbox::DUP);
        //printf("0x%x\n",(long int)root.get());
        OpenThreads::ScopedLock<OpenThreads::Mutex> lock(tile->_texCoordMutex);
        for(int f=0; f< new_texCoords.size(); f++)
            assert(new_texCoords[f].valid());
        if(new_ids.valid())
            tile->texCoordIDIndexPerModel[root.get()]=new_ids;
        tile->texCoordsPerModel[root.get()]=new_texCoords;
        //    }else
        //       root=kdtreeBbox->intersect(ext_bbox,IntersectKdTreeBbox::DUP);
    }else{
        OSG_ALWAYS<<"No kdtree\n";
        assert(0);
    }
    if(!_useVirtualTex){
        TexturingQuery *tq=new TexturingQuery(src,_calib,*(tile->_atlasGen),_useTextureArray);
        tq->_tile=tile;
        bool _reimagePass=(tile->_level != tile->_hintNumLevels);
        std::string mf=src->getFileName();
        int npos=mf.find("/");
        std::string bbox_name=std::string(mf.substr(0,npos)+"/bbox-"+mf.substr(npos+1,mf.size()-9-npos-1)+".ply.txt");
        if(!projectSucess && !_useReImage) {
            projectSucess=tq->projectModel(dynamic_cast<osg::Geode*>(root.get()));
        }

        if(projectSucess){
            {
                OpenThreads::ScopedLock<OpenThreads::Mutex> lock(tile->_texCoordMutex);

                map<SpatialIndex::id_type,int> allIds=calcAllIds(tile->texCoordIDIndexPerModel[root.get()]);
                tq->addImagesToAtlasGen(allIds);
                //int level=tile->_level;
                //int total=tq->_atlasGen._totalImageList.size();
                //  printf("%d %d %d %d 0x%x AAAAAA\n",level,total,tile->_tileX,tile->_tileY,(long int)&(tq->_atlasGen));
            }
        }
        delete tq;

    }
    {
        OpenThreads::ScopedLock<OpenThreads::Mutex> lock(tile->_modelMutex);

        if(!tile->_models){
            tile->_models = new DestinationData(NULL);
        }
        tile->_models->_models.push_back(root.get());
    }


}

class MyReadFromOperation : public BuildOperation
{
public:

    MyReadFromOperation(ThreadPool* threadPool, BuildLog* buildLog, DestinationTile* tile, MyDataSet* dataSet,TexturedSource *src):
            BuildOperation(threadPool, buildLog, "ReadFromOperation", false),
            _tile(tile),
            _dataSet(dataSet),
            _src(src){}

    virtual void build()
    {
        log(osg::NOTICE, "   ReadFromOperation: reading tile level=%u X=%u Y=%u",_tile->_level,_tile->_tileX,_tile->_tileY);
        MyDestinationTile *myt=dynamic_cast<MyDestinationTile*>(_tile);
        _dataSet->processTile(myt,_src);
        //  _dataSet->processTile(_tile,_src);

    }

    DestinationTile* _tile;
    MyDataSet* _dataSet;
    TexturedSource *_src;
};

void MyDataSet::_readRow(Row& row)
{
    log(osg::NOTICE, "_readRow %u",row.size());

    if (_readThreadPool.valid())
    {
        for(Row::iterator citr=row.begin();
        citr!=row.end();
        ++citr)
        {
            CompositeDestination* cd = citr->second;
            //if(!cd)
            //     continue;
            for(CompositeDestination::TileList::iterator titr=cd->_tiles.begin();
            titr!=cd->_tiles.end();
            ++titr)
            {
                for(DestinationTile::Sources::iterator itr = (*titr)->_sources.begin();
                itr != (*titr)->_sources.end();
                ++itr)
                {
                    TexturedSource *source=dynamic_cast<TexturedSource*>((*itr).get());
                    _readThreadPool->run(new MyReadFromOperation(_readThreadPool.get(), getBuildLog(), titr->get(), this,source));
                }
            }
        }

        // wait for the threads to complete.
        _readThreadPool->waitForCompletion();


    }
    else
    {
        for(Row::iterator citr=row.begin();
        citr!=row.end();
        ++citr)
        {
            CompositeDestination* cd = citr->second;
            for(CompositeDestination::TileList::iterator titr=cd->_tiles.begin();
            titr!=cd->_tiles.end();
            ++titr)
            {
                DestinationTile* tile = titr->get();
                log(osg::NOTICE, "   reading tile level=%u X=%u Y=%u",tile->_level,tile->_tileX,tile->_tileY);
                if(tile->_sources.size())
                    log(osg::NOTICE, "   src: %s ",tile->_sources.front()->getFileName().c_str());

                //tile->readFrom(sourceGraph);
                for(DestinationTile::Sources::iterator itr = tile->_sources.begin();
                itr != tile->_sources.end();
                ++itr)
                {
                    MyDestinationTile *myt=dynamic_cast<MyDestinationTile*>(tile);
                    TexturedSource *source=dynamic_cast<TexturedSource*>((*itr).get());
                    processTile(myt,source);
                }

            }
        }
    }
}

void ClippedCopy::AnalyzePrimSet(const osg::PrimitiveSet& prset, const osg::Vec3Array &verts)
{
    //std::cout  << "Prim set type "<< prset.getMode() << std::endl;
    std::vector<int> vertRemap(verts.size(),-1);
    if(!_triangles.valid())
        _triangles = new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES, 0);
    if(1)
    {
        unsigned int ic;
        //unsigned int nprim=0;
        //unsigned int numValid=0;
        for (ic=0; ic < prset.getNumIndices(); ic++)
        {
            // NB the vertices are held in the drawable -
            /*std::cout  <<  "vertex "<< ic << " is index "<<prset.index(ic) << " at " <<
           (verts)[prset.index(ic)].x() << "," <<
           (verts)[prset.index(ic)].y() << "," <<
           (verts)[prset.index(ic)].z() << std::endl;*/
            if(_bbox.contains( (verts)[prset.index(ic)])){
                vertRemap[prset.index(ic)]=_vertices->size();
                _vertices->push_back((verts)[prset.index(ic)]);
            }



        }
        // you might want to handle each type of primset differently: such as:

        switch (prset.getMode())
        {
        case osg::PrimitiveSet::TRIANGLES: // get vertices of triangle
            {
                // std::cout << "Triangles "<< nprim << " is index "<<prset.index(ic) << std::endl;
                for(unsigned int i2=0; i2<prset.getNumIndices()-2; i2+=3)
                {
                    std::vector<bool> outside(3,false);
                    for(int k=0; k <3; k++){
                        outside[k]=(vertRemap[prset.index(i2+k)]>= 0);
                    }

                    if(outside[0] && outside[1] && outside[2]){
                        for(int k=0; k <3; k++){
                            _triangles->push_back(vertRemap[prset.index(i2+k)]);
                        }
                    }
                }
            }
            break;

        default:
            break;
        }
    }
}

osg::Geode* ClippedCopy::makeCopy(osg::Geode *geode){
    osg::Geode *newGeode=new osg::Geode;
    osg::Geometry *new_geom=new osg::Geometry;
    _vertices=new osg::Vec3Array;
    for (unsigned int i=0; i<geode->getNumDrawables(); i++)
    {
        osg::Drawable *drawable = geode->getDrawable(i);
        osg::Geometry *geom = dynamic_cast< osg::Geometry*>(drawable);
        osg::Geometry::PrimitiveSetList& primitiveSets = geom->getPrimitiveSetList();
        osg::Geometry::PrimitiveSetList::iterator itr;


        for(itr=primitiveSets.begin(); itr!=primitiveSets.end(); ++itr){
            switch((*itr)->getMode()){

            case(osg::PrimitiveSet::TRIANGLES):
                //  for( unsigned int j = 0; j < (*itr)->getNumPrimitives(); j++ )
                // {
                //   const osg::PrimitiveSet* prset = (*itr);//geom->getPrimitiveSet(j);
                AnalyzePrimSet(*(*itr), *static_cast<const osg::Vec3Array*>(geom->getVertexArray()));

                break;

            default:
                fprintf(stderr,"ERRROR Only supported for TRIANGLES\n");
            }
        }
    }
    new_geom->addPrimitiveSet(_triangles);
    new_geom->setVertexArray(_vertices.get());
    //osgUtil::SmoothingVisitor::smooth(*new_geom);
    newGeode->addDrawable(new_geom);
    return newGeode;
}


class MyGraphicsContext : public osg::Referenced {
public:
    MyGraphicsContext(BuildLog* buildLog)
    {
        osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
        traits->readDISPLAY();
        traits->x = 0;
        traits->y = 0;
        traits->width = 1;
        traits->height = 1;
        traits->windowDecoration = false;
        traits->doubleBuffer = false;
        traits->sharedContext = 0;
        traits->pbuffer = true;

        _gc = osg::GraphicsContext::createGraphicsContext(traits.get());

        if (!_gc)
        {
            if (buildLog) buildLog->log(osg::NOTICE,"Failed to create pbuffer, failing back to normal graphics window.");

            traits->pbuffer = false;
            _gc = osg::GraphicsContext::createGraphicsContext(traits.get());
        }

        if (_gc.valid())


        {
            _gc->realize();
            _gc->makeCurrent();

            if (buildLog) buildLog->log(osg::NOTICE,"Realized window");
        }
    }

    bool valid() const { return _gc.valid() && _gc->isRealized(); }

private:
    osg::ref_ptr<osg::GraphicsContext> _gc;
};

int MyDataSet::_run()
{

    log(osg::NOTICE,"MyDataSet::_run() %i %i",getDistributedBuildSplitLevel(),getDistributedBuildSecondarySplitLevel());

#ifdef HAVE_NVTT
    bool requiresGraphicsContextInMainThread = (getCompressionMethod() == vpb::BuildOptions::GL_DRIVER);
    bool requiresGraphicsContextInWritingThread = (getCompressionMethod() == vpb::BuildOptions::GL_DRIVER);
#else
    //bool requiresGraphicsContextInMainThread = true;
    bool requiresGraphicsContextInWritingThread = true;
#endif

    int numProcessors = OpenThreads::GetNumberOfProcessors();
#if 0
    if (numProcessors>1)
#endif
    {
        int numReadThreads = int(ceilf(getNumReadThreadsToCoresRatio() * float(numProcessors)));
        if (numReadThreads>=1)
        {
            log(osg::NOTICE,"Starting %i read threads.",numReadThreads);
            _readThreadPool = new ThreadPool(numReadThreads, requiresGraphicsContextInWritingThread);
            _readThreadPool->startThreads();
        }

        int numWriteThreads = int(ceilf(getNumWriteThreadsToCoresRatio() * float(numProcessors)));
        if (numWriteThreads>=1)
        {
            log(osg::NOTICE,"Starting %i write threads.",numWriteThreads);
            _writeThreadPool = new ThreadPool(numWriteThreads, requiresGraphicsContextInWritingThread);
            _writeThreadPool->startThreads();

            //requiresGraphicsContextInMainThread = false;
        }
    }

    /*
    //loadSources();

    int numLevels = getMaximumNumOfLevels();
    if (getRecordSubtileFileNamesOnLeafTile()) ++numLevels;

    createDestination(numLevels);

    if (!_destinationGraph)
    {
        log(osg::WARN, "Error: no destination graph built, cannot proceed with build.");
        return 1;
    }

*/
    bool requiresGenerationOfTiles = getGenerateTiles();

    /*  if (!getIntermediateBuildName().empty())
    {
        osg::ref_ptr<osgTerrain::TerrainTile> terrainTile = createTerrainRepresentation();
        if (terrainTile.valid())
        {
            DatabaseBuilder* db = dynamic_cast<DatabaseBuilder*>(terrainTile->getTerrainTechnique());
            if (db && db->getBuildOptions())
            {
                db->getBuildOptions()->setIntermediateBuildName("");
            }
            _writeNodeFile(*terrainTile,getIntermediateBuildName());
            requiresGenerationOfTiles = false;
        }
    }


    bool printOutContributingSources = true;
    if (printOutContributingSources)
    {
        CompositeDestination* startPoint = _destinationGraph.get();
        if (getGenerateSubtile())
        {
            startPoint = getComposite(getSubtileLevel(), getSubtileX(), getSubtileY());
        }

        if (startPoint)
        {
            DestinationTile::Sources sources = startPoint->getAllContributingSources();
            log(osg::NOTICE,"There are %d contributing source files:",sources.size());

            for(DestinationTile::Sources::iterator itr = sources.begin();
                itr != sources.end();
                ++itr)
            {
                log(osg::NOTICE,"    %s",(*itr)->getFileName().c_str());
            }
        }
        else
        {
            log(osg::NOTICE,"Warning: No destination graph generated.");
        }
    }
*/
    if (requiresGenerationOfTiles)
    {
        // dummy Viewer to get round silly Windows autoregistration problem for GraphicsWindowWin32.cpp
        //osgViewer::Viewer viewer;
        /*
        osg::ref_ptr<MyGraphicsContext> context;

        if (requiresGraphicsContextInMainThread)
        {
            context  = new MyGraphicsContext(getBuildLog());
            if (!context || !context->valid())
            {
                log(osg::NOTICE,"Error: Unable to create graphis context, problem with running osgViewer-%s, cannot run compression.",osgViewerGetVersion());
                return 1;
            }
        }
*/
        int result = 0;
        osgDB::FileType type = osgDB::fileType(getDirectory());
        if (type==osgDB::DIRECTORY)
        {
            log(osg::NOTICE,"   Base Directory already created");
        }
        else if (type==osgDB::REGULAR_FILE)
        {
            log(osg::NOTICE,"   Error cannot create directory as a conventional file already exists with that name");
            return 1;
        }
        else // FILE_NOT_FOUND
        {
            // need to create directory.
            result = vpb::mkpath(getDirectory().c_str(), S_IRWXU | S_IRWXG | S_IRWXO);
        }

        if (result)
        {
            log(osg::NOTICE,"Error: could not create directory, errorno=%i",errno);
            return 1;
        }


        if (getOutputTaskDirectories())
        {
            _taskOutputDirectory = getDirectory() + getTaskName(getSubtileLevel(), getSubtileX(), getSubtileY());
            log(osg::NOTICE,"Need to create output task directory = %s", _taskOutputDirectory.c_str());
            result = 0;
            type = osgDB::fileType(_taskOutputDirectory);
            if (type==osgDB::DIRECTORY)
            {
                log(osg::NOTICE,"   Directory already created");
            }
            else if (type==osgDB::REGULAR_FILE)
            {
                log(osg::NOTICE,"   Error cannot create directory as a conventional file already exists with that name");
                return 1;
            }
            else // FILE_NOT_FOUND
            {
                // need to create directory.
                result = vpb::mkpath(_taskOutputDirectory.c_str(), S_IRWXU | S_IRWXG | S_IRWXO);
            }

            if (result)
            {
                log(osg::NOTICE,"Error: could not create directory, errorno=%i",errno);
                return 1;
            }

#ifdef WIN32
            _taskOutputDirectory.push_back('\\');
#else
            _taskOutputDirectory.push_back('/');
#endif

            if (getGenerateSubtile()) log(osg::NOTICE,"We are a subtile");
            if (getRecordSubtileFileNamesOnLeafTile()) log(osg::NOTICE,"We have to record ../task/name");
        }
        else
        {
            _taskOutputDirectory = getDirectory();
        }

        log(osg::NOTICE,"Task output directory = %s", _taskOutputDirectory.c_str());

        writeDestination();
    }

    return 0;
}

osg::Node* MyCompositeDestination::createSubTileScene()
{
    if (_type==GROUP ||
        _children.empty() ||
        _tiles.empty()) return 0;

    // handle chilren
    typedef std::vector<osg::Node*>  NodeList;
    NodeList nodeList;
    for(ChildList::iterator citr=_children.begin();
    citr!=_children.end();
    ++citr)
    {
        CompositeDestination *c=*citr;
        MyCompositeDestination *child=dynamic_cast<MyCompositeDestination*>(c);
        osg::Node* node = child ? child->createPagedLODScene() : 0;
        if (node) nodeList.push_back(node);
    }

    if (nodeList.size()==1)
    {
        return nodeList.front();
    }
    else if (nodeList.size()>1)
    {
        osg::Group* group = new osg::Group;
        for(NodeList::iterator itr=nodeList.begin();
        itr!=nodeList.end();
        ++itr)
        {
            group->addChild(*itr);
        }
        return group;
    }
    else
    {
        return 0;
    }
}
class WriteOperation : public BuildOperation
{
public:

    WriteOperation(ThreadPool* threadPool, MyDataSet* dataset,MyCompositeDestination* cd, const std::string& filename):
            BuildOperation(threadPool, dataset->getBuildLog(), "WriteOperation", false),
            _dataset(dataset),
            _cd(cd),
            _filename(filename) {}

    virtual void build()
    {
        //notify(osg::NOTICE)<<"   WriteOperation"<<std::endl;

        osg::ref_ptr<osg::Node> node = _cd->createSubTileScene();
        if (node.valid())
        {
            if (_buildLog.valid()) _buildLog->log(osg::NOTICE, "   writeSubTile filename= %s",_filename.c_str());

            _dataset->_writeNodeFileAndImages(*node,_filename);

            _cd->setSubTilesGenerated(true);
            _cd->unrefSubTileData();
        }
        else
        {
            log(osg::WARN, "   failed to writeSubTile node for tile, filename=%s",_filename.c_str());
        }
    }

    MyDataSet*                            _dataset;
    osg::ref_ptr<MyCompositeDestination>  _cd;
    std::string                         _filename;
};

void MyDataSet::_writeRow(Row& row)
{
    log(osg::NOTICE, "_writeRow %u",row.size());
    /*  if(row.size()){
        MyCompositeDestination::TileList t=row.begin()->second->_tiles;
        if(t.size()){
            MyDestinationTile *myt=dynamic_cast<MyDestinationTile*>(&(*t.front()));
            if(myt){
                int texsize=myt->getTextureSizeForLevel(myt->_level);
                int numtex=myt->_atlasGen->_totalImageList.size();
                double memsize=((texsize*texsize*numtex*3)/1024.0)/1024.0;
                log(osg::NOTICE, "   dst: level texture size %d images=%d Size:%.2f MB",texsize,numtex,memsize);
            }
        }
    }*/

    for(Row::iterator citr=row.begin();
    citr!=row.end();
    ++citr)
    {
        MyCompositeDestination* cd = dynamic_cast<MyCompositeDestination*>(citr->second);
        //    if(!cd)
        //       continue;
        MyCompositeDestination* parent =  dynamic_cast<MyCompositeDestination*>(cd->_parent);

        if (parent)
        {
            if (!parent->getSubTilesGenerated() && parent->areSubTilesComplete())
            {
                parent->setSubTilesGenerated(true);

#ifdef NEW_NAMING
                std::string filename = cd->getTileFileName();
#else
                std::string filename = _taskOutputDirectory+parent->getSubTileName();
#endif
                log(osg::INFO, "       _taskOutputDirectory= %s",_taskOutputDirectory.c_str());

                if (_writeThreadPool.valid())
                {
                    _writeThreadPool->run(new WriteOperation(_writeThreadPool.get(), this, parent, filename));
                }
                else
                {
                    osg::ref_ptr<osg::Node> node = parent->createSubTileScene();
                    if (node.valid())
                    {
                        log(osg::NOTICE, "   writeSubTile filename= %s",filename.c_str());
                        _writeNodeFileAndImages(*node,filename);


                        parent->setSubTilesGenerated(true);
                        parent->unrefSubTileData();

                    }
                    else
                    {
                        log(osg::WARN, "   failed to writeSubTile node for tile, filename=%s",filename.c_str());
                    }
                }
            }
        }
        else
        {
            osg::ref_ptr<osg::Node> node = cd->createPagedLODScene();

#ifdef NEW_NAMING
            std::string filename = cd->getTileFileName();
#else
            std::string filename;
#endif

            if (cd->_level==0)
            {

#ifndef NEW_NAMING
                filename = getDirectory() + _tileBasename + _tileExtension;
#endif
                if (_decorateWithMultiTextureControl)
                {
                    node = decorateWithMultiTextureControl(node.get());
                }

                if (getGeometryType()==TERRAIN)
                {
                    node = decorateWithTerrain(node.get());
                }
                else if (_decorateWithCoordinateSystemNode)
                {
                    node = decorateWithCoordinateSystemNode(node.get());
                }

                if (!_comment.empty())
                {
                    node->addDescription(_comment);
                }

                log(osg::NOTICE, "       getDirectory()= %s",getDirectory().c_str());
            }
            else
            {
#ifndef NEW_NAMING
                filename = _taskOutputDirectory + _tileBasename + _tileExtension;
#endif

                log(osg::NOTICE, "       _taskOutputDirectory= %s",_taskOutputDirectory.c_str());
            }

            if (node.valid())
            {
                log(osg::NOTICE, "   writeNodeFile = %u X=%u Y=%u filename=%s",cd->_level,cd->_tileX,cd->_tileY,filename.c_str());

                _writeNodeFileAndImages(*node,filename);
            }
            else
            {
                log(osg::WARN, "   faild to write node for tile = %u X=%u Y=%u filename=%s",cd->_level,cd->_tileX,cd->_tileY,filename.c_str());
            }

            // record the top nodes as the rootNode of the database
            _rootNode = node;

        }
    }

#if 0
    if (_writeThreadPool.valid()) _writeThreadPool->waitForCompletion();
#endif

}
