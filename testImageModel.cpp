#include "PosterPrinter.h"
#include <osg/ArgumentParser>
#include <osgDB/WriteFile>
#include <osgDB/ReadFile>
#include <osg/MatrixTransform>
#include <osg/io_utils>
#include <osg/Texture2D>
#include <osg/TriangleIndexFunctor>
#include <osg/TriangleFunctor>

/* Computing view matrix helpers */
template<class T>
class FindTopMostNodeOfTypeVisitor : public osg::NodeVisitor
{
public:
    FindTopMostNodeOfTypeVisitor()
        :   osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN),
        _foundNode(0)
    {}

    void apply( osg::Node& node )
    {
        T* result = dynamic_cast<T*>( &node );
        if ( result ) _foundNode = result;
        else traverse( node );
    }

    T* _foundNode;
};

template<class T>
T* findTopMostNodeOfType( osg::Node* node )
{
    if ( !node ) return 0;

    FindTopMostNodeOfTypeVisitor<T> fnotv;
    node->accept( fnotv );
    return fnotv._foundNode;
}

osg::Vec2 calcCoordReproj(const osg::Vec3 &vert,const osg::Matrix &toScreen,const osg::Vec2 &size){
    osg::Vec3 tc=vert*toScreen;
    tc.x()/=size.x();
    tc.y()/=size.y();
    return osg::Vec2(tc.x(),tc.y());
}
// triangle functor.
struct JoinTriangleFunctorClass
{


    osg::Vec3Array *newVerts;
    osg::DrawElementsUInt* newPrimitiveSet;


    JoinTriangleFunctorClass() {

        newVerts= new osg::Vec3Array;
        newPrimitiveSet = new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES,0);
    }

    inline void operator() ( const osg::Vec3 &v1, const osg::Vec3 &v2, const osg::Vec3 &v3, bool &)
    {

        int cur=newVerts->size();
        newVerts->push_back(v1);
        newVerts->push_back(v2);
        newVerts->push_back(v2);
        newPrimitiveSet->addElement(cur);
        newPrimitiveSet->addElement(cur+1);
        newPrimitiveSet->addElement(cur+2);



    }
};

typedef osg::TriangleFunctor<JoinTriangleFunctorClass> JoinTriangleFunctor;
int main( int argc, char **argv )
{
    // use an ArgumentParser object to manage the program arguments.
    osg::ArgumentParser arguments(&argc,argv);
    osg::ref_ptr<osg::Image> image;
    osg::Matrix trans(   osg::Matrix::rotate(osg::inDegrees(-90.0f),
                                             1.0f,0.0f,0.0f)*
                         osg::Matrix::rotate(osg::inDegrees(-90.0f),0.0f,
                                             1.0f,0.0f));
    osg::Vec2 texSize(512,512);

    osg::Matrix toScreen;
    osg::Node* model = osgDB::readNodeFile(argv[1]);
    osg::Vec2Array *texCoord=new osg::Vec2Array();
    osg::Geometry *newGeom = new osg::Geometry;
    osg::Group *group= findTopMostNodeOfType<osg::Group>(model);
    osg::Vec3Array *newVerts= new osg::Vec3Array;
    osg::DrawElementsUInt* newPrimitiveSet = new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES,0);
    JoinTriangleFunctor jtf;

    for(int i=0; i< group->getNumChildren(); i++){

        osg::Group *group2  = dynamic_cast< osg::Group*>(group->getChild(i));
        osg::Geode *geode=group2->getChild(0)->asGeode();
        osg::Drawable *drawable=geode->getDrawable(0);
        osg::Geometry *geom = dynamic_cast< osg::Geometry*>(drawable);
         osg::Vec3Array *verts=static_cast<const osg::Vec3Array*>(geom->getVertexArray());
        osg::DrawElementsUInt* primitiveSet = dynamic_cast<osg::DrawElementsUInt*>(geom->getPrimitiveSet(0));
       int offset=newVerts->size();
        for(int j=0; j< (int)verts->size(); j++){
            newVerts->push_back(verts->at(j));
        }
        for(int j=0; j< (int)primitiveSet->getNumIndices(); j++){
            newPrimitiveSet->addElement(offset+primitiveSet->getElement(j));
        }

    }


    render(model,image,toScreen,texSize);
    for(int j=0; j< (int)newVerts->size(); j++){
        std::cout <<calcCoordReproj(newVerts->at(j),toScreen,texSize) << std::endl;

        texCoord->push_back(calcCoordReproj(newVerts->at(j),toScreen,texSize));
    }

    newGeom->setTexCoordArray(0,texCoord);
    newGeom->setVertexArray(newVerts);
    newGeom->addPrimitiveSet(newPrimitiveSet);
    osg::Geode *newGeode=new osg::Geode;
    newGeode->addDrawable(newGeom);
    osg::ref_ptr<osg::Texture2D> texture=new osg::Texture2D(image);
    texture->setWrap(osg::Texture::WRAP_S,osg::Texture::CLAMP_TO_BORDER);
    texture->setWrap(osg::Texture::WRAP_T,osg::Texture::CLAMP_TO_BORDER);
    texture->setFilter(osg::Texture2D::MIN_FILTER,osg::Texture2D::LINEAR_MIPMAP_LINEAR);
    texture->setFilter(osg::Texture2D::MAG_FILTER,osg::Texture2D::LINEAR);

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
    }

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
*/
    osg::StateSet *stateset=newGeode->getOrCreateStateSet();
    stateset->setTextureAttributeAndModes(0,texture,osg::StateAttribute::ON);
    stateset->setMode( GL_LIGHTING, osg::StateAttribute::PROTECTED | osg::StateAttribute::OFF );

    stateset->setDataVariance(osg::Object::STATIC);
    osgDB::Registry::instance()->writeImage( *image,"ass.png",NULL);
    osgDB::writeNodeFile(*newGeode,"test.ive");

    // osg::Vec3 v(1972.38,3932.55,0);
    //osg::Vec3 v(302.3,334.3,0);
    //  std::cout << v*toScreen << " " << toScreen<<std::endl;
    // osgDB::Registry::instance()->writeImage( *image,"ass.png",NULL);


}
