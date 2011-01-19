#include "PosterPrinter.h"
#include <osg/ArgumentParser>
#include <osgDB/WriteFile>
#include <osgDB/ReadFile>
#include <osg/MatrixTransform>
#include <osg/io_utils>
#include <osg/Texture2D>
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
    //render(model,image,toScreen,texSize);
    osg::Geode *geode= findTopMostNodeOfType<osg::Geode>(model);
    osg::Geometry *newGeom = new osg::Geometry;
    osg::Vec2Array *texCoord=new osg::Vec2Array();
    osg::Vec3Array *newVerts= new osg::Vec3Array;

    /*for(int i=0; i< group->getNumChildren(); i++){
*/
        osg::Drawable *drawable = /*group->getChild(0)->asGeode()*/geode->getDrawable(0);
        osg::Geometry *geom = dynamic_cast< osg::Geometry*>(drawable);
        osg::Vec3Array *verts=static_cast<const osg::Vec3Array*>(geom->getVertexArray());
        for(int j=0; j< (int)verts->size(); j++){
            texCoord->push_back(calcCoordReproj(verts->at(j),toScreen,texSize));
            newVerts->push_back(verts->at(j));
        }
        osg::DrawElementsUInt* primitiveSet = dynamic_cast<osg::DrawElementsUInt*>(geom->getPrimitiveSet(0));
        osg::DrawElementsUInt* newPrimitiveSet = new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES,0);

        for(int j=0; j< (int)primitiveSet->getNumPrimitives(); j++){
            newPrimitiveSet->addElement(primitiveSet->getElement(j));
        }

    //}
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

    osgDB::writeNodeFile(*newGeode,"test.ive");

    // osg::Vec3 v(1972.38,3932.55,0);
    //osg::Vec3 v(302.3,334.3,0);
    //  std::cout << v*toScreen << " " << toScreen<<std::endl;
   // osgDB::Registry::instance()->writeImage( *image,"ass.png",NULL);


}
