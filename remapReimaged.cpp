/* -*-c++-*- OpenSceneGraph - Copyright (C) 1998-2006 Robert Osfield
 *
 * This application is open source and may be redistributed and/or modified
 * freely and without restriction, both in commericial and non commericial applications,
 * as long as this copyright notice is maintained.
 *
 * This application is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/

#include <osgDB/ReadFile>
#include <osgDB/WriteFile>

#include <osgUtil/Optimizer>
#include <osg/CoordinateSystemNode>

#include <osg/Switch>
#include <osgText/Text>

#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osg/ComputeBoundsVisitor>
#include <osgDB/FileNameUtils>
#include <osgGA/TrackballManipulator>
#include <osgGA/FlightManipulator>
#include <osgGA/DriveManipulator>
#include <osgGA/KeySwitchMatrixManipulator>
#include <osgGA/StateSetManipulator>
#include <osgGA/AnimationPathManipulator>
#include <osgGA/TerrainManipulator>
#include <osgDB/FileUtils>
#include <iostream>
#include <sstream>
#include <string.h>
#include "imageNode.h"
osg::Group *remapNode(osg::Group *group, osg::ref_ptr<osg::Image> &image,osg::Matrix &viewProj,osg::Vec2 texSize){
    if(!group)
        return NULL;
    //
    if(group->getNumChildren() == 0)
        return group;




    osg::Vec2Array *texCoord=new osg::Vec2Array();
    osg::Geometry *newGeom = new osg::Geometry;
    // osg::Group *group= findTopMostNodeOfType<osg::Group>(model);
    osg::Vec3Array *newVerts= new osg::Vec3Array;
    osg::DrawElementsUInt* newPrimitiveSet = new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES,0);
    osg::Geode *newGeode=new osg::Geode;

    for(int i=0; i< (int)group->getNumChildren(); i++){

        osg::Group *group2  = dynamic_cast< osg::Group*>(group->getChild(i));
        osg::Geode *geode;
        if(group2)
            geode=group2->getChild(0)->asGeode();
        else
            geode = dynamic_cast< osg::Geode*>(group->getChild(i));
        for(int k=0; k<geode->getNumDrawables(); k++){
            osg::Drawable *drawable=geode->getDrawable(k);
            osg::Geometry *geom = dynamic_cast< osg::Geometry*>(drawable);
            osg::Vec3Array *verts=static_cast<const osg::Vec3Array*>(geom->getVertexArray());
            osg::DrawElementsUInt* primitiveSet = dynamic_cast<osg::DrawElementsUInt*>(geom->getPrimitiveSet(0));
            int offset=newVerts->size();
            if(!verts || !primitiveSet)
                continue;
            for(int j=0; j< (int)verts->size(); j++){
                newVerts->push_back(verts->at(j));
            }
            for(int j=0; j< (int)primitiveSet->getNumIndices(); j++){
                newPrimitiveSet->addElement(offset+primitiveSet->getElement(j));
            }

        }
    }



    osg::Matrix window= osg::Matrix::translate(1.0,1.0,1.0)*osg::Matrix::scale(0.5*texSize.x(),0.5*texSize.y(),0.5f);
    for(int j=0; j< (int)newVerts->size(); j++){
        /// std::cout <<calcCoordReproj(newVerts->at(j),toScreen,texSize) << std::endl;
        osg::Vec2 tc=calcCoordReproj(newVerts->at(j),viewProj,window,osg::Vec2(texSize.x(),texSize.y()));
        tc.y()= 1.0-tc.y();
        texCoord->push_back(tc);
    }
    for(int j=0; j< (int)newPrimitiveSet->getNumIndices(); j++){
        if(newPrimitiveSet->getElement(j) < 0 || newPrimitiveSet->getElement(j) > newVerts->size() ){
            printf("ASDADASDASDASDADS\n");
            exit(-1);
        }
    }
    //printf("%d %d\n",newPrimitiveSet->getNumIndices(),        newPrimitiveSet->getNumIndices()/3);


    newGeom->setTexCoordArray(0,texCoord);
    newGeom->setVertexArray(newVerts);
    newGeom->addPrimitiveSet(newPrimitiveSet);
    //newGeom->setUseDisplayList(_useDisplayLists);
    //newGeom->setUseVertexBufferObjects(_useVBO);

    newGeode->addDrawable(newGeom);
    osg::ref_ptr<osg::Texture2D> texture=new osg::Texture2D(image);
    texture->setWrap(osg::Texture::WRAP_S,osg::Texture::CLAMP_TO_BORDER);
    texture->setWrap(osg::Texture::WRAP_T,osg::Texture::CLAMP_TO_BORDER);
    texture->setFilter(osg::Texture2D::MIN_FILTER,osg::Texture2D::LINEAR_MIPMAP_LINEAR);
    texture->setFilter(osg::Texture2D::MAG_FILTER,osg::Texture2D::LINEAR);

    osg::Texture::InternalFormatMode internalFormatMode = osg::Texture::USE_IMAGE_DATA_FORMAT;
    internalFormatMode = osg::Texture::USE_S3TC_DXT1_COMPRESSION;
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
*/
    bool compressedImageRequired = (internalFormatMode != osg::Texture::USE_IMAGE_DATA_FORMAT);
    //  image->s()>=minumCompressedTextureSize && image->t()>=minumCompressedTextureSize &&

    if (0 &&/*compressedImageSupported && */compressedImageRequired )
    {
        //log(osg::NOTICE,"Compressed image");

        bool generateMiMap = true;//getImageOptions(layerNum)->getMipMappingMode()==DataSet::MIP_MAPPING_IMAGERY;
        bool resizePowerOfTwo = true;//getImageOptions(layerNum)->getPowerOfTwoImages();
        //vpb::compress(*_dataSet->getState(),*texture,internalFormatMode,generateMiMap,resizePowerOfTwo,_dataSet->getCompressionMethod(),_dataSet->getCompressionQuality());

        //log(osg::INFO,">>>>>>>>>>>>>>>compressed image.<<<<<<<<<<<<<<");

    }

    osg::StateSet *stateset=newGeode->getOrCreateStateSet();
    stateset->setTextureAttributeAndModes(0,texture,osg::StateAttribute::ON);
    stateset->setMode( GL_LIGHTING, osg::StateAttribute::PROTECTED | osg::StateAttribute::OFF );

    stateset->setDataVariance(osg::Object::STATIC);

    // osg::Vec3 v(1972.38,3932.55,0);
    //osg::Vec3 v(302.3,334.3,0);
    //  std::cout << v*toScreen << " " << toScreen<<std::endl;
    // osgDB::Registry::instance()->writeImage( *image,"ass.png",NULL);
    osg::Group *newGroup=new osg::Group;
    newGroup->addChild(newGeode);
    return newGroup;

}
bool readMatrixToScreen(std::string fname,osg::Matrix &viewProj,osg::Vec2 size){
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

int main(int argc, char** argv)
{
    // use an ArgumentParser object to manage the program arguments.
    osg::ArgumentParser arguments(&argc,argv);

    arguments.getApplicationUsage()->setApplicationName(arguments.getApplicationName());
    arguments.getApplicationUsage()->setCommandLineUsage(arguments.getApplicationName()+" [options] filename ...");

    osgViewer::Viewer viewer(arguments);

    unsigned int helpType = 0;
    if ((helpType = arguments.readHelpType()))
    {
        arguments.getApplicationUsage()->write(std::cout, helpType);
        return 1;
    }

    // report any errors if they have occurred when parsing the program arguments.
    if (arguments.errors())
    {
        arguments.writeErrorMessages(std::cout);
        return 1;
    }
    /*
    if (arguments.argc()<=1)
    {
        arguments.getApplicationUsage()->write(std::cout,osg::ApplicationUsage::COMMAND_LINE_OPTION);
        return 1;
    }
*/

    osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
    traits->x = 0;
    traits->y = 0;
    traits->width = 1;
    traits->height = 1;
    traits->red = 8;
    traits->green = 8;
    traits->blue = 8;
    traits->alpha = 8;
    traits->windowDecoration = false;
    traits->pbuffer = true;
    traits->doubleBuffer = true;
    traits->sharedContext = 0;
    osg::ref_ptr<osg::GraphicsContext> pbuffer;

    pbuffer = osg::GraphicsContext::createGraphicsContext(traits.get());
    if (pbuffer.valid())
    {
        osg::notify(osg::NOTICE)<<"Pixel buffer has been created successfully."<<std::endl;
    }
    else
    {
        osg::notify(osg::NOTICE)<<"Pixel buffer has not been created successfully."<<std::endl;
    }






    osg::BoundingSphere bs;
    osg::BoundingBox bb ;
    std::string _tileBasename="/home/mattjr/data/d100/real_root_L0_X0_Y0/real";
    int currentLevel=2;
    int   xdel[] = {-1, 0, 1, 0,  0,1,-1,1,-1};
    int   ydel[] = { 0, 0, 0,-1,  1,1,-1,-1,1};
    //    for(int modelX=0; modelX<4; modelX++){
    //      for(int modelY=0; modelY<4; modelY++){
    for(int i=0; i < 15; i++){
        char tmp[1024];
        sprintf(tmp,"%08d",i);
        //os2<< "/home/mattjr/data/dall_6/mesh-diced/clipped-diced-"<<tmp<<"-lod3.mat";

        std::ostringstream os;

        //  os << _tileBasename << "_L"<<currentLevel<<"_X"<<modelX<<"_Y"<<modelY<<"_subtile.ive";
        os<< "/home/mattjr/data/dall_6/mesh-diced/clipped-diced-"<<tmp<<"-lod3.ive";

        if(osgDB::fileExists(os.str())){
            osg::ref_ptr<osg::Node> node= osgDB::readNodeFile(os.str());
            osg::ref_ptr<osg::Image> image= osgDB::readImageFile(osgDB::getNameLessExtension(os.str()).append(".tif"));
            osg::Matrix viewProj;
            if(node.valid() && image.valid()){
                osg::Vec2 img_size(image->s(),image->t());
                bool res=readMatrixToScreen(osgDB::getNameLessExtension(os.str()).append(".mat"),viewProj,img_size);
                if(res){
                    osg::Group *gp=remapNode(node->asGroup(),image,viewProj,img_size);
                    if(gp){
                        std::ostringstream os_new;
                        //os_new << _tileBasename << "_L"<<currentLevel<<"_X"<<modelX<<"_Y"<<modelY<<"_subtile-blended.ive";
                        os_new<< "/home/mattjr/data/dall_6/mesh-diced/clipped-diced-"<<tmp<<"-blended-lod3.ive";
                        osgDB::writeNodeFile(*gp,os_new.str());
                    }
                }
            }
        }
    }



    //}
}
