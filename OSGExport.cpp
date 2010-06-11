#include "OSGExport.h"
#include <osgUtil/TriStripVisitor>
#include <osgUtil/SmoothingVisitor>
#include <osgUtil/Optimizer>
#include <osg/GraphicsContext>
#include <osg/TexEnvCombine>
#include <osg/PolygonMode>
#include <osg/TextureRectangle>
#include <osgDB/WriteFile>
#include <osg/Point>
#include <osgUtil/Simplifier>
#include <osg/PolygonOffset>
#include <osg/ShapeDrawable>
#include <cv.h>
#include <glib.h>
#include <osg/io_utils>
#include <highgui.h>
#include <boost/thread/thread.hpp>
#include "novelty.h"
#include <sys/stat.h>
#include "auv_texture_utils.hpp"
#include "auv_lod.hpp"
#include "auv_gts_utils.hpp"
#include "auv_tex_projection.hpp"
#include "adt_image_norm.hpp"
//#include "colormaps.h"
using namespace libpolyp;
typedef struct _GHashNode      GHashNode;
using namespace libsnapper;
using namespace squish;



MyGraphicsContext *mgc=NULL;
std::vector<GtsBBox *> bboxes_all;;
std::vector<GeometryCollection *> gc_ptrs;
boost::mutex bfMutex;

class UnrefTextureAtlas : public osg::NodeVisitor
{
public:

    UnrefTextureAtlas():
        osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN)
        {}

    virtual void apply(osg::Node& node)
    {
        if (node.getStateSet()) apply(*node.getStateSet());
        traverse(node);
    }
    
    virtual void apply(osg::Geode& node)
    {
        if (node.getStateSet()) apply(*node.getStateSet());
        
        for(unsigned int i=0;i<node.getNumDrawables();++i)
        {
            osg::Drawable* drawable = node.getDrawable(i);
            if (drawable && drawable->getStateSet()) apply(*drawable->getStateSet());
        }
        
        traverse(node);
    }
    
    virtual void apply(osg::StateSet& stateset)
    {
        // search for the existence of any texture object attributes
        for(unsigned int i=0;i<stateset.getTextureAttributeList().size();++i)
        {
            osg::Texture* texture = dynamic_cast<osg::Texture*>(stateset.getTextureAttribute(i,osg::StateAttribute::TEXTURE));
            if (texture)
            {
                _textureSet.insert(texture);
            }
        }
    }
    
    void unref_ptrs()
    {
       

        for(TextureSet::iterator itr=_textureSet.begin();
            itr!=_textureSet.end();
            ++itr)
        {
            osg::Texture* texture = const_cast<osg::Texture*>(itr->get());
            
            osg::Texture2D* texture2D = dynamic_cast<osg::Texture2D*>(texture);
            osg::Texture3D* texture3D = dynamic_cast<osg::Texture3D*>(texture);
            
	    //    osg::ref_ptr<osg::Image> image = texture2D ? texture2D->getImage() : (texture3D ? texture3D->getImage() : 0);
	    // printf("texture %d image %d\n",texture->referenceCount(),image->referenceCount());
	    if(texture2D){
	      while(texture2D->referenceCount()>0){
		texture2D->unref();
	      }
	    }else if(texture3D){
	      while(texture3D->referenceCount()>0){
		texture3D->unref();
	      }
	    }

	    //printf("texture %d image %d\n",texture->referenceCount(),image->referenceCount());
	    }
	}

    
    typedef std::set< osg::ref_ptr<osg::Texture> > TextureSet;
    TextureSet                          _textureSet;
 
    
};

class SetDynamicStateSetVisitor : public osg::NodeVisitor
{
public:

    SetDynamicStateSetVisitor():
        osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN),
        _numStateSetChanged(0)
    {
      osg::notify(osg::INFO)<< "Running DynamicStateSet..."<<std::endl;
    }
        
    ~SetDynamicStateSetVisitor()
    {
       osg::notify(osg::INFO)<<"  Number of StateState removed "<<_numStateSetChanged<<std::endl;
    }

    virtual void apply(osg::Node& node)
    {
        if (node.getStateSet())
        {
	  node.getStateSet()->setDataVariance(osg::Object::DYNAMIC);
            ++_numStateSetChanged;
        }
        traverse(node);
    }
    
    virtual void apply(osg::Geode& node)
    {
        if (node.getStateSet())
        {
          node.getStateSet()->setDataVariance(osg::Object::DYNAMIC);
	  ++_numStateSetChanged;
        }
        
        for(unsigned int i=0;i<node.getNumDrawables();++i)
        {
            osg::Drawable* drawable = node.getDrawable(i);
            if (drawable && drawable->getStateSet())
            {
	      drawable->getStateSet()->setDataVariance(osg::Object::DYNAMIC);
	      ++_numStateSetChanged;
	    
            }
        }
        
        traverse(node);
    }
    
    unsigned int _numStateSetChanged;
};

void OSGExporter::compress(osg::Texture2D* texture2D, osg::Texture::InternalFormatMode internalFormatMode){
 
  if(!state)
    state = new osg::State;
  
  
  osg::ref_ptr<osg::Image> image = texture2D->getImage();
  if (image.valid() && 
      (image->getPixelFormat()==GL_RGB || image->getPixelFormat()==GL_RGBA) ){//&&
      //     (image->s()>=32 && image->t()>=32)){
    //internalFormatMode=osg::Texture::USE_S3TC_DXT1_COMPRESSION;
    texture2D->setInternalFormatMode(internalFormatMode);
    
    // need to disable the unref after apply, other the image could go out of scope.
    bool unrefImageDataAfterApply = texture2D->getUnRefImageDataAfterApply();
    texture2D->setUnRefImageDataAfterApply(false);
   
    // get OpenGL driver to create texture from image.
    texture2D->apply(*state);
    
    // restore the original setting
    texture2D->setUnRefImageDataAfterApply(unrefImageDataAfterApply);

    image->readImageFromCurrentTexture(state->getContextID(),true);
 
    texture2D->setInternalFormatMode(osg::Texture::USE_IMAGE_DATA_FORMAT);
  }

}

 void bin_face_mat_osg (T_Face * f, gpointer * data){
  MaterialToGeometryCollectionMap *mtgcm=(MaterialToGeometryCollectionMap *)data[0];
  //uint uiFace =  *((guint *) data[1]);
  if(mtgcm->count(f->material) <= 0){
    GeometryCollection *gc_tmp = new GeometryCollection;
    gc_ptrs.push_back(gc_tmp);
    (*mtgcm)[f->material]= gc_tmp;
  }
  GeometryCollection& gc = *(*mtgcm)[f->material];
  gc._numPoints += 3;
  gc._numPrimitives += 1;
  if (f->material >= 0) {
    gc._numPrimitivesWithTexCoords += 1;
    
  }
  GUINT_TO_POINTER ((*((guint *) data[1]))++);
}


static void add_face_mat_osg (T_Face * f, gpointer * data){
 
  MaterialToGeometryCollectionMap *mtgcm=(MaterialToGeometryCollectionMap *)data[0];
  float *zrange = (float *)data[4];
  map<int,string> *textures = (map<int,string> *)data[3];
  ClippingMap *cm=(ClippingMap *)data[2];
  GeometryCollection& gc = *(*mtgcm)[f->material];
  osg::BoundingBox &texLimits=(*cm)[osgDB::getSimpleFileName((*textures)[f->material])];
  //int planeTexSize=(long int)data[7];
  
  map<int,int> *texnum2arraynum=(map<int,int> *)data[9];
  vector<Plane3D> *planes=(vector<Plane3D> *)data[10];
  osg::PrimitiveSet::Mode mode;
  
  mode = osg::PrimitiveSet::TRIANGLES;
  
  gc._geom->addPrimitiveSet(new osg::DrawArrays(mode,gc._coordCount,3));
  gc._coordCount += 3;
  TVertex * v1,* v2,* v3;
  gts_triangle_vertices(&GTS_FACE(f)->triangle,(GtsVertex **)& v1, 
			(GtsVertex **)&v2, (GtsVertex **)&v3);

  /* (*gc._vertices++).set(GTS_VERTEX(v1)->p.y,GTS_VERTEX(v1)->p.x,-GTS_VERTEX(v1)->p.z);
  (*gc._vertices++).set(GTS_VERTEX(v2)->p.y,GTS_VERTEX(v2)->p.x,-GTS_VERTEX(v2)->p.z);
  (*gc._vertices++).set(GTS_VERTEX(v3)->p.y,GTS_VERTEX(v3)->p.x,-GTS_VERTEX(v3)->p.z);*/
 (*gc._vertices++).set(GTS_VERTEX(v1)->p.x,GTS_VERTEX(v1)->p.y,GTS_VERTEX(v1)->p.z);
  (*gc._vertices++).set(GTS_VERTEX(v2)->p.x,GTS_VERTEX(v2)->p.y,GTS_VERTEX(v2)->p.z);
  (*gc._vertices++).set(GTS_VERTEX(v3)->p.x,GTS_VERTEX(v3)->p.y,GTS_VERTEX(v3)->p.z);

  if(gc._planeTexValid && planes){
    if(v1->plane >= 0)
      (*gc._texcoordsPlane++).set((*planes)[v1->plane].u[0], (*planes)[v1->plane].u[1],(*planes)[v1->plane].u[2],(*planes)[v1->plane].d0);
    else
      (*gc._texcoordsPlane++).set(-1.0,-1.0,-1.0,-1.0);    
    
    if(v2->plane >= 0)
   (*gc._texcoordsPlane++).set((*planes)[v2->plane].u[0], (*planes)[v2->plane].u[1],(*planes)[v2->plane].u[2],(*planes)[v2->plane].d0); 
 else
   (*gc._texcoordsPlane++).set(-1.0,-1.0,-1.0,-1.0);   
 
 if(v3->plane >= 0)
   (*gc._texcoordsPlane++).set((*planes)[v3->plane].u[0], (*planes)[v3->plane].u[1],(*planes)[v3->plane].u[2],(*planes)[v3->plane].d0); 
    else
      (*gc._texcoordsPlane++).set(-1.0,-1.0,-1.0,-1.0);   
    
  
  
}
 
  if (gc._texturesActive){// && f->material >= 0){


    texLimits.expandBy(v1->u,1-v1->v,0.0);
    texLimits.expandBy(v2->u,1-v2->v,0.0);
    texLimits.expandBy(v3->u,1-v3->v,0.0);

    if(f->material >= 0){
      (*gc._texcoords++).set(v1->u,1-v1->v);    
      (*gc._texcoords++).set(v2->u,1-v2->v); 
      (*gc._texcoords++).set(v3->u,1-v3->v); 
    }
    if(texnum2arraynum != NULL){
      
      for(int i=0; i< 4; i++){
	if(f->materialB[i] == -1){
	  (*gc._texcoordsTexArray[i]++).set(-1,-1,-1); 
	  (*gc._texcoordsTexArray[i]++).set(-1,-1,-1); 
	  (*gc._texcoordsTexArray[i]++).set(-1,-1,-1); 
	  
	}else{
	  (*gc._texcoordsTexArray[i]++).set(v1->uB[i], 1 - v1->vB[i],(*texnum2arraynum)[f->materialB[i]]);
	  (*gc._texcoordsTexArray[i]++).set(v2->uB[i], 1 - v2->vB[i],(*texnum2arraynum)[f->materialB[i]]); 
	  (*gc._texcoordsTexArray[i]++).set(v3->uB[i], 1 - v3->vB[i],(*texnum2arraynum)[f->materialB[i]]); 
	}
      }
    }

  }else
    gc._texturesActive=false;

  if(gc._colorsActive){
    //  if(!shader_colori){
    {
      if(!zrange){
	(*gc._colors++).set(0.5,0.5,0.5,0.0);
	(*gc._colors++).set(0.5,0.5,0.5,0.0);
	(*gc._colors++).set(0.5,0.5,0.5,0.0);
      }
      float range=zrange[1]-zrange[0];
    
      Lut_Vec color;
      float val;// r,g,b,val;
      Colors::eColorMap map=Colors::eRainbowMap;
      val = clamp ( ( GTS_VERTEX(v1)->p.z -zrange[0] )/range , 0.0f, 1.0f);
      
      color=GlobalColors()->Get(map, val); 
      (*gc._colors++).set(color[0],color[1],color[2],1.0);
      //  jet_color_map(val,r,g,b);
      //(*gc._colors++).set(r,b,g,1.0);
      
      val = clamp (  ( GTS_VERTEX(v2)->p.z -zrange[0] )/range, 0.0f, 1.0f);;    
      color=GlobalColors()->Get(map, val); 
      (*gc._colors++).set(color[0],color[1],color[2],1.0);
      //jet_color_map(val,r,g,b);
      // (*gc._colors++).set(r,b,g,1.0);
      
      val = clamp ( ( GTS_VERTEX(v3)->p.z -zrange[0] )/range, 0.0f, 1.0f);   
      color=GlobalColors()->Get(map, val); 
      (*gc._colors++).set(color[0],color[1],color[2],1.0);

      //jet_color_map(val,r,g,b);
      //  (*gc._colors++).set(r,b,g,1.0);
    }/*else{
      //(*gc._colors++).set(v1->r/255.0,v1->g/255.0,v1->b/255.0,1.0);
    
     // (*gc._colors++).set(v2->r/255.0,v2->g/255.0,v2->b/255.0,1.0);
     // (*gc._colors++).set(v3->r/255.0,v3->g/255.0,v3->b/255.0,1.0);
      (*gc._colors++).set(v1->r,v1->g,v1->b,1.0);
    
      (*gc._colors++).set(v2->r,v2->g,v2->b,1.0);
      (*gc._colors++).set(v3->r,v3->g,v3->b,1.0);

    }*/
    
  }
}


osg::ref_ptr<osg::Image>OSGExporter::cacheCompressedImage(IplImage *img,string name,int tex_size){
  string ddsname=osgDB::getNameLessExtension(name);
  ddsname +=".dds";

  IplImage *tex_img=cvCreateImage(cvSize(tex_size,tex_size),
				  IPL_DEPTH_8U,3);

  if(img && tex_img)
    cvResize(img,tex_img);
  else
    printf("Invalid Images\n");

  osg::ref_ptr<osg::Image> image= Convert_OpenCV_TO_OSG_IMAGE(tex_img);
  osg::ref_ptr<osg::Texture2D> texture = new osg::Texture2D;
  texture->setImage(image.get());
  compress(texture.get());
  osgDB::writeImageFile(*image.get(),ddsname);
  cvReleaseImage(&tex_img);
  return image;
}

osg::ref_ptr<osg::Image>OSGExporter::cacheImage(IplImage *img,string name,int tex_size,bool ret){
 

  IplImage *tex_img=cvCreateImage(cvSize(tex_size,tex_size),
				  IPL_DEPTH_8U,3);

  if(img && tex_img)
    cvResize(img,tex_img);
  else
    printf("Invalid Images\n");

  if(ret){
    osg::ref_ptr<osg::Image> image= Convert_OpenCV_TO_OSG_IMAGE(tex_img);
    osg::ref_ptr<osg::Texture2D> texture = new osg::Texture2D;
    texture->setImage(image.get());
    osgDB::writeImageFile(*image.get(),name);
    cvReleaseImage(&tex_img);
    return image;
  }
  else {
    cvSaveImage(name.c_str(),tex_img);
    cvReleaseImage(&tex_img);

  }
    return NULL;
}

osg::Image *OSGExporter::decompressImage(osg::Image * img_ptr){

  osg::Image *image_full= new osg::Image();
  unsigned char *newdata = new unsigned char[img_ptr->s()*img_ptr->t()*3*sizeof(unsigned char)];
  decompressed_ptrs.push_back(image_full);

  DecompressImageRGB(newdata,img_ptr->s(),img_ptr->t(),img_ptr->data(),squish::kDxt1);
 
  image_full->setImage(img_ptr->s(),img_ptr->t(),img_ptr->r(),osg::Texture::USE_IMAGE_DATA_FORMAT,GL_RGB,GL_UNSIGNED_BYTE,newdata,osg::Image::USE_NEW_DELETE);

  return image_full;
}

osg::Image *OSGExporter::getCachedCompressedImage(string name,int size){
  IplImage *img=NULL;
  string basename=osgDB::getSimpleFileName(name);
  string ddsname=osgDB::getNameLessExtension(name);
  ddsname +=".dds";

  bool cachedLoaded=false;
  osg::Image *filecached;

  if(compressed_img_cache.find(basename) == compressed_img_cache.end() || !compressed_img_cache[basename] ){
    
    if(FileExists(ddsname)){
      filecached=osgDB::readImageFile(ddsname);
      if(filecached){
	cachedLoaded=true;
	filecached->ref();

      }
    }
    
    if(!cachedLoaded){
      img = cvLoadImage(name.c_str(),-1);
      if(!img){
	printf("In Cached Compressed Img Load failed %s.\n",name.c_str());
	return NULL;
      }
      else{
	osg::ref_ptr<osg::Image> comp_img=cacheCompressedImage(img,ddsname,size).get();
	filecached=comp_img.get();
	comp_img->ref();
      }
    }
    filecached->setFileName(basename);
    compressed_img_cache[basename]=filecached;

  }else{  
    filecached=compressed_img_cache[basename];
  }

  if(filecached && filecached->s() == size && filecached->t() == size){

    if(!compress_tex || do_atlas){
      osg::Image *ret=decompressImage(filecached);
      return ret;
    }
    return filecached;
  
  }


  int resize=filecached->s();
  unsigned int i=0;
  for(i=0; i < filecached->getNumMipmapLevels()-1; i++){
    if(resize <= size)
      break;
    resize >>= 1;
  }

  if(resize == 0)
    resize=1;
  
  osg::Image *retImage=new osg::Image;
  downsampled_img_ptrs.push_back(retImage);
  int datasize=filecached->getTotalSizeInBytesIncludingMipmaps()-filecached->getMipmapOffset(i);
  unsigned char *newdata = new unsigned char[datasize];
  resize_data_ptrs.push_back(newdata);
  memcpy(newdata,filecached->getMipmapData(i),datasize);
  retImage->setImage(resize,resize,filecached->r(),filecached->getInternalTextureFormat() ,filecached->getPixelFormat(),filecached->getDataType(),newdata,osg::Image::USE_NEW_DELETE);
  osg::Image::MipmapDataType mipmaps;
  mipmaps=filecached->getMipmapLevels();
 
  int newsize= mipmaps[i]-mipmaps[i-1];
  int base=mipmaps[i];
 
  std::vector<unsigned int>newmipmap;



  for(int k=i; k < (int)mipmaps.size(); k++){
    newmipmap.push_back(newsize+(mipmaps[k]-base));
  }
 
  retImage->setMipmapLevels(newmipmap);


  retImage->setFileName(basename);
  
  if(!compress_tex || do_atlas){
    osg::Image *ret=decompressImage(retImage);
  
    return ret;
  }
  else
    return retImage;
}


osg::Image *OSGExporter::getCachedImage(string name,int size){
  IplImage *img=NULL;
  string basename=osgDB::getSimpleFileName(name);
 


  bool cachedLoaded=false;
  IplImage *filecached=NULL;

  if(tex_image_cache.find(basename) == tex_image_cache.end() || !tex_image_cache[basename] ){
    
    if(FileExists(name)){
      filecached=cvLoadImage(name.c_str(),1);

      if(filecached){
	cachedLoaded=true;

      }
    }
    
    if(!cachedLoaded){
      img = cvLoadImage((string("img/")+basename).c_str(),1);
      if(!img){
	printf("In Cached Compressed Img Load failed %s.\n",name.c_str());
	return NULL;
      }
      else{
	osg::ref_ptr<osg::Image> comp_img=cacheImage(img,name,size).get();
	filecached=img;
      }
    }
 
    tex_image_cache[basename]=filecached;

  }else{  
    filecached=tex_image_cache[basename];
  }

 

  IplImage *tex_img=NULL;
  IplImage *small_img=NULL;
 if(filecached->width == size && filecached->height == size){
   tex_img=filecached;
   osg::Image *retImage =Convert_OpenCV_TO_OSG_IMAGE(tex_img,true,false);
   return retImage;
 }else{
   small_img  =cvCreateImage(cvSize(size,size),
			   IPL_DEPTH_8U,3);
   
   
   
   if(filecached && small_img)
     cvResize(filecached,small_img);
   else
     printf("Invalid Images\n");
 
   osg::Image *retImage =Convert_OpenCV_TO_OSG_IMAGE(small_img,false,false);
   cvReleaseImageHeader(&small_img);

   return retImage;
 }
}
#define TEXUNIT_ARRAY       0
#define TEXUNIT_HIST        2
#define TEXUNIT_INFO        3
#define TEXUNIT_PLANES       1




void OSGExporter::calcHists( MaterialToGeometryCollectionMap &mtgcm, map<int,string> textures, Hist_Calc &histCalc){

  MaterialToGeometryCollectionMap::iterator itr;
   for(itr=mtgcm.begin(); itr!=mtgcm.end(); ++itr){
     GeometryCollection& gc = *(itr->second);
     if(!gc._texturesActive )
       continue;
     string name=textures[itr->first];
        IplImage  *scaled=NULL;
     scaled=tex_image_cache[osgDB::getSimpleFileName(name)];
    
    
     if(!scaled){
       printf("Image null in getTextureHists %s\n",name.c_str());
       continue;
     }
     //     IplImage *tmp=cvCreateImage(cvSize(img->s(),img->t()),
     //			 IPL_DEPTH_8U,3);
    
     IplImage *mask=cvNewGray(scaled);
     cvZero(mask);

     //DecompressImageRGB((unsigned char *)tmp->imageData,tmp->width,tmp->height,img->data(),squish::kDxt1);

     int count = 3;
     CvPoint pt[3];
     CvPoint* tri=pt;
     for(int i=0; i< (int) gc._texcoordArray->size(); i+=3){
       for(int j=0; j<3; j++){
	 
	 osg::Vec2f tc = (*gc._texcoordArray)[i+j];
	 tc *= scaled->width;
	 pt[j]=cvPoint((int)tc[1],(int)tc[0]);

       }
       
       cvFillPoly(mask, &tri, &count, 1, CV_RGB(255,255,255));
     }
     histCalc.calc_hist(scaled,mask,1/*BGR*/);
     if(gpuNovelty){
       osg::Texture *tex=osg_tex_map[itr->first];
       if(!tex){
	 printf("Null Texture in calcHist %s\n",textures[itr->first].c_str());
	 continue;
       }
	 
       IplImage *rgba=cvCreateImage(cvSize(scaled->width,scaled->height),
				    IPL_DEPTH_8U,4);
     
       cvCvtColor(scaled, rgba, CV_RGB2RGBA);
       cvSetImageCOI(rgba,4);
       cvCopy(histCalc.imageTex,rgba);
       cvSetImageCOI(rgba,0);
       // cvNamedWindow("ASDAS",-1);
       // cvShowImage("ASDAS",rgba);
       // cvWaitKey(0);
       osg::Image *imageWithG=Convert_OpenCV_TO_OSG_IMAGE(rgba,false);
       //osgDB::Registry::instance()->writeImage( *imageWithG,"ass.png",NULL);
       // exit(0);
       osg::Texture2D* texture2D = dynamic_cast<osg::Texture2D*>(tex);
       texture2D->setImage(imageWithG);
       //   compress(texture2D,osg::Texture::USE_S3TC_DXT5_COMPRESSION);
       }
     // cvReleaseImage(&scaled);
     cvReleaseImage(&mask);
   }
   
  
}

void OSGExporter::addNoveltyTextures( MaterialToGeometryCollectionMap &mtgcm, map<int,string> textures, Hist_Calc &histCalc,CvHistogram *hist){
  
  MaterialToGeometryCollectionMap::iterator itr;
   for(itr=mtgcm.begin(); itr!=mtgcm.end(); ++itr){
     GeometryCollection& gc = *(itr->second);
     if(!gc._texturesActive )
       continue;
     IplImage *novelty_channel=NULL;
     string name=textures[itr->first];
     if(novelty_image_cache.count(osgDB::getSimpleFileName(name)) > 0 && novelty_image_cache[osgDB::getSimpleFileName(name)]){
       novelty_channel=novelty_image_cache[osgDB::getSimpleFileName(name)];
     }
     osg::Texture *tex=osg_tex_map[itr->first];
     IplImage  *tmp=NULL;
     tmp=tex_image_cache[osgDB::getSimpleFileName(name)];
     if(!tmp || !tex){
       printf("Image null in addNovletyTextures %s\n",osgDB::getSimpleFileName(name).c_str());
       continue;
     }
     IplImage *scaled=cvCreateImage(cvSize(histCalc.width,histCalc.height),
				    IPL_DEPTH_8U,3);
     cvResize(tmp,scaled);
     if(!novelty_channel){
       if(!hist)
 	 continue;

       IplImage *med=cvNewGray(scaled);
       
       //     DecompressImageRGB((unsigned char *)tmp->imageData,tmp->width,tmp->height,img->data(),squish::kDxt1);
       
       
       IplImage *novelty_image=  histCalc.get_novelty_img(scaled,hist); 
       mcvNormalize((IplImage*)novelty_image, (IplImage*)novelty_image, 0, 255);
       //       IplImage *texInfo=cvNewColor(novelty_image);
       IplImage *tmpG=cvNewGray(novelty_image);
       
       //cvZero(texInfo);
       cvConvertScale(novelty_image,tmpG);
       
       cvSmooth(tmpG,med,CV_MEDIAN,9);
       novelty_channel=med;
       novelty_image_cache[osgDB::getSimpleFileName(name)]=novelty_channel;
       cvReleaseImage(&tmpG);
       cvReleaseImage(&novelty_image);
     }
     
     
     IplImage *rgba=cvCreateImage(cvSize(histCalc.width,histCalc.height),
				  IPL_DEPTH_8U,4);
  
     IplImage *scaled_novelty=cvCreateImage(cvSize(histCalc.width,histCalc.height),IPL_DEPTH_8U,1);

     cvResize(novelty_channel,scaled_novelty);

     cvCvtColor(scaled, rgba, CV_RGB2RGBA);
     cvSetImageCOI(rgba,4);
     cvCopy(scaled_novelty,rgba);
     cvSetImageCOI(rgba,0);
     /*      cvNamedWindow("ASDAS",-1);
       cvShowImage("ASDAS",scaled_novelty);
       cvWaitKey(5);*/
     osg::Image *imageWithG=Convert_OpenCV_TO_OSG_IMAGE(rgba,false);
     //osgDB::Registry::instance()->writeImage( *imageWithG,"ass.png",NULL);
     // exit(0);
     osg::Texture2D* texture2D = dynamic_cast<osg::Texture2D*>(tex);
     texture2D->setImage(imageWithG);
     //   compress(texture2D,osg::Texture::USE_S3TC_DXT5_COMPRESSION);
     /* }else{
	    cvSetImageCOI(texInfo,1);
	    cvCopy(med,texInfo);
	    cvSetImageCOI(texInfo,0);
	    cvNamedWindow("ASDAS",-1);
	    cvShowImage("ASDAS",texInfo);
	    cvWaitKey(0);
	    osg::Image *texI=Convert_OpenCV_TO_OSG_IMAGE(texInfo,false);
	    osg::Texture2D* texture2D = new osg::Texture2D(texI);
	    compress(texture2D,osg::Texture::USE_S3TC_DXT1_COMPRESSION);
	    gc._geom->getStateSet()->setTextureAttribute(TEXUNIT_INFO,texture2D );
	 
	    cvReleaseImage(&texInfo);
	    }*/
   }
   
   //    cvReleaseImage(&tmp);
}

   
   
  



bool OSGExporter::convertGtsSurfListToGeometry(GtsSurface *s, map<int,string> textures,ClippingMap *cm,int proj_tex_size,int tex_size,osg::ref_ptr<osg::Geode>*group,vector<Plane3D> planes,vector<TriMesh::BBox> bounds,osg::Matrix *rot,VerboseMeshFunc vmcallback,float *zrange,std::map<int,osg::Matrixd> *camMatrices,std::map<string,int> *classes,int num_class_id,osg::Group *toggle_ptr)
{

  gpuNovelty=false;
   _tex_size=tex_size;
   map<int,int> texnum2arraynum;
  gpointer data[11];
  gint n=0;
  data[0]=&mtgcm;
  data[1] = &n;
  data[2]=cm;
  data[3]=&textures;
  data[4]=zrange;
  data[6]=zrange;
  data[7]=(void *)_planeTexSize;
  
  if(_tex_array_blend)
    data[9]=&texnum2arraynum;
  else
    data[9]=NULL;

  if(usePlaneDist)
    data[10]=&planes;
  else
    data[10]=NULL;

  if(classes && num_class_id != 0)
    useClasses=true;
  else
    useClasses=false;


  if(useClasses)
    shader_coloring=true;

  
  //data[5]=hists;
  //tempFF=getPlaneTex(planes);
 
  int numimgpertex;
  if(_tex_array_blend)
    numimgpertex=150;
  else
    numimgpertex=1;
  //int overlap=max(1,(int)(numimgpertex * 0.1));
  
  gts_surface_foreach_face (s, (GtsFunc) bin_face_mat_osg , data);
  MaterialToGeometryCollectionMap::iterator itr;

  vector<pair<GeometryCollection *,vector<int> > > gcAndTexIds;

  int count=0;
  bool first=true;
  GeometryCollection *curGC=NULL;
  
  for(itr=mtgcm.begin(); itr!=mtgcm.end(); ++itr){
    GeometryCollection& gc = *(itr->second);
    if(count % numimgpertex == 0 || first){
      vector<int> ids;
      if(itr->first >= 0)
	ids.push_back(itr->first);
      gcAndTexIds.push_back(make_pair(itr->second,ids));
      curGC=itr->second;
    }
    else{
      gcAndTexIds.back().first->_numPoints += gc._numPoints;
      gcAndTexIds.back().first->_numPrimitives += gc._numPrimitives;
      gcAndTexIds.back().first->_numPrimitivesWithTexCoords += gc._numPrimitivesWithTexCoords;
      if(itr->first >= 0)
	gcAndTexIds.back().second.push_back(itr->first);
      itr->second=curGC;
    }
    count++;
    first=false;
  }
  


  int tex_count=0;

  for(int gci=0; gci< (int)gcAndTexIds.size(); gci++){
    GeometryCollection& gc = (*gcAndTexIds[gci].first);
    if (gc._numPrimitives){
      
       
      gc._geom = new osg::Geometry;
       
      osg::Vec3Array* vertArray = new osg::Vec3Array(gc._numPoints);
      gc._vertices = vertArray->begin();
      gc._geom->setVertexArray(vertArray);
      gc._planeTexValid=false;
      // set up color.
      if(!shader_coloring){
	osg::Vec4Array* colorsArray = new osg::Vec4Array(gc._numPoints);
	 
	 
	gc._colors=colorsArray->begin();                 
	gc._colorsActive=true;
	gc._geom->setColorArray(colorsArray);
	 
	gc._geom->setColorBinding(osg::Geometry::BIND_PER_VERTEX);

	

      }else
	gc._colorsActive=false;
      
      if(!_tex_array_blend && gcAndTexIds[gci].second.size() > 1){
	fprintf(stderr,"Error blending off yet more then one texture per geometry\n");
      }
      osg::ref_ptr<osg::Texture2DArray> textureArray;
      if(_tex_array_blend){
	textureArray =new osg::Texture2DArray;
	osg::Program* program=NULL;
	program = new osg::Program;
	program->setName( "microshader" );
	osg::Shader *lerp=new osg::Shader( osg::Shader::FRAGMENT);
	loadShaderSource( lerp, basedir+"lerp.frag" );
	program->addShader(  lerp );
	textureArray->setTextureSize(tex_size,tex_size,gcAndTexIds[gci].second.size());
	osg::StateSet* stateset = new osg::StateSet;
	stateset->setAttributeAndModes( program, osg::StateAttribute::ON );
	
	
	stateset->addUniform( new osg::Uniform("theTexture", TEXUNIT_ARRAY) );
	stateset->addUniform( new osg::Uniform( "weights", osg::Vec3(0.640f, 0.370f, 0.770f) ));
	stateset->addUniform( new osg::Uniform( "shaderOut", 0));
	
	stateset->setTextureAttribute(TEXUNIT_ARRAY, textureArray.get());
	stateset->setDataVariance(osg::Object::STATIC);
	osg::Vec2Array* texcoordArray = new osg::Vec2Array(gc._numPoints);
	gc._texcoords = texcoordArray->begin();
	gc._geom->setTexCoordArray(0,texcoordArray);
	gc._texturesActive=true;
	for(int i=0; i< NUM_TEX_BLEND_COORDS; i++){
	  osg::Vec3Array* texcoordBlendArray =new osg::Vec3Array(gc._numPoints);
	  gc._texcoordsTexArray[i] = texcoordBlendArray->begin();
	  gc._geom->setTexCoordArray(i+1,texcoordBlendArray);
	}
	gc._geom->setStateSet(stateset);
	//printf("New GC size %d i = %d \n",gcAndTexIds[gci].second.size(),gci);
	for(int g=0; g< (int)gcAndTexIds[gci].second.size(); g++)
	  //  printf("Dude %d\n",gcAndTexIds[gci].second[g]); 
	  ;
      }

      int imgNum=0;
      // set up texture if needed.
      for(int j=0; j < (int)gcAndTexIds[gci].second.size(); j++){
	int tidx=gcAndTexIds[gci].second[j];
	if(tidx < 0 ||  textures.count(tidx) <= 0){
	  printf("Really should never get here !!!!!\n");
	}

	std::string filename=prefixdir+textures[tidx];
	osg::notify(osg::INFO) << "ctex " << filename  << std::endl;
	char fname[255];
	sprintf(fname,"mesh/%s",textures[tidx].c_str());
	++tex_count;
	if(vmcallback)
	  vmcallback(tex_count,mtgcm.size()-1);
	if(verbose)
	  printf("\rLoading Texture: %03d/%03d",tex_count,(int)mtgcm.size()-1);
	if(!ive_out)
	  if(verbose)printf("\n");	 
	fflush(stdout); 
	
	osg::ref_ptr<osg::Image> image;
	//	if(do_atlas)
	 image=getCachedImage(filename,tex_size);
	//	else
	// image=getCachedCompressedImage(filename,tex_size);
	int baseTexUnit=0;
	
	  //LoadResizeSave(filename,fname, (!ive_out),tex_size);
	if (image.valid()){	   


	  if(_tex_array_blend){

	    texnum2arraynum[tidx]=j;
	    textureArray->setImage(imgNum,image.get());
	    textureArray->setUnRefImageDataAfterApply(false);
	    osg_tex_arr_ptrs.push_back(textureArray);
	    imgNum++;
	  }else{
  
	    // create state
	    osg::StateSet* stateset = new osg::StateSet;
	    // create texture
	    osg::Texture2D* texture = new osg::Texture2D;
	    /*   if(do_atlas || compress_tex) 
	      texture->setUnRefImageDataAfterApply( true );
	    else
	      texture->setUnRefImageDataAfterApply( false );
	    */
	    texture->setImage(image.get());
            texture->setWrap(osg::Texture::WRAP_S,osg::Texture::CLAMP_TO_BORDER);
            texture->setWrap(osg::Texture::WRAP_T,osg::Texture::CLAMP_TO_BORDER);
           texture->setBorderColor(osg::Vec4(0.0,1.0,0.0,1.0));
            //texture->setBorderWidth(1.0);
	    //texture->setInternalFormatMode(internalFormatMode);
	    stateset->setTextureAttributeAndModes(baseTexUnit,texture,
						  osg::StateAttribute::ON);

	   
	    if(shader_coloring){
	   
	      
	      if(useClasses && classes){
		int class_id=-1;
		if(classes->count(osgDB::getNameLessExtension(textures[tidx])))
		  class_id=(*classes)[osgDB::getNameLessExtension(textures[tidx])];
		stateset->addUniform( new osg::Uniform( "classid", class_id/(float)(num_class_id)));
	      }
	      
	      stateset->setDataVariance(osg::Object::STATIC);
	    }
	    if(use_proj_tex){
                      //printf("Using proj texture\n");
	   

	osg::Program* program=NULL;
	program = new osg::Program;
	program->setName( "colorshaderasdas" );
        program->setParameter(GL_GEOMETRY_VERTICES_OUT_EXT,4);
        program->setParameter(GL_GEOMETRY_INPUT_TYPE_EXT,GL_POINTS);
        program->setParameter(GL_GEOMETRY_OUTPUT_TYPE_EXT,GL_POINTS);

	//  printf("Here\n");
	osg::Shader *hcolorf=new osg::Shader( osg::Shader::FRAGMENT);
	osg::Shader *hcolorv=new osg::Shader( osg::Shader::VERTEX);
	loadShaderSource( hcolorf, basedir+"proj.frag" );
	loadShaderSource( hcolorv, basedir+ "proj.vert" );
	program->addShader(  hcolorf );
        program->addShader(  hcolorv );
        stateset->setAttributeAndModes( program,
                                        osg::StateAttribute::ON );
        osg::Matrix mat=(*camMatrices)[tidx];
        osg::Matrix texScale(0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1);
        texScale(0,0)=1.0/(double)(proj_tex_size);
        texScale(1,1)=1.0/(double)(proj_tex_size);
       // cout << "tex Scale "<<texScale <<endl;
        osg::Vec3 v1(-133.381,74.1,15.0319);
         osg::Matrix mat2=mat;
      //  mat2.postMult(texScale);
        //std::cout <<"BUG " <<v1*mat << " \n
         //std::cout <<"RES " << v1*mat2<<endl;
        stateset->addUniform( new osg::Uniform( "teMat",mat2));
        // stateset->setTextureAttributeAndModes(baseTexUnit, texMat, osg::StateAttribute::ON);
        //    stateset->addUniform( new osg::Uniform( "fc", osg::Vec4(_calib->fcx,_calib->fcy,_calib->ccx,_calib->ccy)));


    }

	   
	    gc._texturesActive=true;
	    stateset->setDataVariance(osg::Object::STATIC);
	    
	    gc._geom->setStateSet(stateset);
	    
	    osg::Vec2Array* texcoordArray = new osg::Vec2Array(gc._numPoints);
	    gc._texcoords = texcoordArray->begin();
	    gc._texcoordArray = texcoordArray;
	    gc._geom->setTexCoordArray(0,texcoordArray);
	    if(computeHists){
	      osg::Vec4Array* texcoordPlanes = new osg::Vec4Array(gc._numPoints);
	      gc._texcoordsPlane = texcoordPlanes->begin();
	      gc._geom->setTexCoordArray(TEXUNIT_PLANES,texcoordPlanes);
	      gc._planeTexValid=true;
	    }
	    
	  
	  
	    osg_tex_ptrs.push_back(texture);
	    osg_tex_map[tidx]=texture;
	  }
	}
      }
    }
  }
  if(verbose)
    printf("\n");
 
  
    gts_surface_foreach_face (s, (GtsFunc) add_face_mat_osg , data);

    osg::ref_ptr<osg::Geode> untextured = new osg::Geode;
    osg::ref_ptr<osg::Geode> textured = new osg::Geode;
    osg::TextureRectangle* histTex=NULL;
    osg::ref_ptr<osg::Program> novelty_program= new osg::Program;

   
     
     
     novelty_program->setName( "microshader" );
      osg::Shader *novelty=new osg::Shader( osg::Shader::FRAGMENT);
      osg::Shader *distNovelty=new osg::Shader( osg::Shader::VERTEX);
      if(gpuNovelty){
	loadShaderSource( novelty, basedir+"novelty-live.frag" );
      }else{
	loadShaderSource( novelty, basedir+"novelty.frag" );
	//	loadShaderSource( novelty, "ass.frag" );
      }
      loadShaderSource( distNovelty, basedir+"novelty.vert" );
      //loadShaderSource( distNovelty, "ass.vert" );
      
      novelty_program->addShader(  novelty );
      novelty_program->addShader( distNovelty );
       //osg::TextureRectangle* planeTex=NULL;
      if(computeHists){
	Hist_Calc histCalc(_tex_size,_tex_size,16,1);

	if(!run_higher_res){
	  calcHists(mtgcm,textures,histCalc);
	  histCalc.get_hist(finalhist);
	  // cvSave("hist1.xml",finalhist);
	}
	if(gpuNovelty)
	  histTex= getTextureHists(finalhist);
	else
	  addNoveltyTextures(mtgcm,textures,histCalc,finalhist);
	
	// add everthing into the Geode.   
	//	planeTex=getPlaneTex(planes,_planeTexSize);
      }
      
      
  
      
      
      osgUtil::SmoothingVisitor smoother;
 
      for(int i=0; i< (int)gcAndTexIds.size(); i++)
    {
      GeometryCollection& gc = *(gcAndTexIds[i].first);
      if (gc._geom)
        {
           gc._geom->setUseDisplayList(true);
          // gc._geom->setUseVertexBufferObjects(true);
                //  gc._geom->setUseDisplayList(false);
          //  tessellator.retessellatePolygons(*gc._geom);
        
	  smoother.smooth(*gc._geom);
	   if(gc._texturesActive ){


	       
	   
	     
	       textured->addDrawable(gc._geom);
		   
	   
	   
	   }
	   else{
	  
	    osg::StateSet *utstateset = gc._geom->getOrCreateStateSet();
	    if(shader_coloring){
	      osg::Program* program=NULL;
	      program = new osg::Program;
	      program->setName( "colorshader" );
	      //  printf("Here\n");
	      osg::Shader *hcolorf=new osg::Shader( osg::Shader::FRAGMENT);
	      osg::Shader *hcolorv=new osg::Shader( osg::Shader::VERTEX);
	      loadShaderSource( hcolorf, basedir+"hcolor.frag" );
	      loadShaderSource( hcolorv, basedir+"hcolor.vert" );
	      program->addShader(  hcolorf );
	      program->addShader(  hcolorv );
	      utstateset->setAttributeAndModes( program, osg::StateAttribute::ON );

	      utstateset->addUniform( new osg::Uniform( "zrange", osg::Vec3(zrange[0], zrange[1], 0.0f) ));
	      //	      utstateset->addUniform( new osg::Uniform( "shaderOut", 0));
	      utstateset->addUniform( new osg::Uniform( "untex",true));
	      utstateset->setDataVariance(osg::Object::STATIC);
	      
	    }
	     
	
	    
	    {
	      osg::Material* material = new osg::Material;
	      
	      osg::Vec4 specular( 0.18, 0.18, 0.18, 0.18 );
	      specular *= 1.5;
	      float mat_shininess = 64.0f ;
	      osg::Vec4 ambient (0.92, 0.92, 0.92, 0.95 );
	      osg::Vec4 diffuse( 0.8, 0.8, 0.8, 0.85 );
	      
	    
	      material->setAmbient(osg::Material::FRONT_AND_BACK,ambient);
	      
	      material->setSpecular(osg::Material::FRONT_AND_BACK,specular);
	      material->setShininess(osg::Material::FRONT_AND_BACK,mat_shininess);
	      //     material->setColorMode(  osg::Material::AMBIENT_AND_DIFFUSE);
	      //if(!applyNonVisMat){
	      
	      utstateset->setAttribute(material);
	      // }else {
	      
	      if(applyNonVisMat){
		osg::PolygonOffset* polyoffset = new osg::PolygonOffset;
		polyoffset->setFactor(-1.0f);
		polyoffset->setUnits(-1.0f);
		//    osg::PolygonMode* polymode = new osg::PolygonMode;
		//polymode->setMode(osg::PolygonMode::FRONT_AND_BACK,osg::PolygonMode::LINE);
		utstateset->setAttributeAndModes(polyoffset,osg::StateAttribute::OVERRIDE|osg::StateAttribute::ON);
		//utstateset->setAttributeAndModes(polymode,osg::StateAttribute::OVERRIDE|osg::StateAttribute::ON);
		
		/*
		  osg::Material* material = new osg::Material;
		  utstateset->setAttributeAndModes(material,osg::StateAttribute::OVERRIDE|osg::StateAttribute::ON);
		  utstateset->setMode(GL_LIGHTING,osg::StateAttribute::OVERRIDE|osg::StateAttribute::OFF);
		  //fprintf(stderr,"Apply non vis\n");
		  */
		
	      }
	    }
	    untextured->addDrawable(gc._geom);
	  }

        }

    }
      if(usePlaneDist){

	osg::Switch *toggle_plane=new osg::Switch();
	osg::Geode *planeGeode=new osg::Geode();
	osg::Geode *boxGeode=new osg::Geode();
	osg::StateSet* _stateset = new osg::StateSet;
	osg::PolygonMode* _polygonmode = new osg::PolygonMode;
	_polygonmode->setMode(osg::PolygonMode::FRONT_AND_BACK,
			      osg::PolygonMode::LINE);
	_stateset->setAttribute(_polygonmode);
	_stateset->setMode( GL_LIGHTING, osg::StateAttribute::PROTECTED | osg::StateAttribute::OFF );

   
	
	

	for(int i=0; i<(int)bounds.size(); i++){

	  osg::ref_ptr<osg::Box> b=new osg::Box(osg::Vec3(bounds[i].center()[0],
							  bounds[i].center()[1],
							  bounds[i].center()[2]),
						bounds[i].size()[0],
						bounds[i].size()[1],
						bounds[i].size()[2]);
	  
	  //  printf("%f %f %f\n",bounds[i].center()[0],bounds[i].center()[1],bounds[i].center()[2]);

	  //	  printf("%f %f %f\n",bounds[i].size()[0],bounds[i].size()[1],bounds[i].size()[2]);

	  osg::ShapeDrawable* _shapedrawable = new osg::ShapeDrawable;
	  _shapedrawable->setColor(osg::Vec4(1.0,0.0,0.0,0.80));
	  _shapedrawable->setShape(b.get());
	  _shapedrawable->setStateSet(_stateset); 
	  boxGeode->addDrawable(_shapedrawable);
	  osg::Geometry* linesGeom = new osg::Geometry();
	  Plane3D p=planes[i];
	  osg::Vec3 center(bounds[i].center()[0],bounds[i].center()[1],bounds[i].center()[2]);
	  osg::Vec3 rot_center=rot->preMult(center);
	  //	  cout << rot_center;
	  osg::Vec3Array* vertices = displayPlane(p,Point3D(rot_center[0],rot_center[1],rot_center[2]));
	  
	  // pass the created vertex array to the points geometry object.
	  linesGeom->setVertexArray(vertices);
	  
	  // set the colors as before, plus using the above
	  osg::Vec4Array* colors = new osg::Vec4Array;
	  colors->push_back(osg::Vec4(0.0f,1.0f,0.0f,1.0f));
	  linesGeom->setColorArray(colors);
	  linesGeom->setColorBinding(osg::Geometry::BIND_OVERALL);
	  
	  
	  // set the normal in the same way color.
	  osg::Vec3Array* normals = new osg::Vec3Array;
	  normals->push_back(osg::Vec3(0.0f,-1.0f,0.0f));
	  linesGeom->setNormalArray(normals);
	  linesGeom->setNormalBinding(osg::Geometry::BIND_OVERALL);
	  
	  
	  // This time we simply use primitive, and hardwire the number of coords to use 
    // since we know up front,
	  linesGeom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES,0,vertices->size()));
	  linesGeom->setStateSet(_stateset); 
	  planeGeode->addDrawable(linesGeom); 
	 
	}
      
	osg::MatrixTransform *rotT =new osg::MatrixTransform(*rot);
	rotT->addChild(boxGeode);
	toggle_plane->addChild(rotT);
	toggle_plane->addChild(planeGeode);

	toggle_plane->setAllChildrenOff();
	if(toggle_ptr)
	  toggle_ptr->addChild(toggle_plane);
	osg::Point* point = new osg::Point();
	point->setSize( 4.0 );
	textured->getOrCreateStateSet()->setAttribute( point, osg::StateAttribute::ON );
      }
      
  	    if(computeHists){
	      /*	int max_planes=32;
	if(max_planes < planes.size()){
	  fprintf(stderr,"Max planes to small\n");
	  exit(-1);
	}
	osg::Vec4Array* planeArray = new osg::Vec4Array(max_planes);
	
	if(usePlaneDist){
	  for(int i=0; i <max_planes; i++){
	    osg::Vec4 plane;
	      if(i < planes.size()){
		plane[0]= planes[i].u[0];
		plane[1]=planes[i].u[1];
		plane[2]=planes[i].u[2];
		plane[3]=planes[i].d0;
	      }
	      else{
		plane[0]=0;
		plane[1]=0;
		plane[2]=0;
		plane[3]=0;
	      }
	      
	      cout << plane<<endl;
	  planeArray->push_back(plane);
	  }	      

	
	 
	  textured->getOrCreateStateSet()->addUniform(new osg::Uniform("planeArray", 
	  planeArray) );
      }*/

	  if(gpuNovelty){
	    textured->getOrCreateStateSet()->addUniform(new osg::Uniform("hist", 
							       TEXUNIT_HIST) );
	    textured->getOrCreateStateSet()->setTextureAttribute(TEXUNIT_HIST,
						       histTex);
	    textured->getOrCreateStateSet()->addUniform( new osg::Uniform("binsize",
								16.0f));
	  }
	  
	  //textured->getOrCreateStateSet()->addUniform(new osg::Uniform("rtex",0));
	  //	 textured->getOrCreateStateSet()->addUniform(new osg::Uniform("infoT",
	  //	       TEXUNIT_INFO));
	  
	  
	 //	 textured->getOrCreateStateSet()->addUniform(new osg::Uniform("planes", 
	 //				       TEXUNIT_PLANES) );
	 //	 textured->getOrCreateStateSet()->setTextureAttribute(TEXUNIT_PLANES,
	 //					      planeTex);
	 textured->getOrCreateStateSet()->setAttributeAndModes(novelty_program,
								osg::StateAttribute::ON );
      }else if(shader_coloring){     
	osg::Program* program=NULL;
	program = new osg::Program;
	program->setName( "colorshader" );
	//  printf("Here\n");
	osg::Shader *hcolorf=new osg::Shader( osg::Shader::FRAGMENT);
	osg::Shader *hcolorv=new osg::Shader( osg::Shader::VERTEX);
	loadShaderSource( hcolorf, basedir+"hcolor.frag" );
	loadShaderSource( hcolorv, basedir+"hcolor.vert" );
	program->addShader(  hcolorf );
	program->addShader(  hcolorv );
	textured->getOrCreateStateSet()->setAttributeAndModes( program,
							       osg::StateAttribute::ON );
	textured->getOrCreateStateSet()->addUniform(new osg::Uniform( "zrange", 
					       osg::Vec3(zrange[0],
							 zrange[1], 0.0f)));
	textured->getOrCreateStateSet()->addUniform( new osg::Uniform( "untex",
								       false));

	
	      
      }else {
	if(computeHists){
	  textured->getOrCreateStateSet()->setMode(GL_BLEND,
						   osg::StateAttribute::OFF);
	  textured->getOrCreateStateSet()->setMode(GL_ALPHA_TEST,
						   osg::StateAttribute::OFF);
	  
	}else{
	  
	  
	  osg::TexEnvCombine *te = new osg::TexEnvCombine;    
	  // Modulate diffuse texture with vertex color.
	  te->setCombine_RGB(osg::TexEnvCombine::REPLACE);
	  te->setSource0_RGB(osg::TexEnvCombine::TEXTURE);
	  te->setOperand0_RGB(osg::TexEnvCombine::SRC_COLOR);
	  te->setSource1_RGB(osg::TexEnvCombine::PREVIOUS);
	  te->setOperand1_RGB(osg::TexEnvCombine::SRC_COLOR);
	  
	  // Alpha doesn't matter.
	  te->setCombine_Alpha(osg::TexEnvCombine::REPLACE);
	  te->setSource0_Alpha(osg::TexEnvCombine::PREVIOUS);
	  te->setOperand0_Alpha(osg::TexEnvCombine::SRC_ALPHA);
	  
	  textured->getOrCreateStateSet()->setTextureAttribute(0, te);
	  
	  /*	osg::Material* material = new osg::Material;
		
		osg::Vec4 specular( 0.18, 0.18, 0.18, 0.18 );
		specular *= 1.5;
		float mat_shininess = 64.0f ;
		osg::Vec4 ambient (0.92, 0.92, 0.92, 0.95 );
		osg::Vec4 diffuse( 0.8, 0.8, 0.8, 0.85 );
		
		
		material->setAmbient(osg::Material::FRONT_AND_BACK,ambient);
		
		material->setSpecular(osg::Material::FRONT_AND_BACK,specular);
		material->setShininess(osg::Material::FRONT_AND_BACK,mat_shininess);
		//     material->setColorMode(  osg::Material::AMBIENT_AND_DIFFUSE);
		
		stateset->setAttribute(material);*/
	}
      }
	    
 if(textured->getNumDrawables())
  group[0]=textured.get();
 else 
   group[0]=NULL;

if(untextured->getNumDrawables() )
   group[1]=untextured.get();
 else
   group[1]=NULL;

  return true;
}

class CompressTexturesVisitor : public osg::NodeVisitor
{
public:

    CompressTexturesVisitor(osg::Texture::InternalFormatMode internalFormatMode):
        osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN),
        _internalFormatMode(internalFormatMode) {}

    virtual void apply(osg::Node& node)
    {
        if (node.getStateSet()) apply(*node.getStateSet());
        traverse(node);
    }
    
    virtual void apply(osg::Geode& node)
    {
        if (node.getStateSet()) apply(*node.getStateSet());
        
        for(unsigned int i=0;i<node.getNumDrawables();++i)
        {
            osg::Drawable* drawable = node.getDrawable(i);
            if (drawable && drawable->getStateSet()) apply(*drawable->getStateSet());
        }
        
        traverse(node);
    }
    
    virtual void apply(osg::StateSet& stateset)
    {
        // search for the existance of any texture object attributes
        for(unsigned int i=0;i<stateset.getTextureAttributeList().size();++i)
        {
            osg::Texture* texture = dynamic_cast<osg::Texture*>(stateset.getTextureAttribute(i,osg::StateAttribute::TEXTURE));
            if (texture)
            {
                _textureSet.insert(texture);
            }
        }
    }
    
    void compress()
    {
      osg::notify(osg::INFO)<<"Compress TEXTURE Visitor Compressing"<<std::endl;
      if(!mgc){
	MyGraphicsContext context;
	if (!context.valid()){
	  osg::notify(osg::INFO)<<"Error: Unable to create graphis context - cannot run compression"<<std::endl;
	  return;
        }
      }
        osg::ref_ptr<osg::State> state = new osg::State;

        for(TextureSet::iterator itr=_textureSet.begin();
            itr!=_textureSet.end();
            ++itr)
        {
            osg::Texture* texture = const_cast<osg::Texture*>(itr->get());
            
            osg::Texture2D* texture2D = dynamic_cast<osg::Texture2D*>(texture);
            osg::Texture3D* texture3D = dynamic_cast<osg::Texture3D*>(texture);
            
            osg::ref_ptr<osg::Image> image = texture2D ? texture2D->getImage() : (texture3D ? texture3D->getImage() : 0);
            if (image.valid() && 
                (image->getPixelFormat()==GL_RGB || image->getPixelFormat()==GL_RGBA) &&
                (image->s()>=32 && image->t()>=32))
            {
                texture->setInternalFormatMode(_internalFormatMode);
                
                // need to disable the unref after apply, other the image could go out of scope.
                bool unrefImageDataAfterApply = texture->getUnRefImageDataAfterApply();
		
                texture->setUnRefImageDataAfterApply(false);
                
                // get OpenGL driver to create texture from image.
                texture->apply(*state);

                // restore the original setting
                texture->setUnRefImageDataAfterApply(unrefImageDataAfterApply);

                image->readImageFromCurrentTexture(0,true);

                texture->setInternalFormatMode(osg::Texture::USE_IMAGE_DATA_FORMAT);
            }
        }
    }
    
    typedef std::set< osg::ref_ptr<osg::Texture> > TextureSet;
    TextureSet                          _textureSet;
    osg::Texture::InternalFormatMode    _internalFormatMode;

};


bool OSGExporter::outputModelOSG(char *out_name,  osg::ref_ptr<osg::Geode> *group,osg::Group *toggle_ptr) {
 
  run_higher_res=true;
  osg::Geode *tex=group[0].get();
  osg::Geode *untex =group[1].get();
  osg::Matrix trans(   osg::Matrix::rotate(osg::inDegrees(-90.0f),
		                           1.0f,0.0f,0.0f)*
		       osg::Matrix::rotate(osg::inDegrees(-90.0f),0.0f,
					   1.0f,0.0f));


  osg::ref_ptr<osg::MatrixTransform> positioned1 = new osg::MatrixTransform(trans);

  
  osg::ref_ptr<osg::MatrixTransform> positioned2= new osg::MatrixTransform(trans);

   
  if(do_atlas && tex){
 
    if(verbose)
      printf("Texture Atlas Creation\n"); 
    osgUtil::Optimizer optimizer;
    osgUtil::Optimizer::TextureAtlasVisitor ctav(&optimizer);
    osgUtil::Optimizer::TextureAtlasBuilder &tb=ctav.getTextureAtlasBuilder();
    tb.setMargin(0);
    tb.setMaximumAtlasSize(2048,2048);
    tex->accept(ctav);
    ctav.optimize();
    int options=0;
    options |= osgUtil::Optimizer::DEFAULT_OPTIMIZATIONS;
    options |=osgUtil::Optimizer::TEXTURE_ATLAS_BUILDER;
    optimizer.optimize(tex,options);
 
    if(compress_tex){
      if(computeHists){
	CompressTexturesVisitor ctv(osg::Texture::USE_S3TC_DXT5_COMPRESSION);
	tex->accept(ctv);
	ctv.compress();
      }else{
	CompressTexturesVisitor ctv(osg::Texture::USE_S3TC_DXT1_COMPRESSION);
	tex->accept(ctv);
	ctv.compress();
      }
      
    }
    SetDynamicStateSetVisitor sdssv;
    tex->accept(sdssv);
  }
  osgDB::ReaderWriter::WriteResult result;
  char outtex[255];
  string outname_str(out_name);
  if(tex && tex->getNumDrawables()){

      
    positioned1->addChild(tex);
    if(toggle_ptr)
      positioned1->addChild(toggle_ptr);
    strcpy(outtex,(outname_str.substr(0,outname_str.length()-4)+string("-t.ive")).c_str());
    
    result = osgDB::Registry::instance()->writeNode(*positioned1,outtex,osgDB::Registry::instance()->getOptions());
    if (result.success())	{
      if(verbose)
	osg::notify(osg::NOTICE)<<"Data written to '"<<outtex<<"'."<< std::endl;
    }
    else if  (result.message().empty()){
      osg::notify(osg::NOTICE)<<"Warning: file write to '"<<outtex<<"' no supported."<< std::endl;
      // root->getChild(0)=NULL;
    }else
      osg::notify(osg::NOTICE)<<"Writer output: "<< result.message()<<std::endl;
  }  

  if(untex && untex->getNumDrawables()){
    positioned2->addChild(untex);
    strcpy(outtex,(outname_str.substr(0,outname_str.length()-4)+string("-u.ive")).c_str());
    
    
    result = osgDB::Registry::instance()->writeNode(*positioned2,outtex,osgDB::Registry::instance()->getOptions());
    if (result.success())	{
      if(verbose)
	osg::notify(osg::NOTICE)<<"Data written to '"<<outtex<<"'."<< std::endl;
    }
    else if  (result.message().empty()){
      osg::notify(osg::NOTICE)<<"Warning: file write to '"<<outtex<<"' no supported."<< std::endl;
      // root->getChild(0)=NULL;
    }
  }
  /*if(do_atlas){
    UnrefTextureAtlas uta;
    tex->accept(uta);
    uta.unref_ptrs();
    }*/
  /*
  {
   MaterialToGeometryCollectionMap::iterator itr;
   for(itr=mtgcm.begin(); itr!=mtgcm.end(); ++itr){
     itr->second->_geom->unref();
     // if(itr->second) 
     //delete itr->second;
     
     
     }
     }*/
 mtgcm.clear();
  return true;

  /*  root->removeChild(0,1);
      geode=NULL;*/
  
}




osg::Image *OSGExporter::LoadResizeSave(string filename,string outname,bool save,int tex_size){

  IplImage *fullimg=NULL;
  bool cached=false;
  if(tex_image_cache.find(filename) == tex_image_cache.end() || !tex_image_cache[filename] ){
    fullimg=cvLoadImage(filename.c_str(),1);
   
  }
  else{

    fullimg=tex_image_cache[filename];
   
    if(fullimg && fullimg->width < tex_size){
      printf("Warning Upsampling from cached so reloading\n");
      fullimg=cvLoadImage(filename.c_str(),-1);
    }else{
      cached=true;
    }
  }

  IplImage *cvImg=cvCreateImage(cvSize(tex_size,tex_size),
				IPL_DEPTH_8U,3);
  
  if(fullimg){
    
    cvResize(fullimg,cvImg);
    cvReleaseImage(&fullimg);
    tex_image_cache[filename]=cvImg;
  }
  
  else {
    printf("Failed to load %s\n",filename.c_str());
    cvReleaseImage(&cvImg);
    return NULL;
  }

  osg::Image* image=NULL;
  
  if(compress_tex && !_hardware_compress){
    image =Convert_OpenCV_TO_OSG_IMAGE(cvImg,!cached,true);
  }else
    image =Convert_OpenCV_TO_OSG_IMAGE(cvImg,!cached,false);
 
  image->setFileName(osgDB::getSimpleFileName(filename));
  if(save){  
    //printf("Writing %s\n",outname.c_str());
    osgDB::writeImageFile(*image,outname);
    image->setFileName(outname);
  }
  return image;
}





static void texcoord_foreach_face (T_Face * f,
				   texGenData *data)
{
  
  int indexClosest=find_closet_img_trans(&GTS_FACE(f)->triangle,
					 data->bboxTree,data->camPosePts,
					 data->back_trans,data->calib,bboxes_all,data->margin);
  //fprintf(ffp,"%d\n",indexClosest);
  if(indexClosest == INT_MAX){
    /*fprintf(errFP,"Failed traingle\n");
      gts_write_triangle(&GTS_FACE(f)->triangle,NULL,errFP);
      fflush(errFP);*/
    if(data->verbose)
      libpolyp::tex_add_verbose(data->count++,data->total,data->reject++);
    return;
  }


  if(apply_tex_to_tri(f,data->calib,data->back_trans[indexClosest],indexClosest,data->tex_size,data->margin,data->use_dist_coords))
    data->validCount++;
  else{
    //printf("Index closest %d\n",indexClosest);
    find_closet_img_trans(&GTS_FACE(f)->triangle,
			  data->bboxTree,data->camPosePts,
			  data->back_trans,data->calib,bboxes_all,data->margin);
  }
 
  if(data->verbose)
    tex_add_verbose(data->count++,data->total,data->reject);

}


static void findimg_foreach_face (T_Face * f,
                                   texGenData *data)
{

  int indexClosest=find_first_bbox(&GTS_FACE(f)->triangle,
                                         data->bboxTree,bboxes_all,data->back_trans);
  //fprintf(ffp,"%d\n",indexClosest);
  if(indexClosest == INT_MAX){
   // fprintf(stderr,"Failed traingle\n");
      //gts_write_triangle(&GTS_FACE(f)->triangle,NULL,stderr);
     // fflush(stderr);
    if(data->verbose)
      libpolyp::tex_add_verbose(data->count++,data->total,data->reject++);
    return;
  }

//if(indexClosest != 93)
//    return;
  if(apply_texid_to_tri(f,indexClosest))
    data->validCount++;


  if(data->verbose)
    tex_add_verbose(data->count++,data->total,data->reject);

}

static void texcoord_blend_foreach_face (T_Face * f,
				   texGenData *data)
{
  int idx[NUM_TEX_BLEND_COORDS];
  bool found=find_blend_img_trans(&GTS_FACE(f)->triangle,
					 data->bboxTree,data->camPosePts,
				  data->back_trans,data->calib,idx,bboxes_all,data->margin);
  for(int i=0; i <NUM_TEX_BLEND_COORDS; i++)
    //fprintf(ffp,"%d ",idx[i]);
    // fprintf(ffp,"\n");
  if(!found){
    /*fprintf(errFP,"Failed traingle\n");
      gts_write_triangle(&GTS_FACE(f)->triangle,NULL,errFP);
      fflush(errFP);*/
    if(data->verbose)
      libpolyp::tex_add_verbose(data->count++,data->total,data->reject++);
    return;
  }
    
  if(apply_blend_tex_to_tri(f,data->calib,data->back_trans,idx,data->tex_size,data->margin))
    data->validCount++;
  else{
    //printf("Failed\n");
  }
 
  if(data->verbose)
    tex_add_verbose(data->count++,data->total,data->reject);

}
static void findborder_blend_foreach_face (T_Face * f,
				     texGenData *data)
{
  GtsTriangle * t = &GTS_FACE(f)->triangle;
  TVertex * v1,* v2,* v3; 
  gts_triangle_vertices(t,(GtsVertex **)& v1, 
			(GtsVertex **)&v2, (GtsVertex **)&v3);
  for(int i=0; i < NUM_TEX_BLEND_COORDS; i++){
    if(f->materialB[i] != v1->idB[i] || f->materialB[i] != v2->idB[i] || f->materialB[i] != v3->idB[i]){
    boost::mutex::scoped_lock scoped_lock(bfMutex);
    data->border_faces->push_back(f);
    return;
    }
  }
}


static void findborder_foreach_face (T_Face * f,
				     texGenData *data)
{
  GtsTriangle * t = &GTS_FACE(f)->triangle;
  TVertex * v1,* v2,* v3; 
  gts_triangle_vertices(t,(GtsVertex **)& v1, 
			(GtsVertex **)&v2, (GtsVertex **)&v3);
  if(f->material != v1->id || f->material != v2->id || f->material != v3->id){
    boost::mutex::scoped_lock scoped_lock(bfMutex);
    data->border_faces->push_back(f);
  }
}

void gen_mesh_tex_coord(GtsSurface *s ,Camera_Calib *calib, std::map<int,GtsMatrix *> back_trans,GNode * bboxTree,int tex_size, int num_threads,int verbose,int blend,int margin,bool use_dist_coords){
  //ffp=fopen("w.txt","w");
  
  //errFP=fopen("err.txt","w");
  std::vector<GtsPoint> camPosePts;
  GtsPoint transP;
  if(verbose)
    printf("Size %d\n",(int)back_trans.size());
  map<int,GtsMatrix *>::iterator iter;
  for(  iter=back_trans.begin();  iter != back_trans.end(); iter++){
  
    GtsMatrix *m= gts_matrix_inverse(iter->second); 
    transP.x=m[0][3];
    transP.y=m[1][3];
    transP.z=m[2][3];
    camPosePts.push_back(transP);
    gts_matrix_destroy(m);
  }

  
  gts_surface_foreach_vertex(s,(GtsFunc)set_tex_id_unknown,NULL);

  std::vector<T_Face *> border_faces;

  texGenData tex_data;
  tex_data.bboxTree=bboxTree;
  tex_data.camPosePts =camPosePts;
  tex_data.margin=margin;
  tex_data.use_dist_coords=use_dist_coords;
  tex_data.back_trans=back_trans;
  tex_data.validCount=0;
  tex_data.tex_size=tex_size;
  tex_data.count=1;
  tex_data.reject=0;
  tex_data.total=gts_surface_face_number(s);
  tex_data.calib=calib;
  tex_data.verbose=verbose;
  tex_data.border_faces = &border_faces;
  if(!blend){
    
    if(num_threads > 1)
      threaded_surface_foreach_face (s, (GtsFunc)texcoord_foreach_face ,
				     &tex_data ,num_threads);
    else
      gts_surface_foreach_face (s, (GtsFunc)texcoord_foreach_face ,&tex_data);
  }else {
    if(num_threads > 1)
      threaded_surface_foreach_face (s, (GtsFunc)texcoord_blend_foreach_face ,
				     &tex_data ,num_threads);
    else
      gts_surface_foreach_face (s, (GtsFunc)texcoord_blend_foreach_face ,&tex_data);
  }
  //fclose(ffp);
if(verbose)
      printf("\nChecking weird border cases...\n");
 if(!blend){
   if(num_threads > 1)
     threaded_surface_foreach_face (s, (GtsFunc)findborder_foreach_face ,
				    &tex_data ,num_threads);   
   else
     gts_surface_foreach_face (s, (GtsFunc)findborder_foreach_face ,&tex_data );
  
 }else{

  if(num_threads > 1)
     threaded_surface_foreach_face (s, (GtsFunc)findborder_blend_foreach_face ,
				    &tex_data ,num_threads);   
   else
     gts_surface_foreach_face (s, (GtsFunc)findborder_blend_foreach_face ,&tex_data );
 
 }

 tex_data.count=1;
 tex_data.reject=0;
  tex_data.total =border_faces.size();
  for(int i=0; i < (int) border_faces.size(); i++){
    T_Face *f2 = copy_face(border_faces[i],s);

    if(!blend){
      int idx=find_closet_img_trans(&GTS_FACE(f2)->triangle,bboxTree,camPosePts,back_trans,calib,bboxes_all,margin);
      if(apply_tex_to_tri(f2,calib,back_trans[idx],idx,tex_size,margin,use_dist_coords))
	tex_data.validCount++;
    }else{
      int idx[NUM_TEX_BLEND_COORDS];
      if(find_blend_img_trans(&GTS_FACE(f2)->triangle,bboxTree,camPosePts,back_trans,calib,idx,bboxes_all,margin))
	if(apply_blend_tex_to_tri(f2,calib,back_trans,idx,tex_size,margin))
	  tex_data.validCount++;
    }


    gts_surface_remove_face(s,GTS_FACE(border_faces[i]));
    if(verbose)
      tex_add_verbose(tex_data.count++,tex_data.total,tex_data.reject);
  }
  if(verbose)
    printf("\nValid tex %d\n", tex_data.validCount);
 
	   
  
}

void gen_mesh_tex_id(GtsSurface *s ,GNode * bboxTree, std::map<int,GtsMatrix *> back_trans,int num_threads,int verbose){



  gts_surface_foreach_vertex(s,(GtsFunc)set_tex_id_unknown,NULL);

  texGenData tex_data;
  tex_data.bboxTree=bboxTree;

  tex_data.validCount=0;
  tex_data.count=1;
  tex_data.reject=0;
  tex_data.total=gts_surface_face_number(s);
  tex_data.verbose=verbose;
  tex_data.back_trans=back_trans;


    if(num_threads > 1)
      threaded_surface_foreach_face (s, (GtsFunc)findimg_foreach_face ,
                                     &tex_data ,num_threads);
    else
      gts_surface_foreach_face (s, (GtsFunc)findimg_foreach_face ,&tex_data);

}

OSGExporter::~OSGExporter(){
  {
    map<string, IplImage *>::const_iterator itr;
    for(itr = tex_image_cache.begin(); itr != tex_image_cache.end(); ++itr){
      IplImage *tmp=(*itr).second;
      cvReleaseImage(&tmp); 
    }
    tex_image_cache.clear();
  }
  for(unsigned int i=0; i<decompressed_ptrs.size(); i++){
    //   decompressed_ptrs[i]->unref();
  }

 for(unsigned int i=0; i<resize_data_ptrs.size(); i++){
   if(resize_data_ptrs[i])
     delete resize_data_ptrs[i];
  }
 for(unsigned int i=0; i<downsampled_img_ptrs.size(); i++){
   // printf(" %d\n",downsampled_img_ptrs[i]->referenceCount());  
   do{
     downsampled_img_ptrs[i]->unref();
   }while(downsampled_img_ptrs[i]->referenceCount()> 0);
 }
  decompressed_ptrs.clear();
 {
   std::map<string,osg::Image * >::const_iterator itr;
   for(itr = compressed_img_cache.begin(); itr != compressed_img_cache.end(); ++itr){
      

      itr->second->unref();
   }
 
 }
  for(unsigned int i=0; i< gc_ptrs.size(); i++)
   if(gc_ptrs[i])
      delete gc_ptrs[i];
 //      Seg faults ????
 
 gc_ptrs.clear();
 
 
}


