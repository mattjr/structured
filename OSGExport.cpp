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
#include <highgui.h>
#include <boost/thread/thread.hpp>
#include "novelty.h"
#include <sys/stat.h>
#include "auv_texture_utils.hpp"
#include "auv_lod.hpp"
#include "auv_gts_utils.hpp"
#include "auv_tex_projection.hpp"

using namespace libpolyp;
typedef struct _GHashNode      GHashNode;
using namespace libsnapper;
using namespace squish;



MyGraphicsContext *mgc=NULL;
std::vector<GtsBBox *> bboxes_all;;

boost::mutex bfMutex;

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
  if(mtgcm->count(f->material) <= 0)
    (*mtgcm)[f->material]= new GeometryCollection;
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
  int planeTexSize=(int)data[7];
  
  map<int,int> *texnum2arraynum=(map<int,int> *)data[9];

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

  if(gc._planeTexValid){
    if(v1->plane >= 0)
      (*gc._texcoordsPlane++).set(v1->plane %planeTexSize, v1->plane / planeTexSize);    
    else
      (*gc._texcoordsPlane++).set(-1.0,-1.0);    

 if(v2->plane >= 0)
      (*gc._texcoordsPlane++).set(v2->plane %planeTexSize, v2->plane / planeTexSize);    
    else
      (*gc._texcoordsPlane++).set(-1.0,-1.0);    

 if(v3->plane >= 0)
      (*gc._texcoordsPlane++).set(v3->plane %planeTexSize, v3->plane / planeTexSize);    
    else
      (*gc._texcoordsPlane++).set(-1.0,-1.0);    
    
  
  
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
    if(!zrange){
      (*gc._colors++).set(0.5,0.5,0.5,0.0);
      (*gc._colors++).set(0.5,0.5,0.5,0.0);
      (*gc._colors++).set(0.5,0.5,0.5,0.0);
    }
    float range=zrange[1]-zrange[0];
   
    Lut_Vec color;
    float val;// r,g,b,val;
    Colors::eColorMap map=Colors::eRainbowMap;
    val = ( GTS_VERTEX(v1)->p.z -zrange[0] )/range;  
    color=GlobalColors()->Get(map, val); 
    (*gc._colors++).set(color[0],color[1],color[2],1.0);
    //  jet_color_map(val,r,g,b);
    //(*gc._colors++).set(r,b,g,1.0);
   
    val = ( GTS_VERTEX(v2)->p.z -zrange[0] )/range;    
    color=GlobalColors()->Get(map, val); 
    (*gc._colors++).set(color[0],color[1],color[2],1.0);
    //jet_color_map(val,r,g,b);
    // (*gc._colors++).set(r,b,g,1.0);
   
    val = ( GTS_VERTEX(v3)->p.z -zrange[0] )/range;    
    color=GlobalColors()->Get(map, val); 
    (*gc._colors++).set(color[0],color[1],color[2],1.0);
    //jet_color_map(val,r,g,b);
    //  (*gc._colors++).set(r,b,g,1.0);
    
    
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

osg::Image *decompressImage(osg::Image *img){

  IplImage *tmp=cvCreateImage(cvSize(img->s(),img->t()),IPL_DEPTH_8U,3);
  DecompressImageRGB((unsigned char *)tmp->imageData,tmp->width,tmp->height,img->data(),squish::kDxt1);
  osg::Image *image_full= Convert_OpenCV_TO_OSG_IMAGE(tmp,false);
  return image_full;
}

osg::Image *OSGExporter::getCachedCompressedImage(string name,int size){
  IplImage *img=NULL;
  string basename=osgDB::getSimpleFileName(name);
  string ddsname=osgDB::getNameLessExtension(name);
  ddsname +=".dds";
  bool cachedLoaded=false;
  osg::Image *filecached=NULL;
  osg::Image *retImage=new osg::Image;
  if(compressed_img_cache.find(basename) == compressed_img_cache.end() || !compressed_img_cache[basename] ){
    
    if(FileExists(ddsname)){
      filecached=osgDB::readImageFile(ddsname);
      if(filecached)
	cachedLoaded=true;
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

    if(!compress_tex)
      return decompressImage(filecached);
   
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

  int datasize=filecached->getTotalSizeInBytesIncludingMipmaps()-filecached->getMipmapOffset(i);
  unsigned char *newdata = new unsigned char[datasize];
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
  
  if(!compress_tex)
    return decompressImage(retImage);
  else
    return retImage;
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
      
     osg::Image *img=compressed_img_cache[name];
     if(!img){
       printf("Image null in getTextureHists %s\n",name.c_str());
       continue;
     }
     IplImage *tmp=cvCreateImage(cvSize(img->s(),img->t()),
				 IPL_DEPTH_8U,3);
     IplImage *mask=cvNewGray(tmp);
     cvZero(mask);

     DecompressImageRGB((unsigned char *)tmp->imageData,tmp->width,tmp->height,img->data(),squish::kDxt1);

     int count = 3;
     CvPoint pt[3];
     CvPoint* tri=pt;
     for(int i=0; i< (int) gc._texcoordArray->size(); i+=3){
       for(int j=0; j<3; j++){
	 
	 osg::Vec2f tc = (*gc._texcoordArray)[i+j];
	 tc *= tmp->width;
	 pt[j]=cvPoint((int)tc[1],(int)tc[0]);

       }
       
       cvFillPoly(mask, &tri, &count, 1, CV_RGB(255,255,255));
     }
    
     histCalc.calc_hist(tmp,mask,1/*BGR*/);
     if(gpuNovelty){
       osg::Texture *tex=osg_tex_map[itr->first];
       IplImage *rgba=cvCreateImage(cvSize(tmp->width,tmp->height),
				    IPL_DEPTH_8U,4);
       cvCvtColor(tmp, rgba, CV_RGB2RGBA);
       cvSetImageCOI(rgba,4);
       cvCopy(histCalc.imageTex,rgba);
       cvSetImageCOI(rgba,0);
       osg::Image *imageWithG=Convert_OpenCV_TO_OSG_IMAGE(rgba,false);
       osg::Texture2D* texture2D = dynamic_cast<osg::Texture2D*>(tex);
       texture2D->setImage(imageWithG);
       compress(texture2D,osg::Texture::USE_S3TC_DXT5_COMPRESSION);
     }
     cvReleaseImage(&tmp);
     cvReleaseImage(&mask);
   }
   
  
}

void OSGExporter::addNoveltyTextures( MaterialToGeometryCollectionMap &mtgcm, map<int,string> textures, Hist_Calc &histCalc,CvHistogram *hist){

  MaterialToGeometryCollectionMap::iterator itr;
   for(itr=mtgcm.begin(); itr!=mtgcm.end(); ++itr){
     GeometryCollection& gc = *(itr->second);
     if(!gc._texturesActive )
       continue;
     string name=textures[itr->first];
      
     osg::Image *img=compressed_img_cache[name];
     if(!img){
       printf("Image null in getTextureHists %s\n",name.c_str());
       continue;
     }
     IplImage *tmp=cvCreateImage(cvSize(img->s(),img->t()),
				 IPL_DEPTH_8U,3);
     IplImage *med=cvNewGray(tmp);

     DecompressImageRGB((unsigned char *)tmp->imageData,tmp->width,tmp->height,img->data(),squish::kDxt1);

 
     IplImage *novelty_image=  histCalc.get_novelty_img(tmp,hist); 
     //mcvNormalize((IplImage*)novelty_image, (IplImage*)novelty_image, 0, 255);
     IplImage *texInfo=cvNewColor(novelty_image);
     IplImage *tmpG=cvNewGray(novelty_image);
     mcvNormalize((IplImage*)novelty_image, (IplImage*)novelty_image, 0, 255);
     cvConvertScale(novelty_image,tmpG);
     
     cvSmooth(tmpG,med,CV_MEDIAN,9);
     //   scaleJetImage( (IplImage*)novelty_image, (IplImage*)testImg);
  
     cvSetImageCOI(texInfo,1);
     cvCopy(med,texInfo);
     cvSetImageCOI(texInfo,0);
     osg::Image *texI=Convert_OpenCV_TO_OSG_IMAGE(texInfo,false);
     osg::Texture2D* texture2D = new osg::Texture2D(texI);
     compress(texture2D,osg::Texture::USE_S3TC_DXT1_COMPRESSION);
     gc._geom->getStateSet()->setTextureAttribute(TEXUNIT_INFO,texture2D );
     cvReleaseImage(&texInfo);
     cvReleaseImage(&tmpG);
     cvReleaseImage(&novelty_image);
     cvReleaseImage(&med);
     cvReleaseImage(&tmp);

   }
   
  
}


bool OSGExporter::convertGtsSurfListToGeometry(GtsSurface *s, map<int,string> textures,ClippingMap *cm,int tex_size,osg::ref_ptr<osg::Geode>*group,vector<Plane3D> planes,vector<TriMesh::BBox> bounds,VerboseMeshFunc vmcallback,float *zrange)
{
   _tex_size=tex_size;
   map<int,int> texnum2arraynum;
  MaterialToGeometryCollectionMap mtgcm;
  gpointer data[10];
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
      {
	osg::Vec4Array* colorsArray = new osg::Vec4Array(gc._numPoints);
	 
	 
	gc._colors=colorsArray->begin();                 
	gc._colorsActive=true;
	gc._geom->setColorArray(colorsArray);
	 
	gc._geom->setColorBinding(osg::Geometry::BIND_PER_VERTEX);

	

      }
      
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
	stateset->addUniform( new osg::Uniform( "shaderOut", 2));
	
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
	
	 
	osg::ref_ptr<osg::Image> image=getCachedCompressedImage(filename,tex_size);

	
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
	    // texture->setUnRefImageDataAfterApply( false );
	    texture->setImage(image.get());
	    //texture->setInternalFormatMode(internalFormatMode);
	    stateset->setTextureAttributeAndModes(0,texture,
						  osg::StateAttribute::ON);
	    if(computeHists){
	      stateset->setMode(GL_BLEND,osg::StateAttribute::OFF);
	      stateset->setMode(GL_ALPHA_TEST,osg::StateAttribute::OFF);
	      
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
	      
	      stateset->setTextureAttribute(0, te);
	    }
	    gc._texturesActive=true;
	    stateset->setDataVariance(osg::Object::STATIC);
	    
	    gc._geom->setStateSet(stateset);
	    
	    osg::Vec2Array* texcoordArray = new osg::Vec2Array(gc._numPoints);
	    gc._texcoords = texcoordArray->begin();
	    gc._texcoordArray = texcoordArray;
	    gc._geom->setTexCoordArray(0,texcoordArray);
	    if(computeHists){
	      osg::Vec2Array* texcoordPlanes = new osg::Vec2Array(gc._numPoints);
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
    osg::Program* program=NULL;

   
     
      program = new osg::Program;
      program->setName( "microshader" );
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
      
      program->addShader(  novelty );
       program->addShader( distNovelty );
 
      if(computeHists){
	
	Hist_Calc histCalc(_tex_size,_tex_size,16,1);
	CvHistogram *finalhist=NULL;    
	calcHists(mtgcm,textures,histCalc);
	histCalc.get_hist(finalhist);
	if(gpuNovelty)
	  histTex= getTextureHists(finalhist);
	else
	  addNoveltyTextures(mtgcm,textures,histCalc,finalhist);
      }
      
      
      // add everthing into the Geode.   
      osg::TextureRectangle* planeTex=	 getPlaneTex(planes,_planeTexSize);
      
      
      osgUtil::SmoothingVisitor smoother;
 
      for(int i=0; i< (int)gcAndTexIds.size(); i++)
    {
      GeometryCollection& gc = *(gcAndTexIds[i].first);
      if (gc._geom)
        {
	  gc._geom->setUseDisplayList(false);
          //  tessellator.retessellatePolygons(*gc._geom);
        
	  smoother.smooth(*gc._geom);
	   if(gc._texturesActive ){
	   
	       if(gpuNovelty){
		 gc._geom->getStateSet()->addUniform(new osg::Uniform("hist", 
								      TEXUNIT_HIST) );
		 gc._geom->getStateSet()->setTextureAttribute(TEXUNIT_HIST,
							      histTex);
		 gc._geom->getStateSet()->addUniform( new osg::Uniform("binsize",
								       16.0f));
	       }
	       if(computeHists){
		 gc._geom->getStateSet()->addUniform( new osg::Uniform( "shaderOut", 1));
		 gc._geom->getStateSet()->addUniform( new osg::Uniform( "weights", osg::Vec3(1.12f, 0.66f, 0.26f) ));
		 
		 
		 gc._geom->getStateSet()->addUniform(new osg::Uniform("rtex",0));
		 gc._geom->getStateSet()->addUniform(new osg::Uniform("infoT",
								      TEXUNIT_INFO));
		 
		 
		 gc._geom->getStateSet()->addUniform(new osg::Uniform("planes", 
									TEXUNIT_PLANES) );
		 gc._geom->getStateSet()->setTextureAttribute(TEXUNIT_PLANES,
							      planeTex);
		 
		 gc._geom->getStateSet()->setAttributeAndModes( program,
								osg::StateAttribute::ON );
	       }
	     
	       
	      textured->addDrawable(gc._geom);
	   }
	 
	  
	  else{
	    osg::StateSet *utstateset = gc._geom->getOrCreateStateSet();
	    osg::Material* material = new osg::Material;
	    
	    osg::Vec4 specular( 0.18, 0.18, 0.18, 0.18 );
	    specular *= 1.5;
	    float mat_shininess = 64.0f ;
	    osg::Vec4 ambient (0.02, 0.02, 0.05, 0.05 );
	    
	    osg::Vec4 light0_ambient( 0, 0, 0, 0 );
	    osg::Vec4 diffuse( 0.8, 0.8, 0.8, 0.85 );
	    
	    osg::Vec4 light1_diffuse( -0.01, -0.01, -0.03, -0.03 );
	    osg::Vec4 light0_specular( 0.85, 0.85, 0.85, 0.85 );
	    material->setAmbient(osg::Material::FRONT_AND_BACK,ambient);
	    
	    material->setSpecular(osg::Material::FRONT_AND_BACK,specular);
	    material->setShininess(osg::Material::FRONT_AND_BACK,mat_shininess);
	    material->setColorMode(  osg::Material::AMBIENT_AND_DIFFUSE);
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
	    untextured->addDrawable(gc._geom);
	  }

        }

    }
      if(usePlaneDist){

	osg::StateSet* _stateset = new osg::StateSet;
	osg::PolygonMode* _polygonmode = new osg::PolygonMode;
	_polygonmode->setMode(osg::PolygonMode::FRONT_AND_BACK,
			      osg::PolygonMode::LINE);
	_stateset->setAttribute(_polygonmode);
	_stateset->setMode( GL_LIGHTING, osg::StateAttribute::PROTECTED | osg::StateAttribute::OFF );
   
	
	
	
	for(int i=0; i<(int)bounds.size(); i++){

	  osg::ref_ptr<osg::Box> b=new osg::Box(osg::Vec3(bounds[i].center()[1],
							  bounds[i].center()[0],
							  -bounds[i].center()[2]),
						bounds[i].size()[1],
						bounds[i].size()[0],
						bounds[i].size()[2]);
	  
	  
	  osg::ShapeDrawable* _shapedrawable = new osg::ShapeDrawable;
	  _shapedrawable->setColor(osg::Vec4(1.0,0.0,0.0,0.80));
	  _shapedrawable->setShape(b.get());
	  _shapedrawable->setStateSet(_stateset); 
	  textured->addDrawable(_shapedrawable);
	  osg::Geometry* linesGeom = new osg::Geometry();
	  Plane3D p=planes[i];
	  float tmp = p.u[0];
	  p.u[0]=p.u[1];
	  p.u[1]=tmp;
	  p.u[2]=-p.u[2];
	  
	  
	  osg::Vec3Array* vertices = displayPlane(p,Point3D(bounds[i].center()[1],bounds[i].center()[0],-bounds[i].center()[2]));
	  
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
	  textured->addDrawable(linesGeom); 
	}
      

  
	osg::Point* point = new osg::Point();
	point->setSize( 4.0 );
	textured->getOrCreateStateSet()->setAttribute( point, osg::StateAttribute::ON );
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
     osg::notify(osg::NOTICE)<<"Compress TEXTURE Visitor Compressing"<<std::endl;
     MyGraphicsContext context;
        if (!context.valid())
        {
            osg::notify(osg::NOTICE)<<"Error: Unable to create graphis context - cannot run compression"<<std::endl;
            return;
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


bool OSGExporter::outputModelOSG(char *out_name,  osg::ref_ptr<osg::Geode> *group) {

  osg::Geode *tex=group[0].get();
  osg::Geode *untex =group[1].get();

 
  
  if(!compress_tex && tex){
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
    CompressTexturesVisitor ctv(osg::Texture::USE_S3TC_DXT5_COMPRESSION);
    tex->accept(ctv);
    ctv.compress();
  }
  osgDB::ReaderWriter::WriteResult result;
  char outtex[255];
  string outname_str(out_name);
  if(tex && tex->getNumDrawables()){
    strcpy(outtex,(outname_str.substr(0,outname_str.length()-4)+string("-t.ive")).c_str());
    
    result = osgDB::Registry::instance()->writeNode(*tex,outtex,osgDB::Registry::instance()->getOptions());
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
    strcpy(outtex,(outname_str.substr(0,outname_str.length()-4)+string("-u.ive")).c_str());
    
    
    result = osgDB::Registry::instance()->writeNode(*untex,outtex,osgDB::Registry::instance()->getOptions());
    if (result.success())	{
      if(verbose)
	osg::notify(osg::NOTICE)<<"Data written to '"<<outtex<<"'."<< std::endl;
    }
    else if  (result.message().empty()){
      osg::notify(osg::NOTICE)<<"Warning: file write to '"<<outtex<<"' no supported."<< std::endl;
      // root->getChild(0)=NULL;
    }
  }
  
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

OSGExporter::~OSGExporter(){
  map<string, IplImage *>::const_iterator itr;
  for(itr = tex_image_cache.begin(); itr != tex_image_cache.end(); ++itr){
    IplImage *tmp=(*itr).second;
    cvReleaseImage(&tmp); 
  }
  tex_image_cache.clear();
}


