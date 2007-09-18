#include "OSGExport.h"
#include <osgUtil/TriStripVisitor>
#include <osgUtil/SmoothingVisitor>
#include <osg/GraphicsContext>
#include <osgDB/WriteFile>


//#include <osgUtil/Tessellator>
// collect all the data relavent to a particular osg::Geometry being created.

void OSGExporter::compress(osg::Texture2D* texture2D){
 
  if(!state)
    state = new osg::State;
  
  
  osg::ref_ptr<osg::Image> image = texture2D->getImage();
  if (image.valid() && 
      (image->getPixelFormat()==GL_RGB || image->getPixelFormat()==GL_RGBA) &&
      (image->s()>=32 && image->t()>=32)){
    //internalFormatMode=osg::Texture::USE_S3TC_DXT1_COMPRESSION;
    texture2D->setInternalFormatMode(internalFormatMode);
    
    // need to disable the unref after apply, other the image could go out of scope.
    bool unrefImageDataAfterApply = texture2D->getUnRefImageDataAfterApply();
    texture2D->setUnRefImageDataAfterApply(false);
   
    // get OpenGL driver to create texture from image.
    texture2D->apply(*state);
    
    // restore the original setting
    texture2D->setUnRefImageDataAfterApply(unrefImageDataAfterApply);
    
    image->readImageFromCurrentTexture(0,true);
    texture2D->setInternalFormatMode(osg::Texture::USE_IMAGE_DATA_FORMAT);
  }

}

static void bin_face_mat_osg (T_Face * f, gpointer * data){
  MaterialToGeometryCollectionMap *mtgcm=(MaterialToGeometryCollectionMap *)data[0];
  //uint uiFace =  *((guint *) data[1]);

  GeometryCollection& gc = (*mtgcm)[f->material];
  gc._numPoints += 3;
  gc._numPrimitives += 1;
  if (f->material >= 0) 
    gc._numPrimitivesWithTexCoords += 1;

  GUINT_TO_POINTER ((*((guint *) data[1]))++);
}

static void add_face_mat_osg (T_Face * f, gpointer * data){
 
  MaterialToGeometryCollectionMap *mtgcm=(MaterialToGeometryCollectionMap *)data[0];
 
  GeometryCollection& gc = (*mtgcm)[f->material];
  
  osg::PrimitiveSet::Mode mode;
  
  mode = osg::PrimitiveSet::TRIANGLES;
  
  gc._geom->addPrimitiveSet(new osg::DrawArrays(mode,gc._coordCount,3));
  gc._coordCount += 3;
  TVertex * v1,* v2,* v3;
  gts_triangle_vertices(&GTS_FACE(f)->triangle,(GtsVertex **)& v1, 
			  (GtsVertex **)&v2, (GtsVertex **)&v3);
 

  (*gc._vertices++).set(GTS_VERTEX(v1)->p.x,GTS_VERTEX(v1)->p.y,GTS_VERTEX(v1)->p.z);
  (*gc._vertices++).set(GTS_VERTEX(v2)->p.x,GTS_VERTEX(v2)->p.y,GTS_VERTEX(v2)->p.z);
  (*gc._vertices++).set(GTS_VERTEX(v3)->p.x,GTS_VERTEX(v3)->p.y,GTS_VERTEX(v3)->p.z); 

  /*
  (*gc._colors++).set(v3->r,v3->b,v3->g,1.0);
  (*gc._colors++).set(v2->r,v2->b,v2->g,1.0);
  (*gc._colors++).set(v1->r,v1->b,v1->g,1.0);
  */
  if (gc._texturesActive && f->material >= 0){
    (*gc._texcoords++).set(v1->u,1-v1->v);
    (*gc._texcoords++).set(v2->u,1-v2->v); 
    (*gc._texcoords++).set(v3->u,1-v3->v); 
    
  }
}



osg::Geode* OSGExporter::convertGtsSurfListToGeometry(GtsSurface *s, std::vector<string> textures) 
{
  
  MaterialToGeometryCollectionMap mtgcm;
  gpointer data[2];
  gint n=0;
  data[0]=&mtgcm;
  data[1] = &n;
      
   gts_surface_foreach_face (s, (GtsFunc) bin_face_mat_osg , data);
   MaterialToGeometryCollectionMap::iterator itr;
   for(itr=mtgcm.begin(); itr!=mtgcm.end(); ++itr){
     GeometryCollection& gc = itr->second;
     if (gc._numPrimitives){
      
       
       gc._geom = new osg::Geometry;
       
       osg::Vec3Array* vertArray = new osg::Vec3Array(gc._numPoints);
       gc._vertices = vertArray->begin();
       gc._geom->setVertexArray(vertArray);
       
       // set up color.
       /*
	 osg::Vec4Array* colorsArray = new osg::Vec4Array(gc._numPoints);
	  		
       gc._colors=colorsArray->begin();                 
       gc._geom->setColorArray(colorsArray);
       gc._geom->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
          */

       // set up texture if needed.
       if (itr->first >= 0 && itr->first < (int)textures.size()){
	 
	 std::string filename=prefixdir+textures[itr->first];
	 osg::notify(osg::INFO) << "ctex " << filename  << std::endl;
	 char fname[255];
	 sprintf(fname,"mesh/tex-%04d.png",itr->first);

	 if(!context){
	   printf("Can't use OPENGL without valid context");
	   exit(0);
	 }
	 osg::Image* image =osgDB::readImageFile(filename);
	 if(image)
	   image->scaleImage(tex_size,tex_size,1,GL_UNSIGNED_BYTE);
	 else 
	   printf("Failed to load %s\n",filename.c_str());

	 if(!tex_saved){
	 
	   if(!ive_out){	     
	     osgDB::writeImageFile(*image,fname);
	     image->setFileName(fname);
	   }
	 }
	 if (image){
	     
	     // create state
	     osg::StateSet* stateset = new osg::StateSet;
	     
	     // create texture
	     osg::Texture2D* texture = new osg::Texture2D;
	     texture->setImage(image);
	     stateset->setTextureAttributeAndModes(0,texture,
						   osg::StateAttribute::ON);
	     gc._texturesActive=true;
	    
	       
	     gc._geom->setStateSet(stateset);
	     
	     osg::Vec2Array* texcoordArray = new osg::Vec2Array(gc._numPoints);
	     gc._texcoords = texcoordArray->begin();
	     gc._geom->setTexCoordArray(0,texcoordArray);

	     if(compress_tex)
	       compress(texture);
	 }
       }
     }
   }
   
   gts_surface_foreach_face (s, (GtsFunc) add_face_mat_osg , data);
   osg::Geode* geode = new osg::Geode;
    
    // osgUtil::Tessellator tessellator;
    
    // add everthing into the Geode.    
    osgUtil::SmoothingVisitor smoother;
    for(itr=mtgcm.begin();
        itr!=mtgcm.end();
        ++itr)
    {
        GeometryCollection& gc = itr->second;
        if (gc._geom)
        {
            
          //  tessellator.retessellatePolygons(*gc._geom);
        
            smoother.smooth(*gc._geom);
            
            geode->addDrawable(gc._geom);
        }

    }
    return geode;
}

int OSGExporter::convertModelOSG(GtsSurface *s,std::vector<string> textures,std::string fileNameOut) {

  std::string ext = osgDB::getFileExtension(fileNameOut);
  ive_out= (ext=="ive");
  if(!ive_out){
    std::cout<<"Warning: compressing texture only supported when out";
    std::cout << "puting to .ive"<<std::endl;
    compress_tex=false;
    
  }

  
  osg::Group* root = new osg::Group;
  osg::Geode* geode = convertGtsSurfListToGeometry(s,textures);
  geode->setName(fileNameOut);
  root->addChild(geode);

  osgDB::ReaderWriter::WriteResult result = osgDB::Registry::instance()->writeNode(*root,fileNameOut);
  if (result.success())	{
    osg::notify(osg::NOTICE)<<"Data written to '"<<fileNameOut<<"'."<< std::endl;
    return true;
  }
  else if  (result.message().empty()){
    osg::notify(osg::NOTICE)<<"Warning: file write to '"<<fileNameOut<<"' no supported."<< std::endl;
  }
  else    {
    osg::notify(osg::NOTICE)<<result.message()<< std::endl;
  }
  
  return false;
  
}

