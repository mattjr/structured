#include "OSGExport.h"
#include <osgUtil/TriStripVisitor>
#include <osgUtil/SmoothingVisitor>
#include <osg/GraphicsContext>
#include <osgDB/WriteFile>
#include <osgUtil/Simplifier>
#include <cv.h>
#include <glib.h>
#include <highgui.h>
#include <boost/thread/thread.hpp>
typedef struct _GHashNode      GHashNode;
using namespace libsnapper;
std::vector<GtsBBox *> bboxes_all;;

IplImage *doCvResize(osg::Image *img,int size){
  IplImage *in=cvCreateImageHeader(cvSize(img->s(),img->t()),IPL_DEPTH_8U,3);
  in->imageData=(char *)img->data();
  IplImage *tmp=cvCreateImage(cvSize(size,size),IPL_DEPTH_8U,3);
  cvResize(in,tmp);
  cvReleaseImageHeader(&in);
 int pixelFormat = GL_RGB;
 int dataType = GL_UNSIGNED_BYTE;
 //Detetes img data upon set
 img->setImage(tmp->width,tmp->height,1,3,pixelFormat,dataType,(unsigned char*)tmp->imageData,osg::Image::NO_DELETE);


 return tmp;
}
boost::mutex bfMutex;
//FILE *errFP;
int lastBP;
bool Export3DS(GtsSurface *s,const char *c3DSFile,vector<string> material_names)
#ifdef USE_LIB3DS 
  ;
#else
{printf("Not using lib3ds code cannot output 3ds\n");
  exit(0);
}
#endif
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
 

  /* (*gc._vertices++).set(GTS_VERTEX(v1)->p.x,GTS_VERTEX(v1)->p.y,GTS_VERTEX(v1)->p.z);
  (*gc._vertices++).set(GTS_VERTEX(v2)->p.x,GTS_VERTEX(v2)->p.y,GTS_VERTEX(v2)->p.z);
  (*gc._vertices++).set(GTS_VERTEX(v3)->p.x,GTS_VERTEX(v3)->p.y,GTS_VERTEX(v3)->p.z); 
  // printf("%f %f %f\n",GTS_VERTEX(v3)->p.x,GTS_VERTEX(v3)->p.y,GTS_VERTEX(v3)->p.z);*/

 
  (*gc._vertices++).set(GTS_VERTEX(v3)->p.y,GTS_VERTEX(v3)->p.x,-GTS_VERTEX(v3)->p.z);
  (*gc._vertices++).set(GTS_VERTEX(v2)->p.y,GTS_VERTEX(v2)->p.x,-GTS_VERTEX(v2)->p.z);
  (*gc._vertices++).set(GTS_VERTEX(v1)->p.y,GTS_VERTEX(v1)->p.x,-GTS_VERTEX(v1)->p.z);
 
  /*
  (*gc._colors++).set(v3->r,v3->b,v3->g,1.0);
  (*gc._colors++).set(v2->r,v2->b,v2->g,1.0);
  (*gc._colors++).set(v1->r,v1->b,v1->g,1.0);
  */
  if (gc._texturesActive && f->material >= 0){
 
    (*gc._texcoords++).set(v3->u,1-v3->v); 
   
    (*gc._texcoords++).set(v2->u,1-v2->v); 
  (*gc._texcoords++).set(v1->u,1-v1->v);  
   
  }
}



osg::ref_ptr<osg::Geode> OSGExporter::convertGtsSurfListToGeometry(GtsSurface *s, map<int,string> textures,int tex_size) 
{
  
  MaterialToGeometryCollectionMap mtgcm;
  gpointer data[2];
  gint n=0;
  data[0]=&mtgcm;
  data[1] = &n;
      
   gts_surface_foreach_face (s, (GtsFunc) bin_face_mat_osg , data);
   MaterialToGeometryCollectionMap::iterator itr;
   int tex_count=0;
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
   
       if (itr->first >= 0 &&  textures.count(itr->first) > 0){
	 
	 std::string filename=prefixdir+textures[itr->first];
	 osg::notify(osg::INFO) << "ctex " << filename  << std::endl;
	 char fname[255];
	 sprintf(fname,"mesh/tex-%04d.png",itr->first);

	 if(!context){
	   printf("Can't use OPENGL without valid context");
	   exit(0);
	 }
	 printf("\rLoading and Scaling Texture: %03d/%03d",++tex_count,mtgcm.size());
	 if(!ive_out)
	   printf("\n");	 
	 fflush(stdout); 
	
	 
	 osg::ref_ptr<osg::Image> image=LoadResizeSave(filename,fname, (!ive_out),tex_size);
	 if (image.valid()){	     
	   // create state
	   osg::StateSet* stateset = new osg::StateSet;
	   
	     // create texture
	   osg::Texture2D* texture = new osg::Texture2D;
	   texture->setUnRefImageDataAfterApply( true );
	   texture->setImage(image.get());
	   stateset->setTextureAttributeAndModes(0,texture,
						 osg::StateAttribute::ON);
	   gc._texturesActive=true;
	   stateset->setDataVariance(osg::Object::STATIC);
	   
	   gc._geom->setStateSet(stateset);
	     
	   osg::Vec2Array* texcoordArray = new osg::Vec2Array(gc._numPoints);
	   gc._texcoords = texcoordArray->begin();
	   gc._geom->setTexCoordArray(0,texcoordArray);
	   
	 
	   if(!state)
	     state = new osg::State;
	   texture->apply(*state);
	   
	   
	   osg_tex_ptrs.push_back(texture);
	 }
       }
     }
   }
  
   printf("\n");
   gts_surface_foreach_face (s, (GtsFunc) add_face_mat_osg , data);
   osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    
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

osg::ref_ptr<osg::Group> OSGExporter::convertModelOSG(GtsSurface *s,std::map<int,string> textures,char *out_name,int tex_size) {


  
  string format = osgDB::getFileExtension(string(out_name));
  
  ive_out= (format=="ive");
  
  if(format==".3ds"){
    Export3DS(s,out_name,textures,tex_size);
    return NULL;
  }else if(!ive_out && compress_tex){
    std::cout<<"Warning: compressing texture only supported when out";
    std::cout << "puting to .ive"<<std::endl;
    compress_tex=false;
  }    
 
  osg::ref_ptr<osg::Group> root_ptr = new osg::Group;

  osg::ref_ptr<osg::Geode> geode = convertGtsSurfListToGeometry(s,textures,tex_size);
  osg::Group * root = root_ptr.get();

  geode->setName(out_name);
  root->addChild(geode.get());
  printf("Texture Atlas Creation\n"); 
  osgUtil::Optimizer optimzer;
  osgUtil::Optimizer::TextureAtlasVisitor tav(&optimzer);
  osgUtil::Optimizer::TextureAtlasBuilder &tb=tav.getTextureAtlasBuilder();
  tb.setMaximumAtlasSize(2048,2048);
  root->accept(tav);
  tav.optimize();
  optimzer.optimize(root);
 
  if(compress_tex){
    int atlas_compressed_size=0;
    if(tex_size == 32)
      atlas_compressed_size=1024;
    printf("Texture Compression\n");
    CompressTexturesVisitor ctv(osg::Texture::USE_S3TC_DXT5_COMPRESSION,
				atlas_compressed_size);
    root->accept(ctv); 
    ctv.compress();
    
  }
 
  osgDB::ReaderWriter::WriteResult result = osgDB::Registry::instance()->writeNode(*root,out_name,osgDB::Registry::instance()->getOptions());
  if (result.success())	{
    osg::notify(osg::NOTICE)<<"Data written to '"<<out_name<<"'."<< std::endl;
    
    root=NULL;
    
  }
  else if  (result.message().empty()){
    osg::notify(osg::NOTICE)<<"Warning: file write to '"<<out_name<<"' no supported."<< std::endl;
    
  }
  
  
  return root;

  /*  root->removeChild(0,1);
      geode=NULL;*/
  
}
#ifdef USE_LIB3DS
/**
 * Create a camera and insert it into the file.
 *
 * \param file		3ds file.
 * \param position	camera position
 * \param target	camera target
 * \param near_range	camera near range
 * \param far_range	camera far range
 * \param fov		camera field of view; 45° is a good value.
 * \param name		camera name
 */

void
create_camera(Lib3dsFile *file, Lib3dsVector position, Lib3dsVector target,
	double near_range, double far_range, double fov, const char *name)
{
	Lib3dsCamera *camera = lib3ds_camera_new(name);

	memcpy(camera->position, position, sizeof(camera->position));
	memcpy(camera->target, target, sizeof(camera->target));
	camera->near_range = near_range;
	camera->far_range = far_range;
	camera->fov = fov;
	lib3ds_file_insert_camera(file, camera);
}

#define	EYE_HGT		5.5		/* assuming feet here */
/**
 * Create some cameras
 */
void
add_cameras( Lib3dsFile *file)
{
	Lib3dsVector	campos, target;
	double		eye = EYE_HGT;
	float	radius = 12.;

	target[0] = 0.; target[1] = 0.; target[2] = eye;
	campos[0] = 0.; campos[1] = -radius*3; campos[2] = eye;
	create_camera(file, campos, target, 0., radius*6, 45., "Front_view");

	campos[0] = -radius*3; campos[1] = 0.; campos[2] = eye;
	create_camera(file, campos, target, 0., radius*6, 45., "Left_side");

	campos[0] = -radius*2; campos[1] = -radius*2; campos[2] = radius*2;
	create_camera(file, campos, target, 0., radius*6, 45., "Aerial");

	campos[0] = 0.; campos[1] = radius*.5; campos[2] = eye;
	create_camera(file, campos, target, 0., radius*6, 60., "Inside");
}


static void store_vertex_3ds (TVertex * v, gpointer * data ){
 
  Lib3dsMesh *pMesh = (Lib3dsMesh *)data[0];
  uint uiVertex =  *((guint *) data[1]);

  pMesh->pointL[uiVertex].pos[0] =  GTS_VERTEX(v)->p.x;
  pMesh->pointL[uiVertex].pos[1] = GTS_VERTEX(v)->p.y;
  pMesh->pointL[uiVertex].pos[2] = GTS_VERTEX(v)->p.z;
  pMesh->texelL[uiVertex][0] = v->u;
  //Definition of top bottom of texture diffrent 1-y
  pMesh->texelL[uiVertex][1] = 1 - v->v;
  if(pMesh->texelL[uiVertex][0] >= 1.0 || pMesh->texelL[uiVertex][0] <= 0.0)
    pMesh->texelL[uiVertex][0] =0.0;

  if(pMesh->texelL[uiVertex][1] >= 1.0 || pMesh->texelL[uiVertex][1] <= 0.0)
    pMesh->texelL[uiVertex][1] =0.0;
  GTS_OBJECT(v)->reserved = GUINT_TO_POINTER ((*((guint *) data[1]))++);

 
}
static void store_face_3ds (T_Face * f, gpointer * data){

  Lib3dsMesh *pMesh = (Lib3dsMesh *)data[0];
  uint uiFace =  *((guint *) data[1]);
  MaterialToIDMap  material_names =  *((MaterialToIDMap *) data[2]);
  
  TVertex * v1, * v2, * v3;
  gts_triangle_vertices (&GTS_FACE(f)->triangle, (GtsVertex **)&v1, (GtsVertex **)&v2,(GtsVertex **)&v3);
  pMesh->faceL[uiFace].points[0] = GPOINTER_TO_UINT (GTS_OBJECT (v3)->reserved);
  pMesh->faceL[uiFace].points[1] = GPOINTER_TO_UINT (GTS_OBJECT (v2)->reserved);
  pMesh->faceL[uiFace].points[2] = GPOINTER_TO_UINT (GTS_OBJECT (v1)->reserved);
  gint id=f->material;

	
  if(id < 0 ){//|| !((v1->id == v2->id) && (v2->id == v3->id))){
    strcpy( pMesh->faceL[uiFace].material,"");
  
  }
  else
    strcpy(pMesh->faceL[uiFace].material,material_names[id].c_str() );
 
  GUINT_TO_POINTER ((*((guint *) data[1]))++);
}

void add_node(Lib3dsFile* file, Lib3dsMesh* mesh)
{
 Lib3dsNode* node;
 Lib3dsQuatKey* key_q;
 Lib3dsLin3Key* key_l;
 if (!file) {
	return;
 }
 if (!mesh) {
	return;
 }
 node = lib3ds_node_new_object();
 if (!node) {
	printf("Error creating new node\n");
	return;
 }
 node->parent_id = LIB3DS_NO_PARENT;
 node->next = 0;
 node->childs = 0;
 static int node_id = 0;
 node->node_id = node_id;
 node_id++;
 strncpy(node->name, mesh->name, 64);

 node->data.object.rot_track.keyL = 0;
 key_q = lib3ds_quat_key_new();
 key_q->q[0] = 0;
 key_q->q[1] = 0;
 key_q->q[2] = 0;
 key_q->q[3] = 1;
 lib3ds_quat_track_insert(&node->data.object.rot_track, key_q);
 lib3ds_quat_track_setup(&node->data.object.rot_track);

 node->data.object.pos_track.keyL = 0;
 key_l = lib3ds_lin3_key_new();
 lib3ds_lin3_track_insert(&node->data.object.pos_track, key_l);
 lib3ds_lin3_track_setup(&node->data.object.pos_track);

 node->data.object.scl_track.keyL = 0;
 key_l = lib3ds_lin3_key_new();
 key_l->value[0] = 1;
 key_l->value[1] = 1;
 key_l->value[2] = 1;
 lib3ds_lin3_track_insert(&node->data.object.scl_track, key_l);
 lib3ds_lin3_track_setup(&node->data.object.scl_track);


 lib3ds_file_insert_node(file, node);
}
std::string relative_path(std::string pathString){
	        std::string a = pathString;
	        unsigned int f = pathString.rfind('/');
	        if (f < pathString.length()) {
 	                a = a.substr(f+1);
 	        }
 	        return (a);
}

osg::Image* Convert_OpenCV_TO_OSG_IMAGE(IplImage* cvImg,bool flip){

        if(cvImg->nChannels == 3){
	  if(flip){
                // Flip image from top-left to bottom-left origin
                if(cvImg->origin == 0) {
                        cvConvertImage(cvImg , cvImg, CV_CVTIMG_FLIP);
                        cvImg->origin = 1;
                }

                // Convert from BGR to RGB color format
                //printf("Color format %s\n",cvImg->colorModel);
                cvCvtColor( cvImg, cvImg, CV_BGR2RGB );
	  }
                osg::Image* osgImg = new osg::Image();

                osgImg->setImage(
                        cvImg->width, //s
                        cvImg->height, //t
                        1, //r 3
                        3, //GLint internalTextureformat, (GL_LINE_STRIP,0x0003)
                        6407, // GLenum pixelFormat, (GL_RGB, 0x1907)
                        5121, // GLenum type, (GL_UNSIGNED_BYTE, 0x1401)
                        (unsigned char *)(cvImg->imageData), // unsigned char* data
                        osg::Image::NO_DELETE // AllocationMode mode (shallow copy)
                        );//int packing=1); (???)

                //printf("Conversion completed\n");
                return osgImg;
        }
        else {
                printf("Unrecognized image type");
                return 0;
        }

}
osg::Image *OSGExporter::LoadResizeSave(string filename,string outname,bool save,int tex_size){

  IplImage *fullimg=NULL;
  bool cached=false;
  if(tex_image_cache.find(filename) == tex_image_cache.end() || !tex_image_cache[filename] ){
    fullimg=cvLoadImage(filename.c_str(),-1);
   
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
  
  osg::Image* image =Convert_OpenCV_TO_OSG_IMAGE(cvImg,!cached);

  if(save){  
    printf("Writing %s\n",outname.c_str());
    osgDB::writeImageFile(*image,outname);
    image->setFileName(outname);
  }
  return image;
}
bool OSGExporter::Export3DS(GtsSurface *s,const char *c3DSFile,map<int,string> material_names,int tex_size){
  char cTemp[512];
  Lib3dsFile *pFile = lib3ds_file_new();
  // std::vector <string> tex_names;
  MaterialToGeometryCollectionMap mtgcm;
  gpointer data[3];
  gint n=0;
  data[0]=&mtgcm;
  data[1] = &n;
  
  gts_surface_foreach_face (s, (GtsFunc) bin_face_mat_osg , data);
  MaterialToGeometryCollectionMap::iterator itr;
  MaterialToIDMap newMatMap;
  int tex_count=0;
  for(itr=mtgcm.begin(); itr!=mtgcm.end(); ++itr){

       // set up texture if needed.
    if (itr->first >= 0 && itr->first < (int)material_names.size()){
      
      std::string filename=prefixdir+material_names[itr->first];
      osg::notify(osg::INFO) << "ctex " << filename  << std::endl;
      //char fname[255];
     char tname[255];
      
     string path(c3DSFile);
     sprintf(tname,"tex-%s-%04d-tex",relative_path(path).c_str(),itr->first);
     string fname(tname);
     newMatMap[itr->first]=fname;
     
     printf("\rLoading and Scaling Texture: %03d/%03d",++tex_count,mtgcm.size());
     if(!tex_saved)
       printf("\n");
  
     LoadResizeSave(filename,"mesh/"+fname+".png", (!tex_saved),tex_size);
   
     Lib3dsMaterial *pMaterial = lib3ds_material_new();
     
     strcpy(pMaterial->name,fname.c_str() );
     
     strcpy(pMaterial->texture1_map.name,(fname+".png").c_str() );
      lib3ds_file_insert_material(pFile, pMaterial);
    }       
  }
  //
  // Create new mesh.
  //
  
  gint uiVertices=gts_surface_vertex_number(s);
  gint uiFaces=gts_surface_face_number(s);
  unsigned int meshNum=0;
  sprintf(cTemp, "Model_%.4u", meshNum);
  Lib3dsMesh *pMesh = lib3ds_mesh_new(cTemp);
  
  if(!lib3ds_mesh_new_point_list(pMesh, uiVertices)){
    lib3ds_mesh_free(pMesh);
  }
  
  if(!lib3ds_mesh_new_texel_list(pMesh, uiVertices)) {
    lib3ds_mesh_free(pMesh);
  }
  
  if(!lib3ds_mesh_new_face_list(pMesh, uiFaces)){
    lib3ds_mesh_free(pMesh);
  }

  //
  // Fill in vertex, texel and face data.
  //
  
   n=0;
  data[0] = pMesh;
  data[1] = &n;
  
  gts_surface_foreach_vertex (s, (GtsFunc) store_vertex_3ds, data);
  gint fcount=0;
  data[1] = &fcount;
  data[2] = &newMatMap;

  gts_surface_foreach_face (s, (GtsFunc) store_face_3ds, data);
  gts_surface_foreach_vertex (s, (GtsFunc) gts_object_reset_reserved, NULL);

  lib3ds_file_insert_mesh(pFile, pMesh);
  add_node(pFile,pMesh);
  add_cameras( pFile);
  bool bResult = lib3ds_file_save(pFile, c3DSFile) == LIB3DS_TRUE;
  lib3ds_file_free(pFile);

  return bResult;
}
static bool find_min_project_dist_bbox(GtsVertex ** triVert,GtsMatrix* back_trans,Camera_Calib *calib, double &dist){

  double vX[3],vY[3];
  GtsPoint tmpP[3];

  
 for(int i=0; i< 3; i++)
   tmpP[i]=triVert[i]->p;
 
 for(int i=0; i < 3; i++){
   gts_point_transform(&tmpP[i],back_trans);
   camera_frame_to_dist_pixel_coords(*calib,tmpP[i].x,tmpP[i].y,tmpP[i].z,
				     vX[i],vY[i]);
 }
 
 
 for(int i=0; i < 3; i++){
   
   if(vX[i]> calib->width || vY[i] > calib->height || vX[i] < 0.0 || vY[i] < 0.0 ){
     return false;
   }
 }
 dist=0.0;
 for(int i=0; i < 3; i++){
   dist+= sqrt(pow((calib->width/2.0)-vX[i],2)+pow((calib->height/2.0)-vY[i],2));
   //((  (vY[i])-(calib->width/2.0))+ (vX[i]-(calib->height/2.0))/2.0) ;
   
   // 1/min(pow(vX[i]-calib->width-(calib->width/2.0),2),
   //	pow(vY[i]-calib->height-(calib->height/2.0),2));
  
 }


 // sqrt(pow((calib->width/2.0)-vX[i],2)+pow((calib->height/2.0)-vY[i],2)); 
       /*  printf("%f(%f) %f(%f) = %f\n",vY[0],
	   (  (vY[0])-(calib->width/2.0)),
	   vX[0],
	   (vX[0]-(calib->height/2.0)),
	   
	   
	   );*/
 //  100000/min(pow(vX[0]-calib->width-(calib->width/2.0),2),
 //pow(vY[0]-calib->height-(calib->height/2.0),2));
 //pow(vY[i]-calib->height-(calib->height/2.0),2))
 /*1/max(pow(vX-calib->width-(calib->width/2.0),2),
   pow(vY-calib->height-(calib->height/2.0),2));*/
 //
 return true;
}

static void set_tex_id_unknown (TVertex *v)
{
  v->id= -1;
}

gdouble dist_to_bbox_center(GtsBBox*bb,GtsPoint *pt){
  GtsPoint center;
  center.x = (bb->x1 + bb->x2) / 2.0f;
  center.y = (bb->y1 + bb->y2) / 2.0f;
  center.z = (bb->z1 + bb->z2) / 2.0f;
  return gts_point_distance(pt,&center);
}
int find_closet_img_trans(GtsTriangle *t,GNode* bboxTree, std::vector<GtsPoint> camPosePts,std::map<int,GtsMatrix *> back_trans,Camera_Calib *calib,int type){
 
  
  if(bboxTree==NULL)
    return 0;
  int index=INT_MAX;
  if(type ==0){
    /*   GtsVertex * v1,* v2,* v3;
    gts_triangle_vertices(t,(GtsVertex **)& v1, 
			  (GtsVertex **)&v2, (GtsVertex **)&v3);
    
    fprintf(errFP,"%f %f %f %f %f %f %f %f %f \n",
	    v1->p.x,v1->p.y,v1->p.z,v2->p.x,v2->p.y,v2->p.z,
	    v3->p.x,v3->p.y,v3->p.z);
	
    GtsPoint tc;
    tc.x=(v1->p.x+v2->p.x+v3->p.x)/3.0;
    tc.y=(v1->p.y+v2->p.y+v3->p.y)/3.0;
    tc.z=(v1->p.z+v2->p.z+v3->p.z)/3.0;
    int bestProjection=INT_MAX;
    int p=lastBP;
    for(p -=2; p < lastBP+2; p++){
      GtsPoint tcT=tc;
      gts_point_transform(&tcT,back_trans[p]);
      double vX,vY;
      tcT=v1->p;
      gts_point_transform(&tcT,back_trans[p]);
      camera_frame_to_dist_pixel_coords(*calib,tcT.x,tcT.y,tcT.z,vX,vY);
      fprintf(errFP,"V1 proj:%d %f %f %f %f %f\n",p,tcT.x,tcT.y,tcT.z,vX,vY);
      tcT=v2->p;
      gts_point_transform(&tcT,back_trans[p]);
      camera_frame_to_dist_pixel_coords(*calib,tcT.x,tcT.y,tcT.z,vX,vY);
      fprintf(errFP,"V2 %d %f %f %f %f %f\n",p,tcT.x,tcT.y,tcT.z,vX,vY);
      tcT=v3->p;
      gts_point_transform(&tcT,back_trans[p]);
      camera_frame_to_dist_pixel_coords(*calib,tcT.x,tcT.y,tcT.z,vX,vY);
      fprintf(errFP,"V3 %d %f %f %f %f %f\n",p,tcT.x,tcT.y,tcT.z,vX,vY);
    }
 GSList * list ,*itr;
    
    
    GtsVertex * v[3];
    gts_triangle_vertices(t,(GtsVertex **)& v[0], 
			    (GtsVertex **)&v[1], (GtsVertex **)&v[2]);
    
    tc.x=(v[0]->p.x+v[1]->p.x+v[2]->p.x)/3.0;
    tc.y=(v[0]->p.y+v[1]->p.y+v[2]->p.y)/3.0;
    tc.z=(v[0]->p.z+v[1]->p.z+v[2]->p.z)/3.0;
   
    bestProjection=INT_MAX;
    int quickMethod=1;

    itr = list =  gts_bb_tree_point_closest_bboxes(bboxTree,&tc);
    double minDist=DBL_MAX;
    double dist=DBL_MAX;
      while (itr) {
	GtsBBox * bbox=GTS_BBOX (itr->data);
	if(!gts_bbox_point_is_inside(bbox,&tc)){
	  itr = itr->next;
	  continue;
	}     
	int val= (int)bbox->bounded;
	if(find_min_project_dist_bbox(v,back_trans[val],calib, dist))
	  fprintf(errFP,"Aleged valid %d\n",val);
	
	itr = itr->next;
      }
      fflush(errFP);
      return INT_MAX;*/
    /*
    for(int i=0; i < (int)camPosePts.size(); i++){
      double dist=gts_point_triangle_distance(&camPosePts[i],t);
      
      GtsVertex * v1,* v2,* v3;
      gts_triangle_vertices(t,(GtsVertex **)& v1, 
			    (GtsVertex **)&v2, (GtsVertex **)&v3);
          printf("%f %f %f %f %f %f dist: %f\n",//transP.x,transP.y,transP.z,
       v1->p.x,v1->p.y,v1->p.z, dist);
      
      if(dist < minDist){
	minDist=dist;
	index=i;
      }
      }*/
  }else{
    GSList * list ,*itr;
    
    
    GtsVertex * v[3];
    gts_triangle_vertices(t,(GtsVertex **)& v[0], 
			    (GtsVertex **)&v[1], (GtsVertex **)&v[2]);
    GtsPoint tc;
    tc.x=(v[0]->p.x+v[1]->p.x+v[2]->p.x)/3.0;
    tc.y=(v[0]->p.y+v[1]->p.y+v[2]->p.y)/3.0;
    tc.z=(v[0]->p.z+v[1]->p.z+v[2]->p.z)/3.0;
   
    int bestProjection=INT_MAX;

    itr = list =  gts_bb_tree_point_closest_bboxes(bboxTree,&tc);
    double minDist=DBL_MAX;
    double dist=DBL_MAX;
    
    while (itr) {
      GtsBBox * bbox=GTS_BBOX (itr->data);
      if(!gts_bbox_point_is_inside(bbox,&tc)){
	  itr = itr->next;
	  continue;
      }     
      int val= (int)bbox->bounded;
      if(find_min_project_dist_bbox(v,back_trans[val],calib, dist)){
	if(dist < minDist){
	  minDist=dist;
	 bestProjection=val;
	}
      }
      itr = itr->next;
    }
    g_slist_free (list);   
    
    //Brute force quick method failed
    if(bestProjection == INT_MAX){
      minDist=DBL_MAX;
      for(int i =0; i < (int)bboxes_all.size(); i++){
	int val= (int)bboxes_all[i]->bounded;
	if(find_min_project_dist_bbox(v,back_trans[val],calib, dist)){
	  if(dist < minDist){
	    minDist=dist;
	    bestProjection=val;
	  }
	}
      } 
    }
     
    return bestProjection;
  }
  
  return index;

}



typedef struct _texGenData{
  GNode *bboxTree;
  std::vector<GtsPoint> camPosePts;
  
  
  int tex_size;
  std::map<int,GtsMatrix *> back_trans;
  int validCount;
  int count;
  int reject;
  int total;
std::vector<T_Face *> *border_faces;
  Camera_Calib *calib;
}texGenData;

#ifdef USE_SURFACE_BTREE
static gint foreach_face (GtsFace * f, 
			  gpointer t_data,
			  gpointer * info)
#else /* not USE_SURFACE_BTREE */
static void foreach_face (GtsFace * f, 
			  gpointer t_data,
			  gpointer * info)
#endif /* not USE_SURFACE_BTREE */
{
  (*((GtsFunc) info[0])) (f, info[1]);
#ifdef USE_SURFACE_BTREE
  return FALSE;
#endif /* USE_SURFACE_BTREE */
}
struct _GHashNode
{
  gpointer   key;
  gpointer   value;
  GHashNode *next;
};

struct _GHashTable
{
  gint             size;
  gint             nnodes;
  GHashNode      **nodes;
  GHashFunc        hash_func;
  GEqualFunc       key_equal_func;
  GDestroyNotify   key_destroy_func;
  GDestroyNotify   value_destroy_func;
};
struct threaded_hash_exec
{
  threaded_hash_exec(int rangeLow,int rangeHi, GHashTable *hash_table,GHFunc func, gpointer	  user_data) : rangeLow(rangeLow),rangeHi(rangeHi),hash_table(hash_table), func(func),user_data(user_data) { }
   void operator()()
   {
     GHashNode *node;
     for (int i = rangeLow; i < rangeHi; i++)
       for (node = hash_table->nodes[i]; node; node = node->next)
	 (* func) (node->key, node->value, user_data);

   }

  
  int rangeLow,rangeHi;
  GHashTable *hash_table;

  GHFunc	  func;
  gpointer	  user_data;
};



void
threaded_hash_table_foreach (GHashTable *hash_table,
		      GHFunc	  func,
			     gpointer	  user_data,int num_threads)
{
    
  g_return_if_fail (hash_table != NULL);
  g_return_if_fail (func != NULL);
  int task_size=(int)ceil(hash_table->size /(double)num_threads);
  boost::thread_group thread_gr;
  threaded_hash_exec *the[num_threads];
 
  for(int i=0; i < num_threads; i++)
    the[i] = new threaded_hash_exec(i*task_size,min((i+1)*task_size,hash_table->size),hash_table,func,user_data);

  for(int i=0; i < num_threads; i++)
    thread_gr.create_thread(*the[i]);
  
 thread_gr.join_all();
}
void gts_write_triangle (GtsTriangle * t, 
			 GtsPoint * o,
			 FILE * fptr)
{
  gdouble xo = o ? o->x : 0.0;
  gdouble yo = o ? o->y : 0.0;
  gdouble zo = o ? o->z : 0.0;

  g_return_if_fail (t != NULL && fptr != NULL);

 
  fprintf (fptr, "OFF 3 1 0\n"
	   "%g %g %g\n%g %g %g\n%g %g %g\n3 0 1 2\n})\n",
	   GTS_POINT (GTS_SEGMENT (t->e1)->v1)->x - xo, 
	   GTS_POINT (GTS_SEGMENT (t->e1)->v1)->y - yo,
	   GTS_POINT (GTS_SEGMENT (t->e1)->v1)->z - zo,
	   GTS_POINT (GTS_SEGMENT (t->e1)->v2)->x - xo, 
	   GTS_POINT (GTS_SEGMENT (t->e1)->v2)->y - yo, 
	   GTS_POINT (GTS_SEGMENT (t->e1)->v2)->z - zo,
	   GTS_POINT (gts_triangle_vertex (t))->x - xo,
	   GTS_POINT (gts_triangle_vertex (t))->y - yo,
	   GTS_POINT (gts_triangle_vertex (t))->z - zo);
	 
}
/**
 * threaded_surface_foreach_face:
 * @s: a #GtsSurface.
 * @func: a #GtsFunc.
 * @data: user data to be passed to @func.
 *
 * Calls @func once for each face of @s.
 */
void threaded_surface_foreach_face (GtsSurface * s,
			       GtsFunc func, 
				    gpointer data,int num_threads)
{
  gpointer info[2];

  g_return_if_fail (s != NULL);
  g_return_if_fail (func != NULL);

  /* forbid removal of faces */
  s->keep_faces = TRUE;
  info[0] = (void *)func;
  info[1] = data;
#ifdef USE_SURFACE_BTREE
  g_tree_traverse (s->faces, (GTraverseFunc) foreach_face, G_IN_ORDER,
		   info);
#else /* not USE_SURFACE_BTREE */
  threaded_hash_table_foreach (s->faces, (GHFunc) foreach_face, info,num_threads);
#endif /* not USE_SURFACE_BTREE */
  /* allow removal of faces */
  s->keep_faces = FALSE;
}

static void texcoord_foreach_face (T_Face * f,
				   texGenData *data)
{
  
  int indexClosest=find_closet_img_trans(&GTS_FACE(f)->triangle,
					 data->bboxTree,data->camPosePts,data->back_trans,data->calib,1);
  if(indexClosest == INT_MAX){
    /*fprintf(errFP,"Failed traingle\n");
    gts_write_triangle(&GTS_FACE(f)->triangle,NULL,errFP);
    fflush(errFP);*/
    libsnapper::tex_add_verbose(data->count++,data->total,data->reject++);
    return;
  }
  
  
  
  if(apply_tex_to_tri(f,data->calib,data->back_trans[indexClosest],indexClosest,data->tex_size))
    data->validCount++;
    else{
      printf("Index closest %d\n",indexClosest);
       find_closet_img_trans(&GTS_FACE(f)->triangle,
	 data->bboxTree,data->camPosePts,data->back_trans,data->calib,0);
    }
    // if(!data->bboxTree)
    tex_add_verbose(data->count++,data->total,data->reject);

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

void gen_mesh_tex_coord(GtsSurface *s ,Camera_Calib *calib, std::map<int,GtsMatrix *> back_trans,GNode * bboxTree,int tex_size, int num_threads){
  
  //errFP=fopen("err.txt","w");
  std::vector<GtsPoint> camPosePts;
  GtsPoint transP;
  printf("Size %d\n",back_trans.size());
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
  
  
  tex_data.back_trans=back_trans;
  tex_data.validCount=0;
  tex_data.tex_size=tex_size;
  tex_data.count=1;
  tex_data.reject=0;
  tex_data.total=gts_surface_face_number(s);
  tex_data.calib=calib;
  tex_data.border_faces = &border_faces;
  if(num_threads > 1)
    threaded_surface_foreach_face (s, (GtsFunc)texcoord_foreach_face ,
				   &tex_data ,num_threads);
else
  gts_surface_foreach_face (s, (GtsFunc)texcoord_foreach_face ,&tex_data);

  printf("\nChecking weird border cases...\n");
  if(num_threads > 1)
    threaded_surface_foreach_face (s, (GtsFunc)findborder_foreach_face ,
				   &tex_data ,num_threads);   
  else
    gts_surface_foreach_face (s, (GtsFunc)findborder_foreach_face ,&tex_data );
  tex_data.count=1;
  tex_data.reject=0;
  tex_data.total =border_faces.size();
  for(int i=0; i < (int) border_faces.size(); i++){
   T_Face *f2 = copy_face(border_faces[i],s);
   int idx=find_closet_img_trans(&GTS_FACE(f2)->triangle,bboxTree,camPosePts,back_trans,calib,1);
    if(apply_tex_to_tri(f2,calib,back_trans[idx],idx,tex_size))
      tex_data.validCount++;
    
    gts_surface_remove_face(s,GTS_FACE(border_faces[i]));
    tex_add_verbose(tex_data.count++,tex_data.total,tex_data.reject);
    }

  printf("\nValid tex %d\n", tex_data.validCount);
 
  
  
}

OSGExporter::~OSGExporter(){
  map<string, IplImage *>::const_iterator itr;
  for(itr = tex_image_cache.begin(); itr != tex_image_cache.end(); ++itr){

    IplImage *tmp=(*itr).second;
    cvReleaseImage(&tmp);
    
  }
  /*
  for(size_t i=0; i < osg_tex_ptrs.size(); i++){
    if(osg_tex_ptrs[i].valid()){
      if(compress_tex){
	osg_tex_ptrs[i]->setUnRefImageDataAfterApply(true);
	osg_tex_ptrs[i]->dirtyTextureObject();
	osg_tex_ptrs[i]->apply(*state);
      }
    }
    }*/
  tex_image_cache.clear();

  //  osg_tex_ptrs.clear();
}
#endif
