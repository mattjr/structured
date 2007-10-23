#include "OSGExport.h"
#include <osgUtil/TriStripVisitor>
#include <osgUtil/SmoothingVisitor>
#include <osg/GraphicsContext>
#include <osgDB/WriteFile>
#include <cv.h>
#include <highgui.h>

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



osg::Geode* OSGExporter::convertGtsSurfListToGeometry(GtsSurface *s, std::vector<string> textures) 
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
   
       if (itr->first >= 0 && itr->first < (int)textures.size()){
	 
	 std::string filename=prefixdir+textures[itr->first];
	 osg::notify(osg::INFO) << "ctex " << filename  << std::endl;
	 char fname[255];
	 sprintf(fname,"mesh/tex-%04d.png",itr->first);

	 if(!context){
	   printf("Can't use OPENGL without valid context");
	   exit(0);
	 }
	 printf("\rLoading and Scaling Texture: %03d/%03d",tex_count++,mtgcm.size());
	 fflush(stdout); 
	 IplImage *cvImg=NULL;
	 osg::ref_ptr<osg::Image> image= LoadResizeSave(filename,fname, (!ive_out),cvImg);
	 if (image.get()){	     
	   // create state
	   osg::StateSet* stateset = new osg::StateSet;
	   
	     // create texture
	   osg::Texture2D* texture = new osg::Texture2D;
	   texture->setUnRefImageDataAfterApply( true );
	   texture->setImage(image.get());
	   stateset->setTextureAttributeAndModes(0,texture,
						 osg::StateAttribute::ON);
	   gc._texturesActive=true;
	   
	   
	   gc._geom->setStateSet(stateset);
	     
	   osg::Vec2Array* texcoordArray = new osg::Vec2Array(gc._numPoints);
	   gc._texcoords = texcoordArray->begin();
	   gc._geom->setTexCoordArray(0,texcoordArray);
	   
	   if(compress_tex)
	     compress(texture);
	   else{
	     if(!state)
	       state = new osg::State;
	     texture->apply(*state);
	   }
	   if(cvImg)
	     cvReleaseImage(&cvImg);
	 }
       }
     }
   }
   
   printf("\n");
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
  
  if(ext == "3ds"){
    Export3DS(s,fileNameOut.c_str(),textures);
    return true;
  }else if(!ive_out && compress_tex){
    std::cout<<"Warning: compressing texture only supported when out";
    std::cout << "puting to .ive"<<std::endl;
    compress_tex=false;
  }    


  
  osg::Group* root = new osg::Group;
  osg::Geode* geode = convertGtsSurfListToGeometry(s,textures);
  geode->setName(fileNameOut);
  root->addChild(geode);

  osgDB::ReaderWriter::WriteResult result = osgDB::Registry::instance()->writeNode(*root,fileNameOut,osgDB::Registry::instance()->getOptions());
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

osg::Image* Convert_OpenCV_TO_OSG_IMAGE(IplImage* cvImg){

        if(cvImg->nChannels == 3)
        {
                // Flip image from top-left to bottom-left origin
                if(cvImg->origin == 0) {
                        cvConvertImage(cvImg , cvImg, CV_CVTIMG_FLIP);
                        cvImg->origin = 1;
                }

                // Convert from BGR to RGB color format
                //printf("Color format %s\n",cvImg->colorModel);
                cvCvtColor( cvImg, cvImg, CV_BGR2RGB );

                osg::Image* osgImg = new osg::Image();

                osgImg->setImage(
                        cvImg->width, //s
                        cvImg->height, //t
                        3, //r
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
osg::Image *OSGExporter::LoadResizeSave(string filename,string outname,bool save,IplImage *cvImg){

  
  
  IplImage *fullimg=cvLoadImage(filename.c_str(),-1);
  cvImg=cvCreateImage(cvSize(tex_size,tex_size),
				     IPL_DEPTH_8U,3);
  
  if(fullimg){
    cvResize(fullimg,cvImg);
    cvReleaseImage(&fullimg);
  }
  else {
    printf("Failed to load %s\n",filename.c_str());
    cvReleaseImage(&cvImg);
    return NULL;
  }
  osg::Image* image =Convert_OpenCV_TO_OSG_IMAGE(cvImg);
  
  if(save){  
    printf("Writing %s\n",outname.c_str());
    osgDB::writeImageFile(*image,outname);
    image->setFileName(outname);
  }
  return image;
}
bool OSGExporter::Export3DS(GtsSurface *s,const char *c3DSFile,vector<string> material_names){
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
     
     printf("\rLoading and Scaling Texture: %03d/%03d",tex_count++,mtgcm.size());
     IplImage *cvImg=NULL;
     LoadResizeSave(filename,"mesh/"+fname+".png", (!tex_saved),cvImg);
     if(cvImg)
       cvReleaseImage(&cvImg);
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

static void set_tex_id_unknown (TVertex *v)
{
  v->id= -1;
}

static int currentBBox=0;

int find_closet_img_trans(GtsTriangle *t,std::vector<GtsBBox *> bboxes, std::vector<GtsPoint> camPosePts,int type){

  double minDist=DBL_MAX;
  
  int index=INT_MAX;
  if(type ==0){
    for(int i=0; i < (int)camPosePts.size(); i++){
      double dist=gts_point_triangle_distance(&camPosePts[i],t);
      
      GtsVertex * v1,* v2,* v3;
      gts_triangle_vertices(t,(GtsVertex **)& v1, 
			    (GtsVertex **)&v2, (GtsVertex **)&v3);
      //    printf("%f %f %f %f %f %f dist: %f\n",//transP.x,transP.y,transP.z,
      //   v1->p.x,v1->p.y,v1->p.z, dist);
      
      if(dist < minDist){
	minDist=dist;
	index=i;
      }
    }
  }else{
    if(currentBBox == -1){
      for(int i=0; i < (int)bboxes.size(); i++){
	if(gts_bbox_overlaps_triangle(bboxes[i],t)){
	  index=i;
	  currentBBox = i;
	  break;
	}
      }
    }else{
      if(gts_bbox_overlaps_triangle(bboxes[currentBBox],t))
	return currentBBox;
      else{
	int i;
	for(i=currentBBox+1; i < (int)bboxes.size(); i++){
	   if(gts_bbox_overlaps_triangle(bboxes[i],t))
	     break;
	 }

	for(; i < (int)bboxes.size(); i++){
	   if(!gts_bbox_overlaps_triangle(bboxes[i],t))
	     break;
	 }
	 return i-1;
	 

      }
	
    }
  }
  return index;

}

gboolean tex_add_verbose ( guint number, guint total)
{
  static guint nmax = 0, nold = 0;
  static GTimer * timer = NULL, * total_timer = NULL;



  if (timer == NULL) {
    nmax = nold = number;
    timer = g_timer_new ();
    total_timer = g_timer_new ();
    g_timer_start (total_timer);
  }

  if (number != nold){// && number % 1 == 0 ){//&& number < nmax && nmax > total) {
    gdouble total_elapsed = g_timer_elapsed (total_timer, NULL);
    gdouble remaining;
    gdouble hours, mins, secs;
    gdouble hours1, mins1, secs1;

    g_timer_stop (timer);

    hours = floor (total_elapsed/3600.);
    mins = floor ((total_elapsed - 3600.*hours)/60.);
    secs = floor (total_elapsed - 3600.*hours - 60.*mins);

    remaining = ((total_elapsed/(gdouble)number) *((gdouble)total-number));
    hours1 = floor (remaining/3600.);
    mins1 = floor ((remaining - 3600.*hours1)/60.);
    secs1 = floor (remaining - 3600.*hours1 - 60.*mins1);

    fprintf (stderr, 
	     "\rFaces: %4u/%4u %3.0f%% %6.0f edges/s "
	     "Elapsed: %02.0f:%02.0f:%02.0f "
	     "Remaining: %02.0f:%02.0f:%02.0f ",
	     number, total,
	     100.*( number)/( total),
	     (number - nold  )/g_timer_elapsed (timer, NULL),
	     hours, mins, secs,
	     hours1, mins1, secs1);
    fflush (stderr);

    nold = number;
    g_timer_start (timer);
  }
  if (number == total) {
    g_timer_destroy (timer);
    g_timer_destroy (total_timer);
    timer =NULL;
    printf("\n");
    return TRUE;
  }
  return FALSE;
}
typedef struct _texGenData{
 std::vector<GtsBBox *> bboxes;
  std::vector<GtsPoint> camPosePts;
  vector<int> *tex_used;
  int oldIndex;
  int tex_size;
  std::vector<GtsMatrix *> back_trans;
  int validCount;
  int count;
  int total;
std::vector<T_Face *> *border_faces;
  Camera_Calib *calib;
}texGenData;

static void texcoord_foreach_face (T_Face * f,
				   texGenData *data)
{
  
  int indexClosest=find_closet_img_trans(&GTS_FACE(f)->triangle,
					 data->bboxes,data->camPosePts,0);
  if(indexClosest == INT_MAX){
      printf("Can't find tex in range\n");
  }
     data->tex_used->push_back(indexClosest);
    
    if(indexClosest !=data->oldIndex){
      data->oldIndex=indexClosest;
    }

    if(apply_tex_to_tri(f,data->calib,data->back_trans[indexClosest],indexClosest,data->tex_size))
      data->validCount++;
     else
       ;//gts_surface_remove_face(s,GTS_FACE(f));
    tex_add_verbose(data->count++,data->total);

}
static void findborder_foreach_face (T_Face * f,
				   texGenData *data)
{
   GtsTriangle * t = &GTS_FACE(f)->triangle;
    TVertex * v1,* v2,* v3; 
    gts_triangle_vertices(t,(GtsVertex **)& v1, 
			  (GtsVertex **)&v2, (GtsVertex **)&v3);
    if(f->material != v1->id || f->material != v2->id || f->material != v3->id)
      data->border_faces->push_back(f);
}

std::vector<int> gen_mesh_tex_coord(GtsSurface *s ,Camera_Calib *calib, std::vector<GtsMatrix *> back_trans,std::vector<GtsBBox *> bboxes,int tex_size){
  

  std::vector<GtsPoint> camPosePts;
  GtsPoint transP;
  for(int i=0; i < (int)back_trans.size(); i++){
      GtsMatrix *m= gts_matrix_inverse(back_trans[i]); 
      transP.x=m[0][3];
      transP.y=m[1][3];
      transP.z=m[2][3];
      camPosePts.push_back(transP);
      gts_matrix_destroy(m);
  }

  vector<int> tex_used;
  gts_surface_foreach_vertex(s,(GtsFunc)set_tex_id_unknown,NULL);

  std::vector<T_Face *> border_faces;

  texGenData tex_data;
  tex_data.bboxes=bboxes;
  tex_data.camPosePts =camPosePts;
  tex_data.tex_used = &tex_used;
  tex_data.oldIndex=INT_MAX;
  tex_data.back_trans=back_trans;
  tex_data.validCount=0;
  tex_data.tex_size=tex_size;
  tex_data.count=1;
  tex_data.total=gts_surface_face_number(s);
  tex_data.calib=calib;
  tex_data.border_faces = &border_faces;
  gts_surface_foreach_face (s, (GtsFunc)texcoord_foreach_face ,&tex_data );
  /*
  while ((f =(T_Face *) gts_surface_traverse_next (t, &level))) {
    int indexClosest=find_closet_img_trans(&GTS_FACE(f)->triangle,
					  bboxes,camPosePts,0);
    if(indexClosest == INT_MAX){
      printf("Can't find tex in range\n");
      continue;
     
    }

    tex_used.push_back(indexClosest);
    
    if(indexClosest !=oldIndex){
      oldIndex=indexClosest;
    }

    if(apply_tex_to_tri(f,calib,back_trans[indexClosest],indexClosest,tex_size))
      validCount++;
     else
       ;//gts_surface_remove_face(s,GTS_FACE(f));
    tex_add_verbose(count++,total);
  }
 
  gts_surface_traverse_destroy (t);
  */
  #warning "Does work when faces are rmoved when not valid tex fixme"
  /*  gts_surface_foreach_face (s, (GtsFunc) pick_first_face,&first );
  t = gts_surface_traverse_new (s, first);
  printf("Checking weird border cases...\n");
  std::vector<T_Face *> border_faces;
  while ((f =(T_Face *) gts_surface_traverse_next (t, &level))) {
   GtsTriangle * t = &GTS_FACE(f)->triangle;
    TVertex * v1,* v2,* v3; 
    gts_triangle_vertices(t,(GtsVertex **)& v1, 
			  (GtsVertex **)&v2, (GtsVertex **)&v3);
    if(f->material != v1->id || f->material != v2->id || f->material != v3->id)
      border_faces.push_back(f);

  }

  */

  printf("Checking weird border cases...\n");
  gts_surface_foreach_face (s, (GtsFunc)findborder_foreach_face ,&tex_data );

  tex_data.count=1;
  tex_data.total =border_faces.size();
  for(int i=0; i < (int) border_faces.size(); i++){
   T_Face *f2 = copy_face(border_faces[i],s);
   int idx=find_closet_img_trans(&GTS_FACE(f2)->triangle,bboxes,camPosePts,0);
    if(apply_tex_to_tri(f2,calib,back_trans[idx],idx,tex_size))
      tex_data.validCount++;
    
    gts_surface_remove_face(s,GTS_FACE(border_faces[i]));
    tex_add_verbose(tex_data.count++,tex_data.total);
  }

  printf("Valid tex %d\n", tex_data.validCount);
 
  sort( tex_used.begin(), tex_used.end() );
  tex_used.erase( unique( tex_used.begin(), tex_used.end() ), tex_used.end() );
  return tex_used;
}
#endif
