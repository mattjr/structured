#include "OSGExport.h"
#include <osgUtil/TriStripVisitor>
#include <osgUtil/SmoothingVisitor>
#include <osg/GraphicsContext>
#include <osgDB/WriteFile>


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
 

  (*gc._vertices++).set(GTS_VERTEX(v1)->p.x,GTS_VERTEX(v1)->p.y,GTS_VERTEX(v1)->p.z);
  (*gc._vertices++).set(GTS_VERTEX(v2)->p.x,GTS_VERTEX(v2)->p.y,GTS_VERTEX(v2)->p.z);
  (*gc._vertices++).set(GTS_VERTEX(v3)->p.x,GTS_VERTEX(v3)->p.y,GTS_VERTEX(v3)->p.z); 
  // printf("%f %f %f\n",GTS_VERTEX(v3)->p.x,GTS_VERTEX(v3)->p.y,GTS_VERTEX(v3)->p.z);
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
  if(ext == "3ds"){
    Export3DS(s,fileNameOut.c_str(),textures);
    return true;
  }else if(ext != "ive" && compress_tex){
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
  for(itr=mtgcm.begin(); itr!=mtgcm.end(); ++itr){

       // set up texture if needed.
    if (itr->first >= 0 && itr->first < (int)material_names.size()){
      
      std::string filename=prefixdir+material_names[itr->first];
      osg::notify(osg::INFO) << "ctex " << filename  << std::endl;
      //char fname[255];
     char tname[255];
      
   
      sprintf(tname,"tex-%04d-tex",itr->first);
      string fname(tname);
      newMatMap[itr->first]=fname;

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
	  osgDB::writeImageFile(*image,"mesh/"+fname+".png");
	  image->setFileName("mesh/"+fname+".png");
	}
      }
      
      
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
#endif
