#include "OSGExport.h"
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
bool OSGExporter::Export3DS(GtsSurface *s,const char *c3DSFile,map<int,string> material_names,int tex_size,VerboseMeshFunc vmcallback){
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
      sprintf(tname,"%s-%04d-tex",osgDB::getStrippedName(path).c_str(),itr->first);
      string fname(tname);
      newMatMap[itr->first]=fname;
      if(vmcallback)
	vmcallback(++tex_count,mtgcm.size());
      //printf("\rLoading and Scaling Texture: %03d/%03d",++tex_count,mtgcm.size());
    
  
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
#endif
