//!
//! \file auv_mesh.cpp
//!
//!Meshing functions
//! \author Ian Mahon
//!

#include <libsnapper/auv_stereo_geometry.hpp>

#include <iomanip>
#include <fstream>
#include <stack>
#include "KDtree.h"
#include "TriMesh_algo.h"
#include "mesh_proc.hpp"

#include <boost/version.hpp>
#include <math.h>
using namespace libsnapper;
using namespace libplankton;


using namespace std;


namespace mesh_proc
{
guint nmax = 0, nold = 0;
GTimer * timer = NULL, * total_timer = NULL;		       
boost::once_flag once = BOOST_ONCE_INIT;
boost::once_flag once2 = BOOST_ONCE_INIT;
boost::once_flag once3 = BOOST_ONCE_INIT;
//maps [0,1] --> (r,g,b) where blue is close to 0 and red is close to 1.
void jet_color_map(const float& val,float &r,float &g, float &b) {
  
  if(val > 1.0 || val < 0.0)
    r=b=g=0.0;
  
  r = std::min(4.0f * val - 1.5f,-4.0f * val + 4.5f) ;
  g = std::min(4.0f * val - 0.5f,-4.0f * val + 3.5f) ;
  b = std::min(4.0f * val + 0.5f,-4.0f * val + 2.5f) ;
  
  
  r = clamp(r, 0.0f, 1.0f);
  g = clamp(g, 0.0f, 1.0f);
  b = clamp(b, 0.0f, 1.0f);
  
  
}

// Remove edge longer then thresh
void edge_len_thresh(TriMesh *mesh,double thresh)
{
  mesh->need_faces();
  int numfaces = mesh->faces.size();


  vector<bool> toremove(numfaces, false);
  for (int i = 0; i < numfaces; i++) {
    const point &v0 = mesh->vertices[mesh->faces[i][0]];
    const point &v1 = mesh->vertices[mesh->faces[i][1]];
    const point &v2 = mesh->vertices[mesh->faces[i][2]];
    float d01 = dist2(v0, v1);
    float d12 = dist2(v1, v2);
    float d20 = dist2(v2, v0);
    if (d01 > thresh || d12 > thresh || d20 > thresh)
      toremove[i] = true;
  }
  remove_faces(mesh, toremove);
  remove_unused_vertices(mesh);
}

// Remove edge longer then thresh
void edge_len_thresh_percent(TriMesh *mesh,double thresh)
{
  double sum=0.0;
  mesh->need_faces();
  int numfaces = mesh->faces.size();
  for (int i = 0; i < numfaces; i++) {
    const point &v0 = mesh->vertices[mesh->faces[i][0]];
    const point &v1 = mesh->vertices[mesh->faces[i][1]];
    const point &v2 = mesh->vertices[mesh->faces[i][2]];
    sum+= dist2(v0, v1);
    sum += dist2(v1, v2);
    sum+= dist2(v2, v0);
  }
  sum/=numfaces;
  double threshlen=sum * thresh;
  //printf("Avg len %f Thresh %f\n",sum,threshlen);
  vector<bool> toremove(numfaces, false);
  for (int i = 0; i < numfaces; i++) {
    const point &v0 = mesh->vertices[mesh->faces[i][0]];
    const point &v1 = mesh->vertices[mesh->faces[i][1]];
    const point &v2 = mesh->vertices[mesh->faces[i][2]];
    float d01 = dist2(v0, v1);
    float d12 = dist2(v1, v2);
    float d20 = dist2(v2, v0);
    if (d01 > threshlen || d12 > threshlen || d20 > threshlen)
      toremove[i] = true;
  }
  remove_faces(mesh, toremove);
  remove_unused_vertices(mesh);
}

T_Face *copy_face (T_Face * t,
			       GtsSurface * s)
{
 
  TVertex *n1,*n2,*n3;
  GtsEdge * e1,*e2,*e3;
  TVertex * v1,* v2,* v3;
 
  gts_triangle_vertices(&GTS_FACE(t)->triangle,(GtsVertex **)& v1, 
			  (GtsVertex **)&v2, (GtsVertex **)&v3);
  
  
  n1=(TVertex*)gts_vertex_new (t_vertex_class(), GTS_VERTEX(v1)->p.x, GTS_VERTEX(v1)->p.y,
		     GTS_VERTEX(v1)->p.z);
  n2=(TVertex*)gts_vertex_new (t_vertex_class(), GTS_VERTEX(v2)->p.x, GTS_VERTEX(v2)->p.y,
		     GTS_VERTEX(v2)->p.z);
  n3=(TVertex*)gts_vertex_new (t_vertex_class(), GTS_VERTEX(v3)->p.x, GTS_VERTEX(v3)->p.y,
		     GTS_VERTEX(v3)->p.z);

  n1->plane=v1->plane;
  n2->plane=v2->plane;
  n3->plane=v3->plane;

  e1=gts_edge_new (gts_edge_class(),  GTS_VERTEX(n1),  GTS_VERTEX(n2));
  e2=gts_edge_new (gts_edge_class(),  GTS_VERTEX(n2),  GTS_VERTEX(n3));
  e3=gts_edge_new (gts_edge_class(),  GTS_VERTEX(n3),  GTS_VERTEX(n1));
  
  T_Face * f2=(T_Face *)gts_face_new (s->face_class,
			  e1,e2,e3);
  gts_surface_add_face (s, GTS_FACE(f2));
					
  return f2;
 
}

/*GtsMatrix *get_sensor_to_world_trans(Vector veh_pose, Vector sensor_pose,GtsMatrix *trans){
  if(!trans)
    trans=gts_matrix_identity (NULL);
  Matrix T_sensor_nav(4,4);
  Matrix T_nav_world(4,4);
  Matrix T_sensor_world(4,4);
  Matrix T_world_sensor(4,4);

  T_sensor_nav.clear();
  T_nav_world.clear();
  T_sensor_world.clear();
  T_world_sensor.clear();
 
  double slam_x = veh_pose[AUV_POSE_INDEX_X];
  double slam_y = veh_pose[AUV_POSE_INDEX_Y];
  double slam_z = veh_pose[AUV_POSE_INDEX_Z];
  double slam_phi   =veh_pose[AUV_POSE_INDEX_PHI];
  double slam_theta =veh_pose[AUV_POSE_INDEX_THETA];
  double slam_psi   = veh_pose[AUV_POSE_INDEX_PSI];
  Matrix slam_R =libplankton::rot_mat_back( slam_phi, slam_theta, slam_psi );

  double sensor_x = sensor_pose[AUV_POSE_INDEX_X];
  double sensor_y = sensor_pose[AUV_POSE_INDEX_Y];
  double sensor_z = sensor_pose[AUV_POSE_INDEX_Z];
  double sensor_phi = sensor_pose[AUV_POSE_INDEX_PHI];
  double sensor_theta = sensor_pose[AUV_POSE_INDEX_THETA];
  double sensor_psi = sensor_pose[AUV_POSE_INDEX_PSI];
  Matrix sensor_nav_R = libplankton::rot_mat_back( sensor_phi, 
						      sensor_theta,
						      sensor_psi);
 
  
  sub_matrix(T_sensor_nav, 0, 3, 0, 3) = sensor_nav_R;
  T_sensor_nav(0, 3) = sensor_x;
  T_sensor_nav(1, 3) = sensor_y;
  T_sensor_nav(2, 3) = sensor_z;
  T_sensor_nav(3, 3) = 1;

  // std::cout<<std::setprecision(3)<< " T_sensor_nav" << T_sensor_nav <<"\n";
  
  
	
  sub_matrix(T_nav_world, 0, 3, 0, 3) = slam_R;
  T_nav_world(0, 3) = slam_x;
  T_nav_world(1, 3) = slam_y;
  T_nav_world(2, 3) = slam_z;
  T_nav_world(3, 3) = 1;

  T_sensor_world = prod(T_nav_world,T_sensor_nav);
  for(int i=0; i <4; i++)
    for(int j=0; j<4; j++)
      trans[i][j]=T_sensor_world(i,j);
  return trans;
}

void fill_gts_matrix(double *cam_pose,GtsMatrix *&trans){
  if(!trans)
    trans=gts_matrix_identity (NULL);
 
  Matrix T_cam_world(4,4);
  
  T_cam_world.clear();
   
  double slam_x = cam_pose[AUV_POSE_INDEX_X];
  double slam_y = cam_pose[AUV_POSE_INDEX_Y];
  double slam_z = cam_pose[AUV_POSE_INDEX_Z];
  double slam_phi   =cam_pose[AUV_POSE_INDEX_PHI];
  double slam_theta =cam_pose[AUV_POSE_INDEX_THETA];
  double slam_psi   = cam_pose[AUV_POSE_INDEX_PSI];
  Matrix slam_R =libplankton::rot_mat_baget_sensor_to_world_transck( slam_phi, slam_theta, slam_psi );

  sub_matrix(T_cam_world, 0, 3, 0, 3) = slam_R;
  T_cam_world(0, 3) = slam_x;
  T_cam_world(1, 3) = slam_y;
  T_cam_world(2, 3) = slam_z;
  T_cam_world(3, 3) = 1;


  for(int i=0; i <4; i++)
    for(int j=0; j<4; j++)
      trans[i][j]=T_cam_world(i,j);
}
*/
#if 0
void timer_init(){

  timer = g_timer_new ();
  total_timer = g_timer_new ();
  g_timer_start (total_timer);
}
void timer_destroy(){
  g_timer_destroy (timer);
  g_timer_destroy (total_timer);
  timer =NULL;

}

gboolean tex_add_verbose ( guint number, guint total, int reject)
{
  
  if (timer == NULL) { 
    boost::call_once(&timer_init, once);
    nmax = nold = number;
  }
  
  if (number != nold && number % 121 == 0 ){// && number % 1 == 0 ){//&& number < nmax && nmax > total) {
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
	     "\rFace: %7u/%7u %3.0f%% %2.1fk face/s "
	     "Elap: %02.0f:%02.0f:%02.0f "
	     "Rem: %02.0f:%02.0f:%02.0f Rej %2u",
	     number, total,
	     100.*( number)/( total),
	     ((number - nold  )/g_timer_elapsed (timer, NULL))/1000.0,
	     hours, mins, secs,
	     hours1, mins1, secs1,reject);
    fflush (stderr);

    nold = number;
    g_timer_start (timer);
  }
  if (number == total) {
    boost::call_once(&timer_destroy, once2);
 #if (BOOST_VERSION / 100 % 1000)  < 35
    once = BOOST_ONCE_INIT;
    once2 = BOOST_ONCE_INIT;
 #else
    once.epoch = BOOST_ONCE_INITIAL_FLAG_VALUE;
    once2.epoch = BOOST_ONCE_INITIAL_FLAG_VALUE;
 #endif
 
    return TRUE;
  }
  return FALSE;
}
gboolean stop_number_verbose (gdouble cost, guint number, guint * min)
{
  static guint nmax = 0, nold = 0;
  static GTimer * timer = NULL, * total_timer = NULL;


  g_return_val_if_fail (min != NULL, TRUE);

  if (timer == NULL) { 
   timer = g_timer_new ();
   total_timer = g_timer_new ();
   g_timer_start (total_timer);
   
    nmax = nold = number;
  }

 

  if (number != nold && number % 121 == 0 &&
      number < nmax && nmax > *min) {
    gdouble total_elapsed = g_timer_elapsed (total_timer, NULL);
    gdouble remaining;
    gdouble hours, mins, secs;
    gdouble hours1, mins1, secs1;

    g_timer_stop (timer);

    hours = floor (total_elapsed/3600.);
    mins = floor ((total_elapsed - 3600.*hours)/60.);
    secs = floor (total_elapsed - 3600.*hours - 60.*mins);

    remaining = total_elapsed*((nmax - *min)/(gdouble) (nmax - number) - 1.);
    hours1 = floor (remaining/3600.);
    mins1 = floor ((remaining - 3600.*hours1)/60.);
    secs1 = floor (remaining - 3600.*hours1 - 60.*mins1);

    fprintf (stderr, 
	     "\rEdges: %10u %3.0f%% %6.0f edges/s "
	     "Elapsed: %02.0f:%02.0f:%02.0f "
	     "Remaining: %02.0f:%02.0f:%02.0f ",
	     number, 
	     100.*(nmax - number)/(nmax - *min),
	     (nold - number)/g_timer_elapsed (timer, NULL),
	     hours, mins, secs,
	     hours1, mins1, secs1);
    fflush (stderr);

    nold = number;
    g_timer_start (timer);
  }


  if (number < *min) {
    g_timer_destroy (timer);
    g_timer_destroy (total_timer);
    timer =NULL;
    return TRUE;
  }
  return FALSE;
}
#endif
TriMesh *get_dense_grid(IplImage *mask,std::vector<libplankton::Vector> points){

   int ngrid = mask->width * mask->height;
   TriMesh *mesh = new TriMesh();
   if(!points.size())
     return NULL;
   mesh->grid_width=mask->width;
   mesh->grid_height=mask->height;

   mesh->grid.resize(ngrid, TriMesh::GRID_INVALID);
   int cnt=0;
   int valid=0;
   for(unsigned int row=0;row<(unsigned int)mask->height;row++){
     for(unsigned int col=0;col<(unsigned int)mask->width;col++){
   short val=(CV_IMAGE_ELEM(mask,uchar,row,col));
   if(val > 0){
       mesh->grid[cnt]=valid;
     libpolyp::point p(points[valid][0],points[valid][1],points[valid][2]);
     mesh->vertices.push_back(p);
     valid++;
   }
   cnt++;

     }
   }
   TriMesh::verbose=0;
   mesh->triangulate_grid();

   return mesh;
 }

typedef struct Vertex {
  float x,y,z,confidence,r,g,b,ex,ey,ez;

  void *other_props;       /* other properties */
} Vertex;

typedef struct Face {
  unsigned char nverts;    /* number of vertex indices in list */
  int *verts;              /* vertex index list */
  void *other_props;       /* other properties */
} Face;




static void write_vertex_ply (TVertex * v, gpointer * data)
{
  FILE * fp =(FILE*) data[0];
  bool bin=(bool)data[3];


  if(bin){
    float data[3];
    data[0]=GTS_VERTEX(v)->p.x;
    data[1]=GTS_VERTEX(v)->p.y;
    data[2]=GTS_VERTEX(v)->p.z;
    fwrite (data,sizeof(float),3,fp);
  }
  else
    fprintf (fp, "%g %g %g\n", GTS_VERTEX(v)->p.x, GTS_VERTEX(v)->p.y, GTS_VERTEX(v)->p.z);
  /*  fprintf (fp, " %g %g %g", v->ex, v->ey,v->ez);
 if(*confidence)
    fprintf (fp, " %g", v->confidence);
  fprintf (fp, " %g %g %g", v->r, v->g,v->b);
  */
    // fprintf (fp, " %g %g", v->u, v->v);




   GTS_OBJECT(v)->reserved = GUINT_TO_POINTER ((*((guint *) data[1]))++);


}
static void write_face_ply (GtsTriangle * t,  gpointer * data)
{
  FILE * fp =(FILE*) data[0];
  bool bin=(bool)data[3];

 GtsVertex * v1, * v2, * v3;
  gts_triangle_vertices (t, &v1, &v2, &v3);
  if(bin){
    char data[(4*3) + (1)];
    char *ptr=data;
    *ptr=3;
    ptr++;
    *((int *)ptr)= GPOINTER_TO_UINT (GTS_OBJECT (v1)->reserved);
    *((int *)ptr+1)= GPOINTER_TO_UINT (GTS_OBJECT (v2)->reserved);
    *((int *)ptr+2)= GPOINTER_TO_UINT (GTS_OBJECT (v3)->reserved);

    fwrite (data,sizeof(char),13,fp);
  }else{
    fprintf (fp, "3 %u %u %u",
         GPOINTER_TO_UINT (GTS_OBJECT (v1)->reserved),
         GPOINTER_TO_UINT (GTS_OBJECT (v2)->reserved),
         GPOINTER_TO_UINT (GTS_OBJECT (v3)->reserved));
    fputc ('\n', fp);
  }
}


  void auv_write_ply(GtsSurface *s,FILE *f,bool write_conf,const char *comment,bool binary){

  guint n = 0;
  gpointer data[4];

  int textured=0;
  g_return_if_fail (s != NULL);
  g_return_if_fail (f != NULL);

  data[0] = f;
  data[1] = &n;
  data[2]=&write_conf;
  data[3]=(void *)binary;


  gint ver_nr=gts_surface_vertex_number(s);
  gint tri_nr=gts_surface_face_number(s);
  fprintf(f, "ply\n");
  if(binary)
    fprintf(f, "format binary_little_endian 1.0\n");
  else
    fprintf(f, "format ascii 1.0\n");
  if(comment)
  fprintf(f, "comment %s\n", comment);
  fprintf(f, "element vertex %i\n", ver_nr);
  fprintf(f, "property float x\n");
  fprintf(f, "property float y\n");
  fprintf(f, "property float z\n");
  /*
  fprintf(f, "property float ex\n");
  fprintf(f, "property float ey\n");
  fprintf(f, "property float ez\n");
  if(write_conf)
    fprintf(f, "property float confidence\n");
  fprintf(f, "property float red\n");
  fprintf(f, "property float green\n");
  fprintf(f, "property float blue\n");*/
  if(textured){
    fprintf(f, "property float u\n");
    fprintf(f, "property float v\n");
  }


  fprintf(f, "element face %i\n", tri_nr);
  fprintf(f, "property list uchar int vertex_indices\n");
  fprintf(f, "end_header\n");

  gts_surface_foreach_vertex (s, (GtsFunc) write_vertex_ply, data);
  gts_surface_foreach_face (s, (GtsFunc) write_face_ply, data);
  gts_surface_foreach_vertex (s, (GtsFunc) gts_object_reset_reserved, NULL);

}

GtsSurface *auv_mesh_pts(GPtrArray *vertices,double zedgemax,int coarsenNumber){

 unsigned int i;
 GSList *list = NULL;
 GtsTriangle *t;
 GtsSurface *surface;
 GtsVertex *v1, *v2, *v3;

 /* create triangle enclosing all the vertices (needed for delaunay) */
 for (i = 0; i < vertices->len; i++)
   list = g_slist_prepend(list, g_ptr_array_index(vertices, i));
 t = gts_triangle_enclosing(gts_triangle_class(), list, 1);
 g_slist_free(list);

 surface = gts_surface_new(gts_surface_class(), (GtsFaceClass*)t_face_class(),
               gts_edge_class(), t_vertex_class());

 gts_surface_add_face(surface, (GtsFace*)t_face_new(t_face_class(),
                            t->e1, t->e2, t->e3));
 /* add all vertices to the surface */
 for (i = 0; i < vertices->len; i++) {

   TVertex * v1 =(TVertex*) g_ptr_array_index (vertices, i);
   GtsVertex * v = gts_delaunay_add_vertex (surface,\
                        GTS_VERTEX(v1), NULL);


   g_assert (v != GTS_VERTEX(v1));
   if (v != NULL) {
     //printf("Duplicate vertex\n");
     gts_vertex_replace (GTS_VERTEX(v1), v);
   }
   // printf("\rPercent Complete %.1f%%",(double)i/vertices->len*100.0);
 }
 //printf("\n");

 /* remove enclosing triangle */
 gts_triangle_vertices(t, &v1, &v2, &v3);
 gts_allow_floating_vertices = TRUE;
 gts_object_destroy(GTS_OBJECT(v1));
 gts_object_destroy(GTS_OBJECT(v2));
 gts_object_destroy(GTS_OBJECT(v3));
 gts_allow_floating_vertices = FALSE;

 if(coarsenNumber){
 fprintf(stderr,"Coarsen not implimented fail!\n");//  coarsen(surface,coarsenNumber);
 exit(-1);
 }
 clean_surf_pts(surface,zedgemax);

 return surface;

}
static void build_list (gpointer data, GSList ** list)
{
  /* always use O(1) g_slist_prepend instead of O(n) g_slist_append */
  *list = g_slist_prepend (*list, data);

}
static void edge_cleanup (GtsSurface * surface,double edgemult)
{
  GtsSurfaceQualityStats stats;
  gts_surface_quality_stats(surface,&stats);

  GSList * edges = NULL;
  GSList * i;
  int cleaned=0;
  g_return_if_fail (surface != NULL);

  /* build list of edges */
  gts_surface_foreach_edge (surface, (GtsFunc) build_list, &edges);

  /* remove degenerate and duplicate edges.
     Note: we could use gts_edges_merge() to remove the duplicates and then
     remove the degenerate edges but it is more efficient to do everything 
     at once (and it's more pedagogical too ...) */

  /* We want to control manually the destruction of edges */
  gts_allow_floating_edges = TRUE;

  i = edges;

  if(edgemult != 0){
    double edgelencutoff=edgemult;
    //printf("edge len cutoff is %f\n",edgelencutoff);
  
    while (i) {
      GtsEdge * e = (GtsEdge *)i->data;
      double length =abs(e->segment.v1->p.z -e->segment.v2->p.z);
      if(length > edgelencutoff){ 
	/* destroy e */
	cleaned++;
	gts_object_destroy (GTS_OBJECT (e));
      }
      i = i->next;
    }
  }

  //printf("Cleaned %d\n",cleaned);
  /*  i=edges;
  while (i) {
    GtsEdge * e = (GtsEdge *)i->data;
    GtsEdge * duplicate;
    if (GTS_SEGMENT (e)->v1 == GTS_SEGMENT (e)->v2) // edge is degenerate
      // destroy e 
      gts_object_destroy (GTS_OBJECT (e));
    else if ((duplicate = gts_edge_is_duplicate (e))) {
      // replace e with its duplicate
      gts_edge_replace (e, duplicate);
      // destroy e 
      gts_object_destroy (GTS_OBJECT (e));
    }
    i = i->next;
  }
  */
  /* don't forget to reset to default */
  gts_allow_floating_edges = FALSE;

  /* free list of edges */
  g_slist_free (edges);
}


static void vertex_clean (GtsVertex * v)
{
 
  gts_vertex_is_contact (v, TRUE);
}
static void vertex_cleanup (GtsSurface * s)
{
  gts_surface_foreach_vertex (s, (GtsFunc) vertex_clean, NULL);

  
}

static void triangle_cleanup (GtsSurface * s)
{
  GSList * triangles = NULL;
  GSList * i;

  g_return_if_fail (s != NULL);

  /* build list of triangles */
  gts_surface_foreach_face (s, (GtsFunc) build_list, &triangles);
  
  /* remove duplicate triangles */
  i = triangles;
  while (i) {
    GtsTriangle * t =(GtsTriangle *) i->data;
    if (gts_triangle_is_duplicate (t))
      /* destroy t, its edges (if not used by any other triangle)
	 and its corners (if not used by any other edge) */
      gts_object_destroy (GTS_OBJECT (t));
    i = i->next;
  }
  
  /* free list of triangles */
  g_slist_free (triangles);
}

void clean_surf_pts(GtsSurface *surface,double edgemult){
  
  vertex_cleanup(surface);
  /* eliminate degenerate and duplicate edges */
  edge_cleanup (surface,edgemult);
  /* eliminate duplicate triangles */
  triangle_cleanup (surface);
  
  
}

bool valid_tex_coord(Camera_Calib &calib,gfloat x,gfloat y){
 //
   // Test for valid image coords
   //

   if( x >= 0.0f && x < calib.width &&
       y >= 0.0f && y < calib.height   ){
     
     return true;
   }       
   else{
     
     return false;
   }  
}




bool valid_tex_coord_margin(Camera_Calib &calib,double &x,double &y,int margin){
 //
   // Test for valid image coords
   //

   if( x >= 0.0f-margin && x < calib.width+margin &&
       y >= 0.0f -margin&& y < calib.height+margin   ){
     
     if(x <= 0.0)
       x=0.0;
     if(y <= 0.0)
       y=0.0;

     if(x >= calib.width)
       x=calib.width-1;
     if(y >= calib.height)
       y=calib.height-1;

     return true;
   }       
   else{
     
     return false;
   }  
}

}

static void t_face_destroy( GtsObject * object )
{
   /* do object-specific cleanup here */

   /* do not forget to call destroy method of the parent */
   (* GTS_OBJECT_CLASS (t_face_class ())->parent_class->destroy) (object);
}

static void t_face_attribute( GtsObject *object, GtsObject *from )
{
   T_Face *o = T_FACE( object );
   T_Face *f = T_FACE( from );

   /*  o->iObject = f->iObject;
       o->iTerrain = f->iTerrain;*/
   o->material = f->material;
   /*   o->object = f->object;
	o->type = f->type;*/
   //   if ( o->f.triangle.e1 && o->f.triangle.e2 && o->f.triangle.e3 )
   // T_face_compute_normal( o );
}

static void t_face_class_init( T_FaceClass * klass )
{
   /* define new methods and overload inherited methods here */

   /* example of overloading of the destroy() function 
   (not needed if you don't need to do object-specific cleanup) */
   GTS_OBJECT_CLASS(klass)->destroy = t_face_destroy;
   GTS_OBJECT_CLASS(klass)->attributes = t_face_attribute;
}

static void t_face_init (T_Face * object)
{
  /* initialize object here */
  //object->type = 0;
   object->material = (size_t)-1;
   for(int i=0; i<NUM_TEX_BLEND_COORDS; i++)
     object->materialB[i] = (size_t)-1;
   /* object->iTerrain = (size_t)-1;
   object->object = (size_t)-1;
   object->iObject = (size_t)-1;*/
   /*object->normal[0] = 0.0;
   object->normal[1] = 0.0;
   object->normal[2] = 0.0;*/
}

T_FaceClass * t_face_class (void)
{
  static T_FaceClass * klass = NULL;

  if (klass == NULL) {
    GtsObjectClassInfo T_face_info = {
      "T_Face",
      sizeof (T_Face),
      sizeof (T_FaceClass),
      (GtsObjectClassInitFunc) t_face_class_init,
      (GtsObjectInitFunc) t_face_init,
      (GtsArgSetFunc) NULL,
      (GtsArgGetFunc) NULL
    };
    klass = (T_FaceClass*)gts_object_class_new (GTS_OBJECT_CLASS (gts_face_class ()),
				  &T_face_info);
  }

  return klass;
}

T_Face * t_face_new (T_FaceClass * klass, GtsEdge * e1, GtsEdge * e2, GtsEdge * e3, size_t m)
{
   T_Face * object;

   object = T_FACE( gts_face_new( GTS_FACE_CLASS( klass ), GTS_EDGE( e1 ), GTS_EDGE( e2 ), GTS_EDGE( e3 ) ) );
   //object->type = t;
   object->material = m;
   
   //   T_face_compute_normal( object );

   return object;
}

/* TVertex: Object */


static void t_vertex_init (TVertex * object)
{
  /* initialize object here */

  object->id=-1;
  object->u=0.0;
  object->v=0.0;
  for(int i=0; i< NUM_TEX_BLEND_COORDS; i++)
    object->uB[i] =  object->vB[i]=0.0;
  
     
  /* object->confidence=0.0;
  object->r=1.0;
  object->g=1.0;
  object->b=1.0;*/

  //(* GTS_OBJECT_CLASS (t_vertex_class ())->parent_class->init) (GTS_VERTEX(object));
}


GtsVertexClass * t_vertex_class (void)
{
  static GtsVertexClass * klass = NULL;

  if (klass == NULL) {
    GtsObjectClassInfo t_vertex_info = {
      "TVertex",
      sizeof (TVertex),
      sizeof (GtsVertexClass),
      (GtsObjectClassInitFunc)NULL,
    (GtsObjectInitFunc)  t_vertex_init,
      (GtsArgSetFunc) NULL,
      (GtsArgGetFunc) NULL
    };
    klass =(GtsVertexClass*)gts_object_class_new (GTS_OBJECT_CLASS (gts_vertex_class ()),
				 &t_vertex_info);
  }

  return klass;
}


#define BIGNUM 2147483647
#define NO_COMP -1
#define FOR_EACH_ADJACENT_FACE(mesh,v,f) \
	for (int f_ind = 0, f = mesh->adjacentfaces[v][0]; \
	     (f_ind < mesh->adjacentfaces[v].size()) && \
	     ((f = mesh->adjacentfaces[v][f_ind]) || 1); \
	     f_ind++)


static bool conn_vert = false;	// Consider connectivity along vertices? (else edges)



// Are two faces connected along an edge (or vertex)?
bool connected(const TriMesh *in, int f1, int f2)
{
	int f10=in->faces[f1][0], f11=in->faces[f1][1], f12=in->faces[f1][2];
	int f20=in->faces[f2][0], f21=in->faces[f2][1], f22=in->faces[f2][2];

	if (conn_vert)
		return f10 == f20 || f10 == f21 || f10 == f22 ||
		       f11 == f20 || f11 == f21 || f11 == f22 ||
		       f12 == f20 || f12 == f21 || f12 == f22;
	else
		return (f10 == f20 && (f11 == f22 || f12 == f21)) ||
		       (f10 == f21 && (f11 == f20 || f12 == f22)) ||
		       (f10 == f22 && (f11 == f21 || f12 == f20)) ||
		       (f11 == f20 && f12 == f22) ||
		       (f11 == f21 && f12 == f20) ||
		       (f11 == f22 && f12 == f21);
}


// Helper function for find_comps, below.  Finds and marks all the faces
// connected to f.
void find_connected(const TriMesh *in,
		    vector<int> &comps, vector<int> &compsizes,
		    int f, int whichcomponent)
{
	stack<int> s;
	s.push(f);
	while (!s.empty()) {
		int currface = s.top();
		s.pop();
		for (int i = 0; i < 3; i++) {
			int vert = in->faces[currface][i];
			FOR_EACH_ADJACENT_FACE(in, vert, adjface) {
				if (comps[adjface] != NO_COMP ||
				    !connected(in, adjface, currface))
					continue;
				comps[adjface] = whichcomponent;
				compsizes[whichcomponent]++;
				s.push(adjface);
			}
		}
	}
}

// Find the connected components of TriMesh "in".
// Outputs:
//  comps is a vector that gives a mapping from each face to its
//   associated connected component.
//  compsizes holds the size of each connected component
void find_comps(TriMesh *in, vector<int> &comps, vector<int> &compsizes)
{
	if (in->vertices.empty())
		return;
	if (in->faces.empty())
		return;
	in->need_adjacentfaces();

	int nf = in->faces.size();
	comps.clear();
	comps.reserve(nf);
	comps.resize(nf, NO_COMP);
	compsizes.clear();

	for (int i = 0; i < nf; i++) {
		if (comps[i] != NO_COMP)
			continue;
		int comp = compsizes.size();
		comps[i] = comp;
		compsizes.push_back(1);
		find_connected(in, comps, compsizes, i, comp);
	}
}


// Helper class for comparing two integers by finding the elements at those
// indices within some array and comparing them
template <class Array>
class CompareArrayElements {
private:
	const Array &a;
public:
	CompareArrayElements(const Array &_a) : a(_a)
		{}
	bool operator () (int i1, int i2) const
	{
		return (a[i1] > a[i2]);
	}
};


// Sorts the connected components from largest to smallest.  Renumbers the
// elements of compsizes to reflect this new numbering.
void sort_comps(vector<int> &comps, vector<int> &compsizes)
{
	if (compsizes.size() < 1)
		return;

	unsigned int i;
	vector<int> comp_pointers(compsizes.size());
	for (i = 0; i < comp_pointers.size(); i++)
		comp_pointers[i] = i;

	sort(comp_pointers.begin(), comp_pointers.end(),
	     CompareArrayElements< vector<int> >(compsizes));

	vector<int> remap_table(comp_pointers.size());
	for (i = 0; i < comp_pointers.size(); i++)
		remap_table[comp_pointers[i]] = i;
	for (i = 0; i < comps.size(); i++)
		comps[i] = remap_table[comps[i]];

	vector<int> newcompsizes(compsizes.size());
	for (i = 0; i < compsizes.size(); i++)
		newcompsizes[i] = compsizes[comp_pointers[i]];
	compsizes = newcompsizes;
}


// Print out the connected components that are bigger than morethan and
// smaller than lessthan.  The largest min(nprint, total) components are
// printed out, unless morethan == 0 and lessthan != BIGNUM, in which case


// Select a particular connected component, and delete all other vertices from
// the mesh.
void select_comp(TriMesh *in, const vector<int> &comps, int whichcc)
{
	int numfaces = in->faces.size();
	vector<bool> toremove(numfaces, false);
	for (unsigned int i = 0; i < in->faces.size(); i++) {
		if (comps[i] != whichcc)
			toremove[i] = true;
	}

	remove_faces(in, toremove);
	remove_unused_vertices(in);
}


// Select the requested connected components (as given by the "morethan",
// "lessthan", and "total" parameters), and delete all other vertices from
// the mesh.
void select_comps_by_size(TriMesh *in, const vector<int> &comps,
			  const vector<int> &compsizes,
			  int morethan, int lessthan, int total)
{
	if (compsizes.size() < 1 || total < 1)	
		return;
	if (compsizes.size() == 1 &&
	    (compsizes[0] > lessthan || compsizes[0] < morethan)) {
		in->faces.clear();
		in->vertices.clear();
		return;
	}

	int keep_first = 0;
	while (keep_first < compsizes.size() && compsizes[keep_first] > lessthan)
		keep_first++;
	int keep_last = compsizes.size()-1;
	while (keep_last > -1 && compsizes[keep_last] < morethan)
		keep_last--;
	if (keep_last-keep_first+1 > total) {
		if (morethan == 0 && lessthan != BIGNUM) {
			// Keep the smallest components
			keep_first = keep_last+1-total;
		} else {
			// Keep the largest components
			keep_last = keep_first+total-1;
		}
	}

	int numfaces = in->faces.size();
	vector<bool> toremove(numfaces, false);
	for (int i = 0; i < numfaces; i++) {
		if (comps[i] < keep_first || comps[i] > keep_last)
			toremove[i] = true;
	}

	remove_faces(in, toremove);
	remove_unused_vertices(in);
}


void mesh_proc::cc_size_filter(TriMesh *in,int size){
  vector<int> comps;
  vector<int> compsizes;
  
  in->need_faces();
  in->tstrips.clear();
  int numfaces = in->faces.size();
  find_comps(in, comps, compsizes);
  vector<bool> toremove(numfaces, false);
  vector<int> comps_removed;
    for(unsigned int i=0; i < compsizes.size(); i++){
      if(compsizes[i] < size){
	comps_removed.push_back(i);
	
      }
    }


    for (unsigned int i = 0; i < in->faces.size(); i++) {
      for(unsigned int j=0; j<comps_removed.size(); j++){
	if (comps[i] == comps_removed[j])
	  toremove[i] = true;
      }
    }
    
    remove_faces(in, toremove);
    remove_unused_vertices(in);
}
bool check_same(point pt1,point pt2,double eps){
 
  if(fabs(pt1[0]-pt2[0]) < eps && fabs(pt1[1]-pt2[1]) < eps && fabs(pt1[2]-pt2[2]) < eps)
    return true;
  else 
    return false;
}

// Remove vertices that aren't referenced by any face
void mesh_proc::remove_duplicate_vertices(TriMesh *mesh,double eps)
{
	int nv = mesh->vertices.size();
	if (!nv)
		return;

	bool had_faces = !mesh->faces.empty();
	mesh->need_faces();
	vector<bool> dupl(nv, false);
	for (int i = 0; i < nv; i++) {
	  for(int j=0; j< nv; j++){
	    if(i==j)
	      continue;
	    dupl[i]=check_same(mesh->vertices[i],mesh->vertices[j],eps);
	  }
	}
	remove_vertices(mesh, dupl);
	if (!had_faces)
		mesh->faces.clear();
}
// Quick 'n dirty portable random number generator 
static inline float tinyrnd()
{
	static unsigned trand = 0;
	trand = 1664525u * trand + 1013904223u;
	return (float) trand / 4294967296.0f;
}
// Compute a "feature size" for the mesh: computed as 1% of 
// the reciprocal of the 10-th percentile curvature 
float feature_size(TriMesh *mesh)
{
	mesh->need_curvatures();
	int nv = mesh->curv1.size();
	int nsamp = min(nv, 500);
	
	vector<float> samples;
	samples.reserve(nsamp * 2);
        
	for (int i = 0; i < nsamp; i++) {
		// Quick 'n dirty portable random number generator  
		static unsigned randq = 0;
		randq = unsigned(1664525) * randq + unsigned(1013904223);

		int ind = int(tinyrnd() * nv);
		samples.push_back(fabs(mesh->curv1[ind]));
		samples.push_back(fabs(mesh->curv2[ind]));
	}
	const float frac = 0.1f;
	const float mult = 0.01f;
	int which = int(frac * samples.size());
	nth_element(samples.begin(), samples.begin() + which, samples.end());
	float f = mult / samples[which];
	if (!isfinite(f)) {
		mesh->need_bsphere();
		f = mesh->bsphere.r;
	}
		
 TriMesh::dprintf( "Feature size = %f\n", f);
	return f;
}
bool  mesh_proc::find_closest_pt_idx(TriMesh *mesh, const point &p,
				    int &ind,double _maxdist)
{


  KDtree *kd = new KDtree(mesh->vertices);

  float fs = feature_size(mesh);
  float maxdist = _maxdist* fs;
  TriMesh::dprintf("Using maxdist = %f\n", maxdist);
  float maxdist2 = sqr(maxdist);

  const float *match = kd->closest_to_pt(p, maxdist2);
  if (!match)
    return false;
  ind = (match - (const float *) &(mesh->vertices[0][0])) / 3;
  if (ind < 0 || ind >= mesh->vertices.size())
    return false;
 
  return true;
  
}


float mesh_proc::clamp(float x, float low,float hi){
  if(x > hi)
    return hi;
  if(x < low)
    return low;
  return x;
}
