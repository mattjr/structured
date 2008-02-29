/*

  Compute the axis aligned bounding box that fits around the
  vertices within a ply file.

  Brian Curless
  June 1995

  Steve Marschner, December 2000
  Generalized to multiple input files.

*/


#include <ply.h>
#include <stdlib.h>
#include <strings.h>
#include <math.h>
#include <limits.h>
#include <stdio.h>
#include <float.h>

#define MAX(a, b) (a) > (b) ? (a) : (b)
#define MIN(a, b) (a) < (b) ? (a) : (b)

typedef struct Vertex {
    float x, y, z;
    float diff_r, diff_g, diff_b;
  int index;
} Vertex;

static char *elem_names[] = { 
  "vertex","face"
};
typedef struct Face {
  unsigned char nverts;    /* number of vertex indices in list */
  int *verts;              /* vertex index list */
  //void *other_props;       /* other properties */
  int valid;
} Face;

void
read_file(FILE *inFile);
static PlyProperty vert_props[] = { 
  {"x", PLY_FLOAT, PLY_FLOAT, 0, 0, PLY_START_TYPE, PLY_START_TYPE, 0},
  {"y", PLY_FLOAT, PLY_FLOAT, 0, 0, PLY_START_TYPE, PLY_START_TYPE, 0},
  {"z", PLY_FLOAT, PLY_FLOAT, 0, 0, PLY_START_TYPE, PLY_START_TYPE, 0},
};
PlyProperty face_props[] = { /* list of property information for a vertex */
  {"vertex_indices", PLY_INT, PLY_INT, offsetof(Face,verts),
   1, PLY_UCHAR, PLY_UCHAR, offsetof(Face,nverts)},
};
typedef struct BBox {
    float minx, miny, minz, maxx, maxy, maxz;
} BBox;

void write_file(BBox *bboxes,int num);
PlyFile *readPlyFile(FILE *inFile, Vertex **pVerts, int *pNumVerts);
void initbbox(BBox *b);
void updatebbox(BBox *b, Vertex *verts, int numVerts,float eps,bool usez);
void printbbox(BBox *b);
void printUsage();

static int nverts,nfaces;
static Vertex **vlist;
static Face **flist;
static PlyOtherElems *other_elements = NULL;
static PlyOtherProp *vert_other,*face_other;
static int nelems;
static char **elist;
static int num_comments;
static char **comments;
static int num_obj_info;
static char **obj_info;
static int file_type;
void
printusage(char *progname)
{
  fprintf (stderr, "usage: %s [in.ply ...]\n", progname);
  fprintf (stderr, "   or: %s < in.ply\n", progname);
  exit (-1);
}


int
main(int argc, char**argv)
{    
   Vertex *verts = NULL;
   int numVerts;
   char *s;
   char *progname;
   FILE *inFile = NULL;
   float eps=0.0;

  
   bool usez=false;
   progname = argv[0];
   int count=0;
   /* parse -flags */
   while (--argc > 0 && (*++argv)[0]=='-') {
      for (s = argv[0]+1; *s; s++)
	 switch (*s) {
	 case 'z':
	   usez=true;
	   break;
	 case 'e':
	   if(argc < 1) printusage(progname);
	   eps= atof (*++argv);
	   argc-=1;
	   break;
	 default:
	     printusage(progname);
	     break;
	 }
   }    
    int   nummeshes=argc-2;
 BBox bboxes[nummeshes];
 

   /* optional input files (if not, read stdin ) */
   if (argc > 0) {
       while (argc > 0 && *argv[0] != '-' && count < nummeshes) {
	   inFile = fopen(argv[0], "r");
	   if (inFile == NULL) {
	       fprintf(stderr, "Error: Couldn't open input file %s\n", argv[0]);
	       printusage(progname);
	   }
	   argc --;
	   argv ++;

	   readPlyFile(inFile, &verts, &numVerts);

	   if (verts == NULL) {
	       fprintf(stderr, "Obtained no vertices from %s.\n", argv[0]);
	       exit(1);
	   }
	   initbbox(&bboxes[count]);
	   updatebbox(&bboxes[count], verts, numVerts,eps,usez);
	   //  clipfinalmesh(bboxes[count]);
	   //updatebbox(&box, verts, numVerts);
	   //printbbox(&bboxes[count]);
	   count++;
	   //fprintf(stderr,"%s",argv[0]);
       } 
   } else {
       readPlyFile(stdin, &verts, &numVerts);

       if (verts == NULL) {
	   fprintf(stderr, "Obtained no vertices from %s.\n", argv[0]);
	   exit(1);
	   }
       
       // updatebbox(&box, verts, numVerts);
   }
   fprintf(stderr,"EPS %f\n",eps);
   fprintf(stderr,"Num verts %d\n",numVerts);
   argv++;

   //  fprintf(stderr,"%s",argv[0]);
   inFile = fopen(argv[0], "r");
   read_file(inFile);
   fprintf(stderr,"Num verts %d\n",nfaces);
   /* if (argc > 0) {
     fprintf(stderr, "Error: Unhandled arg: %s\n", argv[0]);
     printusage(progname);
     exit(-1);
     }*/

   write_file(bboxes,nummeshes);
  
   
   exit(0);
}


void
initbbox(BBox *b)
{
    b->minx = FLT_MAX;
    b->miny = FLT_MAX;
    b->minz = FLT_MAX;

    b->maxx = -FLT_MAX;
    b->maxy = -FLT_MAX;
    b->maxz = -FLT_MAX;
}

int Keep_Face(float xmin, float ymin, float zmin,
              float xmax, float ymax, float zmax, Face *face)
{
    float maxx;         // The maximum X-coord of the face
    Vertex *maxvert;    // The vertex with maximum X-coord
    int i;

    if (face->nverts <= 0)
        return 0;

    maxvert = vlist[face->verts[0]];
    maxx = maxvert->x;

    for (i=1; i<face->nverts; ++i)
	if (vlist[face->verts[i]]->x > maxx)  {
            maxvert = vlist[face->verts[i]];
	    maxx = maxvert->x;
        }

    if (maxvert->x > xmax) return 0;
    if (maxvert->x < xmin) return 0;
    if (maxvert->y > ymax) return 0;
    if (maxvert->y < ymin) return 0;
    if (maxvert->z > zmax) return 0;
    if (maxvert->z < zmin) return 0;

    return 1;
}

void
updatebbox(BBox *b, Vertex *verts, int numVerts,float eps,bool usez)
{
    int i;

    for(i = 0; i < numVerts; i++) {
	b->minx = MIN(b->minx, verts[i].x);
	b->miny = MIN(b->miny, verts[i].y);
	b->minz = MIN(b->minz, verts[i].z);
	
	b->maxx = MAX(b->maxx, verts[i].x);
	b->maxy = MAX(b->maxy, verts[i].y);
	b->maxz = MAX(b->maxz, verts[i].z);	



    }
	b->minx-=eps;
	b->miny-=eps;
	b->minz-=eps;


	b->maxx+=eps;
	b->maxy+=eps;
	b->maxz+=eps;
	if(!usez){
	  b->minz=FLT_MIN;
	  b->maxx=FLT_MAX;
	}
}


void
printbbox(BBox *b)
{
    printf("\n");
    printf("%f %f %f\n", b->minx, b->miny, b->minz);
    printf("%f %f %f\n", b->maxx, b->maxy, b->maxz);
    printf("\n");
}



PlyFile *
readPlyFile(FILE *inFile, Vertex **pVerts, int *pNumVerts)
{
    int i, j;
    int nelems, numVerts;
    char **elist;
    int file_type;
    float version;
    char *elem_name;
    int nprops, num_vert_props;
    int num_elems;
    PlyProperty **plist;
    Vertex *verts;
    PlyFile *ply;

    vert_props[0].offset = offsetof(Vertex, x);
    vert_props[1].offset = offsetof(Vertex, y);
    vert_props[2].offset = offsetof(Vertex, z);
    num_vert_props = 3;

    ply  = ply_read (inFile, &nelems, &elist);
    ply_get_info (ply, &version, &file_type);

    if (!ply)
	exit(1);

    verts = NULL;

    for (i = 0; i < nelems; i++) {

	/* get the description of the first element */
	elem_name = elist[i];
	plist = ply_get_element_description 
	    (ply, elem_name, &num_elems, &nprops);
	
	/* if we're on vertex elements, read them in */
	if (equal_strings ("vertex", elem_name)) {
	    
	    numVerts = *pNumVerts = num_elems;
	    verts = (Vertex *)malloc(sizeof(Vertex)*numVerts);
	    
	    /* set up for getting vertex elements */
	    ply_get_element_setup (ply, elem_name, num_vert_props, vert_props);
	    
	    /* grab all the vertex elements */
	    for (j = 0; j < numVerts; j++)
		ply_get_element (ply, (void *) &verts[j]);
	}
    }

    if (*pVerts) free(*pVerts);
    *pVerts = verts;

    return ply;
}


void
printUsage()
{
    fprintf(stderr, "\n");
    fprintf(stderr, "plybbox <ply-file>\n");
    fprintf(stderr, "\n");
    fprintf(stderr, "  Plybbox prints the min and max x,y,z values of the vertices\n");
    fprintf(stderr, "  in the Ply file.  The output is relatively unformated:\n");
    fprintf(stderr, "\n");
    fprintf(stderr, "     <minx> <miny> <minz>\n");
    fprintf(stderr, "     <maxx> <maxy> <maxz>\n");
    fprintf(stderr, "\n");
}

/******************************************************************************
Read in the PLY file from standard in.
******************************************************************************/

void
read_file(FILE *inFile)
{
  int i,j,k;
  PlyFile *ply;
  int nprops;
  int num_elems;
  PlyProperty **plist;
  char *elem_name;
  float version;


  /*** Read in the original PLY object ***/


  ply  = ply_read (inFile, &nelems, &elist);
  ply_get_info (ply, &version, &file_type);

  for (i = 0; i < nelems; i++) {

    /* get the description of the first element */
    elem_name = elist[i];
    plist = ply_get_element_description (ply, elem_name, &num_elems, &nprops);

    if (equal_strings ("vertex", elem_name)) {

      /* create a vertex list to hold all the vertices */
      vlist = (Vertex **) malloc (sizeof (Vertex *) * num_elems);
      nverts = num_elems;

      /* set up for getting vertex elements */

      ply_get_property (ply, elem_name, &vert_props[0]);
      ply_get_property (ply, elem_name, &vert_props[1]);
      ply_get_property (ply, elem_name, &vert_props[2]);
      /*   vert_other = ply_get_other_properties (ply, elem_name,
                     offsetof(Vertex,other_props));
      */
      /* grab all the vertex elements */
      for (j = 0; j < num_elems; j++) {
        vlist[j] = (Vertex *) malloc (sizeof (Vertex));
        ply_get_element (ply, (void *) vlist[j]);
      }
    }
    else if (equal_strings ("face", elem_name)) {

      /* create a list to hold all the face elements */
      flist = (Face **) malloc (sizeof (Face *) * num_elems);
      nfaces = num_elems;

      /* set up for getting face elements */

      ply_get_property (ply, elem_name, &face_props[0]);
      /*face_other = ply_get_other_properties (ply, elem_name,
                     offsetof(Face,other_props));
      */
      /* grab all the face elements */
      for (j = 0; j < num_elems; j++) {
        flist[j] = (Face *) malloc (sizeof (Face));
        ply_get_element (ply, (void *) flist[j]);
	/* DEBUG
	fprintf(stderr, "face %d: %d verts: %d %d %d\n",
		j, flist[j]->nverts,
		flist[j]->verts[0],
		flist[j]->verts[1],
		flist[j]->verts[2]);
	*/
      }
    }
    else
      other_elements = ply_get_other_element (ply, elem_name, num_elems);
  }

  comments = ply_get_comments (ply, &num_comments);
  obj_info = ply_get_obj_info (ply, &num_obj_info);

  ply_close (ply);
}
/******************************************************************************
Write out the PLY file to standard out.
Ignore all the points (and corresponding faces) below
the plane.
******************************************************************************/


void write_file(BBox *bboxes,int num)
{
  int i,j,k;
  PlyFile *ply;
  int num_elems;
  char *elem_name;
  int vert_count;
  int face_count;

  /*** Write out the final PLY object ***/


  ply = ply_write (stdout, nelems, elist, file_type);

  // count the vertices that are above the plane
  vert_count = 0;
  for (i = 0; i < nverts; i++) {
    // Set the index to either the index number, or -1...
    if(1){ 
      vlist[i]->index = vert_count;
      vert_count++;
    } else {
      vlist[i]->index = -1;
    }
  }
    for (i = 0; i < nfaces; i++) {
      flist[i]->valid=false;
    }
  // count the faces that are still valid
  face_count = 0;
  
  for (i = 0; i < nfaces; i++) {
    for(int j=0; j < num; j++){
      bool valid = Keep_Face(bboxes[j].minx,bboxes[j].miny,bboxes[j].minz,
			     bboxes[j].maxx,bboxes[j].maxy,bboxes[j].maxz,flist[i]);
      if(valid){
	
	flist[i]->valid=true;
      }
    }    
  }

  for(i = 0; i < nfaces; i++) {
    // If face not valid, set nverts to 0, so it won't
    // get written out later.
    if (flist[i]->valid) {
      face_count++;
    } else {
      flist[i]->nverts = 0;
    }
  }
  fprintf(stderr,"valid face %d\n",face_count);
  /* describe what properties go into the vertex and face elements */

  ply_element_count (ply, "vertex", vert_count);
  ply_describe_property (ply, "vertex", &vert_props[0]);
  ply_describe_property (ply, "vertex", &vert_props[1]);
  ply_describe_property (ply, "vertex", &vert_props[2]);
  //ply_describe_other_properties (ply, vert_other, offsetof(Vertex,other_props));
  ply_element_count (ply, "face", face_count);
  ply_describe_property (ply, "face", &face_props[0]);
  //ply_describe_other_properties (ply, face_other, offsetof(Face,other_props));

  ply_describe_other_elements (ply, other_elements);

  for (i = 0; i < num_comments; i++)
    ply_put_comment (ply, comments[i]);

  for (i = 0; i < num_obj_info; i++)
    ply_put_obj_info (ply, obj_info[i]);

  ply_header_complete (ply);

  /* set up and write the vertex elements */

  ply_put_element_setup (ply, "vertex");

  for (i = 0; i < nverts; i++)
    if (vlist[i]->index > -1)
      ply_put_element (ply, (void *) vlist[i]);

  /* set up and write the face elements */
  ply_put_element_setup (ply, "face");

  for (i = 0; i < nfaces; i++) {
    if (flist[i]->nverts == 0)
      continue;
    for (j = 0; j < flist[i]->nverts; j++)
      flist[i]->verts[j] = (vlist[flist[i]->verts[j]])->index;
    ply_put_element (ply, (void *) flist[i]);
  }

  ply_put_other_elements (ply);

  /* close the PLY file */
  ply_close (ply);
}
