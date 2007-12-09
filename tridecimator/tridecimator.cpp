#include <vector>
#include <limits>

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
using namespace std;

// stuff to define the mesh
#include <vcg/simplex/vertexplus/base.h>
#include <vcg/simplex/faceplus/base.h>
#include <vcg/simplex/edge/edge.h>
#include <vcg/complex/trimesh/base.h>
#include <vcg/complex/trimesh/hole.h>
#include <vcg/math/quadric.h>
#include <vcg/complex/trimesh/clean.h>
#include<vcg/complex/trimesh/base.h>
#include<vcg/simplex/vertexplus/component_ocf.h>
#include<vcg/simplex/faceplus/component_ocf.h>
#include <vcg/complex/trimesh/update/topology.h>

// io
#include <wrap/io_trimesh/import.h>
#include <wrap/io_trimesh/export_ply.h>
#include <wrap/io_trimesh/export_stl.h>

// update
#include <vcg/complex/trimesh/update/topology.h>
#include <vcg/complex/trimesh/update/bounding.h>
#include <vcg/complex/trimesh/update/normal.h>

// local optimization
#include <vcg/complex/local_optimization.h>
#include <vcg/complex/local_optimization/tri_edge_collapse_quadric.h>

#include "remove_small_cc.h"

using namespace vcg;
using namespace tri;
class CEdge;   
class CFaceO;
class CVertexO;

//Vert Mem Occupancy  --- 44b ---

class CVertexO  : public VertexSimp2< CVertexO, CEdge, CFaceO, 
				      vert::Coord3f,     /* 12b */ 
				      vert::BitFlags,    /*  4b */
				      vert::Normal3f,    /* 12b */
				      vert::Qualityf,    /*  4b */
				      vert::VFAdj,       /*  4b */
				      vert::Mark,        /*  4b */
				      vert::Color4b      /*  4b */
				      >{ 
};

  
class CEdge : public Edge<CEdge,CVertexO> {
public:
  inline CEdge() {};
  inline CEdge( CVertexO * v0, CVertexO * v1):Edge<CEdge,CVertexO>(v0,v1){};
  static inline CEdge OrderedEdge(VertexType* v0,VertexType* v1){
    if(v0<v1) return CEdge(v0,v1);
    else return CEdge(v1,v0);
  }

  //inline CEdge( Edge<CEdge,CVertexO> &e):Edge<CEdge,CVertexO>(e){};
};

//Face Mem Occupancy  --- 32b ---

class CFaceO    : public FaceSimp2<  CVertexO, CEdge, CFaceO,  
				     face::InfoOcf,              /* 4b */
				     face::VertexRef,            /*12b */
				     face::BitFlags,             /* 4b */
				     face::Normal3f,             /*12b */
				     face::MarkOcf,              /* 0b */
				     face::Color4bOcf,           /* 0b */
				     face::FFAdjOcf,             /* 0b */
				     face::VFAdjOcf,             /* 0b */
				     face::WedgeTexturefOcf      /* 0b */
				     > {};

class CMeshO    : public vcg::tri::TriMesh< vector<CVertexO>, face::vector_ocf<CFaceO> > {
public :
  int sfn; //The number of selected faces.
};
typedef	SimpleTempData<CMeshO::VertContainer, math::Quadric<double> > QuadricTemp;


class QHelper
{
public:
  QHelper(){};
  static void Init(){};
  static math::Quadric<double> &Qd(CVertexO &v) {return TD()[v];}
  static math::Quadric<double> &Qd(CVertexO *v) {return TD()[*v];}
  static CVertexO::ScalarType W(CVertexO * /*v*/) {return 1.0;};
  static CVertexO::ScalarType W(CVertexO & /*v*/) {return 1.0;};
  static void Merge(CVertexO & /*v_dest*/, CVertexO const & /*v_del*/){};
  static QuadricTemp* &TDp() {static QuadricTemp *td; return td;}
  static QuadricTemp &TD() {return *TDp();}
};


class MyTriEdgeCollapse: public vcg::tri::TriEdgeCollapseQuadric< CMeshO, MyTriEdgeCollapse, QHelper > {
public:
  typedef  vcg::tri::TriEdgeCollapseQuadric< CMeshO,  MyTriEdgeCollapse, QHelper> TECQ;
  typedef  CMeshO::VertexType::EdgeType EdgeType;
  inline MyTriEdgeCollapse(  const EdgeType &p, int i) :TECQ(p,i){}
};

/**********************************************************
Mesh Classes for Quadric Edge collapse based simplification

For edge collpases we need verteses with:
- V->F adjacency
- per vertex incremental mark
- per vertex Normal


Moreover for using a quadric based collapse the vertex class 
must have also a Quadric member Q();
Otherwise the user have to provide an helper function object 
to recover the quadric.

******************************************************/
// The class prototypes.

void Usage()
{
  printf(
	 "---------------------------------\n"
	 "         TriSimp V.1.0 \n"
	 "     http://vcg.isti.cnr.it\n"
	 "    http://vcg.sourceforge.net\n"
	 "   release date: "__DATE__"\n"
	 "---------------------------------\n\n"
	 "TriDecimator 1.0 \n"__DATE__"\n"
	 "Copyright 2003-2006 Visual Computing Lab I.S.T.I. C.N.R.\n"
	 "\nUsage:  "\
	 "tridecimator file1 file2 face_num [opt]\n"\
	 "Where opt can be:\n"\
	 "     -e# QuadricError threshold  (range [0,inf) default inf)\n"
	 "     -b# Boundary Weight (default .5)\n"
	 "     -q# Quality threshold (range [0.0, 0.866],  default .3 )\n"
	 "     -n# Normal threshold  (degree range [0,180] default 90)\n"
	 "     -E# Minimal admitted quadric value (default 1e-15, must be >0)\n"
	 "     -Q#  Use Quality filtering with val range [0.0, 1.0] (default no)\n"
	 "     -N[y|n]  Use or not Normal Threshold (default no)\n"
	 "     -A[y|n]  Use or not Area Weighted Quadrics (default yes)\n"
	 "     -O[y|n]  Use or not vertex optimal placement (default yes)\n"
	 "     -S#  Use  connected comp size filtering  diameter [must be >0]\n"
	 "     -B[y|n]  Preserve or not mesh boundary (default no)\n"
	 "     -T[y|n]  Preserve or not Topology (default no)\n"
	 "     -H#  Fill holes up to size[ range > 0]\n"
	 "     -P       Before simplification, remove duplicate & unreferenced vertices\n"
	 "     -F     FLip mesh\n"
	 "     -f<xffile>     transform mesh by xffile\n"
	 );
  exit(-1);
}

// mesh to simplify

CMeshO cm;
int main(int argc ,char**argv){
  if(argc<4) Usage();
  bool FlipMesh=false;
  bool QualityClean=false;
  float QualityCleanVal=0.3;
  char *xfname=NULL;
  bool SizeClean=false;
  bool CleaningFlag=false;
  float minDiaSmallCC=100.0;
  bool EdgeLenClean=true;
  float EdgeLen=0;
  int MaxHoleSize=0;
  bool FillHoles=false;
  int TargetFaceNum=0;
  string tgtStr(argv[3]);
  string edgelenthresh;

  MyTriEdgeCollapse::SetDefaultParams();
  TriEdgeCollapseQuadricParameter qparams=MyTriEdgeCollapse::Params();

  // parse command line.
  for(int i=4; i < argc;){
    if(argv[i][0]=='-')
      switch(argv[i][1]){ 
       case 'F' :   FlipMesh	= true;  printf("Flipping Mesh\n");    
	  break;
      case 'N' : if(argv[i][2]=='y') { qparams.NormalCheck	= true;  printf("Using Normal Deviation Checking\n");	}	else { qparams.NormalCheck	= false; printf("NOT Using Normal Deviation Checking\n");	}        break;	
      case 'f' :	xfname =argv[i]+2;	           printf("Transforming with %s file\n",xfname); 	 break;	

      case 'O' : if(argv[i][2]=='y') { qparams.OptimalPlacement	= true;  printf("Using OptimalPlacement\n");	}
	else { qparams.OptimalPlacement	= false; printf("NOT Using OptimalPlacement\n");	}        break;		
   
      case 'B' : if(argv[i][2]=='y') { qparams.FastPreserveBoundary	= true;  printf("Preserving Boundary\n");	}
	else { qparams.FastPreserveBoundary	= false; printf("NOT Preserving Boundary\n");	}        break;		
      case 'T' : if(argv[i][2]=='y') { qparams.PreserveTopology	= true;  printf("Preserving Topology\n");	}
	else { qparams.PreserveTopology	= false; printf("NOT Preserving Topology\n");	}        break;		
      case 'q' :	qparams.QualityThr	= atof(argv[i]+2);	           printf("Setting Quality Thr to %f\n",atof(argv[i]+2)); 	 break;			
      case 'n' :	qparams.NormalThrRad = math::ToRad(atof(argv[i]+2));  printf("Setting Normal Thr to %f deg\n",atof(argv[i]+2)); break;	
      case 'e' :	edgelenthresh=string(argv[i]+2);  EdgeLenClean=true; printf("Cleaning edge len\n"); break;	
      case 'b' :	qparams.BoundaryWeight  = atof(argv[i]+2);			printf("Setting Boundary Weight to %f\n",atof(argv[i]+2)); break;		
      case 'P' :	CleaningFlag=true;  printf("Cleaning mesh before simplification\n",atof(argv[i]+2)); break;	
       
      case 'Q' :	QualityClean=true; QualityCleanVal=atof(argv[i]+2); printf("Cleaning with quality threshold of %f\n",atof(argv[i]+2)); break;	
      case 'S' :	SizeClean=true; minDiaSmallCC=atof(argv[i]+2); printf("Cleaning with connected componet diameter threshold of %f\n",atof(argv[i]+2)); break;	
      case 'H' :	FillHoles=true; MaxHoleSize=atoi(argv[i]+2); printf("Filling holes with a maximum size of %d\n",atoi(argv[i]+2)); break;	
   
      default  :  printf("Unknown option '%s'\n", argv[i]);
	exit(0);
      }
    i++;
  }
  Matrix44f matrix;
  /* Read xf file if given... */
  if (xfname) {
    FILE *xf = fopen(xfname, "r");
    if (xf == NULL) {
      fprintf(stderr, "Error, couldn't open .xf file %s\n", xfname);
      Usage();
      exit(-1);
    }
    for (int i=0; i < 4; i++) {
      float a,b,c,d;
      fscanf(xf, "%f %f %f %f\n", &a, &b, &c, &d);
      matrix.ElementAt(i,0)=a;
      matrix.ElementAt(i,1)=b;
      matrix.ElementAt(i,2)=c;
      matrix.ElementAt(i,3)=d;
      //xfmat.setElem(i,0,a);
      //xfmat.setElem(i,1,b);
      //xfmat.setElem(i,2,c);
      //xfmat.setElem(i,3,d);
    }
    fclose(xf);
  }

  int t1,t2,t3;
  MyTriEdgeCollapse::Params()=qparams;

  int err=vcg::tri::io::ImporterPLY<CMeshO>::Open(cm,argv[1]);
  if(err) {
    printf("Unable to open mesh %s : '%s'\n",argv[1],vcg::tri::io::Importer<CMeshO>::ErrorMsg(err));
    exit(-1);
  }
  printf("Mesh loaded Verts: %d Faces: %d \n",cm.vn,cm.fn);
  tri::UpdateBounding<CMeshO>::Box(cm);

  if(EdgeLenClean){
    if(edgelenthresh.substr(edgelenthresh.size()-1) == "%")
      EdgeLen=  ( cm.bbox.Diag()* 0.01 *atoi(edgelenthresh.substr(0,edgelenthresh.size()-1).c_str()));
    else
      EdgeLen=atof(edgelenthresh.c_str());
    printf("EdgeLen %f\n",EdgeLen);
    tri::Clean<CMeshO>::RemoveFaceOutOfRangeEdgeSel<false>(cm,0,EdgeLen );
  }
  tri::UpdateNormals<CMeshO>::PerVertexNormalizedPerFace(cm);
  tri::UpdateBounding<CMeshO>::Box(cm);
  


  
  
  if(QualityClean){
    CMeshO::VertexIterator vi;
    for(vi=cm.vert.begin();vi!=cm.vert.end();++vi)
      if(!(*vi).IsD() && (*vi).Q()<QualityCleanVal){
	(*vi).SetD();
	cm.vn--;
      } 
 
    CMeshO::FaceIterator fi;
    for(fi=cm.face.begin();fi!=cm.face.end();++fi) 
      if(!(*fi).IsD())
	if((*fi).V(0)->IsD() ||(*fi).V(1)->IsD() ||(*fi).V(2)->IsD() ) {
	  (*fi).SetD();
	  --cm.fn;
	} 
    printf("After Quality Clean Vertex: %d Faces: %d\n",cm.vn,cm.fn);
  }
 


  if(SizeClean){
    cm.face.EnableFFAdjacency();
    cm.face.EnableMark();
    tri::UpdateTopology<CMeshO>::FaceFace(cm);
    tri::UpdateFlags<CMeshO>::FaceBorderFromFF(cm);
    RemoveSmallConnectedComponentsDiameter<CMeshO>(cm,minDiaSmallCC);
    printf("After Size Clean Vertex: %d Faces: %d\n",cm.vn,cm.fn);
  }
 

  int dup= tri::Clean<CMeshO>::RemoveDuplicateVertex(cm);
  int unref= tri::Clean<CMeshO>::RemoveUnreferencedVertex(cm);
  int deg= vcg::tri::Clean<CMeshO>::RemoveDegenerateFace(cm);
  
  printf("Removed %i degenrate faces %i duplicate and %i unreferenced vertices from mesh\n",deg,dup,unref);
 
  
  tri::UpdateNormals<CMeshO>::PerVertexNormalizedPerFace(cm);
  tri::UpdateBounding<CMeshO>::Box(cm);
  
 
  cm.face.EnableVFAdjacency();
  tri::UpdateTopology<CMeshO>::VertexFace(cm);	
  tri::UpdateFlags<CMeshO>::FaceBorderFromNone(cm);
  
  if(tgtStr.substr(tgtStr.size()-1) == "%")
    TargetFaceNum= (int) rint( cm.fn * 0.01 *atoi(tgtStr.substr(0,tgtStr.size()-1).c_str()));
  else
    TargetFaceNum=atoi(tgtStr.c_str());
  
  if(TargetFaceNum != 0){
    printf("Reducing it to %i\n",TargetFaceNum);
 
    math::Quadric<double> QZero;
    QZero.Zero();
    QuadricTemp TD(cm.vert);
    QHelper::TDp()=&TD;

    TD.Start(QZero);
 
    MyTriEdgeCollapse::SetHint(MyTriEdgeCollapse::HNHasVFTopology);
    MyTriEdgeCollapse::SetHint(MyTriEdgeCollapse::HNHasBorderFlag);

    vcg::LocalOptimization<CMeshO> DeciSession(cm);
    t1=clock();	
    printf("Initializing simplification\n");
    DeciSession.Init<MyTriEdgeCollapse >();
    t2=clock();	


    DeciSession.SetTargetSimplices(TargetFaceNum);
    DeciSession.SetTimeBudget(0.01f); // this allow to update the progress bar 10 time for sec...

    int startFn=cm.fn;
    int faceToDel=cm.fn-TargetFaceNum;
    while( DeciSession.DoOptimization() && cm.fn>TargetFaceNum )
      {
	printf("Percent %2d%%\r",(100-100*(cm.fn-TargetFaceNum)/(faceToDel)));
	fflush(stdout);
      };
    t3=clock();
    DeciSession.Finalize<MyTriEdgeCollapse >();
    printf("Simplified Mesh Verts: %d Faces: %d Error %g \n",cm.vn,cm.fn,DeciSession.currMetric);
 
 
  }
  
 
  if(FillHoles){
    cm.face.EnableFFAdjacency();
    cm.face.EnableMark();
    tri::UpdateTopology<CMeshO>::FaceFace(cm);
    tri::UpdateFlags<CMeshO>::FaceBorderFromFF(cm);
    int nonManif=tri::Clean<CMeshO>::RemoveNonManifoldFace(cm);


    tri::Hole<CMeshO>::EarCuttingIntersectionFill<tri::SelfIntersectionEar< CMeshO> >(cm,MaxHoleSize,false);
    assert(tri::Clean<CMeshO>::IsFFAdjacencyConsistent(cm));
    tri::UpdateNormals<CMeshO>::PerVertexNormalized(cm);	    
   printf("After Hole filling Verts: %d Faces: %d\n",cm.vn,cm.fn);
  }
  if(FlipMesh)
    tri::Clean<CMeshO>::FlipMesh(cm);
  
  printf("Completed in %.2f sec (%.2f init + %.2f proc)\n",(t3-t1)/(double)CLOCKS_PER_SEC,(t2-t1)/(double)CLOCKS_PER_SEC,(t3-t2)/(double)CLOCKS_PER_SEC);
  string filename=string(argv[2]);
  string format=filename.substr(filename.length()-3);

  if(format == "stl")	{
    int result = vcg::tri::io::ExporterSTL<CMeshO>::Save(cm,filename.c_str());
    if(result!=0){
      printf("Saving Error %s for file %s\n", vcg::tri::io::ExporterSTL<CMeshO>::ErrorMsg(result),filename.c_str());
      return -1;
    }
   
  }else if(format == "ply")	{
    int result = vcg::tri::io::ExporterPLY<CMeshO>::Save(cm,filename.c_str());
    if(result!=0){
      printf("Saving Error %s for file %s\n", vcg::tri::io::ExporterPLY<CMeshO>::ErrorMsg(result),filename.c_str());
      return -1;
    }
  }else {
    printf("Format %s unknown\n",format.c_str());
    return -1;
  }

  return 0;

}
