#include <osg/ArgumentParser>
#include <stdio.h>
#include <vcg/complex/trimesh/clean.h>
#include <wrap/io_trimesh/import_ply.h>
#include <wrap/io_trimesh/export_ply.h>
#include "meshmodel.h"
#include "quadric_tex_simp.h"
#include "quadric_simp.h"

using namespace vcg;
using namespace std;
void QuadricTexSimplification(CMeshO &m,int  TargetFaceNum, bool Selected, CallBackPos *cb);
#if 0
int main(int argc ,char**argv){
    osg::ArgumentParser argp(&argc,argv);

    if(argp.argc() < 3){
        fprintf(stderr, "Usage  meshfile -outfile filename -thresh threshold \n");
        return -1;
    }
    double percentReduce=0.1;
    std::string outfile="out.ply";

    argp.read("-per",percentReduce);
    argp.read("-out",outfile);

    CMeshO cm;
    int err=tri::io::ImporterPLY<CMeshO>::Open(cm,argv[1]);



    //vcg::tri::UpdateTopology<CMeshO>::FaceFace(cm);
    // vcg::tri::UpdateFlags<CMeshO>::FaceBorderFromFF(cm);
    //vcg::tri::UpdateNormals<CMeshO>::PerVertexNormalized(cm);

    if(err) {
        fprintf(stderr,"Unable to open mesh %s : '%s'\n",argv[1],vcg::tri::io::ImporterPLY<CMeshO>::ErrorMsg(err));
        exit(-1);
    }


    cm.face.EnableFFAdjacency();
    cm.face.EnableMark();
    cm.vert.EnableMark();
    vcg::tri::UpdateTopology<CMeshO>::FaceFace(cm);

    cm.vert.EnableVFAdjacency();
    cm.face.EnableVFAdjacency();

    vcg::tri::UpdateTopology<CMeshO>::VertexFace(cm);

    fprintf(stderr,"Mesh loaded Verts: %d Faces: %d \n",cm.vn,cm.fn);




    tri::UpdateNormals<CMeshO>::PerVertexNormalizedPerFace(cm);
    tri::UpdateBounding<CMeshO>::Box(cm);


    /*
  int dup= tri::Clean<CMeshO>::RemoveDuplicateVertex(cm);
  int unref= tri::Clean<CMeshO>::RemoveUnreferencedVertex(cm);
  int deg= vcg::tri::Clean<CMeshO>::RemoveDegenerateFace(cm);
*/
    //fprintf(stderr,"Removed %i degen faces %i dup and %i unref vert\n",deg,dup,unref);
    /*if ( ! tri::Clean<CMeshO>::IsTwoManifoldFace(cm) ) {

       printf("freak out\n");
    }*/


    tri::UpdateNormals<CMeshO>::PerVertexNormalizedPerFace(cm);
    tri::UpdateBounding<CMeshO>::Box(cm);

    tri::UpdateTopology<CMeshO>::VertexFace(cm);
    tri::UpdateFlags<CMeshO>::FaceBorderFromNone(cm);

    int TargetFaceNum = cm.fn*percentReduce;
    printf("Target Face %d Orig %d\n",TargetFaceNum,cm.fn);
    if(!tri::HasPerWedgeTexCoord(cm))
    {
        cerr <<"Doing non textured simp. Mesh has no Texture.\n";
        tri::MyTriEdgeCollapse::SetDefaultParams();
        tri::TriEdgeCollapseQuadricParameter &pp = tri::MyTriEdgeCollapse::Params();
        pp.PreserveBoundary=true;
        QuadricSimplification(cm,TargetFaceNum,false,  NULL);

    }else{
        if ( ! tri::Clean<CMeshO>::HasConsistentPerWedgeTexCoord(cm) ) {
            cerr <<"Mesh has some inconsistent tex coords (some faces without texture)\n"; // text
            return false; // can't continue, mesh can't be processed
        }


        tri::MyTriEdgeCollapseQTex::SetDefaultParams();
        tri::TriEdgeCollapseQuadricTexParameter & pp=tri::MyTriEdgeCollapseQTex::Params();

        /* pp.QualityThr = par.getFloat("QualityThr");
     pp.ExtraTCoordWeight = par.getFloat("Extratcoordw");
     pp.OptimalPlacement = par.getBool("OptimalPlacement");
     pp.PreserveBoundary = par.getBool("PreserveBoundary");
     pp.QualityQuadric = par.getBool("PlanarQuadric");
    pp.NormalCheck = par.getBool("PreserveNormal");*/

        pp.PreserveBoundary =true;
        QuadricTexSimplification(cm,TargetFaceNum,false, NULL);
    }
    /* int nullFaces=tri::Clean<CMeshO>::RemoveFaceOutOfRangeArea(cm,0);
    if(nullFaces) printf( "PostSimplification Cleaning: Removed %d null faces\n", nullFaces);
    int deldupvert=tri::Clean<CMeshO>::RemoveDuplicateVertex(cm);
    if(deldupvert) printf( "PostSimplification Cleaning: Removed %d duplicated vertices\n", deldupvert);
    int delvert=tri::Clean<CMeshO>::RemoveUnreferencedVertex(cm);
    if(delvert) printf( "PostSimplification Cleaning: Removed %d unreferenced vertices\n",delvert);
    */
    tri::Allocator<CMeshO>::CompactVertexVector(cm);
    tri::Allocator<CMeshO>::CompactFaceVector(cm);
    tri::UpdateNormals<CMeshO>::PerVertexNormalizedPerFace(cm);
    tri::UpdateBounding<CMeshO>::Box(cm);
    bool binaryFlag =false;
    vcg::tri::io::PlyInfo pi;

    int result = tri::io::ExporterPLY<CMeshO>::Save(cm,outfile.c_str(),binaryFlag);
    if(result !=0) {
        fprintf(stderr,"Unable to open write %s : '%s'\n",outfile.c_str(),vcg::tri::io::ExporterPLY<CMeshO>::ErrorMsg(result));
        exit(-1);
    }
}
#endif
#if 1
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
            "     -e [# or %] Clip faces with edges longer then # or % of bbox\n"
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
    bool EdgeLenClean=false;
    float EdgeLen=0;
    int MaxHoleSize=0;
    bool FillHoles=false;
    int TargetFaceNum=0;
    string tgtStr(argv[3]);
    string edgelenthresh;

    // MyTriEdgeCollapse::SetDefaultParams();
    //TriEdgeCollapseQuadricParameter qparams=MyTriEdgeCollapse::Params();

    // parse command line.
    for(int i=4; i < argc;){
        if(argv[i][0]=='-')
            switch(argv[i][1]){
            case 'e' :	edgelenthresh=string(argv[i]+2);  EdgeLenClean=true; fprintf(stderr,"Cleaning edge len\n"); break;


            case 'F' :   FlipMesh	= true;  fprintf(stderr,"Flipping Mesh\n");
            case 'S' :	SizeClean=true; minDiaSmallCC=atof(argv[i]+2); fprintf(stderr,"Cleaning with connected componet diameter threshold of %f\n",atof(argv[i]+2)); break;
                break;
                /*      case 'N' : if(argv[i][2]=='y') { qparams.NormalCheck	= true; 	qparams.NormalThrRad = M_PI/4.0;  fprintf(stderr,"Using Normal Deviation Checking\n");	}	else { qparams.NormalCheck	= false; fprintf(stderr,"NOT Using Normal Deviation Checking\n");	}        break;
      case 'f' :	xfname =argv[i]+2;	           fprintf(stderr,"Transforming with %s file\n",xfname); 	 break;

      case 'O' : if(argv[i][2]=='y') { qparams.OptimalPlacement	= true;  fprintf(stderr,"Using OptimalPlacement\n");	}
        else { qparams.OptimalPlacement	= false; fprintf(stderr,"NOT Using OptimalPlacement\n");	}        break;

      case 'B' : if(argv[i][2]=='y') { qparams.FastPreserveBoundary	= true;  fprintf(stderr,"Preserving Boundary\n");	}
        else { qparams.FastPreserveBoundary	= false; fprintf(stderr,"NOT Preserving Boundary\n");	}        break;
      case 'T' : if(argv[i][2]=='y') { qparams.PreserveTopology	= true;  fprintf(stderr,"Preserving Topology\n");	}
        else { qparams.PreserveTopology	= false; fprintf(stderr,"NOT Preserving Topology\n");	}        break;
      case 'q' :	qparams.QualityThr	= atof(argv[i]+2);	           fprintf(stderr,"Setting Quality Thr to %f\n",atof(argv[i]+2)); 	 break;
      case 'n' :	qparams.NormalThrRad = math::ToRad(atof(argv[i]+2));  fprintf(stderr,"Setting Normal Thr to %f deg\n",atof(argv[i]+2)); break;

      case 'b' :	qparams.BoundaryWeight  = atof(argv[i]+2);			fprintf(stderr,"Setting Boundary Weight to %f\n",atof(argv[i]+2)); break;
      case 'P' :	CleaningFlag=true;  fprintf(stderr,"Cleaning mesh before simplification\n",atof(argv[i]+2)); break;

      case 'Q' :	QualityClean=true; QualityCleanVal=atof(argv[i]+2); fprintf(stderr,"Cleaning with quality threshold of %f\n",atof(argv[i]+2)); break;


        */
            case 'H' :	FillHoles=true; MaxHoleSize=atoi(argv[i]+2); fprintf(stderr,"Filling holes with a maximum size of %d\n",atoi(argv[i]+2)); break;
            default  :  fprintf(stderr,"Unknown option '%s'\n", argv[i]);
            //exit(0);
        }
        i++;
    }

    int t1,t2,t3;
    //MyTriEdgeCollapse::Params()=qparams;
    string fname(argv[1]);
    string format=fname.substr(fname.length()-3);
    int err;

    if(format == "ply")
        err=vcg::tri::io::ImporterPLY<CMeshO>::Open(cm,argv[1]);





    if(err) {
        fprintf(stderr,"Unable to open mesh %s : '%s'\n",argv[1],vcg::tri::io::ImporterPLY<CMeshO>::ErrorMsg(err));
        exit(-1);
    }

    cm.face.EnableFFAdjacency();
    cm.face.EnableMark();
    cm.vert.EnableMark();
    vcg::tri::UpdateTopology<CMeshO>::FaceFace(cm);

    cm.vert.EnableVFAdjacency();
    cm.face.EnableVFAdjacency();

    vcg::tri::UpdateTopology<CMeshO>::VertexFace(cm);

    fprintf(stderr,"Mesh loaded Verts: %d Faces: %d \n",cm.vn,cm.fn);




    tri::UpdateNormals<CMeshO>::PerVertexNormalizedPerFace(cm);
    tri::UpdateBounding<CMeshO>::Box(cm);











  int dup= tri::Clean<CMeshO>::RemoveDuplicateVertex(cm);
  int unref= tri::Clean<CMeshO>::RemoveUnreferencedVertex(cm);
  int deg= vcg::tri::Clean<CMeshO>::RemoveDegenerateFace(cm);

    //fprintf(stderr,"Removed %i degen faces %i dup and %i unref vert\n",deg,dup,unref);
    /*if ( ! tri::Clean<CMeshO>::IsTwoManifoldFace(cm) ) {

       printf("freak out\n");
    }
*/
    printf("Here\n");

    tri::UpdateNormals<CMeshO>::PerVertexNormalizedPerFace(cm);
    tri::UpdateBounding<CMeshO>::Box(cm);



    tri::UpdateTopology<CMeshO>::VertexFace(cm);
    tri::UpdateFlags<CMeshO>::FaceBorderFromNone(cm);

    if(tgtStr.substr(tgtStr.size()-1) == "%")
        TargetFaceNum= (int) rint( cm.fn * 0.01 *atoi(tgtStr.substr(0,tgtStr.size()-1).c_str()));
    else if(tgtStr.substr(tgtStr.size()-1) == "r"){
        double res=atof(tgtStr.substr(0,tgtStr.size()-1).c_str());
        if(res <= 0)
            TargetFaceNum=0;
        else{
            CMeshO::FaceIterator fi;
            double totalA=0;
            for(fi=cm.face.begin(); fi!=cm.face.end();++fi){
                double area=DoubleArea<CFaceO>(*fi);
                totalA+=area;
            }
            TargetFaceNum= (int) rint(( totalA) / res);
            fprintf(stderr,"Spacial Res %f area %f faces %d currently %d\n", res,totalA,TargetFaceNum,cm.fn);
        }
    }else
        TargetFaceNum=(int)rint(atof(tgtStr.c_str()));

    if(TargetFaceNum != 0){
        fprintf(stderr,"Reducing it to %i\n",TargetFaceNum);

        QuadricSimplification(cm,TargetFaceNum,false,NULL);//false,false,true,false);
    }

    if(FlipMesh)
        tri::Clean<CMeshO>::FlipMesh(cm);

    //fprintf(stderr,"Completed in %.2f sec (%.2f init + %.2f proc)\n",(t3-t1)/(double)CLOCKS_PER_SEC,(t2-t1)/(double)CLOCKS_PER_SEC,(t3-t2)/(double)CLOCKS_PER_SEC);
    string filename=string(argv[2]);


    format=filename.substr(filename.length()-3);
    vcg::tri::io::PlyInfo pi;

    if(format == "ply")	{
        int result = vcg::tri::io::ExporterPLY<CMeshO>::Save(cm,filename.c_str(),pi.mask);
        if(result!=0){
            fprintf(stderr,"Saving Error %s for file %s\n", vcg::tri::io::ExporterPLY<CMeshO>::ErrorMsg(result),filename.c_str());
            return -1;
        }
    }else{

        fprintf(stderr,"Format %s unknown\n",format.c_str());
        return -1;
    }

    return 0;

}
#endif
