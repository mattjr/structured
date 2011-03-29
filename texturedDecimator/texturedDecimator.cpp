#include <vector>
#include <limits>

#include <stdio.h>
#include <stdlib.h>

using namespace std;

// stuff to define the mesh
#include <vcg/simplex/vertex/base.h>
#include <vcg/simplex/face/base.h>
#include <vcg/simplex/edge/base.h>
#include <vcg/complex/trimesh/base.h>

#include <vcg/math/quadric.h>
#include <vcg/complex/trimesh/clean.h>
#include <vcg/container/simple_temporary_data.h>
// io
#include <wrap/io_trimesh/import.h>
#include <wrap/io_trimesh/export_ply.h>

// update
#include <vcg/complex/trimesh/update/topology.h>

// local optimization
#include <vcg/complex/local_optimization.h>
#include <vcg/complex/local_optimization/tri_edge_collapse_quadric.h>
#include "meshmodel.h"
#include <osg/Vec2>
#include "TexturingQuery.h"
using namespace vcg;
using namespace tri;


#include "quadric_tex_simp.h"

void QuadricTexSimplification(CMeshO &m,int  TargetFaceNum, bool Selected, CallBackPos *cb);


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
            "     -Q[y|n]  Use or not Quality Threshold (default yes)\n"
            "     -N[y|n]  Use or not Normal Threshold (default no)\n"
            "     -A[y|n]  Use or not Area Weighted Quadrics (default yes)\n"
            "     -O[y|n]  Use or not vertex optimal placement (default yes)\n"
            "     -S[y|n]  Use or not Scale Independent quadric measure(default yes) \n"
            "     -B[y|n]  Preserve or not mesh boundary (default no)\n"
            "     -T[y|n]  Preserve or not Topology (default no)\n"
            "     -H[y|n]  Use or not Safe Heap Update (default no)\n"
            "     -P       Before simplification, remove duplicate & unreferenced vertices\n"
            );
    exit(-1);
}

// mesh to simplify
CMeshO mesh;

int main(int argc ,char**argv){
    if(argc<4) Usage();
   // std::string in_texcoord_file=argv[3];
   // std::string out_texcoord_file=argv[4];
    int FinalSize=atoi(argv[3]);
    osg::Vec2Array *texCoords=new osg::Vec2Array;
    osg::Vec4Array *ids=new osg::Vec4Array;

    /*if(!loadCached(in_texcoord_file,ids,texCoords)){
        std::cerr << "Can't load tex file\n";
        return -1;
    }
    string mf=argv[1];
    TexturedSource *sourceModel=new TexturedSource(vpb::Source::MODEL,argv[1],"bbox-"+mf.substr(0,mf.size()-9)+".ply.txt");
    idmap_t allIds;
   idbackmap_t  backMap;
    calcAllIdsBack(ids,allIds,backMap);
*/
    //int t0=clock();
    int err=vcg::tri::io::Importer<CMeshO>::Open(mesh,argv[1]);
    if(err)
    {
        printf("Unable to open mesh %s : '%s'\n",argv[1],vcg::tri::io::Importer<CMeshO>::ErrorMsg(err));
        exit(-1);
    }
    printf("mesh loaded %d %d \n",mesh.vn,mesh.fn);
 //   int FinalSize=FinalSizePer*mesh.fn;
    TriEdgeCollapseQuadricTexParameter &qparams = MyTriEdgeCollapseQTex::Params() ;
    MyTriEdgeCollapseQTex::SetDefaultParams();
    qparams.QualityThr  =.5;
    qparams.QualityCheck=true;
    float TargetError=numeric_limits<float>::max();
    bool CleaningFlag =false;
    // parse command line.
    for(int i=4; i < argc;)
    {
        if(argv[i][0]=='-')
            switch(argv[i][1])
            {
            case 'H' : MyTriEdgeCollapseQTex::Params().SafeHeapUpdate=true; printf("Using Safe heap option\n"); break;
            case 'Q' : if(argv[i][2]=='y') { qparams.QualityCheck	= true;  printf("Using Quality Checking\n");	}
                else { qparams.QualityCheck	= false; printf("NOT Using Quality Checking\n");	}                break;
				case 'N' : if(argv[i][2]=='y') { qparams.NormalCheck	= true;  printf("Using Normal Deviation Checking\n");	}
                                    else { qparams.NormalCheck	= false; printf("NOT Using Normal Deviation Checking\n");	}        break;
				case 'O' : if(argv[i][2]=='y') { qparams.OptimalPlacement	= true;  printf("Using OptimalPlacement\n");	}
                                    else { qparams.OptimalPlacement	= false; printf("NOT Using OptimalPlacement\n");	}        break;
				case 'S' : if(argv[i][2]=='y') { qparams.ScaleIndependent	= true;  printf("Using ScaleIndependent\n");	}
                                    else { qparams.ScaleIndependent	= false; printf("NOT Using ScaleIndependent\n");	}        break;
				case 'B' : if(argv[i][2]=='y') { qparams.PreserveBoundary	= true;  printf("Preserving Boundary\n");	}
                                    else { qparams.PreserveBoundary	= false; printf("NOT Preserving Boundary\n");	}        break;
				case 'T' : if(argv[i][2]=='y') { qparams.PreserveTopology	= true;  printf("Preserving Topology\n");	}
                                    else { qparams.PreserveTopology	= false; printf("NOT Preserving Topology\n");	}        break;
				case 'q' :	qparams.QualityThr	= atof(argv[i]+2);	           printf("Setting Quality Thr to %f\n",atof(argv[i]+2)); 	 break;			
                                    //				case 'n' :	qparams.NormalThrRad = math::ToRad(atof(argv[i]+2));  printf("Setting Normal Thr to %f deg\n",atof(argv[i]+2)); break;
				case 'b' :	qparams.BoundaryWeight  = atof(argv[i]+2);			printf("Setting Boundary Weight to %f\n",atof(argv[i]+2)); break;		
				case 'e' :	TargetError = float(atof(argv[i]+2));			printf("Setting TargetError to %g\n",atof(argv[i]+2)); break;		
				case 'P' :	CleaningFlag=true;  printf("Cleaning mesh before simplification\n"); break;	

				default  :  printf("Unknown option '%s'\n", argv[i]);
                                exit(0);
                            }
        i++;
    }
    //  mesh.mask |= vcg::tri::io::Mask::IOM_VERTTEXCOORD;

    //mesh.mask |= vcg::tri::io::Mask::IOM_WEDGTEXCOORD;
  /*  CMeshO::VertexPointer  vp;
    CMeshO::VertexIterator vi;
    vcg::SimpleTempData<CMeshO::VertContainer,int> indices(mesh.vert);
    int j;
    for(j=0,vi=mesh.vert.begin();vi!=mesh.vert.end();++vi,++j){
        vp=&(*vi);
        indices[vi] = j;
        //vp->EnableMark();

    }
    CMeshO::FaceIterator   fi;
    printf("idx %d t %d \n",mesh.vn ,texCoords->size());
//Need to redensifiy mesh
    string path="/home/mattjr/auvdata/r20090804_084719_scott_25_dense_repeat_auv5_deep/renav20090804/mesh/cache-tex/";
    mesh.textures.resize(allIds.size());
   idmap_t::const_iterator end = allIds.end();
    for (idmap_t::const_iterator it = allIds.begin(); it != end; ++it)
    {   if(sourceModel->_cameras.count(it->first))
         mesh.textures[it->second]=path+sourceModel->_cameras[it->first].filename;
    }

    for(fi=mesh.face.begin(); fi!=mesh.face.end(); ++fi)
        for(unsigned int k=0;k<3;k++){
        int idx=indices[(*fi).cV(k)];
        assert(idx <texCoords->size());
        osg::Vec2 texCoord=(*texCoords)[idx];
        osg::Vec4 id=(*ids)[idx];
    //    (*fi).EnableMark();

        (*fi).WT(k).u()=texCoord[0];
        (*fi).WT(k).v()=texCoord[1];
        (*fi).WT(k).n()=allIds[id[0]];
    }*/
    vcg::tri::io::PlyInfo pi;
    pi.mask |= vcg::tri::io::Mask::IOM_WEDGTEXCOORD;
   // pi.mask |= vcg::tri::io::Mask::IOM_WEDGCOLOR;

 //  vcg::tri::io::ExporterPLY<CMeshO>::Save(mesh,"assy.ply",true,pi);
// exit(-1);
  //  vcg::tri::io::ImporterPLY<CMeshO>::Open(mesh,argv[2],pi);

//exit(-1);
    if(CleaningFlag){
        int dup = tri::Clean<CMeshO>::RemoveDuplicateVertex(mesh);
        int unref =  tri::Clean<CMeshO>::RemoveUnreferencedVertex(mesh);
        printf("Removed %i duplicate and %i unreferenced vertices from mesh \n",dup,unref);
    }

    if(!tri::HasPerWedgeTexCoord(mesh))
    {
        printf("Warning: nothing have been done. Mesh has no Texture.\n");
        return false;
    }
    if ( ! tri::Clean<CMeshO>::HasConsistentPerWedgeTexCoord(mesh) ) {
        printf( "Mesh has some inconsistent tex coords (some faces without texture)\n");
    }
    printf("reducing it to %i\n",FinalSize);
  //  mesh.face.EnableMark();
    tri::UpdateTopology<CMeshO>::FaceFace(mesh);
    tri::UpdateFlags<CMeshO>::FaceBorderFromFF(mesh);
    tri::UpdateNormals<CMeshO>::PerVertexNormalizedPerFace(mesh);
    tri::UpdateBounding<CMeshO>::Box(mesh);
    tri::UpdateTopology<CMeshO>::VertexFace(mesh);
    tri::UpdateFlags<CMeshO>::FaceBorderFromNone(mesh);

    QuadricTexSimplification(mesh,FinalSize,false,NULL);
#if 0
    assert(tri::HasPerVertexMark(mesh));

    vcg::tri::UpdateBounding<CMeshO>::Box(mesh);
    math::Quadric<double> QZero;
    QZero.SetZero();
    QuadricTemp TD3(mesh.vert,QZero);
    QuadricTexHelper::TDp3()=&TD3;
    std::vector <std::pair<vcg::TexCoord2<float>,Quadric5<double> > > qv;
    Quadric5Temp TD(mesh.vert,qv);
    QuadricTexHelper::TDp()=&TD;


    // decimator initialization
    vcg::LocalOptimization<CMeshO> DeciSession(mesh);

    int t1=clock();
    DeciSession.Init<MyTriEdgeCollapseQTex >();
    int t2=clock();
    printf("Initial Heap Size %i\n",DeciSession.h.size());

    DeciSession.SetTargetSimplices(FinalSize);
    DeciSession.SetTimeBudget(0.5f);
    if(TargetError< numeric_limits<float>::max() ) DeciSession.SetTargetMetric(TargetError);

    while(DeciSession.DoOptimization() && mesh.fn>FinalSize && DeciSession.currMetric < TargetError)
        printf("Current Mesh size %7i heap sz %9i err %9g \r",mesh.fn,DeciSession.h.size(),DeciSession.currMetric);
    int t3=clock();
    printf("mesh  %d %d Error %g \n",mesh.vn,mesh.fn,DeciSession.currMetric);
    printf("\nCompleted in (%i+%i) msec\n",t2-t1,t3-t2);
#endif
  //  mesh.textures.resize(2);
    //Needed to force the ply to output tex id
    vcg::tri::io::ExporterPLY<CMeshO>::Save(mesh,argv[2],true,pi);
   /* std::string hash=getHash(argv[2]);

    vcg::SimpleTempData<CMeshO::VertContainer,int> indices2(mesh.vert);

    for(j=0,vi=mesh.vert.begin();vi!=mesh.vert.end();++vi,++j){
        vp=&(*vi);
        indices2[vi] = j;
    }
    texCoords->resize(mesh.vn);
    ids->resize(mesh.vn);
    for(fi=mesh.face.begin(); fi!=mesh.face.end(); ++fi)
        for(unsigned int k=0;k<3;k++){
        int idx=indices[(*fi).cV(k)];
       (*texCoords)[idx][0]=  (*fi).WT(k).u();
       (*texCoords)[idx][1]=  (*fi).WT(k).v();
       osg::Vec4 v(-1,-1,-1,-1);
       v[0]=backMap[(*fi).WT(k).n()];
       (*ids)[idx]=v;
       printf("FFF %f %f\n",v[0],(*fi).WT(k).n());
    }
    writeCached(out_texcoord_file,hash,ids,texCoords);
    vcg::tri::io::ExporterPLY<CMeshO>::Save(mesh,"assy2.ply",true,pi);
*/
    return 0;

}
