#include <vector>
#include <limits>

#include <stdio.h>
#include <stdlib.h>

using namespace std;

// stuff to define the mesh
#include <vcg/simplex/vertex/base.h>
#include <vcg/simplex/face/base.h>
#include <vcg/simplex/edge/base.h>


#include <vcg/math/quadric.h>
#include <vcg/complex/algorithms/clean.h>

// io
#include <wrap/io_trimesh/import.h>
#include <wrap/io_trimesh/export_ply.h>
#include <wrap/io_trimesh/export_obj.h>

// *include the algorithms for updating: */
#include <vcg/complex/algorithms/update/topology.h>	/* topology */
#include <vcg/complex/algorithms/update/bounding.h>	/* bounding box */
#include <vcg/complex/algorithms/update/normal.h>		/* normal */
#include<vcg/complex/append.h>
#include <vcg/complex/algorithms/attribute_seam.h>

// local optimization
#include <vcg/complex/algorithms/local_optimization.h>
#include <vcg/complex/algorithms/local_optimization/tri_edge_collapse_quadric.h>
#include <osg/Vec2>
#include <osgDB/WriteFile>
#include "TexturingQuery.h"
using namespace vcg;
using namespace tri;
#include "meshmodel.h"

#include "quadric_tex_simp.h"

void QuadricTexSimplification(CMeshO &m,int  TargetFaceNum, bool Selected, CallBackPos *cb);

inline void ExtractVertex(const CMeshO & srcMesh, const CMeshO::FaceType & f, int whichWedge, const CMeshO & dstMesh, CMeshO::VertexType & v)
{
    (void)srcMesh;
    (void)dstMesh;
    // This is done to preserve every single perVertex property
    // perVextex Texture Coordinate is instead obtained from perWedge one.
    v.ImportData(*f.cV(whichWedge));
    v.T() = f.cWT(whichWedge);
}
inline bool CompareVertex(const CMeshO & m, const CMeshO::VertexType & vA, const CMeshO::VertexType & vB)
{
    (void)m;
    return (vA.cT() == vB.cT());
}
void writeOSG(string fname,CMeshO &m){
    cout << "Pre expand "<< m.fn<< " verts: "<<m.vn<<endl;
      tri::AttributeSeam::SplitVertex(m, ExtractVertex, CompareVertex);
      cout << "Post expand "<< m.fn<< " verts: "<<m.vn<<endl;

    osg::ref_ptr<osg::Vec3Array> verts=new osg::Vec3Array;
    osg::ref_ptr<osg::Vec2Array> texcoords=new osg::Vec2Array;
    osg::ref_ptr<osg::Vec2Array> auxData=new osg::Vec2Array;

    osg::ref_ptr<osg::DrawElementsUInt> tri=new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES, 0);
     CMeshO::VertexPointer  vp;
     CMeshO::VertexIterator vi;
     CMeshO::FacePointer fp;
    CMeshO::FaceIterator fi;

    SimpleTempData<CMeshO::VertContainer,int> indices(m.vert);
    int j;
    for(j=0,vi=m.vert.begin();vi!=m.vert.end();++vi){
            vp=&(*vi);
            indices[vi] = j++;
            verts->push_back(osg::Vec3(vp->P()[0],vp->P()[1],vp->P()[2]));
            if( HasPerVertexTexCoord(m) )
            {
               texcoords->push_back(osg::Vec2(vp->T().u(),vp->T().v()));
            }
            osg::Vec2 aux(-1,-1);
            if( HasPerVertexQuality(m)  )
            {
                aux.y()=vp->Q();
            }
            if( HasPerVertexColor(m)  )
            {
                aux.x()=vp->C()[0]/255.0;
            }
            auxData->push_back(aux);


    }
    for(j=0,fi=m.face.begin();fi!=m.face.end();++fi)
            {


                    fp=&(*fi);
                    if( ! fp->IsD() )
                    {
                        for(int k=0; k<3; k++)
                            tri->push_back(indices[fp->cV(k)]);
                    }
            }
    osg::ref_ptr<osg::Geode> geode=new osg::Geode;
    osg::ref_ptr<osg::Geometry> geo=new osg::Geometry;
    geo->setVertexArray(verts);
    geo->setTexCoordArray(0,texcoords);
    geo->setTexCoordArray(1,auxData);

    geo->addPrimitiveSet(tri);
    geode->addDrawable(geo);
    osgDB::writeNodeFile(*geode,fname);
}
void splitMeshOuput(CMeshO &m,const char *basename, int splitting){
    tri::AttributeSeam::SplitVertex(m, ExtractVertex, CompareVertex);

    char fname[1024];
    int mesh_count=0;
    int vertexLeft=m.vn;
    while(vertexLeft>0){
        CMeshO destMesh;

        int selectedV=0;
        CMeshO::VertexIterator vi;
        CMeshO::FaceIterator   fi;
        for(vi=m.vert.begin();vi!=m.vert.end() && selectedV<splitting;++vi){
            if(!(*vi).IsD() && !(*vi).IsS() ){
                (*vi).SetS();
                selectedV++;
            }
        }



        // select all points involved
      //  tri::UpdateSelection<CMeshO>::ClearVertex(mesh);
        tri::UpdateSelection<CMeshO>::FaceFromVertexStrict(m);
        // tri::UpdateSelection<CMeshO>::ClearVertex(mesh);
     //    tri::UpdateSelection<CMeshO>::VertexFromFaceLoose(mesh);

        tri::Append<CMeshO,CMeshO>::Mesh(destMesh, m, true);

        int numFacesSel = tri::UpdateSelection<CMeshO>::CountFace(m);
        int numVertSel  = tri::UpdateSelection<CMeshO>::CountVertex(m);

        printf("Moving faces %d verts %d\n",numFacesSel,numVertSel);
        tri::UpdateSelection<CMeshO>::ClearVertex(m);
        tri::UpdateSelection<CMeshO>::VertexFromFaceStrict(m);
        for(fi=m.face.begin();fi!=m.face.end();++fi)
            if(!(*fi).IsD() && (*fi).IsS() )
                tri::Allocator<CMeshO>::DeleteFace(m,*fi);
        for(vi=m.vert.begin();vi!=m.vert.end();++vi)
            if(!(*vi).IsD() && (*vi).IsS() )
                tri::Allocator<CMeshO>::DeleteVertex(m,*vi);
        vcg::tri::Allocator<CMeshO>::CompactFaceVector(m);

        tri::UpdateSelection<CMeshO>::ClearVertex(m);
        tri::UpdateSelection<CMeshO>::ClearFace(m);

        sprintf(fname,"%s-%04d.obj",basename,mesh_count++);
        vcg::tri::io::PlyInfo pi;
        pi.mask |= vcg::tri::io::Mask::IOM_WEDGTEXCOORD;
        pi.mask |= vcg::tri::io::Mask::IOM_VERTCOLOR;
        pi.mask |= vcg::tri::io::Mask::IOM_FACEQUALITY;
        int mask = pi.mask;
        destMesh.textures.resize(0);
        mask &= ~vcg::tri::io::Mask::IOM_WEDGTEXCOORD;
        mask |= vcg::tri::io::Mask::IOM_VERTNORMAL;
        mask |= vcg::tri::io::Mask::IOM_VERTTEXCOORD;

        vcg::tri::UpdateFlags<CMeshO>::VertexClear(destMesh,CMeshO::VertexType::SELECTED);
        vcg::tri::UpdateFlags<CMeshO>::FaceClear(destMesh,CMeshO::FaceType::SELECTED);
        tri::UpdateBounding<CMeshO>::Box(destMesh);						// updates bounding box
                             for(fi=destMesh.face.begin();fi!=destMesh.face.end();++fi)	// face normals
                                     face::ComputeNormalizedNormal(*fi);
                             tri::UpdateNormals<CMeshO>::PerVertex(destMesh);				// vertex normals
  printf("Writing %s verts: %d faces : %d %s\n",fname,destMesh.vn,destMesh.fn,fname);


        vcg::tri::io::ExporterOBJ<CMeshO>::Save(destMesh,fname, mask);
        printf("Writing %s verts: %d faces : %d %s\n",fname,destMesh.vn,destMesh.fn,fname);
        vertexLeft=0;
        for(vi=m.vert.begin();vi!=m.vert.end();++vi){
            if(!(*vi).IsD())
                vertexLeft++;
        }
        printf("Vert left %d\n",vertexLeft);
        //  currentMesh->clearDataMask(MeshModel::MM_FACEFACETOPO | MeshModel::MM_FACEFLAGBORDER);
        vcg::tri::UpdateFlags<CMeshO>::VertexClear(m,CMeshO::VertexType::SELECTED);
              vcg::tri::UpdateFlags<CMeshO>::FaceClear(m,CMeshO::FaceType::SELECTED);
     //   vcg::tri::UpdateFlags<CMeshO>::FaceClear(destMesh,CMeshO::FaceType::SELECTED);
        //Log("Moved %i faces and %i vertices to layer %i", numFacesSel, numVertSel, md.meshList.size());
    }
    sprintf(fname,"%s.txt");

    FILE *fp=fopen(fname,"w");
    fprintf(fp,"%d\n",mesh_count-1);
    fclose(fp);
}
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

    //int t0=clock();
    int err=vcg::tri::io::ImporterPLY<CMeshO>::Open(mesh,argv[1]);
    if(FinalSize ==0 )
        FinalSize=mesh.fn;
    if(err)
    {
        printf("Unable to open mesh %s : '%s'\n",argv[1],vcg::tri::io::Importer<CMeshO>::ErrorMsg(err));
        exit(-1);
    }
    printf("mesh loaded %s %d %d \n",argv[1],mesh.vn,mesh.fn);
 //   int FinalSize=FinalSizePer*mesh.fn;
    TriEdgeCollapseQuadricTexParameter &qparams = MyTriEdgeCollapseQTex::Params() ;
    MyTriEdgeCollapseQTex::SetDefaultParams();
    qparams.QualityThr  =.5;
    qparams.QualityCheck=true;
    qparams.ExtraTCoordWeight=1.0;
    float TargetError=numeric_limits<float>::max();
    bool CleaningFlag =false;
    bool UseVertStop=false;
    bool FlipFlag=false;

    int splitting=0;
    char basename[1024];
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
                case 'F' : FlipFlag=true; break;

                                case 'V': UseVertStop=true;break;
                                case 's' :	splitting = atoi(argv[i]+2);  printf("Splitting output at %d\n",atoi(argv[i]+2)); break;
                                case 'u' :	strcpy(basename ,argv[i]+2);  printf("Splitting output fn %s\n",basename); break;


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
    pi.mask |= vcg::tri::io::Mask::IOM_VERTCOLOR;
    pi.mask |= vcg::tri::io::Mask::IOM_FACEQUALITY;

   // pi.mask |= vcg::tri::io::Mask::IOM_WEDGCOLOR;

 //  vcg::tri::io::ExporterPLY<CMeshO>::Save(mesh,"assy.ply",true,pi);
// exit(-1);
  //  vcg::tri::io::ImporterPLY<CMeshO>::Open(mesh,argv[2],pi);

//exit(-1);
    if(CleaningFlag){
        int dup2 = tri::Clean<CMeshO>::RemoveDuplicateFace(mesh);
        int dup = tri::Clean<CMeshO>::RemoveDuplicateVertex(mesh);
        int unref =  tri::Clean<CMeshO>::RemoveUnreferencedVertex(mesh);
        printf("Removed %i duplicate vetex %i duplicate faces  and %i unreferenced vertices from mesh \n",dup,dup2,unref);
    }
    if(FlipFlag)
        tri::Clean<CMeshO>::FlipMesh(mesh);
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
#if 0
    QuadricTexSimplification(mesh,FinalSize,false,NULL);
#else
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

   // DeciSession.SetTargetSimplices(FinalSize);
    DeciSession.SetTimeBudget(0.5f);
    if(TargetError< numeric_limits<float>::max() ) DeciSession.SetTargetMetric(TargetError);

    if(UseVertStop){
        DeciSession.SetTargetVertices(FinalSize);
        printf("Using Vertex Stop\n");
        while(DeciSession.DoOptimization() && mesh.vn>FinalSize && DeciSession.currMetric < TargetError)
          printf("Current Mesh size %7i heap sz %9i err %9g \r",mesh.vn,DeciSession.h.size(),DeciSession.currMetric);
    }
    else{
        DeciSession.SetTargetSimplices(FinalSize);
        while(DeciSession.DoOptimization() && mesh.fn>FinalSize && DeciSession.currMetric < TargetError)
          printf("Current Mesh size %7i heap sz %9i err %9g \r",mesh.fn,DeciSession.h.size(),DeciSession.currMetric);
  }


//    while(DeciSession.DoOptimization() && mesh.fn>FinalSize && DeciSession.currMetric < TargetError)
//        printf("Current Mesh size %7i heap sz %9i err %9g \r",mesh.fn,DeciSession.h.size(),DeciSession.currMetric);
    int t3=clock();
    printf("mesh  vn: %d fn: %d Error %g \n",mesh.vn,mesh.fn,DeciSession.currMetric);
    printf("\nCompleted in (%i+%i) msec\n",t2-t1,t3-t2);
#endif
  //  mesh.textures.resize(2);
    //Needed to force the ply to output tex id
    std::pair<int,int> delInfo= tri::Clean<CMeshO>::RemoveSmallConnectedComponentsSize(mesh,25);

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

    writeOSG((osgDB::getNameLessExtension(argv[2])+".ive").c_str(),mesh);

    return 0;

}
