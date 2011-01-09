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
        pp.PreserveBoundary=false;
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
    int nullFaces=tri::Clean<CMeshO>::RemoveFaceOutOfRangeArea(cm,0);
    if(nullFaces) printf( "PostSimplification Cleaning: Removed %d null faces\n", nullFaces);
    int deldupvert=tri::Clean<CMeshO>::RemoveDuplicateVertex(cm);
    if(deldupvert) printf( "PostSimplification Cleaning: Removed %d duplicated vertices\n", deldupvert);
    int delvert=tri::Clean<CMeshO>::RemoveUnreferencedVertex(cm);
    if(delvert) printf( "PostSimplification Cleaning: Removed %d unreferenced vertices\n",delvert);
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
