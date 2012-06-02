#include <osg/ArgumentParser>
#include <stdio.h>

// stuff to define the mesh
#include <vcg/simplex/vertex/base.h>
#include <vcg/simplex/face/base.h>
#include <vcg/simplex/edge/base.h>


#include <vcg/math/quadric.h>
#include <vcg/complex/algorithms/clean.h>

// io
#include <wrap/io_trimesh/import.h>
#include <wrap/io_trimesh/export_ply.h>

// *include the algorithms for updating: */
#include <vcg/complex/algorithms/update/topology.h>	/* topology */
#include <vcg/complex/algorithms/update/bounding.h>	/* bounding box */
#include <vcg/complex/algorithms/update/normal.h>		/* normal */
#include "meshmodel.h"
using namespace vcg;

int main(int argc ,char**argv){
    osg::ArgumentParser argp(&argc,argv);

    if(argp.argc() < 3){
        fprintf(stderr, "Usage  meshfile -outfile filename -thresh threshold \n");
        return -1;
    }
    double threshold= -1.0;
    std::string outfile="out.ply";

    argp.read("-thresh",threshold);
    argp.read("-out",outfile);
    bool tex=argp.read("-tex");
bool multtex=false;
 if(argp.read("-multtex")){
     tex=true;
     multtex=true;
 }


    vcg::tri::io::PlyInfo pi;
    bool flip=argp.read("-flip");
    bool color=argp.read("-color");
    float CCPerc;
    bool clean=argp.read("-cleansize",CCPerc);
    if(tex){
        pi.mask |= vcg::tri::io::Mask::IOM_WEDGTEXCOORD;

    }
    if(argp.read("-wedge"))
        pi.mask |= vcg::tri::io::Mask::IOM_FACEQUALITY;


    if(color)
        pi.mask |= vcg::tri::io::Mask::IOM_VERTCOLOR;

    CMeshO cm;
    int err=tri::io::ImporterPLY<CMeshO>::Open(cm,argv[1],pi);

    if(cm.fn >0){
        if ( tex && ! tri::Clean<CMeshO>::HasConsistentPerWedgeTexCoord(cm) ) {
            printf( "Mesh has some inconsistent tex coords (some faces without texture)\n");
        }

        //vcg::tri::UpdateTopology<CMeshO>::FaceFace(cm);
        // vcg::tri::UpdateFlags<CMeshO>::FaceBorderFromFF(cm);
        //vcg::tri::UpdateNormals<CMeshO>::PerVertexNormalized(cm);

        if(err) {
            fprintf(stderr,"Unable to open mesh %s : '%s'\n",argv[1],vcg::tri::io::ImporterPLY<CMeshO>::ErrorMsg(err));
            exit(-1);
        }

        if(threshold >0)
           int total = tri::Clean<CMeshO>::MergeCloseVertex(cm, threshold);
        if(flip){
            tri::Clean<CMeshO>::FlipMesh(cm);
        }
        if(clean){
            tri::UpdateBounding<CMeshO>::Box(cm);
            //  cm.face.EnableFFAdjacency();
            //  cm.face.EnableMark();



            int dup2= tri::Clean<CMeshO>::RemoveDuplicateFace(cm);

            int dup= tri::Clean<CMeshO>::RemoveDuplicateVertex(cm);
            int unref= tri::Clean<CMeshO>::RemoveUnreferencedVertex(cm);
            int deg= vcg::tri::Clean<CMeshO>::RemoveDegenerateFace(cm);
            float minCC= CCPerc*cm.bbox.Diag();
            printf("Cleaning Min CC %f\n",minCC);
            tri::UpdateTopology<CMeshO>::FaceFace(cm);
            tri::UpdateTopology<CMeshO>::FaceFace(cm);
            tri::UpdateFlags<CMeshO>::FaceBorderFromFF(cm);

            std::pair<int,int> delInfo= tri::Clean<CMeshO>::RemoveSmallConnectedComponentsDiameter(cm,minCC);
            printf("Removed %d/%d\n",delInfo.second,delInfo.first);

        }
    }

    bool binaryFlag =true;
    if(tex)
        pi.mask |= vcg::tri::io::Mask::IOM_WEDGTEXCOORD;
    if(color)
        pi.mask |= vcg::tri::io::Mask::IOM_VERTCOLOR;
    if(multtex){
    cm.textures.push_back("");
    cm.textures.push_back("");

    }
    int result = tri::io::ExporterPLY<CMeshO>::Save(cm,outfile.c_str(),binaryFlag,pi);

    //int result = tri::io::ExporterPLY<CMeshO>::Save(cm,outfile.c_str(),binaryFlag);
    if(result !=0) {
        fprintf(stderr,"Unable to open write %s : '%s'\n",outfile.c_str(),vcg::tri::io::ExporterPLY<CMeshO>::ErrorMsg(result));
        exit(-1);
    }
}
