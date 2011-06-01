#include <osg/ArgumentParser>
#include <stdio.h>
#include <vcg/complex/trimesh/clean.h>
#include <wrap/io_trimesh/import_ply.h>
#include <wrap/io_trimesh/export_ply.h>

#include "meshmodel.h"
using namespace vcg;

int main(int argc ,char**argv){
    osg::ArgumentParser argp(&argc,argv);

    if(argp.argc() < 3){
        fprintf(stderr, "Usage  meshfile -outfile filename -thresh threshold \n");
        return -1;
    }
    double threshold=0.1;
    std::string outfile="out.ply";

    argp.read("-thresh",threshold);
    argp.read("-out",outfile);
    bool tex=argp.read("-tex");
    bool flip=argp.read("-flip");
    bool color=argp.read("-color");
    float CCPerc;
    bool clean=argp.read("-cleansize",CCPerc);
    vcg::tri::io::PlyInfo pi;
    if(tex)
     pi.mask |= vcg::tri::io::Mask::IOM_WEDGTEXCOORD;
    CMeshO cm;
    int err=tri::io::ImporterPLY<CMeshO>::Open(cm,argv[1],pi);

 	if (tex && ! tri::Clean<CMeshO>::HasConsistentPerWedgeTexCoord(cm) ) {
        printf( "Mesh has some inconsistent tex coords (some faces without texture)\n");
    }

    //vcg::tri::UpdateTopology<CMeshO>::FaceFace(cm);
    // vcg::tri::UpdateFlags<CMeshO>::FaceBorderFromFF(cm);
    //vcg::tri::UpdateNormals<CMeshO>::PerVertexNormalized(cm);

    if(err) {
        fprintf(stderr,"Unable to open mesh %s : '%s'\n",argv[1],vcg::tri::io::ImporterPLY<CMeshO>::ErrorMsg(err));
        exit(-1);
    }


     int total = tri::Clean<CMeshO>::MergeCloseVertex(cm, threshold);
    bool binaryFlag =false;
  if(flip){
         tri::Clean<CMeshO>::FlipMesh(cm);
     }
 if(clean){
         tri::UpdateBounding<CMeshO>::Box(cm);
       //  cm.face.EnableFFAdjacency();
         //  cm.face.EnableMark();
           tri::UpdateTopology<CMeshO>::FaceFace(cm);
           tri::UpdateFlags<CMeshO>::FaceBorderFromFF(cm);

         float minCC= CCPerc*cm.bbox.Diag();
         printf("Cleaning Min CC %f\n",minCC);
            std::pair<int,int> delInfo= tri::Clean<CMeshO>::RemoveSmallConnectedComponentsDiameter(cm,minCC);
            printf("Removed %d/%d\n",delInfo.second,delInfo.first);
            int dup= tri::Clean<CMeshO>::RemoveDuplicateVertex(cm);
             int unref= tri::Clean<CMeshO>::RemoveUnreferencedVertex(cm);
             int deg= vcg::tri::Clean<CMeshO>::RemoveDegenerateFace(cm);

     }
 vcg::tri::io::PlyInfo pi;
    if(tex)
     pi.mask |= vcg::tri::io::Mask::IOM_WEDGTEXCOORD;
    if(color)
     pi.mask |= vcg::tri::io::Mask::IOM_VERTCOLOR;
    int result = tri::io::ExporterPLY<CMeshO>::Save(cm,outfile.c_str(),binaryFlag,pi);

   // int result = tri::io::ExporterPLY<CMeshO>::Save(cm,outfile.c_str(),binaryFlag);
    if(result !=0) {
        fprintf(stderr,"Unable to open write %s : '%s'\n",outfile.c_str(),vcg::tri::io::ExporterPLY<CMeshO>::ErrorMsg(result));
        exit(-1);
    }
}
