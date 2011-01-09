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

    CMeshO cm;
    int err=tri::io::ImporterPLY<CMeshO>::Open(cm,argv[1]);



    //vcg::tri::UpdateTopology<CMeshO>::FaceFace(cm);
    // vcg::tri::UpdateFlags<CMeshO>::FaceBorderFromFF(cm);
    //vcg::tri::UpdateNormals<CMeshO>::PerVertexNormalized(cm);

    if(err) {
        fprintf(stderr,"Unable to open mesh %s : '%s'\n",argv[1],vcg::tri::io::ImporterPLY<CMeshO>::ErrorMsg(err));
        exit(-1);
    }


     int total = tri::Clean<CMeshO>::MergeCloseVertex(cm, threshold);
    bool binaryFlag =false;
    vcg::tri::io::PlyInfo pi;

    int result = tri::io::ExporterPLY<CMeshO>::Save(cm,outfile.c_str(),binaryFlag);
    if(result !=0) {
        fprintf(stderr,"Unable to open write %s : '%s'\n",outfile.c_str(),vcg::tri::io::ExporterPLY<CMeshO>::ErrorMsg(result));
        exit(-1);
    }
}
