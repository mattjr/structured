#if 1
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
    CMeshO cm;
    int err=tri::io::ImporterPLY<CMeshO>::Open(cm,argv[1]);

    if(!tri::HasPerWedgeTexCoord(cm))
    {
        printf("Warning: nothing have been done. Mesh has no Texture.\n");
        return false;
    }
    if ( ! tri::Clean<CMeshO>::HasConsistentPerWedgeTexCoord(cm) ) {
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
     if(flip){
         tri::Clean<CMeshO>::FlipMesh(cm);
     }
    bool binaryFlag =true;

    vcg::tri::io::PlyInfo pi;
    if(tex)
     pi.mask |= vcg::tri::io::Mask::IOM_WEDGTEXCOORD;
    int result = tri::io::ExporterPLY<CMeshO>::Save(cm,outfile.c_str(),binaryFlag,pi);
    if(result !=0) {
        fprintf(stderr,"Unable to open write %s : '%s'\n",outfile.c_str(),vcg::tri::io::ExporterPLY<CMeshO>::ErrorMsg(result));
        exit(-1);
    }
}
#endif
