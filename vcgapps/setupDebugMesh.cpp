// stuff to define the mesh
#include <vcg/simplex/vertex/base.h>
#include <vcg/simplex/face/base.h>
#include <vcg/simplex/edge/base.h>
#include <osg/ArgumentParser>


#include <vcg/math/quadric.h>
#include <vcg/complex/algorithms/clean.h>

// io
#include <wrap/io_trimesh/import.h>
#include <wrap/io_trimesh/export_ply.h>

// *include the algorithms for updating: */
#include <vcg/complex/algorithms/update/topology.h>	/* topology */
#include <vcg/complex/algorithms/update/bounding.h>	/* bounding box */
#include <vcg/complex/algorithms/update/normal.h>		/* normal */

// local optimization
#include <vcg/complex/algorithms/local_optimization.h>
#include <vcg/complex/algorithms/local_optimization/tri_edge_collapse_quadric.h>
#include <vcg/complex/algorithms/attribute_seam.h>
#include <osg/Vec3>

using namespace vcg;
using namespace tri;
#include "meshmodel.h"
#define MyMesh CMeshO
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
CMeshO mesh;
int main(int argc ,char**argv){
    osg::ArgumentParser argp(&argc,argv);

    if(argp.argc() < 2){
        fprintf(stderr, "Usage  meshfile -outfile filename -thresh threshold \n");
        return -1;
    }
    bool binaryFlag =true;

    int err=vcg::tri::io::Importer<MyMesh>::Open(mesh,argv[1]);
    for ( CMeshO::FaceIterator fi = mesh.face.begin(); fi != mesh.face.end(); ++fi)
      if(!(*fi).IsD())
      {  CMeshO::FaceType &f=(*fi);
          for(int i=0; i<3; i++)
              (*fi).WT(i).N()=0;

      }
    std::string texfile;
    if(argp.read("-tex",texfile))
         mesh.textures.push_back(texfile);
    osg::Vec3 minC(255,255,255),maxC(0.0,0.0,0.0);
    bool FlipFlag=argp.read("-F");

    std::string outfile="out.ply";
    argp.read("-out",outfile);
    if(FlipFlag)
           tri::Clean<MyMesh>::FlipMesh(mesh);
    vcg::tri::io::PlyInfo pi2;

       pi2.mask |= vcg::tri::io::Mask::IOM_WEDGTEXCOORD;
        tri::io::ExporterPLY<CMeshO>::Save(mesh,outfile.c_str(),binaryFlag,pi2);

}
