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
#include <vcg/complex/algorithms/update/texture.h>

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
static void WedgeTexFromVertexTex(CMeshO &m)
   {
     for(CMeshO::FaceIterator fi=m.face.begin();fi!=m.face.end();++fi)
             if(!(*fi).IsD())
                 {
                  for(int i=0;i<3;++i)
                  {
                    (*fi).WT(i).U() = (*fi).V(i)->T().U();
                    (*fi).WT(i).V() = (*fi).V(i)->T().V();
                  }
                 }
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
   /* for ( CMeshO::FaceIterator fi = mesh.face.begin(); fi != mesh.face.end(); ++fi)
      if(!(*fi).IsD())
      {  CMeshO::FaceType &f=(*fi);
          for(int i=0; i<3; i++)
              (*fi).WT(i).N()=0;

      }
    vcg::tri::io::PlyInfo pi2;

    pi2.mask |= vcg::tri::io::Mask::IOM_WEDGTEXCOORD;
     tri::io::ExporterPLY<CMeshO>::Save(mesh,"kni.ply",binaryFlag,pi2);
*/
    osg::Vec3 minC(255,255,255),maxC(0.0,0.0,0.0);
    bool FlipFlag=argp.read("-F");
    if(argp.read("--normcolor") && HasPerVertexColor(mesh) ){
        CMeshO::VertexIterator vi;

        for(vi=mesh.vert.begin();vi!=mesh.vert.end();++vi){
            if((*vi).IsD() )
                continue;
            for(int i=0;i<3; i++){
                if((*vi).C()[i] < minC[i])
                    minC[i]=(*vi).C()[i];
                if((*vi).C()[i] > maxC[i])
                    maxC[i]=(*vi).C()[i];
            }
        }
        for(vi=mesh.vert.begin();vi!=mesh.vert.end();++vi){
            if((*vi).IsD() )
                continue;
            for(int i=0;i<3; i++){
                (*vi).C()[i]=  (int)round((((*vi).C()[i]-minC[i])/(maxC[i]-minC[i]))*255.0);

            }
        }
    }

    for ( CMeshO::FaceIterator fi = mesh.face.begin(); fi != mesh.face.end(); ++fi)
      if(!(*fi).IsD())
      {  CMeshO::FaceType &f=(*fi);
          for(int i=0; i<3; i++)
              (*fi).WT(i).N()=0;
                                     if( ! ( (f.WT(0).N() == f.WT(1).N()) && (f.WT(0).N() == (*fi).WT(2).N()) )  ){
                                         printf("Su\n");
                                         return false; // all the vertices must have the same index.
                                     }

                                     if((*fi).WT(0).N() <0){
                                         printf("LEss\n");

                                         return false; // no undefined texture should be allowed
                                     }
      }
    if(!HasPerVertexTexCoord(mesh)){
        fprintf(stderr,"Can't get per vertex tex coords! Bailing.\n");
        exit(-1);
    }
    WedgeTexFromVertexTex(mesh);

    std::string outfile="out.ply";
    argp.read("-out",outfile);

    double threshold=-1.0;

    argp.read("-thresh",threshold);

  //  int dup = tri::Clean<MyMesh>::RemoveDuplicateVertex(mesh);
   /* int dup2 = tri::Clean<MyMesh>::RemoveDuplicateFace(mesh);
    int degen =  tri::Clean<MyMesh>::RemoveDegenerateVertex(mesh);
    int unref =  tri::Clean<MyMesh>::RemoveUnreferencedVertex(mesh);
*/
   // printf("Removed %i duplicate vetex %i duplicate faces  %d degen and %i unreferenced vertices from mesh \n",dup,dup2,degen,unref);

    //tri::AttributeSeam::SplitVertex(mesh, ExtractVertex, CompareVertex);
    if(threshold >0){
       int total = tri::Clean<CMeshO>::MergeCloseVertex(mesh, threshold);

       printf("Merged %d total %d\n",total,mesh.vn);
    }
    tri::UpdateBounding<CMeshO>::Box(mesh);
    //  cm.face.EnableFFAdjacency();
    //  cm.face.EnableMark();
    tri::UpdateTopology<CMeshO>::FaceFace(mesh);
    tri::UpdateFlags<CMeshO>::FaceBorderFromFF(mesh);
    if(FlipFlag)
        tri::Clean<MyMesh>::FlipMesh(mesh);
    float CCPerc=0.05;
    float minCC= CCPerc*mesh.bbox.Diag();
   /* printf("Cleaning Min CC %f\n",minCC);
    std::pair<int,int> delInfo= tri::Clean<CMeshO>::RemoveSmallConnectedComponentsDiameter(mesh,minCC);
    printf("Removed %d/%d\n",delInfo.second,delInfo.first);
     unref= tri::Clean<CMeshO>::RemoveUnreferencedVertex(mesh);*/
    vcg::tri::io::PlyInfo pi;

    pi.mask |= vcg::tri::io::Mask::IOM_WEDGTEXCOORD;
    pi.mask |= vcg::tri::io::Mask::IOM_VERTCOLOR;
    //pi.mask &= ~vcg::tri::io::Mask::IOM_VERTCOLOR;

    int result2 = tri::io::ExporterPLY<CMeshO>::Save(mesh,outfile.c_str(),binaryFlag,pi);
}
