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
void splitMeshOuput(CMeshO &m,const char *basename, int splitting);
int main(int argc ,char**argv){
    if(argc<2) Usage();
   // std::string in_texcoord_file=argv[3];
   // std::string out_texcoord_file=argv[4];

    //int t0=clock();
    int err=vcg::tri::io::ImporterPLY<CMeshO>::Open(mesh,argv[1]);

    if(err)
    {
        printf("Unable to open mesh %s : '%s'\n",argv[1],vcg::tri::io::Importer<CMeshO>::ErrorMsg(err));
        exit(-1);
    }
    printf("mesh loaded %s %d %d \n",argv[1],mesh.vn,mesh.fn);
 //   int FinalSize=FinalSizePer*mesh.fn;

    int splitting=0;
    bool CleaningFlag=false;
    char basename[1024];
    // parse command line.
    for(int i=2; i < argc;)
    {
        if(argv[i][0]=='-')
            switch(argv[i][1])
            {
                                case 'P' :	CleaningFlag=true;  printf("Cleaning mesh before simplification\n"); break;
                                case 's' :	splitting = atoi(argv[i]+2);  printf("Splitting output at %d\n",atoi(argv[i]+2)); break;
                                case 'u' :	strcpy(basename ,argv[i]+2);  printf("Splitting output fn %s\n",basename); break;
                                default  :  printf("Unknown option '%s'\n", argv[i]);
                                exit(0);
                            }
        i++;
    }



    if(CleaningFlag){
        int dup2 = tri::Clean<CMeshO>::RemoveDuplicateFace(mesh);
        int dup = tri::Clean<CMeshO>::RemoveDuplicateVertex(mesh);
        int unref =  tri::Clean<CMeshO>::RemoveUnreferencedVertex(mesh);
        printf("Removed %i duplicate vetex %i duplicate faces  and %i unreferenced vertices from mesh \n",dup,dup2,unref);
    }

    if(!tri::HasPerWedgeTexCoord(mesh))
    {
        printf("Warning: nothing have been done. Mesh has no Texture.\n");
        return false;
    }
    if ( ! tri::Clean<CMeshO>::HasConsistentPerWedgeTexCoord(mesh) ) {
        printf( "Mesh has some inconsistent tex coords (some faces without texture)\n");
    }
    tri::UpdateTopology<CMeshO>::FaceFace(mesh);
    tri::UpdateFlags<CMeshO>::FaceBorderFromFF(mesh);
    tri::UpdateNormals<CMeshO>::PerVertexNormalizedPerFace(mesh);
    tri::UpdateBounding<CMeshO>::Box(mesh);
    tri::UpdateTopology<CMeshO>::VertexFace(mesh);
    tri::UpdateFlags<CMeshO>::FaceBorderFromNone(mesh);

    if(splitting>0)
          splitMeshOuput(mesh,basename,splitting);
    else{
        Usage();
        return -1;
    }
    return 0;

}

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
void splitMeshOuput(CMeshO &m,const char *basename, int splitting){
    tri::AttributeSeam::SplitVertex(m, ExtractVertex, CompareVertex);

    char fname[1024];
    int mesh_count=0;
    int faceLeft=m.fn;
    std::set<CMeshO::FacePointer> touched;
    while(faceLeft>0){
        CMeshO destMesh;

        int selectedV=0;
        CMeshO::VertexIterator vi;
        tri::UpdateSelection<CMeshO>::ClearVertex(m);
        tri::UpdateSelection<CMeshO>::ClearFace(m);
        CMeshO::FaceIterator   fi;
        for(fi = m.face.begin(); fi != m.face.end() &&selectedV<(splitting-3); ++fi){
                if( !(*fi).IsD() && !(*fi).IsS() &&   touched.count(&(*fi)) ==0){
                   (*fi).SetS();
                    touched.insert(&(*fi));
                    for(int i=0; i< 3; i++){
                        if(!(*fi).V(i)->IsS() && !(*fi).V(i)->IsD()) {
                            (*fi).V(i)->SetS(); ++selectedV;
                        }
                    }
                }
        }
        printf("SelectedV %d\n",selectedV);
        /*for(vi=m.vert.begin();vi!=m.vert.end() && selectedV<(splitting-3);++vi){
            if(!(*vi).IsD() && !(*vi).IsS() && touched.count(&(*vi)) ==0){
                (*vi).SetS();
                selectedV++;
                touched.insert(&(*vi));
                tri::UpdateSelection<CMeshO>::FaceFromVertexLoose(m);
                if(tri::UpdateSelection<CMeshO>::VertexFromFaceStrict(m)){
                     for(vi2=m.vert.begin();vi2!=m.vert.end();++vi2){
                         if(!(*vi2).IsD() && (*vi2).IsS() && touched.count(&(*vi2)) ==0){
                             selectedV++;
                             touched.insert(&(*vi2));
                         }
                     }
                }

            }
        }*/



        // select all points involved
      //  tri::UpdateSelection<CMeshO>::ClearVertex(mesh);
        //tri::UpdateSelection<CMeshO>::FaceFromVertexStrict(m);
        // tri::UpdateSelection<CMeshO>::ClearVertex(mesh);
     //    tri::UpdateSelection<CMeshO>::VertexFromFaceLoose(mesh);

        tri::Append<CMeshO,CMeshO>::Mesh(destMesh, m, true);

        int numFacesSel = tri::UpdateSelection<CMeshO>::CountFace(m);
        int numVertSel  = tri::UpdateSelection<CMeshO>::CountVertex(m);

        printf("Moving faces %d verts %d\n",numFacesSel,numVertSel);
        tri::UpdateSelection<CMeshO>::ClearVertex(m);
        tri::UpdateSelection<CMeshO>::VertexFromFaceStrict(m);
      /*  for(fi=m.face.begin();fi!=m.face.end();++fi)
            if(!(*fi).IsD() && (*fi).IsS() )
                tri::Allocator<CMeshO>::DeleteFace(m,*fi);
        for(vi=m.vert.begin();vi!=m.vert.end();++vi)
            if(!(*vi).IsD() && (*vi).IsS() )
                tri::Allocator<CMeshO>::DeleteVertex(m,*vi);
        vcg::tri::Allocator<CMeshO>::CompactFaceVector(m);

       */
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
        faceLeft=0;
      /*  for(vi=m.vert.begin();vi!=m.vert.end();++vi){
            if(!(*vi).IsD() && touched.count(&(*vi)) ==0)
                faceLeft++;
        }
        */
        for(fi = m.face.begin(); fi != m.face.end() ; ++fi){
                if( !(*fi).IsD() && touched.count(&(*fi)) ==0){
                    faceLeft++;
                }
        }
        printf("FAce left %d %d\n",faceLeft,touched.size());
        //  currentMesh->clearDataMask(MeshModel::MM_FACEFACETOPO | MeshModel::MM_FACEFLAGBORDER);
        vcg::tri::UpdateFlags<CMeshO>::VertexClear(m,CMeshO::VertexType::SELECTED);
              vcg::tri::UpdateFlags<CMeshO>::FaceClear(m,CMeshO::FaceType::SELECTED);
     //   vcg::tri::UpdateFlags<CMeshO>::FaceClear(destMesh,CMeshO::FaceType::SELECTED);
        //Log("Moved %i faces and %i vertices to layer %i", numFacesSel, numVertSel, md.meshList.size());
    }
    string path=osgDB::getFilePath(basename).size() >0 ? osgDB::getFilePath(basename): ".";
    sprintf(fname,"%s/octree.txt",path.c_str());

    FILE *fp=fopen(fname,"w");
    fprintf(fp,"%d\n",mesh_count-1);
    fclose(fp);
}
