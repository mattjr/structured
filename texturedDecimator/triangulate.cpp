

#include <assert.h>
#include <limits.h>
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
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osg/io_utils>
#include "Clipper.h"
#include <osgUtil/SmoothingVisitor>
#include "PLYWriterNodeVisitor.h"
#include <osgUtil/Optimizer>
#include <osgUtil/DelaunayTriangulator>
#include <osgUtil/Optimizer>
#include <osgUtil/SmoothingVisitor>
#include <osgUtil/MeshOptimizers>
#define LINUX 1
#define REAL double
#define ANSI_DECLARATORS 1
extern "C"{
#include "triangle.h"
};
using namespace std;
using namespace vcg;
using namespace tri;
void triangulation_run(osg::Vec3Array *pts,osg::DrawElementsUInt *primset){
    REAL *points= new REAL[pts->size()*2];
    int *markers= new int[pts->size()];
   REAL *zvals = (REAL *) malloc(pts->size() * sizeof(REAL));

    const int offset = 100;
    for(int i=0; i < pts->size(); i++){
        points[i*2]  = pts->at(i)[0];
        points[i*2+1]= pts->at(i)[1];
        markers[i]= i+offset;
        zvals[i]= pts->at(i)[2];
    }
    // input
    triangulateio in;
    in.numberofpoints= pts->size();
    in.pointlist= points;
    in.pointmarkerlist= markers;
    in.numberofpointattributes = 1;
    in.numberofholes= 0;
    in.trianglelist= 0;
    in.pointattributelist = zvals;


    // output
    triangulateio out;
    out.pointlist= 0;
    out.trianglelist= 0;
    out.pointmarkerlist= 0;
    out.numberofpointattributes= 0;
    out.numberofholes= 0;
    out.pointattributelist= 0;

    out.numberoftriangleattributes= 0;
    // do triangulation
    triangulate( "-z -i", &in, &out, 0);

    // create any new points that are necessary
    for (int i=0; i< out.numberofpoints; i++)
    {
        if (out.pointmarkerlist[i]< offset)
        {
            float z = out.pointattributelist[i];

            pts->push_back(osg::Vec3 (out.pointlist[i*2],
                                      out.pointlist[i*2+1],z));

            out.pointmarkerlist[i]= pts->size()-1+ offset;
        }
    }

    // clean up from previous triangulation
    //   primset->clear;

    // create the triangles
    for (int i=0; i< out.numberoftriangles; i++)
    {
        primset->push_back( out.pointmarkerlist[ out.trianglelist[i*3]]- offset);
        primset->push_back(out.pointmarkerlist[ out.trianglelist[i*3+1]]- offset);
        primset->push_back(out.pointmarkerlist[ out.trianglelist[i*3+2]]- offset);

        //if (poly_.inside( triangle->mid_point()))
          //  tris_.push_back( triangle);
    }


    // clean memory
    delete[] out.pointlist;
    delete[] out.pointmarkerlist;
    delete[] out.trianglelist;
    delete[] points;
    delete[] markers;
    delete[] zvals;
/*
    struct triangulateio in, out;
            int i;

            // point list
            in.numberofpoints = pts->size();
            in.pointlist = (REAL *) malloc(in.numberofpoints * 2 * sizeof(REAL));
            in.numberofpointattributes = 1;
            in.pointattributelist = (REAL *) malloc(in.numberofpoints * sizeof(REAL));
            int *markers= (int *)malloc(pts->size()*sizeof(int));

            in.pointmarkerlist= markers;
            const int offset = 100;

            for ( i = 0; i < in.numberofpoints; ++i )
            {
                osg::Vec3 p3=pts->at(i);
                    in.pointlist[2*i] = p3[0];
                    in.pointlist[2*i + 1] = p3[1];

                    in.pointattributelist[i] = p3[2];
                    markers[i]= i+offset;

            }

           // in.pointmarkerlist = (int *) NULL;
            in.numberoftriangles = 0;

            // no segment list
            in.numberofsegments = 0;
            in.segmentlist = (int *) NULL;
            in.segmentmarkerlist = (int *) NULL;

            // no holes or regions
            in.numberofholes = 0;
            in.holelist = (REAL *) NULL;
            in.numberofregions = 0;
            in.regionlist = (REAL *) NULL;

            // prep the output structures
            out.pointlist = (REAL *) NULL;        // Not needed if -N switch used.
            out.pointattributelist = (REAL *) NULL;
            out.pointmarkerlist = (int *) NULL;   // Not needed if -N or -B switch used.
            out.trianglelist = (int *) NULL;      // Not needed if -E switch used.
            out.triangleattributelist = (REAL *) NULL;
            out.neighborlist = (int *) NULL;      // Needed only if -n switch used.
            out.segmentlist = (int *) NULL;
            out.segmentmarkerlist = (int *) NULL;
            out.edgelist = (int *) NULL;          // Needed only if -e switch used.
            out.edgemarkerlist = (int *) NULL;    // Needed if -e used and -B not used.

            // Triangulate the points.  Switches are chosen:
            // number everything from zero (z),
            triangulate("-z -i -q", &in, &out, NULL);

            // now copy the triangle results back into vtdata structures
            for ( i = 0; i < out.numberofpoints; ++i )
            {
                if (out.pointmarkerlist[i]< offset)
                {  double z = out.pointattributelist[i];
                    pts->push_back(osg::Vec3(out.pointlist[2*i], out.pointlist[2*i + 1], z));
                    out.pointmarkerlist[i]= pts->size()-1+ offset;
                }

            }
            for ( i = 0; i < out.numberoftriangles; ++i )
            {
                    int n1 = out.pointmarkerlist[ out.trianglelist[i*3]]- offset;//out.trianglelist[i * 3];
                    int n2 = out.pointmarkerlist[ out.trianglelist[i*3+1]];//out.trianglelist[i * 3 + 1];
                    int n3 = out.pointmarkerlist[ out.trianglelist[i*3+2]];//out.trianglelist[i * 3 + 2];
                     primset->push_back( n1);
                     primset->push_back( n2);
                     primset->push_back( n3);

            }
            // free mem allocated to the "Triangle" structures
            free(in.pointlist);
            free(in.segmentlist);

            free(out.pointlist);
            free(out.pointattributelist);
            free(out.pointmarkerlist);
            free(out.trianglelist);
            free(out.triangleattributelist);
            free(out.neighborlist);
            free(out.segmentlist);
            free(out.segmentmarkerlist);
            free(out.edgelist);
            free(out.edgemarkerlist);

*/
}

int main( int argc, char **argv )
{
    // use an ArgumentParser object to manage the program arguments.
    osg::ArgumentParser arguments(&argc,argv);


    // set up the usage document, in case we need to print out how to use this program.
    arguments.getApplicationUsage()->setDescription(arguments.getApplicationName() +" is the example which demonstrates Depth Peeling");
    arguments.getApplicationUsage()->setCommandLineUsage(arguments.getApplicationName()+" filename");
    string outfilename="out.ply";
    arguments.read("--outfile",outfilename);
    double thresh=0.05;

    arguments.read("--thresh",thresh);

    osg::Node *node=osgDB::readNodeFiles(arguments);
    osgUtil::Optimizer::MergeGeodesVisitor visitor;

    node->accept(visitor);
    osgUtil::Optimizer::MergeGeometryVisitor mgv;
    mgv.setTargetMaximumNumberOfVertices(INT_MAX);
    node->accept(mgv);
    osgUtil::GeometryCollector gc(NULL, osgUtil::Optimizer::DEFAULT_OPTIMIZATIONS);
    node->accept(gc);
    osgUtil::GeometryCollector::GeometryList geomList = gc.getGeometryList();

    if(geomList.size() > 1){
        OSG_ALWAYS << "Number of collected geometies " << geomList.size() << "problem "<<endl;
        //   OSG_FATAL << "Number of collected geometies " << geomList.size() << "problem "<<endl;
        assert(0);

    }
  //  osg::ref_ptr<osgUtil::DelaunayTriangulator> trig = new osgUtil::DelaunayTriangulator();
    osg::Geometry* gm = *geomList.begin();
    osg::Vec3Array *v=(osg::Vec3Array*)gm->getVertexArray();
    FILE *fp=fopen("tmp.txt","w");
    for(int i=0; i<v->size(); i++){
        fprintf(fp,"%f %f\n",v->at(i)[0],v->at(i)[1]);
    }
    fclose(fp);
    std::cout << v->size()<<std::endl;
    //trig->setInputPointArray(v);
   // trig->triangulate();
    osg::DrawElementsUInt *origDrawElem=new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES, 0);
    triangulation_run(v,origDrawElem);
 //   osg::DrawElementsUInt *origDrawElem=trig->getTriangles();
    osg::DrawElementsUInt *newDrawElem=new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES, 0);


   for(int i=0; i< origDrawElem->getNumIndices()-2; i+=3){

        double dist1=sqrt(pow(v->at(origDrawElem->getElement(i+0))[0]-v->at(origDrawElem->getElement(i+1))[0],2.0)+
                          pow(v->at(origDrawElem->getElement(i+0))[1]-v->at(origDrawElem->getElement(i+1))[1],2.0)+
                          pow(v->at(origDrawElem->getElement(i+0))[2]-v->at(origDrawElem->getElement(i+1))[2],2.0));

        double dist2=sqrt(pow(v->at(origDrawElem->getElement(i+1))[0]-v->at(origDrawElem->getElement(i+2))[0],2.0)+
                          pow(v->at(origDrawElem->getElement(i+1))[1]-v->at(origDrawElem->getElement(i+2))[1],2.0)+
                          pow(v->at(origDrawElem->getElement(i+1))[2]-v->at(origDrawElem->getElement(i+2))[2],2.0));

        double dist3=sqrt(pow(v->at(origDrawElem->getElement(i+2))[0]-v->at(origDrawElem->getElement(i+0))[0],2.0)+
                          pow(v->at(origDrawElem->getElement(i+2))[1]-v->at(origDrawElem->getElement(i+0))[1],2.0)+
                          pow(v->at(origDrawElem->getElement(i+2))[2]-v->at(origDrawElem->getElement(i+0))[2],2.0));

        osg::Vec3 v1=v->at(origDrawElem->getElement(i+0));
        osg::Vec3 v2=v->at(origDrawElem->getElement(i+1));
        osg::Vec3 v3=v->at(origDrawElem->getElement(i+2));

        osg::Vec3 e1 = v2-v1;
        osg::Vec3 e2 = v3-(v1);
        double        area = 0.5 * (e1^e2).length();
        double thresh2=0.07;
        //   printf("%f %f %f \n",dist1,dist2,dist3);
        //        printf("area %f \n",area);
        if(!(dist1 > thresh || dist2 > thresh || dist3 > thresh))
            // if(area>thresh && !(dist1 > thresh2 || dist2 > thresh2 || dist3 > thresh2))
        {
            for(int j=0; j<3; j++){
                newDrawElem->addElement(origDrawElem->getElement(i+j));

            }
        }
    }
  //  printf("Gheed %d\n",newDrawElem->getNumIndices());
    gm->setPrimitiveSet(0,newDrawElem);
    osg::Vec4Array* colors = new osg::Vec4Array(1);
    colors->push_back(osg::Vec4(rand()/(double) RAND_MAX,rand()/(double) RAND_MAX,rand()/(double) RAND_MAX,1));
    gm->setColorArray(colors);
    gm->setColorBinding(osg::Geometry::BIND_OVERALL);
    //create geometry and add it to scene graph
    osg::Geode* geode = new osg::Geode();
    geode->addDrawable(gm);
    //    osgDB::writeNodeFile(*geode,outfilename);
    std::ofstream f(outfilename.c_str());
    PLYWriterNodeVisitor nv(f);
    geode->accept(nv);
    /*   CMeshO cm;
    int err=tri::io::ImporterPLY<CMeshO>::Open(cm,outfilename.c_str());
    double threshold=0.01;
    int selFaceNum = tri::UpdateSelection<CMeshO>::FaceOutOfRangeEdge(cm,0,threshold );
    CMeshO::FaceIterator fi;

    for(fi=cm.face.begin();fi!=cm.face.end();++fi)
        if(!(*fi).IsD() && (*fi).IsS() ) tri::Allocator<CMeshO>::DeleteFace(cm,*fi);
    //cm.clearDataMask(MeshModel::MM_FACEFACETOPO | MeshModel::MM_FACEFLAGBORDER);
    //     int nullFaces=tri::Clean<CMeshO>::RemoveFaceOutOfRangeArea(cm,threshold,threshold);
    bool binaryFlag =true;
    printf("Thresh meeting %d\n",selFaceNum);
    int result = tri::io::ExporterPLY<CMeshO>::Save(cm,"clean.ply",binaryFlag);
*/
}
