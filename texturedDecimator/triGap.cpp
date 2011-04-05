
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
using namespace std;
using namespace vcg;
using namespace tri;

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
    osg::ref_ptr<osgUtil::DelaunayTriangulator> trig = new osgUtil::DelaunayTriangulator();
    osg::Geometry* gm = *geomList.begin();
    osg::Vec3Array *v=(osg::Vec3Array*)gm->getVertexArray();
    FILE *fp=fopen("tmp.txt","w");
    for(int i=0; i<v->size(); i++){
        fprintf(fp,"%f %f\n",v->at(i)[0],v->at(i)[1]);
    }
    fclose(fp);
    std::cout << v->size()<<std::endl;
    trig->setInputPointArray(v);
    trig->triangulate();
    osg::DrawElementsUInt *newDrawElem=new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES, 0);
    osg::DrawElementsUInt *origDrawElem=trig->getTriangles();

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
    printf("Gheed %d\n",newDrawElem->getNumIndices());
    gm->setPrimitiveSet(0,origDrawElem);//newDrawElem);
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
