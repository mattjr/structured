#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osg/io_utils>
#include "Clipper.h"
#include <osgUtil/SmoothingVisitor>
#include "PLYWriterNodeVisitor.h"
#include "TexturedSource.h"
#include "TexturingQuery.h"
#include "Extents.h"
#include "auv_stereo_geometry.hpp"
#include "PLYWriterNodeVisitor.h"
using namespace libsnapper;
using namespace std;

int main( int argc, char **argv )
{
    // use an ArgumentParser object to manage the program arguments.
    osg::ArgumentParser arguments(&argc,argv);

    // set up the usage document, in case we need to print out how to use this program.
    arguments.getApplicationUsage()->setDescription(arguments.getApplicationName() +" is the example which demonstrates Depth Peeling");
    arguments.getApplicationUsage()->setCommandLineUsage(arguments.getApplicationName()+" filename");


    string outfilename;
    if(!arguments.read("--outfile",outfilename)){
        fprintf(stderr,"Need outfile name\n");
        return -1;
    }
    string extraOutput;
    arguments.read("--extra",extraOutput);

    bool debug=false;
    osg::Vec2 size;
    {

        vips::VImage in("subtile.ppm");
        size.x()=in.Xsize();
        size.y()=in.Ysize();
        if(debug) {
            double downsampleRatio=1.0;
            in.affine(downsampleRatio,0,0,downsampleRatio,0,0,0,0,size.x()*downsampleRatio,downsampleRatio*size.y()).write("tmp.jpg");
        }
    }
    std::string mf=argv[1];
    osg::Matrixd viewProj,rotMat;
    readMatrixToScreen("view.mat",viewProj);
    readMatrix("rot.mat",rotMat);


    osg::Matrix bottomLeftToTopLeft;//= (osg::Matrix::scale(1,-1,1)*osg::Matrix::translate(0,size.y(),0));

    osg::Matrix toTex=viewProj*( osg::Matrix::translate(1.0,1.0,1.0)*osg::Matrix::scale(0.5*size.x(),0.5*size.y(),0.5f))*bottomLeftToTopLeft;
    std::vector<osg::ref_ptr<osg::Vec3Array> > coordsArray;
    osg::Node* model = osgDB::readNodeFile(mf);
    if (model)
    {


        osgUtil::Optimizer::MergeGeodesVisitor visitor;

        model->accept(visitor);
        osgUtil::Optimizer::MergeGeometryVisitor mgv;
        mgv.setTargetMaximumNumberOfVertices(INT_MAX);
        model->accept(mgv);

        osgUtil::GeometryCollector gc(NULL, osgUtil::Optimizer::DEFAULT_OPTIMIZATIONS);
        model->accept(gc);
        osgUtil::GeometryCollector::GeometryList geomList = gc.getGeometryList();

        if(geomList.size() != 1){
            OSG_ALWAYS << "Number of collected geometies " << geomList.size() << "problem "<<endl;
            //   OSG_FATAL << "Number of collected geometies " << geomList.size() << "problem "<<endl;
            assert(0);

        }
        osg::Vec3Array *verts=static_cast<const osg::Vec3Array*>((*geomList.begin())->getVertexArray());
        if(!verts){
            fprintf(stderr,"Fail to convert verts\n");
            exit(-1);
        }
        osg::Vec3Array *coords=new osg::Vec3Array(verts->size());
        coordsArray.push_back(coords);
        (*geomList.begin())->setTexCoordArray(0,coords);
#pragma omp for
        for(int j=0; j< (int)verts->size(); j++){
            osg::Vec4 pt(verts->at(j)[0],verts->at(j)[1],verts->at(j)[2],1.0);
            osg::Vec4 rotpt=pt*rotMat;
            osg::Vec4 proj=rotpt*toTex;
            proj.x() /= proj.w();
            proj.y() /= proj.w();
            proj.x() /= size.x();;
            proj.y() /= size.y();
            coords->at(j)=osg::Vec3(proj.x(),proj.y(),0);
        }



        if(debug){

            // set up the texture state.
            osg::Texture2D* texture = new osg::Texture2D;
            texture->setDataVariance(osg::Object::DYNAMIC); // protect from being optimized away as static state.
            texture->setImage(osgDB::readImageFile("tmp.jpg"));

            osg::StateSet* stateset = (*geomList.begin())->getOrCreateStateSet();
            stateset->setTextureAttributeAndModes(0,texture,osg::StateAttribute::ON);
            osgDB::writeNodeFile(*model,osgDB::getNameLessExtension(outfilename).append(".ive"));

        }else{
            if(osgDB::getFileExtension(outfilename) == "ply"){
                std::ofstream f(outfilename.c_str());

                PLYWriterNodeVisitor nv(f,NULL,&(coordsArray));
                model->accept(nv);
                if(extraOutput.length()){
                    osgDB::writeNodeFile(*model,extraOutput);
                }
            }else{
                osgDB::writeNodeFile(*model,outfilename);
            }

        }


    }else
        cerr << "Failed to open "<<mf <<endl;

}
