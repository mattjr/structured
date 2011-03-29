#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osg/io_utils>
#include "Clipper.h"
#include <osgUtil/SmoothingVisitor>
#include "PLYWriterNodeVisitor.h"
#include <osgUtil/Optimizer>

void addDups(osg::Geode *geode);
using namespace std;
int main( int argc, char **argv )
{
    // use an ArgumentParser object to manage the program arguments.
    osg::ArgumentParser arguments(&argc,argv);

    // set up the usage document, in case we need to print out how to use this program.
    arguments.getApplicationUsage()->setDescription(arguments.getApplicationName() +" is the example which demonstrates Depth Peeling");
    arguments.getApplicationUsage()->setCommandLineUsage(arguments.getApplicationName()+" filename");
    string outfilename="out.ply";
    arguments.read("--outfile",outfilename);
    IntersectKdTreeBbox::OverlapMode mode;
    if(arguments.read("-gap"))
        mode=IntersectKdTreeBbox::GAP;
    else if(arguments.read("-dup"))
        mode=IntersectKdTreeBbox::DUP;
    else if(arguments.read("-cut"))
        mode=IntersectKdTreeBbox::CUT;
    else{
        cerr << "Must be -gap or  -dup  or -cut \n";
        return -1;
    }//  arguments.reportRemainingOptionsAsUnrecognized();


    // report any errors if they have occurred when parsing the program arguments.
    if (arguments.errors())
    {
        arguments.writeErrorMessages(std::cout);
        return 1;
    }
    if(arguments.argc() < (2+6)  ){
        fprintf(stderr,"Must pass two meshes and bbox arg must be base dir\n");
        arguments.getApplicationUsage()->write(std::cerr,osg::ApplicationUsage::COMMAND_LINE_OPTION);
        exit(-1);
    }

    osg::Vec3 minV,maxV;
    minV.x() = atof(arguments[2]);
    minV.y() = atof(arguments[3]);
    minV.z() = atof(arguments[4]);

    maxV.x() = atof(arguments[5]);
    maxV.y() = atof(arguments[6]);
    maxV.z() = atof(arguments[7]);
    osg::BoundingBox bb(minV,maxV);
    OSG_NOTICE << bb._min << " " << bb._max << endl;
    osgDB::Registry::instance()->setBuildKdTreesHint(osgDB::ReaderWriter::Options::BUILD_KDTREES);
    osg::ref_ptr<osg::Node> model = osgDB::readNodeFile(arguments[1]);
    osg::ref_ptr<osg::Node> root;
    bool result = false;

   // do

    //while(result);
        if(model.valid()){
        osg::Geode *geode= dynamic_cast<osg::Geode*>(model.get());
        if(!geode)
            geode=model->asGroup()->getChild(0)->asGeode();
        if(geode && geode->getNumDrawables()){
            osg::Drawable *drawable = geode->getDrawable(0);
            osg::KdTree *kdTree = dynamic_cast<osg::KdTree*>(drawable->getShape());
            if(kdTree){
                KdTreeBbox *kdtreeBbox=new KdTreeBbox(*kdTree);
                root=kdtreeBbox->intersect(bb,mode,false);
            }
        }else{
            OSG_ALWAYS << "Model can't be converted to geode\n";
            exit(-1);
        }

    }else{
        OSG_ALWAYS << "Model can't be loaded\n";
        exit(-1);
    }

    if(osgDB::getFileExtension(outfilename) == "ply"){
        std::ofstream f(outfilename.c_str());
        PLYWriterNodeVisitor nv(f);
        root->accept(nv);

    }else{
        osgUtil::SmoothingVisitor sv;
        root->accept(sv);
        osgDB::writeNodeFile(*root,outfilename);
    }
}
