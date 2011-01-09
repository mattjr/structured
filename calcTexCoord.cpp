#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osg/io_utils>
#include "Clipper.h"
#include <osgUtil/SmoothingVisitor>
#include "PLYWriterNodeVisitor.h"
#include "TexturedSource.h"
#include "TexturingQuery.h"
#include "Extents.h"
#include "sha2.h"
#include "auv_stereo_geometry.hpp"
#define BUFLEN 16384

using namespace libsnapper;
using namespace std;
int main( int argc, char **argv )
{
    // use an ArgumentParser object to manage the program arguments.
    osg::ArgumentParser arguments(&argc,argv);

    // set up the usage document, in case we need to print out how to use this program.
    arguments.getApplicationUsage()->setDescription(arguments.getApplicationName() +" is the example which demonstrates Depth Peeling");
    arguments.getApplicationUsage()->setCommandLineUsage(arguments.getApplicationName()+" filename");
    Stereo_Calib *calib=NULL;
    string base_dir=argv[1];
    string stereo_calib_file_name = "stereo.calib";
    stereo_calib_file_name= base_dir+string("/")+stereo_calib_file_name;

    try {
        calib = new Stereo_Calib( stereo_calib_file_name );
    }
    catch( string error ) {
        cerr << "ERROR - " << error << endl;
        exit( 1 );
    }


    string outfilename="out.ply";
    arguments.read("--outfile",outfilename);
    std::string mf=argv[2];
    int npos=mf.find("/");
    std::string bbox_name=std::string(mf.substr(0,npos)+"/bbox-"+mf.substr(npos+1,mf.size()-9-npos-1)+".ply.txt");
    if(!osgDB::fileExists(bbox_name)){
        cerr << "Bbox file " << bbox_name << mf<<" doesn't exist\n";
        return -1;
    }
    sha2 mySha2;
    mySha2.Init(sha2::enuSHA256);
    unsigned char	buf[BUFLEN];
    char buffer1[2048];

    FILE *fp=fopen(mf.c_str(),"rb");
    if(!fp){
        cerr << "MEsh file " << mf<<" doesn't exist\n";
        return -1;
    }
    int l;
    while ((l = fread(buf,1,BUFLEN,fp)) > 0) {

        mySha2.Update(buf,l);
    }
    fclose(fp);
    mySha2.End();
    std::string meshHash=mySha2.StringHash();
    if(osgDB::fileExists(outfilename)){
        //Check hash
        ifstream fin(outfilename.c_str());
        fin.getline(buffer1,2048);

        if(string(buffer1) == meshHash)
        {
            cout << "Valid existing hash of texcoord file skipping\n";
            return 0;
        }else{
            cout << "Differing hashes "<< buffer1<< " != "<<meshHash<<endl;
        }

    }
    TexturedSource *sourceModel=new TexturedSource(vpb::Source::MODEL,mf);
    osgDB::Registry::instance()->setBuildKdTreesHint(osgDB::ReaderWriter::Options::BUILD_KDTREES);
    osg::Node* model = osgDB::readNodeFile(sourceModel->getFileName().c_str());
    if (model)
    {
        vpb::SourceData* data = new vpb::SourceData(sourceModel);
        data->_model = model;
        data->_extents.expandBy(model->getBound());
        sourceModel->setSourceData(data);
        osg::Geode *geode= dynamic_cast<osg::Geode*>(model);
        if(geode && geode->getNumDrawables()){
            osg::Drawable *drawable = geode->getDrawable(0);
            sourceModel->_kdTree = dynamic_cast<osg::KdTree*>(drawable->getShape());
        }else{
            std::cerr << "No drawbables \n";
        }
    }
    TexPyrAtlas atlasGen("null",false);
    TexturingQuery *tq=new TexturingQuery(sourceModel,calib->left_calib,atlasGen,true);
    vpb::MyDestinationTile *tile=new vpb::MyDestinationTile("");

    tq->_tile=tile;
    bool projectSucess=tq->projectModel(dynamic_cast<osg::Geode*>(model));
    if(projectSucess){
        FILE *fp=fopen(outfilename.c_str(),"w");
        if(!fp){
            fprintf(stderr, "Can't write file %s\n",outfilename.c_str());
            return -1;
        }
        if(tile->texCoordIDIndexPerModel.size() < 1){
            fprintf(stderr, "Didn't store any tex coords\n");
            return -1;
        }
        fprintf(fp,"%s\n",meshHash.c_str());
        fprintf(fp,"%d\n",(int)tile->texCoordIDIndexPerModel[0]->size());
        for(int i=0; i< (int)tile->texCoordIDIndexPerModel[0]->size(); i++){
            for(int j=0; j <4; j++){
                int a=(int)tile->texCoordIDIndexPerModel[0]->at(i)[j];
                fwrite((char*)&a,1,sizeof(int),fp);
            }
            for(int j=0; j <2; j++){
                int a=(int)tile->texCoordsPerModel[0]->at(i)[j];
                fwrite((char*)&a,1,sizeof(int),fp);
            }
        }
    }
    delete tq;
}
