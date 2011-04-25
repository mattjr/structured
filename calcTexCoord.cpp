#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osg/io_utils>
#include "Clipper.h"
#include <osgUtil/SmoothingVisitor>
#include <osg/ComputeBoundsVisitor>
#include "PLYWriterNodeVisitor.h"
#include "TexturedSource.h"
#include "TexturingQuery.h"
#include "Extents.h"
#include "auv_stereo_geometry.hpp"
#include "calcTexCoord.h"
#include "PLYWriterNodeVisitor.h"
#include "GLImaging.h"
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
    string path=string(argv[0]);
    unsigned int loc=path.rfind("/");

    string basepath= loc == string::npos ? "./" : path.substr(0,loc+1);
    basepath= osgDB::getRealPath (basepath);
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


    string outfilename;
    if(!arguments.read("--outfile",outfilename)){
        fprintf(stderr,"Need outfile name\n");
        return -1;
    }
    bool reimage=false;
    std::string tex_cache_dir;
    texcache_t cache;
    osg::Vec4 zrange;
    arguments.read("--zrange",zrange[0],zrange[1]);
    int row,col,numRows,numCols,width,height;
    bool imageNode=arguments.read("--imageNode",row,col,numRows,numCols,width,height);
    bool untex=arguments.read("--untex");
    int size;
    if(arguments.read("--tex_cache",tex_cache_dir,size)){
        reimage=true;
        cache.push_back(std::make_pair<std::string,int>(tex_cache_dir,size));
    }
    float rx, ry, rz;
    osg::Matrix inverseM=osg::Matrix::identity();

    if(arguments.read("--invrot",rx,ry,rz)){
        inverseM =osg::Matrix::rotate(
                osg::DegreesToRadians( rx ), osg::Vec3( 1, 0, 0 ),
                osg::DegreesToRadians( ry ), osg::Vec3( 0, 1, 0 ),
                osg::DegreesToRadians( rz ), osg::Vec3( 0, 0, 1 ) );
    }
    osg::Matrix rotM=osg::Matrix::inverse(inverseM);

    std::string mf=argv[2];
    /* std::string sha2hash;
    int res=checkCached(mf,outfilename,sha2hash);
    if(res == -1)
        return -1;
    else if(res == 1)
        return 0;//Hash is valid
    cout <<"Computing hash\n";*/
    //Differing hash or no hash
    int npos=mf.find("/");
    std::string bbox_file=std::string(mf.substr(0,npos)+"/bbox-"+mf.substr(npos+1,mf.size()-9-npos-1)+".ply.txt");
    printf("SS %s\n",bbox_file.c_str());
    TexturedSource *sourceModel=new TexturedSource(vpb::Source::MODEL,mf,bbox_file);
    osgDB::Registry::instance()->setBuildKdTreesHint(osgDB::ReaderWriter::Options::BUILD_KDTREES);
    osg::Node* model = osgDB::readNodeFile(sourceModel->getFileName().c_str());
    osg::ref_ptr<osg::MatrixTransform>xform = new osg::MatrixTransform;
    xform->setDataVariance( osg::Object::STATIC );
    xform->setMatrix(rotM);
    osgUtil::Optimizer::FlattenStaticTransformsVisitor fstv(NULL);
    xform->addChild(model);
    xform->accept(fstv);
    fstv.removeTransforms(xform);
    osg::ComputeBoundsVisitor cbbv(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN);
    model->accept(cbbv);
    osg::BoundingBox bb = cbbv.getBoundingBox();
    zrange[2]=bb.zMin();
    zrange[3]=bb.zMax();
    if (model)
    {
        vpb::SourceData* data = new vpb::SourceData(sourceModel);
        data->_model = model;
        data->_extents.expandBy(model->getBound());
        sourceModel->setSourceData(data);
        osg::Geode *geode= dynamic_cast<osg::Geode*>(model);
        if(geode && geode->getNumDrawables()){
            //addDups(geode);
            osg::Drawable *drawable = geode->getDrawable(0);
            sourceModel->_kdTree = dynamic_cast<osg::KdTree*>(drawable->getShape());
        }else{
            std::cerr << "No drawbables \n";
        }
        //  TexPyrAtlas atlasGen(cache);
        //atlasGen._useAtlas=true;
        vpb::MyDataSet *dataset=new vpb::MyDataSet(calib->left_calib,basepath,false,false,false);
        dataset->_zrange=zrange;
        bool useAtlas=true;
        dataset->_useAtlas=useAtlas;
        dataset->_useBlending=true;
        //     dataset->_useDisplayLists=false;

        vpb::MyDestinationTile *tile=new vpb::MyDestinationTile(cache);
        tile->_mydataSet=dataset;
        tile->_dataSet=dataset;
        tile->_atlasGen->_useAtlas=useAtlas;
        tile->_atlasGen->_useStub=false;

        TexturingQuery *tq=new TexturingQuery(sourceModel,calib->left_calib,*tile->_atlasGen,true);

        tq->_tile=tile;
        bool projectSucess=tq->projectModel(dynamic_cast<osg::Geode*>(model));
        if(projectSucess){
            //  writeCached(outfilename,sha2hash,tile->texCoordIDIndexPerModel.begin()->second,tile->texCoordsPerModel.begin()->second);
            //osg::Geometry *geom = dynamic_cast< osg::Geometry*>( geode->getDrawable(0));
            // for(int f=0; f<tile->texCoordsPerModel.begin()->second.size(); f++)
            //    geom->setTexCoordArray(f,tile->texCoordsPerModel.begin()->second[f]);
            if(!reimage){
                std::ofstream f(outfilename.c_str());
                PLYWriterNodeVisitor nv(f,tile->texCoordIDIndexPerModel.begin()->second,&(tile->texCoordsPerModel.begin()->second));
                model->accept(nv);
            }else{
                // map<SpatialIndex::id_type,int> allIds=calcAllIds(tile->texCoordIDIndexPerModel.begin()->second);
                // tq->addImagesToAtlasGen(allIds);
                tile->_models = new vpb::DestinationData(NULL);
                tile->_models->_models.push_back(model);
                osg::ref_ptr<osg::Node> node=tile->createScene();
                osg::ref_ptr<osg::MatrixTransform>xform = new osg::MatrixTransform;
                xform->setDataVariance( osg::Object::STATIC );
                xform->setMatrix(inverseM);
                xform->addChild(node);
                if(imageNode){
                    osg::Matrixd view,proj;


                    std::stringstream os2;
                    os2<< "view.mat";

                    std::fstream _file(os2.str().c_str(),std::ios::binary|std::ios::in);
                    for(int i=0; i<4; i++)
                        for(int j=0; j<4; j++)
                            _file.read(reinterpret_cast<char*>(&(view(i,j))),sizeof(double));
                    for(int i=0; i<4; i++)
                        for(int j=0; j<4; j++)
                            _file.read(reinterpret_cast<char*>(&(proj(i,j))),sizeof(double));
                    _file.close();

                    imageNodeGL(xform.get(),numRows,numCols,width,height,row,col,view,proj,untex,"png");

                }
                if(!imageNode){
                    osgDB::ReaderWriter::Options* options = new osgDB::ReaderWriter::Options;
                    printf("AAAA %s\n",options->getOptionString().c_str());
                   // options->setOptionString("compressed=1 noTexturesInIVEFile=1 noLoadExternalReferenceFiles=1 useOriginalExternalReferences=1");
                    osgDB::Registry::instance()->setOptions(options);
                    osgDB::writeNodeFile(*xform,osgDB::getNameLessExtension(outfilename).append(".ive"));
                }
                /* osgUtil::Optimizer::FlattenStaticTransformsVisitor fstv(NULL);
                xform->accept(fstv);
                fstv.removeTransforms(xform);
*/

                /*   osgDB::ReaderWriter::WriteResult wr;

                osgDB::ReaderWriter* rw =  osgDB::Registry::instance()->getReaderWriterForExtension("ive");
                wr = rw->writeNode( *dynamic_cast<osg::Geode*>(model), osgDB::getNameLessExtension(outfilename).append(".ive"), new osgDB::Options("compressed 1") );
            //   osgDB::writeNodeFile(*model,osgDB::getNameLessExtension(outfilename).append(".ivez"),osgDB::Registry::instance()->getOptions());
                       //osgDB::Registry::instance()->writeNode(*node,osgDB::getNameLessExtension(outfilename).append(".ive"), osgDB::Registry::instance()->getOptions() );*/
              /*  std::ofstream f(outfilename.c_str());
                PLYWriterNodeVisitor nv(f,tile->texCoordIDIndexPerModel.begin()->second,&(tile->texCoordsPerModel.begin()->second));
                model->accept(nv);*/
                //    if (!wr.success() )     OSG_NOTIFY( osg::WARN ) << "ERROR: Savefailed: " << wr.message() << std::endl;
            }

        }else
            cerr << "Failed to project\n";
        delete tq;
    }else
        cerr << "Failed to open "<<sourceModel->getFileName() <<endl;

}
