#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osg/io_utils>
#include <string>
#include "BuildAtlas.h"
#include "vertexData.h"
#include <vips/vips.h>
#include <vips/version.h>
using namespace std;
int main( int argc, char **argv )
{
    // use an ArgumentParser object to manage the program arguments.
    osg::ArgumentParser arguments(&argc,argv);

    string matname;
    if(!arguments.read("-mat",matname)){
        fprintf(stderr, "need mat\n");
        exit(-1);
    }
#if VIPS_MINOR_VERSION > 24
    vips_init(argv[0]);
#endif
    double scaleFactor=1.0;
    arguments.read("-scale",scaleFactor);
    string basedir="mesh";
    arguments.read("-dir",basedir);
    std::string outfile="temp.obj";
    arguments.read("-outfile",outfile);

    string mosaic_cells_fname;
    if(!arguments.read("-cells",mosaic_cells_fname)){
        fprintf(stderr, "need mat\n");
        exit(-1);
    }
    int totalX,totalY;
    std::vector<mosaic_cell> mosaic_cells;

    loadMosaicCells(mosaic_cells_fname,totalX,totalY,mosaic_cells);

    osg::Matrix viewProj;
    readMatrixToScreen(matname,viewProj);
    ply::VertexDataMosaic vertexData;
    osg::Node *model;
    for(int pos=1;pos<arguments.argc();++pos)
        {
            if (!arguments.isOption(pos))
            {
                //cout << arguments[pos] <<endl;
                model= vertexData.readPlyFile( arguments[pos]);
               // cout << "ver "<<vertexData._vertices->size()<<endl;
              //  cout << "tri "<<vertexData._triangles->size()<<endl;

            }
        }


    osg::Geode *geode=model->asGeode();

    osg::Drawable *drawable=geode->getDrawable(0);
    osg::Geometry *geom = dynamic_cast< osg::Geometry*>(drawable);
    osg::Vec3Array *verts=static_cast< osg::Vec3Array*>(geom->getVertexArray());
    osg::Vec4Array *colors=static_cast< osg::Vec4Array*>(geom->getColorArray());
    osg::Vec4Array *texCoordsStored=static_cast< osg::Vec4Array*>(geom->getTexCoordArray(0));

    osg::DrawElementsUInt* primitiveSet = dynamic_cast<osg::DrawElementsUInt*>(geom->getPrimitiveSet(0));
    //int offset=newVerts->size();
    osg::ref_ptr<VipsAtlasBuilder >tf_atlas=createVTAtlas( viewProj, totalX, totalY,
                 mosaic_cells,
               false,scaleFactor,basedir);

    for(int i=0; i< (int)primitiveSet->getNumIndices()-2; i+=3){
        for(int j=0; j <3; j++){
            int mosaic_id=( int)vertexData._texIds->at(i/3)[1];
            osg::Vec3f &tc =    vertexData._texCoord[0]->at(i+j);

            if(mosaic_id >= 0 && mosaic_id< tf_atlas->atlasSourceMatrix.size()){
                if(tf_atlas->atlasSourceMatrix[mosaic_id] != NULL){
                    //     printf("%d\n", atlas->atlasMatrix[mosaic_id]);

                    // cout<< "orig: "<< tc<<endl;
                    // cout <<"offset "<<atlas->offsetMats[mosaic_id]<<endl;
                    osg::Matrix matrix2;
                    matrix2=dynamic_cast<VipsAtlasBuilder*>(tf_atlas.get())->getTextureMatrix((vips::VImage*)tf_atlas->atlasSourceMatrix[mosaic_id]);


                    const osg::Matrix &matrix =tf_atlas->offsetMats[mosaic_id];//atlas->atlasMatrix[mosaic_id]);

                    // cout << matrix << matrix2<<endl;
                    osg::Vec2 tc2;
                    tc2.set(tc[0]*matrix(0,0) + tc[1]*matrix(1,0) + matrix(3,0),
                            tc[0]*matrix(0,1) + tc[1]*matrix(1,1) + matrix(3,1));

                    //  cout << "shift: "<<tc2<<endl;

                    tc.set(tc2[0]*matrix2(0,0) + tc2[1]*matrix2(1,0) + matrix2(3,0),
                           tc2[0]*matrix2(0,1) + tc2[1]*matrix2(1,1) + matrix2(3,1),0);
                    //   cout << "Final:"<<tc<<endl;
                }else{
                    tc.set(-1,-1,-1);
                    printf("Failed atlas matrix map %d 0x%lx %f %f\n",mosaic_id,(long unsigned int)tf_atlas->atlasSourceMatrix[mosaic_id],tc.x(),tc.y() );
                    printf("Num of map %d\n",(int)tf_atlas->atlasSourceMatrix.size());
                    for(int k=0; k< (int)tf_atlas->atlasSourceMatrix.size(); k++){
                        printf("0x%lx\n",(long unsigned int)tf_atlas->atlasSourceMatrix[k]);
                    }
                    //    std::map< MyDataSet::range<std::pair<double,double > > ,int>::iterator itr;
                    //   for(itr=dynamic_cast<MyDataSet*>(_dataSet)->cell_coordinate_map.begin(); itr!=dynamic_cast<MyDataSet*>(_dataSet)->cell_coordinate_map.end(); itr++)
                    //     cout << "["<<itr->first.min().first<< "," << itr->first.min().second<<" - "<< itr->first.max().first<< "," << itr->first.max().second<<"] : " << itr->second<<"\n";


                }
            }else{
                printf("Failed mosaic_id %d\n",mosaic_id);
                tc.set(-1,-1,-1);

            }
        }
    }
  //  cout << vertexData._colors->size()<<endl;
  //  cout << ((osg::Vec3Array*)model->asGeode()->getDrawable(0)->asGeometry()->getVertexArray())->size()<<endl;
   // cout << ((osg::Vec4Array*)model->asGeode()->getDrawable(0)->asGeometry()->getColorArray())->size()<<endl;
 //   cout << "tex "<<vertexData._texCoord[0]->size()<<endl;
  //  cout << "id "<<vertexData._texIds->size()*3<<endl;
    model->asGeode()->getDrawable(0)->asGeometry()->setTexCoordArray(0,vertexData._texCoord[0]);

  //  cout << model->asGeode()->getDrawable(0)->asGeometry()->getPrimitiveSet(0)->getNumIndices()<<endl;

    osgDB::writeNodeFile(*model,outfile.c_str());



}

