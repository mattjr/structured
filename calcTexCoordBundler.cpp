//
// structured - Tools for the Generation and Visualization of Large-scale
// Three-dimensional Reconstructions from Image Data. This software includes
// source code from other projects, which is subject to different licensing,
// see COPYING for details. If this project is used for research see COPYING
// for making the appropriate citations.
// Copyright (C) 2013 Matthew Johnson-Roberson <mattkjr@gmail.com>
//
// This file is part of structured.
//
// structured is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// structured is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with structured.  If not, see <http://www.gnu.org/licenses/>.
//

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
#include "calcTexCoord.h"
#include "calibFile.h"
#include "PLYWriterNodeVisitor.h"
#include "GLImaging.h"
#include "vertexData.h"
#include "ANNWrapper.h"
#include <opencv2/highgui/highgui.hpp>
typedef struct _imgData{
    osg::BoundingBox bbox;
    osg::Matrix m;
    std::string filename;
    int id;
}imgData;
using namespace std;
struct Camera
{
    float focal;
    cv::Mat R, t;
    cv::Mat invK;

    float x, y, z; // position
    float nx, ny, nz; // normal

    Camera()
    {
        R = cv::Mat::zeros(3, 3, CV_32F);
        t = cv::Mat::zeros(3, 1, CV_32F);
    }
    bool valid;
};


struct Point3D
{
    float x, y, z;
    vector <int> visible_in;
};
struct Face
{
    int image_num;
    unsigned int v1, v2, v3;
    float x, y, z; // centre of triangle
    float nx, ny, nz;
    float u[3], v[4]; // texture UV coords
    vector <int> visible_in;
};
void LoadBundle(const string& filename, vector <Camera> &cameras)
{
    ifstream bundle(filename.c_str());

    if(!bundle) {
        cerr << "LoadBundle(): Error opening " << filename << " for reading" << endl;
        exit(1);
    }

    cout << "Loading " << filename << " ... " << endl;

    stringstream str;
    char line[1024];
    int num;

    bundle.getline(line, sizeof(line)); // header
    bundle.getline(line, sizeof(line));

    str.str(line);
    str >> num;

    cameras.resize(num);

    cout << "    cameras = " << cameras.size() << endl;

    for(int i=0; i < num; i++) {
        Camera new_camera;
        new_camera.valid=true;
        bundle.getline(line, sizeof(line)); // focal, r1, r2

        str.str(line);
        str.clear();
        str >> new_camera.focal;

        bundle.getline(line, sizeof(line)); // rotation 1
        str.str(line);
        str.clear();
        str >> new_camera.R.at<float>(0,0);
        str >> new_camera.R.at<float>(0,1);
        str >> new_camera.R.at<float>(0,2);

        bundle.getline(line, sizeof(line)); // rotation 2
        str.str(line);
        str.clear();
        str >> new_camera.R.at<float>(1,0);
        str >> new_camera.R.at<float>(1,1);
        str >> new_camera.R.at<float>(1,2);

        bundle.getline(line, sizeof(line)); // rotation 3
        str.str(line);
        str.clear();
        str >> new_camera.R.at<float>(2,0);
        str >> new_camera.R.at<float>(2,1);
        str >> new_camera.R.at<float>(2,2);

        bundle.getline(line, sizeof(line)); // translation
        str.str(line);
        str.clear();
        str >> new_camera.t.at<float>(0,0);
        str >> new_camera.t.at<float>(1,0);
        str >> new_camera.t.at<float>(2,0);

        // Camera position and normal
        {
            // http://phototour.cs.washington.edu/bundler/bundler-v0.4-manual.html
            // -R' * t
            float r[9];
            float t[3];

            for(int i=0; i < 3; i++) {
                for(int j=0; j < 3; j++) {
                    r[i*3+j] = new_camera.R.at<float>(i,j);
                }
            }

            for(int i=0; i < 3; i++) {
                t[i] = new_camera.t.at<float>(i,0);
            }

            new_camera.x = -r[6]*t[2] - r[3]*t[1] - r[0]*t[0];
            new_camera.y = -r[7]*t[2] - r[4]*t[1] - r[1]*t[0];
            new_camera.z = -r[8]*t[2] - r[5]*t[1] - r[2]*t[0];

            new_camera.nx = -r[6];
            new_camera.ny = -r[7];
            new_camera.nz = -r[8];;
        }

        cameras[i] = new_camera;
    }

    bundle.close();
}
void readFile(string fname,map<int,imgData> &imageList){

    std::ifstream m_fin(fname.c_str());
    if(!osgDB::fileExists(fname.c_str())||!m_fin.good() ){
        fprintf(stderr,"Can't load %s\n",fname.c_str());
        exit(-1);
    }
    while(!m_fin.eof()){
        imgData cam;
        double low[3], high[3];
        if(m_fin >> cam.id >> cam.filename >> low[0] >> low[1] >> low[2] >> high[0] >> high[1] >> high[2]
                >> cam.m(0,0) >>cam.m(0,1)>>cam.m(0,2) >>cam.m(0,3)
                >> cam.m(1,0) >>cam.m(1,1)>>cam.m(1,2) >>cam.m(1,3)
                >> cam.m(2,0) >>cam.m(2,1)>>cam.m(2,2) >>cam.m(2,3)
                >> cam.m(3,0) >>cam.m(3,1)>>cam.m(3,2) >>cam.m(3,3)){
            cam.bbox.expandBy(low[0],low[1],low[2]);
            cam.bbox.expandBy(high[0],high[1],high[2]);
            imageList[cam.id]=cam;
        }
    }
}
void AssignVisibleCameras(const osg::Vec3Array *vertices, const vector <Point3D> &points, const vector <Camera> &cameras,
                          osg::ref_ptr<osg::DrawElementsUInt> &triangles,osg::ref_ptr<osg::Vec4Array> &textureID,map<int,imgData> &imageList)
{
    ANNWrapper ann;

    vector <Vertex> tmp(points.size());

    for(size_t i=0; i < tmp.size(); i++) {
        tmp[i].x = points[i].x;
        tmp[i].y = points[i].y;
        tmp[i].z = points[i].z;
    }

    ann.SetPoints(tmp);

    for(int i=0; i< (int)triangles->size()-2; i+=3){
        osg::Vec3 v[3];

        v[0]=vertices->at(triangles->at(i));
        v[1]=vertices->at(triangles->at(i+1));
        v[2]=vertices->at(triangles->at(i+2));

        Vertex c;

        c.x = (v[0].x() + v[1].x() + v[2].x())/3.0;
        c.y = (v[0].y() + v[1].y()+ v[2].y())/3.0;
        c.z = (v[0].z() + v[1].z() + v[2].z())/3.0;

        double dist2;
        int index;

        ann.FindClosest(c, &dist2, &index);

        //faces[i].visible_in = points[index].visible_in;
        vector <int> visible_in_valid;
        for( int j=0; j < (int)points[index].visible_in.size(); j++){
            if(imageList.count(points[index].visible_in[j])){
                visible_in_valid.push_back(points[index].visible_in[j]);
            }
        }
        for(int k=0; k<4; k++)
            for(int l=0; l<3; l++){
                if(k < (int)visible_in_valid.size())
                    textureID->at(i+l)[k]=(float)visible_in_valid[k];
                else
                    textureID->at(i+l)[k]= -1.0f;
            }

    }
}

int IMAGE_WIDTH = -1;
int IMAGE_HEIGHT = -1;
void AssignTexture(const osg::Vec3Array *vertices, const vector <Camera> &cameras, osg::ref_ptr<osg::DrawElementsUInt> &triangles,
                   osg::ref_ptr<osg::Vec4Array> &textureID,
                   std::vector<osg::ref_ptr<osg::Vec3Array> > &texCoords)
{
    cv::Mat X(3,1,CV_32F);
    float cx = IMAGE_WIDTH*0.5f;
    float cy = IMAGE_HEIGHT*0.5f;

  // for(size_t i=0; i < faces.size(); i++) {
    for(int i=0; i< (int)triangles->size()-2; i+=3){
        osg::Vec3 v[3];

        v[0]=vertices->at(triangles->at(i));
        v[1]=vertices->at(triangles->at(i+1));
        v[2]=vertices->at(triangles->at(i+2));

        for(int j=0; j<4; j++){
            // Pick the first image
            int image_num = (int)textureID->at(i)[j];//faces[i].image_num;
            if( image_num < 0 || image_num >= (int)cameras.size()){
                printf("Camera: %d (%.2f,%.2f)\n",image_num,-1.0,-1.0);

                continue;
            }
            const Camera &camera = cameras[image_num];

            for(int k=0; k < 3; k++) {
                X.at<float>(0,0) = v[k].x();
                X.at<float>(1,0) = v[k].y();
                X.at<float>(2,0) = v[k].z();

                X = camera.R*X + camera.t;

                float xx = -X.at<float>(0,0) / X.at<float>(2,0);
                float yy = -X.at<float>(1,0) / X.at<float>(2,0);

                float u = (xx*camera.focal + cx) / IMAGE_WIDTH;
                float v = (yy*camera.focal + cy) / IMAGE_HEIGHT;
                if(u > 1.0 || u <0.0 || v>1.0 || v <0.0)
                    continue;
                texCoords[j]->at(i+k)=osg::Vec3(u,v,0.0);
                printf("Camera: %d (%.2f,%.2f)\n",image_num,u,v);
             //   faces[i].u[k] = u;
             //   faces[i].v[k] = v;
            }
        }
    }
}

void GetImageInfo(const string &filename, int &width, int &height)
{
    cv::Mat img = cv::imread(filename);

    if(!img.data) {
        cerr << "Error opening " << filename << endl;
        exit(1);
    }

    width = img.cols;
    height = img.rows;
}

bool LoadPMVSPatch(const string& patch_file, vector <Point3D> &points)
{
    ifstream input(patch_file.c_str());

    if(!input) {
        cerr << "LoadPMVSPatch(): Error opening " << patch_file << " for reading" << endl;
        return false;
    }

    cout << "Loading " << patch_file << " ... " << endl;

    char line[2048];
    char word[128];
    stringstream str;
    int num_pts;
    float not_used;

    input.getline(line, sizeof(line)); // header
    str.str(line);
    str >> word;

    if(word != string("PATCHES")) {
        cerr << "LoadPMVSPatch(): Incorrect header" << endl;
        return false;
    }

    input.getline(line, sizeof(line)); // number of points
    str.str(line);
    str.clear();
    str >> num_pts;

    cout << "    points = " << num_pts << endl;

    points.resize(num_pts);

    for(int i=0; i < num_pts; i++) {
        Point3D &pt = points[i];
        int num;

        input.getline(line, sizeof(line)); // another header
        input.getline(line, sizeof(line)); // position
        str.str(line);
        str.clear();
        str >> pt.x;
        str >> pt.y;
        str >> pt.z;

        input.getline(line, sizeof(line)); // normal
        str.str(line);
        str.clear();
        str >> not_used;
        str >> not_used;
        str >> not_used;

        input.getline(line, sizeof(line)); // debugging stuff
        input.getline(line, sizeof(line)); // num images visible in
        str.str(line);
        str.clear();
        str >> num;

        assert(num > 0);

        input.getline(line, sizeof(line)); // image index
        str.str(line);
        str.clear();

        for(int j=0; j < num; j++) {
            int idx;
            str >> idx;
            pt.visible_in.push_back(idx);
        }

        input.getline(line, sizeof(line)); // possibly visible in
        str.str(line);
        str.clear();
        str >> num;

        input.getline(line, sizeof(line)); // image index
        str.str(line);
        str.clear();
        str >> num;

        for(int j=0; j < num; j++) {
            int idx;
            str >> idx;
            pt.visible_in.push_back(idx);
        }

        input.getline(line, sizeof(line)); // blank line

        if(input.eof()) {
            cerr << "LoadPMVSPatch(): Premature end of file" << endl;
            return false;
        }
    }

    return true;
}

int main( int argc, char **argv )
{
    // use an ArgumentParser object to manage the program arguments.
    osg::ArgumentParser arguments(&argc,argv);

    // set up the usage document, in case we need to print out how to use this program.
    arguments.getApplicationUsage()->setDescription(arguments.getApplicationName() +" is the example which demonstrates Depth Peeling");
    arguments.getApplicationUsage()->setCommandLineUsage(arguments.getApplicationName()+" filename");
    ;
    string path=string(argv[0]);
    unsigned int loc=path.rfind("/");

    string basepath= loc == string::npos ? "./" : path.substr(0,loc+1);
    basepath= osgDB::getRealPath (basepath);
    string base_dir=argv[1];



    string outfilename;
    if(!arguments.read("--outfile",outfilename)){
        fprintf(stderr,"Need outfile name\n");
        return -1;
    }

    string camimgfile;
    if(!arguments.read("--camimgfile",camimgfile)){
        fprintf(stderr,"Need camimgfile \n");
        return -1;
    }
    map<int,imgData> imageList;
    readFile(camimgfile,imageList);

    vector<Camera> cameras;
    vector <Point3D> points;

    LoadBundle(base_dir+ "/bundle.rd.out",cameras);
    GetImageInfo(base_dir + "/visualize/00000000.jpg", IMAGE_WIDTH, IMAGE_HEIGHT);

    for(int i=0; i < 1000; i++) {
        vector <Point3D> tmp;

        char str[256];
        sprintf(str, "%s/models/option-%04d.patch", base_dir.c_str(), i);

        FILE *fp = fopen(str, "r");

        if(fp == NULL) {
            break;
        }

        fclose(fp);

        LoadPMVSPatch(str, tmp);

        points.insert(points.end(), tmp.begin(), tmp.end());
    }


    std::string tex_cache_dir;
    texcache_t cache;
    osg::Vec4 zrange;
    arguments.read("--zrange",zrange[0],zrange[1]);
    float tex_margin=0.0;
    float bbox_margin=0.0;
    if(    arguments.read("--tex-margin",tex_margin)){
        printf("Tex Margin %f\n",tex_margin);
    }
    if(    arguments.read("--bbox-margin",bbox_margin)){
        printf("BBox Margin %f\n",bbox_margin);
    }

 //   int row,col,numRows,numCols,width,height;
  //  int size;


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
    ply::VertexDataMosaic vertexData;
    osg::Node* model = vertexData.readPlyFile(mf.c_str());//osgDB::readNodeFile(sourceModel->getFileName().c_str());

    if(!model){
        fprintf(stderr,"Can't load model %s\n",mf.c_str());
        return -1;
    }
    osg::ref_ptr<osg::KdTreeBuilder>  _kdTreeBuilder = osgDB::Registry::instance()->getKdTreeBuilder()->clone();
    model->accept(*_kdTreeBuilder);
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
        /*osg::KdTree* kdTree=NULL;
        osg::Geode *geode= dynamic_cast<osg::Geode*>(model);
        if(geode && geode->getNumDrawables()){
            //addDups(geode);
            osg::Drawable *drawable = geode->getDrawable(0);
            kdTree = dynamic_cast<osg::KdTree*>(drawable->getShape());
        }else{
            std::cerr << "No drawbables \n";
        }
          */
            osg::ref_ptr<osg::Vec4Array> textureID=new osg::Vec4Array;
            std::vector<osg::ref_ptr<osg::Vec3Array> > texCoords;
            texCoords.resize(4);
            texCoords[0]=new osg::Vec3Array;
            texCoords[1]=new osg::Vec3Array;
            texCoords[2]=new osg::Vec3Array;
            texCoords[3]=new osg::Vec3Array;
            texCoords[0]->resize(vertexData._vertices->size(),osg::Vec3(-1.0,-1.0,-1.0));
            texCoords[1]->resize(vertexData._vertices->size(),osg::Vec3(-1.0,-1.0,-1.0));
            texCoords[2]->resize(vertexData._vertices->size(),osg::Vec3(-1.0,-1.0,-1.0));
            texCoords[3]->resize(vertexData._vertices->size(),osg::Vec3(-1.0,-1.0,-1.0));
            textureID->resize(vertexData._vertices->size(),osg::Vec4(-1.0,-1.0,-1.0,-1.0));;
            AssignVisibleCameras(vertexData._vertices, points, cameras,vertexData._triangles,textureID,imageList);
            AssignTexture(vertexData._vertices,cameras,vertexData._triangles,textureID,texCoords);
       // bool projectSucess=tq->projectModel(dynamic_cast<osg::Geode*>(model),tex_margin);
        if(1){
            //  writeCached(outfilename,sha2hash,tile->texCoordIDIndexPerModel.begin()->second,tile->texCoordsPerModel.begin()->second);
            //osg::Geometry *geom = dynamic_cast< osg::Geometry*>( geode->getDrawable(0));
            // for(int f=0; f<tile->texCoordsPerModel.begin()->second.size(); f++)
            //    geom->setTexCoordArray(f,tile->texCoordsPerModel.begin()->second[f]);
                std::ofstream f(outfilename.c_str());
                std::vector<bool> *marginFace=NULL;
                if(vertexData._texIds.valid() && vertexData._texIds->size()){
                    marginFace= new std::vector<bool>( vertexData._vertices->size(),false);
                    if( vertexData._vertices->size() !=  vertexData._texIds->size()*3){
                        fprintf(stderr,"%d should be 3 time %d\n", (int)vertexData._vertices->size(),  (int)vertexData._texIds->size());
                        exit(-1);
                    }
                    for(int i=0; i< (int)vertexData._triangles->size(); i+=3)
                        for(int j=0;j<3; j++)
                            marginFace->at(vertexData._triangles->at(i+j))= (vertexData._texIds->at(i/3).y() == -999);
                }

                PLYWriterNodeVisitor nv(f,textureID,&texCoords,"",marginFace,vertexData._colors);
                model->accept(nv);


        }else
            cerr << "Failed to project\n";
    }else
        cerr << "Failed to open "<<mf <<endl;

}
