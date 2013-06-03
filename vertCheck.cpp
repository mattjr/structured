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
#include <osg/NodeVisitor>
#include <osgDB/WriteFile>
#include <osgDB/FileNameUtils>
#include <osgUtil/Optimizer>
#include <osg/io_utils>
#include <osg/MatrixTransform>
#include <iostream>
void write_header(std::ostream& _fout,int total_face_count,bool color){
    _fout <<"ply\n";
    _fout <<"format binary_little_endian 1.0\n";
    //_fout <<"comment PLY exporter written by Paul Adams\n";
    _fout <<"element vertex "<<total_face_count <<std::endl;
    _fout <<"property float x\n";
    _fout <<"property float y\n";
    _fout <<"property float z\n";
    if(color){
        _fout <<"property uchar red\n";
        _fout <<"property uchar green\n";
        _fout <<"property uchar blue\n";
    }
    _fout <<"element face " <<total_face_count/3<<std::endl;
    _fout <<"property list uchar int vertex_indices\n";

    _fout <<"end_header\n";
}
void write_all(std::ostream& _fout,osg::DrawElementsUInt *tri,osg::Vec3Array *verts,osg::Vec4Array *colors,bool flip,osg::Matrix m){
    int cnt=0;
    for(int i=0; i< (int)tri->size()-2; i+=3){
        for(int j=0; j<3; j++){
            osg::Vec3 v=verts->at(tri->at(i+j))*m;
            float vf[3];
            vf[0]=v[0];
            vf[1]=v[1];
            vf[2]=v[2];
            _fout.write((char *)vf,3*sizeof(float));
            if(colors && i+j <(int)colors->size() ){
                unsigned char col[3];
                osg::Vec4 c=colors->at(i+j);
               // cout <<c<<endl;
                col[0]=c[0]*255.0;
                col[1]=c[1]*255.0;
                col[2]=c[2]*255.0;
                _fout.write((char *)col,3*sizeof(unsigned char));

            }
        }
    }
    int iout[3];
    unsigned char c=3;
    for(int i=0; i<(int) tri->size()-2; i+=3){
        _fout.write((char *)&c,sizeof(char));

        if(flip){
            iout[0]=i;
            iout[1]=i+1;
            iout[2]=i+2;
        }else{
            iout[0]=i+2;
            iout[1]=i+1;
            iout[2]=i+0;

        }
        _fout.write((char*)iout,sizeof(int)*3);
        cnt++;

    }
    printf("%d\n",cnt);


}
unsigned int countGeometryVertices( osg::Geometry* geom )
{
    if (!geom->getVertexArray())
        return 0;

    // TBD This will eventually iterate over the PrimitiveSets and total the
    //   number of vertices actually used. But for now, it just returns the
    //   size of the vertex array.

    return geom->getVertexArray()->getNumElements();
}

class VertexCounter : public osg::NodeVisitor
{
public:
    VertexCounter( int limit )
        : osg::NodeVisitor( osg::NodeVisitor::TRAVERSE_ALL_CHILDREN ),
        _limit( limit ),
        _total( 0 ) {}
    ~VertexCounter() {}

    int getTotal() { return _total; }
    bool exceeded() const { return _total > _limit; }
    void reset() { _total = 0; }

    virtual void apply( osg::Node& node )
    {
        // Check for early abort. If out total already exceeds the
        //   max number of vertices, no need to traverse further.
        if (exceeded())
            return;
        traverse( node );
    }

    virtual void apply( osg::Geode& geode )
    {
        // Possible early abort.
        if (exceeded())
            return;

        unsigned int i;
        for( i = 0; i < geode.getNumDrawables(); i++ )
        {
            osg::Geometry* geom = dynamic_cast<osg::Geometry *>(geode.getDrawable(i));
            if( !geom )
                continue;

            _total += countGeometryVertices( geom );

            if (_total > _limit)
                break;
        }
    }

protected:
    int _limit;
    int _total;
};
int main( int argc, char **argv )
{
    // use an ArgumentParser object to manage the program arguments.
    osg::ArgumentParser arguments(&argc,argv);
    bool rot=false;
    float rx, ry, rz;
    osg::Matrix inverseM=osg::Matrix::identity();
    if(arguments.read("--invrot",rx,ry,rz)){
        inverseM =osg::Matrix::rotate(
                osg::DegreesToRadians( rx ), osg::Vec3( 1, 0, 0 ),
                osg::DegreesToRadians( ry ), osg::Vec3( 0, 1, 0 ),
                osg::DegreesToRadians( rz ), osg::Vec3( 0, 0, 1 ) );
        rot=true;
    }
    osg::Matrix rotM=osg::Matrix::inverse(inverseM);
    osg::ref_ptr<osg::Node> model = osgDB::readNodeFiles(arguments);
    if(!model.valid()){
        fprintf(stderr,"Failed to load model\n");
        exit(-1);
    }
    if(rot){
        osg::ref_ptr<osg::MatrixTransform>xform = new osg::MatrixTransform;
        xform->setDataVariance( osg::Object::STATIC );
        xform->setMatrix(rotM);
        osgUtil::Optimizer::FlattenStaticTransformsVisitor fstv(NULL);
        xform->addChild(model);
        xform->accept(fstv);
        fstv.removeTransforms(xform);
    }
    std::string outfilename;
    if(!arguments.read("--outfile",outfilename)){
        fprintf(stderr,"Need outfile name\n");
        return -1;
    }
    bool foundcolors=false;
  osg::Vec4Array*colors;
osg::DrawElementsUInt *tri; osg::Vec3Array *verts;
            osg::Geode *geode= dynamic_cast<osg::Geode*>(model.get());
            if(!geode)
                geode=model->asGroup()->getChild(0)->asGeode();
            if(geode && geode->getNumDrawables()){


                osg::Drawable *drawable = geode->getDrawable(0);
                osg::Geometry *geom = dynamic_cast< osg::Geometry*>(drawable);
                colors=  (osg::Vec4Array*)geom->getColorArray();
           verts=static_cast< osg::Vec3Array*>(geom->getVertexArray());

                tri= (osg::DrawElementsUInt*)geom->getPrimitiveSet(0);
               if(arguments.read("--normcolor") ){
                if(colors){
                    foundcolors=true;

                    osg::Vec3 minC(1.0,1.0,1.0),maxC(0.0,0.0,0.0);
                    for(int i=0; i < (int)colors->size(); i++){
                        for(int j=0; j<3; j++){
                            if(colors->at(i)[j] < minC[j] && colors->at(i)[j] > 0.0)
                                minC[j] =colors->at(i)[j] ;

                            if(colors->at(i)[j] > maxC[j] && colors->at(i)[j] < 1.0)
                                maxC[j]= colors->at(i)[j] ;
                        }
                    }
                    osg::Vec3 range;
                    for(int j=0; j<3; j++){
                        range[j]=maxC[j]-minC[j];
                    }
                    std::cout << "Min "<<minC <<" MaxC " << maxC << " range "<<range<<std::endl;
                    for(int i=0; i < (int)colors->size(); i++){
                        for(int j=0; j<3; j++){
                            colors->at(i)[j]=(colors->at(i)[j]- minC[j])/range[j];
                        }

                    }

                }
            }

    }
    VertexCounter vc( 100 );
    model->accept( vc );
    if(vc.getTotal()>0){
        if(osgDB::getFileExtension(outfilename) == "ply"){

            std::ofstream f(outfilename.c_str());
            write_header(f,tri->size(),foundcolors);
            write_all(f,tri,verts,colors,true,inverseM);
            //PLYWriterNodeVisitor nv(f);
            //root->accept(nv);
            f.close();;
        }else{


            osgDB::writeNodeFile(*model,outfilename.c_str());
        }
    }
}

