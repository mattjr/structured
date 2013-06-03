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

#include "GLImaging.h"
#include "MemUtils.h"
#include "TexturedSource.h"
void readNextEntry(std::ifstream &m_fin,    TexturedSource::ProjectionCamera &cam)
{
    double low[3], high[3];
    osg::Matrix m;
    m_fin >> cam.id >> cam.filename >> low[0] >> low[1] >> low[2] >> high[0] >> high[1] >> high[2]
            >> m(0,0) >>m(0,1)>>m(0,2) >>m(0,3)
            >> m(1,0) >>m(1,1)>>m(1,2) >>m(1,3)
            >> m(2,0) >>m(2,1)>>m(2,2) >>m(2,3)
            >> m(3,0) >>m(3,1)>>m(3,2) >>m(3,3);/*
    >> m(0,0) >>m(1,0)>>m(2,0) >>m(3,0)
    >> m(0,1) >>m(1,1)>>m(2,1) >>m(3,1)
    >> m(0,2) >>m(1,2)>>m(2,2) >>m(3,2)
    >> m(0,3) >>m(1,3)>>m(2,3) >>m(3,3);*/
    if (m_fin.good())
    {
        cam.m=m;//osg::Matrix::inverse(m);
        //std::cout << m<<std::endl;

        cam.bb = osg::BoundingBox(low[0],low[1],low[2],high[0],high[1],high[2]);

        // Associate a bogus data array with every entry for testing purposes.
        // Once the data array is given to RTRee:Data a local copy will be created.
        // Hence, the input data array can be deleted after this operation if not
        // needed anymore.
    }
}

int main(int argc, char** argv)
{
    // use an ArgumentParser object to manage the program arguments.
    osg::ArgumentParser arguments(&argc,argv);

    arguments.getApplicationUsage()->setApplicationName(arguments.getApplicationName());
    arguments.getApplicationUsage()->setCommandLineUsage(arguments.getApplicationName()+" [options] filename ...");


    unsigned int width=512;
    unsigned int height=512;
    arguments.read("--pbuffer-only",width,height);
    int _tileColumns=0;
    int _tileRows=0;

    osg::BoundingBox totalbb;


    string ext;
    arguments.read("-nogfx",ext);

    std::vector<picture_cell> cells;
    FILE *fp=fopen(argv[1],"r");
    if(!fp){
        fprintf(stderr,"Can't open %s\n",argv[1]);
        exit(-1);
    }
    int cnt=0;
    while(!feof(fp)){
        char fname[1024];
        float minx,maxx,miny,maxy,minz,maxz;
        int row,col;
        int res=fscanf(fp,"%f %f %f %f %f %f %d %d %s\n",&minx,&maxx,&miny,&maxy,&minz,&maxz,&col,&row,fname);
        if(res != 9){
            fprintf(stderr,"Bad parse\n");
            exit(-1);
        }
        if(cnt==0){
            totalbb=osg::BoundingBox(minx,miny,minz,maxx,maxy,maxz);
            _tileColumns=col;
            _tileRows=row;

        }else{
            picture_cell cell;
            cell.bbox=osg::BoundingBox(minx,miny,minz,maxx,maxy,maxz);
            cell.col=col;
            cell.row=row;
            if(std::string(fname) != "null"){
                cell.name=std::string(fname);
                cells.push_back(cell);

            }
            else
                cell.name=std::string(fname);

        }
        cnt++;

    }





    unsigned int validCount=0;
    for(int i=0; i < (int)cells.size(); i++){
        if(cells[i].name != "null")
            validCount++;
    }
    string path=string(argv[0]);
    unsigned int loc=path.rfind("/");

    string basepath= loc == string::npos ? "./" : path.substr(0,loc+1);
    basepath= osgDB::getRealPath (basepath);
    FILE *fpp=fopen("rangecmd","w");
    //int count=0;
    for(int i=0; i < (int)cells.size(); i++)
    {
        if(cells[i].name == "null" )
            continue;

        char tmp[1024];
        char tmpbb[1024];

        sprintf(tmp,"%s/image_r%04d_c%04d_rs%04d_cs%04d-tmp.%s",diced_img_dir,cells[i].row,cells[i].col,_tileRows,_tileColumns,ext.c_str());
        sprintf(tmpbb,"%s/bbox-tmp-tex-clipped-diced-r_%04d_c_%04d.ply.txt",diced_img_dir,cells[i].row,cells[i].col);
       // std::ifstream fin(tmpbb);
        //TexturedSource::ProjectionCamera cam;
     //   if(fin.good())
       //  readNextEntry(fin,cam);
        fprintf(fpp,"%s/rangeimg %s/tmp-tex-clipped-diced-r_%04d_c_%04d.ply %s/bbox-tmp-tex-clipped-diced-r_%04d_c_%04d.ply.txt \n",basepath.c_str(),
                diced_dir,cells[i].row,cells[i].col,diced_img_dir,cells[i].row,cells[i].col);
    }
    fclose(fpp);





}
