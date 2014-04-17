/* * structured - Tools for the Generation and Visualization of Large-scale
 * Three-dimensional Reconstructions from Image Data. This software includes
 * source code from other projects, which is subject to different licensing,
 * see COPYING for details. If this project is used for research see COPYING
 * for making the appropriate citations.
 * Copyright (C) 2013 Matthew Johnson-Roberson <mattkjr@gmail.com>
 *
 * This file is part of structured.
 *
 * structured is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * structured is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with structured.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef SPLITBOUNDS_H
#define SPLITBOUNDS_H
#include "stereo_cells.hpp"
#include <map>
#include <vector>
#include <stdio.h>
#include <iostream>
#include <osg/io_utils>
#include "OctreeBuilder.h"
template <typename ValType>
struct CellDataT
{
typedef std::vector<std::vector<std::vector<Cell_Data<ValType> > > >  type;
};
int count_vol(const CellDataT<Stereo_Pose_Data>::type &container) ;

#define foreach_vol(var, container) \
    for(unsigned int ii=0; ii < container.size(); ii++)\
    for(unsigned int jj=0; jj < container[ii].size(); jj++)\
    for(typeof((container[ii][jj]).begin()) var = (container[ii][jj]).begin(); \
         var != (container[ii][jj]).end(); \
         ++var)

#define foreach_vol2(var, container) \
    for(unsigned int iii=0; iii < container.size(); iii++)\
    for(unsigned int jjj=0; jjj < container[iii].size(); jjj++)\
    for(typeof((container[iii][jjj]).begin()) var = (container[iii][jjj]).begin(); \
         var != (container[iii][jjj]).end(); \
         ++var)
template <typename CellType>
bool split_boundsOctree(const Bounds &bounds,const std::vector<CellType> &poses ,int maxFaces,int minSplits,typename CellDataT<CellType>::type &Vol){
    OctreeBuilder<CellType> octree;
    int splits[3]={0,0,0};
    int zero[3]={0,0,0};

    std::vector< std::pair<std::vector<CellType> , osg::BoundingBox> > cells;
    octree.setMaxChildNumber(2);
    octree.setMaxFaces(maxFaces);
    octree.build(splits, cells,zero, bounds.bbox, poses );
    //printf("Splits %d %d %d\n",splits[0],splits[1],splits[2]);

    double ran[3];
    for(int i=0; i<3; i++)
        ran[i]=(bounds.bbox._max[i]-bounds.bbox._min[i]);

//    int split=pow(2,largestDepth);
  //  int splits[3];
   // for(int i=0;i<3; i++)
     //   splits[i]=splits[];


  /*  if(minSplits>0){
        while( splits[0]* splits[1]* splits[2] < minSplits){
            splits[0]++;
            splits[1]++;
        }
    }*/
    //Ensure one volume
    for(int i=0; i<3; i++)
        splits[i]=std::max(splits[i],1);
    printf("Split %d poses into %d %d %d\n",(int)poses.size(),splits[0],splits[1],splits[2]);
    double stepSize[3];
    for(int i=0;i<3; i++)
        stepSize[i]=ran[i]/(double)splits[i];

    Vol.resize(splits[0]);
    for(unsigned int j=0; j<Vol.size(); j++){
        Vol[j].resize(splits[1]);
         for(unsigned int k=0; k<Vol[j].size(); k++)
            Vol[j][k].resize(splits[2]);
    }

    for(int i=0; i< splits[0]; i++){
        for(int j=0; j< splits[1]; j++){
            for(int k=0; k< splits[2]; k++){
                osg::BoundingBox cellBounds(bounds.bbox.xMin()+(i*stepSize[0]) ,bounds.bbox.yMin()+(j*stepSize[1]),bounds.bbox.zMin()+(k*stepSize[2]),
                                            bounds.bbox.xMin()+((i+1)*stepSize[0]),bounds.bbox.yMin()+((j+1)*stepSize[1]),bounds.bbox.zMin()+((k+1)*stepSize[2]));
                std::vector<const CellType *> touchedPoses;
                for(int l=0; l<(int)poses.size(); l++){
                    if(cellBounds.intersects(poses[l].bbox))
                        touchedPoses.push_back(&poses[l]);

                }
                Bounds cB;
                cB.bbox=cellBounds;
                Cell_Data<CellType> cell(touchedPoses,cB);
                cell.volIdx[0]=i;
                cell.volIdx[1]=j;
                cell.volIdx[2]=k;
                cell.splits[0]=splits[0];
                cell.splits[1]=splits[1];
                cell.splits[2]=splits[2];

                Vol[i][j][k]=cell;
            }
        }
    }

    return true;

}
#if 0

template <typename CellType>
bool split_boundsOctree(const Bounds &bounds,const std::vector<CellType> &poses ,int maxFaces,int minSplits,typename CellDataT<CellType>::type &Vol){
    OctreeBuilder<CellType> octree;
    int largestDepth=0;
    std::vector< std::pair<std::vector<CellType> , osg::BoundingBox> > cells;
    octree.setMaxChildNumber(2);
    octree.setMaxFaces(maxFaces);
    octree.build(largestDepth, cells,0, bounds.bbox, poses );
    printf("Largest Depth %d\n",largestDepth);
/*
    double ran[3];
    for(int i=0; i<3; i++)
        ran[i]=(bounds.bbox._max[i]-bounds.bbox._min[i]);

    int splits[3];
    for(int i=0;i<3; i++)
        splits[i]=ceil(ran[i]/targetSide);


    if(minSplits>0){
        while( splits[0]* splits[1]* splits[2] < minSplits){
            splits[0]++;
            splits[1]++;
        }
    }
    //Ensure one volume
    for(int i=0; i<3; i++)
        splits[i]=std::max(splits[i],1);
    printf("Split %d poses into %d %d %d\n",(int)poses.size(),splits[0],splits[1],splits[2]);
    double stepSize[3];
    for(int i=0;i<3; i++)
        stepSize[i]=ran[i]/(double)splits[i];
*/
    Vol.resize(1);
   /* for(unsigned int j=0; j<Vol.size(); j++){
        Vol[j].resize(splits[1]);
         for(unsigned int k=0; k<Vol[j].size(); k++)
            Vol[j][k].resize(splits[2]);
    }*/
    Vol[0].resize(1);
    std::cout << "Split into " << cells.size()<<std::endl;
    for(int i=0; i<(int)cells.size(); i++){
        osg::BoundingBox cellBounds=cells[i].second;
        std::vector<const CellType *> touchedPoses;
        int face_count=0;
        for(int l=0; l<(int)cells[i].first.size(); l++){
                touchedPoses.psplit_boundsOctreeush_back(&cells[i].first[l]);
                face_count+=cells[i].first[l].faces;
        }
        Bounds cB;
        cB.bbox=cellBounds;
        Cell_Data<CellType> cell(touchedPoses,cB);
        cell.volIdx[0]=0;
        cell.volIdx[1]=0;
        cell.volIdx[2]=i;
        cell.splits[0]=1;
        cell.splits[1]=1;
        cell.splits[2]=cells.size();
        std::cout << cell <<std::endl;
        //std::cout <<"Faces " << face_count<<std::endl;
        Vol[0][0].push_back(cell);


    }
    return true;

}
#endif
template <typename CellType>
bool split_bounds(const Bounds &bounds,const std::vector<CellType> &poses ,double targetSide,int minSplits,typename CellDataT<CellType>::type &Vol){

    double ran[3];
    for(int i=0; i<3; i++)
        ran[i]=(bounds.bbox._max[i]-bounds.bbox._min[i]);

    int splits[3];
    for(int i=0;i<3; i++)
        splits[i]=ceil(ran[i]/targetSide);


    if(minSplits>0){
        while( splits[0]* splits[1]* splits[2] < minSplits){
            splits[0]++;
            splits[1]++;
        }
    }
    //Ensure one volume
    for(int i=0; i<3; i++)
        splits[i]=std::max(splits[i],1);
    printf("Split %d poses into %d %d %d\n",(int)poses.size(),splits[0],splits[1],splits[2]);
    double stepSize[3];
    for(int i=0;i<3; i++)
        stepSize[i]=ran[i]/(double)splits[i];

    Vol.resize(splits[0]);
    for(unsigned int j=0; j<Vol.size(); j++){
        Vol[j].resize(splits[1]);
         for(unsigned int k=0; k<Vol[j].size(); k++)
            Vol[j][k].resize(splits[2]);
    }

    for(int i=0; i< splits[0]; i++){
        for(int j=0; j< splits[1]; j++){
            for(int k=0; k< splits[2]; k++){
                osg::BoundingBox cellBounds(bounds.bbox.xMin()+(i*stepSize[0]) ,bounds.bbox.yMin()+(j*stepSize[1]),bounds.bbox.zMin()+(k*stepSize[2]),
                                            bounds.bbox.xMin()+((i+1)*stepSize[0]),bounds.bbox.yMin()+((j+1)*stepSize[1]),bounds.bbox.zMin()+((k+1)*stepSize[2]));
                std::vector<const CellType *> touchedPoses;
                for(int l=0; l<(int)poses.size(); l++){
                    if(cellBounds.intersects(poses[l].bbox))
                        touchedPoses.push_back(&poses[l]);

                }
                Bounds cB;
                cB.bbox=cellBounds;
                Cell_Data<CellType> cell(touchedPoses,cB);
                cell.volIdx[0]=i;
                cell.volIdx[1]=j;
                cell.volIdx[2]=k;
                cell.splits[0]=splits[0];
                cell.splits[1]=splits[1];
                cell.splits[2]=splits[2];

                Vol[i][j][k]=cell;
            }
        }
    }

    return true;

}
enum{
  VOLFILL_STEP,
  VRIPCLEAN_STEP,
  PLYMERGE_STEP
};

typedef struct _picture_cell{
    int row;
    int col;
    osg::BoundingBox bbox;
    osg::BoundingBox bboxMargin;

    osg::BoundingBox bboxUnRot;
    osg::BoundingBox bboxMarginUnRot;
    osg::Matrixd m;
    std::string name;
    std::vector<int> images;
    std::vector<int> imagesMargin;

}picture_cell;

class WriteTP{
public:
    WriteTP(double res,std::string fname,std::string basepath,std::string cwd):_res(res),cmdfn(fname),_basepath(basepath),cmdfp(NULL),_cwd(cwd){}
//    virtual bool write_cmd(Cell_Data<Stereo_Pose_Data> cell)=0;
protected:
    double _res;

    std::string cmdfn;
    std::string _basepath;

    FILE *cmdfp;
    std::string _cwd;

};

class WriteSplitTP : public WriteTP{
    public:
    WriteSplitTP(double res,std::string fname,std::string basepath,std::string cwd,const std::vector<Stereo_Pose_Data> &tasks,const  CellDataT<Stereo_Pose_Data>::type &Vol);
    bool write_cmd(const picture_cell &cell);
    const  CellDataT<Stereo_Pose_Data>::type &_vol;
    const std::vector<Stereo_Pose_Data> &_tasks;
    std::string getCmdFileName(){return cmdfn;}
    void close(){ if(cmdfp) fclose(cmdfp); }

};

class WriteBoundVRIP : private WriteTP{
    public:
  WriteBoundVRIP(double res,std::string fname,std::string basepath,std::string cwd,const std::vector<Stereo_Pose_Data> &tasks,double expandBy,double smallCCPer,int expand_vol);
    bool write_cmd(Cell_Data<Stereo_Pose_Data> cell);
    std::string bboxfn;
    std::string getCmdFileName(){return cmdfn;}
    void close(){ if(cmdfp) fclose(cmdfp); }
    double _expandBy;
    double _smallCCPer;
    std::string getPostCmds( CellDataT<Stereo_Pose_Data>::type &vol);
    int _expand_vol;


};
class WriteBoundTP : private WriteTP{
    public:
    WriteBoundTP(double res,std::string fname,std::string basepath,std::string cwd,const std::vector<Stereo_Pose_Data> &tasks,double expandBy,double smallCCPer);
    bool write_cmd(Cell_Data<Stereo_Pose_Data> cell);
    std::string bboxfn;
    std::string getCmdFileName(){return cmdfn;}
    void close(){ if(cmdfp) fclose(cmdfp); if(cmd2fp) fclose(cmd2fp);}
    FILE*cmd2fp;
    double _expandBy;
    double _smallCCPer;



};
osg::Matrix  osgTranspose( const osg::Matrix& src );
void splitPictureCellsEven( std::vector<picture_cell> &cells,    const CellDataT<Stereo_Pose_Data>::type &vol,int _tileRows,int _tileColumns,osg::BoundingBox totalbb,int vpblod,const std::vector<Stereo_Pose_Data> &tasks,bool prog);
void splitPictureCells( std::vector<picture_cell> &cells,    const CellDataT<Stereo_Pose_Data>::type &vol,double margin,std::vector<osg::BoundingBox> &kd_bboxes,osg::BoundingBox totalbb,int vpblod,const std::vector<Stereo_Pose_Data> &tasks);
bool getFaceDivision(osg::ref_ptr<osg::Node> &model,double &avgLen,int &numberFacesAll,unsigned int targetNumTrianglesPerLeaf,   std::vector<osg::BoundingBox> &kd_bboxes);

#endif // SPLITBOUNDS_H
