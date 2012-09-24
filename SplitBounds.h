#ifndef SPLITBOUNDS_H
#define SPLITBOUNDS_H
#include "stereo_cells.hpp"
#include <map>
#include <vector>
#include <stdio.h>
#include <iostream>
#include <osg/io_utils>
template <typename ValType>
struct CellDataT
{
typedef std::vector<std::vector<std::vector<Cell_Data<ValType> > > >  type;
};

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
bool split_bounds(const Bounds &bounds,const std::vector<CellType> &poses ,double targetVolume,int minSplits,typename CellDataT<CellType>::type &Vol){

    double ran[3];
    for(int i=0; i<3; i++)
        ran[i]=(bounds.bbox._max[i]-bounds.bbox._min[i]);

    int splits[3];
    for(int i=0;i<3; i++)
        splits[i]=ceil(ran[i]/targetVolume);


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


class WriteBoundTP : private WriteTP{
    public:
    WriteBoundTP(double res,std::string fname,std::string basepath,std::string cwd,const std::vector<Stereo_Pose_Data> &tasks,double expandBy);
    bool write_cmd(Cell_Data<Stereo_Pose_Data> cell);
    std::string bboxfn;
    std::string getCmdFileName(){return cmdfn;}
    void close(){ if(cmdfp) fclose(cmdfp); if(cmd2fp) fclose(cmd2fp);}
    FILE*cmd2fp;
    double _expandBy;



};
osg::Matrix  osgTranspose( const osg::Matrix& src );

#endif // SPLITBOUNDS_H
