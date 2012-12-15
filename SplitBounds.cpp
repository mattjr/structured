#include "SplitBounds.h"
#include "MemUtils.h"
#include "ShellCmd.h"
#include <osg/KdTree>
#include <osgDB/ReadFile>
#include <osgDB/FileNameUtils>
extern const char *thrpool;
extern const char *serfile;
using namespace std;
int count_vol(const CellDataT<Stereo_Pose_Data>::type &container) {
    CellDataT<Stereo_Pose_Data>::type::iterator var;
    int count=0;
    for(unsigned int ii=0; ii < container.size(); ii++)\
    for(unsigned int jj=0; jj < container[ii].size(); jj++)\
    for(typeof((container[ii][jj]).begin()) var = (container[ii][jj]).begin(); \
         var != (container[ii][jj]).end(); \
         ++var)
            count++;
    return count;
}

osg::Matrix  osgTranspose( const osg::Matrix& src )
{
    osg::Matrix dest;
    for( int i = 0; i < 4; i++ )
        for( int j = 0; j < 4; j++ )
            dest(i,j) = src(j,i);
    return dest;
}

WriteBoundVRIP::WriteBoundVRIP(double res,string fname,std::string basepath,std::string cwd,const std::vector<Stereo_Pose_Data> &tasks,double expandBy):WriteTP(res,fname,basepath,cwd),_expandBy(expandBy){
    cmdfp =fopen(fname.c_str(),"w");
    if(!cmdfp){
        fprintf(stderr,"Can't create cmd file");
        exit(-1);
    }


}
std::string WriteBoundVRIP::getPostCmds( CellDataT<Stereo_Pose_Data>::type &vol){
    vector<string> allmeshes;
    stringstream cmdpost;
    double smallCCPer=0.2;

    foreach_vol(cur,vol){
        char tmp[2048];
        if(cur->poses.size() == 0)
            continue;
        sprintf(tmp,"clean-clipped-diced-%08d_%08d_%08d.ply",
                cur->volIdx[0],cur->volIdx[1],cur->volIdx[2]);
        allmeshes.push_back(tmp);//string(" ")+tmp+string(" "));
    }
    char tmp[1024];
    sprintf(tmp, "%s/tmp-total.txt",diced_dir);
    FILE *fp=fopen(tmp,"w");
    fprintf(fp,"tmp-total.ply %f 1\n",_res);
    fclose(fp);

    cmdpost <<"cd "<<_cwd<<"/"<<diced_dir<< ";"<< _basepath << "/vrip/bin/plymerge ";
    stringstream output;
    output << " > tmp-total.ply;";//" > unclean-total.ply;";
   // sprintf(tmp,"%s/vcgapps/bin/mergeMesh unclean-total.ply  -cleansize %f -P -out tmp-total.ply;",_basepath.c_str(),smallCCPer);
   // output <<tmp;
    sprintf(tmp,"export BASEDIR=\"%s\"; export OUTDIR=\"%s/\";export VRIP_HOME=\"$BASEDIR/vrip\";export VRIP_DIR=\"$VRIP_HOME/src/vrip/\";export PATH=$PATH:$VRIP_HOME/bin;cd %s/$OUTDIR;",_basepath.c_str(),diced_dir,_cwd.c_str());
    output << tmp;
    sprintf(tmp,"$BASEDIR/vrip/bin/vripnew tmp-total.vri tmp-total.txt tmp-total.txt %f -rampscale %f > log-tmp-total.txt;$BASEDIR/vrip/bin/volfill tmp-total.vri exp-total.vri >log-volfill.txt;$BASEDIR/vrip/bin/vripsurf exp-total.vri exp-total.ply >log-surf.txt;",_res,_expandBy);
    output << tmp;
    sprintf(tmp,"%s/vcgapps/bin/mergeMesh exp-total.ply  -cleansize %f -P -out total.ply > log-merge.txt",_basepath.c_str(),smallCCPer);
    output << tmp;
    string cmdfile=string(aggdir)+"/finsplitcmd";
    FILE *fp2=fopen(cmdfile.c_str(),"w");
    foreach_vol(cur,vol){
    fprintf(fp2,"%s/treeBBClip %s/%s/total.ply --bbox %f %f %f %f %f %f -dup --outfile %s/%s/clean_%04d%04d%04d.ply\n",
            _basepath.c_str(),
            _cwd.c_str(),
            diced_dir,
            cur->bounds.bbox.xMin(),
            cur->bounds.bbox.yMin(),
            cur->bounds.bbox.zMin(),
            cur->bounds.bbox.xMax(),
            cur->bounds.bbox.yMax(),
            cur->bounds.bbox.zMax(),
            _cwd.c_str(),
            aggdir,
            cur->volIdx[0],cur->volIdx[1],cur->volIdx[2]);
    }
    fclose(fp2);
    sprintf(tmp,"\nos.system(setupts.basepath +'/%s %s %s %s')\n",
            thrpool,cmdfile.c_str(),serfile,"Presplit");
    return createFileCheckPython(cmdpost.str(),_cwd+"/"+diced_dir,allmeshes,output.str(),4)+tmp;

}
bool WriteBoundVRIP::write_cmd(Cell_Data<Stereo_Pose_Data> cell){
 double smallCCPer=0.2;
 //int runs=1;
 char vrip_seg_fname[2048];
 char conf_name[2048];
  sprintf(vrip_seg_fname,"%s/vripseg-%08d_%08d_%08d.txt",aggdir, cell.volIdx[0],cell.volIdx[1],cell.volIdx[2]);
  sprintf(conf_name,"%s/bbox-clipped-diced-%08d_%08d_%08d.ply.txt",diced_dir,cell.volIdx[0],cell.volIdx[1],cell.volIdx[2]);

  FILE *vrip_seg_fp=fopen(vrip_seg_fname,"w");
 FILE * bboxfp = fopen(conf_name,"w");
  if(!vrip_seg_fp || !bboxfp){
      printf("Unable to open %s\n",vrip_seg_fname);
  }

 fprintf(cmdfp,"export BASEDIR=\"%s\"; export OUTDIR=\"%s/\";export VRIP_HOME=\"$BASEDIR/vrip\";export VRIP_DIR=\"$VRIP_HOME/src/vrip/\";export PATH=$PATH:$VRIP_HOME/bin;cd %s/$OUTDIR;",_basepath.c_str(),aggdir,_cwd.c_str());
 fprintf(cmdfp,"$BASEDIR/vrip/bin/vripnew auto-%08d_%08d_%08d.vri %s %s %f -rampscale %f;$BASEDIR/vrip/bin/vripsurf auto-%08d_%08d_%08d.vri seg-%08d_%08d_%08d.ply ;",cell.volIdx[0],cell.volIdx[1],cell.volIdx[2],osgDB::getSimpleFileName(vrip_seg_fname).c_str(),osgDB::getSimpleFileName(vrip_seg_fname).c_str(),_res,_expandBy,cell.volIdx[0],cell.volIdx[1],cell.volIdx[2],cell.volIdx[0],cell.volIdx[1],cell.volIdx[2]);

 fprintf(cmdfp,"$BASEDIR/treeBBClip seg-%08d_%08d_%08d.ply --bbox %f %f %f %f %f %f -dup --outfile %s/%s/tmp-clipped-diced-%08d_%08d_%08d.ply;",
         cell.volIdx[0],cell.volIdx[1],cell.volIdx[2],
         cell.bounds.bbox.xMin(),
         cell.bounds.bbox.yMin(),
         cell.bounds.bbox.zMin(),
         cell.bounds.bbox.xMax(),
        cell.bounds.bbox.yMax(),
         cell.bounds.bbox.zMax(),
         _cwd.c_str(),
         diced_dir,
         cell.volIdx[0],cell.volIdx[1],cell.volIdx[2]);
 fprintf(cmdfp,"%s/vcgapps/bin/mergeMesh %s/%s/tmp-clipped-diced-%08d_%08d_%08d.ply  -cleansize %f -P -flip -out %s/%s/clean-clipped-diced-%08d_%08d_%08d.ply;",
         _basepath.c_str(),
         _cwd.c_str(),
         diced_dir,
         cell.volIdx[0],cell.volIdx[1],cell.volIdx[2],
         smallCCPer,
         _cwd.c_str(),
         diced_dir,
         cell.volIdx[0],cell.volIdx[1],cell.volIdx[2]);
 fprintf(cmdfp,"cd ..\n");
 for(unsigned int j=0; j <cell.poses.size(); j++){
     const Stereo_Pose_Data *pose=cell.poses[j];
     if(!pose->valid)
         continue;
     //Vrip List
     fprintf(vrip_seg_fp,"%s %f 1\n",pose->mesh_name.c_str(),_res);
     //Gen Tex File bbox
     fprintf(bboxfp, "%d %s " ,pose->id,pose->left_name.c_str());
     save_bbox_frame(pose->bbox,bboxfp);
     osg::Matrix texmat=osgTranspose(pose->mat);
     texmat=osg::Matrix::inverse(texmat);
     for(int k=0; k < 4; k++)
         for(int n=0; n < 4; n++)
             fprintf(bboxfp," %lf",texmat(k,n));
     fprintf(bboxfp,"\n");
 }


 fclose(vrip_seg_fp);
 fclose(bboxfp);



    return true;
}

WriteBoundTP::WriteBoundTP(double res,string fname,std::string basepath,std::string cwd,const std::vector<Stereo_Pose_Data> &tasks,double expandBy):WriteTP(res,fname,basepath,cwd),_expandBy(expandBy){
    cmdfp =fopen(fname.c_str(),"w");
    if(!cmdfp){
        fprintf(stderr,"Can't create cmd file");
        exit(-1);
    }

    cmd2fp =fopen((fname+"2").c_str(),"w");
    if(!cmd2fp){
        fprintf(stderr,"Can't create cmd file");
        exit(-1);
    }
    char tmp[1024];
    sprintf(tmp,"%s/vcglist.txt",aggdir);
    bboxfn=string(tmp);
    FILE *fp=fopen(bboxfn.c_str(),"w");
    for(int i=0; i< (int)tasks.size(); i++)
        fprintf(fp,"%s/%s %f %f %f %f %f %f\n",aggdir,tasks[i].mesh_name.c_str(),
                tasks[i].bbox._min[0],
                tasks[i].bbox._min[1],
                tasks[i].bbox._min[2],
                tasks[i].bbox._max[0],
                tasks[i].bbox._max[1],
                tasks[i].bbox._max[2]
                );
    fclose(fp);

}

bool WriteBoundTP::write_cmd(Cell_Data<Stereo_Pose_Data> cell){
 double smallCCPer=0.2;
 FILE *fps[]={cmdfp,cmd2fp};
 int runs=1;
 const char *app=(runs ==1)? "plymc":"plymc_outofcore";
 for(int i=0; i<runs; i++){
     // fprintf(fps[i],"cd %s;%s/vcgapps/bin/%s -M -V%f -i%d -s %d %d %d %d %d %d -o%s/vol %s",
     char expand_str[1024];
     if(_expandBy > 0.0)
         sprintf(expand_str,"-W%f",_expandBy);
     else
         sprintf(expand_str," ");

 fprintf(fps[i],"cd %s;%s/vcgapps/bin/%s %s -V%f  -s %d %d %d %d %d %d -o%s/vol %s",
       _cwd.c_str(),  _basepath.c_str(),app,expand_str,_res,
                    cell.splits[0],
                    cell.splits[1],
                    cell.splits[2],
                    cell.volIdx[0],
                    cell.volIdx[1],
                    cell.volIdx[2],
                    aggdir,
                    bboxfn.c_str()
        );
 }
    // fprintf(cmdfp,"\n");
     fprintf(cmdfp,";%s/vcgapps/bin/mergeMesh %s/vol_%04d%04d%04d.ply  -cleansize %f -P -out %s/clean_%04d%04d%04d.ply\n",

     /*_res,
                cell.splits[0],
                cell.splits[1],
                cell.splits[2],
                cell.volIdx[0],
                cell.volIdx[1],
                cell.volIdx[2],
                aggdir,
                bboxfn.c_str(),*/
                _basepath.c_str(),
                aggdir,
                cell.volIdx[0],
                cell.volIdx[1],
                cell.volIdx[2],
                smallCCPer,
                aggdir,
                cell.volIdx[0],
                cell.volIdx[1],
                cell.volIdx[2]
                );



    return true;
}


WriteSplitTP::WriteSplitTP(double res,string fname,std::string basepath,std::string cwd,const std::vector<Stereo_Pose_Data> &tasks,const  CellDataT<Stereo_Pose_Data>::type &Vol):WriteTP(res,fname,basepath,cwd),_vol(Vol),_tasks(tasks){
    cmdfp =fopen(fname.c_str(),"w");
    if(!cmdfp){
        fprintf(stderr,"Can't create cmd file");
        exit(-1);
    }

}


bool WriteSplitTP::write_cmd(const picture_cell &cell){
    char shr_tmp[8192];
    string cmdtoRun;
    sprintf(shr_tmp,"cd %s;%s/borderClip ",_cwd.c_str(),
            _basepath.c_str());
    cmdtoRun+=shr_tmp;
    int v_count=0;
    osg::BoundingBox cellMargin;
    double margin=cell.bbox.radius()*2;

    cellMargin.expandBy(cell.bbox.xMin()-(margin),
                               cell.bbox.yMin()-(margin),
                               -FLT_MAX);
    cellMargin.expandBy(cell.bbox.xMax()+(margin),cell.bbox.yMax()+(margin),FLT_MAX);
    string fileList;
    char tmpstrdata[8192];
    foreach_vol(cur,_vol){
        double margin=cur->bounds.bbox.radius()*2;

        osg::BoundingBox curMargin;
        curMargin.expandBy(cur->bounds.bbox.xMin()-(margin),
                                   cur->bounds.bbox.yMin()-(margin),
                                   -FLT_MAX);
        curMargin.expandBy(cur->bounds.bbox.xMax()+(margin),cur->bounds.bbox.yMax()+(margin),FLT_MAX);

        if(!cellMargin.intersects(curMargin))
            continue;
        sprintf(tmpstrdata," %s/clean_%04d%04d%04d.ply",aggdir,cur->volIdx[0],cur->volIdx[1],cur->volIdx[2]);
        fileList+=tmpstrdata;
        v_count++;
    }
    if(v_count== 0)
        return false;
    cmdtoRun+=fileList;

    osg::BoundingBox faceMarginBox;
    double faceMargin=_res*5;
    faceMarginBox.expandBy(cell.bbox.xMin()-faceMargin,cell.bbox.yMin()-faceMargin,cell.bbox.zMin());
    faceMarginBox.expandBy(cell.bbox.xMax()+faceMargin,cell.bbox.yMax()+faceMargin,cell.bbox.zMax());

    sprintf(tmpstrdata," --bbox %.16f %.16f %.16f %.16f %.16f %.16f --bbox-margin %.16f %.16f %.16f %.16f %.16f %.16f -dump -F --outfile %s/un-tmp-tex-clipped-diced-r_%04d_c_%04d.ply;",
            cell.bbox.xMin(),
            cell.bbox.yMin(),
            cell.bbox.zMin(),
            cell.bbox.xMax(),
            cell.bbox.yMax(),
            cell.bbox.zMax(),
            faceMarginBox.xMin(),
            faceMarginBox.yMin(),
            faceMarginBox.zMin(),
            faceMarginBox.xMax(),
            faceMarginBox.yMax(),
            faceMarginBox.zMax(),
            diced_dir,
            cell.row,cell.col);
    cmdtoRun+=tmpstrdata;

    fprintf(cmdfp,"%s %s/vcgapps/bin/sw-shadevis -P -n64 %s/un-tmp-tex-clipped-diced-r_%04d_c_%04d.ply ;",
            cmdtoRun.c_str(),_basepath.c_str(),diced_dir,
             cell.row,cell.col);
    fprintf(cmdfp,"%s/treeBBClip --bbox %.16f %.16f %.16f %.16f %.16f %.16f %s/vis-un-tmp-tex-clipped-diced-r_%04d_c_%04d.ply -dup -qual -F --outfile %s/vis-tmp-tex-clipped-diced-r_%04d_c_%04d.ply \n",
            _basepath.c_str(),
           -FLT_MAX,
            -FLT_MAX,
            -FLT_MAX,
            FLT_MAX,
            FLT_MAX,
            FLT_MAX,
            diced_dir,
            cell.row,cell.col,
             diced_dir,cell.row,cell.col);
    char tp[1024];
    sprintf(tp,"%s/bbox-vis-tmp-tex-clipped-diced-r_%04d_c_%04d.ply.txt",diced_dir,cell.row,cell.col);
    FILE *bboxfp=fopen(tp,"w");

    for(int k=0; k < (int)cell.imagesMargin.size(); k++){
        const Stereo_Pose_Data *pose=(&_tasks[cell.imagesMargin[k]]);
        if(pose && pose->valid){
            fprintf(bboxfp, "%d %s " ,pose->id,pose->left_name.c_str());
            save_bbox_frame(pose->bbox,bboxfp);
            osg::Matrix texmat=osgTranspose(pose->mat);
            texmat=osg::Matrix::inverse(texmat);
            for(int f=0; f < 4; f++)
                for(int n=0; n < 4; n++)
                    fprintf(bboxfp," %lf",texmat(f,n));
            fprintf(bboxfp,"\n");
        }

    }
    fclose(bboxfp);

    return true;

}
bool getFaceDivision(osg::ref_ptr<osg::Node> &model,double &avgLen,int &numberFacesAll,unsigned int targetNumTrianglesPerLeaf,    std::vector<osg::BoundingBox> &kd_bboxes){
    //printf("Getting face divisions\n");
    if(model.valid()){
        osg::Geode *geode= dynamic_cast<osg::Geode*>(model.get());
        if(!geode)
            geode=model->asGroup()->getChild(0)->asGeode();
        if(geode && geode->getNumDrawables()){
            osg::ref_ptr<osg::KdTreeBuilder> kdTreeBuilder = osgDB::Registry::instance()->getKdTreeBuilder()->clone();
            kdTreeBuilder->_buildOptions._targetNumTrianglesPerLeaf=targetNumTrianglesPerLeaf;
            geode->accept(*kdTreeBuilder);
            osg::Drawable *drawable = geode->getDrawable(0);
            osg::KdTree *kdTree = dynamic_cast<osg::KdTree*>(drawable->getShape());
            osg::Geometry *geom = dynamic_cast< osg::Geometry*>(drawable);

            osg::Vec3Array *verts=static_cast< osg::Vec3Array*>(geom->getVertexArray());
            //osg::Vec4Array *colors=static_cast< osg::Vec4Array*>(geom->getColorArray());
            //osg::Vec4Array *texCoordsStored=static_cast< osg::Vec4Array*>(geom->getTexCoordArray(0));

            osg::DrawElementsUInt* primitiveSet = dynamic_cast<osg::DrawElementsUInt*>(geom->getPrimitiveSet(0));
            avgLen=0.0;
            for(int i=0; i< (int)primitiveSet->getNumIndices()-2; i+=3){
                unsigned int i0=primitiveSet->at(i);
                unsigned int i1=primitiveSet->at(i+1);
                unsigned int i2=primitiveSet->at(i+2);
                const osg::Vec3 &v0=verts->at(i0);
                const osg::Vec3 &v1=verts->at(i1);
                const osg::Vec3 &v2=verts->at(i2);
                double d01 = (v0-v1).length2();
                double d12 = (v1 -v2).length2();
                double d20 = (v2-v0).length2();
                avgLen+=d01/primitiveSet->getNumIndices();
                avgLen+=d12/primitiveSet->getNumIndices();
                avgLen+=d20/primitiveSet->getNumIndices();

            }
           // printf("Average Len %f\n",avgLen);
            if(!kdTree){
                fprintf(stderr,"Can't be converted to kdtree\n");
                exit(-1);
            }else{
                osg::KdTree::KdNodeList list;
                list=kdTree->getNodes();
                //printf("Kdtree cells %d\n",(int)list.size());
                for(int i=0; i< (int)list.size(); i++){
                    if(list[i].first < 0)
                        kd_bboxes.push_back(list[i].bb);
                }
                 printf("Kdtree cells %d\n",(int)kd_bboxes.size());

                //cout << list[i].bb <<endl;
            }

            numberFacesAll=geom->getPrimitiveSet(0)->getNumPrimitives();
           // printf("Face %d\n",numberFacesAll);
            return true;

        }
    }
return false;
}
void splitPictureCells( std::vector<picture_cell> &cells,    const CellDataT<Stereo_Pose_Data>::type &vol,double margin,std::vector<osg::BoundingBox> &kd_bboxes,osg::BoundingBox totalbb,int vpblod,const std::vector<Stereo_Pose_Data> &tasks){
    osg::BoundingSphere bs;
    bs.expandBy(totalbb);
    char tmp4[1024];
    cout << "Assign splits...\n";
    int validF=0;
    //#pragma omp parallel num_threads(num_threads)
    {
        //#pragma omp parallel for shared(_tileRows, _tileColumns)
        for(unsigned int i=0; i<kd_bboxes.size(); i++){
            fflush(stdout);
            //osg::Matrix offsetMatrix=   osg::Matrix::scale(_tileColumns, _tileRows, 1.0) *osg::Matrix::translate(_tileColumns-1-2*col, _tileRows-1-2*row, 0.0);
            double left,right,bottom,top;//,znear,zfar;
            osg::Matrix m;//=(view*proj*offsetMatrix);
            //m.getOrtho(left,right,bottom,top,znear,zfar);
            double chunkSizeX=(kd_bboxes[i].xMax()-kd_bboxes[i].xMin());
            double chunkSizeY=(kd_bboxes[i].yMax()-kd_bboxes[i].yMin());

            double widthEnd=kd_bboxes[i].xMax();
            double widthStart=kd_bboxes[i].xMin();
            double heightEnd=kd_bboxes[i].yMax();
            double heightStart=kd_bboxes[i].yMin();
            m(0,0)=0;
            m(0,1)=2/chunkSizeX;
            m(0,2)=0;
            m(0,3)= (-(heightEnd+heightStart)/(heightEnd-heightStart));


            m(1,0)=2/chunkSizeY;
            m(1,1)=0;
            m(1,2)=0;
            m(1,3)= (-(widthEnd+widthStart)/(widthEnd-widthStart));


            m(2,0)=0;
            m(2,1)=0;
            m(2,2)=2;
            m(2,3)=-1;

            m(3,0)=0;
            m(3,1)=0;
            m(3,2)=0;
            m(3,3)= 1;
            // cout <<m<<endl;
            left=widthStart;
            right=widthEnd;
            top=heightEnd;
            bottom=heightStart;
            osg::BoundingBox thisCellBbox=kd_bboxes[i];
              osg::BoundingBox thisCellBbox2;
            thisCellBbox2.expandBy(osg::Vec3(left,bottom,bs.center()[2]-bs.radius()));
            thisCellBbox2.expandBy(osg::Vec3(right,top,bs.center()[2]+bs.radius()));
        //    cout << thisCellBbox << " "<< thisCellBbox2<<endl;

            osg::BoundingBox thisCellBboxMargin;
            thisCellBboxMargin.expandBy(thisCellBbox.xMin()-(margin),thisCellBbox.yMin()-margin,thisCellBbox.zMin()-margin);
            thisCellBboxMargin.expandBy(thisCellBbox.xMax()+(margin),thisCellBbox.yMax()+margin,thisCellBbox.zMax()+margin);

           // cout << thisCellBbox << " "<< thisCellBboxMargin <<endl;
            //  printf("ANNNN %f %f %f %f %f %f\n",left-(margin),bottom-(margin),bs.center()[2]-bs.radius(),right+(margin),top+(margin),bs.center()[2]+bs.radius());
            osg::BoundingBox bboxMarginUnRot;
            bboxMarginUnRot.expandBy(thisCellBboxMargin._min);
            bboxMarginUnRot.expandBy(thisCellBboxMargin._max);
            //  std::cout<< thisCellBbox._max-thisCellBbox._min <<"\n";
            //  std::cout<< "A"<<thisCellBboxMargin._min << " "<< thisCellBboxMargin._max<<"\n\n";
            bool hitcell=false;

            foreach_vol(cur,vol){
                if(cur->poses.size() == 0){
                    continue;
                }

                if(bboxMarginUnRot.intersects(cur->bounds.bbox)){
                    hitcell=true;
                    break;
                }
            }
            /*
            for(int k=0; k< (int)vrip_cells.size(); k++){
              //  cout << "FFF "<<vrip_cells[k].bounds.bbox._min << " "<<vrip_cells[k].bounds.bbox._max<<endl;
              //  cout << "GLOBAL "<<totalbb._min << " "<<totalbb._max<<endl;

                if(bboxMarginUnRot.intersects(vrip_cells[k].bounds.bbox)){
                    hitcell=true;
                    break;
                }
            }*/
            if(!hitcell)
                continue;
            //osg::Timer_t st= osg::Timer::instance()->tick();

            picture_cell cell;
            cell.bbox=thisCellBbox;
            cell.bboxMargin=thisCellBboxMargin;
            cell.bboxUnRot.expandBy(cell.bbox._min);
            cell.bboxUnRot.expandBy(cell.bbox._max);

            cell.bboxMarginUnRot=bboxMarginUnRot;
            int row=i;
            int col=0;
            cell.row=row;
            cell.col=col;
            cell.m=m;
            sprintf(tmp4,"%s/tex-clipped-diced-r_%04d_c_%04d-lod%d.ive",diced_dir,row,col,vpblod);
            cell.name=string(tmp4);
            sprintf(tmp4,"%s/tex-clipped-diced-r_%04d_c_%04d.mat",diced_dir,row,col);
            validF++;
           /* std::fstream _file(tmp4,std::ios::binary|std::ios::out);
            for(int k=0; k<4; k++)
                for(int l=0; l<4; l++){
                    //Transpose matrix
                    _file.write(reinterpret_cast<char*>(&(cell.m(l,k))),sizeof(double));
                }
            _file.close();*/
            // double timeForReadPixels = osg::Timer::instance()->delta_s(st, osg::Timer::instance()->tick());
            //printf("Time %f\n",timeForReadPixels);
            for(int l=0; l < (int)tasks.size(); l++){
                if(!tasks[l].valid || !tasks[l].bbox.valid())
                    continue;

                osg::Vec3 m1=osg::Vec3(tasks[l].bbox_margin.xMin(),tasks[l].bbox_margin.yMin(),tasks[l].bbox_margin.zMin());
                osg::Vec3 m2=osg::Vec3(tasks[l].bbox_margin.xMax(),tasks[l].bbox_margin.yMax(),tasks[l].bbox_margin.zMax());

                osg::BoundingBox imgBox;
                imgBox.expandBy(m1);
                imgBox.expandBy(m2);
                //  cout << m1 << " "<< m2 << " bounds \n";
                // cout <<thisCellBbox._min << " "<< thisCellBbox._max<<" bbox\n";
                if(thisCellBbox.intersects(imgBox)){
                    //printf("ast\n");
                    cell.images.push_back(l);
                }
                if(thisCellBboxMargin.intersects(imgBox)){
                    //printf("!!!ast\n");

                    cell.imagesMargin.push_back(l);
                }
            }
            //#pragma omp critical
            {
                cells.push_back(cell);
            }
           // cout << "i : " <<i<< " " <<cell.images.size()<<" "<<cell.imagesMargin.size()<<endl;

        }
    }


    printf("\nValid %d\n",validF);
    if(validF ==0){
        fprintf(stderr,"No valid cells\n");
        exit(-1);
    }


}
void splitPictureCellsEven( std::vector<picture_cell> &cells,const CellDataT<Stereo_Pose_Data>::type &vol,
                            int _tileRows,int _tileColumns,osg::BoundingBox totalbb,int vpblod,const std::vector<Stereo_Pose_Data> &tasks,bool prog){


    osg::BoundingSphere bs;
    bs.expandBy(totalbb);
    char tmp4[1024];
    cout << "Assign splits...\n";
    int validF=0;
    //#pragma omp parallel num_threads(num_threads)
    {
        //#pragma omp parallel for shared(_tileRows, _tileColumns)
        for(int row=0; row< _tileRows; row++){
            for(int col=0; col<_tileColumns; col++){
                if(prog){
                    printf("\r%04d/%04d %04d/%04d",row,_tileRows,col,_tileColumns);
                    fflush(stdout);
                }
                osg::Matrix offsetMatrix=   osg::Matrix::scale(_tileColumns, _tileRows, 1.0) *osg::Matrix::translate(_tileColumns-1-2*col, _tileRows-1-2*row, 0.0);
                double left,right,bottom,top;//,znear,zfar;
                osg::Matrix m;//=(view*proj*offsetMatrix);
                //m.getOrtho(left,right,bottom,top,znear,zfar);
                double chunkSize=(totalbb.xMax()-totalbb.xMin())/(double)_tileRows;
                double widthEnd=totalbb.xMin()+((row+1)*chunkSize);
                double widthStart=totalbb.xMin()+((row)*chunkSize);
                double heightEnd=totalbb.yMin()+((col+1)*chunkSize);
                double heightStart=totalbb.yMin()+((col)*chunkSize);
                m(0,0)=0;
                m(0,1)=2/chunkSize;
                m(0,2)=0;
                m(0,3)= (-(heightEnd+heightStart)/(heightEnd-heightStart));


                m(1,0)=2/chunkSize;
                m(1,1)=0;
                m(1,2)=0;
                m(1,3)= (-(widthEnd+widthStart)/(widthEnd-widthStart));


                m(2,0)=0;
                m(2,1)=0;
                m(2,2)=2;
                m(2,3)=-1;

                m(3,0)=0;
                m(3,1)=0;
                m(3,2)=0;
                m(3,3)= 1;
                // cout <<m<<endl;
                left=widthStart;
                right=widthEnd;
                top=heightEnd;
                bottom=heightStart;
                osg::BoundingBox thisCellBbox;
                thisCellBbox.expandBy(osg::Vec3(left,bottom,bs.center()[2]-bs.radius()));
                thisCellBbox.expandBy(osg::Vec3(right,top,bs.center()[2]+bs.radius()));
                double margin= (thisCellBbox.xMax()-thisCellBbox.xMin())*0.2;
             //   cout << left << " "<<bottom <<  " "<<right<< " "<<top<<endl;
                osg::BoundingBox thisCellBboxMargin(left-(margin),bottom-(margin),bs.center()[2]-bs.radius()-margin,right+(margin),top+(margin),bs.center()[2]+bs.radius()+margin);

                //  printf("ANNNN %f %f %f %f %f %f\n",left-(margin),bottom-(margin),bs.center()[2]-bs.radius(),right+(margin),top+(margin),bs.center()[2]+bs.radius());
                osg::BoundingBox bboxMarginUnRot;
                bboxMarginUnRot.expandBy(thisCellBboxMargin._min);
                bboxMarginUnRot.expandBy(thisCellBboxMargin._max);
                //  std::cout<< thisCellBbox._max-thisCellBbox._min <<"\n";
                //  std::cout<< "A"<<thisCellBboxMargin._min << " "<< thisCellBboxMargin._max<<"\n\n";
                bool hitcell=false;

                foreach_vol(cur,vol){
                    if(cur->poses.size() == 0){
                        continue;
                    }

                    if(bboxMarginUnRot.intersects(cur->bounds.bbox)){
                        hitcell=true;
                        break;
                    }
                }
                /*
                for(int k=0; k< (int)vrip_cells.size(); k++){
                  //  cout << "FFF "<<vrip_cells[k].bounds.bbox._min << " "<<vrip_cells[k].bounds.bbox._max<<endl;
                  //  cout << "GLOBAL "<<totalbb._min << " "<<totalbb._max<<endl;

                    if(bboxMarginUnRot.intersects(vrip_cells[k].bounds.bbox)){
                        hitcell=true;
                        break;
                    }
                }*/
                if(!hitcell)
                    continue;
                //osg::Timer_t st= osg::Timer::instance()->tick();

                picture_cell cell;
                cell.bbox=thisCellBbox;
                cell.bboxMargin=thisCellBboxMargin;
                cell.bboxUnRot.expandBy(cell.bbox._min);
                cell.bboxUnRot.expandBy(cell.bbox._max);

                cell.bboxMarginUnRot=bboxMarginUnRot;

                cell.row=row;
                cell.col=col;
                cell.m=m;
                sprintf(tmp4,"%s/tex-clipped-diced-r_%04d_c_%04d-lod%d.ive",diced_dir,row,col,vpblod);
                cell.name=string(tmp4);
                sprintf(tmp4,"%s/tex-clipped-diced-r_%04d_c_%04d.mat",diced_dir,row,col);
                validF++;
                std::fstream _file(tmp4,std::ios::binary|std::ios::out);
                for(int k=0; k<4; k++)
                    for(int l=0; l<4; l++){
                        //Transpose matrix
                        _file.write(reinterpret_cast<char*>(&(cell.m(l,k))),sizeof(double));
                    }
                _file.close();
                // double timeForReadPixels = osg::Timer::instance()->delta_s(st, osg::Timer::instance()->tick());
                //printf("Time %f\n",timeForReadPixels);
                for(int i=0; i < (int)tasks.size(); i++){
                    if(!tasks[i].valid || !tasks[i].bbox.valid())
                        continue;

                    osg::Vec3 m1=osg::Vec3(tasks[i].bbox_margin.xMin(),tasks[i].bbox_margin.yMin(),tasks[i].bbox_margin.zMin());
                    osg::Vec3 m2=osg::Vec3(tasks[i].bbox_margin.xMax(),tasks[i].bbox_margin.yMax(),tasks[i].bbox_margin.zMax());

                    osg::BoundingBox imgBox;
                    imgBox.expandBy(m1);
                    imgBox.expandBy(m2);
                    //cout << "SAD "<<tasks[i].left_name.c_str() <<  "\n";
                    //  cout << m1 << " "<< m2 << " bounds \n";
                    // cout <<thisCellBbox._min << " "<< thisCellBbox._max<<" bbox\n";
                    if(thisCellBbox.intersects(imgBox)){
                        //printf("ast\n");
                        cell.images.push_back(i);
                    }
                    if(thisCellBboxMargin.intersects(imgBox)){
                        //printf("!!!ast\n");

                        cell.imagesMargin.push_back(i);
                    }
                }
                //#pragma omp critical
                {
                    cells.push_back(cell);
                }
                char tp[1024];
                sprintf(tp,"%s/even-bbox-vis-tmp-tex-clipped-diced-r_%04d_c_%04d_rs%04d_cs%04d.ply.txt",diced_dir,cell.row,cell.col,_tileRows,_tileColumns);

                FILE *bboxfp=fopen(tp,"w");

                for(int k=0; k < (int)cell.imagesMargin.size(); k++){
                    const Stereo_Pose_Data *pose=(&tasks[cell.imagesMargin[k]]);
                    if(pose && pose->valid){
                        fprintf(bboxfp, "%d %s " ,pose->id,pose->file_name.c_str());
                        save_bbox_frame(pose->bbox,bboxfp);
                        osg::Matrix texmat=osgTranspose(pose->mat);
                        texmat=osg::Matrix::inverse(texmat);
                        for(int f=0; f < 4; f++)
                            for(int n=0; n < 4; n++)
                                fprintf(bboxfp," %lf",texmat(f,n));
                        fprintf(bboxfp,"\n");
                    }

                }
                fclose(bboxfp);

            }
        }

    }
    printf("\nValid %d\n",validF);
    if(validF ==0){
        fprintf(stderr,"No valid cells\n");
        exit(-1);
    }


}
#if 0
vector<string> genTexCmds(vector<picture_cell> cells,string cmdname,bool reparamTex,int typecmd,bool useVirtTex){
    {
        FILE *texcmds_fp =fopen(cmdname.c_str(),"w");
        for(int i=0; i <(int)cells.size(); i++){
            char tmpfn[1024];
            sprintf(tmpfn,"%s/vis-tmp-tex-clipped-diced-r_%04d_c_%04d.ply", diced_dir,cells[i].row,cells[i].col);

            if( cells[i].images.size() == 0 ||  !osgDB::fileExists(tmpfn) || checkIsEmptyPly(tmpfn)){

                continue;



            }

            string remap_mesh_ext=reparamTex ? "remap-" : "flat-";

            sprintf(tmpfn2,"%s/%stex-clipped-diced-r_%04d_c_%04d-lod%d.ply", diced_dir,remap_mesh_ext.c_str(),cells[i].row,cells[i].col,vpblod);
            cfiles.push_back(tmpfn2);




            fprintf(texcmds_fp,"cd %s",
                    cwd);

            string teximgcmd;
            ostringstream sizestr;
            char tmpfn[1024];
            switch(typecmd){
            case REMAP:
                teximgcmd = "vcgapps/bin/reparam";
                sizestr << "--srcsize " <<calib.camera_calibs[0].width << " "<<calib.camera_calibs[0].height << " ";
                sizestr<<"--scale "<<scaleRemapTex;
                break;
            case DEPTH:
                teximgcmd="depthmap";
                sizestr<<"--size "<<ajustedGLImageSizeX<<" "<<ajustedGLImageSizeY;

                break;
            case FLAT:
                teximgcmd="nonmem";
                sizestr<<"--size "<<ajustedGLImageSizeX<<" "<<ajustedGLImageSizeY;

                break;
            case REMAP_FLAT_SIZE:
                teximgcmd = "vcgapps/bin/renderReorderAA";
                sizestr << "--srcsize " <<calib.camera_calibs[0].width << " "<<calib.camera_calibs[0].height << " ";

                //                    sizestr<<"--size "<<ajustedGLImageSizeX<<" "<<ajustedGLImageSizeY;
                sprintf(tmpfn," --remap %s/param-tex-clipped-diced-r_%04d_c_%04d-lod%d.ply ",
                        diced_dir,
                        cells[i].row,cells[i].col,
                        vpblod);
                sizestr << " -pyr ";
                sizestr << tmpfn;
                break;
            }

            if(useVirtTex)
                sizestr<< " --vt " <<VTtileSize<< " " <<tileBorder<< " ";

            fprintf(texcmds_fp,";%s/%s  %s/tex-clipped-diced-r_%04d_c_%04d-lod%d.ply %s/bbox-vis-tmp-tex-clipped-diced-r_%04d_c_%04d.ply.txt %s  --invrot %f %f %f  --image %d %d %d %d -lat %.28f -lon %.28f --jpeg-quality %d --mosaicid %d %s",
                    basepath.c_str(),
                    teximgcmd.c_str(),
                    diced_dir,
                    cells[i].row,cells[i].col,
                    vpblod,
                    diced_dir,
                    cells[i].row,cells[i].col,
                    (base_dir+imgbase).c_str(),
                    rx,ry,rz,
                    cells[i].row,cells[i].col,_tileRows,_tileColumns,
                    latOrigin , longOrigin,jpegQuality,i,sizestr.str().c_str());
            if(blending)
                fprintf(texcmds_fp," --blend");


            fprintf(texcmds_fp,"\n");



        }




    }

    fclose(texcmds_fp);

}
#endif
