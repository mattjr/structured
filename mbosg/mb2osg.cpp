/* -*-c++-*- OpenSceneGraph - Copyright (C) 1998-2006 Robert Osfield 
 *
 * This application is open source and may be redistributed and/or modified   
 * freely and without restriction, both in commericial and non commericial applications,
 * as long as this copyright notice is maintained.
 * 
 * This application is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/

#include <osgDB/ReadFile>
#include <osgUtil/Optimizer>
#include <osg/MatrixTransform>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include "libsnapper/auv_stereo_geometry.hpp"
#include "libsnapper/auv_image_distortion.hpp"
#include "libsnapper/auv_camera_geometry.hpp"
#include "libsnapper/auv_stereo_calib.hpp"
#include "auv_config_file.hpp"
#include "../OSGExport.h"

#include <iostream>


using namespace std;
using namespace libplankton;
using namespace libsnapper;
static int verbose=0;
int main(int argc, char** argv)
{
  string dir;
  // use an ArgumentParser object to manage the program arguments.
  osg::ArgumentParser arguments(&argc,argv);
  arguments.read("-dir",dir);

  if (arguments.argc()<1){
    arguments.getApplicationUsage()->write(std::cout,osg::ApplicationUsage::COMMAND_LINE_OPTION);
    return 1;
  }
  
 
  // any option left unread are converted into errors to write out later.
  arguments.reportRemainingOptionsAsUnrecognized();

    // report any errors if they have occurred when parsing the program arguments.
    if (arguments.errors())
    {
        arguments.writeErrorMessages(std::cout);
        return 1;
    }
    
    float zrange[2];
    char rangefname[255];
    vector<string>mb_filenames;
    ifstream mblist((dir+"/mblist.txt").c_str());
    char tmp[255];
  
    while(!mblist.eof()){
      mblist.getline(tmp,255); 
      string basename=osgDB::getNameLessExtension(string(tmp));
      string total=dir+"/"+basename;
      if( basename.size()>1)
	mb_filenames.push_back(total);
            
    }
    mblist.close();


    sprintf(rangefname,"%s/range.txt",dir.c_str());
    FILE *fp =fopen(rangefname,"r");
    if(!fp){
      fprintf(stderr,"No range.txt file Quitting\n");
      exit(-1);
    }
    fscanf(fp,"%*f %*f %f\n%*f %*f %f\n",&(zrange[0]),&(zrange[1]));
    fclose(fp);

    printf("\rMB: %d/%d LOD: %d/%d",1,(int)mb_filenames.size(),1,3);
    fflush(stdout);
    
    OSGExporter *osgExp=new OSGExporter(dir,false,true,512);    
    char out_name[512];
    const char *subdir="lod";
    const char *mdir="mesh";
    char str[512];
    for(int i=0; i <(int)mb_filenames.size(); i++){
      for(int j=0; j <3; j++){
	osg::ref_ptr<osg::Geode> group[2];
	sprintf(str,"%s-lod%d.ply",mb_filenames[i].c_str(),j);
	sprintf(out_name,"%s/%s/mb-%02d-lod%d.ive",mdir,subdir,i,j);
	GtsSurface *surf = gts_surface_new(gts_surface_class(),
					   (GtsFaceClass *) t_face_class(),
					   gts_edge_class(), t_vertex_class());
	TriMesh::verbose=verbose;
	
	TriMesh *mesh = TriMesh::read(str);
	if(!mesh){
	  fprintf(stderr,"Empty mesh skipping\n");
	  continue;
	}
	bool res=convert_ply(mesh,surf,verbose,NULL);
	if(!res ){
	  printf("Failed to load surface %s\n",
		 str);
	}
	std::map<int,std::string> texture_file_names;
	ClippingMap cm;
	vector<Plane3D> planes;
	vector<TriMesh::BBox> bounds;
	osgExp->convertGtsSurfListToGeometry(surf,texture_file_names,&cm,
                                             0,512,group,planes,bounds,NULL,
					     NULL,zrange,NULL);
	
	osgExp->outputModelOSG(out_name,group);
        printf("\rMB: %d/%d LOD: %d/%d",i+1,(int)mb_filenames.size(),j+1,3);
	fflush(stdout);
      }
    }
    printf("\n");
}
