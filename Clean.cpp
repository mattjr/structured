#include <vcg/complex/trimesh/hole.h>
#include <vcg/math/quadric.h>
#include <vcg/complex/trimesh/clean.h>
#include<vcg/complex/trimesh/base.h>
#include<vcg/simplex/vertexplus/component_ocf.h>
#include<vcg/simplex/faceplus/component_ocf.h>
#include <vcg/complex/trimesh/update/topology.h>
#include <vcg/complex/trimesh/update/position.h>
#include<vcg/complex/trimesh/base.h>
#include <wrap/io_trimesh/import.h>
#include <wrap/io_trimesh/export_ply.h>

#include "tridecimator/remove_small_cc.h"

#include "tridecimator/meshmodel.h"

void dump_pts(FILE *pos_fp,char * filename,bool clean){
 
  
	CMeshO cm;
	int err=vcg::tri::io::ImporterPLY<CMeshO>::Open(cm,filename);
	if(clean){
	  cm.face.EnableFFAdjacency();
	  cm.face.EnableMark();
	  vcg::tri::UpdateTopology<CMeshO>::FaceFace(cm);
	  vcg::tri::UpdateFlags<CMeshO>::FaceBorderFromFF(cm);
	
	  vcg::RemoveSmallConnectedComponentsDiameter<CMeshO>(cm,10.0);
	  int dup= vcg::tri::Clean<CMeshO>::RemoveDuplicateVertex(cm);
	  int unref= vcg::tri::Clean<CMeshO>::RemoveUnreferencedVertex(cm);
	  int deg= vcg::tri::Clean<CMeshO>::RemoveDegenerateFace(cm);
	}
	  /* faceflip(mesh);
	     mesh->need_normals();
	     int nv = mesh->vertices.size();*/
	  float buf[6];
	  vcg::tri::UpdateNormals<CMeshO>::PerVertexNormalized(cm);
	  vcg::tri::Clean<CMeshO>::FlipMesh(cm);

   
      CMeshO::VertexIterator vi;
      for(vi=cm.vert.begin();vi!=cm.vert.end();++vi){


	buf[0]=(*vi).P()[0];
	buf[1]=(*vi).P()[1];
	buf[2]=(*vi).P()[2];

	buf[3]=(*vi).N()[0];
	buf[4]=(*vi).N()[1];
	buf[5]=(*vi).N()[2];
      	fwrite(buf,sizeof(float),6,pos_fp);
      } 

}
