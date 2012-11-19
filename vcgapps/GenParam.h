#ifndef GENPARAM_H
#define GENPARAM_H
#include <osg/Vec3>
#include <string>
#include <vips/vips.h>
#include <vips/vips>

#include <osg/Vec2>
#include <osg/Array>
#include <osg/Drawable>
#include <OGF/cells/types/cells_library.h>
#include <OGF/cells/map/map.h>
#include <OGF/cells/map/map_attributes.h>
#include <OGF/cells/map/map_builder.h>

//osg::Vec3Array* CGALreparam(osg::ref_ptr<osg::Vec3Array> verts,osg::ref_ptr<osg::DrawElementsUInt> triangles);
osg::Vec3Array* OGFreparam(osg::ref_ptr<osg::Vec3Array> verts,osg::ref_ptr<osg::DrawElementsUInt> triangles);
int calcOptimalImageSize(const osg::Vec2 imageSize,osg::Vec3Array *verts,osg::DrawElementsUInt* triangles,std::vector<osg::Vec3Array *>   &texCoord,double scaletex=1.0,int VTtileSize=-1,int border=-1);
int calcOptimalImageSizeRaw(const osg::Vec2 imageSize,osg::Vec3Array *verts,osg::DrawElementsUInt* triangles,std::vector<osg::Vec3Array *>   &texCoord,double scaletex=1.0,int VTtileSize=-1,int border=-1);
void dilateEdge(IMAGE *tmpI,const char *outfile,int count=-1);
void getBoundsForClippingReparam(osg::Vec3Array*coords, osg::Vec2 &minV, osg::Vec2 &maxV);
void dilateEdgeNew(vips::VImage &input,const char *outfile,int count);
class LoaderOSG2OGF  {



private:
    osg::Vec3Array* _verts;
    osg::DrawElementsUInt *_triangles;
public:

   /// Constructor
   /// @param f name of a .ply file that stores the model
    LoaderOSG2OGF (OGF::Map *map,osg::Vec3Array* verts,osg::DrawElementsUInt *triangles) : B(map),_verts(verts),_triangles(triangles) {
        facet_map.resize(_triangles->size(),NULL);
   }

   /// Loads the PLY model and build the CGAL Halfedge Data Structure
   /// @param hds - halfedge data structure.
   bool build ()  {



      B.begin_surface();
      for (int i = 0 ; i < (int)_verts->size() ; i++) {
          B.add_vertex (OGF::Point3d (_verts->at(i)[0],_verts->at(i)[1],_verts->at(i)[2]) );
      }

      for (int i = 0 ; i < (int)_triangles->size()-2 ; i+=3) {
          //if(_triangles->at(i+0) <0 ||_triangles->at(i+1) <0||_triangles->at(i+2) <0)
             // continue;
         B.begin_facet();
         for (int j = 0; j <3; j++) {
             B.add_vertex_to_facet (_triangles->at(i+j));
         }
         B.end_facet();
         for(int j=0;j<3;j++)
         facet_map[i+j]=B.current_facet();

      }
      B.end_surface ();
      std::cout <<"Loading model ... done " << std::endl;
   //   cout << "B.error()" <<_verts->size() << " "<<_triangles->size() << endl;
      if(B.map()){
          return true;
      }
      return false;
   }
   OGF::MapBuilder B;
   std::vector<OGF::Map::Facet*> facet_map;

};



#endif // GENPARAM_H
