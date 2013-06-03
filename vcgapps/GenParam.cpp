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

/*#include <CGAL/basic.h> // include basic.h before testing #defines

#include <CGAL/Cartesian.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/IO/Polyhedron_iostream.h>
#include <CGAL/Parameterization_polyhedron_adaptor_3.h>
#include <CGAL/parameterize.h>
#include <CGAL/Discrete_authalic_parameterizer_3.h>
#include <CGAL/Barycentric_mapping_parameterizer_3.h>

#include <CGAL/Square_border_parameterizer_3.h>
#include <CGAL/Parameterization_mesh_patch_3.h>
*/
//#include <CGAL/Eigen_solver_traits.h>


#include <OGF/cells/types/cells_library.h>
#include <OGF/cells/map/map.h>
#include <OGF/cells/map/map_attributes.h>
#include <OGF/cells/map/map_builder.h>

#include <OGF/cells/map/map_editor.h>
#include <OGF/cells/map/geometry.h>
#include <OGF/cells/map_algos/atlas_generator.h>
#include <OGF/cells/map_algos/pm_manager.h>
#include <OGF/image/types/image.h>
#include <OGF/cells/map_algos/variational_map_splitter.h>
#include <OGF/image/types/image_library.h>
#include <OGF/image/algos/rasterizer.h>
#include <OGF/image/algos/morpho_math.h>
#include <OGF/image/io/image_serializer_ppm.h>
#include <OGF/basic/os/file_system.h>
#include <OGF/cells/io/map_serializer_obj.h>

#include <iostream>
#include <fstream>
#include <cstdlib>
#include <algorithm>
#include <functional>
#include <numeric>
#include <vips/vips>
#include <vips/vips.h>
#include "GenParam.h"

#include <osgDB/FileUtils>
#include <osg/io_utils>
using namespace std;
//#include <CGAL/Polyhedron_incremental_builder_3.h>
int calcOptimalImageSize(const osg::Vec2 imageSize,osg::Vec3Array *verts,osg::DrawElementsUInt* triangles,std::vector<osg::Vec3Array *>   &texCoord,double scaletex,int VTtileSize,int border){
    if(!verts || !triangles || texCoord.size()==0)
    return -1;
std::vector<double> pixelSides;
    for (int i = 0 ; i < (int)triangles->size()-2 ; i+=3) {
        int idx=i/3;
        int in0=triangles->at(i+0);
        int in1=triangles->at(i+1);
        int in2=triangles->at(i+2);
        if(in0 <0 || in1< 0 || in2<0||in0>=verts->size()|| in1>=verts->size()|| in2>=verts->size())
            continue;
        const osg::Vec3 &v1=verts->at(in0);
            const osg::Vec3 &v2=verts->at(in1);
            const osg::Vec3 &v3=verts->at(in2);
            double tex_area=((v2-v1)^(v3-v2)).length()/2.0;
           // cout << tex_area <<endl;
           // cout << v1 << " " << v2 << " "<<v3<<endl;
            double max_orig_tex_area=0.0;
            for(int c=0; c <4; c++){
             //   printf("Bla %f %f\n",texCoord[c]->at(idx).x(),texCoord[c]->at(idx).y());
              osg::Vec3 tc1=  osg::Vec3(texCoord[c]->at(in0).x()*imageSize.x(),texCoord[c]->at(in0).y()*imageSize.y(),0);
              osg::Vec3 tc2=   osg::Vec3(texCoord[c]->at(in1).x()*imageSize.x(),texCoord[c]->at(in1).y()*imageSize.y(),0);
              osg::Vec3 tc3=   osg::Vec3(texCoord[c]->at(in2).x()*imageSize.x(),texCoord[c]->at(in2).y()*imageSize.y(),0);
//              cout <<tc1 <<" " <<tc2<< " "<<tc3<<endl;
              if(tc1.x() < 0 || tc1.y() < 0 ||tc2.x() < 0 || tc2.y() < 0||tc3.x() < 0 || tc3.y() < 0)
                  continue;
             double orig_tex_area=((tc2-tc1)^(tc3-tc2)).length()/2.0;
            // cout <<orig_tex_area<<endl;
             if(orig_tex_area>max_orig_tex_area)
                 max_orig_tex_area=orig_tex_area;
            }
            if(max_orig_tex_area ==0.0)
                continue;
           // printf("%f %f\n",max_orig_tex_area,8192*tex_area);
            double sidePixels=sqrt(max_orig_tex_area/tex_area);
            //cout <<sidePixels<<endl;
            if(isfinite(sidePixels) && sidePixels >0 && sidePixels < pow(2.0f,17))
                pixelSides.push_back(sidePixels);
    }
    if(pixelSides.size() ==0)
        return 32;
    double avgEl=0.0;
    for(int i=0;i < pixelSides.size(); i++){
       avgEl+= (pixelSides[i]/pixelSides.size());
    }
   // long double sum = std::accumulate( pixelSides.begin(), pixelSides.begin()+pixelSides.size(), 0 ) ;

//   double avgEl =sum/pixelSides.size();
   double maxEl= *( std::max_element( pixelSides.begin(), pixelSides.end() ) );
   if(avgEl<0){
       for(int i=0;i < pixelSides.size(); i++)
           printf("%f ",pixelSides[i]);
                   printf("\n");
       avgEl=maxEl/2.0;
   }
    avgEl=std::min(8192.0,avgEl);
   int potSize=osg::Image::computeNearestPowerOfTwo((int)avgEl*scaletex);

   if(VTtileSize> 0 && border >0){
        int adjustedpotSize=(int)potSize-((potSize/VTtileSize)*2*border);
        printf("Max %f Avg %f POT %d Adjusted VT %d\n",maxEl,avgEl,potSize,adjustedpotSize);
        return adjustedpotSize;

   }
       printf("Max %f Avg %f POT %d Adjusted\n",maxEl,avgEl,potSize);

return potSize;
}
int median(vector<double> &v)
{
    size_t n = v.size() / 2;
    nth_element(v.begin(), v.begin()+n, v.end());
    return v[n];
}
int calcOptimalImageSizeRaw(const osg::Vec2 imageSize,osg::Vec3Array *verts,osg::DrawElementsUInt* triangles,std::vector<osg::Vec3Array *>   &texCoord,double scaletex,int VTtileSize,int border){
    if(!verts || !triangles || texCoord.size()==0)
    return -1;
std::vector<double> pixelSides;
    for (int i = 0 ; i < (int)triangles->size()-2 ; i+=3) {
        int idx=i/3;
        int in0=triangles->at(i+0);
        int in1=triangles->at(i+1);
        int in2=triangles->at(i+2);
        if(in0 <0 || in1< 0 || in2<0||in0>=verts->size()|| in1>=verts->size()|| in2>=verts->size())
            continue;
        const osg::Vec3 &v1=verts->at(in0);
            const osg::Vec3 &v2=verts->at(in1);
            const osg::Vec3 &v3=verts->at(in2);
            double tex_area=((v2-v1)^(v3-v2)).length()/2.0;
           // cout << tex_area <<endl;
           // cout << v1 << " " << v2 << " "<<v3<<endl;
            double max_orig_tex_area=0.0;
            for(int c=0; c <4; c++){
             //   printf("Bla %f %f\n",texCoord[c]->at(idx).x(),texCoord[c]->at(idx).y());
              osg::Vec3 tc1=  osg::Vec3(texCoord[c]->at(in0).x()*imageSize.x(),texCoord[c]->at(in0).y()*imageSize.y(),0);
              osg::Vec3 tc2=   osg::Vec3(texCoord[c]->at(in1).x()*imageSize.x(),texCoord[c]->at(in1).y()*imageSize.y(),0);
              osg::Vec3 tc3=   osg::Vec3(texCoord[c]->at(in2).x()*imageSize.x(),texCoord[c]->at(in2).y()*imageSize.y(),0);
//              cout <<tc1 <<" " <<tc2<< " "<<tc3<<endl;
              if(tc1.x() < 0 || tc1.y() < 0 ||tc2.x() < 0 || tc2.y() < 0||tc3.x() < 0 || tc3.y() < 0)
                  continue;
             double orig_tex_area=((tc2-tc1)^(tc3-tc2)).length()/2.0;
            // cout <<orig_tex_area<<endl;
             if(orig_tex_area>max_orig_tex_area)
                 max_orig_tex_area=orig_tex_area;
            }
            if(max_orig_tex_area ==0.0)
                continue;
           // printf("%f %f\n",max_orig_tex_area,8192*tex_area);
            double sidePixels=sqrt(max_orig_tex_area/tex_area);
            //cout <<sidePixels<<endl;
            if(isfinite(sidePixels) && sidePixels >0 && sidePixels < pow(2.0f,17))
                pixelSides.push_back(sidePixels);
    }
    if(pixelSides.size() ==0)
        return 1;
    double avgEl=0.0;
    for(int i=0;i < pixelSides.size(); i++){
       avgEl+= (pixelSides[i]/pixelSides.size());
    }
    double medianVal=median(pixelSides);
   // long double sum = std::accumulate( pixelSides.begin(), pixelSides.begin()+pixelSides.size(), 0 ) ;

//   double avgEl =sum/pixelSides.size();

    //if(avgEl<0)
  /* FILE*fp =fopen("tmp.txt","w");
   for(int i=0;i < pixelSides.size(); i++)
       fprintf(fp,"%f\n",pixelSides[i]);
   fclose(fp);
   {
       for(int i=0;i < pixelSides.size(); i++)
           printf("%f ",pixelSides[i]);
                   printf("\n");
       avgEl=maxEl/2.0;
   }*/

return medianVal*scaletex;
}

namespace OGF {
class AtlasValidator: public ParamValidator{
public:
    void begin_rasterizer(Map* map) {
        ::memset(graph_mem_, 0, graph_size_ * graph_size_) ;

        Box2d box = Geom::map_bbox2d(map) ;

        user_x_min_  = box.x_min() ;
        user_y_min_  = box.y_min() ;
        user_width_  = box.width() ;
        user_height_ = box.height() ;
        user_size_ = ogf_max(user_width_, user_height_) ;
    }

    void compute_fill_and_overlap_ratio(Map* map) {
        begin_rasterizer(map) ;
        FOR_EACH_FACET(Map,map,f) {
            Map::Halfedge* cur = f-> halfedge() ;
            Map::Halfedge* h0 = cur ;
            cur = cur-> next() ;
            Map::Halfedge* h1 = cur ;
            cur = cur-> next() ;
            Map::Halfedge* h2 = cur ;
            do {
                rasterize_triangle(
                            h0->tex_coord(),
                            h1->tex_coord(),
                            h2->tex_coord()
                            ) ;
                h1 = cur ;
                cur = cur-> next() ;
                h2 = cur ;
            } while (h2 != h0) ;
        }
        end_rasterizer() ;
    }

};

    void generate_atlas(Map* the_map) {
        AtlasGenerator the_generator(the_map) ;
        the_generator.set_unglue_hardedges(false) ;
        the_generator.set_auto_cut(true) ;
        the_generator.set_auto_cut_cylinders(true) ;
        the_generator.set_parameterizer("ABF++") ;
//        the_generator.set_max_overlap_ratio(0.0001) ;
        the_generator.set_max_overlap_ratio(0.2) ;
        the_generator.set_max_scaling(120.0) ;
        the_generator.set_min_fill_ratio(0.25) ;
        the_generator.set_pack(true) ;
        OGF::SmoothVariationalMapComponentSplitter *vasmooth =new SmoothVariationalMapComponentSplitter;
        vasmooth->set_max_components(3);
        //vasmooth->set_error_decrease_factor(0.1);;
              the_generator.set_splitter(vasmooth) ;
  //      the_generator.set_splitter("VSASmooth") ;
        the_generator.apply() ;
    }

    void decimate_surface(Map* map, double factor = 0.25) {

           bool is_triangulated = true ;
           FOR_EACH_FACET(Map, map, it) {
               if(!it->is_triangle()) {
                   is_triangulated = false ;
                   break ;
               }
           }

           if(!is_triangulated) {
               std::cerr << "Triangulating surface (this is required by multires algos)"
                         << std::endl ;
               MapEditor editor(map) ;
               editor.split_surface(split_triangulate) ;
           }

           // Needs to be normalized, else PM manager does
           // not operate properly.
           MapNormalizer normalizer(map) ;
           normalizer.apply() ;
           PMManager pm_manager(map);
           pm_manager.init();
           pm_manager.set_progress(nil) ;
           normalizer.unapply() ;
           pm_manager.set_level(int((1.0 - factor) * pm_manager.size())) ;
       }
    // Dump parameterized mesh to an eps file
    static bool write_file_eps(Map& map,
                               const char *pFilename,
                               double scale = 500.0)
    {

        std::ofstream out(pFilename);
        if(!out)
            return false;
       // CGAL::set_ascii_mode(out);

        // compute bounding box
        double xmin,xmax,ymin,ymax;
        xmin = ymin = xmax = ymax = 0;
        FOR_EACH_HALFEDGE(Map,&map,pHalfedge)
        {

            double x1 = scale * pHalfedge->prev()->tex_coord().x();
            double y1 = scale * pHalfedge->prev()->tex_coord().y();
            double x2 = scale * pHalfedge->tex_coord().x();
            double y2 = scale * pHalfedge->tex_coord().y();
            xmin = (std::min)(xmin,x1);
            xmin = (std::min)(xmin,x2);
            xmax = (std::max)(xmax,x1);
            xmax = (std::max)(xmax,x2);
            ymax = (std::max)(ymax,y1);
            ymax = (std::max)(ymax,y2);
            ymin = (std::min)(ymin,y1);
            ymin = (std::min)(ymin,y2);
        }

        out << "%!PS-Adobe-2.0 EPSF-2.0" << std::endl;
        out << "%%BoundingBox: " << int(xmin+0.5) << " "
                                    << int(ymin+0.5) << " "
                                    << int(xmax+0.5) << " "
                                    << int(ymax+0.5) << std::endl;
        out << "%%HiResBoundingBox: " << xmin << " "
                                        << ymin << " "
                                        << xmax << " "
                                        << ymax << std::endl;
        out << "%%EndComments" << std::endl;
        out << "gsave" << std::endl;
        out << "0.1 setlinewidth" << std::endl;

        // color macros
        out << std::endl;
        out << "% RGB color command - r g b C" << std::endl;
        out << "/C { setrgbcolor } bind def" << std::endl;
        out << "/white { 1 1 1 C } bind def" << std::endl;
        out << "/black { 0 0 0 C } bind def" << std::endl;

        // edge macro -> E
        out << std::endl;
        out << "% Black stroke - x1 y1 x2 y2 E" << std::endl;
        out << "/E {moveto lineto stroke} bind def" << std::endl;
        out << "black" << std::endl << std::endl;

        // for each halfedge
        FOR_EACH_HALFEDGE(Map,&map,pHalfedge)
        {
            double x1 = scale * pHalfedge->prev()->tex_coord().x();
            double y1 = scale * pHalfedge->prev()->tex_coord().y();
            double x2 = scale * pHalfedge->tex_coord().x();
            double y2 = scale * pHalfedge->tex_coord().y();
            out << x1 << " " << y1 << " " << x2 << " " << y2 << " E" << std::endl;
        }

        /* Emit EPS trailer. */
        out << "grestore" << std::endl;
        out << std::endl;
        out << "showpage" << std::endl;

        return true;
    }


}
using namespace std;

void getBoundsForClippingReparam(osg::Vec3Array*coords, osg::Vec2 &minV, osg::Vec2 &maxV,double margin){
    minV=osg::Vec2(FLT_MAX,FLT_MAX);
    maxV=osg::Vec2(-FLT_MAX,-FLT_MAX);

    for(int i=0; i< coords->size(); i++){
        for(int j=0; j <2; j++){
            if(!( coords->at(i)[j] >= 0.0 && coords->at(i)[j] <= 1.0))
                continue;

            if(coords->at(i)[j] < minV[j])
                minV[j]=coords->at(i)[j];
            if(coords->at(i)[j] > maxV[j])
                maxV[j]=coords->at(i)[j];
        }

    }
    osg::Vec2 rangeV((maxV-minV).x(),(maxV-minV).y());
    double marginRange=1.0-(2*margin);
    for(int i=0; i< coords->size(); i++){
        for(int j=0; j <2; j++){
            if(!( coords->at(i)[j] >= 0.0 && coords->at(i)[j] <= 1.0))
                continue;

            coords->at(i)[j]= (((coords->at(i)[j]-minV[j])/rangeV[j])*marginRange)+margin;

        }

    }
}

osg::Vec3Array* OGFreparam(osg::ref_ptr<osg::Vec3Array> verts,osg::ref_ptr<osg::DrawElementsUInt> triangles){
    OGF::Map the_map ;
      std::cerr << "==== Step 1/5 == Loading map: " << std::endl ;
      LoaderOSG2OGF map_builder(&the_map,verts,triangles);

      if(!map_builder.build()) {
          std::cerr << "Could not open proces model" << std::endl ;
          exit(-1) ;
      }
      std::cerr << "nb facets: " << the_map.size_of_facets() << " nb vertices:" << the_map.size_of_vertices() << std::endl ;
      std::cerr << std::endl ;
      the_map.compute_normals() ;
      /*OGF::MapSerializer_obj mso;
      ofstream f("temp.obj");
      mso.serialize_write(&the_map,f);*/
      std::cerr << "==== Step 2/5 == Generating texture atlas" << std::endl ;
         OGF::generate_atlas(&the_map) ;
         OGF::AtlasValidator val;
         val.compute_fill_and_overlap_ratio(&the_map);
         std::cerr << "Created atlas with fill ratio of " << val.fill_ratio()<< std::endl ;

   //    OGF::write_file_eps(the_map,"test.eps");
         //exit(0);
         osg::Vec3Array *arr=new osg::Vec3Array;
         arr->resize(triangles->size());
         osg::BoundingBox bbox;

        for (int i = 0 ; i < (int)triangles->size()-2 ; i+=3) {
            if(!map_builder.facet_map[i]){
                fprintf(stderr,"Warning no face coorepsonds to this triangle");
                exit(-1);
            }
             OGF::Map::Facet* it=map_builder.facet_map[i];
             OGF::Map::Halfedge* h = it->halfedge() ;
             int lEdge=1;
             do {
                 OGF::Map::Vertex *vert=h->vertex();
                 double u =  h->tex_coord().x();
                 double v =  h->tex_coord().y();
                 bbox.expandBy(osg::Vec3(u,v,0));
                 arr->at(i+lEdge)=osg::Vec3(osg::Vec3(u,v,0));
                 lEdge=(lEdge+1)%3;
                 h = h->next() ;

             } while (h != it->halfedge());
         }
         cout <<"Final Coords"<<bbox._min<< " "<<bbox._max<<endl;
         return arr;
}
void dilateEdge(IMAGE *tmpI,const char *outfile,int count){
    OGF::Image* img = new OGF::Image(OGF::Image::RGB,tmpI->Xsize,tmpI->Ysize);
    OGF::Memory::byte* mem_img = img->base_mem_byte_ptr();
    vips::VImage tmpImage(mem_img,tmpI->Xsize,tmpI->Ysize,3,vips::VImage::FMTUCHAR);
    vips::VImage v(tmpI);
    v.write(tmpImage);
    OGF::MorphoMath morpho(img) ;
    OGF::ImageSerializer_ppm *p = new OGF::ImageSerializer_ppm;
    std::ofstream outf(outfile);
    if(count == -1)
     count = std::max(2,(int)round(10*(tmpI->Xsize/(double)8192)));
    printf("Dilating %d times\n",count);
    for(int i=0; i<count; i++) {
        morpho.dilate(1) ;
    }
    p->serialize_write(outf,img);
    delete p;

}

void dilateEdgeNew(vips::VImage &input,const char *outfile,int count){
    OGF::Image* img = new OGF::Image(OGF::Image::RGB,input.Xsize(),input.Ysize());
    OGF::Memory::byte* mem_img = img->base_mem_byte_ptr();
    vips::VImage tmpImage(mem_img,input.Xsize(),input.Ysize(),3,vips::VImage::FMTUCHAR);

    input.write(tmpImage);
    OGF::MorphoMath morpho(img) ;
    if(count == -1)
     count = std::max(2,(int)round(10*(input.Xsize()/(double)8192)));
   // printf("Dilating %d times\n",count);
    for(int i=0; i<count; i++) {
        morpho.dilate(1) ;
    }
    tmpImage.write((string(outfile)+":90").c_str());
}


using namespace std;

#if 0
template <class HDS>
class LoaderOSG : public CGAL::Modifier_base<HDS> {



private:
    osg::Vec3Array* _verts;
    osg::DrawElementsUInt *_triangles;
public:

   /// Constructor
   /// @param f name of a .ply file that stores the model
    LoaderOSG (osg::Vec3Array* verts,osg::DrawElementsUInt *triangles) : _verts(verts),_triangles(triangles) {

   }

   /// Loads the PLY model and build the CGAL Halfedge Data Structure
   /// @param hds - halfedge data structure.
   void operator () (HDS& hds) {


      typedef typename HDS::Vertex   Vertex;
      typedef typename Vertex::Point Point;
      CGAL::Polyhedron_incremental_builder_3<HDS> B(hds, true);
      B.begin_surface(_verts->size(),_triangles->size()/3);
      for (int i = 0 ; i < (int)_verts->size() ; i++) {
          B.add_vertex (Point (_verts->at(i)[0],_verts->at(i)[1],_verts->at(i)[2]) );
      }

      for (int i = 0 ; i < (int)_triangles->size()-2 ; i+=3) {
         B.begin_facet();
         for (int j = 0; j <3; j++) {
             B.add_vertex_to_facet (_triangles->at(i+j));
         }
         B.end_facet();
      }
      B.end_surface ();
      cout <<"Loading model ... done " << endl;
   //   cout << "B.error()" <<_verts->size() << " "<<_triangles->size() << endl;
   }
};
// ----------------------------------------------------------------------------
// Private types
// ----------------------------------------------------------------------------

typedef CGAL::Cartesian<double>             Kernel;
typedef CGAL::Polyhedron_3<Kernel>          Polyhedron;

// Polyhedron adaptor
typedef CGAL::Parameterization_polyhedron_adaptor_3<Polyhedron>
                                            Parameterization_polyhedron_adaptor;

// Type describing a border or seam as a vertex list
typedef std::list<Parameterization_polyhedron_adaptor::Vertex_handle>
                                            Seam;


// ----------------------------------------------------------------------------
// Private functions
// ----------------------------------------------------------------------------

// If the mesh is a topological disk, extract its longest border,
// else compute a very simple cut to make it homeomorphic to a disk.
// Return the border of this region (empty on error)
//
// CAUTION: this cutting algorithm is very naive. Write your own!
static Seam cut_mesh(Parameterization_polyhedron_adaptor& mesh_adaptor)
{
    // Helper class to compute genus or extract borders
    typedef CGAL::Parameterization_mesh_feature_extractor<Parameterization_polyhedron_adaptor>
                                            Mesh_feature_extractor;

    Seam seam;              // returned list

    // Get reference to Polyhedron_3 mesh
    Polyhedron& mesh = mesh_adaptor.get_adapted_mesh();

    // Extract mesh borders and compute genus
    Mesh_feature_extractor feature_extractor(mesh_adaptor);
    int nb_borders = feature_extractor.get_nb_borders();
    int genus = feature_extractor.get_genus();

    // If mesh is a topological disk
    if (genus == 0 && nb_borders > 0)
    {
        // Pick the longest border
        seam = feature_extractor.get_longest_border();
    }
    else // if mesh is *not* a topological disk, create a virtual cut
    {
        const int CUT_LENGTH = 6;

        // Build consecutive halfedges array
        Polyhedron::Halfedge_handle seam_halfedges[CUT_LENGTH];
        seam_halfedges[0] = mesh.halfedges_begin();
        if (seam_halfedges[0] == NULL)
            return seam;                    // return empty list
        int i;
        for (i=1; i<CUT_LENGTH; i++)
        {
            seam_halfedges[i] = seam_halfedges[i-1]->next()->opposite()->next();
            if (seam_halfedges[i] == NULL)
                return seam;                // return empty list
        }

        // Convert halfedges array to two-ways vertices list
        for (i=0; i<CUT_LENGTH; i++)
            seam.push_back(seam_halfedges[i]->vertex());
        for (i=CUT_LENGTH-1; i>=0; i--)
            seam.push_back(seam_halfedges[i]->opposite()->vertex());
    }

    return seam;
}


// Dump parameterized mesh to an eps file
static bool write_file_eps(const Parameterization_polyhedron_adaptor& mesh_adaptor,
                           const char *pFilename,
                           double scale = 500.0)
{
    const Polyhedron& mesh = mesh_adaptor.get_adapted_mesh();

    std::ofstream out(pFilename);
    if(!out)
        return false;
    CGAL::set_ascii_mode(out);

    // compute bounding box
    double xmin,xmax,ymin,ymax;
    xmin = ymin = xmax = ymax = 0;
    Polyhedron::Halfedge_const_iterator pHalfedge;
    for (pHalfedge = mesh.halfedges_begin();
         pHalfedge != mesh.halfedges_end();
         pHalfedge++)
    {
        double x1 = scale * mesh_adaptor.info(pHalfedge->prev())->uv().x();
        double y1 = scale * mesh_adaptor.info(pHalfedge->prev())->uv().y();
        double x2 = scale * mesh_adaptor.info(pHalfedge)->uv().x();
        double y2 = scale * mesh_adaptor.info(pHalfedge)->uv().y();
        xmin = (std::min)(xmin,x1);
        xmin = (std::min)(xmin,x2);
        xmax = (std::max)(xmax,x1);
        xmax = (std::max)(xmax,x2);
        ymax = (std::max)(ymax,y1);
        ymax = (std::max)(ymax,y2);
        ymin = (std::min)(ymin,y1);
        ymin = (std::min)(ymin,y2);
    }

    out << "%!PS-Adobe-2.0 EPSF-2.0" << std::endl;
    out << "%%BoundingBox: " << int(xmin+0.5) << " "
                                << int(ymin+0.5) << " "
                                << int(xmax+0.5) << " "
                                << int(ymax+0.5) << std::endl;
    out << "%%HiResBoundingBox: " << xmin << " "
                                    << ymin << " "
                                    << xmax << " "
                                    << ymax << std::endl;
    out << "%%EndComments" << std::endl;
    out << "gsave" << std::endl;
    out << "0.1 setlinewidth" << std::endl;

    // color macros
    out << std::endl;
    out << "% RGB color command - r g b C" << std::endl;
    out << "/C { setrgbcolor } bind def" << std::endl;
    out << "/white { 1 1 1 C } bind def" << std::endl;
    out << "/black { 0 0 0 C } bind def" << std::endl;

    // edge macro -> E
    out << std::endl;
    out << "% Black stroke - x1 y1 x2 y2 E" << std::endl;
    out << "/E {moveto lineto stroke} bind def" << std::endl;
    out << "black" << std::endl << std::endl;

    // for each halfedge
    for (pHalfedge = mesh.halfedges_begin();
         pHalfedge != mesh.halfedges_end();
         pHalfedge++)
    {
        double x1 = scale * mesh_adaptor.info(pHalfedge->prev())->uv().x();
        double y1 = scale * mesh_adaptor.info(pHalfedge->prev())->uv().y();
        double x2 = scale * mesh_adaptor.info(pHalfedge)->uv().x();
        double y2 = scale * mesh_adaptor.info(pHalfedge)->uv().y();
        out << x1 << " " << y1 << " " << x2 << " " << y2 << " E" << std::endl;
    }

    /* Emit EPS trailer. */
    out << "grestore" << std::endl;
    out << std::endl;
    out << "showpage" << std::endl;

    return true;
}



// ----------------------------------------------------------------------------
// main()
// ----------------------------------------------------------------------------

osg::Vec3Array* CGALreparam(osg::ref_ptr<osg::Vec3Array> verts,osg::ref_ptr<osg::DrawElementsUInt> triangles)
{
    std::cerr << "PARAMETERIZATION" << std::endl;
       std::cerr << "  Discrete Authalic Parameterization" << std::endl;
       std::cerr << "  Square border" << std::endl;
//       std::cerr << "  Eigen solver" << std::endl;
       std::cerr << "  Very simple cut if model is not a topological disk" << std::endl;
       std::cerr << "  Output: EPS" << std::endl;

    //***************************************
    // decode parameters
    //***************************************

    if (!verts.valid() || !triangles.valid())
    {
        std::cerr << "Arrays  don't exist\n";
        return NULL;
    }


    //***************************************
    // Read the mesh
    //***************************************

    // Read the mesh

    LoaderOSG<Polyhedron::HDS> builder(verts.get(),triangles.get());

    Polyhedron mesh;
    mesh.delegate(builder);

    if(!mesh.is_valid() || mesh.empty())
    {
        std::cerr << "Error: cannot read mesh" << mesh.is_valid() <<" "<< mesh.empty()<<  std::endl;
        return NULL;
    }
{
        std::ofstream stream("test.off");
 stream <<mesh;
    }
    //***************************************
      // Create Polyhedron adaptor
      //***************************************

      Parameterization_polyhedron_adaptor mesh_adaptor(mesh);

      //***************************************
      // Virtually cut mesh
      //***************************************

      // The parameterization methods support only meshes that
      // are topological disks => we need to compute a "cutting" of the mesh
      // that makes it homeomorphic to a disk
      Seam seam = cut_mesh(mesh_adaptor);
      if (seam.empty())
      {
          std::cerr << "Input mesh not supported: the example cutting algorithm is too simple to cut this shape" << std::endl;
          return NULL;
      }

      // Create a second adaptor that virtually "cuts" the mesh following the 'seam' path
      typedef CGAL::Parameterization_mesh_patch_3<Parameterization_polyhedron_adaptor>
                                              Mesh_patch_polyhedron;
      Mesh_patch_polyhedron   mesh_patch(mesh_adaptor, seam.begin(), seam.end());
      if (!mesh_patch.is_valid())
      {
          std::cerr << "Input mesh not supported: non manifold shape or invalid cutting" << std::endl;
          return NULL;
      }

      //***************************************
      // Discrete Authalic Parameterization (square border)
      // with Eigen solver
      //***************************************

      // Border parameterizer
     typedef CGAL::Square_border_uniform_parameterizer_3<Mesh_patch_polyhedron>
                                                              Border_parameterizer;

      // Eigen solver
    /*  typedef CGAL::Eigen_solver_traits<>                Solver;

      // Discrete Authalic Parameterization (square border)
      // with Eigen solver
      typedef CGAL::Discrete_authalic_parameterizer_3<Mesh_patch_polyhedron,
                                                      ,
                                                      Solver> Parameterizer;
*/
      //typedef CGAL::Discrete_authalic_parameterizer_3<Mesh_patch_polyhedron,
       //                                               Border_parameterizer
                                               //       > Parameterizer;
      typedef  CGAL::Barycentric_mapping_parameterizer_3<Mesh_patch_polyhedron,Border_parameterizer>
                Parameterizer;
      Parameterizer::Error_code err = CGAL::parameterize(mesh_patch, Parameterizer());

    switch(err) {
    case Parameterizer::OK: // Success
        break;
    case Parameterizer::ERROR_EMPTY_MESH: // Input mesh not supported
    case Parameterizer::ERROR_NON_TRIANGULAR_MESH:
    case Parameterizer::ERROR_NO_TOPOLOGICAL_DISC:
    case Parameterizer::ERROR_BORDER_TOO_SHORT:
        std::cerr << "Input mesh not supported: " << Parameterizer::get_error_message(err) << std::endl;
        return NULL;
        break;
    default: // Error
        std::cerr << "Error: " << Parameterizer::get_error_message(err) << std::endl;
        return  NULL;;
        break;
    };

    //***************************************
    // Output
    //***************************************
    // Write Postscript file
    if ( ! write_file_eps(mesh_adaptor, "test.eps") )
       {
           std::cerr << "Error: cannot write file " << "test.eps" << std::endl;
           return NULL;
       }

    // Raw output: dump (u,v) pairs
    osg::Vec3Array *arr=new osg::Vec3Array;
    Polyhedron::Facet_const_iterator pFace;
    typedef Polyhedron::Halfedge_around_facet_circulator
                                                Halfedge_around_facet_circulator;
    typedef Polyhedron::Facet_iterator                           Facet_iterator;

    for ( Facet_iterator i = mesh.facets_begin(); i != mesh.facets_end(); ++i) {
           Halfedge_around_facet_circulator j = i->facet_begin();
           // Facets in polyhedral surfaces are at least triangles.
           CGAL_assertion( CGAL::circulator_size(j) >= 3);
           do {
               double u = mesh_adaptor.info(j)->uv().x();
               double v = mesh_adaptor.info(j)->uv().y();
               arr->push_back(osg::Vec3(u,v,0));

           } while ( ++j != i->facet_begin());
       }
/*
    for (pFace = mesh.facets_begin();
         pFace != mesh.facets_end();
         pFace++)
    {



        Halfedge_facet_circulator j = pFace->facet_begin();
        // Facets in polyhedral surfaces are at least triangles.
        CGAL_assertion( CGAL::circulator_size(j) >= 3);
        do {
            double u = mesh_adaptor.info(j)->uv().x();
            double v = mesh_adaptor.info(j)->uv().y();
            arr->push_back(osg::Vec3(u,v,0));

        } while ( ++j != pFace->facet_begin());

    }*/
   /* Polyhedron::Vertex_const_iterator pVertex;
    for (pVertex = mesh.vertices_begin();
        pVertex != mesh.vertices_end();
        pVertex++)
    {
        // (u,v) pair is stored in any halfedge
        double u = mesh_adaptor.info(pVertex->halfedge())->uv().x();
        double v = mesh_adaptor.info(pVertex->halfedge())->uv().y();
      //  std::cout << "(u,v) = (" << u << "," << v << ")" << std::endl;
        arr->push_back(osg::Vec3(u,v,0));
    }*/

    return arr;
}

#endif
