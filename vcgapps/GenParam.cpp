#include <CGAL/basic.h> // include basic.h before testing #defines

#include <CGAL/Cartesian.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/IO/Polyhedron_iostream.h>
#include <CGAL/Parameterization_polyhedron_adaptor_3.h>
#include <CGAL/parameterize.h>
#include <CGAL/Discrete_authalic_parameterizer_3.h>
#include <CGAL/Barycentric_mapping_parameterizer_3.h>

#include <CGAL/Square_border_parameterizer_3.h>
#include <CGAL/Parameterization_mesh_patch_3.h>

//#include <CGAL/Eigen_solver_traits.h>

#include <iostream>
#include <fstream>
#include <cstdlib>

#include <osgDB/FileUtils>

#include <CGAL/Polyhedron_incremental_builder_3.h>
using namespace std;
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

