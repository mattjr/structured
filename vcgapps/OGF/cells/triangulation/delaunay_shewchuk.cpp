
/*
 *  OGF/Graphite: Geometry and Graphics Programming Library + Utilities
 *  Copyright (C) 2000-2005 INRIA - Project ALICE
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 *  If you modify this software, you should include a notice giving the
 *  name of the person performing the modification, the date of modification,
 *  and the reason for such modification.
 *
 *  Contact: Bruno Levy - levy@loria.fr
 *
 *     Project ALICE
 *     LORIA, INRIA Lorraine, 
 *     Campus Scientifique, BP 239
 *     54506 VANDOEUVRE LES NANCY CEDEX 
 *     FRANCE
 *
 *  Note that the GNU General Public License does not permit incorporating
 *  the Software into proprietary programs. 
 *
 * As an exception to the GPL, Graphite can be linked with the following (non-GPL) libraries:
 *     Qt, SuperLU, WildMagic and CGAL
 */

#include <OGF/cells/triangulation/delaunay_shewchuk.h>

#define VOID void 
#define REAL double
#define ANSI_DECLARATORS
extern "C" {
#include <OGF/cells/third_party/triangle.h>
}

#include <string.h>

namespace OGF {

/*
    DelaunayShewchuk::DelaunayShewchuk() { 
        in_begin_end_ = false ;
    }

    DelaunayShewchuk::~DelaunayShewchuk() {
    }
    
    void DelaunayShewchuk::begin() {
        ogf_assert(!in_begin_end_) ;
        in_begin_end_ = true ;
        vertices_.clear() ;

        // For the moment:
        // we clear the target Triangulation and reinsert the vertices.
        TriangulationVertexLock is_locked(target()) ;
        std::vector<Point2d> vertices ;
        std::vector<bool> locked ;
        FOR_EACH_VERTEX(Triangulation, target(), it) {
            double x = it->point().x() ;
            double y = it->point().y() ;
            vertices.push_back(Point2d(x,y)) ;
            locked.push_back(is_locked[it]) ;
        }
        target()->clear() ;
        for(unsigned int i=0; i<vertices.size(); i++) {
            Triangulation::Vertex* v = add_vertex(vertices[i]) ;
            is_locked[v] = locked[i] ;
        }
    }
    
    
    Triangulation::Vertex* DelaunayShewchuk::add_vertex(const Point2d& p) {
        ogf_assert(in_begin_end_) ;
        Point3d q(p.x(), p.y(), 0.0) ;
        Triangulation::Vertex* v = new_vertex() ;
        v->set_point(q) ;
        vertices_.push_back(v) ;
        return v ;
    }

    static void free_triangulateio(struct triangulateio* tri) {
        free(tri->pointlist) ;
        free(tri->pointattributelist) ;
        free(tri->pointmarkerlist) ;
        free(tri->trianglelist) ;
        free(tri->triangleattributelist) ;
        free(tri->trianglearealist) ;
        free(tri->neighborlist) ;
        free(tri->segmentlist) ;
        free(tri->segmentmarkerlist) ;
        free(tri->holelist) ;
        free(tri->regionlist) ;
        free(tri->edgelist) ;
        free(tri->edgemarkerlist) ;
        free(tri->normlist) ;
        memset(tri, 0, sizeof(struct triangulateio)) ;
    }

    void DelaunayShewchuk::end() {
        ogf_assert(in_begin_end_) ;

        struct triangulateio in ;
        struct triangulateio out ;

        memset(&in,  0, sizeof(struct triangulateio)) ;
        memset(&out, 0, sizeof(struct triangulateio)) ;

        in.numberofpoints = vertices_.size() ;
        in.pointlist = (double*)malloc(in.numberofpoints * 2 * sizeof(double)) ;
        for(unsigned int i=0; i<vertices_.size(); i++) {
            in.pointlist[2*i  ] = vertices_[i]->point().x() ;
            in.pointlist[2*i+1] = vertices_[i]->point().y() ;
        }

        // Q: quiet
        // z: numbering starts from 0
        // n: output neighbors
        triangulate((char*)"Qzn", &in, &out, nil) ;

        std::vector<Triangulation::Facet*> facets ;

        // Step 1: create facets and initialize facet->vertex pointers
        for(int i=0; i<out.numberoftriangles; i++) {
            Triangulation::Facet* f = new_facet() ;
            for(int j=0; j<3; j++) {
                int vindex = out.trianglelist[3*i+j] ;
                ogf_parano_assert(vindex >= 0 && vindex < vertices_.size()) ;
                Triangulation::Vertex* v = vertices_[vindex] ;
                set_facet_vertex(f, j, v) ;
                set_vertex_facet(v, f) ;
            }
            facets.push_back(f) ;
        }

        // Step 2 : initialize facet->facet pointers
        for(int i=0; i<out.numberoftriangles; i++) {
            Triangulation::Facet* f1 = facets[i] ;
            for(int j=0; j<3; j++) {
                int nindex = out.neighborlist[3*i+j] ;
                Triangulation::Facet* f2 = nil ;
                if(nindex >= 0) {
                    ogf_parano_assert(nindex < out.numberoftriangles) ;
                    f2 = facets[nindex] ;
                }
                set_facet_adjacent(f1, j, f2) ;
            }
        }

        free_triangulateio(&in) ;
        free_triangulateio(&out) ;
        
        in_begin_end_ = false ;
    }
*/  

}

