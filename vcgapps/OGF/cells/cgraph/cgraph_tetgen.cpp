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

#include <OGF/cells/cgraph/cgraph_tetgen.h>
#include <OGF/cells/cgraph/cgraph_attributes.h>
#include <OGF/cells/cgraph/cgraph_builder.h>
#include <OGF/cells/map/map.h>
#include <OGF/cells/map/map_attributes.h>
#include <OGF/basic/debug/logger.h>
#include <string>
#include <sstream>

namespace OGF {
    
    CGraphTetgen::CGraphTetgen(CGraph* target) : CGraphMutator(target) {
        add_steiner_points_on_exterior_boundary_ = true ;
        add_steiner_points_on_interior_boundary_ = true ;
        tag_regions_ = false ;
        max_tet_shape_ = 2.0 ;
        max_tet_volume_ = -1.0 ;
        lock_intersections_ = false ;
    }

    void CGraphTetgen::tetrahedralize() {
        if(surface_ != nil) {
            graphite_to_tetgen() ;
        }

        FOR_EACH_HALFEDGE(Map, surface_, it) {
            if(it->is_border()) {
                Logger::err("CGraphTetgen") 
                    << "Surface was not closed, cannot stuff open surface" 
                    << std::endl ;
                return ;
            }
        }

        // Create tetgen argument string.

        std::ostringstream s ;
        // p: input data is surfacic
        // n: output tet neighbors
        // q: desired quality
        s << "pnq" << max_tet_shape_ ;
        if(max_tet_volume_ > 0.0) {
            s << "a" << max_tet_volume_ ;
        }

        // AA: generate region tags 
        // for each shell.
        if(tag_regions_) {
            s << "AA" ;
        }
        
        // YY: prohibit steiner points on boundaries
        // (first Y for exterior boundary, second Y for the
        // other ones).

        if(
            add_steiner_points_on_exterior_boundary_ &&
            !add_steiner_points_on_interior_boundary_
        ) {
            Logger::warn("CGraphTetgen") 
                << "Invalid combination of flags (do not preserve exterior boundary and preserve interior ones)"
                << " - preserving exterior boundary as well ..."
                << std::endl ;
            add_steiner_points_on_exterior_boundary_ = false ;
        }

        if(!add_steiner_points_on_exterior_boundary_) {
            s << "Y" ;
        }

        if(!add_steiner_points_on_interior_boundary_) {
            s << "Y" ;
        }

        std::string tetgen_args = s.str() ;
        try {
            ::tetrahedralize((char*)(tetgen_args.c_str()), &tetgen_surface_, &tetgen_volume_) ;
        } catch(...) {
            Logger::err("CGraphTetgen") 
                << "tetgen encountered an error, relaunching in diagnose mode" << std::endl ;
            ::tetrahedralize("d", &tetgen_surface_, &tetgen_volume_) ;

            if(lock_intersections_) {
                // Mark the vertices of the facets that intersect.
                MapVertexLock is_locked(surface_) ;
                std::set<int>& isects = tetgen_surface_.isectvertices ;
                int cur_id = 1 ;
                FOR_EACH_VERTEX(Map, surface_, it) {
                    is_locked[it] = (isects.find(cur_id) != isects.end()) ;
                    cur_id++ ;
                }
            }
        }
        if(target() != nil) {
            tetgen_to_graphite() ;
        }
    }

    void CGraphTetgen::graphite_to_tetgen() {
        tetgen_surface_.initialize() ;
        MapVertexAttribute<int> map_vertex_id(surface_) ;
        int cur_id = 1 ;
        FOR_EACH_VERTEX(Map, surface_, it) {
            map_vertex_id[it] = cur_id ;
            cur_id++ ;
        }
        tetgen_surface_.firstnumber = 1 ;
        tetgen_surface_.numberofpoints = surface_->size_of_vertices() ;
        tetgen_surface_.pointlist = new double[tetgen_surface_.numberofpoints*3] ;

        int i = 0 ;
        FOR_EACH_VERTEX(Map, surface_, it) {
            tetgen_surface_.pointlist[i] = it->point().x() ; i++ ;
            tetgen_surface_.pointlist[i] = it->point().y() ; i++ ;
            tetgen_surface_.pointlist[i] = it->point().z() ; i++ ;
        }

        tetgenio::facet* f ;
        tetgenio::polygon* p ;
        tetgen_surface_.numberoffacets = surface_->size_of_facets() ;
        tetgen_surface_.facetlist = new tetgenio::facet[tetgen_surface_.numberoffacets] ;

        i = 0 ;
        FOR_EACH_FACET(Map, surface_, it) {
            f = &(tetgen_surface_.facetlist[i]) ;
            f->numberofpolygons = 1 ;
            f->polygonlist = new tetgenio::polygon[f->numberofpolygons] ;
            p = f->polygonlist ;
            p->numberofvertices = it->degree() ;
            p->vertexlist = new int[p->numberofvertices] ;
            Map::Halfedge* h = it->halfedge() ; int j = 0 ;
            do {
                p->vertexlist[j] = map_vertex_id[h->vertex()] ;
                h = h->next() ; j++ ;
            } while (h != it->halfedge()) ;
            f->numberofholes = 0 ;
            f->holelist = nil ;
            i++ ;
        }
    }
    
    void CGraphTetgen::tetgen_to_graphite() {
        CGraphCellAttribute<double> region ;
        if(tag_regions_) {
            region.bind(target(), "region") ;
        }

        if(target()->size_of_meta_cells() == 0) {
            CGraphBuilder builder ;
            builder.set_target(target()) ;
            builder.begin_volume() ;
            builder.build_meta_tetrahedron() ;
            builder.end_volume() ;
        }

        // Create vertices
        std::vector<CGraph::Vertex*> vertices ;
        double* p = tetgen_volume_.pointlist ;
        for(int i=0; i<tetgen_volume_.numberofpoints; i++) {
            vertices.push_back(new_vertex(Point3d(p[0], p[1], p[2]))) ;
            p += 3 ;
        }

        // Create cells
        std::vector<CGraph::Cell*> cells ;
        CGraph::MetaCell* meta_tet = target()->meta_cell(0) ;
        int* t = tetgen_volume_.tetrahedronlist ;
        for(int i=0; i<tetgen_volume_.numberoftetrahedra; i++) {
            CGraph::Cell* c = new_cell(meta_tet) ;
            for(int j=0; j<4; j++) {
                set_cell_vertex(c, j, vertices[ t[j] - tetgen_volume_.firstnumber ] ) ;
            }
            if(tag_regions_) {
                region[c] = tetgen_volume_.tetrahedronattributelist[i] ;
            }
            cells.push_back(c) ;
            t += 4 ;
        }

        // Create adjacency
        int* pn = tetgen_volume_.neighborlist ;
        for(int i=0; i<tetgen_volume_.numberoftetrahedra; i++) {
            for(int j=0; j<4; j++) {
                int adjacent = pn[j] ;
                if(adjacent >= 0) {
                    set_cell_adjacent(
                        cells[i],
                        j,
                        cells[adjacent - tetgen_volume_.firstnumber]
                    ) ;
                }
            }
            pn += 4 ;
        }

    }


//--------------------------------------------------------------------------------------------------


}
