/*
 *  GXML/Graphite: Geometry and Graphics Programming Library + Utilities
 *  Copyright (C) 2000 Bruno Levy
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
 *  Contact: Bruno Levy
 *
 *     levy@loria.fr
 *
 *     ISA Project
 *     LORIA, INRIA Lorraine, 
 *     Campus Scientifique, BP 239
 *     54506 VANDOEUVRE LES NANCY CEDEX 
 *     FRANCE
 *
 *  Note that the GNU General Public License does not permit incorporating
 *  the Software into proprietary programs. 
 */
 

#include <OGF/cells/map_algos/hierarchical_map_parameterizer.h>
#include <OGF/cells/map_algos/map_parameterizer.h>
#include <OGF/cells/map_algos/pm_manager.h>
#include <OGF/cells/map_algos/floater.h>
#include <OGF/cells/map/map.h>
#include <OGF/cells/map/map_editor.h>
#include <OGF/cells/map/geometry.h>
#include <OGF/math/geometry/polygon2d.h>

#include <algorithm>

#include <stdlib.h>

namespace OGF {

//_________________________________________________________

    HierarchicalMapParameterizer::HierarchicalMapParameterizer(
    ) {
        map_ = nil ;
        pm_manager_ = nil ;
        reduction_factor_ = 10.0 ;
        strict_kernels_ = true ;
        triangles_only_ = true ;
        lock_borders_ = true ;
        hierarchical_ = true ;
    }
    
    bool HierarchicalMapParameterizer::do_parameterize_disc(Map* map) {

/*
        std::cerr << "do_parameterize_disc: " << map << std::endl ;
        {
            std::cerr << "HierarchicalMapParameterizer::do_parameterize_disc - checking Map locked vertices on border" << std::endl ;
            MapVertexLock is_locked(map) ;
            FOR_EACH_VERTEX(Map, map, it) {
                if(is_locked[it] && it->is_on_border()) {
                    std::cerr << "============ locked vertex on border" << std::endl ;
                }
            }
        }
*/

        map_ = map ;

        if(hierarchical_) {
            pm_manager_ = new PMManager(map) ;
            pm_manager_->set_decimate_border(!lock_borders_) ;
            // [Bruno] pm_manager_->set_decim_proportion(1.0) ;
            Logger::out("HierarchicalMapParameterizer") 
                << "creating progressive mesh" << std::endl ;
            pm_manager_->init() ;
            Logger::out("HierarchicalMapParameterizer") 
                << "progressive mesh created" << std::endl ;
        } else {
            pm_manager_ = nil ;
        }
        

        optimize() ;

        delete pm_manager_ ;
        pm_manager_ = nil ;
        map_ = nil ;  
        return true ;
    }

    void HierarchicalMapParameterizer::optimize() {
        int nb_vertices = map_->size_of_vertices();
        VertexSplit res;
        if((pm_manager_ == nil) || (pm_manager_->next_coarsening().new_block() == nil)) {
            std::cerr << "Optimize: non-hierarchical mode" << std::endl ;
            compute_initial_parameterization() ;
            optimize_parameterization(500) ;
            update_graphics(map_) ; // just for fun
        } else {
            while (
                pm_manager_->next_coarsening().new_block() != nil
            ) {
                res = pm_manager_->coarsen();
            }


            int nb_vertices_removed = nb_vertices - map_->size_of_vertices() ;
            
            //______________________________________________


            compute_initial_parameterization() ;
            update_graphics(map_) ; // just for fun !
            std::cerr << "computed initial parameterization" << std::endl ;

            //______________________________________________

            double proportion = 0.0 ;
            int init_iteration = 30 ;
            bool terminate = false ;


            // start the hierarchical optimization
            do {
                res = pm_manager_->next_refinement();
                res = pm_manager_->refine();

                Point2d result ;
                terminate = get_valid_tex_coord(res, result) ;
                if(terminate) {
                    MapVertexLock is_locked(map_) ;
                    is_locked[res.new_block()->vertex()] = true ;
                    
                }
                res.new_block()->prev()->set_tex_coord(result);

                if( 
                    (1.0 - ((double)(nb_vertices - map_->size_of_vertices()) /
                        (double)nb_vertices_removed) ) >= proportion 
                ) {
                    double temp = 1.0 - proportion ;
                    // configure your iteration scheme here
                    int num = int(temp * (double)init_iteration) ;
                    optimize_parameterization( num ) ;
                    update_graphics(map_) ; // just for fun 
                    proportion += 0.05 ;
                }
                if(map_->size_of_vertices() == nb_vertices){
                    optimize_parameterization(20) ;
                    update_graphics(map_) ; // just for fun
                }
            } while (
                res.new_block() != nil &&
                nb_vertices != map_->size_of_vertices() &&
                !terminate
            ) ;
        } 
    }

    Point2d HierarchicalMapParameterizer::kernel_barycenter(
        const Polygon2d& neigh, bool& terminate
    ) {
        Polygon2d kernel ;
        Geom::kernel(neigh, kernel) ;
        if(kernel.size() == 0) {
            std::cerr << "Empty kernel" << std::endl ;
            Geom::save_polygon(neigh, "P.obj") ;
            terminate = true ;
            return neigh[0] ;
        }
        Point2d result =  Geom::barycenter(kernel) ;
        terminate = false ;
        return result ;
    }


    bool HierarchicalMapParameterizer::get_valid_tex_coord(
        VertexSplit& split, Point2d& vt
    ) {
        // If strict kernels, return the barycenter of
        // the neighbor's kernel.
        if(strict_kernels_) {
            Map::Halfedge* halfedge_target = split.new_block()->opposite() ;
            Polygon2d neighbors;
            Map::Halfedge* hir = halfedge_target;
            do {    
                neighbors.push_back(hir->opposite()->tex_coord());
                hir = hir->next_around_vertex() ;
            } while ( hir != halfedge_target);
            
            ogf_assert(neighbors.size() >= 3) ;
            if(neighbors.size() == 3){
                if(split.right_position() == nil) { 
                    // new_block->opposite is a border
                    vt = Geom::barycenter(neighbors[0], neighbors[2]) ;
                    return false ;
                } else {
                    bool terminate ;
                    vt = kernel_barycenter(neighbors, terminate) ;
                    return terminate ;
                }
            } else {
                bool terminate ;
                vt = kernel_barycenter(neighbors, terminate) ;
                return terminate ;
            }
        } else {
            // If no strict kernel, return the barycenter of the neighbors
            double x = 0.0 ;
            double y = 0.0 ;
            int count = 0 ;
            Map::Halfedge* halfedge_target = split.new_block()->opposite() ;
            Map::Halfedge* hir = halfedge_target;
            do {    
                const Point2d& p = hir->opposite()->tex_coord() ;
                x += p.x() ;
                y += p.y() ;
                count++ ;
                hir = hir->next_around_vertex() ;
            } while ( hir != halfedge_target);
            vt = Point2d(x/double(count), y/double(count)) ;
            return false ;
        }
    }

//_________________________________________________________

    bool HierarchicalMapParameterizer::vertex_is_in_kernel(Map::Vertex* v) {
        if(!lock_borders_ && v->is_on_border()) {
            return true ;
        }
        ogf_assert(!v->is_on_border()) ;
        Polygon2d P ;
        Map::Halfedge* h = v->halfedge() ;
        const Point2d& p = h->tex_coord() ;
        do {
            P.push_back(h->opposite()->tex_coord()) ;
            h = h->next_around_vertex() ;
        } while(h != v->halfedge()) ; 
        return Geom::point_is_in_kernel(P,p) ;
    }

    bool HierarchicalMapParameterizer::check_kernels() {
        bool result = true ;
        FOR_EACH_VERTEX(Map, map_, it) {
            if(it->is_on_border()) {
                continue ;
            }
            if(!vertex_is_in_kernel(it)) {
                std::cerr << "Point is not in kernel" << std::endl ;
                result = false ;
            }
        }
        if(!result) {
            std::cerr << "Found some invalid kernels" << std::endl ;
        }
        return result ;
    }

//__________________________________________________________________________

    RandomDescentMapParameterizer::RandomDescentMapParameterizer() {
    }

    bool RandomDescentMapParameterizer::do_parameterize_disc(Map* disc) {
        facet_area_.bind(disc) ;
        bool result = HierarchicalMapParameterizer::do_parameterize_disc(disc) ;
        facet_area_.unbind() ;
        return result ;
    }


    
    void RandomDescentMapParameterizer::optimize_parameterization(int nb_iter) {

        Logger::out("HierarchicalParameterizer") 
            << "optimize criterion, nb_iter=" << nb_iter << std::endl ;

        std::vector<OptimVertex> sorted_vertices ;

        {FOR_EACH_VERTEX(Map, map_, it) {
            if(lock_borders_ && it->is_on_border()) {
                continue ;
            }
            sorted_vertices.push_back(OptimVertex(it,vertex_criterion(it))) ;
        }}

        {FOR_EACH_FACET(Map, map_, it) {
            facet_area_[it] = Geom::facet_area(it) ; 
        }}

        for(int iter=0; iter<nb_iter; iter++) {

            std::sort(
                sorted_vertices.begin(),sorted_vertices.end(), OptimVertexCompareGT()
            ) ;
            
            {for(unsigned int i=0; i<sorted_vertices.size(); i++) {
                optimize_vertex(sorted_vertices[i]) ;
            }}
        }

        update_graphics(map_) ; // just for fun
    }
    
    double RandomDescentMapParameterizer::vertex_criterion(Map::Vertex* v) {
        if(strict_kernels_ && !vertex_is_in_kernel(v)) {
            return 1e30 ;
        }
        double result = 0 ;
        Map::Halfedge* h = v->halfedge() ;
        do {
            if(h->facet() != nil) {
                result += facet_criterion(h->facet()) ;
            }
            h = h->next_around_vertex() ;
        } while(h != v->halfedge()) ;
        return result ;
    }

    double RandomDescentMapParameterizer::facet_criterion(Map::Facet* f) {
        return 0.0 ;
    }

    void RandomDescentMapParameterizer::optimize_vertex(OptimVertex& o) {
        Map::Vertex* v = o.vertex ;
        if(lock_borders_ && v->is_on_border()) {
            return ;
        }
        Vector2d dir = trust_region_radius(v) * random_direction() ;
        linear_search(o, dir) ;
    }    
    
    void RandomDescentMapParameterizer::linear_search(OptimVertex& o, const Vector2d& dir) {
        Map::Vertex* v = o.vertex ;
        if(lock_borders_ && v->is_on_border()) {
            return ;
        }
        Map::Halfedge* h = v->halfedge() ;
        double orig_criterion = vertex_criterion(v) ;

        // Shit happens [Forrest Gump]
        // Note: this can happen if an edge connects two vertices
        // on the same side of the square.
        if(Numeric::is_nan(orig_criterion)) {
            return ;
        }

        Point2d orig_pos = h->tex_coord() ;
        
        double low  = 0 ; 
        double criterion_low = orig_criterion ;
        double high = 1 ;
        h->set_tex_coord(orig_pos + dir) ; 
        double criterion_high = vertex_criterion(v) ;
        double criterion_m = criterion_high ;
        
        int i = 0 ;
        while((criterion_high > 1e10 || i < 5) && i < 20) {
            i++ ;
            double m = 0.5 * (low + high) ;
            h->set_tex_coord(orig_pos + m * dir) ;
            criterion_m = vertex_criterion(v) ;
            if(criterion_low < criterion_high) {
                high = m ;
                criterion_high = criterion_m ;
            } else {
                low = m ;
                criterion_low = criterion_m ;
            }
        }

        if(criterion_m > orig_criterion) {
            o.criterion = criterion_m ;
            h->set_tex_coord(orig_pos) ;
        } else {
            o.criterion = orig_criterion ;
        }
    }
    
    double RandomDescentMapParameterizer::trust_region_radius(Map::Vertex* v) {
        double result = 0 ;
        int degree = 0 ;
        Map::Halfedge* h = v->halfedge() ;
        const Point2d& p = h->tex_coord() ;
        do {
            const Point2d& p2 = h->opposite()->tex_coord() ;
            result += (p2 - p).norm() ;
            degree++ ;
            h = h->next_around_vertex() ;
        } while(h != v->halfedge()) ;
        result /= double(2*degree) ;
        return result ;
    }
    
    Vector2d RandomDescentMapParameterizer::random_direction() {
        double alpha = Numeric::random_float64() * 2.0 * M_PI ;
        return Vector2d(::cos(alpha), ::sin(alpha)) ;
    }
    
//__________________________________________________________________________
    
}
