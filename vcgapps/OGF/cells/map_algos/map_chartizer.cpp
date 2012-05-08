/*
 *  OGF/Graphite: Geometry and Graphics Programming Library + Utilities
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
 *
 * This chartizer has been implemented by Bruno Vallet 05/06/2003.
 * It is an implementation in Graphite of H. Hoppe's MCGIM with a few improvements.
 *
 */
 

#include <OGF/cells/map_algos/map_chartizer.h>
#include <OGF/cells/map_algos/map_components.h>
#include <OGF/cells/map_algos/map_topology.h>
#include <OGF/cells/map/geometry.h>
#include <OGF/cells/map/map_editor.h>
#include <OGF/math/geometry/principal_axes.h>
#include <OGF/basic/debug/logger.h>
#include <OGF/basic/debug/progress.h>

#include <deque>
#include <stack>
#include <queue>

namespace OGF {

    /*-+-* Public *-+-*/
	
    // constructor
    MapChartizer::MapChartizer(Map* map
    ) : map_(map),

        // Tuning (defaults)
        num_charts_(5),
        geom_dist_(true),
        cos_(true),
        angle_(false),
        curv_(false),
        epsilon_(1.0),

        // Binding attributes
        // chart index now bound in apply()
        cost_(map),
        dist_(map),
        is_set_(map),
        is_old_center_(map),

        // Priority queue
        eval_(dist_),
        S_(eval_) {
    }

    // main
    void MapChartizer::apply() {

        if(chart_index_name_.length() != 0) {
            chart_index_.bind(map_, chart_index_name_) ;
        } else {
            chart_index_.bind(map_) ;
        }

        if(curv_) {
            if(
                k1_.is_defined(map_, "k1") &&
                k2_.is_defined(map_, "k2") 
            ) {
                k1_.bind(map_, "k1") ;
                k2_.bind(map_, "k2") ;
                Logger::out("Charts") << "using stored curvatures" << std::endl ;
            } else {
                Logger::out("Charts") << "using angular default" << std::endl ;
            }
        }

        bool continue_condition = false ;
        Map::Facet* furthest_facet = nil ;

        // initialise is_old_center 
        FOR_EACH_FACET(Map, map_, it) is_old_center_[it] = false ;

        compute_cost() ;
        // we choose to add 1 chart to each new connex
        num_charts_ += find_connex() - 1 ;

        ProgressLogger progress(num_charts_) ;

        Logger::out("Charts") << "Charting on " << chart_.size()
                              << " connex component" 
                              << (chart_.size() > 1 ? "s..." : "...") << std::endl ;

        do {
            Logger::out("Charts") << "Iterating with " << chart_.size()
                                  << " chart" 
                                  << (chart_.size() > 1 ? "s." : ".") << std::endl ;

            continue_condition = false ;
            // grows charts from chart centers and returns furthest facet
            reset_span() ;
            furthest_facet = grow_charts() ;

            compute_borders() ;

            // compute new charts centers
            reset_span() ;
            for(unsigned int i = 0; i < chart_.size(); i++)
                {
                    chart_[i]->center = center(i) ;
                    if(!is_old_center_[chart_[i]->center]) {
                        continue_condition = true ;
                    }
                    is_old_center_[chart_[i]->center] = true ;
                }
            // if the wanted number of charts is not reached, add furthest facet
            if(int(chart_.size()) < num_charts_) {
                continue_condition = true ;
                create_chart(furthest_facet) ;
            }

            if(progress.is_canceled()) {
                return ;
            }

            update_graphics(map_) ;
            progress.next() ;

        } while(continue_condition) ;

        Logger::out("Charts") << "...charted : " << chart_.size()
                              << " chart" << (chart_.size() > 1 ? "s " : " ") 
                              << "generated." << std::endl ;
    }

    // get the vector of border halfedges after calling apply
    std::vector<Map::Halfedge*>& MapChartizer::get_border() {
        global_border_.clear() ;
        for(unsigned int i = 0; i < chart_.size(); i++)
            for(unsigned int j = 0; j < chart_[i]->border.size(); j++)
                global_border_.push_back(chart_[i]->border[j]) ;
        return global_border_ ;
    }

    // Authalic border
    std::vector<Map::Halfedge*>& MapChartizer::get_authalic_border() {
        MapVertexAttribute<double> rigid(map_, "rigid") ;
        std::deque<Map::Halfedge*> Aborder ;
        global_border_.clear() ;
        // init is_set
        int nf = 0 ;
        {FOR_EACH_FACET(Map, map_, it) {
            is_set_[it] = false ;
            nf++ ;
        }}
        {FOR_EACH_VERTEX(Map, map_, it) {
            rigid[it] = false ;
        }}
        // find border
        {FOR_EACH_HALFEDGE(Map, map_, it) {
            if(it->is_border()) {
                Aborder.push_back(it) ;
            }
        }}
        int neutral_steps = 0 ;
        // iterator to find removable blocs
        Map::Halfedge* current_border = nil ;
        // other border edge of removable bloc
        Map::Halfedge* next_border = nil ;
        // main loop
        do{
            do {
                current_border = Aborder.front() ;
                Aborder.pop_front() ;
                if(is_Aborder(current_border)) {
                    next_border = current_border->next_around_vertex() ;
                    // if we found a removable triangle
                    if(is_Aborder(next_border->opposite())) {
                        is_set_[next_border->facet()] = true ;
                        Aborder.push_back(next_border->prev()) ;
                        global_border_.push_back(next_border->prev()) ;
                        neutral_steps = 0 ;
                        nf-- ;
                    }
                    // if we found a removable bloc
                    else if(
                        is_Aborder(next_border->next_around_vertex()->opposite())
                    ) {
                        next_border = next_border->next_around_vertex() ;
                        is_set_[current_border->opposite()->facet()] = true ;
                        is_set_[next_border->facet()] = true ;
                        Aborder.push_back(current_border->opposite()->next()) ;
                        Aborder.push_back(next_border->prev()) ;
                        global_border_.push_back(
                            current_border->opposite()->next()
                        ) ;
                        global_border_.push_back(next_border->prev()) ;
                        neutral_steps = 0 ;
                        nf-=2 ;
                    }
                    else {
                        Aborder.push_back(current_border) ;
                        Logger::out("Aborder")
                            << "Neutral steps: " << neutral_steps
                            << "  Aborder.size(): " << Aborder.size() <<std::endl ;
                    }
                }
                neutral_steps++ ;
            } while(neutral_steps < 2 * int(Aborder.size()) ) ;
            int Asize = Aborder.size() ;
            do{
                current_border = Aborder.front() ;
                Aborder.pop_front() ;
                Asize-- ;
                rigid[current_border->vertex()] = 1.0 ;
                rigid[current_border->next()->vertex()] = 1.0 ;
                rigid[current_border->prev()->vertex()] = 1.0 ;
                if(is_Aborder(current_border)) {
                    next_border = 
                        current_border->next_around_vertex()->next_around_vertex() ;
                    is_set_[next_border->facet()] = true ;
                    nf-- ;
                    do {
                        Aborder.push_back(next_border->prev()) ;
                        global_border_.push_back(next_border->prev()) ;
                        is_set_[next_border->facet()] = true ;
                        nf-- ;
                        next_border = next_border->next_around_vertex() ;
                    } while(!is_Aborder(next_border->opposite())) ;
                }
            } while(Asize > 0) ;
        } while(nf > 0) ;
        return global_border_ ;
    }

    /*-+-* Initializers *-+-*/

    // cost of passing from a triangle to an adjacent one
    // stored as an attribute of the edge shared by these triangles
    void MapChartizer::compute_cost() {
        // compute costs
        FOR_EACH_EDGE(Map, map_, it) if(!it->is_border_edge()) {
            double dist = 1.0 ;
            Map::Halfedge* op = it->opposite();
            Map::Halfedge* h = it->next() ;
            if(geom_dist_) {
                Vector3d V(0,0,0), Vop(0,0,0), Vi = Geom::vector(it) ;
                // sets cost to the distance between the centers of gravity
                // G and G' of facets F=it->facet() and F'=op->facet()
                // Method: we call P and P' the origins of it and op
                // Vi and V'j the other vertices in facets F and F'
                // PG = PP + PP' + sum(PVi)
                // PG' = PP + PP' + sum(PV'i)
                // so V = G'G = sum(PVi) - sum(PV'i)
                // we recursively calculate Vi as PVi-1 + Vi-1Vi
                int num_edge = 2 ;
                do{
                    Vi = Vi + Geom::vector(h) ;
                    V = V + Vi ;
                    num_edge++ ;
                    h = h->next() ;
                } while(h != it->prev()) ;
                Vi = Geom::vector(op) ;
                h = op->next() ;
                int num_edge_op = 2 ;
                do{
                    Vi = Vi + Geom::vector(h) ;
                    Vop = Vop + Vi ;
                    num_edge_op++ ;
                    h = h->next() ;
                } while(h != op->prev()) ;
                dist = ((1.0/num_edge) * V - (1.0/num_edge_op) * Vop).norm() ;
            }
            cost_[cf_edge(op)] = dist ;

            // multiplies cost by curvature or angle or 1 - cos(angle)
            double curv = 1.0 ;
            if(curv_) {
                if(k1_.is_bound() && k2_.is_bound()) {
                    // Integral of the absoluve value of the total curvature
                    double total_curv = k1_[h->vertex()] * k2_[op->vertex()] ;
                    curv = ::sqrt(::fabs(total_curv)) * (
                        Geom::facet_area(h->facet()) +
                        Geom::facet_area(op->facet())
                    ) ;
                } else {
                    h = it ;
                    curv = 0 ;
                    do {
                        curv += Geom::angle(
                            Geom::vector(h), Geom::vector(h->next_around_vertex())
                        ) ;
                        h = h->next_around_vertex() ;
                    } while(h != it) ;
                    h = op ;
                    do {
                        curv += Geom::angle(
                            Geom::vector(h), Geom::vector(h->next_around_vertex())
                        ) ;
                        h = h->next_around_vertex() ;
                    } while(h != op) ;
                    curv -= ::fabs(curv - 4 * M_PI) ;
                }

            }
            if(angle_ || cos_) {
                double cos = Geom::cos_angle(
                    Geom::facet_normal(it->facet()), Geom::facet_normal(op->facet())
                ) ;
                if(angle_) curv *= ::acos(cos) ;
                if(cos_) curv *= 1.0 - cos ;
            }
            if(curv < epsilon_) curv = epsilon_ ;
            cost_[cg_edge(it)] = dist * curv ;
        }
    }

    // resets the attributes used by the spanning
    void MapChartizer::reset_span() {
        // clear spanning attributes
        FOR_EACH_FACET(Map, map_, it) {
            dist_[it] = Numeric::big_double ;
            is_set_[it] = false ;
        }
    }

    // inits of chart creation
    void MapChartizer::create_chart(Map::Facet* f) {
        chart_.push_back(new Chart) ;
        chart_[chart_.size() - 1]->center = f ;
        is_old_center_[f] = true ;
    }

    /*-+-* Connex components finder *-+-*/

    //Main
    int MapChartizer::build_connex() {
        int ret ;
        MapVertexAttribute<double> connex(map_, "connex") ;
        ret = find_connex() ;
        FOR_EACH_VERTEX(Map, map_, it) if(!it->halfedge()->is_border())
            connex[it] = (double)chart_index_[it->halfedge()->facet()] ;
        return ret ;
    }

    int MapChartizer::find_connex() {
        int num_connex = 0 ;
			
        // put each facet in chart -1 = no chart
        FOR_EACH_FACET(Map, map_, it) chart_index_[it] = -1 ;
        while(true) {
            // take first facet in no chart
            Map::Facet* connex_center = new_connex_center() ;
            if(connex_center == nil) return num_connex;
            num_connex++ ;
            create_chart(connex_center) ;

            // span from this facet to paint all the connex
            paint_connex(connex_center, chart_.size() - 1) ;
        }
        return 0 ; // for compiler
    }

    // span the chart number on all the connex
    void MapChartizer::paint_connex(Map::Facet* f, int i) {
        std::queue<Map::Facet*> to_paint ;
        chart_index_[f] = i;
        to_paint.push(f) ;
        while(!to_paint.empty()) {
            Map::Facet* top_facet = to_paint.front() ;
            to_paint.pop() ;
            Map::Halfedge* h = top_facet->halfedge() ;
            do {
                if(
                    !h->is_border_edge() && 
                    chart_index_[h->opposite()->facet()] < 0
                ) {
                    chart_index_[h->opposite()->facet()] = i;
                    to_paint.push(h->opposite()->facet()) ;
                }
                h = h->next() ;
            } while(h != top_facet->halfedge()) ;
        }
    }

    // find a new facet in no chart
    Map::Facet* MapChartizer::new_connex_center() {
        FOR_EACH_FACET(Map, map_, it) if(chart_index_[it] == -1) return it ;
        return nil ;
    }
		
    /*-+-* Utils *-+-*/

    // finds the border using chart indices
    void MapChartizer::compute_borders() {
        // clear borders
        for(unsigned int i = 0; i < chart_.size(); i++)
            chart_[i]->border.clear() ;

        // compute new borders from chart indices
        FOR_EACH_EDGE(Map, map_, it) {
            if( !it->is_border_edge() &&
                chart_index_[it->facet()] >= 0 &&
                chart_index_[it->facet()] != chart_index_[it->opposite()->facet()] 
            ) {
                chart_[chart_index_[it->facet()]]->border.push_back(it) ;
                chart_[chart_index_[it->opposite()->facet()]]->border.push_back(
                    it->opposite()
                ) ;
            }
        }
    }

    // span charts with the pre-computed cost function and returns last added facet
    Map::Facet* MapChartizer::grow_charts() {
        // Initialisation
        Map::Facet* top_facet = nil ;

        // add seeds = chart centers
        for(unsigned int i = 0; i < chart_.size(); i++) {
            dist_[chart_[i]->center] = 0 ;
            chart_index_[chart_[i]->center] = i ;
            set(chart_[i]->center) ;
        } ;

        // span from seeds
		while(!S_.empty()) {
			/*we use the fact that multisets are sorted to get the top element
			  and to 'pop' it.*/
			top_facet = *(S_.begin()) ;
			S_.erase(S_.begin()) ;
			if(!is_set_[top_facet]) set(top_facet) ;
        }
			
        // return last added = furthest facet
        return top_facet ;
    }

    // computes the center of chart i
    Map::Facet* MapChartizer::center(int i) {
        Map::Facet* top_facet = chart_[i]->center ;
        // for closed objects (with no border), just take the same center as before
        if(chart_[i]->border.size() == 0) return top_facet ;

        // adds seeds = border facets
        for(unsigned int j = 0; j < chart_[i]->border.size(); j++) {
            Map::Facet* f = chart_[i]->border[j]->facet() ;
            dist_[f] = 0.0 ;
            set(f, i) ;
        }
        while(!S_.empty()) {
			/*we use the fact that multisets are sorted to get the top element
			  and to 'pop' it.*/
			top_facet = *(S_.begin()) ;
			S_.erase(S_.begin()) ;
            if(!is_set_[top_facet]) set(top_facet, i) ;
        }
        // return last added = center facet
        return top_facet ;
    }

    // sets f and updates distances and chart indices for f's neighboors
    void MapChartizer::set(Map::Facet* f) {
        double next_dist;
        is_set_[f] = true ;
        /*Logger::out("Charts") << "Setting facet at dist. " << dist_[f]
          << " from center " << chart_index_[f] << std::endl ;*/

        // update neighboors' distance to seed
        Map::Halfedge* h = f->halfedge() ;
        do {
            if (!h->is_border_edge()) {
                Map::Facet* next_facet = h->opposite()->facet() ;
                next_dist = dist_[f] + cost_[cg_edge(h)] ;
                if(dist_[next_facet] > next_dist) {
					// If the element previousely existed, we remove it
					// before updating his distance and reinserting it.
					// This prevents corruption of the order of the elements.
					S_.erase(next_facet);
					dist_[next_facet] = next_dist ;
                    chart_index_[next_facet] = chart_index_[f] ;
					S_.insert(next_facet);
                }
            }
            h = h->next();
        } while(h != f->halfedge()) ;
    }

    // sets f and updates distances only for neighboors in chart i
    void MapChartizer::set(Map::Facet* f, int i) {
        // security
        ogf_assert(chart_index_[f] == i) ;

        double next_dist ;
        is_set_[f] = true ;

        // update neighboors' distance to seed
        Map::Halfedge* h = f->halfedge() ;
        do {
            if (!h->is_border_edge()) {
                Map::Facet* next_facet = h->opposite()->facet() ;
                next_dist = dist_[f] + cost_[cf_edge(h)] ;
                if(chart_index_[next_facet] == i && dist_[next_facet] > next_dist) {
                    // If the element previousely existed, we remove it
					// before updating his distance and reinserting it.
					// This prevents corruption of the order of the elements.
					S_.erase(next_facet);
					dist_[next_facet] = next_dist ;
                    S_.insert(next_facet);
                }
            }
            h = h->next();
        } while(h != f->halfedge()) ;
    }		

    //_________________________________________________________

}

