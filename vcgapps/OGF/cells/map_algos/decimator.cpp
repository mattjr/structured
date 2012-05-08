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
 */

#include <OGF/cells/map_algos/decimator.h>
#include <OGF/cells/map/geometry.h>
#include <OGF/basic/debug/progress.h>

namespace OGF{
    
    Decimator::Decimator(Map* s) {
        map_ = s;
        nb_vertex_to_remove_ = int((map_->size_of_vertices()*50.0)/100.0); 
        threshold_ = 1;
        set_strategy(VOLUME_AND_BORDER_BASED);
        progress_ = nil ;
    }
    
    void Decimator::set_proportion_to_remove(double d) {
        nb_vertex_to_remove_ = int(double (map_->size_of_vertices())*d) ;
    }
    
    void Decimator::set_threshold(double threshold){
        threshold_ = threshold;
    }
    
    Decimator::~Decimator() {
    }

    void Decimator::collapse_edge(Map::Halfedge* h){
        MapEditor editor(map_);
        editor.collapse_edge(h);
    }

    void Decimator::apply(){

        FOR_EACH_FACET(Map,map_,ti) {
            ogf_assert(ti->is_triangle()) ;
        }

        importance_.bind(map_-> vertex_attribute_manager());
        accumulated_cost_.bind(map_-> vertex_attribute_manager());
        halfedge_to_collapse_.bind(map_-> vertex_attribute_manager());
        vertex_density_.bind_if_defined(map_->vertex_attribute_manager(), "density")  ;

        FOR_EACH_VERTEX(Map,map_,vi) {
            compute_vertex_importance(vi);
            accumulated_cost_[vi]=0;
        }
        
        MapEditor editor(map_);
        
        MapVertexHeap heap(
            map_,importance_
        );

        heap.init_with_all_surface_vertices();
        
        int nb_vertex_to_remove_init = nb_vertex_to_remove_ ;

        int last_val = 0 ;
        while (!heap.empty() && (nb_vertex_to_remove_ > 0)) {
			Map::Vertex*  top = heap.pop();
            if(progress_ != nil) {
                int val = (nb_vertex_to_remove_init - nb_vertex_to_remove_) ;
                val *= 20 ;
                val /= nb_vertex_to_remove_init ;
                if(val != last_val) {
                    progress_->notify(val) ;
                    last_val = val ;
                }
            }

            if (
                halfedge_to_collapse_[top] != nil &&
                can_collapse(halfedge_to_collapse_[top]) 
            ) {
                nb_vertex_to_remove_--;
                std::vector<Map::Vertex*> neighbors;
                Map::Halfedge* cir = top->halfedge();
                do {
                    neighbors.push_back(cir->opposite()->vertex());
                    cir = cir->next_around_vertex() ;
                } while ( cir != top->halfedge());
                
                ogf_assert(
                    halfedge_to_collapse_[top]->opposite()->vertex() == top
                );
                
                Map::Vertex* v = halfedge_to_collapse_[top]->vertex();
                accumulated_cost_[v] +=compute_vertex_importance(top);
                
/*
//  TODO: check if we can remove that (but it seems to be used nowhere)
                halfedge_to_collapse_[*iter]->vertex()->set_value(
                    halfedge_to_collapse_[*iter]->vertex()->value() + 
                    halfedge_to_collapse_[*iter]->opposite()->vertex()->value()
                    -0.5
                ) ;
*/	  
                ogf_assert(can_collapse(halfedge_to_collapse_[top]));
                collapse_edge(halfedge_to_collapse_[top]);
                
                for (unsigned int i=0; i<neighbors.size();i++){
                    compute_vertex_importance(neighbors[i]);
                    if (heap.contains(neighbors[i])) {
                        heap.update_cost(neighbors[i]);
                    } else {
                        if (importance_[neighbors[i]] < threshold_)
                            heap.push(neighbors[i]);
                    }
                }
            } 
        }
    }
    

    double Decimator::delta_volume_cause_by_collapse(Map::Halfedge* h){
        double delta_vol=0; 
        Map::Halfedge* cir = h->opposite();
        do {
            if (
                cir !=  h->opposite() && cir != h->prev() && !cir->is_border()
            ) {
                Vector3d v0 = 
                    cir->next()->vertex()->point() - h->vertex()->point();
                Vector3d v1 = 
                    cir->opposite()->vertex()->point() - h->vertex()->point();
                Vector3d v2 = Geom::vector(h->opposite());
                
                delta_vol += ::fabs((v0^v1) * v2);
            }	
            cir = cir->next_around_vertex() ;
        } while (cir != h->opposite());
        return delta_vol;
    }
    
    double Decimator::delta_area_on_border_cause_by_collapse(
        Map::Halfedge* h
    ){
        if (h->is_border())
            return (
                Geom::vector(h) ^
                Geom::vector(h->prev()->opposite())
            ).norm();
        else if (h->opposite()->is_border())
            return (
                Geom::vector(h->opposite()->next()) ^
                Geom::vector(h->opposite())
            ).norm();
        return 0;  // h may be a non-border halfedge.
    }
    
    
    double Decimator::delta_shape_cause_by_collapse(Map::Halfedge* h) {
        double delta_vol=0; 
        double area = 0;
        Map::Halfedge* cir = h->opposite();
        do{
            if (cir !=  h->opposite() && cir != h->prev() && !cir->is_border()
            ) {
                Vector3d v0 =
                    cir->next()->vertex()->point() - h->vertex()->point();
                Vector3d v1 =
                    cir->opposite()->vertex()->point() - h->vertex()->point();
                Vector3d v2 = Geom::vector(h->opposite());
                
                area += (v0^v1).norm() / 2.0;
                delta_vol += ::fabs((v0^v1) * v2);
            }	
            cir = cir->next_around_vertex();
        } while (cir != h->opposite());
        return  delta_vol / ::pow(area,1.5);
    }
    
    double sigma_angles(Map::Vertex* v) {
        Map::Halfedge* cir = v->halfedge();
        double angle=0;
        do {
            Vector3d v0 = Geom::vector(cir->opposite()); v0.normalize();
            Vector3d v1 = Geom::vector(cir->next());     v1.normalize();
            angle += ::acos(v0*v1);
            cir = cir->next_around_vertex();
        } while (cir !=  v->halfedge());
        return angle;
    }
    
    double Decimator::delta_angular_default_cause_by_collapse(
        Map::Halfedge* h
    ) {
        Map::Halfedge* cir = h->opposite();
        double angle=0;
        do {
            Map::Vertex* v=cir->opposite()->vertex();
            double original_angle=sigma_angles(v);
            if (v!=h->vertex()){
                original_angle -= Geom::angle(
                    Geom::vector(cir), Geom::vector(cir->prev()->opposite()
                    )
                ) ;
                original_angle -= Geom::angle(
                    Geom::vector(cir), Geom::vector(cir->opposite()->next())
                ) ;
                Vector3d v = 
                    h->vertex()->point() - cir->opposite()->vertex()->point();
                original_angle += Geom::angle(
                    v, Geom::vector(cir->prev()->opposite())
                ) ;
                original_angle += Geom::angle(
                    v, Geom::vector(cir->opposite()->next())
                ) ;
            } else {
                Map::Halfedge* innercir = h->opposite();
                original_angle -= Geom::angle(
                    Geom::vector(h->opposite()), Geom::vector(h->next())
                ) ;
                original_angle -= Geom::angle(
                    Geom::vector(h->opposite()),
                    Geom::vector(h->opposite()->prev()->opposite())
                ) ;
                
                do {
                    if (innercir!=h->opposite()  && innercir->next()!=h)
                        original_angle += 
                            Geom::angle(
                                innercir->opposite()->vertex()->point() -
                                h->vertex()->point(),
                                innercir->next()->vertex()->point() -
                                h->vertex()->point()
                            ) ;
                    innercir = innercir->next_around_vertex();
                } while (innercir != h->opposite());
            }
            angle += ::fabs(2.0*M_PI - original_angle );
            cir = cir->next_around_vertex();
        } while (cir != h->opposite());
        return angle;
    }
    
    double Decimator::edge_collapse_importance(Map::Halfedge* h){
        if (strategy_ == LENGHT_BASED){
            return Geom::vector(h).norm();
        }
        if (strategy_ == ANGULAR_DEFAULT){
            return delta_angular_default_cause_by_collapse(h);
        }
	
        double delta_vol=delta_volume_cause_by_collapse(h); 
        double delta_area=delta_area_on_border_cause_by_collapse(h);
        
        double min_angle_sin_2=1;
        Map::Halfedge* cir = h->opposite();
        do {
            if (
                cir !=  h->opposite() &&
                cir != h->prev() &&
                !cir->is_border()
	    ) {
                Vector3d v0 =
                    cir->next()->vertex()->point() - h->vertex()->point();
                Vector3d v1 =
                    cir->opposite()->vertex()->point() - h->vertex()->point();
                v0.normalize(); v1.normalize(); 
                min_angle_sin_2 = ogf_min(min_angle_sin_2,(v0^v1).norm2());
            }
            cir = cir->next_around_vertex() ;
        } while (cir != h->opposite());
        if (min_angle_sin_2<1e-10) {
            return 1e20;
        }
        return ::pow(delta_vol,0.333) + 5.0 * ::pow(delta_area,0.5);
    }




    bool Decimator::can_collapse(Map::Halfedge* h){
        // check topological validity
        MapEditor editor(map_);
        if (!editor.can_collapse_edge(h)) {
            return false ;
        }
        
        // don't simplify the border too much
        if (
            !h->is_border() &&
            !h->opposite()->is_border() &&
            h->opposite()->vertex()->is_on_border()
        ) {
            return false ;
        }
        
        // check that no normal swap occur
        Map::Halfedge* cir = h->opposite();
        do {
            if(
                cir !=  h->opposite() && cir != h->prev() && !cir->is_border()
	    ) {
                Vector3d v0 =
                    cir->next()->vertex()->point() - h->vertex()->point();
                Vector3d v1 =
                    cir->opposite()->vertex()->point() - h->vertex()->point();
	  
                v0.normalize();
                v1.normalize();

                Vector3d vertex_normal = 
                    Geom::vertex_normal(h->opposite()->vertex());

                Vector3d future_normal = v0 ^ v1;
                future_normal.normalize();
                double tmp = vertex_normal * future_normal;
                if ( 
                    (tmp < 0) || 
                    (v0 * v1 < -0.999) || 
                    (v0 * v1 > 0.999)
                ) {
                    return false; 
                }
            }	
            cir = cir->next_around_vertex() ;
        } while (cir != h->opposite());
        return true;
    }


    double Decimator::compute_vertex_importance(Map::Vertex* v){
        Map::Halfedge* cir = v->halfedge();
        importance_[v] = 10e20;
        halfedge_to_collapse_[v] = nil ;
        do {
            if (can_collapse(cir->opposite())) {
                double imp = edge_collapse_importance(cir->opposite());
                if (imp < importance_[v]) {
                    importance_[v] = imp;
                    halfedge_to_collapse_[v] = cir->opposite();
                }
            }
            cir = cir->next_around_vertex();
        } while (cir != v->halfedge());

        if(vertex_density_.is_bound()) {
            importance_[v] *= (1.0 + vertex_density_[v]) ;
        }

        double result = importance_[v] +  accumulated_cost_[v];
        return result ;
    }
  
}
