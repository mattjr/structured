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
 

#include <OGF/cells/map_algos/pm_manager.h>
#include <OGF/cells/map_algos/decimator.h>
#include <OGF/cells/map/geometry.h>

namespace OGF {

//_________________________________________________________

    /**
     * used internally by the PMManager. This is a slightly modified version
     * of the Decimator.
     */
    class PMDecimator : public Decimator {
    public :
        PMDecimator(Map* map, PMManager* manager) : Decimator(map) {
            pm_manager_ = manager ;
        }
        
        void collapse_edge(Map::Halfedge* h){
            VertexSplit split ;
            split.left_position_ = h->next()->opposite();
            split.new_block_ = h;

            if (!h->opposite()->is_border()) {
                split.right_position_ = h->opposite()->prev()->opposite();
                pm_manager_->vsplit_.push_back(split);
                pm_manager_->check_surface_integrity();
                pm_manager_->undo_split(split);
                pm_manager_->check_surface_integrity();
            } else {
                ogf_assert(h->opposite()->is_border());
                pm_manager_->vsplit_.push_back(split);
                pm_manager_->check_surface_integrity();
                pm_manager_->undo_split(split);
                pm_manager_->check_surface_integrity();
            }
        }

        static double angle_between_halfedges(
            Map::Halfedge *h0, Map::Halfedge *h1
        ){
            Vector3d v0 = Geom::vector(h0);
            Vector3d v1 = Geom::vector(h1);
            return Geom::angle(v0, v1) ;
        }
        
        double anglular_defect(Map::Vertex* v) {
            double sigma_angles =0;
            Map::Halfedge* cir = v->halfedge();
            do{
                sigma_angles += angle_between_halfedges(
                    cir, cir->next()->opposite()
                );
                cir = cir->next_around_vertex();
            } while( cir != v->halfedge());
            return sigma_angles - 2.0*M_PI ;
        }

        bool can_collapse(Map::Halfedge* h) {


            bool keep_small_features = true ;

            if(keep_small_features) {

                // don't remove high signifiant frequencies inside the surface
                if ( fabs(anglular_defect(h->prev()->vertex())) > M_PI/6.0) {
                    return false;
                }

                // don't remove high signifiant frequencies on border
                if ( h->opposite()->is_border() &&
                    angle_between_halfedges(
                        h,h->opposite()->next()->opposite()
                    )
                    >  M_PI/6.0
                ) {
                    return false ;
                }
                
                if (
                    pm_manager_->vertex_orbit_size(h->next()->vertex()) <= 3 ||
                    (
                        !h->opposite()->is_border() &&
                        pm_manager_->vertex_orbit_size(
                            h->opposite()->next()->vertex()
                        ) <= 3
                    )
                ) {
                    return false ;
                }
            }


            if (
                !h->opposite()->is_border() &&
                h->opposite()->vertex()->is_on_border()
            ) {
                return false ; // yep, this is ugly !!
            }

            // check that no normal swap occurs
            Vector3d vertex_normal = 
                Geom::vertex_normal(h->opposite()->vertex());
            Map::Halfedge* cir = h->opposite() ;
            do{
                if (
                    cir !=  h->opposite() &&
                    cir != h->prev() &&
                    !cir->is_border()
                ) {
                    Vector3d v0 =
                        cir->next()->vertex()->point() - 
                        h->vertex()->point() ;

                    Vector3d v1 =
                        cir->opposite()->vertex()->point() - 
                        h->vertex()->point() ;
	  
                    Vector3d future_normal = v0 ^ v1;
                    double tmp = vertex_normal * future_normal;
                    if ( 
                        (tmp < 0) || (v0 * v1 < -0.999) || (v0 * v1 > 0.999)
                    ) {
                        return false; 
                    }
                }	
                cir = cir->next_around_vertex() ;
            } while (cir != h->opposite());
            return pm_manager_->can_collapse(h);
        }

        PMManager *pm_manager_;
  } ;

//_________________________________________________________

    PMManager::PMManager(Map* s) : MapMutator(s) {
        cur_ = 0 ;
        progress_ = nil ;
        decim_border_ = true ;
        if(pm_vertex_lock_.is_defined(s, "pm_is_locked")) {
            pm_vertex_lock_.bind(s, "pm_is_locked") ;
        }
    }


    bool PMManager::init() {
        if(chart_id_.is_defined(target(),"chart")) {
            chart_id_.bind(target(),"chart") ;
            vertex_kind_.bind(target(),"kind") ;
            FOR_EACH_VERTEX(Map, target(), it) {
                vertex_kind_[it] = get_vertex_kind(it) ;
            }
        }

        PMDecimator decim(target(),this) ;
        decim.set_proportion_to_remove(1.0) ; // decimate to the max...
        decim.set_progress(progress_) ;
        decim.apply() ;
        
        while (next_refinement().is_valid()) {
            refine() ;
        }
        
        if(chart_id_.is_bound()) {
            chart_id_.unbind() ;
        }

        if(vertex_kind_.is_bound()) {
            vertex_kind_.unbind() ;
        }

        return true;
    }

    int PMManager::get_vertex_kind(Map::Vertex* v) {
        int id1 = -1 ;
        int id2 = -1 ;
        int id3 = -1 ;
        Map::Halfedge* h = v->halfedge() ;
        do {
            if(!h->is_border()) {
                int id = chart_id_[h->facet()] ;
                if(id1 == -1 || id1 == id) {
                    id1 = id ;
                } else if(id2 == -1 || id2 == id) {
                    id2 = id ;
                } else if(id3 == -1 || id3 == id) {
                    id3 = id ;
                }
            }
            h = h->next_around_vertex() ;
        } while(h != v->halfedge()) ;
        if(id3 != -1) {
            return 0 ; // "Vertex" touches more than two different charts
        }
        if(id2 != -1) {
            return 1 ; // "Edge" touches two different charts
        }
        return 2 ;     // "Chart" touches one chart only
    }

    void PMManager::get_neighbors(Map::Vertex* v, int& neigh1, int& neigh2) {
        ogf_assert(get_vertex_kind(v) == 1) ;
        neigh1 = -1 ;
        neigh2 = -1 ;
        Map::Halfedge* h = v->halfedge() ;
        do {
            if(!h->is_border()) {
                int id = chart_id_[h->facet()] ;
                if(neigh1 == -1 || neigh1 == id) {
                    neigh1 = id ;
                } else if(neigh2 == -1 || neigh2 == id) {
                    neigh2 = id ;
                }
            }
            h = h->next_around_vertex() ;
        } while(h != v->halfedge()) ;
        if(neigh1 > neigh2) {
            ogf_swap(neigh1, neigh2) ;
        }
    }

    VertexSplit PMManager::next_refinement() {
        if (cur_<1 ) {
            return VertexSplit() ;
        }
        return vsplit_[cur_-1];
    }
  
    VertexSplit PMManager::next_coarsening() {	
        if (cur_ >= int(vsplit_.size())) {
            return VertexSplit();
        }
        return vsplit_[cur_];
    }

    VertexSplit PMManager::refine() {
        VertexSplit result;
        if (cur_<1 ) {
            return result ;
        }
        
        ogf_assert(cur_ > 0  && cur_ <= int(vsplit_.size()));
        ogf_assert(vsplit_[cur_-1].new_block_ != nil);
        ogf_assert(vsplit_[cur_-1].new_block_->opposite() != nil);
        
        result = vsplit_[cur_-1];
        apply_split(vsplit_[cur_-1]);
        
        return result;
    }
  
    VertexSplit PMManager::coarsen() {
	VertexSplit result;
	if (cur_>=int(vsplit_.size())) {
	  return result;
        }

	ogf_assert(cur_ >= 0  && cur_ < int(vsplit_.size())) ;
	ogf_assert(vsplit_[cur_].new_block_ != nil) ;
	ogf_assert(vsplit_[cur_].new_block_->opposite() != nil) ;

	result = vsplit_[cur_];
	undo_split(vsplit_[cur_]);
        
        return result;
    }

    void PMManager::set_level(int level) {
        if(level > cur_) {
            while(level > cur_ && coarsen().is_valid()) ;
        } else {
            while(level < cur_ && refine().is_valid()) ;
        }
    }

    bool PMManager::can_collapse(Halfedge* h) {
        
        if(
            pm_vertex_lock_.is_bound() && 
            (
                pm_vertex_lock_[h->vertex()] ||
                pm_vertex_lock_[h->opposite()->vertex()] 
            )
        ) {
            return false ;
        } 

        // [Bruno]: added that to make Ben's multires Stretch L2 
        // work on this version.
        if(!decim_border_ && h->vertex()->is_on_border()) {
            return false ;
        }


        if (h->prev()->opposite()->is_border()) {
            return false; // not yet well implemented so it's forbidden
        }


        // First attempt to constrain PMs on texture atlases,
        // does not work yet.
        if(false && vertex_kind_.is_bound()) {
            if(get_vertex_kind(h->vertex()) > get_vertex_kind(h->opposite()->vertex())) {
                return false ;
            }
            if(
                get_vertex_kind(h->vertex()) == 1 &&
                get_vertex_kind(h->opposite()->vertex()) == 1
            ) {
                int id11, id12 ;
                int id21, id22 ;
                get_neighbors(h->vertex(), id11, id12) ;
                get_neighbors(h->opposite()->vertex(), id21, id22) ;
                if(id11 != id21 || id12 != id22) {
                    return false ;
                }
            }
        }

        Halfedge* opp = h->opposite();

        // don't cut left wing
        if (
            h->next()->opposite()->is_border() &&
            h->prev()->opposite()->is_border()
        ) {
            return false;
        }
        

        // no toblerone
        int num_common_vertex = 0;
        Halfedge* dest_cir = h;
        do {
            Halfedge* org_cir = opp;
            do {
                if (
                    dest_cir->opposite()->vertex() ==
                    org_cir->opposite()->vertex() 
                ) { 
                    num_common_vertex++ ;
                }
                org_cir = org_cir->opposite()->prev();
            } while (org_cir != opp);
            dest_cir = dest_cir->opposite()->prev();
        } while (dest_cir != h) ;

        if (num_common_vertex != 2 && !opp->is_border()) {
            return false ; 
        }
        if (num_common_vertex != 1 && opp->is_border()) {
            return false ;
        }


        if ( opp->is_border()) {
            // don't fill holes
            if ( opp == opp->next()->next()->next()) {
                return false;
            }
            // don't remove 2 points
            if ( h->next()->opposite() == h->opposite()->prev()) {
                return false;
            }
            
            ogf_assert ( h->next()->opposite() != h->prev()->opposite());
            ogf_assert ( vertex_orbit_size(h->next()->vertex())>=3);
            return true;
        }
      
      
        if (
            h->vertex()->is_on_border() && opp->vertex()->is_on_border()
        ) {
            return false;
        }
        
        // don't cut right wing
        if (
            opp->next()->opposite()->is_border() &&
            opp->prev()->opposite()->is_border()
        ) {
            return false;
        }

        if (vertex_orbit_size(h->vertex())<=2) { 
            return false;
        }
        
        if (vertex_orbit_size(opp->vertex())<=2) { 
            return false;
        }
        
        return true;
    }
    

    bool PMManager::can_undo_split(VertexSplit split) {
        
        if (split.right_position_ == nil) {
            Halfedge* h1 = split.left_position_;
            Halfedge* h0 = h1->opposite()->prev()->opposite()->prev();
//          Halfedge* h2 = h1->opposite()->next()->opposite();
            Halfedge* h3 = h1->opposite()->prev()->opposite()->next();
            Halfedge* h  = split.new_block_;
            if (!h->opposite()->is_border()) return false;
            if (h0->prev()==h3)              return false; // remove hole
            if (h0==h1)                      return false; // remove 2 vertices
        }
        return true;
    }

    int PMManager::vertex_orbit_size(Vertex* v) {
        int res =0;
        Halfedge* cir = v->halfedge();
        do {
            res++;
            cir=cir->opposite()->prev();
        } while (res<1000 &&  cir != v->halfedge());
        return res;
    }

    void PMManager::undo_split(VertexSplit split) {
        cur_++;
        ogf_assert( can_collapse(split.new_block_));
        ogf_assert( can_undo_split(split) );
        if (split.new_block_->opposite()->is_border()){
            Halfedge* h1 = split.left_position_;
            Halfedge* h0 = h1->opposite()->prev()->opposite()->prev();
            Halfedge* h2 = h1->opposite()->next()->opposite();
            Halfedge* h3 = h1->opposite()->prev()->opposite()->next();
            Halfedge* h  = split.new_block_;
            
            make_vertex_key(h->prev());
            make_vertex_key(h2);
            make_vertex_key(h1);
            make_facet_key(h);

            deactivate_vertex(h->opposite()->vertex());
            deactivate_facet(h->facet());
            deactivate_halfedge(h->prev());
            deactivate_halfedge(h->next());
            deactivate_halfedge(h);
            deactivate_halfedge(h->opposite());
            
            link(h1,h2,2);
            link(h0,h3,1);
            set_halfedge_opposite(h->next(),NULL) ;
            set_halfedge_opposite(h->prev(),NULL) ;
            
            set_vertex_on_orbit(h2,h2->vertex());
            set_vertex_on_orbit(h0,h0->vertex());
            
            set_tex_vertex_on_orbit(h1,h1->tex_vertex());
            
            ogf_assert(h0->next()->prev() == h0);
            ogf_assert(h3->next()->prev() == h3);
            ogf_assert(h0->prev()->next() == h0);
            ogf_assert(h3->prev()->next() == h3);

        } else {

            Halfedge* h  = split.new_block_;
            Halfedge* opp  =h->opposite();
            Halfedge* h0 = split.right_position_;
            Halfedge* h1 = split.left_position_;
            Halfedge* h2 = h1->opposite()->next()->opposite();
            Halfedge* h3 = h0->opposite()->prev()->opposite();


            ogf_assert(h1 != h0->opposite());
            ogf_assert(h != h1);
            ogf_assert(h != h0->opposite());

            make_vertex_key(h->prev());
            make_vertex_key(h2);
            make_vertex_key(h1);
            make_vertex_key(h0);
            make_facet_key(h);
          
            deactivate_vertex(opp->vertex());
            deactivate_facet(h->facet());
            deactivate_facet(opp->facet());
            deactivate_halfedge(h->prev());
            deactivate_halfedge(h->next());
            deactivate_halfedge(h);
            deactivate_halfedge(opp->prev());
            deactivate_halfedge(opp->next());
            deactivate_halfedge(opp);

            link(h1,h2,2);
            link(h0,h3,2);
          
            set_halfedge_opposite(h->next(),NULL) ;
            set_halfedge_opposite(h->prev(),NULL) ;
            set_halfedge_opposite(opp->next(),NULL) ;
            set_halfedge_opposite(opp->prev(),NULL) ;
          
            set_vertex_on_orbit(h2,h2->vertex());
            set_vertex_on_orbit(h0,h0->vertex());
            set_vertex_on_orbit(h1,h1->vertex());

            set_tex_vertex_on_orbit(h1,h1->tex_vertex());
        }
    }
    

    void PMManager::apply_split(VertexSplit split) {
        cur_--;
        
        if (split.new_block_->opposite()->is_border()) {
            Halfedge* h  = split.new_block_;
            Halfedge* h1 = split.left_position_;
            Halfedge* h0 = h1;
            while (!h0->is_border()) {
                h0 = h0->next()->opposite();
            }

            Halfedge* h2 = h1->opposite();
            Halfedge* h3 = h2;
            while (!h3->is_border()) {
                h3 = h3->prev()->opposite();
            }

            make_facet_key(h);
            make_vertex_key(h2);
            make_vertex_key(h->prev());
            make_vertex_key(h1);

            link(h0,h->opposite(),1);
            link(h->next(),h1,2);
            link(h->prev(),h2,2);
            link(h->opposite(),h3,1);

            set_facet_on_orbit(h,h->facet());
            set_vertex_on_orbit(h->prev(),h->prev()->vertex());
            set_vertex_on_orbit(h1,h1->vertex());
            set_vertex_on_orbit(h2,h2->vertex());
	
            activate_vertex(h->prev()->vertex());
            activate_facet(h->facet());
            activate_halfedge(h->prev());
            activate_halfedge(h->opposite());
            activate_halfedge(h->next());
            activate_halfedge(h);

            set_halfedge_tex_vertex(h,h1->tex_vertex());
            set_halfedge_tex_vertex(h->next(),h2->tex_vertex());
            set_tex_vertex_on_orbit(h2->prev(),h->prev()->tex_vertex());

        } else {

            Halfedge* h  = split.new_block_;
            Halfedge* opp = h->opposite();
          
            Halfedge* h0 = split.right_position_;
            Halfedge* h1 = split.left_position_;
            Halfedge* h2 = h1->opposite();
            Halfedge* h3 = h0->opposite();
          
            make_facet_key(h);
            make_facet_key(opp);
          
            make_vertex_key(h2);
            make_vertex_key(h0);
            make_vertex_key(h1);
            make_vertex_key(h->prev());

            link(h0,opp->prev(),2);
            link(h3,opp->next(),2);
            link(h1,h->next(),2);
            link(h2,h->prev(),2);

            set_facet_on_orbit(h,h->facet());
            set_facet_on_orbit(opp,opp->facet());
          
            set_vertex_on_orbit(h0,h0->vertex());
            set_vertex_on_orbit(h1,h1->vertex());
            set_vertex_on_orbit(h2,h2->vertex());
            set_vertex_on_orbit(h->prev(),h->prev()->vertex());

            activate_vertex(h->prev()->vertex());
          
            activate_facet(h->facet());
            activate_facet(opp->facet());
          
            activate_halfedge(h->prev());
            activate_halfedge(h->next());
            activate_halfedge(h);
          
            activate_halfedge(opp->prev());
            activate_halfedge(opp->next());
            activate_halfedge(opp);
          
            set_halfedge_tex_vertex(h,h1->tex_vertex());
            set_halfedge_tex_vertex(h->opposite()->prev(),h1->tex_vertex());
          
            set_halfedge_tex_vertex(h->next(),h2->tex_vertex());
            set_halfedge_tex_vertex(h->opposite()->next(),h0->tex_vertex());
            set_tex_vertex_on_orbit(h2->prev(),h->prev()->tex_vertex());
        }
    }


    bool PMManager::is_active(Halfedge* h) {
        FOR_EACH_HALFEDGE(Map,target(),inner_h){
            if( &*inner_h == h ) {
                return true ;
            }
        } 
        return false ;
    }
    
    void PMManager::check_surface_integrity(){
        return ;

        FOR_EACH_VERTEX(Map,target(),vi) {
            ogf_assert(is_active( vi->halfedge()));
        }

        FOR_EACH_FACET(Map,target(),fi) {
            ogf_assert(is_active( fi->halfedge()));
        }

        { FOR_EACH_HALFEDGE(Map,target(),h) {
            ogf_assert(is_active(h->opposite()));
            ogf_assert(is_active(h->next()));
            ogf_assert(is_active(h->prev()));
        } }

        { FOR_EACH_HALFEDGE(Map,target(),h) {
            ogf_assert(h->vertex()!= h->opposite()->vertex());
            ogf_assert(h->next()->next() != h);
            ogf_assert(h->opposite()->opposite() == h);
            ogf_assert(h->is_border() || h->next()->prev() == h);
            ogf_assert(!h->is_border() || h->next()->prev() == h);
            ogf_assert(h->vertex()->halfedge()->vertex()==h->vertex());
            ogf_assert(h->facet()->halfedge()->facet()==h->facet());

            Halfedge* cir = h;
            Vertex* v = h->vertex();
            int i=0;
            do {
                Halfedge* inner_cir = h;
                int nb=0;
                do {
                    if (
                        inner_cir->opposite()->vertex() == 
                        cir->opposite()->vertex()
                    ) {
                        nb++;
                    }
                    inner_cir = inner_cir->opposite()->prev();
                } while (inner_cir != h);

                ogf_assert(nb==1);
                ogf_assert(i++<100);
                ogf_assert(cir->vertex()==v); 
                cir = cir->opposite()->prev();
            } while ( cir != h );
            i=0;
            Facet* f = h->facet();
            do {
                ogf_assert(i++<10000);
                ogf_assert(cir->facet()==f);
                cir = cir->next();
            } while ( cir != h);
        } }
    }

}

