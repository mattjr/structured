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

#include <OGF/cells/map_algos/gluer.h>
#include <OGF/cells/map/map_editor.h>
#include <OGF/cells/map/geometry.h>
#include <OGF/basic/debug/progress.h>

#include <stack>

//__________________________________________________________

namespace OGF {

    Gluer::Gluer() : 
        tolerance_(0), map_(nil), id1_(-1), id2_(-1), glue_func_(GLUE_ALWAYS), parent_(nil) {
    }

    bool Gluer::test_glue_func(
        Map::Halfedge* h1, Map::Halfedge* h2
    ) const {

        if(parent_ != nil) {
            return parent_->test_glue_func(h1, h2) ;
        }

        if(glue_func_ == GLUE_ALWAYS) {
            return true ;
        }

        ogf_assert(vertex_attribute_.is_bound()) ;
        int id1 = vertex_attribute_[h1->vertex()] ;
        int id2 = vertex_attribute_[h2->vertex()] ;
        switch(glue_func_) {
        case GLUE_ALWAYS:
            return true ;
            break ;
        case GLUE_NEQ: 
            return (id1 != id2) ;
            break ;
        case GLUE_EQ: 
            return (id1 == id2) ;
            break ;
        case GLUE_1_OR_2: 
            return 
                ((id1 == id1_) || (id1 == id2_)) &&
                ((id2 == id1_) || (id2 == id2_)) ; 
            break ;
        case GLUE_1_AND_2:
            return 
                (id1 != id2) && (
                    ((id1 == id1_) && (id2 == id2_)) ||
                    ((id1 == id2_) && (id2 == id1_)) 
                ) ; 
            break ;
        }
        return false ;
    }

    Gluer::~Gluer() {
    }

    //_______________________________________________________

    // Lexicographic compare
    bool DefaultGluer::Gluer_PtsCmp::operator()
	(const Point3d& p0,const Point3d& p1) const {
        if (p0.x() > p1.x()) {
            return false ;
        } else if (p0.x() < p1.x()) {
            return true;
        } else {
            if (p0.y() > p1.y()) {
                return false ;
            } else if (p0.y() < p1.y()) {
                return true ;
            } else {
                if (p0.z() > p1.z()) {
                    return false ;
                } else if (p0.z() < p1.z()) {
                    return true ;
                } else {
                    return false ;
                }
            }
        }
        ogf_assert(false); 
        return true;
    }
    


    Point3d DefaultGluer::grid_node(const Point3d &p) {

        // Snap points coordinates on a grid
        // having its step equal to tolerance_

        // Just to limit problems on cubes borders
        Point3d pt (
            p.x() + 0.01*p.y() + 0.01*p.z(),
            p.y() + 0.01*p.z() + 0.01*p.x(),
            p.z() + 0.01*p.x() + 0.01*p.y()
        ) ;

        return Point3d (
            (3.0 * tolerance_) * (long int) (pt.x() / (3.0 * tolerance_)),
            (3.0 * tolerance_) * (long int) (pt.y() / (3.0 * tolerance_)),
            (3.0 * tolerance_) * (long int) (pt.z() / (3.0 * tolerance_))
        ) ;
    } 

    void DefaultGluer::add_halfedge
    (
        Map::Halfedge* hi			  
    ){
        if (hi->is_border()) {
            Point3d p = hi->opposite()->vertex()->point();
            // TODO: eliminate zero-length edges
      
            ogf_assert(
                hi->next()->opposite()->vertex() == hi->vertex()
            ) ;
            if (tolerance_ == 0) {
                point_searcher[p].push_back(exist.size());
                PointSearcher::iterator cur = point_searcher.find(p);
                ogf_assert(cur != point_searcher.end());
            } else {
                point_searcher[grid_node(p)].push_back(exist.size());
            }
            exist.push_back(true);
            halfedges.push_back(hi);
        }
    }
    
    
    int DefaultGluer::nb_halfedges_like(
        Map::Halfedge* h
    ) {
        h = h->opposite() ;
        const Point3d& p1 = h->vertex()->point() ;
        PointSearcher::iterator it = point_searcher.end() ;
        if(tolerance_ == 0) {
            it = point_searcher.find(p1) ;
        } else {
            it = point_searcher.find(grid_node(p1)) ;
        }

        int result = 0 ;

        if (it != point_searcher.end()) {
            std::vector<int>& vect = it->second;
            for (unsigned int j=0; j<vect.size();  j++) {
                if(have_same_location(h,halfedges[ vect[j] ])) {
                    result++ ;
                }
            }
        }

        return result ;

    }

    bool DefaultGluer::is_non_manifold(Map::Halfedge* hi) {
        int nb_tot = 
            nb_halfedges_like(hi) + nb_halfedges_like(hi->opposite()) ;
        return (nb_tot > 2) ;
    }

    bool DefaultGluer::can_merge(Map::TexVertex* tv1, Map::TexVertex* tv2) {
        if(!has_attributes_) {
            return false ;
        }

        const Point2d& uv1 = tv1->tex_coord() ;
        const Point2d& uv2 = tv2->tex_coord() ;
        const Vector3d& n1 = tex_vertex_normal_[tv1] ;
        const Vector3d& n2 = tex_vertex_normal_[tv2] ;

        double d = (uv2 - uv1).norm() ;
        double a = Geom::angle(n1,n2) ;

        double dmax = 1e-3 ;
        double amax = 15.0 * (M_PI / 180.0) ;

        return (d <= dmax && a <= amax) ;
    }

    bool DefaultGluer::test_info(Map::Halfedge* h1, Map::Halfedge* h2) {
        ogf_assert(h1->is_border()) ;
        ogf_assert(h2->is_border()) ;
        
        if(!has_attributes_) {
            return true ;
        }

        int mat1 = facet_material_id_[h1->opposite()->facet()] ;
        int mat2 = facet_material_id_[h2->opposite()->facet()] ;
        
        if(mat1 != mat2) {
            return false ;
        }

        return true ;
    }

    void DefaultGluer::apply() {

        if(map_== nil) {
            std::cerr << "Gluer: No surface" << std::endl ;
            return ;
        }
    
        has_attributes_ = false ;
        if(
            MapTexVertexNormal::is_defined(map_) &&
            MapFacetMaterialId::is_defined(map_)
        ) {
            has_attributes_ = true ;
            tex_vertex_normal_.bind(map_) ;
            facet_material_id_.bind(map_) ;
        }

        MapEditor editor(map_) ;
    
    
        // add halfedges to search tree
        {FOR_EACH_HALFEDGE(Map, map_, hi) {
            add_halfedge(hi);
        }}
 
        std::vector<Map::Halfedge*> to_glue ;


        for (unsigned int i=0;i<exist.size();i++) {
            if ( exist[i]){
                ogf_assert(halfedges[i]->is_border());
                PointSearcher::iterator cur = point_searcher.end() ;
                Point3d p = halfedges[i]->vertex()->point();
                if (tolerance_==0) {
                    cur  = point_searcher.find(p);
                } else {
                    cur  = point_searcher.find(grid_node(p));
                }

                if (cur != point_searcher.end()) {
                    std::vector<int>& vect = cur->second;
                    for (unsigned int j=0; j<vect.size() && exist[i]; j++) {
                        if (exist[vect[j]]) {
                            ogf_assert(halfedges[vect[j]]->is_border());
                            if (
                                have_same_location(
                                    halfedges[i],halfedges[vect[j]]
                                ) &&
                                editor.can_glue(
                                    halfedges[i],halfedges[vect[j]]
                                ) 
                            ) {
                                exist[i] = false;
                                exist[vect[j]] = false;
                                if(test_glue_func(
                                       halfedges[i], halfedges[vect[j]]
                                   ) && !is_non_manifold(halfedges[i])
                                     && test_info(
                                          halfedges[i],halfedges[vect[j]]
                                        )
                                     && Geom::edge_length(halfedges[i]) > 
                                        tolerance_
                                ) {
                                    to_glue.push_back(halfedges[i]) ;
                                    to_glue.push_back(halfedges[vect[j]]) ;
                                }
                            }
                        }
                    }
                }
            }
        }

        std::vector<Map::Halfedge*>::iterator it = to_glue.begin() ;
        while(it != to_glue.end()) {
            Map::Halfedge* h1 = *it ;
            it++ ;
            Map::Halfedge* h2 = *it ;
            it++ ;
            editor.glue(h1, h2, false) ;
        }

        if(has_attributes_) {
            { FOR_EACH_VERTEX(Map, map(), it) {
                Map::Halfedge* h1 = it->halfedge() ;
                do {
                    Map::Halfedge* h2 = h1->next_around_vertex() ;

                    if(can_merge(h1->tex_vertex(), h2->tex_vertex())) {
                        set_halfedge_tex_vertex(h2, h1->tex_vertex()) ;
                    }
                    h1 = h2 ;
                } while(h1 != it->halfedge()) ;
            }}
        }
    }  

    bool DefaultGluer::have_same_location(
        Map::Halfedge* h0, Map::Halfedge* h1
    ) {
        return (
            (
                h0->vertex()->point() -
                h1->opposite()->vertex()->point()
            ).norm2() <= tolerance_ * tolerance_
            && 
            (
                h0->opposite()->vertex()->point() - 
                h1->vertex()->point()
            ).norm2() <= tolerance_ * tolerance_
        ) ; 
    }

    //___________________________________________________
    
    void RecursiveGluer::apply() {
        double d = 0.0 ;
        while(d < tolerance()) {
            DefaultGluer gluer ;
            gluer.set_map(map()) ;
            gluer.set_parent(this) ;
            gluer.set_vertex_attribute(vertex_attribute()) ;
            gluer.set_tolerance(d) ;
            gluer.set_glue_func(glue_func()) ;
            gluer.set_id1(id1()) ;
            gluer.set_id2(id2()) ;
            gluer.apply() ;
            if (d == 0) {
                d = 10e-10;
            } else if (d == tolerance()) {
                break ;
            } else if (d*10.0 > tolerance()) {
                d = tolerance();
            } else {
                d *= 10.0 ;
            }
        }
    } 

    //____________________________________________________
    
    void AutoSetGluer::apply() {
        double max_length = 1e-10 ;
        double min_length = 1e10 ;
        FOR_EACH_HALFEDGE(Map, map(), iter) {
            double length = (
                iter->vertex()->point() - iter->opposite()->vertex()->point() 
            ).norm();
            if (length > 1e-8) {
                min_length = ogf_min(min_length, length);
            }
            max_length = ogf_max(max_length, length) ;
        }
        double max_ratio = 0.01;
        set_tolerance(ogf_max(min_length,max_length*max_ratio) /2.0);
        RecursiveGluer::apply();
    }


//--------------------------------------------------------------------------------------------------------------------

    FastMapComponentExtractor::FastMapComponentExtractor(Map* map) : 
        map_(map), visited_facet_(map), visited_vertex_(map) {
    }

    MapComponent* FastMapComponentExtractor::extract_component(Map::Facet* start) {
        MapComponent* result = new MapComponent(map_) ;
        set_target(result) ;
        std::stack<Map::Facet*> S ;
        S.push(start) ;
        visited_facet_[start] = true ;
        while(!S.empty()) {
            Map::Facet* top = S.top() ;
            S.pop() ;
            facets().push_back(top) ;
            Map::Halfedge* h = top->halfedge() ;
            do {
                halfedges().push_back(h) ;
                if(!visited_vertex_[h->vertex()]) {
                    vertices().push_back(h->vertex()) ;
                    visited_vertex_[h->vertex()] = true ;
                }
                if(h->opposite()->is_border()) {
                    halfedges().push_back(h->opposite()) ;
                } else {
                    Map::Facet* f = h->opposite()->facet() ;
                    if(!visited_facet_[f]) {
                        S.push(f) ;
                        visited_facet_[f] = true ;
                    }
                }
                h = h->next() ;
            } while(h != top->halfedge()) ;
        }
        FOR_EACH_VERTEX(MapComponent, result, it) {
            visited_vertex_[it] = false ;
        }
        FOR_EACH_FACET(MapComponent, result, it) {
            visited_facet_[it] = false ;
        }
        return result ;
    }

//--------------------------------------------------------------------------------------------------------------------

    ReorientGluer::ReorientGluer(Map* map) : extractor_(map), editor_(map) {
    }

    bool ReorientGluer::facet_is_isolated(Map::Facet* f) {
        Map::Halfedge* h = f->halfedge() ;
        do {
            if(!h->opposite()->is_border()) {
                return false ;
            }
            h = h->next() ;
        } while(h != f->halfedge()) ;
        return true ;
    }
    
    void ReorientGluer::glue_reorient(
        Map::Halfedge* h1, Map::Halfedge* h2, bool reorient 
    ) {
        ogf_assert(h1->is_border()) ;
        ogf_assert(h2->is_border()) ;
        if(reorient) {
            Map::Facet* f = h1->opposite()->facet() ;
            // Optimization: try to flip smaller facets
            // rather than large ones. 
            if(facet_is_isolated(h2->opposite()->facet())) {
                f =  h2->opposite()->facet() ;
            }
            MapComponent_var comp = extractor_.extract_component(f) ;
            editor_.inside_out(comp, false) ;
        } 
        editor_.glue(h1, h2) ;
    }

    void ReorientGluer::glue_reorient(
        Map::Halfedge* h1, Map::Halfedge* h2, 
        MapVertexAttribute<int>& vertex_id
    ) {
        ogf_assert(h1->is_border()) ;
        ogf_assert(h2->is_border()) ;
        ogf_assert(
            (
                (vertex_id[h1->vertex()] == vertex_id[h2->vertex()]) &&
                (vertex_id[h1->opposite()->vertex()] == vertex_id[h2->opposite()->vertex()]) 
            ) || (
                (vertex_id[h1->vertex()] == vertex_id[h2->opposite()->vertex()]) &&
                (vertex_id[h2->vertex()] == vertex_id[h1->opposite()->vertex()])
            )
        ) ;
        bool reorient = (vertex_id[h1->vertex()] == vertex_id[h2->vertex()]) ;
        glue_reorient(h1, h2, reorient) ;
    }

    void ReorientGluer::glue_reorient(
        Map::Halfedge* h1, Map::Halfedge* h2, 
        MapVertexAttribute<IntPair>& vertex_id
    ) {
        ogf_assert(h1->is_border()) ;
        ogf_assert(h2->is_border()) ;
        
        ogf_assert(
            (
                (vertex_id[h1->vertex()] == vertex_id[h2->vertex()]) &&
                (vertex_id[h1->opposite()->vertex()] == vertex_id[h2->opposite()->vertex()]) 
            ) || (
                (vertex_id[h1->vertex()] == vertex_id[h2->opposite()->vertex()]) &&
                (vertex_id[h2->vertex()] == vertex_id[h1->opposite()->vertex()])
            )
        ) ;
        
        bool reorient = (vertex_id[h1->vertex()] == vertex_id[h2->vertex()]) ;
        glue_reorient(h1, h2, reorient) ;
    }

    Map::Halfedge* ReorientGluer::find_halfedge(Map::Vertex* v1, Map::Vertex* v2, bool& inverted) {
        Map::Halfedge* cur = v1->halfedge() ;
        Map::Halfedge* result = nil ;
        do {
            if(cur->is_border() && cur->opposite()->vertex() == v2) {
                result = cur ;
                break ;
            }
            cur = cur->next_around_vertex() ;
        } while(cur != v1->halfedge()) ;
        if(result == nil) {
            inverted = true ;
            cur = v2->halfedge() ;
            do {
                if(cur->is_border() && cur->opposite()->vertex() == v1) {
                    result = cur ;
                    break ;
                }
                cur = cur->next_around_vertex() ;
            } while(cur != v2->halfedge()) ;
        } else {
            inverted = false ;
        }
        return result ;
    }
    
    void ReorientGluer::glue_reorient(
        Map::Vertex* v11, Map::Vertex* v12, 
        Map::Vertex* v21, Map::Vertex* v22
    ) {
        bool h1_inverted, h2_inverted ;
        Map::Halfedge* h1 = find_halfedge(v11, v21, h1_inverted) ;
        Map::Halfedge* h2 = find_halfedge(v12, v22, h2_inverted) ;
        bool reorient = (h1_inverted ^ h2_inverted) ;
        glue_reorient(h1, h2, reorient) ;
    }
    
    void ReorientGluer::glue_reorient(
        Map::Halfedge* h1, Map::Halfedge* h2
    ) {
        const Point3d& p11 = h1->vertex()->point() ;
        const Point3d& p12 = h1->opposite()->vertex()->point() ;
        const Point3d& p21 = h2->vertex()->point() ;
        const Point3d& p22 = h2->opposite()->vertex()->point() ;
        
        bool reorient = 
            (p11 - p21).norm() + (p12 - p22).norm() < 
            (p11 - p22).norm() + (p12 - p21).norm() ;
        
        glue_reorient(h1, h2, reorient) ;
    }

    void ReorientGluer::glue(MapVertexAttribute<int>& vertex_id) {

        typedef std::map< IntPair, Map::Halfedge*> CcMap ;
        CcMap e2e ;
        
        typedef std::pair<Map::Halfedge*,Map::Halfedge*> EdgePair ;
        std::vector<EdgePair> edges_to_glue ;

        FOR_EACH_FACET(Map, editor_.target(), it) {
            Map::Halfedge* h = it->halfedge() ;
            do {
                int k1 = vertex_id[h->prev()->vertex()] ;
                int k2 = vertex_id[h->vertex()] ;
                IntPair k ; k.first = ogf_min(k1,k2) ; k.second = ogf_max(k1,k2) ;

                CcMap::iterator jt = e2e.find(k) ;
                if(jt == e2e.end()) {
                    e2e[k] = h ;
                } else {
                    Map::Halfedge* opp = jt->second ;
                    edges_to_glue.push_back(EdgePair(h, opp)) ;
                }

                h = h->next() ;
            } while(h != it->halfedge()) ;
        }

        {
            ProgressLogger logger(edges_to_glue.size()) ;
            for(unsigned int i=0; i<edges_to_glue.size(); i++) {
                logger.notify(i) ;
                if(logger.is_canceled()) {
                    break ;
                }
                Map::Halfedge* h1 = edges_to_glue[i].first->opposite() ;
                Map::Halfedge* h2 = edges_to_glue[i].second->opposite() ;
                if(
                    h1->is_border() && 
                    h2->is_border() &&
                    (
                        (
                            vertex_id[h1->vertex()] == vertex_id[h2->prev()->vertex()] &&
                            vertex_id[h2->vertex()] == vertex_id[h1->prev()->vertex()] 
                        ) ||  (
                            vertex_id[h1->vertex()] == vertex_id[h2->vertex()] &&
                            vertex_id[h1->prev()->vertex()] == vertex_id[h2->prev()->vertex()] 
                        )
                    )
                ) {
                    glue_reorient(h1, h2, vertex_id) ;
                }
            }
        }

    }

//----------------------------------------------------------------------------------------------

}


