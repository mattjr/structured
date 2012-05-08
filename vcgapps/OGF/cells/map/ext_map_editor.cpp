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
 

#include <OGF/cells/map/ext_map_editor.h>
#include <OGF/cells/map/map_cell_heap.h>
#include <OGF/cells/map/map_attributes.h>
#include <OGF/cells/map/geometry.h>

#include <OGF/basic/debug/logger.h>

#include <set>
#include <queue>
#include <vector>

namespace OGF {

    //_________________________________________________________________

    /**
     * Used by match_borders
     */
    class MapBorder : public std::vector<Map::Halfedge*> {
    public:
        typedef std::vector<double> svector ;
        MapBorder(Map* surface, Map::Halfedge* start)  ;
        void update() ;
        void set_surface(Map* s) { surface_ = s ; }
        Map* surface() const { return surface_ ; }
        svector& s() { return s_ ; }
        double s(int i) { 
            ogf_assert(i >= 0 && i < int(s_.size())) ;
            return s_[i] ; 
        }

        int find(double s) ;
        Map::Vertex* insert_vertex(double s) ;
        void collapse_edge(int i) ;
        bool contains(Map::Halfedge* h) const ;

    protected:
        void init_s() ;
        
    private:
        Map* surface_ ;
        svector s_ ;
        Map::Halfedge* start_ ;
    } ;

    
    static double length(Map::Halfedge* h) {
        Point3d p1 = h->vertex()->point() ;
        Point3d p2 = h->opposite()->vertex()->point() ;
        return (p2 - p1).norm() ;
    }

    MapBorder::MapBorder(
        Map* surface, Map::Halfedge* start 
    ) : surface_(surface), start_(start) { 
        update() ;
    }

    void MapBorder::update() {
        ExtMapEditor::get_border_between_anchors(surface_, start_, *this) ;
        init_s() ;
    }

    void MapBorder::init_s() {
        s_.clear() ;
        double cur_s = 0 ;
        s_.push_back(cur_s) ;
        { for(unsigned int i=1; i < size(); i++) {
            cur_s += length((*this)[i]) ;
            s_.push_back(cur_s) ;
        }}
        double total_length = cur_s ;
        { for(unsigned int i=0; i < size(); i++) {
            s_[i] /= total_length ;
        }}
    }

    int MapBorder::find(double s) {
        ogf_assert(s >= 0 && s <= 1) ;
        for(unsigned int i=1; i<size(); i++) {
            if(s_[i-1] <= s && s_[i] >= s) {
                return i ;
            }
        }
        ogf_assert(false) ;
        return -1 ;
    }


    Map::Vertex* MapBorder::insert_vertex(double s) {
        int i = find(s) ;

        Map::Halfedge* h = (*this)[i] ;

        double s2 = (s - s_[i-1]) / (s_[i] - s_[i-1]) ;
        double s1 = 1.0 - s2 ;
        Point3d p1 = (*this)[i-1]->vertex()->point() ;
        Point3d p2 = (*this)[i]->vertex()->point() ;

        MapEditor editor(surface_) ;
        Map::Vertex* v = editor.split_edge(h, true) ;

        v->set_point(
            Point3d(
                s1 * p1.x() + s2 * p2.x(),
                s1 * p1.y() + s2 * p2.y(),
                s1 * p1.z() + s2 * p2.z()             
            )
        ) ;

        update() ;
        return v ;
    }

    void MapBorder::collapse_edge(int i) {
        ogf_assert(i >= 0 && i < int(size()) - 1) ;

        MapVertexLock is_locked(surface_) ;

        Map::Vertex* v1 = (*this)[i]->vertex() ;
        Map::Vertex* v2 = (*this)[i+1]->vertex() ;

        bool to_lock = is_locked[v1] || is_locked[v2] ;

        Point3d p1 = v1->point() ;
        Point3d p2 = v2->point() ;
        Point3d p (
            0.5 * (p1.x() + p2.x()),
            0.5 * (p1.y() + p2.y()),
            0.5 * (p1.z() + p2.z())
        ) ;
        Map::Halfedge* prev = (*this)[i]->prev() ;
        MapEditor editor(surface_) ;
        editor.erase_center_vertex(v1) ;
        editor.erase_center_vertex(v2) ;
        Map::Vertex* v = editor.split_edge(prev->next(), true) ;
        start_ = prev->next() ;

        v->set_point(p) ;
        if(to_lock) {
            is_locked[v] = true ;
        }
        update() ;
    }

    bool MapBorder::contains(Map::Halfedge* h) const {
        for(unsigned int i=1; i<size(); i++) {
            if((*this)[i] == h) {
                return true ;
            }
        }
        return false ;
    }

    void filter_small_edges(MapBorder& b, double min_dist) {
        std::cerr << "filter" << std::endl ;
        bool did_smthg = true ;
        while(did_smthg) {
            std::cerr << "size=" << b.size() << std::endl ;
            did_smthg = false ;
            for(int i=0; i < (int(b.size()) - 1); i++) {
                if((b.s(i+1) - b.s(i)) < min_dist) {
                    std::cerr << "collapse " << i << std::endl ;
                    b.collapse_edge(i) ;
                    did_smthg = true ;
                    break ;
                }
            }
        }
    }

//_________________________________________________________
    
    bool ExtMapEditor::zip_edge(
        Map::Vertex* src
    ) {
        Map::Halfedge* h1 = nil ;
        Map::Halfedge* h2 = nil ;
        Map::Halfedge* it = src->halfedge() ;
        do {
            if(it->is_border()) {
                if(h1 == nil) {
                    h1 = it ;
                } else {
                    h2 = it ;
                }
            }
            if(it->opposite()->is_border()) {
                if(h1 == nil) {
                    h1 = it->opposite() ;
                } else {
                    h2 = it->opposite() ;
                }
            }
            it = it->next_around_vertex() ;
        } while(it != src->halfedge()) ;
        if(h1 != nil && h2 != nil) {
            glue(h1,h2) ;
        }
        return true ;
    }
    
    bool ExtMapEditor::cut_edges(
        Map::Vertex* from, Map::Vertex* to
    ) {
        MapVertexAttribute<double> distance(target()) ;
        MapVertexAttribute<double> distance_estimate(target()) ;
        MapVertexAttribute<Map::Halfedge*> previous_node(target()) ;
        
        {FOR_EACH_VERTEX(Map, target(), it) {
            distance[it] = 1e19;
            distance_estimate[it] = 1e19;
            previous_node[it] = nil ;
        }}

        MapVertexHeap Q(target(), distance_estimate) ;
        Q.init_with_all_surface_vertices();        

        bool target_hit = false ;
        
        distance[from] = 0;
        distance_estimate[from] = ( from->point() - to->point() ).norm() ;
    
        Q.update_cost(from);

        while (!target_hit && !Q.empty()) {
            Map::Vertex* u = Q.pop() ;
            Map::Halfedge* h = u->halfedge();
            do {
                Map::Vertex* v = h->opposite()->vertex();
                if (distance[v] > distance[u] + (u->point() - v->point()).norm()) {
                    distance[v] = distance[u] + (u->point() - v->point()).norm();
                    distance_estimate[v] = distance[v] + ( v->point() - to->point() ).norm();
                    previous_node[v] = h;
                    if(Q.contains(v)) {
                        Q.update_cost(v) ;
                    }
                    if(v == to) {
                        target_hit = true ;
                    }
                }
                h=h->next_around_vertex() ;
            } while (h != u->halfedge());
        }

        if(!target_hit) {
            return false ;
        }

        Map::Vertex* v = to ;
        do {
            Map::Halfedge* h = previous_node[v] ;
            v = h->vertex();
            unglue(h,false) ;
        } while (v!=from) ;

        return true ;
    }

    void show_tex_border(MapBorder& b) {
        std::cerr << "size=" << b.size() << "," << b.s().size() << std::endl ;
        {for(unsigned int i = 0 ; i < b.size(); i++) {
            std::cerr << b.s(i) << " " ;
        }}
        std::cerr << std::endl ;
    }

    bool ExtMapEditor::match_borders(
        Map::Halfedge* h1,
        Map::Halfedge* h2,
        bool do_merge_geo_vertices,
        bool do_merge_tex_vertices
    ) {
        MapBorder b1(target(), h1) ;
        MapBorder b2(target(), h2) ;

        if(b1.contains(h2) || b2.contains(h1)) {
            Logger::err("MatchBorders")
                << "cannot match a border with itself" << std::endl ;
            return false ;
        }

        MapBorder::svector s1 = b1.s() ;
        MapBorder::svector s2 = b2.s() ;
        
        {for(unsigned int i = 1 ; i < s1.size() - 1 ; i++) {
            b2.insert_vertex(1.0 - s1[i]) ;
        }}

        {for(unsigned int i = 1 ; i < s2.size() - 1 ; i++) {
            b1.insert_vertex(1.0 - s2[i]) ;
        }}


// Does not work for the moment...
//filter_small_edges(b1, 0.05) ;
//filter_small_edges(b2, 0.05) ;
//   show_tex_border(b1) ;
//   show_tex_border(b2) ;

        if(do_merge_tex_vertices) {
            for(unsigned int i=0; i<b1.size(); i++) {
                unsigned int j = (b1.size() - i - 1) ;
                merge_tex_vertices(
                    b1[i]->vertex(),
                    b2[j]->vertex()
                ) ;

                MapVertexLock is_locked(target()) ;

                is_locked[ b1[i]->vertex() ] = true ;
                is_locked[ b2[i]->vertex() ] = true ;
            }
        }
        
        // Rem: this is done after merge_tex_vertices,
        //  since it destroys the halfedges !!!
        if(do_merge_geo_vertices) {
            for(unsigned int i=1; i<b1.size(); i++) {
                unsigned int j = (b1.size() - 1 - i) + 1 ;
                glue(b1[i], b2[j]) ;
            }
        }

        return true ;
    }

    void ExtMapEditor::get_border_between_anchors(
        Map* s,
        Map::Halfedge* h,
        MapBorder& out
    ) {
        MapVertexLock is_locked(s) ;
        out.clear() ;
        out.set_surface(s) ;
        Map::Halfedge* cur = prev_around_border(h) ;
        while(!is_locked[cur->vertex()]) {
            cur = prev_around_border(cur) ;
        }
        do {
            out.push_back(cur) ;
            cur = next_around_border(cur) ;
        } while(!is_locked[cur->vertex()]) ;
        out.push_back(cur) ;
    }


    Map::Halfedge* ExtMapEditor::next_around_border(
        Map::Halfedge* h
    ) {
        ogf_assert(h->is_border()) ;
        return h->next() ;
    }
    
    Map::Halfedge* ExtMapEditor::prev_around_border(
        Map::Halfedge* h
    ) {
        ogf_assert(h->is_border()) ;
        return h->prev() ;
    }

    
    void ExtMapEditor::compute_normals_around_vertex(Map::Vertex* v) {
        Vector3d n = Geom::vertex_normal(v) ;
        MapTexVertexNormal tex_vertex_normal(target()) ;
        Map::Halfedge* h = v->halfedge() ;
        do {
            tex_vertex_normal[h->tex_vertex()] = n ;
            h = h->next_around_vertex() ;
        } while(h != v->halfedge()) ;
        if(MapFacetNormal::is_defined(target())) {
            MapFacetNormal facet_normal(target()) ;
            Map::Halfedge* h = v->halfedge() ;
            do {
                if(h->facet() != nil) {
                    facet_normal[h->facet()] = Geom::facet_normal(h->facet()) ;
                    h = h->next_around_vertex() ;
                }
            } while(h != v->halfedge()) ;
        }
    }

    void ExtMapEditor::compute_normals_around_facet(Map::Facet* f) {
        if(MapFacetNormal::is_defined(target())) {
            MapFacetNormal facet_normal(target()) ;
            facet_normal[f] = Geom::facet_normal(f) ;
        }     
        MapTexVertexNormal tex_vertex_normal(target()) ;
        Map::Halfedge* h = f->halfedge() ;
        do {
            tex_vertex_normal[h->tex_vertex()] = 
                Geom::vertex_normal(h->vertex()) ;
            h = h->next() ;
        } while(h != f->halfedge()) ;
    }
    
    void ExtMapEditor::compute_normals_around_edge(Map::Halfedge* h) {
        MapTexVertexNormal tex_vertex_normal(target()) ;
        tex_vertex_normal[h->tex_vertex()] = Geom::vertex_normal(h->vertex()) ;
        tex_vertex_normal[h->prev()->tex_vertex()] = 
            Geom::vertex_normal(h->opposite()->vertex()) ;
        if(MapFacetNormal::is_defined(target())) {
            MapFacetNormal facet_normal(target()) ;
            if(h->facet() != nil) {
                facet_normal[h->facet()] = 
                    Geom::facet_normal(h->facet()) ;
            }
            if(h->opposite()->facet() != nil) {
                facet_normal[h->opposite()->facet()] = 
                    Geom::facet_normal(h->opposite()->facet()) ;
            }
        }     
        
    }


    //----------------------------- utilities for split_facet ------------------

    static bool halfedge_exists_between(Map::Vertex* v1, Map::Vertex* v2) {
        Map::Halfedge* h = v1->halfedge() ;
        do {
            if(h->opposite()->vertex() == v2) {
                return true ;
            }
            h = h->next_around_vertex() ;
        } while(h != v1->halfedge()) ;
        return false ;
    }

    typedef std::vector<Map::Vertex*> VertexLoop ;
    static void split_loop(
        const VertexLoop& loop, Map::Vertex*& v1, Map::Vertex*& v2,
        Map::Facet* f
    ) {
        std::vector<double> s ;
        s.push_back(0.0) ;
        double cur_s = 0 ;
        { for(unsigned int i=1; i<loop.size(); i++) {
            cur_s += (loop[i]->point() - loop[i-1]->point()).norm() ;
            s.push_back(cur_s) ;
        }}
        double total_length = cur_s ;
        total_length += (loop[loop.size() - 1]->point() - loop[0]->point()).norm() ;

        double best_rij = Numeric::big_double ;
        v1 = nil ; 
        v2 = nil ;
        
        for(unsigned int i=0; i<loop.size(); i++) {
            for(unsigned int j=0; j<loop.size(); j++) {

                // Do not split using vertices
                // already connected by an edge.
                if((i == j) || halfedge_exists_between(loop[i], loop[j])) {
                    continue ;
                }

                double dsij = ogf_min(s[i] - s[j], total_length - (s[i] - s[j])) ;
                double dxij = (loop[i]->point() - loop[j]->point()).norm() ;
                dsij = ogf_max(dsij, 1e-6) ;
                dxij = ogf_max(dxij, 1e-6) ;
                double rij = dxij / dsij ;
                if(rij < best_rij) {
                    best_rij = rij ;
                    v1 = loop[i] ;
                    v2 = loop[j] ;
                }
            }
        }
    }


    static Map::Halfedge* split_facet(Map* map, Map::Facet* f) {
        VertexLoop loop ;
        Map::Halfedge* h = f->halfedge() ; 
        do {
            loop.push_back(h->vertex()) ;
            h = h->next() ;
        } while(h != f->halfedge()) ;
        Map::Vertex* v1 ;
        Map::Vertex* v2 ;
        split_loop(loop, v1, v2, f) ;

        if(v1 == nil || v2 == nil) {
            return nil ;
        }
        
        Map::Halfedge* h1 = v1->halfedge() ;
        while(h1->facet() != f) {
            h1 = h1->next_around_vertex() ;
        }
        Map::Halfedge* h2 = v2->halfedge() ;
        while(h2->facet() != f) {
            h2 = h2->next_around_vertex() ;
        }

        MapEditor editor(map) ;
        editor.split_facet(h1, h2) ;
        return h1->next() ;
    }

//_____________________________________________________________________________

    void ExtMapEditor::subdivide_facet(Map::Facet* f, int max_nb_vertices) {
        int nb_vertices = f->nb_vertices() ;
        if(f->nb_vertices() <= max_nb_vertices) {
            return ;
        }
        Map::Halfedge* h = ::OGF::split_facet(target(), f) ;
        if(h != nil) {
            Map::Facet* f1 = h->facet() ;
            Map::Facet* f2 = h->opposite()->facet() ;

            if(
                f1->nb_vertices() >= nb_vertices 
            ) {
                MapVertexLock is_locked(target()) ;
                Map::Halfedge* h = f1->halfedge() ;
                do {
                    is_locked[h->vertex()] = true ;
                    h = h->next() ;
                } while(h != f1->halfedge()) ;
                Logger::warn("Tesselate Facet") << "encountered strange topology" << std::endl ;
                return ;
            }
            
            if(
                f2->nb_vertices() >= nb_vertices 
            ) {
                MapVertexLock is_locked(target()) ;
                Map::Halfedge* h = f2->halfedge() ;
                do {
                    is_locked[h->vertex()] = true ;
                    h = h->next() ;
                } while(h != f2->halfedge()) ;
                Logger::warn("Tesselate Facet") << "encountered strange topology" << std::endl ;
                return ;
            }

            subdivide_facet(f1, max_nb_vertices) ;
            subdivide_facet(f2, max_nb_vertices) ;
        }
    }

//_________________________________________________________

    Map::Facet* ExtMapEditor::create_facet_between_edges(
        Map::Halfedge* h1, Map::Halfedge* h2
    ) {
        Halfedge* k2 = make_polygon(4) ;
        if(!k2->is_border()) {
            k2 = k2->opposite() ;
        }
        
        k2->vertex()->set_point(h2->prev()->vertex()->point()) ;
        k2->prev()->vertex()->set_point(h2->vertex()->point()) ;

        Map::Halfedge* k1 = k2->next()->next() ;

        k1->vertex()->set_point(h1->prev()->vertex()->point()) ;
        k1->prev()->vertex()->set_point(h1->vertex()->point()) ;

        Map::Halfedge* k1n = k1->next();
        Map::Halfedge* k2n = k2->next();

        glue(h2,k2) ;
        glue(h1,k1) ;

        if(k1n->next()->next() == k1n) {
            zip_edge(k1n->vertex()) ;
        }

        if(k2n->next()->next() == k2n) {
            zip_edge(k2n->vertex()) ;
        }

        return k1->facet() ;

    }
    

}

