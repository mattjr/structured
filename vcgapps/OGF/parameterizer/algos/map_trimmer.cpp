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

#include <OGF/parameterizer/algos/map_trimmer.h>
/*
#include <OGF/anisotropy/algos/shortest_path.h>
*/

#include <OGF/parameterizer/algos/topology.h>
#include <OGF/cells/map/geometry.h>
#include <OGF/cells/map/map_builder.h>
#include <OGF/cells/map/map_attributes.h>
#include <OGF/math/geometry/polygon3d.h>
#include <OGF/basic/debug/progress.h>

#include <stack>
#include <algorithm>

namespace OGF {

    static bool intersect_edges(
        Map::Facet* f, 
        Graph::Vertex* v1, Graph::Vertex* v2,
        Graph::Vertex* w1, Graph::Vertex* w2,
        Point3d& I
    ) ;

    static bool intersect_edges(
        Map::Facet* f, Graph::Halfedge* h1, Graph::Halfedge* h2,
        Point3d& I
    ) ;


//________________________________________________________________________________________

    void MapTrimmer::intersect_trimming_curve() {
        std::cerr << "vertices" << std::endl ;
        intersect_trimming_curve_in_vertices() ;
        std::cerr << "edges" << std::endl ;
        intersect_trimming_curve_in_edges(1e-6) ;
        std::cerr << "facets" << std::endl ;
        intersect_trimming_curve_in_facets() ;
        std::cerr << "done." << std::endl ;
    }

    // TODO1: when there is more than one intersection per facet, we still got
    // problems (missing edges, embedding not conform)
    // TODO2: stars_intersection should return the element of
    // smallest dimension

    void MapTrimmer::intersect_trimming_curve_in_facets() {
        MapFacetAttribute<std::vector<Graph::Halfedge*> > embedded_segs(map_) ;

        FOR_EACH_EDGE(Graph, graph_, it) {
            const MapCellEmbedding& e1 = embedding_[it->vertex()] ;
            const MapCellEmbedding& e2 = embedding_[it->opposite()->vertex()] ;
            
            MapCellEmbedding stars_isect = Topo::segment_embedding(map_, e1, e2) ;

            // What do we do for "in_vertex" and "in_edge" cases ?
            
            // In facet
            if(stars_isect.dimension() == 2) {
                embedded_segs[stars_isect.facet()].push_back(it) ;
            }
            
        }

        FOR_EACH_FACET(Map, map_, it) {
            if(merge_intersections_in_facets_) {
                intersect_trimming_curve_in_facet_and_merge(it, embedded_segs[it]) ;
            } else {
                intersect_trimming_curve_in_facet(it, embedded_segs[it]) ;
            }
        }
    }

    void MapTrimmer::intersect_trimming_curve_in_facet_and_merge(
        Map::Facet* f, std::vector<Graph::Halfedge*>& segs
    ) {
        std::set<Graph::Halfedge*> intersected_edges ;
        int nb_intersections = 0 ;
        Point3d I(0,0,0) ;
        for(unsigned int i=0; i<segs.size(); i++) {
            for(unsigned int j=0; j<i; j++) {
                Graph::Halfedge* h1 = segs[i] ;
                Graph::Halfedge* h2 = segs[j] ;
                Point3d cur_I ;
                if(intersect_edges(f, h1, h2, cur_I)) {
                    nb_intersections++ ;
                    I = I + (cur_I - Origin()) ;
                    intersected_edges.insert(h1) ;
                    intersected_edges.insert(h2) ;
                }
            }
        }

        if(nb_intersections != 0) {
            I.set_x(I.x() / double(nb_intersections)) ;
            I.set_y(I.y() / double(nb_intersections)) ;
            I.set_z(I.z() / double(nb_intersections)) ;
            Graph::Vertex* x = graph_editor_.new_vertex(I) ;
            embedding_[x] = MapCellEmbedding(map_, f) ;
            int kind = 1 ;
            for(
                std::set<Graph::Halfedge*>::iterator jt = intersected_edges.begin() ;
                jt != intersected_edges.end(); jt++
            ) {
                insert_intersection(*jt, x, kind) ;
                kind++ ;
            }
        }
    }

    void MapTrimmer::intersect_trimming_curve_in_facet(
        Map::Facet* f, std::vector<Graph::Halfedge*>& segs
    ) {
        std::vector<IsectEdges> edges ;
        {for(unsigned int i = 0; i<segs.size(); i++) {
            edges.push_back(IsectEdges()) ;
            edges.rbegin()->push_back(segs[i]) ;
        }}
        for(unsigned int i=0; i<edges.size(); i++) {
            for(unsigned int j=i+1; j<edges.size(); j++) {
                IsectEdges& e1 = edges[i] ;
                IsectEdges& e2 = edges[j] ;
                Point3d I ;
                if(intersect_edges(f, e1.v1(), e1.v2(), e2.v1(), e2.v2(), I)) {
                    Graph::Vertex* gv = graph_editor_.new_vertex(I) ;
                    embedding_[gv] = MapCellEmbedding(map_, f) ;
                    insert_intersection(e1, gv) ;
                    insert_intersection(e2, gv) ;
                }
            }
        }
    }

//________________________________________________________________________________________

    static Point2d project_point(Map::Facet* f, const Point3d& p1) {
        Vector3d X = Geom::vector(f->halfedge()) ;
        Vector3d Y = Geom::vector(f->halfedge()->next()) ;
        Vector3d Z = X ^ Y ;
        Y = Z ^ X ;
        X.normalize() ;
        Y.normalize() ;
        Vector3d V = p1 - f->halfedge()->vertex()->point() ;
        return Point2d(V*X, V*Y) ;
    }

    static Point3d unproject_point(Map::Facet* f, const Point2d& p1) {
        Vector3d X = Geom::vector(f->halfedge()) ;
        Vector3d Y = Geom::vector(f->halfedge()->next()) ;
        Vector3d Z = X ^ Y ;
        Y = Z ^ X ;
        X.normalize() ;
        Y.normalize() ;
        return f->halfedge()->vertex()->point() + p1.x() * X + p1.y() * Y ;
    }

    static inline bool intersect_segments(
        const Point2d& p1, const Point2d& p2,
        const Point2d& q1, const Point2d& q2,
        Point2d& result
    ) {
        
        Vector2d Vp = p2 - p1 ;
        Vector2d Vq = q2 - q1 ;
        Vector2d pq = q1 - p1 ;
        
        double a =  Vp.x() ;
        double b = -Vq.x() ;
        double c =  Vp.y() ;
        double d = -Vq.y() ;

        double delta = a*d-b*c ;
        if(delta == 0.0) {
            return false ;
        }
            
        double tp = ( d * pq.x() - b * pq.y()) / delta ;
        double tq = (-c * pq.x() + a * pq.y()) / delta ;        

        result = Point2d(
            (1.0 - tp) * p1.x() + tp * p2.x(),
            (1.0 - tp) * p1.y() + tp * p2.y()
        ) ;

        return ((tp >= 0) && (tq >= 0) && (tp <= 1) && (tq <= 1)) ;
    }
    


    static bool intersect_edges(
        Map::Facet* f, 
        Graph::Vertex* v1, Graph::Vertex* v2,
        Graph::Vertex* w1, Graph::Vertex* w2,
        Point3d& I
    ) {

        // Do not intersect adjacent segs
        if(
            v1 == w1 || v1 == w2 ||
            v2 == w1 || v2 == w2
        ) {
            return false ;
        }


        Point2d p1 = project_point(f, v1->point()) ;
        Point2d p2 = project_point(f, v2->point()) ;
        Point2d q1 = project_point(f, w1->point()) ;
        Point2d q2 = project_point(f, w2->point()) ;
        Point2d J ;
        if(!intersect_segments(p1,p2,q1,q2,J)) {
            return false ;
        }
        I = unproject_point(f,J) ;
        return true ;
    }

    static bool intersect_edges(
        Map::Facet* f, Graph::Halfedge* h1, Graph::Halfedge* h2,
        Point3d& I
    ) {
        Graph::Vertex* v1 = h1->vertex() ;
        Graph::Vertex* v2 = h1->opposite()->vertex() ;
        Graph::Vertex* w1 = h2->vertex() ;
        Graph::Vertex* w2 = h2->opposite()->vertex() ;
        return intersect_edges(f,v1,v2,w1,w2,I) ;
    }

    double MapTrimmer::IsectEdges::local_coord(Graph::Halfedge* h, Graph::Vertex* v) {
        const Point3d& p1 = h->opposite()->vertex()->point() ;
        const Point3d& p2 = h->vertex()->point() ;
        Vector3d V = p2 - p1 ;
        double Vnorm2 = V.norm2() ;
        return (v->point() - p1) * V / Vnorm2 ;
    }

    void MapTrimmer::insert_intersection(MapTrimmer::IsectEdges& edges, Graph::Vertex* x) {
        for(IsectEdges::iterator it = edges.begin(); it != edges.end(); it++) {
            double s = edges.local_coord( *it, x) ;
            if(s >= 0 && s <= 1) {
                
                Graph::Halfedge* h1 = *it ;
                Graph::Vertex* v1 = h1->vertex() ;
                Graph::Vertex* v2 = h1->opposite()->vertex() ;
                ogf_assert(v1 != v2) ;
                Graph::Halfedge* new_h1 = check_connect_vertices(v1,x) ;
                graph_editor_.copy_attributes(new_h1, h1) ;
                graph_editor_.copy_attributes(new_h1->opposite(), h1) ;
                Graph::Halfedge* new_h2 = check_connect_vertices(v2,x) ;
                graph_editor_.copy_attributes(new_h2, h1) ;
                graph_editor_.copy_attributes(new_h2->opposite(), h1) ;
                graph_editor_.erase_edge(h1) ;

                *it = new_h1 ;
                edges.insert(it, new_h2->opposite()) ;

                return ;
            }
        }
//        ogf_assert_not_reached ;
        std::cerr << "!! Did not find intersection in edge" << std::endl ;
    }

//________________________________________________________________________________________


    MapTrimmer::MapTrimmer(Graph* trimming_curve) : map_editor_(nil) {
        lock_branchings_ = false ;
        graph_ = trimming_curve ;
        ogf_assert(embedding_.is_defined(trimming_curve,"embedding")) ;
        embedding_.bind(trimming_curve,"embedding") ;
        is_corner_.bind(trimming_curve,"is_corner") ;
        map_ = nil ;
        FOR_EACH_VERTEX(Graph, graph_, it) {
            if(embedding_[it].map() != nil) {
                if(map_ == nil) {
                    map_ = embedding_[it].map() ;
                } else {
                    // Check that all vertices are embedded in the same map
                    ogf_assert(embedding_[it].map() == map_) ;
                }
            }
        }
        ogf_assert(map_ != nil) ;
        embedded_.bind(map_, "embedded") ;
        map_editor_.set_target(map_) ;
        graph_editor_.set_target(graph_) ;
        kind_.bind(graph_, "kind") ;
        if(normal_.is_defined(graph_, "normal")) {
            normal_.bind(graph_, "normal") ;
        }
        merge_intersections_in_facets_ = false ;
    }
    
    MapTrimmer::~MapTrimmer() {
    }


    static bool have_common_neighbors(Graph::Vertex* v1, Graph::Vertex* v2) {
        Graph::Halfedge* h1 = v1->halfedge() ;
        do {

            Graph::Halfedge* h2 = v2->halfedge() ;
            do {
                if(h1->opposite()->vertex() == h2->opposite()->vertex()) {
                    return true ;
                }
                h2 = h2->next_around_vertex() ;
            } while(h2 != v2->halfedge()) ;

            h1 = h1->next_around_vertex() ;
        } while(h1 != v1->halfedge()) ;

        return false ;
    }

    void MapTrimmer::intersect_trimming_curve_in_edges(double threshold, bool extremities_only) {

        std::vector< MapTrimmerVerticesInEdge> to_embed ;            
        MapHalfedgeAttribute< std::vector<Graph::Vertex*> > embedded(map_) ;

        FOR_EACH_VERTEX(Graph, graph_, it) {
            MapCellEmbedding& em = embedding_[it] ;
            if(em.dimension() == 1) {
                Map::Halfedge* h = em.halfedge()->edge_key() ;
                embedded[h].push_back(it) ;
                if(it->degree() == 1) {
//                    std::cerr << "found extremity embedded in edge" << std::endl ;
                }
            }
        }

        FOR_EACH_EDGE(Map, map_, it) {
            
            double L = Geom::edge_length(it) ;
            L *= threshold ;

            std::vector<Graph::Vertex*>& vertices = embedded[it] ;
            if(vertices.size() >= 2) {
                for(unsigned int i=0; i<vertices.size(); i++) {
                    for(unsigned int j=i+1; j<vertices.size(); j++) {
                        if(
                            (vertices[i] != vertices[j]) &&
                            ((vertices[i]->point() - vertices[j]->point()).norm() < L)
                        ) {
                            if(
                                extremities_only && 
                                vertices[i]->degree() != 1 && 
                                vertices[j]->degree() != 1
                            ) {
                                continue ;
                            }
                            //   When an intersection in a facet is near an edge,
                            // we do not want to merge the two vertices in the edge,
                            // since this would alter the structure of the cut-graph
                            // in a region where it is regular.
                            if(have_common_neighbors(vertices[i], vertices[j])) {
                                continue ;
                            }
                            std::cerr << "   Intersection in edge, merging vertices" << std::endl ;
                            Graph::Vertex* gv = merge_vertices(vertices[i], vertices[j]) ;
                            std::cerr << "    merged" << std::endl ;
                            vertices[i] = gv ;
                            vertices[j] = gv ;
                        }
                    }
                }
            }
        }
        
        
        // Detect intersections between graph edge and graph edge 
        // embedded in edge that already exists in the triangulation
        {
            std::vector<Graph::Halfedge*> halfedges_to_split ;
            std::vector<Graph::Vertex*>   vertices_to_insert ;
            FOR_EACH_EDGE(Graph, graph_, it) {
                ogf_assert(it->opposite() != nil) ;
                Graph::Vertex* gv1 = it->vertex() ;
                Graph::Vertex* gv2 = it->opposite()->vertex() ;
                MapCellEmbedding& em1 = embedding_[gv1] ;
                MapCellEmbedding& em2 = embedding_[gv2] ;
                if(em1.dimension() == 0 && em2.dimension() == 0) {
                    Map::Halfedge* h = Topo::find_halfedge_between(em2.vertex(), em1.vertex()) ;
                    ogf_assert(h != nil) ;
                    std::vector<Graph::Vertex*>& vertices = embedded[h->edge_key()] ;
                    for(unsigned int i=0; i<vertices.size(); i++) {
                        vertices_to_insert.push_back(vertices[i]) ;
                        halfedges_to_split.push_back(it) ;
                    }
                }
            }
            ogf_assert(halfedges_to_split.size() == vertices_to_insert.size()) ;
            std::set<Graph::Halfedge*> splitted_halfedges ;
            for(unsigned int i=0; i<halfedges_to_split.size(); i++) {
                Graph::Halfedge* h = halfedges_to_split[i] ;
                Graph::Vertex* v = vertices_to_insert[i] ;
                if(splitted_halfedges.find(h) != splitted_halfedges.end()) {
                    std::cerr << "h already splitted" << std::endl ;
                    continue ;
                }
                if(splitted_halfedges.find(h->opposite()) != splitted_halfedges.end()) {
                    std::cerr << "h->opposite() already splitted" << std::endl ;
                    continue ;
                }
                splitted_halfedges.insert(h) ;
                insert_intersection(h, v) ;
            }
        }

    }

    void MapTrimmer::intersect_trimming_curve_in_vertices() {
        GraphVertexLock is_isect(graph_) ;
        MapVertexAttribute<std::vector<Graph::Vertex*> > embedded_vertices(map_) ;
        FOR_EACH_VERTEX(Graph, graph_, it) {
            if(embedding_[it].dimension() == 0) {
                embedded_vertices[embedding_[it].vertex()].push_back(it) ;
            }
        }
        FOR_EACH_VERTEX(Map, map_, it) {
            std::vector<Graph::Vertex*>& vertices = embedded_vertices[it] ;
            if(vertices.size() > 1) {
                Graph::Vertex* v = vertices[0] ;
                for(unsigned int i=1; i<vertices.size(); i++) {
                    std::cerr << "Merging vertices (embedded in same vertex)" << std::endl ;
                    v = merge_vertices(v, vertices[i]) ;
                    is_isect[v] = true ;
                }
            }
        }
    }

    Graph::Vertex* MapTrimmer::insert_intersection(
        Graph::Halfedge* h1, Graph::Halfedge* h2, const Point3d& I
    ) {
        Graph::Vertex* x = graph_editor_.new_vertex(I) ;
        insert_intersection(h1,x,1) ;
        insert_intersection(h2,x,2) ;
        return x ;
    }

    Graph::Vertex* MapTrimmer::insert_intersection(Graph::Halfedge* h1, const Point3d& I, int kind) {
        Graph::Vertex* x = graph_editor_.new_vertex(I) ;
        insert_intersection(h1,x,kind) ;
        return x ;  
    }

    void MapTrimmer::insert_intersection(Graph::Halfedge* h1, Graph::Vertex* x, int kind) {
        kind_[h1] = kind ;
        kind_[h1->opposite()] = kind ;
        Graph::Vertex* v1 = h1->vertex() ;
        Graph::Vertex* v2 = h1->opposite()->vertex() ;
        ogf_assert(v1 != v2) ;
        Graph::Halfedge*  h = nil ;
        h = check_connect_vertices(v1,x) ;
        graph_editor_.copy_attributes(h, h1) ;
        graph_editor_.copy_attributes(h->opposite(), h1) ;
        h = check_connect_vertices(v2,x) ;
        graph_editor_.copy_attributes(h, h1) ;
        graph_editor_.copy_attributes(h->opposite(), h1) ;
        graph_editor_.erase_edge(h1) ;
    }

    void MapTrimmer::fix_trimming_curve_singularities(bool merge) {
        MapFacetAttribute<bool>  f_is_singular(map_, "is_singular") ;
        MapHalfedgeAttribute<bool> h_is_singular(map_, "is_singular") ;
        
        std::vector<Graph::Halfedge*> to_remove ;
        FOR_EACH_EDGE(Graph, graph_, it) {
            const MapCellEmbedding& from_e = embedding_[it->vertex()] ;
            const MapCellEmbedding& to_e = embedding_[it->opposite()->vertex()] ;
            MapCellEmbedding em = Topo::segment_embedding(map_, from_e, to_e) ;
            if(em.dimension() == 2 && f_is_singular[em.facet()]) {
                to_remove.push_back(it) ;
            }
            if(em.dimension() == 1 && h_is_singular[em.halfedge()]) {
                to_remove.push_back(it) ;
            }
        }
        GraphEditor editor(graph_) ;
        for(unsigned int i=0; i<to_remove.size(); i++) {
            editor.erase_edge(to_remove[i]) ;
        }

        // Update "embedded" attribute in surface
        // (needed by next phase)
        FOR_EACH_VERTEX(Map, map_, it) {
            embedded_[it] = nil ;
        }
        FOR_EACH_VERTEX(Graph, graph_, it) {
            if(embedding_[it].dimension() == 0) {
                embedded_[embedding_[it].vertex()] = it ;
            }
        }

        MapHalfedgeAttribute< std::vector<Graph::Vertex*> > vertices_in_edge(map_) ;
        FOR_EACH_VERTEX(Graph, graph_, it) {
            if(embedding_[it].dimension() == 1) {
                vertices_in_edge[embedding_[it].halfedge()->edge_key()].push_back(it) ;
            }
        }

        if(merge) {
            FOR_EACH_FACET(Map, map_, it) {
                if(f_is_singular[it]) {
                    std::vector<Graph::Vertex*> to_connect ;
                    Map::Halfedge* h = it->halfedge() ;
                    do {
                        for(unsigned int i=0; i<vertices_in_edge[h->edge_key()].size(); i++) {
                            to_connect.push_back(vertices_in_edge[h->edge_key()][i]) ;
                        }
                        if(embedded_[h->vertex()] != nil) {
                            to_connect.push_back(embedded_[h->vertex()]) ;
                        }
                        h = h->next() ;
                    } while(h != it->halfedge()) ;
                    MapCellEmbedding em(map_, it) ;
                    connect_vertices(editor, to_connect, em) ;
                }
            }

            // Process isolated singular edges
            FOR_EACH_EDGE(Map, map_, it) {
                if(h_is_singular[it]) {
                    if(it->facet() != nil && f_is_singular[it->facet()]) continue ;
                    if(it->opposite()->facet() != nil && f_is_singular[it->opposite()->facet()]) continue ;
                    std::vector<Graph::Vertex*> to_connect ;
                    for(unsigned int i=0; i<vertices_in_edge[it->edge_key()].size(); i++) {
                        to_connect.push_back(vertices_in_edge[it->edge_key()][i]) ;
                    }
                    if(embedded_[it->vertex()] != nil) {
                        to_connect.push_back(embedded_[it->vertex()]) ;
                    }
                    if(embedded_[it->prev()->vertex()] != nil) {
                        to_connect.push_back(embedded_[it->prev()->vertex()]) ;
                    }
                    MapCellEmbedding em(map_, it) ;
                    connect_vertices(editor, to_connect, em) ;
                }
            }
        }
    }

    void MapTrimmer::connect_vertices(
        GraphEditor& editor, std::vector<Graph::Vertex*>& to_connect, MapCellEmbedding& em
    ) {
        if(to_connect.size() >= 2) {
            double x = 0 ;
            double y = 0 ;
            double z = 0 ;
            for(unsigned int i=0; i<to_connect.size(); i++) {
                x += to_connect[i]->point().x() ;
                y += to_connect[i]->point().y() ;
                z += to_connect[i]->point().z() ;
            }
            double c = double(to_connect.size()) ;
            Point3d g = Point3d(x/c, y/c, z/c) ;
            Graph::Vertex* gv = editor.new_vertex(g) ;
            embedding_[gv] = em ; 
            for(unsigned int i=0; i<to_connect.size(); i++) {
                editor.connect_vertices(gv, to_connect[i]) ;
            }                    
        }
    }

    static Point2d interpolate_tex_coord(Map::Halfedge* h, const Point3d& p) {
        Vector3d e = Geom::vector(h) ;
        Vector3d v = p - h->opposite()->vertex()->point() ;
        double w2 = (v * e) / e.norm2() ;
        ogf_clamp(w2, 0.0, 1.0) ;
        double w1 = 1.0 - w2 ;
        const Point2d& uv1 = h->prev()->tex_coord() ;
        const Point2d& uv2 = h->tex_coord() ;
        return Point2d(
            w1 * uv1.x() + w2 * uv2.x(),
            w1 * uv1.y() + w2 * uv2.y()
        ) ;
    }

    static Point2d interpolate_tex_coord(Map::Facet* f, const Point3d& p) {
        return Geom::facet_xyz_to_uv(f, p) ;
    }

    void MapTrimmer::check_conformity() {
        bool ok = true ;
        std::cerr << "Check conformity ----------------------" << std::endl ;
        {FOR_EACH_VERTEX(Graph, graph_, it) {
            if(embedding_[it].map() == nil) {
                std::cerr << "Vertex is not embedded" << std::endl ;
                ok = false ;
            }
        }}
        {FOR_EACH_EDGE(Graph, graph_, it) {
            Graph::Vertex* v1 = it->vertex() ;
            Graph::Vertex* v2 = it->opposite()->vertex() ;
            if(v1 == v2) {
                std::cerr << "Edge connects vertex to itself" << std::endl ;
            }
            if(Topo::segment_embedding(map_,embedding_[v1], embedding_[v2]).is_nil()) {
                std::cerr << "Stars do not intersect"
                          << " dim1= " << embedding_[v1].dimension()
                          << " dim2= " << embedding_[v2].dimension()
                          << std::endl ;
                ok = false ;
            }
            if(
                embedding_[v1].dimension() == 0 && embedding_[v2].dimension() == 0 &&
                embedding_[v1].vertex() == embedding_[v2].vertex()
            ) {
                std::cerr << "Both ends are embedded in the same vertex" << std::endl ;
                ok = false ;
            }
        }}
        if(ok) {
            std::cerr << "Embedding is OK" << std::endl ;
        } else {
            std::cerr << "!!! EMBEDDING IS NOT CONFORM !!!" << std::endl ;
        }
        std::cerr << "---------------------------------------" << std::endl ;

        {
            int count = 0 ;
            FOR_EACH_HALFEDGE(Graph, graph_, it) {
                Graph::Vertex* v1 = it->vertex() ;
                Graph::Vertex* v2 = it->opposite()->vertex() ;
                if(v1->degree() > 2 && embedding_[v2].dimension() == 2) {
                    count++ ;
                }
            }
//            if(count != 0) {
//                std::cerr << count << " vert(ex)(ices) for which" << std::endl; 
//                std::cerr << "    neighbor of branching vertex embedded in facet" << std::endl ;
//                std::cerr << "    (this is not a problem with the new ordered_star() function)" << std::endl ;
//            }
        }

        {
            std::vector<Graph::Vertex*> to_delete ;
            FOR_EACH_VERTEX(Graph, graph_, it) {
                if(it->halfedge() == nil) {
                    std::cerr << "isolated vertex in graph" << std::endl ;
                    to_delete.push_back(it) ;
                }
            }
            GraphEditor editor(graph_) ;
            for(unsigned int i=0; i<to_delete.size(); i++) {
                editor.erase_vertex(to_delete[i]) ;
            }
        }
    }

//__________________________________________________________________________________________________

    void MapTrimmer::create_map_from_trimming_line(Map* target, bool squares_only) {

        MapEditor editor(map_) ;
        editor.merge_all_tex_vertices() ;

        GraphVertexLock graph_is_locked(graph_) ;
        FOR_EACH_VERTEX(Graph, graph_, it) {
            graph_is_locked[it] = (it->degree() > 2) ;
        }

        MapBuilder builder(target) ;
        builder.begin_surface() ;
        GraphVertexAttribute<int> vertex_id(graph_) ;
        {
            int cur_id = 0 ;
            FOR_EACH_VERTEX(Graph, graph_, it) {
                if(graph_is_locked[it]) {
                    builder.add_vertex(it->point()) ;
                    vertex_id[it] = cur_id ;
                    cur_id++ ;
                } else {
                    vertex_id[it] = -1 ;
                }
            }
        }

        GraphHalfedgeAttribute<bool> is_visited(graph_) ;
        FOR_EACH_HALFEDGE(Graph, graph_, it) {
            if(!is_visited[it]) {
                std::vector<int> facet_vertices ;
                int nb_corners = 0 ;

                Graph::Halfedge* h = it ;
                do {
                    int vid = vertex_id[h->vertex()] ;
                    if(vid != -1) {
                        // Note: we avoid inserting the same vertex in the same facet several times.
                        if(
                            std::find(
                                facet_vertices.begin(), facet_vertices.end(), vid
                            ) == facet_vertices.end()
                        ) {
                            facet_vertices.push_back(vid) ;
                            if(is_chart_corner(h)) {
                                nb_corners++ ;
                            }
                        }
                    }
                    is_visited[h] = true ;
                    h = ordered_next(h) ;
                } while(h != it) ;

                if(nb_corners != 4) {
                    std::cerr << "nb_corners = " << nb_corners << std::endl ;
                }

                if(!squares_only || nb_corners == 4) {
                    if(facet_vertices.size() < 3 ) {
                        continue ;
                    }
                    builder.begin_facet() ;
                    for(unsigned int i=0; i<facet_vertices.size(); i++) {
                        builder.add_vertex_to_facet(facet_vertices[i]) ;
                    }
                    builder.end_facet() ;
                }
            }
        }
        builder.end_surface() ;
    }

    void MapTrimmer::get_star_border(
        const MapCellEmbedding& em, std::vector<Map::Halfedge*>& star_border
    ) {
        star_border.clear() ;
        switch(em.dimension()) {
        case 0: {
            Map::Vertex* mv = em.vertex() ;
            Map::Halfedge* mh = mv->halfedge() ;
            do {
                if(mh->vertex() == mv) {
                    mh = mh->opposite()->next() ;
                }
                star_border.push_back(mh) ;
                mh = mh->next() ;
            } while(mh != mv->halfedge()) ;
        } break ;
        case 1: {
            Map::Halfedge* mhstart = em.halfedge() ;
            Map::Halfedge* mh = mhstart->next() ;
            do {
                star_border.push_back(mh) ;
                mh = mh->next() ;
            } while(mh != mhstart) ;
            mhstart = mhstart->opposite() ;
            mh = mhstart->next() ;
            do {
                star_border.push_back(mh) ;
                mh = mh->next() ;
            } while(mh != mhstart) ;
        } break ;
        case 2: {
            Map::Facet* mf = em.facet() ;
            Map::Halfedge* mh = mf->halfedge() ;
            do {
                star_border.push_back(mh) ;
                mh = mh->next() ;
            } while(mh != mf->halfedge()) ;
        } break ;
        default: {
            ogf_assert(false) ;
        } break ;
        }
    }

    namespace Geom {
        inline Vector3d vector(const Graph::Halfedge* h) {
            return h->vertex()->point() - h->opposite()->vertex()->point() ;
        }
    }

    struct OrderedStarEdge {
        OrderedStarEdge(Graph::Halfedge* h_in, double a_in) : h(h_in), alpha(a_in) { }
        Graph::Halfedge* h ;
        double alpha ;
        bool operator<(const OrderedStarEdge& rhs) const { 
            return (alpha < rhs.alpha) ;
        }
    } ;

    void MapTrimmer::get_graph_ordered_star_from_geometry(
        Graph::Vertex* gv, std::vector<Graph::Halfedge*>& gstar
    ) {
        Vector3d Z = normal(gv) ;
        Vector3d X = Geom::perpendicular(Z) ;
        Vector3d Y = Z ^ X ; Y.normalize() ;
        X = Y ^ Z ;          X.normalize() ;

        std::vector<OrderedStarEdge> ordered_star ;

        Graph::Halfedge* h = gv->halfedge() ;
        do {
            Vector3d V = Geom::vector(h->opposite()) ;
            if((V.norm() < 1e-10 && h->opposite()->vertex()->degree() == 2)) {
                std::cerr << "Using two-rings neighbor to disambiguate" << std::endl ;
                V = h->opposite()->next_around_vertex()->opposite()->vertex()->point() - gv->point() ;
            }
            double alpha = Geom::angle(X,V) ;
            if(V*Y < 0) {
                alpha = -alpha ;
            }
            ordered_star.push_back(OrderedStarEdge(h, alpha)) ;
            h = h->next_around_vertex() ;
        } while(h != gv->halfedge()) ;
        std::sort(ordered_star.begin(), ordered_star.end()) ;
        for(unsigned int i=0; i<ordered_star.size(); i++) {
            gstar.push_back(ordered_star[i].h) ;
        }
    }

    void MapTrimmer::get_graph_ordered_star(Graph::Vertex* gv, std::vector<Graph::Halfedge*>& gstar) {

        // Step 1: get the embeddings of the neighbors of the vertex.
        std::map<Map::Halfedge*, MapTrimmerVerticesInEdge> gv_in_mh ; 
        std::map<Map::Vertex*, Graph::Vertex*> gv_in_mv ; 
        Graph::Halfedge* gh = gv->halfedge() ;
        do {
            Graph::Vertex* cur = gh->opposite()->vertex() ;
            MapCellEmbedding& em = embedding_[cur] ;
            switch(em.dimension()) {
            case 0: {
                gv_in_mv[em.vertex()] = cur ;
            } break ;
            case 1: {
                Map::Halfedge* mh = em.halfedge() ;
                gv_in_mh[mh].push_back( MapTrimmerVertexInEdge(cur,mh) ) ;
                gv_in_mh[mh->opposite()].push_back( MapTrimmerVertexInEdge(cur,mh->opposite()) ) ;
            } break ;
            default: {
                std::cerr << "[ordered star] embedding dim = " << em.dimension() << std::endl ;
                std::cerr << "[ordered star] neighbor of branching vertex embedded in facet"
                          << " - using geometry" << std::endl ;
                get_graph_ordered_star_from_geometry(gv, gstar) ;
                return ;
            } break ;
            }
            gh = gh->next_around_vertex() ;
        } while(gh != gv->halfedge()) ;

        // Step 2: traverse the cells in the border of the star of the map vertex 
        // and list the graph vertices embedded in each cell.

        std::vector<Map::Halfedge*> m_star_border ;
        get_star_border(embedding_[gv], m_star_border) ;
        for(unsigned int i=0; i<m_star_border.size(); i++) {
            Map::Halfedge* mh = m_star_border[i] ;

            // List (and sort) the vertices embedded in the edge (if any)
            {
                std::map<Map::Halfedge*, MapTrimmerVerticesInEdge>::iterator it = 
                    gv_in_mh.find(mh) ;
                if(it != gv_in_mh.end()) {
                    MapTrimmerVerticesInEdge& em = it->second ;
                    em.sort() ;
                    for(unsigned int j=0; j<em.size(); j++) {
                        Graph::Halfedge* gh = Topo::find_halfedge_between(em[j].v, gv) ;
                        if(gh != nil) {
                            gstar.push_back(gh) ;
                        } else {
                            std::cerr << "did not find star edge" << std::endl ;
                        }
                    }
                }
            }
            
            // List the vertex embedded in the edge extremity (if any)
            {
                std::map<Map::Vertex*, Graph::Vertex*>::iterator it = 
                    gv_in_mv.find(mh->vertex()) ;
                if(it != gv_in_mv.end()) {
                    Graph::Halfedge* gh = Topo::find_halfedge_between(it->second, gv) ;
                    if(gh != nil) {
                        gstar.push_back(gh) ;
                    } else {
                        std::cerr << "did not find star edge" << std::endl ;
                    }
                }
            }
        }
    }
    
    Graph::Halfedge* MapTrimmer::ordered_next(Graph::Halfedge* h) {
        // If h->vertex() is of degree 1 or 2, there is no ambiguity,
        // else the order is computed from the embedding of the
        // vertex neighbors (see get_graph_ordered_star()).
        Graph::Halfedge* result = nil ;
        switch(h->vertex()->degree()) {
        case 1: {
            std::cerr << "Dangling edge detected !!" << std::endl ;
            result = h->opposite() ;
        } break ;
        case 2: {
            result = h->next_around_vertex()->opposite() ;
        } break ;
        default: {
            std::vector<Graph::Halfedge*> ordered_star ;
            get_graph_ordered_star_from_geometry(h->vertex(), ordered_star) ;
//            get_graph_ordered_star(h->vertex(), ordered_star) ;
            for(unsigned int i=0; i<ordered_star.size(); i++) {
                if(ordered_star[i] == h) {
                    int k = (i + 1) % ordered_star.size() ;
                    result = ordered_star[k]->opposite() ;
                    break ;
                }
            }
            if(result == nil) {
                std::cerr << "this edge not found in ordered star" << std::endl ;
            }
            ogf_assert(result != nil) ;
        } break ;
        }
        return result ;
    }

//___________________________________________________________________________________________________________

    static bool halfedge_exists_between(Graph::Vertex* v1, Graph::Vertex* v2) {
        Graph::Halfedge* h = v1->halfedge() ;
        do {
            if(h->opposite()->vertex() == v2) {
                return true ;
            }
            h = h->next_around_vertex() ;
        } while(h != v1->halfedge()) ;
        return false ;
    }

    typedef std::vector<Graph::Vertex*> VertexLoop ;
    static void split_loop(
        const VertexLoop& loop, Graph::Vertex*& v1, Graph::Vertex*& v2
    ) {

        int min_diff = 3 ;
        if(loop.size() == 5) {
            min_diff = 2 ;
        }

        bool size_is_odd = ((loop.size() & 1) != 0) ;

        std::vector<double> s ;
        s.push_back(0.0) ;
        double cur_s = 0 ;
        for(unsigned int i=1; i<loop.size(); i++) {
            cur_s += (loop[i]->point() - loop[i-1]->point()).norm() ;
            s.push_back(cur_s) ;
        }
        double total_length = cur_s ;
        total_length += (loop[loop.size() - 1]->point() - loop[0]->point()).norm() ;

        double best_rij = Numeric::big_double ;
        v1 = nil ; 
        v2 = nil ;
        
        for(unsigned int i=0; i<loop.size(); i++) {
            for(unsigned int j=0; j<loop.size(); j++) {

                // Do not split using vertices
                // already connected by an edge.

                int diff1 = (j - i) ;
                if(diff1 < 0) { diff1 = -diff1 ; }
                int diff2 = loop.size() - diff1 ;
                int diff = ogf_min(diff1, diff2) ;
                bool diff_is_odd = ((diff & 1) != 0) ;

                if(
                    diff < min_diff || halfedge_exists_between(loop[i], loop[j]) ||
                    (!size_is_odd && !diff_is_odd)
                ) {
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

    struct EdgeToInsert {
    public:
        EdgeToInsert(Graph::Halfedge* l_in, Graph::Vertex* v1_in, Graph::Vertex* v2_in) {
            v1   = v1_in ;
            v2   = v2_in ;
            loop = l_in ;
        }
        Graph::Vertex* v1 ;
        Graph::Vertex* v2 ;
        Graph::Halfedge* loop ;
    } ;

    static void split_loop_recursive(
        Graph::Halfedge* loop, VertexLoop& L, std::vector<EdgeToInsert>& edges
    ) {
        if(L.size() <= 4) {
            return ;
        }
        Graph::Vertex* v1 ;
        Graph::Vertex* v2 ;
        split_loop(L, v1, v2) ;
        if(v1 != nil && v2 != nil) {
            edges.push_back(EdgeToInsert(loop, v1, v2)) ;
        } else {
            std::cerr << "split_loop_recursive: something strange happened" << std::endl ;
        }
        VertexLoop L1 ;
        VertexLoop L2 ;
        unsigned int i = 0 ;
        while(L[i] != v1) {
            i++ ;
        }
        while(L[i] != v2) {
            L1.push_back(L[i]) ;
            i = (i + 1) % L.size() ;
        }
        L1.push_back(L[i]) ;
        while(L[i] != v1) {
            L2.push_back(L[i]) ;
            i = (i + 1) % L.size() ;
        }
        L2.push_back(L[i]) ;
        split_loop_recursive(loop,L1,edges) ;
        split_loop_recursive(loop,L2,edges) ;
    }

    bool MapTrimmer::is_chart_corner(Graph::Halfedge* h1) {

        if(h1->vertex()->degree() <= 2) {
            return false ;
        }

        if(h1->vertex()->degree() >= 4) {
            return true ;
        }

        // degree = 3, this is the difficult case
        ogf_assert(h1->vertex()->degree() == 3) ;

        int k1 = kind_[h1] ;
        int k2 = kind_[ordered_next(h1)] ;

        if(k1 == 0 || k2 == 0) {
            return true ;
        }
        return (k1 != k2) ;
    }

    Vector3d MapTrimmer::normal(const MapCellEmbedding& em) {
        Vector3d result(0,0,0) ;
        switch(em.dimension()) {
        case 0: {
            result =  Geom::vertex_normal(em.vertex()) ;
        } break ;
        case 1: {
            Map::Halfedge* h = em.halfedge() ;
            if(h->facet() != nil) {
                result = Geom::facet_normal(h->facet()) ;
            }
            if(h->opposite()->facet() != nil) {
                result = result + Geom::facet_normal(h->opposite()->facet()) ;
            }
            result.normalize() ;
        } break ;
        case 2: {
            result = Geom::facet_normal(em.facet()) ;
        } break ;
        default: {
            ogf_assert_not_reached ;
        } break ;
        }
        return result ;
    }

    Vector3d MapTrimmer::normal(Graph::Vertex* v) {
        if(normal_.is_bound()) {
            return normal_[v] ;
        }
        return normal(embedding_[v]) ;
    }

    Graph::Vertex* MapTrimmer::merge_vertices(Graph::Vertex* v1, Graph::Vertex* v2) {
        // TODO: more work to do here on kind_ management.
        // Rem: we'd better use geometry, I think ...
        if(v1->degree() == 2 && v2->degree() == 2) {
            kind_[v1->halfedge()] = 1 ;
            kind_[v1->halfedge()->opposite()] = 1 ;
            kind_[v1->halfedge()->next_around_vertex()] = 1 ;
            kind_[v1->halfedge()->next_around_vertex()->opposite()] = 1 ;            

            kind_[v2->halfedge()] = 2 ;
            kind_[v2->halfedge()->opposite()] = 2 ;
            kind_[v2->halfedge()->next_around_vertex()] = 2 ;
            kind_[v2->halfedge()->next_around_vertex()->opposite()] = 2 ;            
        }
        return graph_editor_.merge_vertices(v1,v2) ;
    }

    Graph::Halfedge* MapTrimmer::check_connect_vertices(Graph::Vertex* v1, Graph::Vertex* v2) {
        Graph::Halfedge* result = nil ;
        if(v1 == v2) {
            std::cerr << "connect(v,v) called" << std::endl ;
            return nil ;
        }
        result = Topo::find_halfedge_between(v1,v2) ;
        if(result != nil) {
            std::cerr << "connect(v1,v2): found existing edge" << std::endl ;
        } else {
            result = graph_editor_.connect_vertices(v1,v2) ;
        }
        return result ;
    }
    
}
