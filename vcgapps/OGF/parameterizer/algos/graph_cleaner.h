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
 

#ifndef __PARAMETERIZER_ALGOS_GRAPH_CLEANER__
#define __PARAMETERIZER_ALGOS_GRAPH_CLEANER__

#include <OGF/parameterizer/common/common.h>
#include <OGF/cells/graph/graph.h>
#include <OGF/cells/graph/graph_attributes.h>
#include <OGF/cells/map/map_embedding.h>

#include <vector>
#include <map>

namespace OGF {

    class PARAMETERIZER_API GraphCleanerArc {
    public:
        GraphCleanerArc(Graph* graph, Graph::Halfedge* h) ;
        Graph* graph() { return graph_ ; }
        double length() const ;
        bool is_closed() const { return closed_ ; }
        Graph::Vertex* extremity_1() const { 
            ogf_assert(!closed_) ;
            ogf_assert(!deleted_) ;
            return *(vertices.begin()) ;  
        }
        Graph::Vertex* extremity_2() const { 
            ogf_assert(!closed_) ;
            ogf_assert(!deleted_) ;
            return *(vertices.rbegin()) ; 
        }
        Graph::Vertex* other_extremity(Graph::Vertex* v) ;

        std::vector<Graph::Vertex*> vertices ;
        std::vector<Graph::Halfedge*> halfedges ;

        bool is_visited() const { return visited_ ; }
        void set_visited(bool x = true) { visited_ = x ; }
        bool is_deleted() const { return deleted_ ; }
        void set_deleted(bool x = true) { deleted_ = x ; }

        bool operator==(const GraphCleanerArc& rhs) const ;

    private:
        Graph* graph_ ;
        bool closed_ ;
        bool visited_ ;
        bool deleted_ ;
    } ;

    //____________________________________________________________________________________

    class PARAMETERIZER_API GraphCleaner {
    public:
        GraphCleaner(Graph* graph) ;
        bool delete_closed_loops() ;
        bool delete_duplicated_arcs() ;
        void clean_locks() ;
        bool delete_dangling_arcs() ;

    protected:
        void mark_arc(GraphCleanerArc& arc) ;
        void delete_arc(GraphCleanerArc& arc) ;
        void find_arcs_between(
            Graph::Vertex* v1, Graph::Vertex* v2, 
            std::vector<GraphCleanerArc*>& arcs
        ) ;
        Vector3d normal(Graph::Vertex* v) ;

    private:
        Graph* graph_ ;
        GraphHalfedgeAttribute<bool> is_marked_ ;
        std::vector<GraphCleanerArc> arcs_ ;
        std::map<Graph::Vertex*, std::vector<GraphCleanerArc*> > extremity_to_arc_ ;
        GraphVertexAttribute<MapCellEmbedding> embedding_ ;
    } ;

} 

#endif
