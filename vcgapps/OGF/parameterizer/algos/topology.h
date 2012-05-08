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
 

#ifndef __PARAMETERIZER_ALGOS_TOPOLOGY__
#define __PARAMETERIZER_ALGOS_TOPOLOGY__

#include <OGF/parameterizer/common/common.h>
#include <OGF/cells/map/map.h>
#include <OGF/cells/map/map_embedding.h>
#include <OGF/cells/map/map_attributes.h>
#include <OGF/cells/graph/graph.h>

namespace OGF {

    namespace Topo {

        //_________________________________ Graph ____________________________

        PARAMETERIZER_API Graph::Halfedge* find_halfedge_between(
            Graph::Vertex* v1, Graph::Vertex* v2
        ) ;

        //_________________________________ Map _____________________________
        
        PARAMETERIZER_API Map::Halfedge* find_halfedge_between(
            Map::Vertex* v1, Map::Vertex* v2
        ) ;

        PARAMETERIZER_API Map::Halfedge* find_halfedge_between(
            Map::Facet* f1, Map::Facet* f2
        ) ;

        inline bool on_same_facet(Map::Halfedge* h1, Map::Halfedge* h2) {
            return ((h1->facet() == h2->facet()) && (h1->facet() != nil)) ;
        }

        PARAMETERIZER_API bool facet_has_vertex(Map::Facet* f, Map::Vertex* v) ;

        PARAMETERIZER_API Map::Facet* find_facet_between(
            Map::Vertex* v1, Map::Vertex* v2
        ) ;

        //_________________________________ MapEmbedding ____________________        

        MapCellEmbedding PARAMETERIZER_API segment_embedding(
            Map* map, const MapCellEmbedding& v1_em, const MapCellEmbedding& v2_em 
        ) ;


    }

}

#endif

