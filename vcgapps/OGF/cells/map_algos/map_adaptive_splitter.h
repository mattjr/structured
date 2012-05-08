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
 

#ifndef __OGF_CELLS_MAP_ALGOS_MAP_ADAPTIVE_SPLITTER__
#define __OGF_CELLS_MAP_ALGOS_MAP_ADAPTIVE_SPLITTER__

#include <OGF/cells/common/common.h>
#include <OGF/cells/map/map_attributes.h>
#include <OGF/cells/map/map_editor.h>
#include <OGF/cells/map/map_embedding.h>

namespace OGF {

    //__________________________________________________________________

    class CELLS_API MapAdaptiveSplitter : public MapMutator {
    public:
        MapAdaptiveSplitter(Map* map) ;
        virtual ~MapAdaptiveSplitter() ;
        virtual void apply() ;

    protected:
        virtual bool needs_to_split(Map::Halfedge* h) = 0 ;
        void split_edges() ;
        void split_facets() ;
        void split_facet(Map::Facet* f) ;
        void split_facet_n(Map::Facet* f) ;        
        void split_facet_2(Map::Facet* f) ;
        void split_facet_1(Map::Facet* f) ;
        void find_next_new_vertex(Map::Halfedge*& h) ;
        int nb_new_vertices(Map::Facet* f) ;

    private:
        MapVertexAttribute<bool> is_new_vertex_ ;
        MapEditor editor_ ;
    } ;

    //__________________________________________________________________

    class CELLS_API AttributeMapAdaptiveSplitter : public MapAdaptiveSplitter {
    public:
        AttributeMapAdaptiveSplitter(Map* map) ;
        void set_threshold(double x) { threshold_ = x ; }
        void set_split_if_smaller(double x) { split_if_smaller_ = (x != 0) ; }
        void set_vertex_attribute(MapVertexAttribute<double>& attribute) ;
        void set_halfedge_attribute(MapHalfedgeAttribute<double>& attribute) ;        
        void set_facet_attribute(MapFacetAttribute<double>& attribute) ;        

    protected:
        virtual bool needs_to_split(Map::Halfedge* h) ;

    private:
        MapVertexAttribute<double>* vertex_attribute_ ;
        MapHalfedgeAttribute<double>* halfedge_attribute_ ;
        MapFacetAttribute<double>* facet_attribute_ ;
        double threshold_ ;
        bool split_if_smaller_ ;
    } ;

    //__________________________________________________________________

}

#endif

