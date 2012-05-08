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
 

#ifndef ___MAP_EMBEDDING__
#define ___MAP_EMBEDDING__

#include <OGF/cells/common/common.h>
#include <OGF/cells/map/map.h>

namespace OGF {

    /**
     * MapCellEmbedding can be used for instance to represent
     * a line embedded in a surface. The Graph representing 
     * the line then uses a GraphVertexAttribute<MapCellEmbedding>.
     */
    class MapCellEmbedding {
    public:
        MapCellEmbedding(Map* map, Map::Vertex* v) : map_(map), dimension_(0) {
            cell_.vertex_ = v ;
        }

        MapCellEmbedding(Map* map, Map::Halfedge* h) : map_(map), dimension_(1) {
            cell_.halfedge_ = h ;
        }

        MapCellEmbedding(Map* map, Map::Facet* f) : map_(map), dimension_(2) {
            cell_.facet_ = f ;
        }

        MapCellEmbedding() : map_(nil), dimension_(-1) { 
            cell_.vertex_ = nil ;
        }

        Map* map() const { return map_ ; }
        int dimension() const { return dimension_ ; }

        bool is_nil() const { return (dimension_ == -1) ; }

        Map::Vertex* vertex() const {
            ogf_assert(dimension_ == 0) ;
            return cell_.vertex_ ;
        }

        Map::Halfedge* halfedge() const {
            ogf_assert(dimension_ == 1) ;
            return cell_.halfedge_ ;
        }

        Map::Facet* facet() const {
            ogf_assert(dimension_ == 2) ;
            return cell_.facet_ ;
        }

    private:
        Map* map_ ;
        int dimension_ ;
        union {
            Map::Vertex* vertex_ ;
            Map::Halfedge* halfedge_ ;
            Map::Facet* facet_ ;
        } cell_ ;
    } ;

    //_____________________________________________________________________________________

    /**
     * represents a point embedded in a cell of a map.
     */
    class MapEmbeddedPoint {
    public:
        MapEmbeddedPoint(
            Map* map, const Point3d& p, Map::Vertex* v
        ) : point_(p), embedding_(map, v) {
        }
        MapEmbeddedPoint(
            Map* map, const Point3d& p, Map::Halfedge* h
        ) : point_(p), embedding_(map, h) {
        }
        MapEmbeddedPoint(
            Map* map, const Point3d& p, Map::Facet* f
        ) : point_(p), embedding_(map, f) {
        }
        MapEmbeddedPoint(
            const Point3d& p, const MapCellEmbedding& em
        ) : point_(p), embedding_(em) {
        }

        MapEmbeddedPoint() {}

        const Point3d& point() const { return point_ ; }
        void set_point(const Point3d& p) { point_ = p ; }
        const MapCellEmbedding& embedding() const { return embedding_ ; }
        void set_embedding(const MapCellEmbedding& em) { embedding_ = em ; }

    private:
        Point3d point_ ;
        MapCellEmbedding embedding_ ;
    } ;

    //_____________________________________________________________________________________

}

#endif
