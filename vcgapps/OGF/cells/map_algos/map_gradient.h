/*
 *  GXML/Graphite: Geometry and Graphics Programming Library + Utilities
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

#ifndef __OGF_CELLS_MAP_ALGO_MAP_GRADIENT__
#define __OGF_CELLS_MAP_ALGO_MAP_GRADIENT__

#include <OGF/cells/common/common.h>
#include <OGF/cells/map/map.h>

namespace OGF {


    class CELLS_API MapGradient {
    public:
        MapGradient(Map* map) : map_(map) { }


        /** 
         * returns the interpolated directional derivative relative to W
         * at vertex v.
         */
        Vector3d compute_gradient(Map::Vertex* v, const Vector2d& w) ;


        /** 
         * returns the directional derivative relative to W in
         * the triangle ( h1->vertex(), h2->vertex(), h3->vertex() ).
         */
        Vector3d compute_gradient(
            Map::Halfedge* h1,
            Map::Halfedge* h2,
            Map::Halfedge* h3,
            const Vector2d& W
        ) ;


    protected:
        
        void compute_gradient(
            const Point3d& p1, const Point3d& p2, const Point3d& p3,
            const Point2d& q1, const Point2d& q2, const Point2d& q3,
            double* TU,
            double* TV
        ) ;

        void compute_gradient(
            Map::Halfedge* h1,
            Map::Halfedge* h2,
            Map::Halfedge* h3,
            double* TU,
            double* TV
        ) ;


    private:
        Map* map_ ;
    } ;
} 

#endif
