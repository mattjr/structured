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
 

#ifndef __CELLS_MAP_ALGOS_ANGLES_TO_UV__
#define __CELLS_MAP_ALGOS_ANGLES_TO_UV__

#include <OGF/cells/common/common.h>
#include <OGF/cells/map_algos/lscm.h>
#include <OGF/cells/map/map_attributes.h>
#include <OGF/math/geometry/complex.h>


namespace OGF {

    //_______________________________________________________________________

    class LinearSolver ;

    /**
     * Computes (u,v) texture coordinates in function of
     * angles. Angles are specified by an Halfedge attribute
     * named "angle" (for instance, computed thanks to the ABF
     * method).
     */

    enum AnglesToUVMode { ANGLES=0, ANGLES_ITERATIVE=1, DOT_PRODUCTS=2 } ;

    class CELLS_API AnglesToUV : public MapParameterizerLSCM {
    public:
        AnglesToUV() ;
        void set_mode(AnglesToUVMode m) { mode_ = m ; }
        AnglesToUVMode mode() const { return mode_ ; }
        virtual bool do_parameterize_disc(Map* map) ;

        Complex complex_angle(
            Map::Halfedge* h0, Map::Halfedge* h1, Map::Halfedge* h2
        ) ;

        virtual void setup_conformal_map_relations(
            LinearSolver& solver, 
            Map::Halfedge* h0,
            Map::Halfedge* h1,
            Map::Halfedge* h2
        ) ;

    private:
        AnglesToUVMode mode_ ;
        MapHalfedgeAttribute<double> angle_ ;
        MapHalfedgeAttribute<double> gamma_ ;
    } ;


    //_______________________________________________________________________

}

#endif

