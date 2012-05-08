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

#ifndef __OGF_CELLS_MAP_ALGOS_STRETCH_MAP_PARAMETERIZER__
#define __OGF_CELLS_MAP_ALGOS_STRETCH_MAP_PARAMETERIZER__

#include <OGF/cells/common/common.h>
#include <OGF/cells/map_algos/hierarchical_map_parameterizer.h>

namespace OGF {
    
    class CELLS_API StretchMapParameterizer 
        : public RandomDescentMapParameterizer {
    public:
        StretchMapParameterizer() ;
        static void compute_derivatives(
            const Point2d& p1, const Point2d& p2, const Point2d& p3,
            const Point3d& q1, const Point3d& q2, const Point3d& q3,
            double A, Vector3d& Ss, Vector3d& St
        ) ;
        virtual double facet_criterion(Map::Facet* f) ;
        virtual void compute_initial_parameterization() ;
    } ;

}

#endif
