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
 

#ifndef __MESH_TOOLS_MATH_NORMAL_CYCLE__
#define __MESH_TOOLS_MATH_NORMAL_CYCLE__

#include <OGF/math/common/common.h>
#include <OGF/math/geometry/types.h>

namespace OGF {

//_________________________________________________________

    /**
     * NormalCycle evaluates the curvature tensor in function
     * of a set of dihedral angles and associated vectors.
     * Reference:
     *    Restricted Delaunay Triangulation and Normal Cycle,
     *    D. Cohen-Steiner and J.M. Morvan,
     *    SOCG 2003
     */
    class MATH_API NormalCycle {
    public:
        NormalCycle() ;
        void begin() ;
        void end() ;
        /**
         * Note: the specified edge vector needs to be pre-clipped
         * by the neighborhood.
         */
        void accumulate_dihedral_angle(
            const Vector3d& edge, double angle, double neigh_area = 1.0
        ) ;
        const Vector3d& eigen_vector(int i) const { return axis_[i_[i]] ; }
        double eigen_value(int i) const   { return eigen_value_[i_[i]] ;  } 

        const Vector3d& N() const    { return eigen_vector(2) ; }
        const Vector3d& Kmax() const { return eigen_vector(1) ; }
        const Vector3d& Kmin() const { return eigen_vector(0) ; }

        double n() const    { return eigen_value(2) ; }
        double kmax() const { return eigen_value(1) ; }
        double kmin() const { return eigen_value(0) ; }

    private:
        double center_[3] ;
        Vector3d axis_[3] ;
        double eigen_value_[3] ;
        double M_[6] ;
        int i_[3] ;
    } ;
    
//_________________________________________________________

}

#endif

