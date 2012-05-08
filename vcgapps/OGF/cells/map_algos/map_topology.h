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
 

#ifndef __CELLS_MAP_TOPOLOGY__
#define __CELLS_MAP_TOPOLOGY__

#include <OGF/cells/common/common.h>
#include <OGF/cells/map/map.h>

namespace OGF {

    class MapComponent ;

//_________________________________________________________

    /**
     * computes some topological invariants of a MapComponent.
     */
    class CELLS_API MapComponentTopology {
    public:
        MapComponentTopology(const MapComponent* comp) ;
        
        /**
         * returns the Euler-Poincarre characteristic,
         * xi = 2 for a sphere, xi = 1 for a disc.
         */
        int xi() const ;

        /** returns 0 for a closed surface. */
        int number_of_borders() const { return number_of_borders_ ; }

        bool is_closed() const { return number_of_borders_ == 0 ; }
        bool is_almost_closed(int max_border_size = 3) const ;
        
        /** returns the number of edges in the largest border. */
        int largest_border_size() const { return largest_border_size_ ; }

        bool is_sphere() const {
            return (number_of_borders() == 0) && (xi() == 2) ;
        }
        
        bool is_disc() const {
            return (number_of_borders() == 1) && (xi() == 1) ;
        }

        bool is_cylinder() const {
            return (number_of_borders() == 2) && (xi() == 0) ;
        }

        bool is_torus() const {
            return (number_of_borders() == 0) && (xi() == 0) ;
        }


    private:
        const MapComponent* component_ ;
        int number_of_borders_ ;
        int largest_border_size_ ;
    } ;
    
//_________________________________________________________

}
#endif

