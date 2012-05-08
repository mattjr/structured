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

#include <OGF/parameterizer/algos/hlscm.h>
#include <OGF/cells/map_algos/lscm.h>
#include <OGF/cells/types/cells_library.h>

namespace OGF {
 
    MapParameterizerHLSCM::MapParameterizerHLSCM() {
        strict_kernels_ = false ;
        lock_borders_ = false ;
//        set_reduction_factor(1.0) ; // TODO
    }

    void MapParameterizerHLSCM::compute_initial_parameterization() {
        MapParameterizerLSCM lscm ;
        lscm.parameterize_disc(map_) ;
    }
    
    void MapParameterizerHLSCM::optimize_parameterization(int nb_iter) {
        MapParameterizerLSCM lscm ;
        lscm.set_nb_iter(nb_iter) ;
        lscm.set_user_locks(true) ;
        lscm.parameterize_disc(map_) ;
    }
}
