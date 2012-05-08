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

#include <OGF/cells/map_algos/map_projection.h>
#include <OGF/cells/map_algos/map_components.h>

namespace OGF {

    bool MapParameterizerProjection::do_parameterize_disc(Map* map) {
        map_ = map ;
        get_bounding_box() ;
        principal_axes(O_, X_, Y_) ;
        X_.normalize() ; Y_.normalize() ;
        Z_ = X_ ^ Y_ ;

        double ox = 1e30, oy = 1e30, oz = 1e30 ;
        FOR_EACH_VERTEX(Map, map_, it) {
            ox = ogf_min(ox, (it->point() - O_) * X_) ;
            oy = ogf_min(oy, (it->point() - O_) * Y_) ;
            oz = ogf_min(oz, (it->point() - O_) * Z_) ;
        }        
        O_ = O_ + ox * X_ + oy * Y_ + oz * Z_ ;

        FOR_EACH_VERTEX(Map, map_, it) {
            double u = (it->point() - O_) * X_ ;
            double v = (it->point() - O_) * Y_ ;
            it->halfedge()->set_tex_coord(Point2d(u,v)) ;
        }

        double mx = -1e30, my = -1e30, mz = -1e30 ;
        FOR_EACH_VERTEX(Map, map_, it) {
            mx = ogf_max(mx, (it->point() - O_) * X_) ;
            my = ogf_max(my, (it->point() - O_) * Y_) ;
            mz = ogf_max(mz, (it->point() - O_) * Z_) ;
        }        

        if(mx < 1e-30) { mx = 1e-3 ; }
        if(my < 1e-30) { my = 1e-3 ; }
        if(mz < 1e-30) { mz = 1e-3 ; }

        X_ = mx * X_ ; 
        Y_ = my * Y_ ;
        Z_ = mz * Z_ ;

        return true ;
    }

}

