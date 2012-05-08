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

#ifndef __OGF_CELLS_CGRAPH_TETGEN__
#define __OGF_CELLS_CGRAPH_TETGEN__

#include <OGF/cells/common/common.h>
#include <OGF/cells/cgraph/cgraph.h>
#include <OGF/cells/third_party/tetgen.h>
// #include <OGF/volume/grob/voxel_grid.h>
#include <vector>

namespace OGF {

    class VoxelGrid ;

    /**
     * Interface between Graphite and the quality
     * mesher tetgen. CGraphTetgen generates
     * a quality tetrahedralization from a set
     * of closed shells.
     */

    class CELLS_API CGraphTetgen : public CGraphMutator {
    public:
        CGraphTetgen(CGraph* target = nil) ;
        void set_surface(Map* map) { surface_ = map ; }
        void set_add_steiner_points_on_exterior_boundary(bool x) {
            add_steiner_points_on_exterior_boundary_ = x  ;
        }
        void set_add_steiner_points_on_interior_boundary(bool x) {
            add_steiner_points_on_interior_boundary_ = x  ;
        }
        void set_max_tet_shape(double x) { max_tet_shape_ = x ; }
        void set_max_tet_volume(double x) { max_tet_volume_ = x ; }
        void set_lock_intersections(bool x) { lock_intersections_ = x ; }
        void set_tag_regions(bool x) { tag_regions_ = x ; }
        void tetrahedralize() ;

    protected:
        void graphite_to_tetgen() ;
        void tetgen_to_graphite() ;

    private:
        Map* surface_ ;
        bool add_steiner_points_on_exterior_boundary_ ;
        bool add_steiner_points_on_interior_boundary_ ;
        bool lock_intersections_ ;
        bool tag_regions_ ;
        double max_tet_shape_  ;
        double max_tet_volume_ ;
        tetgenio tetgen_surface_ ;
        tetgenio tetgen_volume_ ;
    } ;

}

#endif
