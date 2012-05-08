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
 

#ifndef __CELLS_MAP_ALGOS_PACKER__
#define __CELLS_MAP_ALGOS_PACKER__

#include <OGF/cells/common/common.h>
#include <OGF/cells/map_algos/map_components.h>
#include <OGF/cells/map/map_attributes.h>

namespace OGF {

//_________________________________________________________


    /**
     * Packs a set of surfaces in parameter (texture) space.
     */

    class CELLS_API Packer {
    public:
        Packer() ;
        void pack_map_components(MapComponentList& surfaces) ;
        void pack_map(Map* surface) ;

        int image_size_in_pixels() const { return image_size_in_pixels_ ; }

        /**
         * specifies the size of the image, it is used to
         * compute the width of the margins.
         */
        void set_image_size_in_pixels(int size) {
            image_size_in_pixels_ = size ;
        }

        int margin_width_in_pixels() const { return margin_width_in_pixels_ ; }

        /**
         * specifies the width of the margins added around the components
         * in texture space. This avoids undesirable blends due to mip-mapping.
         */
        void set_margin_width_in_pixels(int width) {
            margin_width_in_pixels_ = width ;
        } 


    protected:
        void normalize_surface_components(MapComponentList& surfaces) ;
        void normalize_surface_component(MapComponent* component) ;
        Map::Halfedge* largest_border(MapComponent* component) ;

    private:

        double total_area_3d_ ;
        int image_size_in_pixels_ ;
        int margin_width_in_pixels_ ;
        MapHalfedgeAttribute<bool> is_visited_ ;

        MapHalfedgeAttribute<int> seam_type_ ;

        // A Packer cannot be copied
        Packer(const Packer& rhs) ;
        Packer& operator=(const Packer& rhs) ;
    } ;
  
    //_________________________________________________________

}
#endif

