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
 

#include <OGF/cells/cgraph/scgraph.h>
#include <OGF/cells/map/map_builder.h>
#include <OGF/basic/containers/arrays.h>
#include <OGF/basic/debug/logger.h>
#include <OGF/basic/os/stopwatch.h>
#include <algorithm>

namespace OGF {

//_________________________________________________________

    SCGraph::SCGraph() {

        // Construct the meta cell

        MapBuilder builder(&(meta_cell_.map())) ;
        builder.begin_surface() ;
        for(int i = 0; i < 8; i++) {
            builder.add_vertex(Point3d()) ;
        }
        
        // U- face
        builder.begin_facet() ;
        builder.add_vertex_to_facet(0) ;
        builder.add_vertex_to_facet(2) ;
        builder.add_vertex_to_facet(3) ;
        builder.add_vertex_to_facet(1) ;
        builder.end_facet() ;

        // U+ face
        builder.begin_facet() ;
        builder.add_vertex_to_facet(4) ;
        builder.add_vertex_to_facet(5) ;
        builder.add_vertex_to_facet(7) ;
        builder.add_vertex_to_facet(6) ;
        builder.end_facet() ;

        // V- face
        builder.begin_facet() ;
        builder.add_vertex_to_facet(0) ;
        builder.add_vertex_to_facet(1) ;
        builder.add_vertex_to_facet(5) ;
        builder.add_vertex_to_facet(4) ;
        builder.end_facet() ;

        // V+ face
        builder.begin_facet() ;
        builder.add_vertex_to_facet(2) ;
        builder.add_vertex_to_facet(6) ;
        builder.add_vertex_to_facet(7) ;
        builder.add_vertex_to_facet(3) ;
        builder.end_facet() ;

        // W- face
        builder.begin_facet() ;
        builder.add_vertex_to_facet(0) ;
        builder.add_vertex_to_facet(4) ;
        builder.add_vertex_to_facet(6) ;
        builder.add_vertex_to_facet(2) ;
        builder.end_facet() ;

        // W+ face
        builder.begin_facet() ;
        builder.add_vertex_to_facet(5) ;
        builder.add_vertex_to_facet(1) ;
        builder.add_vertex_to_facet(3) ;
        builder.add_vertex_to_facet(7) ;
        builder.end_facet() ;

        builder.end_surface() ;

        meta_cell_.initialize_from_map() ;
    }

    void SCGraph::allocate(int nu_in, int nv_in, int nw_in) {

        // An additional margin of ghost cells is added around the
        //   grid, so that no special boundary code is required.
        // Note that since nu_in, nv_in, nw_in denote the number
        //   of vertices, only one additional row is needed.
        
        int nu = nu_in + 1 ;
        int nv = nv_in + 1 ;
        int nw = nw_in + 1 ;

        user_nu_ = nu_in ;
        user_nv_ = nv_in ;
        user_nw_ = nw_in ;
        store_.allocate(nu, nv, nw) ;
        nuvw_ = nu * nv * nw ;

        int du = 1     ;
        int dv = nu    ;
        int dw = nu*nv ;

        adjacent_offset_[0] = -du ; // U-: u-1,v,w
        adjacent_offset_[1] =  du ; // U+: u+1,v,w
        adjacent_offset_[2] = -dv ; // V-: u,v-1,w
        adjacent_offset_[3] =  dv ; // V+: u,v+1,w
        adjacent_offset_[4] = -dw ; // W-: u,v,w-1
        adjacent_offset_[5] =  dw ; // W+: u,v,w+1

        vertex_offset_[0] = 0  + 0  + 0  ; // u  ,v  ,w
        vertex_offset_[1] = 0  + 0  + dw ; // u  ,v  ,w+1
        vertex_offset_[2] = 0  + dv + 0  ; // u  ,v+1,w
        vertex_offset_[3] = 0  + dv + dw ; // u  ,v+1,w+1
        vertex_offset_[4] = du + 0  + 0  ; // u+1,v  ,w
        vertex_offset_[5] = du + 0  + dw ; // u+1,v  ,w+1
        vertex_offset_[6] = du + dv + 0  ; // u+1,v+1,w
        vertex_offset_[7] = du + dv + dw ; // u+1,v+1,w+1

        // Ghostify the additional margin

        int umax = store_.size(0) - 1 ;
        int vmax = store_.size(1) - 1 ;
        int wmax = store_.size(2) - 1 ;

        VertexCellFlags flags ;

        { for(int u=0; u<=umax; u++) {
            for(int v=0; v<=vmax; v++) {
                flags.make_ghost(store_(u,v,0)) ;
                flags.make_ghost(store_(u,v,wmax)) ;
            }
        }}

        { for(int v=0; v<=vmax; v++) {
            for(int w=0; w<=wmax; w++) {
                flags.make_ghost(store_(0,v,w)) ;
                flags.make_ghost(store_(umax,v,w)) ;
            }
        }}

        { for(int w=0; w<=wmax; w++) {
            for(int u=0; u<=umax; u++) {
                flags.make_ghost(store_(u,0,w)) ;
                flags.make_ghost(store_(u,vmax,w)) ;
            }
        }}
    }
    
    void SCGraph::set_point( int u, int v, int w, const Point3d& p ) {
        scgraph_assert(u >= 0 && u < user_nu_) ;
        scgraph_assert(v >= 0 && v < user_nv_) ;
        scgraph_assert(w >= 0 && w < user_nw_) ;

        // add the margin
        VertexCell& vc = store_(u+1, v+1, w+1) ;
        vc.set_point(p) ;
    }
    
    void SCGraph::remove_cell(int u, int v, int w) {
        scgraph_assert(u >= 0 && u < user_nu_) ;
        scgraph_assert(v >= 0 && v < user_nv_) ;
        scgraph_assert(w >= 0 && w < user_nw_) ;
        VertexCellFlags flags ;
        flags.make_ghost(
            cell(get_cell_id(u,v,w))
        ) ;
    }
    
//_________________________________________________________

}

