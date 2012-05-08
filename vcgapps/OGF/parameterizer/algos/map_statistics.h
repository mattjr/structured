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
 *
 * This code from B. Vallet 07/2003 computes statistics on the surface
 */
 

#ifndef __CELLS_MAP_ALGOS_STATISTICS__
#define __CELLS_MAP_ALGOS_STATISTICS__

#include <OGF/parameterizer/common/common.h>
#include <OGF/cells/map/map_attributes.h>
#include <OGF/cells/map/geometry.h>
#include <OGF/basic/containers/arrays.h>
#include <OGF/basic/types/types.h>

namespace OGF {

    class PARAMETERIZER_API MapStatistics {
    public:
        MapStatistics(Map* map) ;

        /*-+-+-* Initialisation *-+-+-*/

        /** initializes the stats: call if you want correct stats after a mesh modification */
        void init() ;
		
        /*-+-+-* Logger *-+-+-*/

        /** Logs out stats on the surface.
            - Mesh: infos on the size of the mesh
            - Geom: infos on the geometrical size of the model
            - Error: Number of flat facets or null edges 
            - Param: Infos on the deformations between 3D and texture space */
        void show(bool mesh, bool geom, bool error, bool param) ;

        /*-+-+-* components *-+-+-*/

        /** number of components of the mesh */
        int num_components() ;
		
        /*-+-+-* vertex *-+-+-*/

        /** number of vertices */
        int num_vertex();
        /** number of interior vertices = num_vertex - border_size */
        int num_interior_vertex();
        /** double[6] = [xmin, xmax, ymin, ymax, zmin, zmax] */
        double* bounding_box();
        /** gravity center of the vertices */
        Point3d gravity_center();

        /*-+-+-* facet *-+-+-*/

        /** number of facets */
        int num_facet();
        /** number of null area facets in texture space */
        int num_flat_facet_2D();
        /** number of null area facets in 3D space */ 
        int num_flat_facet_3D();
        /** sum of all the facet areas in texture space */
        double total_area_2D();
        /** smallest facet area in texture space */
        double min_facet_area_2D();
        /** average facet area in texture space */
        double average_facet_area_2D();
        /** largest facet area in texture space */
        double max_facet_area_2D();
        /** sum of all the facet areas in 3D space */
        double total_area_3D();
        /** smallest facet area in 3D space */
        double min_facet_area_3D();
        /** average facet area in 3D space */
        double average_facet_area_3D();
        /** largest facet area in 3D space */
        double max_facet_area_3D();
        /** tests if the surface is parameterized.
            test is true if all facets has null area in texture space*/
        bool is_parameterized();

        /*-+-+-* edge *-+-+-*/

        /** number of edges */
        int num_edge();
        /** number of interior edges = num_edge - border_size */
        int num_interior_edge();
        /** number of edges of null length in texture space */
        int num_null_edge_2D();
        /** number of edges of null length in 3D space */
        int num_null_edge_3D();
        /** number of border edges = number of border vertices */
        int border_size();
        /** Geometric length of the border in texture space */
        double border_length2D();
        /** Geometric length of the border in 3D space */
        double border_length3D();
        /** sum of the edge lengths in texture space */
        double total_length_2D();
        /** smallest edge length in texture space */
        double min_edge_length_2D();
        /** average edge length in texture space = total_length_2D/num_edge */
        double average_edge_length_2D();
        /** largest edge length in texture space */
        double max_edge_length_2D();
        /** sum of the edge lengths in 3D space */
        double total_length_3D();
        /** smallest edge length in 3D space */
        double min_edge_length_3D();
        /** average edge length in 3D space = total_length_3D/num_edge */
        double average_edge_length_3D();
        /** largest edge length in 3D space */
        double max_edge_length_3D();
        /** ratio between the sqrt of total area and border length in 3D.
            charactreristic of the difficulty to parameterize */
        double sock_factor();
		
        /*-+-+-* area *-+-+-*/

        /** average relative error between facet areas in 2D and 3D */
        double average_area_deformation();
        /** largest relative error between facet areas in 2D and 3D */
        double max_area_deformation();
        /** average relative error between objective and real Jacobian.
            Objective Jacobian is computed by a CJP_DotProd and stored in facet attribute "Jt" */
        double jacobian_error() ;

        /*-+-+-* angle *-+-+-*/
		
        /** average relative error between angles in 2D and 3D */
        double average_angle_deformation();
        /** largest relative error between angles in 2D and 3D */
        double max_angle_deformation();

        /*-+-+-* length *-+-+-*/
		
        /** average relative error between edge lengths in 2D and 3D */
        double average_length_deformation();
        /** largest relative error between edge lengths in 2D and 3D */
        double max_length_deformation();

    private:

        Map* surface_ ;

        // common computation
        void facet() ;
        void edge() ;
        void area() ;
        void angle() ;
        void length() ;
		
        /*-+-+-* flags *-+-+-*/
        // wpc = was previously computed
		
        bool wpc_num_components_ ;

        bool wpc_num_vertex_ ;
        bool wpc_bounding_box_ ;
        bool wpc_gravity_center_ ;

        bool wpc_edge_ ;
        bool wpc_num_edge_ ;
        bool wpc_border_size_ ;

        bool wpc_facet_ ;
        bool wpc_num_facet_ ;
		
        bool wpc_area_ ;
        bool wpc_angle_ ;
        bool wpc_length_ ;

        /*-+-+-* components *-+-+-*/
        int num_components_ ;

        /*-+-+-* vertex *-+-+-*/
        int num_vertex_ ;
        double bbox_[6] ;
        Point3d G_ ;

        /*-+-+-* triangle *-+-+-*/
        int num_facet_, num_flat2D_, num_flat3D_ ;
        double S2tot_, S2min_, S2max_, S3tot_, S3min_, S3max_ ;
        bool is_parameterized_ ;

        /*-+-+-* edge *-+-+-*/
        int num_edge_, num_short2D_, num_short3D_, border_size_ ;
        double border_length2D_, border_length3D_ ;
        double L2tot_, L2min_, L2max_, L3tot_, L3min_, L3max_ ;
		
        /*-+-+-* area *-+-+-*/
        double delta_area_, area_max_ ;

        /*-+-+-* angle *-+-+-*/
        MapFacetAttribute<double> jacobian_ ;
        double delta_angle_, angle_max_, J_err_ ;

        /*-+-+-* length *-+-+-*/
        double delta_length_, length_max_ ;

    } ;

}

#endif
