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

// #define OGF_PARANOID

#include <OGF/parameterizer/algos/map_statistics.h>
#include <OGF/cells/map_algos/map_chartizer.h>
#include <OGF/cells/map/geometry.h>
#include <math.h>

namespace OGF {

    /*-+-+-* Constructor *-+-+-*/

    MapStatistics::MapStatistics(Map* map):
        surface_(map),
        jacobian_(map, "Jf")
    {
        init() ;
    }

    /*-+-+-* Initialisation *-+-+-*/
    void MapStatistics::init() {

        // flags
        wpc_num_components_ = false ;
        wpc_num_vertex_ = wpc_bounding_box_ = wpc_gravity_center_ = false ;
        wpc_edge_ = wpc_num_edge_ =	wpc_border_size_ = false ;
        wpc_facet_ = wpc_num_facet_ = false ;
        wpc_area_ =	wpc_angle_ = wpc_length_ = false ;
		
        /*-+-+-* Mesh *-+-+-*/
		
        // components
        num_components_ = 0 ;

        // vertex
        num_vertex_ = 0 ;
        for(int i = 0; i < 6; i++) bbox_[i] = 0 ;
        G_ = Point3d(0, 0, 0) ;

        // facet
        num_facet_ = num_flat2D_ = num_flat3D_ = 0;
        S2tot_ = S3tot_ = 0 ;
        S2max_ = S3max_ = -Numeric::big_double ;
        S2min_ = S3min_ = Numeric::big_double ;
        is_parameterized_ = false ;

        // edge
        num_edge_ = 0 ;
        num_short2D_ = num_short3D_ = border_size_ = 0 ;
        border_length2D_ = border_length3D_ = 0 ;
        L2tot_ = L3tot_ = 0 ;
        L2max_ = L3max_ = -Numeric::big_double ;
        L2min_ = L3min_ = Numeric::big_double ;

        /*-+-+-* Parameterization *-+-+-*/

        // area
        delta_area_ = area_max_ = J_err_ = 0 ;

        // angle
        delta_angle_ = angle_max_ = 0 ;

        // length
        delta_length_ = length_max_ = 0;

    }

    /*-+-+-* Logger *-+-+-*/
    void MapStatistics::show(bool mesh, bool geom, bool error, bool param){
        if(mesh) {
            Logger::out("Components")
                << num_components() << std::endl ;

            Logger::out("Mesh stats")
                << num_vertex() << " vertices, "
                << num_facet() << " facets, "
                << border_size() << "/" << num_edge() << " border edges."
                << sock_factor() << " sock factor."
                << std::endl ;
        }

        if(geom) {
            bounding_box() ;
            Logger::out("Vertex bounding box")
                << "["
                << bbox_[0] << "," << bbox_[1] << "]x["
                << bbox_[2] << "," << bbox_[3] << "]x["
                << bbox_[4] << "," << bbox_[5]
                << "]" << std::endl ;

            gravity_center() ;

            Logger::out("Gravity center")
                << "("
                << G_.x() << ", " << G_.y() << ", " << G_.z()
                << ")" << std::endl ;

            Logger::out("Surface area")
                << "area2D = " << total_area_2D() << " area3D = " << total_area_3D()
                << std::endl ;

            Logger::out("Facet area 3D")
                << min_facet_area_3D() << " min. "
                << average_facet_area_3D() << " av. "
                << max_facet_area_3D() << " max."
                << std::endl ;

            if(is_parameterized()) {
                Logger::out("Facet area 2D")
                    << min_facet_area_2D() << " min. "
                    << average_facet_area_2D() << " av. "
                    << max_facet_area_2D() << " max."
                    << std::endl ;
            }

            Logger::out("Mesh length")
                << "Total length 2D = " << total_length_2D()
                << " Total length3D = " << total_length_3D()
                << std::endl ;

            if(is_parameterized_) {
                Logger::out("Edge length 2D")
                    << min_edge_length_2D() << " min. "
                    << average_edge_length_2D() << " av. "
                    << max_edge_length_2D() << " max."
                    << std::endl ;
            }

            Logger::out("Edge length 3D")
                << min_edge_length_3D() << " min. "
                << average_edge_length_3D() << " av. "
                << max_edge_length_3D() << " max."
                << std::endl ;

            if(border_size() > 0) {
                Logger::out("Border count")
                    << "Total length2D " << border_length2D()
                    << " Total length3D " << border_length3D()
                    << std::endl ;
            }
            else Logger::out("Border count") << "Surface is closed." << std::endl ;
        }


        if(error) {
            Logger::out("Errors")
                << num_null_edge_3D() << " null 3D edges, "
                << num_null_edge_2D() << " null 2D edges. "
                << num_flat_facet_3D() << " flat 3D facets, "
                << num_flat_facet_2D() << " flat 2D facets."
                << std::endl ;
        }

        if(param && is_parameterized()){
            Logger::out("Atlas stats")
                << "Surfacic deformation: "
                << average_area_deformation() << "% av. "
                << max_area_deformation() << "% max."
                << std::endl ;
            Logger::out("Atlas stats")
                << "Error on Jacobian: "
                << jacobian_error() << "%."
                << std::endl ;

            Logger::out("Atlas stats")
                << "Angular deformation: "
                << average_angle_deformation() << "% av. "
                << max_angle_deformation() << "% max."
                << std::endl ;

            Logger::out("Atlas stats")
                << "Length deformation: "
                << average_length_deformation() << "% av. "
                << max_length_deformation() << "% max."
                << std::endl ;
        }
        else Logger::out("Atlas Stats") << "Surface is not parameterized." << std::endl ;
    }

    /*-+-+-* components *-+-+-*/
    int MapStatistics::num_components() {
        // We will use MapChartizer's components finder
        if(!wpc_num_components_) {
            MapChartizer map_chartizer(surface_) ;
            num_components_ = map_chartizer.get_connex_num() ;
            wpc_num_components_ = true ;
        }
        return num_components_ ;
    }

    /*-+-+-* vertex *-+-+-*/
    int MapStatistics::num_vertex() {
        if(!wpc_num_vertex_) {
            FOR_EACH_VERTEX(Map, surface_, Vit) num_vertex_++ ;
            wpc_num_vertex_ = true ;
        }
        return num_vertex_ ;
    }

    int MapStatistics::num_interior_vertex() {
        return num_vertex() - border_size() ;
    }

    double* MapStatistics::bounding_box() {
        if(!wpc_bounding_box_) {
            {for(int i = 0; i < 5; i+=2) { bbox_[i] =  Numeric::big_double; }}
            {for(int i = 1; i < 6; i+=2) { bbox_[i] = -Numeric::big_double; }}
            FOR_EACH_VERTEX(Map, surface_, Vit) {
                if(Vit->point().x() < bbox_[0]) bbox_[0] = Vit->point().x() ;
                else if(Vit->point().x() > bbox_[1])  bbox_[1] = Vit->point().x() ;
                if(Vit->point().y() < bbox_[2]) bbox_[2] = Vit->point().y() ;
                else if(Vit->point().y() > bbox_[3])  bbox_[3] = Vit->point().y() ;
                if(Vit->point().z() < bbox_[4]) bbox_[4] = Vit->point().z() ;
                else if(Vit->point().z() > bbox_[5])  bbox_[5] = Vit->point().z() ;
            }
            wpc_bounding_box_ = true ;
        }
        return bbox_ ;
    }

    Point3d MapStatistics::gravity_center() {
        if(!wpc_gravity_center_) {
            double Gx = 0, Gy = 0, Gz = 0 ;
            FOR_EACH_VERTEX(Map, surface_, Vit) {
                num_vertex_++ ;
                // bounding box

                // center of gravVity
                Gx+=Vit->point().x() ;
                Gy+=Vit->point().y() ;
                Gy+=Vit->point().z() ;
            }
            Gx/=num_vertex_ ;
            if(Gx < 1e-9) Gx = 0.0 ;
            Gy/=num_vertex_ ;
            if(Gy < 1e-9) Gy = 0.0 ;
            Gz/=num_vertex_ ;
            if(Gz < 1e-9) Gz = 0.0 ;
            G_ = Point3d(Gx, Gy, Gz) ;
            wpc_gravity_center_ = true ;
        }
        return G_ ;
    }

    /*-+-+-* facet *-+-+-*/

    void MapStatistics::facet(){
        if(!wpc_facet_) {
            double S2, S3;
            FOR_EACH_FACET(Map, surface_, Fit) {
                S2 = Geom::facet_area2d(Fit->halfedge()->facet());
                S3 = Geom::facet_area(Fit->halfedge()->facet());

                if(S2 < 1e-9) num_flat2D_++ ;
                if(S2 > 0) S2tot_ += S2 ;
                if(S2 < S2min_) S2min_ = S2 ;
                if(S2 > S2max_) S2max_ = S2 ;

                if(S3 < 1e-9) num_flat3D_++ ;
                if(S3 > 0) S3tot_ += S3 ;
                if(S3 < S3min_) S3min_ = S3 ;
                if(S3 > S3max_) S3max_ = S3 ;
            }
            wpc_facet_ = true ;
        }
    }

    int MapStatistics::num_facet(){
        if(!wpc_num_facet_) {
            FOR_EACH_FACET(Map, surface_, Fit) num_facet_++ ;
            wpc_num_facet_ = true ;
        }
        return num_facet_ ;
    }

    int MapStatistics::num_flat_facet_2D(){
        facet() ;
        return num_flat2D_ ;
    }

    int MapStatistics::num_flat_facet_3D(){
        facet() ;
        return num_flat3D_ ;
    }

    double MapStatistics::total_area_2D(){
        facet() ;
        return S2tot_ ;
    }

    double MapStatistics::min_facet_area_2D(){
        facet() ;
        return S2min_ ;
    }

    double MapStatistics::average_facet_area_2D(){
        return total_area_2D()/num_facet() ;
    }

    double MapStatistics::max_facet_area_2D(){
        facet() ;
        return S2max_ ;
    }

    double MapStatistics::total_area_3D(){
        facet() ;
        return S3tot_ ;
    }

    double MapStatistics::min_facet_area_3D(){
        facet() ;
        return S3min_ ;
    }

    double MapStatistics::average_facet_area_3D(){
        return total_area_3D()/num_facet() ;
    }

    double MapStatistics::max_facet_area_3D(){
        facet() ;
        return S3max_ ;
    }

    bool MapStatistics::is_parameterized() {
        return (num_flat_facet_2D() < num_facet()) ;
    }


    /*-+-+-* edge *-+-+-*/

    // edge geom info
    void MapStatistics::edge(){
        if(!wpc_edge_) {
            double L2, L3 ;
            FOR_EACH_EDGE(Map, surface_, Eit) {
                num_edge_++ ;
                L2 = Geom::vector2d(Eit).norm() ;
                L3 = Geom::vector(Eit).norm() ;
                if( L2 < 1e-9) num_short2D_++ ;
                if( L3 < 1e-9) num_short3D_++ ;
                if(L2 < L2min_) L2min_ = L2 ;
                else if(L2 > L2max_) L2max_ = L2 ;
                if(L3 < L3min_) L3min_ = L3 ;
                else if(L3 > L3max_) L3max_ = L3 ;
                if(L2 > 0) L2tot_ += L2 ;
                if(L3 > 0) L3tot_ += L3 ;
                if(Eit->is_border_edge()) {
                    border_length2D_ += L2 ;
                    border_length3D_ += L3 ;
                }
            }
            wpc_edge_ = true ;
        }
    }

    int MapStatistics::num_edge(){
        if(!wpc_num_edge_) {
            FOR_EACH_EDGE(Map, surface_, it)
                num_edge_++ ;
            wpc_num_edge_ = true ;
        }
        return num_edge_ ;
    }

    int MapStatistics::border_size(){
        if(!wpc_border_size_) {
            FOR_EACH_EDGE(Map, surface_, Eit)
                if(Eit->is_border_edge()) border_size_++ ;
            wpc_border_size_ = true ;
        }
        return border_size_ ;
    }

    int MapStatistics::num_interior_edge(){
        return num_edge() - border_size();
    }

    int MapStatistics::num_null_edge_2D(){
        edge() ;
        return num_short2D_ ;
    }

    int MapStatistics::num_null_edge_3D(){
        edge() ;
        return num_short3D_ ;
    }

    double MapStatistics::border_length2D(){
        edge() ;
        return border_length2D_ ;
    }

    double MapStatistics::border_length3D(){
        edge() ;
        return border_length3D_ ;
    }

    double MapStatistics::total_length_2D(){
        edge() ;
        return L2tot_ ;
    }

    double MapStatistics::min_edge_length_2D(){
        edge() ;
        return L2min_ ;
    }

    double MapStatistics::average_edge_length_2D(){
        return total_length_2D() / num_edge() ;
    }

    double MapStatistics::max_edge_length_2D(){
        edge() ;
        return L2max_ ;
    }

    double MapStatistics::total_length_3D(){
        edge() ;
        return L3tot_ ;
    }

    double MapStatistics::min_edge_length_3D(){
        edge() ;
        return L3min_ ;
    }

    double MapStatistics::average_edge_length_3D(){
        return total_length_3D() / num_edge() ;
    }

    double MapStatistics::max_edge_length_3D(){
        edge() ;
        return L3max_ ;
    }

    double MapStatistics::sock_factor(){
        return sqrt(4 * M_PI * total_area_3D() / (border_length3D() * border_length3D()) ) ;
    }

    /*-+-+-* area *-+-+-*/

    void MapStatistics::area() {
        if(!wpc_area_) {
            double d, S2, S3, Jtot = 0 ;
            {FOR_EACH_FACET(Map, surface_, Fit) {
                Jtot += jacobian_[Fit] * Geom::facet_area(Fit->halfedge()->facet()) ;
            }}
            {FOR_EACH_FACET(Map, surface_, Fit) {
                S2 = Geom::facet_area2d(Fit->halfedge()->facet()) ;
                S3 = Geom::facet_area(Fit->halfedge()->facet()) ;
                d = ogf_abs(1 - (S3 * total_area_2D())/(S2*total_area_3D())) ;
                if(d > area_max_ && d < 1e9) area_max_ = d ;
                d *= S2 / total_area_2D() ;
                if(d > 0) delta_area_ += d ;
                J_err_ += ogf_abs(S2 / total_area_2D() - jacobian_[Fit] * S3 / Jtot ) ;
            }}
            delta_area_ *= 100 ;
            area_max_ *= 100 ;
            J_err_ *= 100.0 ;
            wpc_area_= true ;
        }
    }


    double MapStatistics::average_area_deformation(){
        area() ;
        return delta_area_ ;
    }

    double MapStatistics::max_area_deformation(){
        area() ;
        return area_max_ ;
    }

    double MapStatistics::jacobian_error(){
        area() ;
        return J_err_ ;
    }

    /*-+-+-* angle *-+-+-*/
    void MapStatistics::angle(){
        if(!wpc_angle_) {
            double d, A2, A3 ;
            FOR_EACH_HALFEDGE(Map, surface_, Hit) {
                //angle deformation
                A2 = M_PI - Geom::angle(Geom::vector2d(Hit), Geom::vector2d(Hit->next())) ;
                A3 = M_PI - Geom::angle(Geom::vector(Hit), Geom::vector(Hit->next())) ;
                d = ogf_abs(100*(1 - (A3/A2))) ;
                if(d > angle_max_ && d < 1e9) angle_max_ = d ;
                if(d > 0) delta_angle_ += d ;
            }
            delta_angle_ /= 2 * num_edge() ;
            wpc_angle_ = true ;
        }
    }

    double MapStatistics::average_angle_deformation(){
        angle() ;
        return delta_angle_ ;
    }

    double MapStatistics::max_angle_deformation(){
        angle() ;
        return angle_max_ ;
    }

    /*-+-+-* length *-+-+-*/
    void MapStatistics::length(){
        if(!wpc_length_) {
            double d, L2, L3 ;
            FOR_EACH_EDGE(Map, surface_, Eit){
                L2 = Geom::vector2d(Eit).norm()/total_length_2D() ;
                L3 = Geom::vector(Eit).norm()/total_length_3D() ;
                d = ogf_abs(100*(L2 - L3)) ;
                if(d > 0) delta_length_ += d ;
                d /= L2 ;
                if(d > length_max_ && d < 1e9) length_max_ = d ;
            }
            wpc_length_ = true ;
        }
    }

    double MapStatistics::average_length_deformation(){
        length() ;
        return delta_length_ ;
    }

    double MapStatistics::max_length_deformation(){
        length();
        return length_max_ ;
    }	
}
//___________________________________________________________________
