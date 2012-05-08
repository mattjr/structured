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
 


#ifndef __OGF_CELLS_THIRD_PARTY_GENERIC_RASTERIZER_3D_
#define __OGF_CELLS_THIRD_PARTY_GENERIC_RASTERIZER_3D_

#include <OGF/cells/common/common.h>

#include <OGF/basic/types/types.h>
#include <OGF/math/geometry/types.h>
#include <OGF/basic/debug/assert.h>
#include <OGF/image/types/image.h>
#include <iostream>

namespace OGF {

/**************************************************************************/		
    class CELLS_API VolumeGL {
    public:
        typedef int coord ;
        enum {large_coord = 100000} ;
		
        static inline coord vgl_max(VolumeGL::coord x, VolumeGL::coord y) {
            return x > y ? x : y ;
        } 
		
        static inline coord vgl_min(VolumeGL::coord x, VolumeGL::coord y) {
            return x < y ? x : y ;
        } 
		
        static inline coord vgl_sgn(VolumeGL::coord x) {
            return (x == 0) ? 0 : ((x > 0) ? 1 : -1) ;
        }
		
        static inline void vgl_swap(coord& x, coord& y) {
            coord tmp = x ;
            x = y ;
            y = tmp ;
        }	    
	    
        class Vertex {
        public:
            Vertex(coord u, coord v, coord w, float val = 0) {
                coords_[0] = u ;
                coords_[1] = v ;
                coords_[2] = w ;
                value_ = val ;
            } 
            Vertex() { }
            coord u() const { return coords_[0] ; } 
            coord v() const { return coords_[1] ; } 
            coord w() const { return coords_[2] ; }
            float value() const { return value_ ; }
            void u(coord in) { coords_[0] = in ; }
            void v(coord in) { coords_[1] = in ; }
            void w(coord in) { coords_[2] = in ; }
            void value(float val) { value_ = val ; }
	
        protected:
            coord coords_[3] ;
            float value_ ;
        } ;
	
        struct PolygonBucket {
            coord u ;
            coord w ;
        } ;
	
        struct PolyhedronBucket {
            coord u ;
        } ;
    } ;



    
/**************************************************************************/
	
	
    template <class PROCESS> class CELLS_API GenericRasterizer3d : public PROCESS , public VolumeGL {
    public:
        typedef PROCESS ProcessType ;
        typedef int coord ;
        enum {large_coord = 100000} ;
        GenericRasterizer3d(int nu, int nv, int nw, bool no_empty_cells, bool barycentric_ponderation /*= false*/) {
            nu_   = nu ;
            nv_   = nv ;
            nw_   = nw ;
            nuv_  = nu_ * nv_ ;
            nuvw_ = nuv_ * nw_ ;
            no_empty_cells_ = no_empty_cells ;
            barycentric_ponderation_ = barycentric_ponderation ;
            min_imp = 1.0 ;
            max_imp = 0.0 ;
		    
            polygon_left_  = new PolygonBucket[nv_] ;
            polygon_right_ = new PolygonBucket[nv_] ;
		
            polyhedron_left_ = new PolyhedronBucket[nv_] ;
            polyhedron_right_ = new PolyhedronBucket[nv_] ;
		
            bottom_plane_  = new coord[nu_ * nv_] ;
            top_plane_ = new coord[nu_ * nv_] ;
        }   
	    
        ~GenericRasterizer3d() {
            delete[] polygon_left_ ; polygon_left_ = nil ;
            delete[] polygon_right_ ; polygon_right_ = nil ;
		
            delete[] polyhedron_left_ ; polyhedron_left_ = nil ;
            delete[] polyhedron_right_ ; polyhedron_right_ = nil ;
		
            delete[] bottom_plane_ ; bottom_plane_ = nil ;
            delete[] top_plane_ ; top_plane_ = nil ;
        }
	
		
        void value(float value_in) {
            value_ = value_in ;
        }
		
        void begin_polyhedron() {
            int i ;
            for(i=0; i<nv_; i++) {
                polyhedron_left_[i].u = large_coord ;
                polyhedron_right_[i].u = -large_coord ;
            }
            polyhedron_v_min_ = large_coord ;
            polyhedron_v_max_ = -large_coord ;
		
            for(i = 0 ; i < nuv_; i++) {
                bottom_plane_[i] = large_coord ;
                top_plane_[i] = -large_coord ;
            }
        }
	    
        void end_polyhedron() {
            do_polyhedron() ;
        }
        void begin_face() {
            current_vertex_ = 0 ;
            for(int i=0;i<nv_;i++) {
                polygon_left_[i].u = large_coord ;
                polygon_right_[i].u = -large_coord ;
            }
        }

        void end_face() {
            do_face() ;
        }
		
        void vertex(coord u, coord v, coord w) {
            ogf_assert(current_vertex_ < vertex_buffer_size - 1) ;
            ogf_assert(u >= 0 && u < nu_) ;
            ogf_assert(v >= 0 && v < nv_) ;
            ogf_assert(w >= 0 && w < nw_) ;
            vertex_buffer_[current_vertex_].u(u) ;
            vertex_buffer_[current_vertex_].v(v) ;
            vertex_buffer_[current_vertex_].w(w) ;
            vertex_buffer_[current_vertex_].value(value_) ;
            current_vertex_++ ;
        }

        void rasterize_polyhedron(Point3d v1, Point3d v2, Point3d v3, Point3d v4) {
            // computes center of mass :
            double x1 = v1.x(); double y1 = v1.y(); double z1 = v1.z();
            double x2 = v2.x(); double y2 = v2.y(); double z2 = v2.z();
            double x3 = v3.x(); double y3 = v3.y(); double z3 = v3.z();
            double x4 = v4.x(); double y4 = v4.y(); double z4 = v4.z();
            ug = (x1+x2+x3+x4) / 4.0;
            vg = (y1+y2+y3+y4) / 4.0;
            wg = (z1+z2+z3+z4) / 4.0;
            d_max = 0.0;
            double the_d = compute_distance(x1,y1,z1,ug,vg,wg) ;
            d_max = (d_max<the_d) ? the_d : d_max ;
            the_d = compute_distance(x2,y2,z2,ug,vg,wg) ;
            d_max = (d_max<the_d) ? the_d : d_max ;
            the_d = compute_distance(x3,y3,z3,ug,vg,wg) ;
            d_max = (d_max<the_d) ? the_d : d_max ;
            the_d = compute_distance(x4,y4,z4,ug,vg,wg) ;
            d_max = (d_max<the_d) ? the_d : d_max ;
            d_max += 1.5 ;
			
            begin_polyhedron() ;
            begin_face() ;
            vertex(corres_pixel(v1.x(),0),corres_pixel(v1.y(),1),corres_pixel(v1.z(),2)) ;
            vertex(corres_pixel(v2.x(),0),corres_pixel(v2.y(),1),corres_pixel(v2.z(),2)) ;
            vertex(corres_pixel(v3.x(),0),corres_pixel(v3.y(),1),corres_pixel(v3.z(),2)) ;
            end_face() ;
            begin_face() ;
            vertex(corres_pixel(v2.x(),0),corres_pixel(v2.y(),1),corres_pixel(v2.z(),2)) ;
            vertex(corres_pixel(v3.x(),0),corres_pixel(v3.y(),1),corres_pixel(v3.z(),2)) ;
            vertex(corres_pixel(v4.x(),0),corres_pixel(v4.y(),1),corres_pixel(v4.z(),2)) ;
            end_face() ;
            begin_face() ;
            vertex(corres_pixel(v3.x(),0),corres_pixel(v3.y(),1),corres_pixel(v3.z(),2)) ;
            vertex(corres_pixel(v4.x(),0),corres_pixel(v4.y(),1),corres_pixel(v4.z(),2)) ;
            vertex(corres_pixel(v1.x(),0),corres_pixel(v1.y(),1),corres_pixel(v1.z(),2)) ;
            end_face() ;
            begin_face() ;
            vertex(corres_pixel(v4.x(),0),corres_pixel(v4.y(),1),corres_pixel(v4.z(),2)) ;
            vertex(corres_pixel(v1.x(),0),corres_pixel(v1.y(),1),corres_pixel(v1.z(),2)) ;
            vertex(corres_pixel(v2.x(),0),corres_pixel(v2.y(),1),corres_pixel(v2.z(),2)) ;
            end_face() ;
            end_polyhedron() ; // the rasterizing process is launched

            if (no_empty_cells_)
                PROCESS::pixel( corres_pixel(ug,0) , corres_pixel(vg,1) , corres_pixel(wg,2) , 1.0) ; // we add the center of gravity of the polyhedron to avoid empty volume.
            //PROCESS::mark(corres_pixel(v1.x(),0) , corres_pixel(v1.y(),1), corres_pixel(v1.z(),2), 1.0) ;
            //PROCESS::mark(corres_pixel(v2.x(),0) , corres_pixel(v2.y(),1), corres_pixel(v2.z(),2), 1.0) ;
            //PROCESS::mark(corres_pixel(v3.x(),0) , corres_pixel(v3.y(),1), corres_pixel(v3.z(),2), 1.0) ;
            //PROCESS::mark(corres_pixel(v4.x(),0) , corres_pixel(v4.y(),1), corres_pixel(v4.z(),2), 1.0) ;
        }
	
    protected:
		
        coord corres_pixel(double d, int type) {
            coord limit ; 
            switch(type) {
            case 0: // x
                limit = nu_ ; break ;
            case 1: // y
                limit = nv_ ; break ;
            case 2 : // z
                limit = nw_ ; break ;
            default :
                ogf_assert_not_reached ;
            }
            //if (d > double(limit)-0.5) { return(limit-1) ; }
            if (d == double(limit)) { return(limit-1) ; }
            //else { return int_round(d) ; }
            else { return pix_round(d) ; }
        }

        double compute_distance(double x, double y, double z, double u, double v, double w) {
            double x_dist = x-u ;
            double y_dist = y-v ;
            double z_dist = z-w ;
            double dist = sqrt((x_dist*x_dist)+(y_dist*y_dist)+(z_dist*z_dist)) ;
            return(dist) ;
        }

        coord int_round(double x) {
            coord floor = int(x) ;
            coord output = ((x-double(floor))<=0.5 ) ? floor : floor+1 ;
            return(output) ;
        }

        coord pix_round(double x) {
            return(int(x)) ;
        }

        void do_face() {
            int nb_vertices = current_vertex_ ;
		
            //std::cerr << "nb vertices:" << nb_vertices << std::endl ;
		
            coord normal_u ;
            coord normal_v ;
            coord normal_w ;
            get_face_normal(normal_u, normal_v, normal_w) ; // Q : how come the normal is never used ?
            if(normal_w == 0) {
                return ;
            }
            int min = get_min_vertex() ;
            for(
                int i = (min + 1) % nb_vertices ; 
                (i + 1) % nb_vertices != min ;  
                i = (i + 1) % nb_vertices
            ) {
                //std::cerr << "call do_triangle" << std::endl ;
                do_triangle(min, i, (i+1)%nb_vertices) ;
                //std::cerr << "done triangle" << std::endl ;
            }

        }
	    
        void do_polyhedron() {
            if (barycentric_ponderation_) {
                min_imp = 0.0 ;
                max_imp = 1.0 ;
                for(coord v = polyhedron_v_min_ ; v < polyhedron_v_max_ +1 ; v++) { //ch
                    for(coord u = polyhedron_left_[v].u ; u < polyhedron_right_[v].u +1 ; u++) { //ch
                        coord w1 = bottom_plane_[v * nu_ + u] ;
                        coord w2 = top_plane_[v * nu_ + u] ;
                        if(w1 < 0 || w2 >= nw_) {
                            Logger::out("3DRasterizer") << "Problem in Rasterizer : Overflow" << std::endl ;
                            continue ;
                        }
                        for(coord w = w1 /*+1*/  ; w < w2 /*+1*/ ; w++) { //ch
                            int uc = u ;
                            ogf_clamp(uc, 0, PROCESS::w_ - 1) ;
                            int vc = v ;
                            ogf_clamp(vc, 0, PROCESS::h_ - 1) ;

                            double importance ;
                            importance = 1.0 -(compute_distance(uc,vc,w,ug,vg,wg)/d_max) ; 
                            if (min_imp>importance) {min_imp = importance ;}
                            if (max_imp<importance) {max_imp = importance ;}
                        }
                    }
                }
            }

            for(coord v = polyhedron_v_min_ ; v < polyhedron_v_max_ +1 ; v++) { //ch
                for(coord u = polyhedron_left_[v].u ; u < polyhedron_right_[v].u +1 ; u++) { //ch
                    coord w1 = bottom_plane_[v * nu_ + u] ;
                    coord w2 = top_plane_[v * nu_ + u] ;
                    if(w1 < 0 || w2 >= nw_) {
                        Logger::out("3DRasterizer") << "Problem in Rasterizer : Overflow" << std::endl ;
                        continue ;
                    }
                    for(coord w = w1 /*+1*/  ; w < w2 /*+1*/ ; w++) { //ch
                        int uc = u ;
                        ogf_clamp(uc, 0, PROCESS::w_ - 1) ;
                        int vc = v ;
                        ogf_clamp(vc, 0, PROCESS::h_ - 1) ;

                        // 2kill
                        /*if (w == w1 || w==w2-1) {
                          PROCESS::mark( uc, vc, w, 0.5) ;
                          PROCESS::pixel( uc, vc, w, 0.5) ;
                          }
                          else {
                          PROCESS::mark(uc, vc, w, 1.0) ;
                          PROCESS::pixel( uc, vc, w, 1.0) ;
                          }*/

                        // barycentric ponderation :

                        double importance ;
                        if (barycentric_ponderation_) {
                            importance = 1.0 -(compute_distance(uc,vc,w,ug,vg,wg)/d_max) ; 
                            importance = (importance-min_imp)/(max_imp-min_imp) ;
                            if (!(importance>=0.0 && importance <=1.0)) {
                                Logger::out("3DRasterizer") << "Problem : weight out of [0,1] = " << importance << std::endl;
                            }
                            else {
                                PROCESS::mark(uc, vc, w, importance) ;
                                PROCESS::pixel( uc, vc, w, importance) ;
                                //if (min_imp>importance) {min_imp = importance ;}
                                //if (max_imp<importance) {max_imp = importance ;}
                            }
                        }
                        else {
                            importance = 1.0 ;
                            PROCESS::mark(uc, vc, w, importance) ;
                            PROCESS::pixel( uc, vc, w, importance) ;
                        }
                    }
                }
            }
        }
	    
        void get_face_normal(coord& u, coord& v, coord& w) {
            int nb_vertices = current_vertex_ ;
            u = 0 ;
            v = 0 ;
            w = 0 ;
            for(int i=0; i<nb_vertices; i++) {
                int j = i+1 ;
                if(j >= nb_vertices) {
                    j = 0 ;
                }
                int k = j+1 ;
                if(k >= nb_vertices) {
                    k = 0 ;
                }
		        
                coord u1 = vertex_buffer_[i].u() -  vertex_buffer_[j].u();
                coord v1 = vertex_buffer_[i].v() -  vertex_buffer_[j].v();
                coord w1 = vertex_buffer_[i].w() -  vertex_buffer_[j].w();
		
                coord u2 = vertex_buffer_[k].u() -  vertex_buffer_[j].u();
                coord v2 = vertex_buffer_[k].v() -  vertex_buffer_[j].v();
                coord w2 = vertex_buffer_[k].w() -  vertex_buffer_[j].w();
		
                u += v1 * w2 - w1 * v2 ;
                v += w1 * u2 - u1 * w2 ;
                w += u1 * v2 - v1 * u2 ;
            }		    
        }
	    
        void get_face_v_extents(coord& vmin, coord& vmax) {
            int nb_vertices = current_vertex_ ;
            vmin = large_coord ;
            vmax = -large_coord ;
            for(int i=0; i<nb_vertices; i++) {
                vmin = vgl_min(vmin, vertex_buffer_[i].v()) ;
                vmax = vgl_max(vmax, vertex_buffer_[i].v()) ;
            }
        }
	    
        void get_triangle_v_extents(int ind[3], coord& vmin, coord& vmax) {
            vmin = large_coord ;
            vmax = -large_coord ;
            for(int i=0; i<3; i++) {
                vmin = vgl_min(vmin, vertex_buffer_[ind[i]].v()) ;
                vmax = vgl_max(vmax, vertex_buffer_[ind[i]].v()) ;
            }
        }
	
        void do_triangle(int i, int j ,int k) {
            //std::cerr << "triangle " << i << " " << j << " " << k << std::endl ;
		
            { for(int i=0;i<nv_;i++) {
                    polygon_left_[i].u = large_coord ;
                    polygon_right_[i].u = -large_coord ;
                } }
		
            int ind[3] ;
            ind[0] = i ;
            ind[1] = j ;
            ind[2] = k ;
		
            bool is_top_face = false ;
		
            coord v_min ;
            coord v_max ;
            get_triangle_v_extents(ind, v_min, v_max) ;
		
            //std::cerr << "vmin=" << v_min << " vmax=" << v_max << std::endl ;
		
            if(v_min == v_max) {
                return ;
            }
		
            polyhedron_v_min_ = vgl_min(v_min, polyhedron_v_min_) ;
            polyhedron_v_max_ = vgl_max(v_max, polyhedron_v_max_) ;    
		
            {for(int i=0; i < 3; i++) {
		
                    int j = (i+1)%3 ;
		        
                    //one edge of the triangle is selected
                    coord u1 = vertex_buffer_[ind[i]].u() ;
                    coord v1 = vertex_buffer_[ind[i]].v() ;
                    coord w1 = vertex_buffer_[ind[i]].w() ;
                    coord u2 = vertex_buffer_[ind[j]].u() ;
                    coord v2 = vertex_buffer_[ind[j]].v() ;
                    coord w2 = vertex_buffer_[ind[j]].w() ;
		
                    if(v1 == v2) {
                        continue ;
                    }
		
                    bool is_left_side = false ;
		
                    // I want the set of lit pixels to be
                    // independant from the order of the 
                    // extremities.
		        
                    bool swp = 0 ;
		        
                    if(v2 == v1) {
                        if(u1 > u2) {
                            swp = true ;
                        }
                    } else {
                        if(v1 > v2) {
                            swp = true ;
                        }
                    }
		
                    if(swp) {
                        vgl_swap(u1,u2) ;
                        vgl_swap(v1,v2) ;
                        vgl_swap(w1,w2) ;
                    }
		
                    // Bresenham algorithm for the selected edge of the triangle.
                    coord du = u2 - u1 ;
                    coord dv = v2 - v1 ;
                    coord dw = w2 - w1 ;
                    coord su = vgl_sgn(du) ;
                    coord sv = vgl_sgn(dv) ;
                    coord sw = vgl_sgn(dw) ;
                    du *= su ;
                    dv *= sv ;
                    dw *= sw ;
                    coord u = u1 ;
                    coord v = v1 ;
                    coord w = w1 ;
				
                    update_left_right_bucket(u,v,w,is_left_side) ; // Q : how come "is left side" and "is_top_face" are never used ?
		
                    coord eu = dv - /*2 **/ du ; // (vertices are in the middle of the voxels)
                    coord ew = dv - /*2 **/ dw ; // (vertices are in the middle of the voxels)
                    while(v < v2 - 1) {
		
                        v += sv ;
                        eu -= 2 * du ;
                        ew -= 2 * dw ;
		
                        while(eu < 0) { // check stopping criterion : stopping too early ? too late ?
                            u += su ;
                            eu += 2 * dv ;
                        }
		
                        while(ew < 0) { // check stopping criterion : stopping too early ? too late ?
                            w += sw ;
                            ew += 2 * dv ;
                        }

                        update_left_right_bucket(u,v,w,is_left_side) ; // the rasterized pixel limits in u and w FOR THIS EDGE are stored, IF they extend the already known domain.
                    } 
                    update_left_right_bucket(u2,v2,w2,is_left_side) ; // because the loop goes only up to v2-1. //try2kill
                }}
            // end of loop for selected edge of triangle
		    
            // now the u and w are determined on the EDGES of the projected polyhedron,
            // let's rasterize the inner domain (between the edges), for the w coordinate.
            // Reminder : this is still for one triangle out of the whole face. it's possible for another triangle of the same face to span a larger w domain.
            for(coord v = v_min; v <= v_max; v++) {
                coord u1 = polygon_left_[v].u ; 
                coord w1 = polygon_left_[v].w ; 
                coord u2 = polygon_right_[v].u ;
                coord w2 = polygon_right_[v].w ;
		
                coord u = u1 ;
                coord w = w1 ;
                coord du = u2 - u1 ;
                coord dw = w2 - w1 ;
                coord su = vgl_sgn(du) ;
                coord sw = vgl_sgn(dw) ;
                du *= su ;
                dw *= sw ;
		
                //update_bot_top_bucket(u,v,w,is_top_face) ;
		
                coord ew = du - /*2 **/ dw ; // (vertices are in the middle of the voxels)
                while(u < u2 - 1) {
		            
                    u += su ;
                    ew -= 2 * dw ;
		
                    while(ew < 0) {
                        w += sw ;
                        ew += 2 * du ;
                    }
                    update_bot_top_bucket(u,v,w,is_top_face) ;
                } 
		
                u = u2 ; w = w2 ;
                update_bot_top_bucket(u,v,w,is_top_face) ; // the rasterized pixel limits in w FOR THIS TRIANGLE are stored, IF they extend the already known domain.
            }
        }

	    
        int get_min_vertex() {
            int nb_vertices = current_vertex_ ;
            int min = 0 ;
            for(int i=1; i<nb_vertices; i++) {
                if(vertex_buffer_[i].u() < vertex_buffer_[min].u()) {
                    min = i ;
                } else {
                    if(vertex_buffer_[i].u() == vertex_buffer_[min].u()) {
                        if(vertex_buffer_[i].v() < vertex_buffer_[min].v()) {
                            min = i ;
                        } else {
                            if(vertex_buffer_[i].v() == vertex_buffer_[min].v()) {
                                if(vertex_buffer_[i].w() < vertex_buffer_[min].w()) {
                                    min = i ;
                                }
                            }
                        }
                    }
                }
            }
            return min ;
        }
	
	
        void update_left_right_bucket(coord u, coord v, coord w, bool is_left_side) {
            if(u < polygon_left_[v].u) {
                polygon_left_[v].u = u ;
                polygon_left_[v].w = w ;
            }
		
            polyhedron_left_[v].u = vgl_min(polyhedron_left_[v].u, u) ;  
		
            if(u > polygon_right_[v].u) {
                polygon_right_[v].u = u ;
                polygon_right_[v].w = w ;
            }
		
            polyhedron_right_[v].u = vgl_max(polyhedron_right_[v].u, u) ;  
        }
	    
        void update_bot_top_bucket(coord u, coord v, coord w, bool is_top) {
            if(w < bottom_plane_[v * nu_ + u]) {
                bottom_plane_[v * nu_ + u] = w ;
            }
		
            if(w > top_plane_[v * nu_ + u]) {
                top_plane_[v * nu_ + u] = w ;
            }		    
        }

 	
    private:
        GenericRasterizer3d(const GenericRasterizer3d& rhs) ;
        GenericRasterizer3d& operator=(const GenericRasterizer3d& rhs) ;
	
    private:
        coord nu_ ;
        coord nv_ ;
        coord nw_ ;
        coord nuv_ ;
        coord nuvw_ ;
        coord polyhedron_v_min_ ;
        coord polyhedron_v_max_ ;
        PolygonBucket* polygon_left_ ;
        PolygonBucket* polygon_right_ ;
        PolyhedronBucket* polyhedron_left_ ;
        PolyhedronBucket* polyhedron_right_ ;
        coord* bottom_plane_ ;
        coord* top_plane_ ;
        coord polyhedron_y_min_ ;
        coord polyhedron_y_max_ ;
        float value_ ;
        bool no_empty_cells_ ; // if true, the center of gravity of the cells is passed to the PROCESS::pixel. It avoids having no PROCESS::pixel calls in the case of very little cells (containing no complete voxels).
        bool barycentric_ponderation_ ;
        double ug,vg,wg ;
        double d_max ; // max of the distance between a tet's center of mass and its vertices.
        double min_imp, max_imp ;

        enum {vertex_buffer_size = 1000} ;
        Vertex vertex_buffer_[vertex_buffer_size] ;
        int current_vertex_ ;
    };




/**************************************************************************/	

    class CELLS_API RasterizerProcess3d { //base class for the Rasterizer Process. The actual rasterizer process will derive from this class (and will need to have a "pixel" function)
    public:
        //RasterizerProcess3d() : image_(nil),w_(0), h_(0), d_(0) { }
        RasterizerProcess3d() {
            image_ = nil ;
            w_ = 0 ;
            h_ = 0 ;
            d_ = 0 ;
            wd_ = 0.0 ;
            hd_ = 0.0 ;
            dd_ = 0.0 ;
        }

        RasterizerProcess3d(Image* image) {
            set_image(image) ;
        }

        void set_image_dimensions(int w, int h, int d) {
            w_ = w ;
            h_ = h ;
            d_ = d ;
            wd_ = double(w_) ;
            hd_ = double(h_) ;
            dd_ = double(d_) ;
            inv_w_ = 1.0 / wd_ ;
            inv_h_ = 1.0 / hd_ ;
            inv_d_ = 1.0 / dd_ ;
        }
        void set_image(Image* image0) { 
            image_ = image0 ; 
            if(image0 != nil) {
                set_image_dimensions(image0->width(), image0->height(), image0->depth()) ;
            } 
        }
        Image* image() const { return image_ ; }

        void set_exponent(int exponent) {
            exponent_ = exponent ;
        }

        void set_zscaling(double zscaling) {
            zscaling_ = zscaling ;
        }
        
        void double_to_pixel(double xd, double yd, double zd, int& x, int& y, int& z) const {
            x = int(xd /** wd_*/) ;
            ogf_clamp(x, 0, w_ - 1) ;
            y = int(yd /** hd_*/) ;
            ogf_clamp(y, 0, h_ - 1) ;
            z = int(zd /** dd_*/) ;
            ogf_clamp(z, 0, d_ - 1) ;
        }

        void pixel_to_vertex(int x, int y, int z, double& xd, double& yd, double& zd) const {
            xd = (double(x) + 0.5) /** inv_w_*/ ;
            yd = (double(y) + 0.5) /** inv_h_*/ ;
            zd = (double(z) + 0.5) /** inv_d_*/ ;
        }
        
        void pixel_to_double(int x, int y, int z, double& xd, double& yd, double& zd) const {
            xd = (double(x) + 0.5) * inv_w_ ;
            yd = (double(y) + 0.5) * inv_h_ ;
            zd = (double(z) + 0.5) * inv_d_ ;
        }
        //}
    protected:
        Image* image_ ;
        int exponent_ ;
        int w_ ;
        int h_ ;
        int d_ ;
        double wd_ ;
        double hd_ ;
        double dd_ ;
        double inv_w_ ;
        double inv_h_ ;
        double inv_d_ ;
        double zscaling_ ;
    } ;	

/**************************************************************************/

    class CELLS_API CellValueProcess : public RasterizerProcess3d {
    public:
        typedef int coord ;
        CellValueProcess(Image* image) : RasterizerProcess3d(image) { } ;
        CellValueProcess() { } ;
        double value(Memory::pointer p) { return double(*(float*)p) ;  }
		
        virtual void pixel(coord u,coord v,coord w, double weight) ;
        virtual void mark(coord u,coord v,coord w, double weight) ;

        virtual void reset_acc() ;
        virtual double get_r() ;
        virtual double get_g() ;
        virtual double get_b() ;
        virtual double get_a() ;

    private:
    } ;

/**************************************************************************/

}	

#endif
