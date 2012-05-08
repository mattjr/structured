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

#include <OGF/math/geometry/polygon2d.h>
#include <OGF/math/geometry/polygon3d.h>
#include <OGF/math/geometry/types.h>
#include <OGF/basic/containers/arrays.h>
#include <stdlib.h> // for qsort

namespace OGF {

    namespace Geom {


//------------------------ Helpers for convex hull ---------------------
// Note: this implementation uses qsort, it would be possible to rewrite
// it using STL's algorithms (would be cleaner and faster)

        inline bool ccw(
            Point2d* P, int i, int j, int k
        ) {
            const Point2d& pi = P[i] ; 
            const Point2d& pj = P[j] ;
            const Point2d& pk = P[k] ;
            double a = pi.x() - pj.x() ;
            double b = pi.y() - pj.y() ;
            double c = pk.x() - pj.x() ;
            double d = pk.y() - pj.y() ;
            return (a*d - b*c <= 0) ;
        }
    
        inline int cmp_sgn(
            const Point2d& p1, const Point2d& p2
        ) {
            double b1 = p1.x() - p2.x() ;
            if(b1 > 0) {
                return 1 ;
            }
            if(b1 < 0) {
                return -1 ;
            }
            double b2 = p1.y() - p2.y() ;
            if(b2 > 0) {
                return 1 ;
            }
            if(b2 < 0) {
                return -1 ;
            }
            return 0 ;
        } 

        // The comparison functions used by qsort takes pointer
        // to the things to sort as arguments.
        static int cmpl(Point2d* p1, Point2d* p2) {
            return cmp_sgn(*p1, *p2) ;
        }
        
        static int cmph(Point2d* p1, Point2d* p2) {
            return cmp_sgn(*p2, *p1) ;
        }

        typedef int (*cmpfunc)(const void*, const void*) ;

        static int make_chain(
            Point2d* V, int n,
            int (*cmp)(Point2d*, Point2d*)
        ) {
            int i, j, s = 1;
            Point2d t;
            
            ::qsort(V, n, sizeof(Point2d), cmpfunc(cmp));

            for (i=2; i<n; i++) {
                for (j=s; j>=1 && ccw(V, i, j, j-1); j--){}
                s = j+1;
                t = V[s]; V[s] = V[i]; V[i] = t;
            }
            return s;
        }

//------------------------------------------------------------------------------


        void convex_hull(const Polygon2d& PP, Polygon2d& result) {
            result.clear() ;
            int n = PP.size() ;
            Point2d* P = new Point2d[n+1] ;
            { for(int i=0; i<n; i++) {
                P[i] = PP[i] ;
            }}
            int u = make_chain(P, n, cmpl);  
            P[n] = P[0];
            int ch = u+make_chain(P+u, n-u+1, cmph);  
            {for(int i=0; i<ch; i++) {
                result.push_back(P[i]) ;
            }}
            delete[] P ;
        }
        
        void minimum_area_enclosing_rectangle(
            const Polygon2d& PP, 
            Vector2d& S, Vector2d& T
        ) {

            // Note: this implementation has O(n2) complexity :-(
            // (where n is the number of vertices in the convex hull)
            // If this appears to be a bottleneck, use a smarter
            // implementation with better complexity.

            Polygon2d P ;
            convex_hull(PP, P) ;

            int N = P.size() ;
            
            // Add the first vertex at the end of P
            P.push_back(P[0]) ;

            double min_area = Numeric::big_double ;

            for(int i=1; i<=N; i++) {
                Vector2d Si = P[i] - P[i-1] ;

                if(Si.norm2() < 1e-20) {
                    continue ;
                }

                Vector2d Ti(-Si.y(), Si.x()) ;
                Si.normalize() ;
                Ti.normalize() ;
                double s0 =  Numeric::big_double ;
                double s1 = -Numeric::big_double ;
                double t0 =  Numeric::big_double ;
                double t1 = -Numeric::big_double ; 
                for(int j=1; j<N; j++) {
                    Vector2d D = P[j] - P[0] ;
                    double s = Si * D ;
                    s0 = ogf_min(s0, s) ;
                    s1 = ogf_max(s1, s) ;
                    double t = Ti * D ;
                    t0 = ogf_min(t0, t) ;
                    t1 = ogf_max(t1, t) ;
                }
                double area = (s1 - s0) * (t1 - t0) ;
                if(area < min_area) {
                    min_area = area ;
                    if((s1 - s0) < (t1 - t0)) {
                        S = Si ;
                        T = Ti ;
                    } else {
                        S = Ti ;
                        T = Si ;
                    }
                }
            }
        }

        bool point_is_in_kernel(const Polygon2d& P, const Point2d& p) {
            Sign sign = ZERO ;
            for(unsigned int i=0 ; i<P.size() ; i++) {
                unsigned int j = (i+1) % P.size() ;
                const Point2d& p1 = P[i] ;
                const Point2d& p2 = P[j] ;

                Sign cur_sign = orient(p, p1, p2) ;
                if(sign == ZERO) {
                    sign = cur_sign ;
                } else {
                    if(cur_sign != ZERO && cur_sign != sign) {
                        return false ;
                    }
                }
            }
            return true ;
        }


        // http://astronomy.swin.edu.au/~pbourke/geometry/polyarea/
        Point2d barycenter(const Polygon2d& P) {
            ogf_assert(P.size() > 0) ;

            double A = signed_area(P) ;

            if(::fabs(A) < 1e-30) {
                return P[0] ;
            }

            double x = 0.0 ;
            double y = 0.0 ;
            for(unsigned int i=0; i<P.size(); i++) {
                unsigned int j = (i+1) % P.size() ;
                const Point2d& t1 = P[i] ;
                const Point2d& t2 = P[j] ;
                double d = (t1.x() * t2.y() - t2.x() * t1.y()) ;
                x += (t1.x() + t2.x()) * d ;
                y += (t1.y() + t2.y()) * d ;
            }
        
            return Point2d(
                x / (6.0 * A),
                y / (6.0 * A)
            ) ;
        }
        

        Point2d vertices_barycenter(const Polygon2d& P) {
            ogf_assert(P.size() != 0) ;
            double x = 0 ;
            double y = 0 ;
            for(unsigned int i=0; i<P.size(); i++) {
                x += P[i].x() ;
                y += P[i].y() ;
            }
            x /= double(P.size()) ;
            y /= double(P.size()) ;
            return Point2d(x,y) ;
        }

        static inline Sign opposite(Sign sign_in) {
            return Sign(-int(sign_in)) ;
        }

        static inline Sign point_is_in_half_plane(
            const Point2d& p, const Point2d& q1, const Point2d& q2,
            bool invert
        ) {
            Sign result = orient(q1, q2, p) ;
            if(invert) {
                result = opposite(result) ;
            }
            return result ;
        }

        static inline bool intersect_segments(
            const Point2d& p1, const Point2d& p2,
            const Point2d& q1, const Point2d& q2,
            Point2d& result
        ) {

            Vector2d Vp = p2 - p1 ;
            Vector2d Vq = q2 - q1 ;
            Vector2d pq = q1 - p1 ;

            double a =  Vp.x() ;
            double b = -Vq.x() ;
            double c =  Vp.y() ;
            double d = -Vq.y() ;

            double delta = a*d-b*c ;
            if(delta == 0.0) {
                return false ;
            }
            
            double tp = (d * pq.x() -b * pq.y()) / delta ;
            
            result = Point2d(
                (1.0 - tp) * p1.x() + tp * p2.x(),
                (1.0 - tp) * p1.y() + tp * p2.y()
            ) ;

            return true ;
        }

        void save_polygon(const Polygon2d& P, const std::string& file_name) {
            std::ofstream out(file_name.c_str()) ;
            {for(unsigned int i=0; i<P.size(); i++) {
                out << "v " << P[i].x() << " " << P[i].y() << std::endl ;
                out << "vt " << P[i].x() << " " << P[i].y() << std::endl ;
            }}
            out << "f " ;
            {for(unsigned int i=0; i<P.size(); i++) {
                out << i+1 << "/" << i+1 << " " ;
            }}
            out << std::endl ;
        }

        void clip_polygon_by_half_plane(
            const Polygon2d& P, 
            const Point2d& q1,
            const Point2d& q2,
            Polygon2d& result,
            bool invert
        ) {
            result.clear() ;

            if(P.size() == 0) {
                return ;
            }

            if(P.size() == 1) {
                if(point_is_in_half_plane(P[0], q1, q2, invert)) {
                    result.push_back(P[0]) ;
                }
                return ;
            }

            Point2d prev_p = P[P.size() - 1] ;
            Sign prev_status = point_is_in_half_plane(
                prev_p, q1, q2, invert
            ) ;

            for(unsigned int i=0; i<P.size(); i++) {
                Point2d p = P[i] ;
                Sign status = point_is_in_half_plane(
                    p, q1, q2, invert
                ) ;
                if(
                    status != prev_status &&
                    status != ZERO &&
                    prev_status != ZERO
                ) {
                    Point2d intersect ;
                    if(intersect_segments(prev_p, p, q1, q2, intersect)) {
                        result.push_back(intersect) ;
                    } else {
                    }
                }

                switch(status) {
                case NEGATIVE:
                    break ;
                case ZERO:
                    result.push_back(p) ;
                    break ;
                case POSITIVE:
                    result.push_back(p) ;
                    break ;
                }

                prev_p = p ;
                prev_status = status ;
            }
        }

        void clip_segment_by_half_plane(
            Segment2d& S,
            const Point2d& Q1, const Point2d& Q2,
            bool invert 
        ) {

            if(S.empty) {
                return ;
            }

            Sign P1_status = point_is_in_half_plane(
                S.p1, Q1, Q2, invert
            ) ;

            Sign P2_status = point_is_in_half_plane(
                S.p2, Q1, Q2, invert
            ) ;

            if(P1_status == NEGATIVE && P2_status == NEGATIVE) {
                S.empty = true ;
                return ;
            }
            
            if(P1_status != P2_status) {
                Point2d I ;
                intersect_segments(S.p1, S.p2, Q1, Q2, I) ;
                if(P1_status == POSITIVE || P1_status == ZERO) {
                    S.p2 = I ;
                } else {
                    S.p1 = I ;
                }
            }
        }


        // Compute the kernel using Sutherland-Hogdman reentrant clipping
        // The kernel is obtained by clipping the polygon with each 
        // half-plane yielded by its sides.
        void kernel(const Polygon2d& P, Polygon2d& result) {

            Array1d<Sign> sign(P.size()) ;
            for(unsigned int i=0; i<P.size(); i++) {
                unsigned int j = ((i+1) % P.size()) ;
                unsigned int k = ((j+1) % P.size()) ;
                sign(j) = orient(P[i],P[j],P[k]) ;
            }

            bool invert = (signed_area(P) < 0) ;

            Polygon2d tmp1 = P ;
            Polygon2d tmp2 ;
            Polygon2d* src = &tmp1 ;
            Polygon2d* dst = &tmp2 ;
            for(unsigned int i=0; i<P.size(); i++) {
                unsigned int j = ((i+1) % P.size()) ;
                const Point2d& p1 = P[i] ;
                const Point2d& p2 = P[j] ;

                if((p2-p1).norm() == 0) {
                    std::cerr << "null edge in poly" << std::endl ;
                    continue ;
                }

                // Optimization: do not clip by convex-convex edges
                // (Thanks to Rodrigo Toledo for the tip !)

                if(!invert && sign(i) != NEGATIVE && sign(j) != NEGATIVE) {
                    continue ;
                }

                if(invert && sign(i) != POSITIVE && sign(j) != POSITIVE) {
                    continue ;
                }

                clip_polygon_by_half_plane(*src, p1, p2, *dst, invert) ;
                ogf_swap(src, dst) ;

            }
            result = *src ;
        }

        // http://astronomy.swin.edu.au/~pbourke/geometry/polyarea/
        double signed_area(const Polygon2d& P) {
            double result = 0 ;
            for(unsigned int i=0; i<P.size(); i++) {
                unsigned int j = (i+1) % P.size() ;
                const Point2d& t1 = P[i] ;
                const Point2d& t2 = P[j] ;
                result += t1.x() * t2.y() - t2.x() * t1.y() ;
            }
            result /= 2.0 ;
            return result ;
        }

        // Clipping with convex window using Sutherland-Hogdman reentrant clipping
        void convex_clip_polygon(
            const Polygon2d& P, const Polygon2d& clip, Polygon2d& result
        ) {
            ogf_parano_assert(polygon_is_convex(clip)) ;
            Polygon2d tmp1 = P ;
            bool invert = (signed_area(tmp1) != signed_area(clip)) ;
            Polygon2d tmp2 ;
            Polygon2d* src = &tmp1 ;
            Polygon2d* dst = &tmp2 ;
            for(unsigned int i=0; i<clip.size(); i++) {
                unsigned int j = ((i+1) % clip.size()) ;
                const Point2d& p1 = clip[i] ;
                const Point2d& p2 = clip[j] ;
                clip_polygon_by_half_plane(*src, p1, p2, *dst, invert) ;
                ogf_swap(src, dst) ;
            }
            result = *src ;
        }

        void convex_clip_segment(
            Segment2d& S, const Polygon2d& window
        ) {
	    ogf_parano_assert(polygon_is_convex(window)) ;
            bool invert = (signed_area(window) < 0) ;
            for(unsigned int i=0; i<window.size(); i++) {
                unsigned int j = ((i+1) % window.size()) ;
                clip_segment_by_half_plane(S, window[i], window[j], invert) ;
            }
        }

        bool polygon_is_convex(const Polygon2d& P) {
            Sign s = ZERO ;
            for(unsigned int i=0; i<P.size(); i++) {
                unsigned int j = ((i+1) % P.size()) ;
                unsigned int k = ((j+1) % P.size()) ;
                Sign cur_s = orient(P[i],P[j],P[k]) ;
                if(s != ZERO && cur_s != ZERO && cur_s != s) {
                    return false ;
                }
                if(cur_s != ZERO) {
                    s = cur_s ;
                }
            }
            return true ;
        }


        void barycentric_coords(const Polygon2d& P, const Point2d& p, std::vector<double>& bary) {
            // Uses the 3D version.
            Polygon3d Q ;
            for(unsigned int i=0; i<P.size(); i++) {
                Q.push_back(Point3d(P[i].x(), P[i].y(), 0.0)) ;
            }
            Point3d q(p.x(), p.y(), 0.0) ;
            barycentric_coords(Q, q, bary) ;
        }

    }
}
