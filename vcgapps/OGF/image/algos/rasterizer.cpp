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

#include <OGF/image/algos/rasterizer.h>
#include <OGF/image/types/image.h>

namespace OGF {

    Rasterizer::Rasterizer(Image* target) {
        target_ = target ;
        ogf_assert(target_->color_encoding() == Image::RGBA) ;
        width_ = target_->width() ;
        height_ = target_->height() ;
        bytes_per_line_ = 4 * width_ ;
        left_ = new ScanLineExtremity[height_] ;
        right_ = new ScanLineExtremity[height_] ;
        graph_mem_ = target_->base_mem() ;
    }

    Rasterizer::~Rasterizer() {
        target_ = nil ;
        delete[] left_ ; left_ = nil ;
        delete[] right_ ; right_ = nil ;
    }

    void Rasterizer::begin_polygon() {
        current_polygon_index_ = 0 ;
    }
    
    void Rasterizer::vertex(const Point2d& p, const Color& c) {
        ogf_assert(current_polygon_index_ < MAX_POLYGON) ;
        Vertex& v = current_polygon_[current_polygon_index_] ;
        v.x = int(p.x() * (width_ - 1)) ;
        v.y = int(p.y() * (height_ - 1)) ;
        ogf_clamp(v.x, 0, width_  - 1) ;
        ogf_clamp(v.y, 0, height_ - 1) ;
        v.r = int(c.r() * 255.0) ;
        v.g = int(c.g() * 255.0) ;
        v.b = int(c.b() * 255.0) ;
        v.a = int(c.a() * 255.0) ; 
        ogf_clamp(v.r, 0, 255) ;
        ogf_clamp(v.g, 0, 255) ;
        ogf_clamp(v.b, 0, 255) ;
        ogf_clamp(v.a, 0, 255) ;
        current_polygon_index_++ ;
    }
    
    void Rasterizer::end_polygon() {
        rasterize(current_polygon_, current_polygon_index_) ;
    }


    void Rasterizer::triangle(
        const Point2d& p0, const Color& c0,
        const Point2d& p1, const Color& c1,
        const Point2d& p2, const Color& c2
    ) {
        begin_polygon() ;
        vertex(p0,c0) ;
        vertex(p1,c1) ;
        vertex(p2,c2) ;
        end_polygon() ;
    }

    static inline int nmg_sgn(int x) {
        return x > 0 ? 1 : (x < 0 ? -1 : 0) ;
    }
    
    void Rasterizer::rasterize(Vertex* v, int nb_vertices) {
        int clock_wise = 0 ;
        int miny = 100000;
        int maxy = -1;

        int i,j,u ;
        
        if(nb_vertices < 2) {
            return ;
        }

        // Step 1: Get clockwise, min and max y 

        for(i=0; i<nb_vertices; i++) {
            int j = i+1;
            if(j == nb_vertices) {
                j = 0;
            }

            int k = j+1;
            if(k == nb_vertices) {
                k = 0;
            }

            int dx1 = v[i].x - v[j].x;
            int dy1 = v[i].y - v[j].y;
            int dx2 = v[k].x - v[j].x;
            int dy2 = v[k].y - v[j].y;
	
            clock_wise += dx1 * dy2 - dx2 * dy1;

            miny = ogf_min(miny,v[i].y);
            maxy = ogf_max(maxy,v[i].y);
        }

        clock_wise = (clock_wise > 0);

        if(miny == maxy) {
            return;
        }

        
        // Step 2: rasterize border

        int y,y1,y2,dy,sy;
        int x,x1,x2,dx,sx,ex;
        int r,r1,r2,dr,sr,er;
        int g,g1,g2,dg,sg,eg;
        int b,b1,b2,db,sb,eb;
        int a,a1,a2,da,sa,ea;


        for(i=0; i<nb_vertices; i++) {
            j = i+1;
            
            if(j==nb_vertices) {
                j = 0;
            }


            if(v[i].y == v[j].y)
                continue;
      
      
            y1 = v[i].y;
            y2 = v[j].y;
            dy = y2 - y1;
            sy = nmg_sgn(dy);
            dy *= sy;
            y  = y1;
            

            x1 = v[i].x;
            x2 = v[j].x;
            dx = x2 - x1;
            sx = nmg_sgn(dx);
            dx *= sx;
            ex = (dx << 1) - dy;
            x  = x1;

            r1 = v[i].r;
            r2 = v[j].r;
            dr = r2 - r1;
            sr = nmg_sgn(dr);
            dr *= sr;
            er = (dr << 1) - dy;
            r  = r1;

            g1 = v[i].g;
            g2 = v[j].g;
            dg = g2 - g1;
            sg = nmg_sgn(dg);
            dg *= sg;
            eg = (dg << 1) - dy;
            g  = g1;

            b1 = v[i].b;
            b2 = v[j].b;
            db = b2 - b1;
            sb = nmg_sgn(db);
            db *= sb;
            eb = (db << 1) - dy;
            b  = b1;

            a1 = v[i].a;
            a2 = v[j].a;
            da = a2 - a1;
            sa = nmg_sgn(da);
            da *= sa;
            ea = (da << 1) - dy;
            a  = a1;

            ScanLineExtremity* extr = 
                (clock_wise ^ (y2 > y1)) ? right_ : left_ ;

	    for(u=0; u <= dy; u++) {
                extr[y].x = x;
                extr[y].r = r;
                extr[y].g = g;
                extr[y].b = b;
                extr[y].a = a;
                
		 y += sy;
		 while(ex >= 0) {
                     x += sx;
                     ex -= dy << 1;
                 }
		 ex += dx << 1;

		 while(er >= 0) {
                     r += sr;
                     er -= dy << 1;
                 }
		 er += dr << 1;

		 while(eg >= 0) {
                     g += sg;
                     eg -= dy << 1;
                 }
		 eg += dg << 1;

		 while(eb >= 0) {
                     b += sb;
                     eb -= dy << 1;
                 }
		 eb += db << 1;
                 
		 while(ea >= 0) {
                     a += sa;
                     ea -= dy << 1;
                 }
		 ea += da << 1;
            }
        }

        // Step 3: rasterize scanlines

        Memory::byte* graph_ptr0 = graph_mem_ + miny * bytes_per_line_ ;
        for(y=miny; y<=maxy; y++) {
            x1 = left_[y].x;
            x2 = right_[y].x;
            dx = x2 - x1 + 1;
            x  = x1;
            Memory::byte* graph_ptr = graph_ptr0 + (x1 << 2) ;

            r1 = left_[y].r;
            r2 = right_[y].r;
            dr = r2 - r1;
            sr = nmg_sgn(dr);
            dr *= sr;
            er = (dr << 1) - dx;
            r  = r1;

            g1 = left_[y].g;
            g2 = right_[y].g;
            dg = g2 - g1;
            sg = nmg_sgn(dg);
            dg *= sg;
            eg = (dg << 1) - dx;
            g  = g1;
            
            b1 = left_[y].b;
            b2 = right_[y].b;
            db = b2 - b1;
            sb = nmg_sgn(db);
            db *= sb;
            eb = (db << 1) - dx;
            b  = b1;

            a1 = left_[y].a;
            a2 = right_[y].a;
            da = a2 - a1;
            sa = nmg_sgn(da);
            da *= sa;
            ea = (da << 1) - dx;
            a  = a1;

            for(x=x1; x <= x2; x++) {
                graph_ptr[0] = Memory::byte(r) ;
                graph_ptr[1] = Memory::byte(g) ;
                graph_ptr[2] = Memory::byte(b) ;
                graph_ptr[3] = Memory::byte(a) ;
                
                while(er >= 0) {
                    r += sr;
                    er -= dx << 1;
                }
                er += dr << 1;

                while(eg >= 0) {
                    g += sg;
                    eg -= dx << 1;
                }
                eg += dg << 1;

                while(eb >= 0) {
                    b += sb;
                    eb -= dx << 1;
                }
                eb += db << 1;

                while(ea >= 0) {
                    a += sa;
                    ea -= dx << 1;
                }
                ea += da << 1;
                graph_ptr += 4 ;
            }
            graph_ptr0 += bytes_per_line_ ;
        }
    }
}

