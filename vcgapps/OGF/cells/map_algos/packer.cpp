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
 

#include <OGF/cells/map_algos/packer.h>
#include <OGF/cells/map/geometry.h>
#include <OGF/math/geometry/polygon2d.h>
#include <OGF/basic/debug/logger.h>
#include <OGF/math/geometry/average_direction.h>

#include <algorithm>
#include <math.h>

namespace OGF {

    class MapComponentBBox{
    public:

        MapComponentBBox(
            MapComponent* surf, const Point2d& min, const Point2d& max
        ) : 
            min_(min), max_(max), surf_(surf), min_func_(nil), max_func_(nil),
            nb_steps_(0) {
        }

        ~MapComponentBBox(){
            free() ;
            surf_ = nil ;
        }

        MapComponentBBox(const MapComponentBBox& rhs) {
            copy(rhs) ;
        }
        
        MapComponentBBox& operator=(const MapComponentBBox& rhs) {
            if(&rhs != this) {
                free() ;
                copy(rhs) ;
            }
            return *this ;    
        }

        void init_max_and_min_func(
            double step, double margin, int margin_width_in_pixels
        ) {

            if(min_func_ != nil) {
                delete[] min_func_ ;
            }
            if(max_func_ != nil) {
                delete[] max_func_ ;
            }

            nb_steps_ = int(1.0 + (max_.x() - min_.x()) / step ) ;
            min_func_ = new double[nb_steps_];
            max_func_ = new double[nb_steps_];
            for (int i=0;i<nb_steps_;i++){
                min_func(i) = max_.y() - min_.y();
                max_func(i) = 0;
            }
            { // TODO:  Would be easier to iterate on facets (now we can)...
                FOR_EACH_VERTEX(MapComponent, surf_, vi) {
                    Map::Vertex *v = vi;
                    double maxX=0;
                    double minX=max_.x();
                    double maxY=0;
                    double minY=max_.y();
                    Map::Halfedge* cir = v->halfedge();
                    do {
                        Point2d pt = cir->opposite()->tex_coord();
                        maxX = ogf_max(maxX, pt.x()) ;
                        minX = ogf_min(minX, pt.x()) ;
                        maxY = ogf_max(maxY, pt.y()) ;
                        minY = ogf_min(minY, pt.y()) ;
                        cir = cir->next_around_vertex() ;
                    } while (cir != v->halfedge());

                    int posx_min = int(
                        double((minX-min_.x()) / step) - 
                        margin_width_in_pixels
                    ) ;

                    int posx_max = int(
                        double((maxX-min_.x()) / step) + 
                        margin_width_in_pixels
                    ) ;
                    for (int posx = posx_min ; posx < posx_max ; posx++) {
                        if (posx>=0 && posx<nb_steps_) {
                            min_func(posx) = 
                                ogf_min(min_func(posx), minY - margin) ;
                            max_func(posx) = 
                                ogf_max(max_func(posx), maxY + margin) ;
                        }
                    }
                }
            }
            
        }

        double& max_func(int i) {
            ogf_assert(i>=0 && i<nb_steps_);
            return max_func_[i];
        }

        double& min_func(int i) {
            ogf_assert(i>=0 && i<nb_steps_);
            return min_func_[i];
        }

        Vector2d size() const {
            return max_-min_;
        }

        double area() {
            Vector2d s = size();
            return s.x()*s.y();
        }
        
        void translate(const Vector2d& v){
            min_=min_+v;
            max_=max_+v; 
            Geom::translate_component2d(surf_,v) ;
        }
        
        bool operator<(const MapComponentBBox& r) const {
            return size().y() > r.size().y();
        }

        const Point2d& min() const { return min_; }
        const Point2d& max() const { return max_; }
        
        Point2d& min() { return min_; }
        Point2d& max() { return max_; }

        MapComponent* surface() { return surf_; }


    protected:
     
        void copy(const MapComponentBBox& rhs) {
            surf_ = rhs.surf_ ;
            nb_steps_ = rhs.nb_steps_ ;

            if(rhs.min_func_ != nil) {
                min_func_ = new double[nb_steps_] ;
                max_func_ = new double[nb_steps_] ;
                for(int i=0; i<nb_steps_; i++) {
                    min_func_[i] = rhs.min_func_[i] ;
                    max_func_[i] = rhs.max_func_[i] ;
                }
            } else {
                ogf_assert(rhs.max_func_ == nil) ;
                min_func_ = nil ;
                max_func_ = nil ;
            }
            min_ = rhs.min_ ;
            max_ = rhs.max_ ;
        }
        
        void free() {
            delete[] min_func_ ;
            delete[] max_func_ ;
            min_func_ = nil ;
            max_func_ = nil ;
            nb_steps_ = 0 ;
        }

    private: 
        Point2d min_, max_ ;
        MapComponent* surf_ ;
        double* min_func_ ;
        double* max_func_ ;
        int nb_steps_ ;
    } ;


    class TetrisPacker {
    public :
        
        TetrisPacker() {
            nb_xpos_ = 1024;
            height_ = new double[nb_xpos_];
            image_size_in_pixels_  = 1024 ;
            margin_width_in_pixels_ = 4 ;
        }
        
        ~TetrisPacker() {
            delete[] height_ ;
            height_ = nil ;
        }

        void set_image_size_in_pixels(int size) {
            image_size_in_pixels_ = size ;
        }

        int margin_width_in_pixels() const { return margin_width_in_pixels_ ; }

        void set_margin_width_in_pixels(int width) {
            margin_width_in_pixels_ = width ;
        } 
        
        void add(const MapComponentBBox& r){
            data_.push_back(r);
        }

        // function used to order the MapComponentBBoxes
        static bool compare(
            const MapComponentBBox& b0, const MapComponentBBox& b1
        ) {
            return  b0.size().y() > b1.size().y();
        }
        
        void add_margin(double margin_size) {
            for (unsigned int i=0;i<data_.size();i++){
                data_[i].translate(
                    Vector2d(
                        -data_[i].min().x() + margin_size,
                        -data_[i].min().y() + margin_size
                    )
                ) ;
                data_[i].max() = data_[i].max() + 
                    Vector2d( 2.0 * margin_size, 2.0 * margin_size) ;
                ogf_assert(data_[i].max().x()>0);
                ogf_assert(data_[i].max().y()>0);
                data_[i].min() = Point2d(0,0) ;
            }
        }
        
        double max_height() {
            double result = 0;
            for (unsigned int i=0;i<data_.size();i++) {
                result = ogf_max(result, data_[i].max().y()) ;
            }
            return result;
        }

        void recursive_apply() {
            // compute margin
            double area = 0 ;      
            for (unsigned int numrect = 0 ; numrect <data_.size();numrect++ ) {
                area += data_[numrect].area() ;
            }
            double margin = 
                (::sqrt(area) / image_size_in_pixels_) * 
                margin_width_in_pixels_ ;
            add_margin(margin) ; 

            // find a first solution
            apply(margin);
            double scoreSup = max_height();
            double borneSup = width_;
            double decalborne = 0.5 * ::sqrt(scoreSup * width_);

            // dichotomy
            for  (int i=0;i<10;i++ ){
                double new_borne = borneSup - decalborne;
                apply(margin,new_borne);
                double max = max_height();
                if (max < new_borne){
                    borneSup = borneSup - decalborne;
                }
                decalborne/=2;
            }
            apply(margin, borneSup);
        }


        void apply(double margin, double width =-1) {
            width_ = width;
            for (unsigned int i=0; i<data_.size(); i++) {
                data_[i].translate(
                    Vector2d(
                        -data_[i].min().x(),
                        -data_[i].min().y()
		    )
                ) ;
            }

            // sort MapComponentBBoxes by their heights ...
            std::sort(data_.begin(),data_.end(),compare);            
            
            {//find the best width
                double max_bbox_width = 0;
                for (unsigned int i=0; i<data_.size(); i++) {
                    max_bbox_width= ogf_max(
                        max_bbox_width, data_[i].size().x()
                    );
                }
                
                if ( width == -1){
                    // try to have a square : width_ = sqrt(area);
                    double area =0;
                
                    for (unsigned int i=0 ; i<data_.size() ; i++){
                        area += data_[i].area();
                    }

                    width_ = ::sqrt(area) *1.1;
                

                    // resize if a square seems to be too bad 
                    // (the first piece is too high)

                    if (data_[0].size().y() > width_) {
                        width_ = area / data_[0].size().y();
                    }
                }

                // be sure all surface can fit in the width
                width_ = ogf_max(
                    max_bbox_width * (double(nb_xpos_ + 2) / double(nb_xpos_)),
                    width_
                );	
            }
            // set the step depending on the width and the discretisation
            step_ = width_ / nb_xpos_;


            // init local min and max height functions
            {
                for (
                    unsigned int numrect = 0 ; numrect <data_.size(); numrect++
                ) {
                data_[numrect].init_max_and_min_func(
                    step_, margin, margin_width_in_pixels_
                ) ;
                }
            }

            // init global height function
            { for (int x=0; x<nb_xpos_; x++) {
                height(x)=0;
            }}
            
            {for (unsigned int i=0;i<data_.size();i++) {
                place(data_[i]);
            }}
        }
        
        
    private:

        double& height(int x) {
            ogf_assert(x >= 0 && x < nb_xpos_) ;
            return height_[x] ;
        }

        const double& height(int x) const {
            ogf_assert(x >= 0 && x < nb_xpos_) ;
            return height_[x] ;
        }
        
        // place() manages the insertion of a new block

        void place(MapComponentBBox& rect){

            const int width_in_pas = int ( (rect.size().x() / step_) + 1 ) ;
            
            // find the best position
            int bestXPos=0;
            double bestHeight = Numeric::big_double;
            double bestFreeArea = Numeric::big_double;
            
            for (int x=0;x<nb_xpos_ - width_in_pas ;x++) {
                
                double localHeight = max_height(x,width_in_pas,rect);
                double localFreeArea = 
                    free_area(x,width_in_pas,localHeight)
                    + localHeight * rect.size().x();
                
                // the best position criterion is the position that 
                // minimize area lost
                // area could be lost under and upper the bbox
                if (localFreeArea < bestFreeArea){
                    bestXPos = x;
                    bestHeight = localHeight;
                    bestFreeArea = localFreeArea;
                }
            }

            // be sure we have a solution
            ogf_assert(bestHeight != Numeric::big_double);
            
            // place the rectangle
            rect.translate(Vector2d(bestXPos*step_,bestHeight));
            for (int i=bestXPos;i<bestXPos + width_in_pas;i++){
                height(i) =  rect.min().y() + rect.max_func(i-bestXPos);
            }
        }
        
        // find the max heigth in the range  [xpos,  xpos + width]
        double max_height(int xpos, int width,MapComponentBBox& rect){
            double result = 0 ;
            for (int i=xpos; i<xpos+width; i++) {
                result = ogf_max( result, height(i)-rect.min_func(i-xpos) );
            }
            return result;
        }
        
        // find the free area in the range under height_max in range
        // [xpos,  xpos + width]

        double free_area(int xpos, int width, double height_max) {
            double result =0;
            for (int i=xpos ; i<xpos+width ; i++) {
                result += step_ * (height_max - height(i));
            }
            return result;
        }

    private:

        int image_size_in_pixels_ ;
        int margin_width_in_pixels_ ;

        int nb_xpos_;
        double width_;
        double step_;
        double *height_;

        std::vector<MapComponentBBox> data_;
    } ;


//_______________________________________________________________________


    Packer::Packer() {
        image_size_in_pixels_ = 1024 ;
        margin_width_in_pixels_ = 4 ;
    }
  

    // There where some problems with nan (Not a number),
    // this code is used to track such problems (seems to
    // be ok now)
    static bool map_component_is_ok(MapComponent* comp) {
        { FOR_EACH_VERTEX(MapComponent, comp, it) {
            const Point2d& p = it->halfedge()->tex_coord() ;
            if(Numeric::is_nan(p.x())) {
                return false ;
            } 
            if(Numeric::is_nan(p.y())) {
                return false ;
            } 
        }}
        return true ;
    }

    void Packer::pack_map(Map* surface) {

        MapComponentsExtractor splitter ;
        MapComponentList components = splitter.extract_components(
            surface
        ) ;

        // Sanity check 2
        for(
            MapComponentList::iterator it = components.begin(); 
            it != components.end(); it++
        ) {
            if(!map_component_is_ok(*it)) {
                FOR_EACH_HALFEDGE(MapComponent, *it, jt) {
                    jt->set_tex_coord(Point2d(0.0, 0.0)) ;
                }
            }
        }

        pack_map_components(components) ;

        MapNormalizer normalizer(surface) ;
        normalizer.normalize_tex_coords() ;
    }
    
    void Packer::pack_map_components(MapComponentList& surfaces) {
        
        Logger::out("Packer") 
            << "nb components:" << surfaces.size() << std::endl ;  
  
        Map* map = surfaces[0]->map() ;
        total_area_3d_ = Geom::map_area(map) ;
        is_visited_.bind(map) ;
        normalize_surface_components(surfaces) ;
        is_visited_.unbind() ;

        // use the tetris packer (more efficient for large dataset)
        // set some application dependant const
        TetrisPacker pack ;  
        pack.set_image_size_in_pixels(image_size_in_pixels()) ;
        pack.set_margin_width_in_pixels(margin_width_in_pixels()) ;
        double area = 0;
        for(
            MapComponentList::iterator it = surfaces.begin() ; 
            it != surfaces.end() ; it++
        ) {
            ogf_assert(map_component_is_ok(*it)) ;
            Box2d box = Geom::component_bbox2d(*it) ;
            double u_min = box.x_min() ;
            double v_min = box.y_min() ;
            double u_max = box.x_max() ;
            double v_max = box.y_max() ;
            
            ogf_assert(!Numeric::is_nan(u_min)) ;
            ogf_assert(!Numeric::is_nan(v_min)) ;
            ogf_assert(!Numeric::is_nan(u_max)) ;
            ogf_assert(!Numeric::is_nan(v_max)) ;
            ogf_assert(u_max >= u_min);
            ogf_assert(v_max >= v_min);
            
            area += (v_max - v_min) * (u_max - u_min);
            
            MapComponentBBox r(
                *it,
                Point2d(u_min,v_min) ,
                Point2d(u_max,v_max)
            );
            pack.add(r);
        }
        
        
        pack.recursive_apply();


        //Aritk tests !!!!
        double total_area = 0;
        {for(
            MapComponentList::iterator it = surfaces.begin() ;
            it != surfaces.end(); it++
	) {
            total_area += Geom::component_area2d(*it) ;
        }}

        ////////////////////////////////////////////////////////
        {
            Box2d box ;
            
            for(
                MapComponentList::iterator it = surfaces.begin() ;
                it != surfaces.end(); it++
            ) {
                box.add_box(Geom::component_bbox2d(*it)) ;
            }
            
            double bbox_area = box.width() * box.height() ; 
            double filling_ratio = total_area / bbox_area ;

            Logger::out("Packer") << "BBox area:"  << bbox_area << std::endl ;
            Logger::out("Packer") << "Filling ratio:" 
                                  << filling_ratio << std::endl ;
        }
        ////////////////////////////////////////////////////////
    
    }
    
    Map::Halfedge* Packer::largest_border(MapComponent* component) {
        {FOR_EACH_HALFEDGE(MapComponent, component, it) {
            is_visited_[it] = false ;
        }}
        int largest_size = 0 ;
        Map::Halfedge* largest = nil ;
        FOR_EACH_HALFEDGE(MapComponent, component, it) {
            if(it->is_border() && !is_visited_[it]) {
                int cur_size = 0 ;
                Map::Halfedge* h = it ;
                do {
                    is_visited_[h] = true ;
                    h = h->next() ;
                    cur_size++ ;
                } while(h != it) ;
                if(cur_size > largest_size) {
                    largest_size = cur_size ;
                    largest = it ;
                }
            }
        }        
        return largest ;
    }

    void Packer::normalize_surface_components(
        MapComponentList& surfaces
    ) {
        if(surfaces.size() == 0) { return ; }
        seam_type_.bind_if_defined(surfaces[0]->map(), "seam") ;
        for(
            MapComponentList::iterator it = surfaces.begin() ;
            it != surfaces.end(); it++
        ) {
            ogf_assert(map_component_is_ok(*it)) ;
            normalize_surface_component(*it) ;
            ogf_assert(map_component_is_ok(*it)) ;
        }
        seam_type_.unbind() ;
    }
    
    static double extent(const Polygon2d& P, const Vector2d& v) {
        double x_min =  1e30 ;
        double x_max = -1e30 ;
        for(unsigned int i=0; i<P.size(); i++) {
            double x = (P[i] - Origin()) * v ;
            x_min = ogf_min(x_min, x) ;
            x_max = ogf_max(x_max, x) ;
        }
        return (x_max - x_min) ;
    }


    static Vector2d longuest_edge(const Polygon2d& P) {
        Vector2d result(0,0) ;
        for(unsigned int i=0; i<P.size(); i++) {
            int j = (i+1)%P.size() ;
            Vector2d cur = P[j] - P[i] ;
            if(cur.norm2() > result.norm2()) {
                result = cur ;
            }
        }
        return result ;
    }


    static void rectangle(const Polygon2d& P, Vector2d& v1, Vector2d& v2) {
        Vector2d up = longuest_edge(P) ; 
       //FIX SIG FPE later in cos_angle
        if(up.norm() ==0)
            return;
        up.normalize() ;
//        up = Vector2d(0,1) ;
        AverageDirection2d dir ;
        dir.begin() ;
        for(unsigned int i=0; i<P.size(); i++) {
            unsigned int j = (i + 1) % P.size() ;
            unsigned int k = (i + 2) % P.size() ;
            unsigned int l = (i + 3) % P.size() ;

            Vector2d v = P[k] - P[j] ;
            Vector2d v_pred = P[j] - P[i] ;
            Vector2d v_succ = P[l] - P[k] ;

            double cos_pred = Geom::cos_angle(v_pred, v) ;
            double cos_succ = Geom::cos_angle(v, v_succ) ;

            double min_cos = ogf_min(::fabs(cos_pred), ::fabs(cos_succ)) ;
            v = (1.0 - min_cos) * v ;


            double cos_alpha = (v * up) / v.norm() ;
            double test = ::fabs(cos_alpha) - 0.5 * sqrt(2.0) ;
            if(test < 0.0) {
                v = Vector2d(-v.y(), v.x()) ;
            }
            if(::fabs(test) < 0.005) {
                v = Vector2d(0,0) ;
                double n = v.norm() ;
                v = up ;
                v.normalize() ;
                v = 0.5 * (1.0 - min_cos) * n * v ;
            }
            dir.add_vector(v) ;
        }
        dir.end() ;
        v1 = dir.average_direction() ;
        v1.normalize() ;
        v2 = Vector2d(-v1.y(), v1.x()) ;
        if(extent(P, v1) > extent(P,v2)) {
            ogf_swap(v1,v2) ;
        }
    }


    // Copied from parameterizer/algos/autoseam.h (we got a cross-dep here, 
    // that's bad)
    enum {
        AutoseamSeamBit = 1,
        AutoseamUBit = 2,
        AutoseamVBit = 4,
        AutoseamStraightBit = 8
    } ;


    void Packer::normalize_surface_component(MapComponent* S) {
        
        bool has_straight_seams = false ;
        if(seam_type_.is_bound()) {
            FOR_EACH_HALFEDGE(MapComponent, S, it) {
                if(seam_type_[it] & AutoseamStraightBit) {
                    has_straight_seams = true ;
                    break ;
                }
            }
        }


        Polygon2d P ;

        {
            Map::Halfedge* b = largest_border(S) ;
            Map::Halfedge* h = b ;
            do {
                P.push_back(h->tex_coord()) ;
                h = h->next() ;
            } while(h != b) ;
        }

        Vector2d v1 ;
        Vector2d v2 ;
        Point2d center = P[0] ;

        if(has_straight_seams) {
            v1 = Vector2d(1.0,0.0) ;
            v2 = Vector2d(0.0,1.0) ;
        } else {
//        Geom::minimum_area_enclosing_rectangle(P, v1, v2) ;
        rectangle(P, v1, v2) ;
        }
        ogf_assert(!Numeric::is_nan(center.x())) ;
        ogf_assert(!Numeric::is_nan(center.y())) ;
        ogf_assert(!Numeric::is_nan(v1.x())) ;
        ogf_assert(!Numeric::is_nan(v1.y())) ;
        ogf_assert(!Numeric::is_nan(v2.x())) ;
        ogf_assert(!Numeric::is_nan(v2.y())) ;

        v1.normalize() ;
        v2.normalize() ;
        
        { FOR_EACH_VERTEX(MapComponent, S, it) {
            const Point2d& old_uv = it->halfedge()->tex_coord() ;
            double new_u = (old_uv - center) * v1 ;
            double new_v = (old_uv - center) * v2 ;
            it->halfedge()->set_tex_coord(Point2d(new_u, new_v)) ;
        }}

        double area3d = Geom::component_area(S) ;
        double area2d = Geom::component_area2d(S) ;
        double factor = 1.0 ;
        if(::fabs(area2d) > 1e-30) {
            factor = ::sqrt(area3d/area2d) ;
        } else {
            factor = 0.0 ;
        }

        ogf_assert(!Numeric::is_nan(area2d)) ;
        ogf_assert(!Numeric::is_nan(area3d)) ;
        ogf_assert(!Numeric::is_nan(factor)) ;

        { FOR_EACH_VERTEX(MapComponent, S, it) {
            const Point2d& old_uv = it->halfedge()->tex_coord() ;
            double new_u = old_uv.x() * factor ;
            double new_v = old_uv.y() * factor ;
            it->halfedge()->set_tex_coord(Point2d(new_u, new_v)) ;
        }}        
    }


}

