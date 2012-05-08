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

#ifndef __OGF_CELLS_MAP_ALGOS_MAP_APPROX__
#define __OGF_CELLS_MAP_ALGOS_MAP_APPROX__


#include <OGF/cells/common/common.h>
#include <OGF/cells/map/map.h>
#include <OGF/cells/map/map_attributes.h>
#include <OGF/cells/map/geometry.h>
#include <OGF/cells/map_algos/map_components.h>
#include <OGF/math/linear_algebra/matrix_util.h>
#include <OGF/math/geometry/normal_cycle.h>
#include <OGF/math/functions/triangle_integral.h>
#include <OGF/basic/debug/progress.h>

#include <stack>
#include <queue>

namespace OGF {


    // ------- Default proxy and fitter classes (uses the norm L1,2) ------

    class L12LinearProxy {
    public:
        static void set_compactness_importance(double x) { compactness_importance_ = x ; }
        static double compactness_importance() { return compactness_importance_ ; }
        static void set_angular_threshold(double x) { 
            angular_threshold_ = x ; n_mode_ = (angular_threshold_ < 1.0) ; 
        }
        static double angular_threshold() { return angular_threshold_ ; }
        static bool n_mode() { return n_mode_ ; }

        L12LinearProxy() : N_(0,0,0), G_(0,0,0) { }
        L12LinearProxy(const Vector3d& v, const Point3d& p) : N_(v), G_(p) { }
        L12LinearProxy(Map::Facet* f) : N_(Geom::facet_normal(f)), G_(Geom::facet_barycenter(f)) { }

        double distance_3(Map::Facet* f, double area) const {

            Map::Vertex* v[3] ;
            v[0]=f->halfedge()->vertex() ;
            v[1]=f->halfedge()->next()->vertex() ;
            v[2]=f->halfedge()->next()->next()->vertex() ;
            double result = 0 ;
            double d[3] ;
            for(int i=0; i<3; i++) {
                Vector3d V = v[i]->point() - G_ ;
                V = V - (V * N_) * N_ ;
                d[i] = V.norm2() ;
                result += d[i] ;
                d[i] = ::sqrt(d[i]) ;
            }
            result += d[0] * d[1] ;
            result += d[1] * d[2] ;
            result += d[2] * d[0] ;
            result /= 6.0 ;
            result *= area ;
            return result ;
        }

        double deviation(Map::Facet* f) const {
            double a = Geom::facet_area(f) ;
            Vector3d n = Geom::facet_normal(f) ;
            double result =  a * (n - N_).norm2() ; 
            if(compactness_importance_ != 0.0) {
                result += compactness_importance_ * distance_3(f,a) ;
            }
            return result ;
        }

        double angle_deviation(Map::Facet* f) const {
            Vector3d n = Geom::facet_normal(f) ;
            return (n ^ N_).norm() ;
        }

    private:
        Vector3d N_ ;
        Point3d G_ ;
        static CELLS_API double compactness_importance_ ;
        static CELLS_API double angular_threshold_ ;
        static CELLS_API bool n_mode_ ;
    } ;

    class L12LinearProxyFitter {
    public:
        void begin() { N_ = Vector3d(0,0,0) ; G_ = Point3d(0,0,0) ; A_ = 0.0 ; }
        void end() { 
            N_.normalize() ; 
            if(L12LinearProxy::compactness_importance() != 0.0 && A_ != 0.0) {
                G_.set_x(G_.x() / A_) ;
                G_.set_y(G_.y() / A_) ;
                G_.set_z(G_.z() / A_) ;
            }
        }
        L12LinearProxy proxy() { return L12LinearProxy(N_, G_) ; }
        void add_facet(Map::Facet* f) {
            double a = Geom::facet_area(f) ;
            N_ = N_ + a * Geom::facet_normal(f) ;
            if(L12LinearProxy::compactness_importance() != 0.0) {
                Point3d g = Geom::facet_barycenter(f) ;
                A_ += a ;
                G_.set_x(G_.x() + a*g.x()) ;
                G_.set_y(G_.y() + a*g.y()) ;
                G_.set_z(G_.z() + a*g.z()) ;
            }
        }

    private:
        Vector3d N_ ;
        Point3d G_ ;
        double A_ ;
    } ;

    //____________________________________________________________________________________

    struct AddFacetToChart {
        AddFacetToChart(
            Map::Facet* f, int c, double e
        ) : facet(f), chart(c), E(e) { 
        }
        Map::Facet* facet ;
        int chart ;
        double E ;
    } ;

    class AddFacetToChartCmp {
    public:
        bool operator()(
            const AddFacetToChart& op1, const AddFacetToChart& op2
        ) {
            return (op1.E > op2.E) ;
        }
    } ;


    typedef std::priority_queue< 
        AddFacetToChart,
        std::vector<AddFacetToChart>,
        AddFacetToChartCmp
    > AddFacetToChartQueue ;
    
    class ChartAttribute : public MapFacetAttribute<int> {
    public:
        typedef MapFacetAttribute<int> superclass ;
        ChartAttribute(Map* map) : superclass(map, "chart") { }
        ChartAttribute(MapComponent* component) : superclass(component->map(), "chart") { }
    } ;


    // This one is there to allow instanciation with MapComponent.
    inline void update_graphics(MapComponent* comp) { /* does nothing */ }

    inline double generic_area(Map* map) { return Geom::map_area(map) ; }
    inline double generic_area(MapComponent* comp) { return Geom::component_area(comp) ; }

    /**
     * A generic implementation of the segmentation algorithm in:
     * Cohen-Steiner, Alliez and Desbrun,
     * Variational Shape Approximation, Siggraph 2004
     */

    template <class Proxy, class ProxyFitter, class MAP = Map> class MapVariationalApprox {
    public:
        MapVariationalApprox(MAP* map) : map_(map), chart_(map), max_error_(-1) {
        }


        void init(
            int nb_proxies, int nb_iter, double min_err = 0.0, int min_nb_proxies = 0
        ) {
            ProgressLogger progress(nb_proxies) ;
            init_one_proxy_per_component() ;
            for(int i=proxy_.size(); i<=nb_proxies; i++) {
                double err = optimize(nb_iter) ;
                if(err < min_err && int(proxy_.size()) >= min_nb_proxies) {
                    return ;
                }
                if(int(proxy_.size()) >= int(map_->size_of_facets())) { return ; }
                Map::Facet* f = new_chart() ;
                chart_[f] = proxy_.size() ;
                if(progress.is_canceled()) {
                    break ;
                }
                progress.notify(i) ;
            }
            if(L12LinearProxy::n_mode()) {
                fill_holes() ;
            }
        }

        double optimize(int nb_iter) {
            double result = 0 ;
            ProgressLogger progress(nb_iter) ;
            nb_iter = ogf_max(1, nb_iter) ;
            for(int i = 0; i<nb_iter; i++) {
                bool get_uninit = (i == nb_iter / 2) ;
                get_proxies(get_uninit) ;
                get_seeds() ;
                flood_fill() ;
                update_graphics(map_) ;
                if(progress.is_canceled()) {
                    break ;
                }
                result = error() ;
                Logger::out("Partition") << "error = " << result << std::endl ;
                progress.notify(i) ;
            }
            return result ;
        }


        void add_charts(
            int nb_charts, int nb_iter, double min_err = 0.0, int min_nb_charts = 0
        ) {
            ProgressLogger progress(nb_charts) ;
            for(int i=0; i<nb_charts; i++) {
                get_proxies() ;
                if(int(proxy_.size()) == int(map_->size_of_facets())) {
                    Logger::out("Partition") << "All facets are separated" << std::endl ;
                    break ;
                }
                Map::Facet* f = new_chart() ;
                chart_[f] = proxy_.size() ;
                double err = optimize(nb_iter) ;
                if(err < min_err && int(proxy_.size()) >= min_nb_charts) {
                    return ;
                }
                if(progress.is_canceled()) {
                    break ;
                }
                Logger::out("Partition") << "error = " << err << std::endl ;
                progress.notify(i) ;
            }
            if(L12LinearProxy::n_mode()) {
                fill_holes() ;
            }
        }

        double error() {
            double result = 0.0 ;
            FOR_EACH_FACET_GENERIC(MAP, map_, it) {
                int chart = chart_[it] ;
                if(chart != -1) {
                    result += proxy_[chart].deviation(it) ;
                }
            }
            result /= generic_area(map_) ;
            return result ;
        }

    protected:

        Map::Facet* new_chart() {
            Map::Facet* result = nil ;
            result = largest_uninitialized_chart() ;
            if(result == nil) {
                result = worst_facet_in_worst_chart() ;
            }
            return result ;
        }

        void compute_chart_sizes() {
            chart_size_.clear() ;
            int nb_proxies = 0 ;
            FOR_EACH_FACET_GENERIC(MAP, map_, it) {
                nb_proxies = ogf_max(chart_[it], nb_proxies) ;
            }
            nb_proxies++ ;
            for(int i=0; i<nb_proxies; i++) {
                chart_size_.push_back(0) ;
            }
            FOR_EACH_FACET_GENERIC(MAP, map_, it) {
                int chart = chart_[it] ;
                if(chart >= 0) {
                    chart_size_[chart]++ ;
                }
            }
        }

        void check_facets() {
            FOR_EACH_FACET_GENERIC(MAP, map_, it) {
                ogf_assert(it != nil) ;
            }
        }

        Map::Facet* worst_facet() {
            compute_chart_sizes() ;
            double e = -1.0 ;
            Map::Facet* result = nil ;
            FOR_EACH_FACET_GENERIC(MAP, map_, it) {
                int chart = chart_[it] ;
                if(chart >= 0 && chart_size_[chart] > 1) {
                    ogf_assert(chart < int(proxy_.size())) ;
                    double cur_e = proxy_[chart].deviation(it) ;
                    if(cur_e > e) {
                        e = cur_e ;
                        result = it ;
                    }
                }
            }
            ogf_assert(result != nil) ;
            return result ;
        }

        int worst_chart() {
            compute_chart_sizes() ;
            std::vector<double> E ;
            for(unsigned int i=0; i<proxy_.size(); i++) {
                E.push_back(0) ;
            }
            FOR_EACH_FACET_GENERIC(MAP, map_, it) {
                int chart = chart_[it] ;
                if(!L12LinearProxy::n_mode()) {
                    ogf_range_assert(chart,0,int(proxy_.size()) - 1) ;
                }
                if(chart >= 0) {
                    E[chart] += proxy_[chart].deviation(it) ;
                }
            }
            double worst_e = -1.0 ;
            int result = -1 ;
            for(unsigned int i=0; i<proxy_.size(); i++) {
                if(E[i] > worst_e && chart_size_[i] > 1) {
                    worst_e = E[i] ;
                    result = i ;
                }
            }
            ogf_assert(result != -1) ;
            return result ;
        }
        
        Map::Facet* worst_facet_in_worst_chart() {
            int proxy = worst_chart() ;
            double e = -1.0 ;
            Map::Facet* result = nil ;
            FOR_EACH_FACET_GENERIC(MAP, map_, it) {
                int chart = chart_[it] ;
                if(chart == proxy) {
                    double cur_e = proxy_[chart].deviation(it) ;
                    if(cur_e > e) {
                        e = cur_e ;
                        result = it ;
                    }
                }
            }
            ogf_assert(result != nil) ;
            return result ;
        }

        void init_one_proxy() {
            Map::Facet* f = map_->facets_begin() ;
            proxy_.clear() ;
            seed_.clear() ;
            proxy_.push_back(Proxy(f)) ;
            seed_.push_back(f) ;
            FOR_EACH_FACET_GENERIC(MAP, map_, it) {
                chart_[it] = 0 ;
            }
        }

        void init_one_proxy_per_component() {
            FOR_EACH_FACET_GENERIC(MAP, map_, it) {
                chart_[it] = -1 ;
            }
            proxy_.clear() ;
            seed_.clear() ;
            FOR_EACH_FACET_GENERIC(MAP, map_, it) {
                if(chart_[it] == -1) {
                    proxy_.push_back(Proxy(it)) ;
                    seed_.push_back(it) ;
                    init_one_proxy_from_facet(it, proxy_.size() - 1) ;
                }
            }
        }

        /** returns the number of facets in the chart */
        int init_one_proxy_from_facet(Map::Facet* f, int chart_id, int background_id = -1) {
            int result = 0 ;
            std::stack<Map::Facet*> S ;
            S.push(f) ;
            while(!S.empty()) {
                Map::Facet* cur = S.top() ;
                S.pop() ;
                if(chart_[cur] != chart_id) {
                    chart_[cur] = chart_id ;
                    result++ ;
                }
                Map::Halfedge* h = cur->halfedge() ;
                do {
                    Map::Facet* neigh = h->opposite()->facet() ;
                    if(neigh != nil && chart_[neigh] == background_id) {
                        S.push(neigh) ;
                    }
                    h = h->next() ;
                } while(h != cur->halfedge()) ;
            }
            return result ;
        }

        int compute_nb_proxies() {
            int nb_proxies = 0 ;
            FOR_EACH_FACET_GENERIC(MAP, map_, it) {
                nb_proxies = ogf_max(nb_proxies, chart_[it]) ;
            }
            nb_proxies++ ;
            return nb_proxies ;
        }

        /** returns the total numer of charts */
        int get_uninitialized_charts() {
            int nb_proxies = compute_nb_proxies() ;
            FOR_EACH_FACET_GENERIC(MAP, map_, it) {
                if(chart_[it] < 0) {
                    init_one_proxy_from_facet(it, nb_proxies) ;
                    nb_proxies++ ;
                }
            }
            return nb_proxies ;
        }

        void add_uninitialized_charts(int min_size) {
            std::vector<int> remap ;
            remap.push_back(0) ;
            remap.push_back(0) ;
            int cur_remap = compute_nb_proxies() ;
            int cur_id = -2 ;
            FOR_EACH_FACET_GENERIC(MAP, map_, it) {
                if(chart_[it] == -1) {
                    int cur_size = init_one_proxy_from_facet(it, cur_id, chart_[it]) ;
                    cur_id-- ;
                    if(cur_size > min_size) {
                        remap.push_back(cur_remap) ; cur_remap++ ;
                    } else {
                        remap.push_back(-1) ;
                    }
                }
            }
            FOR_EACH_FACET_GENERIC(MAP, map_, it) {
                if(chart_[it] < 0) {
                    chart_[it] = remap[-chart_[it]] ;
                }
            }
        }

        Map::Facet* largest_uninitialized_chart() {
            int cur_id = -2 ;
            int max_size = -1 ;
            Map::Facet* result = nil ;
            FOR_EACH_FACET_GENERIC(MAP, map_, it) {
                if(chart_[it] == -1) {
                    int cur_size = init_one_proxy_from_facet(it, cur_id, chart_[it]) ;
                    cur_id-- ;
                    if(cur_size > max_size) {
                        max_size = cur_size ;
                        result = it ;
                    }
                }
            }
            FOR_EACH_FACET_GENERIC(MAP, map_, it) {
                if(chart_[it] < 0) {
                    chart_[it] = -1 ;
                }
            }
            return (max_size > 100) ? result : nil ;
        }

        void get_proxies(bool get_uninit = false) {
            std::vector<ProxyFitter> fitter ;
            if(get_uninit) {
                add_uninitialized_charts(100) ;
            }
            int nb_proxies = compute_nb_proxies() ;
            proxy_.clear() ;
            for(int i=0; i<nb_proxies; i++) {
                proxy_.push_back(Proxy()) ;
                fitter.push_back(ProxyFitter()) ;
                fitter[i].begin() ;
            }
            FOR_EACH_FACET_GENERIC(MAP, map_, it) {
                int chart = chart_[it] ;
                if(chart >= 0) {
                    ogf_assert(chart < int(proxy_.size())) ;
                    fitter[chart].add_facet(it) ;
                } else {
//                    std::cerr << "get_proxies(): NUL proxy" << std::endl ;
                }
            }
            for(int i=0; i<nb_proxies; i++) {
                ogf_assert(i < int(proxy_.size())) ;
                fitter[i].end() ;
                proxy_[i] = fitter[i].proxy() ;
            }
        }
        
        void get_seeds() {
            seed_.clear() ;
            std::vector<double> E ;
            for(unsigned int i=0; i<proxy_.size(); i++) {
                E.push_back(Numeric::big_double) ;
                seed_.push_back(nil) ;
            }
            FOR_EACH_FACET_GENERIC(MAP, map_, it) {
                int chart = chart_[it] ;
                if(chart >= 0) {
                    double d = proxy_[chart].deviation(it) ;
                    if(d < E[chart]) {
                        E[chart] = d ;
                        seed_[chart] = it ;
                    }
                } else {
//                    std::cerr << "get_seeds(): NUL proxy" << std::endl ;
                }
            }
            for(unsigned int i=0; i<seed_.size(); i++) {
                if(seed_[i] == nil) {
//                    std::cerr << "nil seed for chart id" << i << std::endl ;
                }
            }
        }

        void insert_neighbors(Map::Facet* seed, AddFacetToChartQueue& q) {
/*
            // This one is the simple version, as in the paper,
            // the other version seems to me more efficient.
            int chart = chart_[seed] ;
            Map::Halfedge* h = seed->halfedge() ;
            do {
                Map::Facet* f = h->opposite()->facet() ;
                if(f != nil && chart_[f] == -1) {
                    q.push(AddFacetToChart(f, chart, proxy_[chart].deviation(f))) ;
                }
                h = h->next() ;
            } while(h != seed->halfedge()) ;
*/

            Map::Halfedge* h = seed->halfedge() ;
            do {
                Map::Facet* f = h->opposite()->facet() ;
                if(f != nil && chart_[f] == -1) {
                    Map::Halfedge* hh = f->halfedge() ;
                    do {
                        Map::Facet* ff = hh->opposite()->facet() ;
                        if(ff != nil) {
                            int chart = chart_[ff] ;
                            if(chart != -1) {
                                ogf_range_assert(chart, 0, int(proxy_.size()) - 1) ;
                                q.push(AddFacetToChart(f, chart, proxy_[chart].deviation(f))) ;
                            }
                        }
                        hh = hh->next() ;
                    } while(hh != f->halfedge()) ;
                }
                h = h->next() ;
            } while(h != seed->halfedge()) ;
        }


        void flood_fill() {
            FOR_EACH_FACET_GENERIC(MAP, map_, it) {
                chart_[it] = -1 ;
            }

            for(unsigned int i=0; i<proxy_.size(); i++) {
                ogf_assert(i < seed_.size()) ;
                if(seed_[i] != nil) {
                    chart_[seed_[i]] = i ;
                }
            }
            
            AddFacetToChartQueue q ;
            for(unsigned int i=0; i<proxy_.size(); i++) {
                ogf_assert(i < seed_.size()) ;
                if(seed_[i] != nil) {
                    insert_neighbors(seed_[i], q) ;
                }
            }

            while (!q.empty()) {
                AddFacetToChart op = q.top();
                q.pop();
                if(chart_[op.facet] == -1) {
                    ogf_assert(op.chart < int(proxy_.size())) ;
                    if(
                        !L12LinearProxy::n_mode() || 
                        (proxy_[op.chart].angle_deviation(op.facet) < L12LinearProxy::angular_threshold())
                    ) {
                        chart_[op.facet] = op.chart ;
                        insert_neighbors(op.facet, q) ;
                    }
                }
            }
        }

        void fill_holes() {
            std::stack<Map::Halfedge*> S ;
            FOR_EACH_HALFEDGE_GENERIC(MAP, map_, it) {
                Map::Facet* f1 = it->facet() ;
                Map::Facet* f2 = it->opposite()->facet() ;
                if(f1 != nil && f2 != nil && chart_[f1] >= 0 && chart_[f2] < 0) {
                    S.push(it) ;
                }
            }
            while(!S.empty()) {
                Map::Halfedge* h = S.top() ;
                S.pop() ;
                Map::Facet* f1 = h->facet() ;
                Map::Facet* f2 = h->opposite()->facet() ;
                if(chart_[f2] < 0) {
                    chart_[f2] = chart_[f1] ;
                    Map::Halfedge* hh = f2->halfedge() ;
                    do {
                        Map::Facet* f3 = hh->opposite()->facet() ;
                        if(f3 != nil && chart_[f3] < 0) {
                            S.push(hh) ;
                        }
                        hh = hh->next() ;
                    } while(hh != f2->halfedge()) ;
                }
            }
        }


    private:
        MAP* map_ ;
        ChartAttribute chart_ ;
        std::vector<Proxy> proxy_ ;
        std::vector<Map::Facet*> seed_ ;
        std::vector<int> chart_size_ ;

        double max_error_ ;
    } ;

}

#endif
