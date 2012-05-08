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
 

#include <OGF/cells/cgraph/geometry.h>

namespace OGF {

    namespace Geom {

        Vector3d facet_normal(CGraph::Cell* c, unsigned int f, bool normalize) {
            Vector3d result ;
            const CGraph::MetaCell* mc = c->meta_cell() ;
            unsigned int n = mc->nb_vertices_in_facet(f) ;
            for(unsigned int i=0; i<n; i++) {
                unsigned int j = (i + 1) % n ;
                unsigned int k = (j + 1) % n ;
                const Point3d& pi = c->vertex(mc->facet_vertex(f, i))->point() ;
                const Point3d& pj = c->vertex(mc->facet_vertex(f, j))->point() ;
                const Point3d& pk = c->vertex(mc->facet_vertex(f, k))->point() ;
                result = result + ((pi - pj) ^ (pk - pj)) ;
            }
            if(normalize) {
                result.normalize() ;
            }
            return result ;
        }


        double tetra_volume(CGraph::Cell* c) {
            ogf_assert(c->nb_vertices() == 4) ;
            ogf_assert(c->nb_facets() == 4) ;
            return tetra_volume(
                c->vertex(0)->point(), 
                c->vertex(1)->point(), 
                c->vertex(2)->point(), 
                c->vertex(3)->point()
            ) ;
        }


        double cell_volume(CGraph::Cell* c) {
            double result = 0.0 ;
            const Point3d& p0 = c->vertex(0)->point() ;
            for(unsigned int f=0; f<c->nb_facets(); f++) {
                const Point3d& p1 = c->facet_vertex(f,0)->point() ;
                for(unsigned int i=1; i+1<c->nb_vertices_in_facet(f); i++) {
                    const Point3d& p2 = c->facet_vertex(f,i)->point() ;
                    const Point3d& p3 = c->facet_vertex(f,i+1)->point() ;
                    result += tetra_signed_volume(p0, p1, p2, p3) ;
                }
            }
            return ::fabs(result) ;
        }

		inline double min_dihedral_angle_aux_compute_quotient(
			const Point3d& p0, const Point3d& p1,
			const Point3d& p2, const Point3d& p3) {
				return (p1-p0).norm()
					/ triangle_area(p0, p1, p3)
					/ triangle_area(p0, p1, p2) ;
		}

		double tetra_min_dihedral_angle(CGraph::Cell* c) {
			ogf_assert(c->nb_vertices() == 4) ;
			ogf_assert(c->nb_facets() == 4) ;
			
			Point3d p0 = c->vertex(0)->point() ;
			Point3d p1 = c->vertex(1)->point() ;
			Point3d p2 = c->vertex(2)->point() ;
			Point3d p3 = c->vertex(3)->point() ;

			double min_quotient = 
				min_dihedral_angle_aux_compute_quotient(p0, p1, p2, p3);

			min_quotient = 
				(std::min)(min_quotient,
				min_dihedral_angle_aux_compute_quotient(p0, p2, p1, p3));
			min_quotient = 
				(std::min)(min_quotient,
				min_dihedral_angle_aux_compute_quotient(p0, p3, p1, p2));
			min_quotient = 
				(std::min)(min_quotient,
				min_dihedral_angle_aux_compute_quotient(p1, p2, p0, p3));
			min_quotient = 
				(std::min)(min_quotient,
				min_dihedral_angle_aux_compute_quotient(p1, p3, p0, p2));
			min_quotient = 
				(std::min)(min_quotient,
				min_dihedral_angle_aux_compute_quotient(p2, p3, p0, p1));

			const double result (std::asin( double(1.5) * tetra_volume(c) * min_quotient )
				* double(180) / M_PI);

			return fabs(result);
		}

    }

}

