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

#include <OGF/cells/map_algos/stretch_map_parameterizer.h>
#include <OGF/cells/map_algos/floater.h>
#include <OGF/cells/types/cells_library.h>

namespace OGF {
 
    StretchMapParameterizer::StretchMapParameterizer() {
        strict_kernels_ = true ;
        lock_borders_ = true ;
    }

    void StretchMapParameterizer::compute_derivatives(
        const Point2d& p1, const Point2d& p2, const Point2d& p3,
        const Point3d& q1, const Point3d& q2, const Point3d& q3,
        double A, Vector3d& Ss, Vector3d& St
    ) {
        double w = 1.0 / (2.0 * A) ;
        
        Ss = w * (
            (p2.y() - p3.y()) * (q1 - Origin()) +  
            (p3.y() - p1.y()) * (q2 - Origin()) +
            (p1.y() - p2.y()) * (q3 - Origin())
        ) ;

        St = w * (
            (p3.x() - p2.x()) * (q1 - Origin()) +
            (p1.x() - p3.x()) * (q2 - Origin()) +
            (p2.x() - p1.x()) * (q3 - Origin()) 
        ) ;
    }

        
    double StretchMapParameterizer::facet_criterion(Map::Facet* f) {
        Map::Halfedge* h1 = f->halfedge() ;
        Map::Halfedge* h2 = h1->next() ;
        Map::Halfedge* h3 = h2->next() ;

        const Point2d& p1 = h1->tex_coord() ;
        const Point2d& p2 = h2->tex_coord() ;
        const Point2d& p3 = h3->tex_coord() ;

        const Point3d& q1 = h1->vertex()->point() ;
        const Point3d& q2 = h2->vertex()->point() ;
        const Point3d& q3 = h3->vertex()->point() ;
        double A = Geom::triangle_area(p1,p2,p3) ;
        Vector3d Ss, St ;
        compute_derivatives(p1,p2,p3,q1,q2,q3,A,Ss,St) ;
        double a = Ss * Ss ;
        double c = St * St ;
        return facet_area_[f] * ::sqrt(0.5*(a+c)) ;
    }

    void StretchMapParameterizer::compute_initial_parameterization() {
        MapParameterizer_var param = CellsLibrary::instance()->create_map_parameterizer(
            "MeanValueCoordinates"
        ) ;
        param->use_locked_vertices_as_corners(use_locked_vertices_as_corners_) ;
        do_parameterize_disc_using_parameterizer(param, map_) ;
    }

}
