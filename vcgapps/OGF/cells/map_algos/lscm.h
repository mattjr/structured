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

#ifndef __LSCM__
#define __LSCM__

#include <OGF/cells/map_algos/map_parameterizer.h>

namespace OGF {

    class LinearSolver ;

    class CELLS_API MapParameterizerLSCM : public MapParameterizer {
    public:
        MapParameterizerLSCM() ;
        virtual bool do_parameterize_disc(Map* map) ;

        void set_user_locks(bool x) { user_locks_ = x ; }
        /**
         * 0 (default) means SuperLU, else it
         * uses conjugate gradient with nb_iter
         * iterations.
         */
        void set_nb_iter(int x) { nb_iter_ = x ; }

    protected:


        /**
         * Decomposes the surface into a set of virtual triangles,
         * and install relations on each of them.
         */
        virtual void setup_conformal_map_relations(LinearSolver& solver ) ;


        /**
         * Decomposes the facet into a set of virtual triangles,
         * and install relations on each of them.
         */
        virtual void setup_conformal_map_relations(
            LinearSolver& solver, Map::Facet* f
        ) ;

        
        /**
         * Installs relations ensuring angles preservation in the
         * specified triangle.
         */
        virtual void setup_conformal_map_relations(
            LinearSolver& solver, 
            Map::Halfedge* h0,
            Map::Halfedge* h1,
            Map::Halfedge* h2
        ) ;

    private:
        bool user_locks_ ;
        int nb_iter_ ;
    } ;

}


#endif
