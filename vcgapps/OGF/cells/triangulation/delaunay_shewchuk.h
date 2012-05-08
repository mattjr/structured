
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
 

#ifndef __OGF_CELLS_ALGOS_DELAUNAY_SHEWCHUK__
#define __OGF_CELLS_ALGOS_DELAUNAY_SHEWCHUK__

#include <OGF/cells/common/common.h>
#include <OGF/cells/triangulation/delaunay.h>


namespace OGF {

// TODO: replace with implementation of DelaunayAPI2d

    /**
     * Uses Jonathan Richard Shewchuk's "triangle" implementation.
     */
/*
    class CELLS_API DelaunayShewchuk : public Delaunay {
    public:
        DelaunayShewchuk() ;
        virtual ~DelaunayShewchuk() ;
        virtual void begin() ;
        virtual void end() ;
        virtual Triangulation::Vertex* add_vertex(const Point2d& p) ;

    private:
        bool in_begin_end_ ;
        std::vector<Triangulation::Vertex*> vertices_ ;
    } ;
*/
}

#endif


