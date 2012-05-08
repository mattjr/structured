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

#include <OGF/cells/triangulation/delaunay.h>
#include <OGF/basic/debug/logger.h>

namespace OGF {

    DelaunayAPI::DelaunayAPI() : skeleton_is_dirty_(false) {
    }

    DelaunayAPI::~DelaunayAPI() {
    }

    //________________________________________________________________________________

    DelaunayAPI2d::DelaunayAPI2d() : vertices_(nil) {
    }

    DelaunayAPI2d::~DelaunayAPI2d() {
    }

    void DelaunayAPI2d::get_triangles(std::vector<int>& triangles, bool finite_only) {
        triangles.clear() ;
        Logger::err("DelaunayAPI2d") << "get_triangles() is not implemented" << std::endl ;
    }

    void DelaunayAPI2d::get_triangles_adjacent(std::vector<int>& adj_tri, bool finite_only) {
        adj_tri.clear() ;
        Logger::err("DelaunayAPI2d") << "get_triangles_adjacent() is not implemented" << std::endl ;
    }

    //________________________________________________________________________________

    DelaunayAPI3d::DelaunayAPI3d() : vertices_(nil) {
    }

    DelaunayAPI3d::~DelaunayAPI3d() {
    }

    void DelaunayAPI3d::get_tetrahedra(std::vector<int>& tetrahedra, bool finite_only) {
        tetrahedra.clear() ;
        Logger::err("DelaunayAPI3d") << "get_tetrahedra() is not implemented" << std::endl ;
    }

    void DelaunayAPI3d::get_tetrahedra_adjacent(std::vector<int>& adj_tet, bool finite_only) {
        adj_tet.clear() ;
        Logger::err("DelaunayAPI3d") << "get_tetrahedra_adjacent() is not implemented" << std::endl ;
    }

    void DelaunayAPI3d::get_voronoi_cell(unsigned int v, VoronoiCell3d& cell, bool geometry) {
        cell.clear() ;
        Logger::err("DelaunayAPI3d") << "get_voronoi_cell() is not implemented" << std::endl ;
    }


    //________________________________________________________________________________

}
