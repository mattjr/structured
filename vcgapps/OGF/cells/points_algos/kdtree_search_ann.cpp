
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
 


#include <OGF/cells/points_algos/kdtree_search_ann.h>


namespace OGF {

    KdTreeSearchANN::KdTreeSearchANN()  {
        points_ = NULL;
        points_num_ = 0;

        tree_ = NULL;
    }


    KdTreeSearchANN::~KdTreeSearchANN() {
        if(points_)
            annDeallocPts(points_);

        delete tree_;
    }


    void KdTreeSearchANN::begin()  {
        vertices_.clear();

        if(points_)
            annDeallocPts(points_);

        delete tree_;
        tree_ = NULL;
    }


    void KdTreeSearchANN::end()  {
        points_num_ = vertices_.size();
        points_ = annAllocPts(points_num_, 3);

        for(unsigned int i = 0 ; i < points_num_ ; i++)
            {
                Point3d& p = vertices_[i]->point();

                points_[i][0] = p[0];
                points_[i][1] = p[1];
                points_[i][2] = p[2];
            }
		
        tree_ = new ANNkd_tree(points_, points_num_, 3);
    }


    void KdTreeSearchANN::add_point(VertexSet::Vertex* v)  {
        vertices_.push_back(v);
    }


    void KdTreeSearchANN::add_vertex_set(VertexSet& vs)  {
        for(VertexSet::Vertex_iterator it = vs.vertices_begin() ; it != vs.vertices_end() ; ++it)
            vertices_.push_back(it);
    }



    VertexSet::Vertex* KdTreeSearchANN::find_closest_point(const Point3d& p)  {
        ANNcoord ann_p[3];

        ANNidx closest_pt_ix;
        ANNdist closest_pt_dist;

        ann_p[0] = p[0];
        ann_p[1] = p[1];
        ann_p[2] = p[2];

        tree_->annkSearch(ann_p, 1, &closest_pt_ix, &closest_pt_dist);

        return vertices_[closest_pt_ix];
    }

}

