
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
 


#ifndef __OGF_MATH_NUMERIC_EIGEN_SOLVER__
#define __OGF_MATH_NUMERIC_EIGEN_SOLVER__

#include <OGF/math/common/common.h>
#include <OGF/basic/types/counted.h>
#include <OGF/basic/types/smart_pointer.h>

namespace OGF {

    class SparseMatrix ;
    class Vector ;

    class MATH_API EigenSolver : public Counted {
    public:
        
        enum Mode {SMALLEST, LARGEST} ;

        EigenSolver() ;
        virtual ~EigenSolver() ;

        void set_matrix(const SparseMatrix* A) { A_ = A ; }
        void set_right_hand_side_matrix(const SparseMatrix* B) { B_ = B ; }
        void unset_right_hand_side_matrix() { B_ = nil ; }

        void set_threshold(double epsilon) { epsilon_ = epsilon ; }
        void set_max_iter(int max_iter) { max_iter_ = max_iter ; }
        void set_nb_eigens(int nb_eigens) { nb_eigens_ = nb_eigens ; }
        void set_mode(Mode m) { mode_ = m ; }
        void set_shift(double x) { shift_ = x ; shift_invert_ = true ; }
        void unset_shift() { shift_invert_ = false ; }
        void set_direct_solver(const std::string& x) { direct_solver_ = x ; }
        void set_bruno(bool x) { bruno_ = x ; }
        void set_compute_eigenvectors(bool x) { compute_eigen_vectors_ = x ; }
        double get_threshold() { return epsilon_ ; }
        int get_max_iter() { return max_iter_ ; }
        int get_nb_eigens() { return nb_eigens_ ; }
        Mode get_mode() { return mode_ ; }
        double get_shift() { return shift_ ; }
        std::string get_direct_solver() { return direct_solver_  ; }
        bool get_bruno() { return bruno_ ; }
        bool get_compute_eigenvectors() { return compute_eigen_vectors_ ; }

        virtual bool solve() = 0 ;
        virtual double* get_eigen_vector(int index) = 0 ;
        virtual void get_eigen_vector(int index, Vector& v) = 0 ;
        virtual double get_eigen_value(int index) = 0 ;

        virtual bool supports_symmetric_storage() const ; // default returns false

    protected:
        double epsilon_ ;
        int max_iter_ ;
        int nb_eigens_ ;
        Mode mode_ ;
        bool shift_invert_ ;
        std::string direct_solver_ ;
        bool bruno_ ;
        double shift_ ;
        bool compute_eigen_vectors_ ;
        const SparseMatrix* A_ ;
        const SparseMatrix* B_ ;
    } ;
    
    typedef SmartPointer<EigenSolver> EigenSolver_var ;

}

#endif


