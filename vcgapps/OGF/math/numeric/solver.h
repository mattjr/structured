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
 

#ifndef __OGF_MATH_NUMERIC_SOLVER__
#define __OGF_MATH_NUMERIC_SOLVER__

#include <OGF/math/common/common.h>
#include <OGF/basic/debug/assert.h>

namespace OGF {

    class MATH_API SolverVariable {
    public:
        SolverVariable() : x_(0), index_(-1), locked_(false) { }
        
        double value() const { return x_; }
        void set_value(double x_in) { x_ = x_in ; }
        
        void lock()   { locked_ = true ; }
        void unlock() { locked_ = false ; }
        bool is_locked() const { return locked_ ; }
        
        int index() const {
            ogf_debug_assert(index_ != -1) ;
            return index_ ;
        }
        
    protected:
        void set_index(int index_in) {
            index_ = index_in ; 
        }
        
    private:
        double x_ ;
        int index_ ;
        bool locked_ ;
        
        friend class Solver ;
    } ;

//____________________________________________________________________________

    class Progress ;

    class MATH_API Solver {
    public:
        Solver(int nb_variables) ;
        virtual ~Solver() ;

        int nb_variables() const { return nb_variables_ ; }
        
        SolverVariable& variable(int idx) { 
            ogf_assert(idx >= 0 && idx < nb_variables_) ;
            return variable_[idx] ;
        }
            
        const SolverVariable& variable(int idx) const {
            ogf_assert(idx >= 0 && idx < nb_variables_) ;
            return variable_[idx] ;
        }
        
        void set_variable_index(SolverVariable& var, int index) {
            var.set_index(index) ;
        }

        virtual void set_progress(Progress* x) ;
        Progress* progress() const  { return progress_ ; }

        /**
         * should to be called if the transient state
         * of the solver needs to be accessed.
         */
        virtual void update_variables() ;

        /** are messages disabled ? */
        bool quiet() const { return quiet_ ; }

        /** disables or enables all messages */
        virtual void set_quiet(bool x) ;

        bool ok() const { return ok_ ; }

    protected:
        // User representation
        int nb_variables_ ;
        SolverVariable* variable_ ;
        Progress* progress_ ;
        bool quiet_ ;
        bool ok_ ;
    } ;

//____________________________________________________________________________

} 

#endif
