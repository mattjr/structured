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
 

#ifndef ___TRIANGULAR_ARRAY__
#define ___TRIANGULAR_ARRAY__

#include <OGF/basic/containers/arrays.h>

namespace OGF {
    
    /**
     * A 2d array of values aij where only the cells such 
     * that j <= i are represented.
     */
    template <class T> class TriangularArray {
    public:
        typedef TriangularArray<T> thisclass ;
        
        TriangularArray(int size) : size_(size), data_size_(size * (size + 1) / 2) {
            data_ = new T[data_size_] ;
        }
        ~TriangularArray() { delete[] data_; data_ = nil ; }

        int size() const { return size_ ; }
        int data_size() const { return data_size_ ; }

        /**
         * index0 denotes the line, and index1 the column,
         * note that index1 should be less-than or equal to
         * index0.
         */
        T& operator()(int index0, int index1) {
            ogf_array_check(index0, size_) ;
            ogf_array_check(index1, size_) ;
#ifdef OGF_ARRAY_BOUND_CHECK
            ogf_assert(index1 <= index0) ;
#endif
            int offset = index0 * (index0 + 1) / 2 ;
            return data_[offset + index1] ;
        }

        /**
         * index0 denotes the line, and index1 the column,
         * note that index1 should be lerr or equal to
         * index0.
         */
        const T& operator()(int index0, int index1) const {
            ogf_array_check(index0, size_) ;
            ogf_array_check(index1, size_) ;
#ifdef OGF_ARRAY_BOUND_CHECK
            ogf_assert(index1 <= index0) ;
#endif
            int offset = index0 * (index0 + 1) / 2 ;
            return data_[offset + index1] ;
        }

        void set_all(const T& value) {
            for(int i=0; i<data_size_; i++) {
                data_[i] = value ;
            }
        }

        T& from_linear_index(int index) {
            ogf_array_check(index, data_size_) ;
            return data_[index] ;
        }

        const T& from_linear_index(int index) const {
            ogf_array_check(index, data_size_) ;
            return data_[index] ;
        }

        T* data() const { return data_ ; }

    private:
        T* data_ ;
        int size_ ;
        int data_size_ ;

    private:
        TriangularArray(const thisclass& rhs) ;
        thisclass& operator=(const thisclass& rhs) ;
    } ;


}

#endif

