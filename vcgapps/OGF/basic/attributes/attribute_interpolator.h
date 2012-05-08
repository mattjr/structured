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

#ifndef ___ATTRIBUTE_INTERPOLATOR__
#define ___ATTRIBUTE_INTERPOLATOR__

#include <OGF/basic/common/common.h>
#include <OGF/basic/types/types.h>
#include <OGF/basic/types/counted.h>
#include <OGF/basic/types/smart_pointer.h>

#include <map>
#include <string>
#include <typeinfo>

namespace OGF {
    
    //_________________________________________________________________________________________________________

    /**
     * AttributeInterpolator is used to compute linear combinations
     * of attributes attached to an object. For instance, it can be used
     * to interpolate attribute on a surface when generating new vertices.
     */
    class BASIC_API AttributeInterpolator : public Counted {
    public:
        static void initialize() ;
        static void terminate() ;

        static AttributeInterpolator* resolve_by_type(const std::type_info& attribute_type) ;

        virtual ~AttributeInterpolator() ;

        static void bind(
            const std::type_info& attribute_type,
            AttributeInterpolator* interpolator
        ) ;

        virtual void interpolate_2(
            Memory::pointer to, 
            double a1, Memory::pointer x1, 
            double a2, Memory::pointer x2
        ) ;

        virtual void interpolate_3(
            Memory::pointer to, 
            double a1, Memory::pointer x1, 
            double a2, Memory::pointer x2, 
            double a3, Memory::pointer x3
        ) ;

        virtual void interpolate_4(
            Memory::pointer to, 
            double a1, Memory::pointer x1, 
            double a2, Memory::pointer x2, 
            double a3, Memory::pointer x3,
            double a4, Memory::pointer x4
        ) ;

        virtual void interpolate_begin(Memory::pointer to)  = 0 ;
        virtual void interpolate_add(double a, Memory::pointer x) = 0 ;
        virtual void interpolate_end() = 0 ;

    private:
        typedef std::map<std::string, SmartPointer<AttributeInterpolator> > InterpolatorMap ;
        static InterpolatorMap* type_to_interpolator_ ;
    } ;

    typedef SmartPointer<AttributeInterpolator> AttributeInterpolator_var ;

    //_________________________________________________________________________________________________________

    template <class T> class NumericAttributeInterpolator : public AttributeInterpolator {
    public:
        virtual void interpolate_begin(Memory::pointer to) { 
            target_ = (T*) to ;
            *target_ = T() ;
        }

        virtual void interpolate_add(double a, Memory::pointer x) {
            *target_ += a * (*(T*)x) ;
        }

        virtual void interpolate_end() {
            target_ = nil ;
        }

    protected:
        T* target_ ;
    } ;

    //_________________________________________________________________________________________________________

    template <class T> class ogf_register_attribute_interpolator {
    public:
        ogf_register_attribute_interpolator(AttributeInterpolator* interpolator) {
            AttributeInterpolator::bind(typeid(T), interpolator) ;
        }
    } ;

    //_________________________________________________________________________________________________________

    template <class T> class ogf_register_numeric_attribute_interpolator {
    public:
        ogf_register_numeric_attribute_interpolator() {
            AttributeInterpolator::bind(typeid(T), new NumericAttributeInterpolator<T>) ;
        }
    } ;

    //_________________________________________________________________________________________________________

}

#endif
