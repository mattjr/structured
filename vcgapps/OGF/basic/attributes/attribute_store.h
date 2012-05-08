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
 

#ifndef ___ATTRIBUTE_STORE__
#define ___ATTRIBUTE_STORE__

#include <OGF/basic/common/common.h>
#include <OGF/basic/attributes/raw_attribute_store.h>
#include <OGF/basic/attributes/attribute_life_cycle.h>
#include <OGF/basic/attributes/attribute_interpolator.h>
#include <OGF/basic/types/smart_pointer.h>
#include <OGF/basic/types/counted.h>

#include <typeinfo>

namespace OGF {

//_________________________________________________________

    class AttributeManager ;
    class Record ;

    /**
     * stores an attribute, and knows how to construct,copy,destroy
     * instances of the attribute. This class should not be used
     * directly by client code.
     */
    class BASIC_API AttributeStore : public Counted, public RawAttributeStore {
    public:
        
        AttributeStore(
            AttributeLifeCycle* life_cycle,
            AttributeManager* manager = nil
        ) : RawAttributeStore( life_cycle->item_size() ), 
            life_cycle_(life_cycle), manager_(nil), interpolator_(nil) {
            bind(manager) ;
        }
        
        virtual ~AttributeStore() ;

        void construct(
            Memory::pointer addr, Record* record = 0
        ) {
            life_cycle_->construct(addr,record) ;
        }

        void destroy(
            Memory::pointer addr, Record* record = 0
        ) {
            life_cycle_->destroy(addr,record) ;
        }

        void copy(
            Memory::pointer lhs, Record* record_lhs,
            Memory::pointer rhs, const Record* record_rhs
        ) {
            life_cycle_->copy(lhs,record_lhs,rhs,record_rhs) ;
        }

        void copy_construct(
            Memory::pointer lhs, Record* record_lhs,
            Memory::pointer rhs, const Record* record_rhs
        ) {
            life_cycle_->copy_construct(lhs,record_lhs,rhs,record_rhs) ;
        }

        void interpolate_2(
            Memory::pointer to, Record* record_to,
            double a1, Memory::pointer x1, const Record* record_x1,
            double a2, Memory::pointer x2, const Record* record_x2
        ) {
            if(interpolator_ != nil) {
                interpolator_->interpolate_2(to, a1, x1, a2, x2) ;
            } else {
                life_cycle_->destroy(to) ;
                life_cycle_->construct(to) ;
            }
        }

        void interpolate_3(
            Memory::pointer to, Record* record_to,
            double a1, Memory::pointer x1, const Record* record_x1,
            double a2, Memory::pointer x2, const Record* record_x2,
            double a3, Memory::pointer x3, const Record* record_x3
        ) {
            if(interpolator_ != nil) {
                interpolator_->interpolate_3(to, a1, x1, a2, x2, a3, x3) ;
            } else {
                life_cycle_->destroy(to) ;
                life_cycle_->construct(to) ;
            }
        }

        void interpolate_4(
            Memory::pointer to, Record* record_to,
            double a1, Memory::pointer x1, const Record* record_x1,
            double a2, Memory::pointer x2, const Record* record_x2,
            double a3, Memory::pointer x3, const Record* record_x3,
            double a4, Memory::pointer x4, const Record* record_x4
        ) {
            if(interpolator_ != nil) {
                interpolator_->interpolate_4(to, a1, x1, a2, x2, a3, x3, a4, x4) ;
            } else {
                life_cycle_->destroy(to) ;
                life_cycle_->construct(to) ;
            }
        }

        void interpolate_begin(Memory::pointer to, Record* record_to) {
            if(interpolator_ != nil) {
                interpolator_->interpolate_begin(to) ;
            } else {
                life_cycle_->destroy(to) ;
                life_cycle_->construct(to) ;
            }
        }

        void interpolate_add(
            double a, Memory::pointer x, Record* record_x
        ) {
            if(interpolator_ != nil) {
                interpolator_->interpolate_add(a,x) ;
            }
        }

        void interpolate_end() {
            if(interpolator_ != nil) {
                interpolator_->interpolate_end() ;
            }
        }

        void bind(AttributeManager* manager) ;
        AttributeManager* attribute_manager() const { return manager_; }

        virtual const std::type_info& attribute_type_id() const = 0 ;

        /** returns an empty AttributeStore() of the same type. */
        virtual AttributeStore* clone() = 0 ;

    protected:
        AttributeLifeCycle_var life_cycle_ ;
        AttributeManager* manager_ ;
        AttributeInterpolator* interpolator_ ;
    } ;

    typedef SmartPointer<AttributeStore> AttributeStore_var ;

//_________________________________________________________

    /**
     * A typed AttributeStore, templated by the 
     * Record class and the Attribute class. This
     * is used for static and dynamic type checking
     * in the AttributeManager.
     */
    template <class ATTRIBUTE> 
    class GenericAttributeStore : public AttributeStore {
    public:
        GenericAttributeStore(
            AttributeLifeCycle* life_cycle,
            AttributeManager* manager = nil
        ) : AttributeStore(life_cycle, manager) { 
            interpolator_ = AttributeInterpolator::resolve_by_type(typeid(ATTRIBUTE)) ;
        }        
        virtual ~GenericAttributeStore() { }
        virtual const std::type_info& attribute_type_id() const {
            return typeid(ATTRIBUTE) ;
        }
        virtual AttributeStore* clone() {
            return new GenericAttributeStore<ATTRIBUTE>(life_cycle_) ;
        }
    } ;

//_________________________________________________________

}

#endif

