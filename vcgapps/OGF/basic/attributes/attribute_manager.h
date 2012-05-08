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
 

#ifndef ___ATTRIBUTE_MANAGER__
#define ___ATTRIBUTE_MANAGER__

#include <OGF/basic/common/common.h>
#include <OGF/basic/attributes/rat.h>
#include <OGF/basic/attributes/attribute_store.h>

#include <set>
#include <map>
#include <vector>
#include <string>
#include <typeinfo>

namespace OGF {

//_________________________________________________________

    class BASIC_API AttributeManager {
    public:

        enum Mode { FIND=1, CREATE=2, FIND_OR_CREATE=3} ;

        AttributeManager() : size_(0) { }
        virtual ~AttributeManager() ;
        unsigned int capacity() { return rat_.capacity(); }
        unsigned int size() { return size_; }

        void clear() ;

        /**
         * creates new record attributes, and puts the resulting id
         * in the specified Record
         */
        void new_record(Record* to) ;


        /**
         * creates new record attributes, initialized from the source record,
         * and puts the resulting id in the specified Record
         */
        void new_record(Record* to, const Record* from) ;

        /**
         * copies all the attributes of the from Record to the to Record.
         */
        void copy_record(Record* to, const Record* from) ;

        /**
         * destroys the record attributes corresponding to the
         * specified record.
         */
        void delete_record(Record* record) ;

        /**
         * For each attribute, to <- a1 * x1 + a2 * x2 
         */
        void interpolate_record_2(
            Record* to, double a1, Record* x1, double a2, Record* x2
        ) ;

        /**
         * For each attribute, to <- a1 * x1 + a2 * x2 + a3 * x3
         */
        void interpolate_record_3(
            Record* to, double a1, Record* x1, double a2, Record* x2, double a3, Record* x3
        ) ;

        /**
         * For each attribute, to <- a1 * x1 + a2 * x2 + a3 * x3
         */
        void interpolate_record_4(
            Record* to, 
            double a1, Record* x1, double a2, Record* x2, double a3, Record* x3, double a4, Record* x4
        ) ;

        void interpolate_record_begin(Record* to) ;
        void interpolate_record_add(double a, Record* x) ;
        void interpolate_record_end() ;

        void list_named_attributes(std::vector<std::string>& names) ;
        bool named_attribute_is_bound(const std::string& name) ;
        void delete_named_attribute(const std::string& name) ;

        virtual const std::type_info& record_type_id() const = 0 ;
		/* For an easy access to attribute type Jeanne 01/2010 */
        const std::type_info& resolve_named_attribute_type_id( const std::string& name ) ;

    protected:

        /**
         * adds the AttributeStore to the list of managed
         * attributes, resizes it, and constructs
         * all the elements which are not in the free list.
         */
        void register_attribute_store(AttributeStore* as) ;

        /**
         * detroys all the elements which are not in the
         * free list, and removes the AttributeStore from
         * the list of managed attributes.
         */
        void unregister_attribute_store(AttributeStore* as) ;

        void bind_named_attribute_store(
            const std::string& name, AttributeStore* as
        ) ;
        
        AttributeStore* resolve_named_attribute_store(
            const std::string& name
        ) ;

        RAT& rat() { return rat_ ; }
        const RAT& rat() const { return rat_ ; }

        friend class AttributeStore ;
        friend class AttributeBase ;
        template <class RECORD> friend class AttributeCopier ;

    private:
        RAT rat_ ;
        std::set<AttributeStore*> attributes_ ;
        std::map<std::string, AttributeStore_var> named_attributes_ ;

        int size_ ;
    } ;


    template <class RECORD> 
    class GenericAttributeManager : public AttributeManager {
        virtual const std::type_info& record_type_id() const {
            return typeid(RECORD) ;
        }
    } ;


//_________________________________________________________

}
#endif

