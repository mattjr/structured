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

#include <OGF/basic/types/arg.h>
#include <OGF/basic/types/types.h>
#include <OGF/basic/debug/assert.h>
#include <stdlib.h>

//___________________________________________________

namespace OGF {
//---------------------------------------------------
//  Arguments that can have whatever type you need.
//---------------------------------------------------


	const std::string ArgBase::string_value() const {
		std::string res;
		if (!get_value(res)) {
			return value_to_string() ;
		}
		return res;
	}

	int ArgBase::int_value() const {
		int int_val = 0 ;
		std::string string_val ;
		bool arg_has_an_int_value = get_value(int_val) ;
		if( !arg_has_an_int_value && get_value(string_val) ) {
			return ::atoi(string_val.c_str()) ;
		}
		ogf_assert(arg_has_an_int_value);
		return int_val;
	}

	double ArgBase::double_value() const {
		double double_val = 0.0 ;
		std::string string_val ;
		bool arg_has_a_double_value = get_value(double_val) ;
		if( !arg_has_a_double_value && get_value(string_val) ) {
			return ::atof(string_val.c_str()) ;
		}
		ogf_assert(arg_has_a_double_value);
		return double_val;
	}

	// Currently no deserialisation. use string_value to get
	// a printable representation of the value but you won't be able to recreate
	// the argument with it.

	/*
	std::ostream& operator<<(std::ostream& out, const ArgBase& arg){
		return arg.print(out) ;
	}
	
	std::ostream& ArgBase::print(std::ostream& out, const std::string& separator) const{
		out << name() << separator << "YOU FORGOT TO OVERRIDE print FOR THIS ARGUMENT TYPE" ;
		return out ;
	}
	std::string ArgBase::to_string(const std::string& separator) const{
		std::ostringstream oss;
		print(oss, separator);
		return oss.str();
	}
	*/
	//___________________________________________________

	void ArgList::clear(){
		for(ArgList::const_iterator it = begin(); it != end(); ++it) {
			delete (*it);
		}
		::std::vector<ArgBase*>::clear();
	}


	bool ArgList::has_arg(const std::string& arg_name) const {
		return (find_arg(arg_name) != nil) ;
	}
	
	
	ArgBase* ArgList::find_arg(const std::string& arg_name) {
		for(ArgList::iterator it = begin(); it != end(); it++) {
			if((*it)->name() == arg_name) {
				return *it ;
			}
		}
		return nil ;
	}


	const ArgBase* ArgList::find_arg(const std::string& arg_name) const {
		for(ArgList::const_iterator it = begin(); it != end(); it++) {
			if((*it)->name() == arg_name) {
				return *it ;
			}
		}
		return nil ;
	}
	

	void ArgList::append(const ArgList& other, bool overwrite) {
		ArgList::iterator it;
		for(unsigned int i=0; i<other.size(); i++) {
			it = find_arg_it(other[i]->name());
			if( it == end() ) {
				push_back(other[i]->clone());
			} else {
				if(overwrite) {
					// if the element has a different type : output a warning,
					// In any case, erase the element and insert a copy of the other.
					if( (*it)->arg_type_id() != other[i]->arg_type_id() ){
						Logger::warn("Basic") << "During ArgList::append, argument \""<< (*it)->name()
							<< "\" had type " << (*it)->arg_type_id().name()
							<< ", replaced by type " << other[i]->arg_type_id().name() << std::endl;
					}
					delete *it;
					erase(it);
					push_back(other[i]->clone());
				}
			}
		}
	}


	std::string ArgList::arg_string_value(const std::string& arg_name) const {
		return arg_string_value(find_arg(arg_name));
	}
	std::string ArgList::arg_string_value(const unsigned int arg_index) const {
		ogf_assert( arg_index < this->size());
		return arg_string_value((*this)[arg_index]);
	}
	std::string ArgList::arg_string_value(const ArgBase* arg) const {
		if(arg != nil){
			return arg->string_value();
		}
		//ogf_assert_not_reached;
		// as long as we use strings in place of other types we can't dissallow
		// getting the string value of non-string args.
		return std::string();
	}



	int ArgList::arg_int_value(const std::string& arg_name) const {
		return arg_int_value(find_arg(arg_name));
	}
	int ArgList::arg_int_value(const unsigned int arg_index) const {
		ogf_assert( arg_index < this->size());
		return arg_int_value((*this)[arg_index]);
	}
	int ArgList::arg_int_value(const ArgBase* arg) const {
		if(arg != nil){
			return arg->int_value();
		}
		//ogf_assert_not_reached;
		return 0;
	}
	
	
	
	
	double ArgList::arg_double_value(const std::string& arg_name) const {
		return arg_double_value(find_arg(arg_name));
	}
	double ArgList::arg_double_value(const unsigned int arg_index) const {
		ogf_assert( arg_index < this->size());
		return arg_double_value((*this)[arg_index]);
	}
	double ArgList::arg_double_value(const ArgBase* arg) const {
		if(arg != nil){
			return arg->double_value();
		}
		//ogf_assert_not_reached;
		return 0.0;
	}
	


	ArgList::iterator ArgList::find_arg_it(const std::string& arg_name){
		for(ArgList::iterator it = begin(); it != end(); it++) {
			if((*it)->name() == arg_name) {
				return it ;
			}
		}
		return end();
	}

	std::ostream& operator<<(std::ostream& out, const ArgList& args) {
        out << "( " ;
        for(int i=0; i<args.nb_args(); i++) {
            out << args.ith_arg(i)->name() << ":" << args.ith_arg(i)->string_value() ;
            if(i != args.nb_args() - 1) {
                out << " ; " ;
            }
        }
        out << ") " ;
        return out ;
    }


}

