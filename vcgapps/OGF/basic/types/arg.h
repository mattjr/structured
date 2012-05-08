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

#ifndef __OGF_BASIC_TYPES_ARG__
#define __OGF_BASIC_TYPES_ARG__

#include <OGF/basic/common/common.h>
#include <OGF/basic/debug/assert.h>
#include <OGF/basic/types/types.h>

#include <vector>
#include <string>
#include <iostream>
#include <typeinfo>

namespace OGF {

//______________________________________________________________
//  New Arg system. Args can have whatever type you need.
//______________________________________________________________

	class BASIC_API ArgBase {
	public:
		ArgBase() {}
		ArgBase(const std::string& name) : name_(name){}
		virtual ~ArgBase(){}

		const std::string& name() const { return name_ ; } 
		
		template <typename T> bool get_value( T& value ) const ;
		template <typename T> bool set_value( const T& value ) ;
		const std::string string_value() const; 
        int int_value() const ;
        double double_value() const ;

		bool operator<(const ArgBase& other) const { return (this->name() < other.name()); }
		bool operator==(const ArgBase& other) const { return (this->name() == other.name()); }

		virtual const std::type_info& arg_type_id() const = 0 ;
		virtual ArgBase* clone() const = 0;
		
	protected:
		virtual std::string value_to_string() const = 0;

	private:
		std::string name_ ;
	} ;


	template < typename ArgValueType >
	class GenericArg : public ArgBase {
	public:
		GenericArg(const std::string& name, const ArgValueType& value) : ArgBase(name), value_(value) {}
		virtual ~GenericArg() {}

		const ArgValueType& value() const{ return value_; }
		void set_value(const ArgValueType& val){ value_ = val; }
		
		virtual const std::type_info& arg_type_id() const { return typeid(ArgValueType) ; }

		virtual ArgBase* clone() const{
			ArgBase* my_clone = new GenericArg<ArgValueType>(name(), value());
			return my_clone; 
		}

	protected:
		virtual std::string value_to_string() const{
			std::ostringstream oss;
			oss << value_;
			return oss.str();
		}

		ArgValueType value_;
	} ;


	//------------------------------------------------------------

	// Arglist serialization and deserialization are currently disabled. 
	class BASIC_API ArgList : public ::std::vector<ArgBase*> {
	public:		
		ArgList(){}
		ArgList(const ArgList& other){ append(other); }
		~ArgList() { clear(); }
		
		ArgList& operator=(const ArgList& other){
			clear();
			append(other);
			return *this;
		}
		
		void clear();


		int nb_args() const { return size() ; }
		ArgBase* ith_arg(int i) { 
			ogf_assert(i >= 0 && i < nb_args()) ;
			return (*this)[i] ; 
		}
		const ArgBase* ith_arg(int i) const { 
			ogf_assert(i >= 0 && i < nb_args()) ;
			return (*this)[i] ; 
		}

		
		bool has_arg(const std::string& arg_name) const;
		ArgBase* find_arg(const std::string& arg_name);
		const ArgBase* find_arg(const std::string& arg_name) const;

		template <typename T>  bool create_arg(const std::string& arg_name, const T& value) ;

		template <typename T>  bool set_arg_value(const std::string& arg_name, const T& value) ;
		template <typename T>  bool set_arg_value(const unsigned int arg_index, const T& value) ;

		template <typename T>  bool get_arg_value(const std::string& arg_name, T& value) const ;
		template <typename T>  bool get_arg_value(const unsigned int arg_index, T& value) const ;
		
		
		/** Return the std::string value of the argument.
			If the argument does not exist or its type is not std::string,
			returns the empty string.
		*/
		std::string arg_string_value(const std::string& arg_name) const;
		std::string arg_string_value(const unsigned int arg_index) const;

		/** Return the int value of the argument.
			If the type of the argument is std::string, a conversion is attempted with
			 atoi().
			If the argument does not exist or its type is neither int nor std::string,
			returns 0.
		*/
		int arg_int_value(const std::string& arg_name) const ;
		int arg_int_value(const unsigned int arg_index) const ;

		/** Return the double value of the argument.
			If the type of the argument is std::string, a conversion is attempted with
			 atof().
			If the argument does not exist or its type is neither double nor std::string,
			returns 0.0.
		*/
		double arg_double_value(const std::string& arg_name) const ;
		double arg_double_value(const unsigned int arg_index) const ;


		
		/* Append another ArgList.
		   If overwrite_same_type is set, elements with the same names and the
		   same value types are replaced by a copy of the element of the
		   other list. If overwrite_different_type is set, elements with the
		   same names but with different value types are replaced by a copy
		   of the element of the other list.
		*/
		void append(const ArgList& other, bool overwrite = true) ;


	private:
		ArgList::iterator find_arg_it(const std::string& arg_name);
		template <typename T>  bool set_arg_value(ArgBase* arg, const T& value) ;
		template <typename T>  bool get_arg_value(const ArgBase* arg, T& value) const ;
		std::string arg_string_value(const ArgBase* arg) const;
		int arg_int_value(const ArgBase* arg) const ;
		double arg_double_value(const ArgBase* arg) const;

	} ;

	std::ostream& operator<<(std::ostream& out, const ArgList& args)  ;


//------------------------------------------------------------


	template <typename T> bool ArgBase::get_value( T& value ) const {
		if(typeid(T) == arg_type_id()){
			const GenericArg<T>* casted_arg;
			if( casted_arg = dynamic_cast< const GenericArg<T>* >(this) ){
				value = casted_arg->value() ;
				return true ;
			}
		}
		return false;
	}

	template <typename T> bool ArgBase::set_value( const T& value ) {
		if(typeid(T) == arg_type_id()){
			GenericArg<T>* casted_arg;
			if( casted_arg = dynamic_cast< GenericArg<T>* >(this)){
				casted_arg->set_value(value) ;
				return true ;
			}
		}
		return false;
	}


	template <typename T>  bool ArgList::create_arg(const std::string& arg_name, const T& value){
		if(has_arg(arg_name)) {
			return false ;
		}
		push_back(new GenericArg<T>(arg_name, value));
		return true;
	}


	
	template <typename T>  bool ArgList::set_arg_value(const std::string& arg_name, const T& value){
		return set_arg_value(find_arg(arg_name), value);
	}
	template <typename T>  bool ArgList::set_arg_value(const unsigned int arg_index, const T& value){
		ogf_assert( arg_index < this->size());
		return set_arg_value((*this)[arg_index], value);
	}
	template <typename T>  bool ArgList::set_arg_value(ArgBase* arg, const T& value){
		if(arg != nil){
			return arg->set_value(value);
		}
		return false;
	}

	template <typename T>  bool ArgList::get_arg_value(const std::string& arg_name, T& value) const{
		return get_arg_value(find_arg(arg_name),value);
	}
	template <typename T>  bool ArgList::get_arg_value(const unsigned int arg_index, T& value) const{
		ogf_assert( arg_index < this->size());
		return get_arg_value((*this)[arg_index],value);
	}
	template <typename T>  bool ArgList::get_arg_value(const ArgBase* arg, T& value) const{
		if(arg != nil){
			return arg->get_value(value);
		}
		return false;
	}



}
#endif 

