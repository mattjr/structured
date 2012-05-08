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
 

#ifndef __OGF_BASIC_OS_ENVIRONMENT__
#define __OGF_BASIC_OS_ENVIRONMENT__

#include <OGF/basic/common/common.h>
#include <string>
#include <vector>
#include <map>

namespace OGF {
    
    //-----------------------------------------------------------
    
    class BASIC_API EnvironmentVariableObserver {
    public:
        EnvironmentVariableObserver(const std::string& var_name) ;
        virtual void notify_value_changed(const std::string& new_value) = 0 ;
        virtual ~EnvironmentVariableObserver() ;
        const std::string& observed_environment_variable() { return observed_variable_ ; }
    private:
        std::string observed_variable_ ;
    } ;

    //-----------------------------------------------------------

    class BASIC_API EnvironmentVariableObserverList {
    public:
        EnvironmentVariableObserverList() : block_notify_(false) { }
        void notify_observers(const std::string& value) ;
        void add_observer(EnvironmentVariableObserver* observer) ;
        void remove_observer(EnvironmentVariableObserver* observer) ;
    private:
        std::vector<EnvironmentVariableObserver*> observers_ ;
        bool block_notify_ ;
    } ;

    //-----------------------------------------------------------

    class BASIC_API Environment {
    public:
        virtual ~Environment() ;

        /** 
         * note: this environment does not become responsible of the
         * memory management of env.
         */
        virtual bool add_environment(Environment* env) ;
        bool has_value(const std::string& name) const ;
        std::string get_value(const std::string& name) const ;
        virtual bool set_value(const std::string& name, const std::string& value)  ;
        static Environment* instance() ;
        virtual bool resolve(const std::string& name, std::string& value) const = 0 ;

        virtual bool add_observer(const std::string& name, EnvironmentVariableObserver* obs) ;
        virtual bool remove_observer(const std::string& name, EnvironmentVariableObserver* obs) ;
        virtual bool notify_observers(const std::string& name) ;

    protected:
        Environment* find_environment(const std::string& variable_name) ;
        
    private:
        static Environment* instance_ ;
    } ;

    //-----------------------------------------------------------

    class StoredEnvironment : public Environment {
    public:
        virtual ~StoredEnvironment() ;
        virtual bool add_environment(Environment* env) ;
        virtual bool set_value(const std::string& name, const std::string& value) ;

        virtual bool add_observer(const std::string& name, EnvironmentVariableObserver* obs) ;
        virtual bool remove_observer(const std::string& name, EnvironmentVariableObserver* obs) ;
        virtual bool notify_observers(const std::string& name) ;

    protected:
        virtual bool resolve(const std::string& name, std::string& value) const ;

    private:
        std::map<std::string, std::string> values_ ;
        std::vector< Environment* > environments_ ;
        typedef std::map<std::string, EnvironmentVariableObserverList> ObserverMap ;
        ObserverMap observers_ ;
    } ;

    //-----------------------------------------------------------

    class SystemEnvironment : public Environment {
    public:
        virtual bool set_value(const std::string& name, const std::string& value) ;

    protected:
        virtual bool resolve(const std::string& name, std::string& value) const ;
    } ;

    //-----------------------------------------------------------

}

#endif
