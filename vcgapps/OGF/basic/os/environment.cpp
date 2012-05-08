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
 
#include <OGF/basic/os/environment.h>
#include <OGF/basic/types/types.h>
#include <OGF/basic/debug/assert.h>
#include <algorithm>
#include <stdlib.h>

namespace OGF {


    //__________________________________________________________________


    EnvironmentVariableObserver::EnvironmentVariableObserver(const std::string& var_name) {
        observed_variable_ = var_name ;
        bool added_observer = Environment::instance()->add_observer(var_name, this) ;
        ogf_assert(added_observer) ;
    }

    EnvironmentVariableObserver::~EnvironmentVariableObserver() {
        bool removed_observer = Environment::instance()->remove_observer(observed_variable_, this) ;
        ogf_assert(removed_observer) ;
    }

    void EnvironmentVariableObserverList::notify_observers(const std::string& value) {
        if(block_notify_) {
            return ;
        }
        block_notify_ = true ;
        for(unsigned int i = 0; i < observers_.size(); i++) {
            observers_[i]->notify_value_changed(value) ;
        }
        block_notify_ = false ;
    }

    void EnvironmentVariableObserverList::add_observer(EnvironmentVariableObserver* obs) {
        std::vector<EnvironmentVariableObserver*>::iterator it = 
            std::find(observers_.begin(), observers_.end(), obs) ;
        ogf_assert(it == observers_.end()) ;
        observers_.push_back(obs) ;
    }

    void EnvironmentVariableObserverList::remove_observer(EnvironmentVariableObserver* obs) {
        std::vector<EnvironmentVariableObserver*>::iterator it = 
            std::find(observers_.begin(), observers_.end(), obs) ;        
        ogf_assert(it != observers_.end()) ;
        observers_.erase(it) ;
    }

    //__________________________________________________________________

    Environment* Environment::instance_ = nil ;

    Environment::~Environment() {
    }

    bool Environment::add_environment(Environment* env) {
        return false ;
    }
    
    bool Environment::has_value(const std::string& name) const {
        std::string value ;
        return resolve(name, value) ;
    }

    bool Environment::set_value(const std::string&, const std::string&) {
        return false ;
    }
    
    std::string Environment::get_value(const std::string& name) const {
        std::string result ;
        resolve(name, result) ;
        return result ;
    }
    
    Environment* Environment::instance() {
        if(instance_ == nil) {
            instance_ = new StoredEnvironment() ;
            instance_->add_environment(new SystemEnvironment()) ;
        }
        return instance_ ;
    }

    bool Environment::add_observer(const std::string& name, EnvironmentVariableObserver* obs) {
        return Environment::instance()->add_observer(name, obs) ;
    }
    
    bool Environment::remove_observer(const std::string& name, EnvironmentVariableObserver* obs) {
        return Environment::instance()->remove_observer(name, obs) ;
    }
    
    bool Environment::notify_observers(const std::string& name) {
        return Environment::instance()->notify_observers(name) ;
    }

    //__________________________________________________________________

    StoredEnvironment::~StoredEnvironment() {  }

    bool StoredEnvironment::add_environment(Environment* env) {
        environments_.push_back(env) ;
        return true ;
    }
    
    bool StoredEnvironment::set_value(
        const std::string& name, const std::string& value
    ) {
        for(unsigned int i=0; i<environments_.size(); i++) {
            if(environments_[i]->set_value(name,value)) {
                return true ;
            }
        }
        values_[name] = value ;
        notify_observers(name) ;
        return true ;
    }
    
    bool StoredEnvironment::resolve(
        const std::string& name, std::string& value
    ) const {
        std::map<std::string, std::string>::const_iterator 
            it = values_.find(name) ;
        if(it != values_.end()) {
            value = it->second ;
            return true ;
        }
        for(unsigned int i=0; i<environments_.size(); i++) {
            if(environments_[i]->resolve(name,value)) {
                return true ;
            }
        }
        return false ;
    }
    
    bool StoredEnvironment::add_observer(
        const std::string& name, EnvironmentVariableObserver* obs
    ) {
        observers_[name].add_observer(obs) ;
        return true ;
    }
    
    bool StoredEnvironment::remove_observer(
        const std::string& name, EnvironmentVariableObserver* obs
    ) {
        ogf_assert(observers_.find(name) != observers_.end()) ;
        observers_[name].remove_observer(obs) ;
        return true ;
    }
    
    bool StoredEnvironment::notify_observers(const std::string& name) {
        std::string value ;
        bool variable_exists = resolve(name, value) ;
        if(!variable_exists) {
            Logger::err("Environment") << name << ": no such environment variable" << std::endl ;
        }
        ogf_assert(variable_exists) ;
        ObserverMap::iterator it = observers_.find(name) ;
        if(it != observers_.end()) {
            it->second.notify_observers(value) ;
        }
        return true ;
    }
    
    //__________________________________________________________________

    bool SystemEnvironment::set_value(
        const std::string& name, const std::string& value
    ) {
        return false ;
    }
    
    bool SystemEnvironment::resolve(
        const std::string& name, std::string& value
    ) const {
        // For the moment, deactivated under Windows
#ifdef WIN32
        return false ;
#else
        char* result = ::getenv(name.c_str()) ;
        if(result != nil) {
            value = std::string(result) ;
        }
        return (result != nil) ;
#endif
    }

    //__________________________________________________________________    

}
