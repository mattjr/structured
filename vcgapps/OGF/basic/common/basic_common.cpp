/*
 *  GXML/Graphite: Geometry and Graphics Programming Library + Utilities
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
 

#include <OGF/basic/common/common.h>
#include <OGF/basic/os/process.h>
#include <OGF/basic/debug/logger.h>
#include <OGF/basic/modules/module.h>
#include <OGF/basic/attributes/attribute_serializer.h>
#include <OGF/basic/attributes/attribute_interpolator.h>

#include <OGF/basic/modules/modmgr.h>
#include <OGF/basic/os/file_manager.h>

namespace OGF {

/****************************************************************/
    
    void basic_libinit::initialize() {
        // To fix the double initialization problem with
        // VisualC++ .net 7.1
        // (before we find a better fix)
        static bool initialized = false ;
        if(initialized) return ;
        initialized = true ;

        Logger::initialize() ;
        Logger::out("Init") << "Initializing library \'" 
                            << "basic" << "\'" << std::endl ; 

        //_____________________________________________________________
        
        Process::initialize() ;
        Process::install_signal_handlers() ;
        
        FileManager::initialize() ;
        AttributeSerializer::initialize() ;
        AttributeInterpolator::initialize() ;

        ModuleManager::initialize() ;
        ModuleManager::initialize_dynamic_modules() ;


        ogf_register_attribute_type<int>("integer") ;
        ogf_register_attribute_type<double>("real") ;
        ogf_register_attribute_type<bool>("boolean") ;
        ogf_register_attribute_type<std::string>("string") ;

        ogf_register_numeric_attribute_interpolator<double>() ;

        Environment::instance()->set_value("version", "2.0-alpha-4") ;
        Environment::instance()->set_value("release_date", "June 2010") ;

        //_____________________________________________________________

        Module* module_info = new Module ;
        module_info->set_name("basic") ;
        module_info->set_vendor("OGF") ;
        module_info->set_version("1.0-a4") ;
        module_info->set_info("Basic types, services, containers") ;
        Module::bind_module("basic", module_info) ;
        
        Logger::out("Init") << "Initialized library \'" 
                            << "basic" << "\'" << std::endl ; 
    }
    
    void basic_libinit::terminate() {
        Logger::out("Init") << "Terminating library \'" 
                            << "basic" << "\'" << std::endl ; 

        //_____________________________________________________________

        AttributeInterpolator::terminate() ;
        AttributeSerializer::terminate() ;

        ModuleManager::terminate_dynamic_modules() ;
        Module::unbind_module("basic") ;
        ModuleManager::terminate() ;    
        Process::terminate() ;

        //_____________________________________________________________

        Logger::out("Init") << "Terminated library \'" 
                            << "basic" << "\'" << std::endl ; 

        Logger::terminate() ;
    }
    
// You should not need to modify this file below that point.
    
/****************************************************************/
    
    basic_libinit::basic_libinit() {
        increment_users() ;
    }

    basic_libinit::~basic_libinit() {
        decrement_users() ;
    }
    
    void basic_libinit::increment_users() {
        // Note that count_ is incremented before calling
        // initialize, else it would still be equal to
        // zero at module initialization time, which 
        // may cause duplicate initialization of libraries.
        count_++ ;
        if(count_ == 1) {
            initialize() ;
        }
    }
    
    void basic_libinit::decrement_users() {
        count_-- ;
        if(count_ == 0) {
            terminate() ;
        }
    }
    
    int basic_libinit::count_ = 0 ;
    
}

// The initialization and termination functions
// are also declared using C linkage in order to 
// enable dynamic linking of modules.

extern "C" void BASIC_API OGF_basic_initialize() {
    OGF::basic_libinit::increment_users() ;
}

extern "C" void BASIC_API OGF_basic_terminate() {
    OGF::basic_libinit::decrement_users() ;
}


