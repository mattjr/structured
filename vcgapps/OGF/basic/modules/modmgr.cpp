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
 

#include <OGF/basic/modules/modmgr.h>
#include <OGF/basic/modules/module.h>
#include <OGF/basic/os/file_manager.h>
#include <OGF/basic/debug/assert.h>
#include <OGF/basic/debug/logger.h>

#include <string>

#include <iostream>
#include <stdlib.h>
#include <string.h>

#ifdef OGF_STATIC
namespace OGF {
    ModuleManager* ModuleManager::instance_ = nil ;
    bool ModuleManager::is_graphite_a_plugin_ = false ;

    ModuleManager* ModuleManager::instance() {
        return instance_ ;
    }
    
    void ModuleManager::initialize() {
        ogf_assert(instance_ == nil) ;
        instance_ = new ModuleManager() ;
        Environment::instance()->add_environment(instance_) ;
    } 
    
    void ModuleManager::terminate() {
        delete instance_ ;
        instance_ = nil ;
    }
    
    void ModuleManager::initialize_dynamic_modules() {
        Logger::out("ModuleManager") << "Initializing static modules" 
                                     << std::endl ;
        instance()-> do_initialize_modules() ;
        Logger::out("ModuleManager") << "Initialized static modules" 
                                     << std::endl ;
    }
    
    void ModuleManager::terminate_dynamic_modules() {
        Logger::out("ModuleManager") << "Terminating dynamic modules" 
                                     << std::endl ;
        instance()-> do_terminate_modules() ;
        Logger::out("ModuleManager") << "Terminated dynamic modules" 
                                     << std::endl ;
    }
    
    ModuleManager::ModuleManager() {
    }
    
    ModuleManager::~ModuleManager() {
    }
    
    void ModuleManager::do_initialize_modules() {
        Logger::warn("ModuleManager") 
            << "do_initialize module in static mode" 
            << " <<no action>> " << std::endl ;    
    }

    void ModuleManager::do_terminate_modules() {
        Logger::warn("ModuleManager") 
            << "do_terminate module in static mode" 
            << " <<no action>> " << std::endl ;    
    }

    bool ModuleManager::load_module(
        const std::string& module_name, bool quiet
    ) {
        Logger::warn("ModuleManager") 
            << "Loading module \'" << module_name << "\' in static mode" 
            << " <<no action>> " << std::endl ;

        return true ;
    }
}

#else

#    ifdef WIN32
namespace OGF {
    ModuleManager* ModuleManager::instance_ = nil ;
	bool ModuleManager::is_graphite_a_plugin_ = false ;
    
    ModuleManager* ModuleManager::instance() {
        return instance_ ;
    }
    
    void ModuleManager::initialize() {
        ogf_assert(instance_ == nil) ;
        instance_ = new ModuleManager() ;
        Environment::instance()->add_environment(instance_) ;
    }
    
    void ModuleManager::terminate() {
        delete instance_ ;
        instance_ = nil ;
    }
    
    void ModuleManager::initialize_dynamic_modules() {
        Logger::out("ModuleManager") << "Initializing dynamic modules" 
                                     << std::endl ;
        instance()-> do_initialize_modules() ;
        Logger::out("ModuleManager") << "Initialized dynamic modules" 
                                     << std::endl ;
    }
    
    void ModuleManager::terminate_dynamic_modules() {
        Logger::out("ModuleManager") << "Terminating dynamic modules" 
                                     << std::endl ;
        instance()-> do_terminate_modules() ;
        Logger::out("ModuleManager") << "Terminated dynamic modules" 
                                     << std::endl ;
    }
    
    ModuleManager::ModuleManager() {
    }
    
    ModuleManager::~ModuleManager() {
    }
    
    void ModuleManager::do_initialize_modules() {
    }

    void ModuleManager::do_terminate_modules() {
        int nb_funcs = to_terminate_.size() ;
        for(int i = nb_funcs - 1 ; i >= 0; i--) {
            ModuleTerminateFunc module_terminate = to_terminate_[i] ;
            module_terminate() ;
        }
        int nb_modules = module_handles_.size() ;
        ogf_assert(nb_modules == nb_funcs) ;
        for(int i = nb_modules - 1 ; i >= 0; i--) {
            void* module_handle = module_handles_[i] ;
            FreeLibrary( (HMODULE) module_handle) ;
        }
    }

    bool ModuleManager::load_module(
        const std::string& module_name, bool quiet
    ) {

        Logger::out("ModuleManager") 
            << "Loading module \'" << module_name << "\'" << std::endl ;
        
        // Construct the (system-dependent) file name of the module.
        
        std::string module_file_name = 
            FileManager::instance()->dll_prefix() + 
            module_name + FileManager::instance()->dll_extension() ;

        
        if(! FileManager::instance()-> find_binary_file(
               module_file_name
        )) {
            if(!quiet) {
                Logger::err("ModuleManager")  
                    << "module \'" << module_name 
                    << "\' not found" << std::endl ;
            }
            return false ;
        }
        

        void* module_handle = LoadLibrary(module_file_name.c_str());
    
        if(module_handle == nil) {
            // get the error message
            LPVOID lpMsgBuf;
            FormatMessage( 
                FORMAT_MESSAGE_ALLOCATE_BUFFER | 
                FORMAT_MESSAGE_FROM_SYSTEM | 
                FORMAT_MESSAGE_IGNORE_INSERTS,
                NULL,
                GetLastError(),
                MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), // Default language
                (LPTSTR) &lpMsgBuf,
                0,
                NULL 
            );
            // print the error
            Logger::err("ModuleManager")  
                << "Could not open module \'" 
                << module_name << "\'" << std::endl 
                << "                     Error opening file \'"
                << module_file_name << "\'" << std::endl 
                << " reason: " << (LPTSTR) lpMsgBuf << std::endl ;
            // free the error message
            LocalFree( lpMsgBuf );
            return  false ;
        }
        
        // Retreive function pointers to initialize and
        //   terminate the module.

        std::string project_name = "OGF" ;
        std::string init_func_name =
            project_name + "_" + module_name + "_initialize" ;
        
        std::string term_func_name =
            project_name + "_" + module_name + "_terminate" ;        
        
        ModuleInitFunc p_init_func =
            (ModuleInitFunc)GetProcAddress(
                (HMODULE)module_handle, init_func_name.c_str()
            ) ;
        
        ModuleTerminateFunc p_term_func =
            (ModuleTerminateFunc)GetProcAddress(
                (HMODULE)module_handle, term_func_name.c_str()
            ) ;
        
        // Checks whether the module has init/termination functions.

        if(p_init_func == nil || p_term_func == nil) {
            Logger::warn("ModuleManager")
                << "[ModuleManager]: "
                << "Could not find init/term of module \'" 
                << module_name << "\'" << std::endl ;
        } else {
            // invoke the initialization function:
            // No need here to call p_init_func(), it is automatically 
            // called by ctor of objects at global scope
            
            // memorize the module handle
            module_handles_.push_back(module_handle) ;

            // memorize the termination function.
            to_terminate_.push_back(p_term_func) ;
        }
        
        Module* module_info = ModuleManager::resolve_module(module_name) ;
        if(module_info == nil) {
            module_info = new Module ;
            module_info->set_name(module_name) ;
            ModuleManager::bind_module(module_name, module_info) ;
        } 
        module_info->set_is_dynamic(true) ;
        
        Logger::out("ModuleManager") 
            << "Successfully loaded module \'" << module_name << "\'" 
            << std::endl ;

        return true ;
    }
}

#    else

#include <dlfcn.h>

namespace OGF {

//____________________________________________________________________________

    ModuleManager* ModuleManager::instance_ = nil ;
    bool ModuleManager::is_graphite_a_plugin_ = false ;
 
    ModuleManager* ModuleManager::instance() {
        return instance_ ;
    }
    
    void ModuleManager::initialize() {
        ogf_assert(instance_ == nil) ;
        instance_ = new ModuleManager() ;
        Environment::instance()->add_environment(instance_) ;
    }
    
    void ModuleManager::terminate() {
        delete instance_ ;
        instance_ = nil ;
    }

    void ModuleManager::initialize_dynamic_modules() {
        Logger::out("ModuleManager") << "Initializing dynamic modules" 
                                     << std::endl ;
        instance()-> do_initialize_modules() ;
        Logger::out("ModuleManager") << "Initialized dynamic modules" 
                                     << std::endl ;
    }
    
    void ModuleManager::terminate_dynamic_modules() {
        Logger::out("ModuleManager") << "Terminating dynamic modules" 
                                     << std::endl ;
        instance()-> do_terminate_modules() ;
        Logger::out("ModuleManager") << "Terminated dynamic modules" 
                                     << std::endl ;
    }
    
    ModuleManager::ModuleManager() {
    }
    
    ModuleManager::~ModuleManager() {
    }
    
    void ModuleManager::do_initialize_modules() {
        char* c_modules_env = getenv("OGF_MODULES") ; 
        
        if(c_modules_env == nil) {
            return ;
        }
        
        std::string modules_env = c_modules_env ;
        if(modules_env.length() == 0)  {
            return ;
        }

        std::vector<std::string> module_names ;
        String::split_string(modules_env, ':', module_names) ; 
        
        for(unsigned int i=0; i<module_names.size(); i++) {
            load_module(module_names[i]) ;
        }
    }

    void ModuleManager::do_terminate_modules() {
        int nb_funcs = to_terminate_.size() ;
        for(int i = nb_funcs - 1 ; i >= 0; i--) {
            ModuleTerminateFunc module_terminate = to_terminate_[i] ;
            module_terminate() ;
        }
        int nb_modules = module_handles_.size() ;
        ogf_assert(nb_modules == nb_funcs) ;
        for(int i = nb_modules - 1 ; i >= 0; i--) {
            void* module_handle = module_handles_[i] ;
            dlclose(module_handle) ;
        }
    }

    bool ModuleManager::load_module(
        const std::string& module_name, bool quiet
    ) {
        
        Logger::out("ModuleManager") 
            << "Loading module \'" << module_name << "\'" << std::endl ;
        
        // Construct the (system-dependant) file name of the module.
        
        std::string module_file_name = 
            "lib/" + FileManager::instance()->dll_prefix() + 
            module_name + FileManager::instance()->dll_extension() ;

        
        if(! FileManager::instance()-> find_binary_file(
               module_file_name
        )) {
            if(!quiet) {
                Logger::err("ModuleManager")  
                    << "module \'" << module_name 
                    << "\' not found" << std::endl ;
            }
            return false ;
        }
        

        // Invoke the dynamic linker with the following flags :
        //
        // RTLD_NOW:    I want errors to be notified as soon as possible
        // RTLD_GLOBAL: different modules may be inter-dependant.
        
        void* module_handle = dlopen(
            module_file_name.c_str(), RTLD_NOW | RTLD_GLOBAL
        ) ;
    
        if(module_handle == nil) {
            Logger::err("ModuleManager")  
                << "Could not open module \'" 
                << module_name << "\'" << std::endl 
                << "                     Error opening file \'"
                << module_file_name << "\'" << std::endl 
                << " reason: " << dlerror() << std::endl ;
            return  false ;
        }
        
        // Retreive function pointers to initialize and
        //   terminate the module.

        std::string project_name = "OGF" ;
        std::string init_func_name =
            project_name + "_" + module_name + "_initialize" ;
        
        std::string term_func_name =
            project_name + "_" + module_name + "_terminate" ;        
        
        ModuleInitFunc p_init_func =
            (ModuleInitFunc)dlsym(module_handle, init_func_name.c_str()) ;
        
        ModuleTerminateFunc p_term_func =
            (ModuleTerminateFunc)dlsym(module_handle, term_func_name.c_str()) ;
        
        // Checks whether the module has init/termination functions.

        if(p_init_func == nil || p_term_func == nil) {
            Logger::warn("ModuleManager")
                << "[ModuleManager]: "
                << "Could not find init/term of module \'" 
                << module_name << "\'" << std::endl ;
        } else {
            // invoke the initialization function.
            // No need to call p_init_func(), automatically called
            // at DLL loading time by ctor of global objects.
            
            // memorize the module handle
            module_handles_.push_back(module_handle) ;

            // memorize the termination function.
            to_terminate_.push_back(p_term_func) ;
        }
        
        Module* module_info = ModuleManager::resolve_module(module_name) ;
        if(module_info == nil) {
            module_info = new Module ;
            module_info->set_name(module_name) ;
            ModuleManager::bind_module(module_name, module_info) ;
        } 
        module_info->set_is_dynamic(true) ;
        
        Logger::out("ModuleManager") 
            << "Successfully loaded module \'" << module_name << "\'" 
            << std::endl ;

        return true ;
    }

}

#    endif
#endif

namespace OGF {

    bool ModuleManager::bind_module(
        const std::string& module_name, Module* module
    ) {
        if(resolve_module(module_name) != nil) {
            Logger::err("ModuleManager") << "Module \'" << module_name
                                         << "\' is already bound"
                                         << std::endl ;
            return false ;
        }
        modules_[module_name] = module ;
        Environment::notify_observers("loaded_modules") ;
        return true ;
    }
    
    bool ModuleManager::unbind_module(const std::string& module_name) {
        std::map<std::string,Module_var>::iterator 
            it = modules_.find(module_name) ;
        if(it == modules_.end()) {
            Logger::err("ModuleManager") << "No such module: \'" << module_name
                                         << "\'" << std::endl ;
            return false ;
        }
        modules_.erase(it) ;
        Environment::notify_observers("loaded_modules") ;
        return true ;
    }
    
    Module* ModuleManager::resolve_module(const std::string& module_name) {
        std::map<std::string,Module_var>::iterator 
            it = modules_.find(module_name) ;
        if(it == modules_.end()) {
            return nil ;
        }
        return it->second ;
    }
	
    bool ModuleManager::resolve(const std::string& name, std::string& value) const {
        if(name == "loaded_modules") {
            value = "" ;
            for(std::map<std::string,Module_var>::const_iterator it = modules_.begin();
                it != modules_.end(); it++
            ) {
                if(value.length() != 0) {
                    value += ";" ;
                }
                value += it->first ;
            }
            return true;
        } else {
            return false ;
        }
    }

}
