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
 
 
 

#ifndef __OGF_BASIC_MODULES_MODMGR__
#define __OGF_BASIC_MODULES_MODMGR__

#include <OGF/basic/common/common.h>
#include <OGF/basic/modules/module.h>
#include <OGF/basic/os/environment.h>
#include <string>
#include <vector>
#include <map>

typedef void (*ModuleInitFunc)() ;
typedef void (*ModuleTerminateFunc)() ;

//____________________________________________________________________________

namespace OGF {
    
   /**
    * This class performs the management of dynamically
    * loadable modules. 
    */
    class BASIC_API ModuleManager : public Environment {
    public:
        static void initialize() ;
        static void initialize_dynamic_modules() ;
        static void terminate() ;
        static void terminate_dynamic_modules() ;
        
        static ModuleManager* instance() ;
        
        bool load_module(const std::string& module_name, bool quiet = false) ;

        /** Declares a Module object to the ModuleManager */
        bool bind_module(
            const std::string& module_name, Module* module
        ) ;

        /** Removes a Module object from the ModuleManager */
        bool unbind_module(const std::string& module_name) ;

        /** 
         * Retreives a Module object if it has been loaded, else returns nil.
         */
        Module* resolve_module(const std::string& module_name) ;
   
        /** Environment overload. Defines the loaded_modules variable */
        virtual bool resolve(const std::string& name, std::string& value) const ;

		static bool is_graphite_a_plugin() { return is_graphite_a_plugin_; }
		static void set_is_graphite_a_plugin( bool value ) { is_graphite_a_plugin_ = value; }

    protected:
        ModuleManager() ;
        void do_initialize_modules() ;
        void do_terminate_modules() ;
        ~ModuleManager() ;
        
    private:
        std::vector<ModuleTerminateFunc> to_terminate_ ;
        std::vector<void*> module_handles_ ;
        std::map<std::string, Module_var> modules_ ;
        static ModuleManager* instance_ ;
		static bool is_graphite_a_plugin_;
    } ;

//____________________________________________________________________________

}

#endif
