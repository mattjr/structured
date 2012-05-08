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
 
 
 

#ifndef __OGF_BASIC_MODULES_MODULE__
#define __OGF_BASIC_MODULES_MODULE__

#include <OGF/basic/common/common.h>
#include <OGF/basic/types/counted.h>
#include <string>

//____________________________________________________________________________

namespace OGF {

/**
 * Implements the management of dynamic linking.
 * Each Module can register itself to
 * the NamingLibrary, and provide information
 * represented by a Module object. 
 */

    class BASIC_API Module : public Counted {

    public:
        Module() ;


        const std::string& name() { return name_; }
        void set_name(const std::string& name_in) { name_ = name_in ; }

        const std::string& vendor() { return vendor_; }
        void set_vendor(const std::string& vendor_in) { vendor_ = vendor_in ; }

        const std::string& version() { return version_; }
        void set_version(const std::string& version_in) { 
            version_ = version_in ; 
        }

        bool is_dynamic() const { return is_dynamic_; }
        void set_is_dynamic(bool b) { is_dynamic_ = b ; }

        const std::string& info() { return info_; }
        void set_info(const std::string& info_in) { info_ = info_in ; }
        
        /** Declares a Module object to the ModuleManager */
        static bool bind_module(
            const std::string& module_name, Module* module
        ) ;
        
        /** Removes a Module object from the ModuleManager */
        static bool unbind_module(const std::string& module_name) ;

        /** 
         * Retreives a Module object if it has been loaded, else returns nil.
         */
        static Module* resolve_module(const std::string& module_name) ;

    private:
        std::string name_ ;
        std::string vendor_ ;
        std::string version_ ;
        bool is_dynamic_ ;
        std::string info_ ;
    } ;

    typedef SmartPointer<Module> Module_var ;

//____________________________________________________________________________

}

#endif
