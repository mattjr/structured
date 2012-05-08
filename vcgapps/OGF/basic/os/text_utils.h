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


#ifndef __OGF_BASIC_OS_TEXT_UTILS__
#define __OGF_BASIC_OS_TEXT_UTILS__

#include <OGF/basic/common/common.h>

#include <string>
#include <vector>
#include <map>
#include <iostream>

namespace OGF {
    namespace TextUtils {

        class BASIC_API Environment {
        public:
            bool has_variable(const std::string& name) const ;
            std::string value(const std::string& name) const ;
            const std::vector<std::string>& values(const std::string& name) const ;
            void set_value(const std::string& name, const std::string& value) ;
            void set_values(const std::string& name, const std::vector<std::string>& values) ;
            void append_value(const std::string& name, const std::string& value) ;
            void append_values(const std::string& name, const std::vector<std::string>& values) ;
            void clear_value(const std::string& name) ;
            void clear() ;
            void print(std::ostream& out) const ;

        private:
            typedef std::map< std::string, std::vector<std::string> > EnvMap ;        
            EnvMap data_ ;
        } ;

        inline std::ostream& operator<<(std::ostream& out, const Environment& env) {	
            env.print(out) ;
            return out ;
        }

        void BASIC_API read_environment_file(
            const std::string& file_name,
            Environment& environment
        ) ;

        void BASIC_API find_and_replace(
            std::istream& in, std::ostream& out,
            const Environment& env
        ) ;

        void BASIC_API concatenate(
            std::istream& in, std::ostream& out
        ) ;

        bool BASIC_API file_contains_string(
            const std::string& file_name, const std::string& x
        ) ;

        void BASIC_API flip_slashes(std::string& s) ;

    }
}

#endif
