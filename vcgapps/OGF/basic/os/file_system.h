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
 

#ifndef __OGF_BASIC_OS_FILE_SYSTEM__
#define __OGF_BASIC_OS_FILE_SYSTEM__

#include <OGF/basic/common/common.h>

#include <string>
#include <vector>

namespace OGF {

    namespace FileSystem {

        // OS-dependent functions

        bool BASIC_API is_file(const std::string& filename) ;
        bool BASIC_API is_directory(const std::string& filename) ;    
        bool BASIC_API create_directory(const std::string& path) ; // Warning: path should be absolute.
        bool BASIC_API delete_directory(const std::string& path) ;
        bool BASIC_API delete_file(const std::string& filename) ;
        void BASIC_API get_directory_entries(
            const std::string& dirname_in, std::vector<std::string>& result
        ) ;
        std::string BASIC_API get_current_working_directory() ;
        bool BASIC_API set_current_working_directory(const std::string& path) ;
        bool BASIC_API rename_file(const std::string& old_name, const std::string& new_name) ;
        unsigned int BASIC_API get_time_stamp(const std::string& filename) ;

        // OS-independent functions

        std::string BASIC_API extension(const std::string& path) ;
        std::string BASIC_API base_name(const std::string& path) ;
        std::string BASIC_API dir_name(const std::string& path) ;

        void BASIC_API get_directory_entries(
            const std::string& dirname_in, 
            std::vector<std::string>& result, bool recursive 
        ) ;

        void BASIC_API get_files(
            const std::string& dirname, 
            std::vector<std::string>& result, bool recursive = false
        ) ;

        void BASIC_API get_subdirectories(
            const std::string& dirname, 
            std::vector<std::string>& result, bool recursive = false
        ) ;

        bool BASIC_API copy_file(const std::string& original, const std::string& copy);

    }

}

#endif
