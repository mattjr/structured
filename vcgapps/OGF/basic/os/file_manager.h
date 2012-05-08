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
 

#ifndef __OGF_BASIC_OS_FILE_MANAGER__
#define __OGF_BASIC_OS_FILE_MANAGER__

#include <OGF/basic/common/common.h>
#include <string>
#include <vector>

namespace OGF {

//_________________________________________________________

/**
 * FileManager makes it possible to retreive a set of
 * files used by an application. The files are retreived
 * thanks to the environment variable OGF_PATH. OGF_PATH
 * is a colon-separated list of pathes.
 */

    class BASIC_API FileManager {
    public:
        static void initialize() ;
        static void terminate() ;
        static FileManager* instance() ;

        /**
         * returns the full path to file_name, retreived in OGF_PATH.
         * If not found, returns an empty string.
         */
        bool find_file(std::string& file_name, bool verbose=true) const ;

        /**
         * returns the full path to file_name, retreived in 
         * PATH/binaries/$OGF_ARCH/, where PATH takes all values
         * in OGF_PATH. dll_prefix() is prepended and dll_extension()
         * is appened to file_name.
         */
        bool find_binary_file(
            std::string& file_name, bool verbose=true
        ) const ;
    
        /**
         * returns the system-dependant dll prefix ('lib' for Unixes, and
         * nothing for Windows). This function is used by find_binary_file().
         */
        std::string dll_prefix() const ;

        /**
         * returns the system-dependant dll extension ('.so' for Unixes,
         * and '.dll' for Windows). 
         * This function is used by find_binary_file().
         */
        std::string dll_extension() const ;

        std::string base_name(const std::string& path) const ;
        std::string dir_name(const std::string& path) const ;
        std::string extension(const std::string& path) const ;

        bool can_read_file(const std::string& file_name) const ;

    protected:
        FileManager() ;


    private:
        static FileManager* instance_ ;
        std::vector<std::string> ogf_path_ ;
    } ;


//_________________________________________________________

}
#endif

