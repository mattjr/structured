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
 

#include <OGF/basic/os/file_manager.h>
#include <OGF/basic/os/file_system.h>
#include <OGF/basic/types/types.h>
#include <OGF/basic/debug/logger.h>
#include <OGF/basic/modules/modmgr.h>

#include <iostream>
#include <fstream>

#include <stdlib.h>
#include <ctype.h>

namespace OGF {

//_________________________________________________________

    static std::string get_project_root() {
        std::string project_root ;
#ifdef WIN32

		// get the current working directory
		std::string start = FileSystem::get_current_working_directory() ;
		std::vector<std::string> path ;
		String::split_string(start, '/', path) ;

		bool found = false ;

		while(path.size() > 0) {
			std::string tmp ;
			String::join_strings(path, '/', tmp) ;		
			project_root = tmp ;
			if(FileSystem::is_file(project_root + "/PROJECT_ROOT")) {
				found = true ;
				break ;
			}else if (FileSystem::is_directory(project_root + "/plugins")) {
				if (FileSystem::is_file(project_root + "/GraphiteTwo/PROJECT_ROOT")) {
					found = true;
					project_root += "/GraphiteTwo";
					break;
				}
			}
			path.pop_back() ;
		}

		if(!found) {
			project_root.clear();

			/* The current_working_directory cannot be identified as Graphite directory.
			 * If GRAPHITE_ROOT environment variable is defined; project_root is equal to GRAPHITE_ROOT
			 * if the string returned is empty Graphite will crash sooner or later.
			 */
			size_t requiredSize;
			getenv_s( &requiredSize, NULL, 0, "GRAPHITE_ROOT");
			char* environment_var = new char[requiredSize];
			getenv_s( &requiredSize, environment_var, requiredSize, "GRAPHITE_ROOT" );
			
			if( environment_var == nil ){
				Logger::err("FileManager") << "GRAPHITE_ROOT is not defined" <<std::endl;
				return project_root;
			}
			Logger::out("FileManager") 
				<< "Graphite is runned by another program. The path to Graphite directory is : " << environment_var << std::endl ;  
			project_root += environment_var; 
			ModuleManager::set_is_graphite_a_plugin( true );
		}

#else //WIN32
		project_root = PROJECT_ROOT;
#endif
		return project_root ;
	}


    FileManager* FileManager::instance_ = nil ;

    void FileManager::initialize() {
		std::string project_root = get_project_root() ;  // to set is_graphite_a_plugin before creating the FileManager
        instance_ = new FileManager() ;
#ifndef OGF_MINIMAL_KERNEL
        Environment::instance()->set_value("PROJECT_ROOT", project_root) ;
        Logger::out("FileManager") << "PROJECT_ROOT = " << project_root << std::endl ;  
#endif
    }

    void FileManager::terminate() {
        delete instance_ ;
        instance_ = nil ;
    }

    FileManager::FileManager() {
#ifdef WIN32
		char buff[2048] ;
		GetModuleFileName(GetModuleHandle("graphite.exe"), buff, 2048); 
		int len = strlen(buff) - 1;
		while (buff[len] != '\\')
			len--;
		buff[len+1] = 0;
        std::string cwd(buff) ;
		Logger::out("FileManager") << "current directory=" 
                                   << cwd << std::endl ;
    
		if( ModuleManager::is_graphite_a_plugin() ){
			/* In ogf_path_ the path to Graphite binaries must be available.
			 * Used by the find_binary_file function. */
			ogf_path_.push_back( ".." );
			ogf_path_.push_back( "." );
			ogf_path_.push_back( cwd ); // Not sure that this this is of any use
#ifdef _DEBUG
			ogf_path_.push_back( std::string(get_project_root() + "\\build\\Windows\\binaries\\bin\\debug") );
#else
			ogf_path_.push_back( std::string(get_project_root() + "\\build\\Windows\\binaries\\bin\\release") );
#endif
			ogf_path_.push_back( std::string(get_project_root() + "\\lib") );
		}
		else {
		std::string ogf_path = cwd  
            + ":.."
            + ":." 
			+ ":" + get_project_root() + "\\lib" ;
        String::split_string(ogf_path, ':', ogf_path_) ;
		}

#else
	const std::string project_root = get_project_root();
	const std::string binary_dir = BINARY_DIR;
	std::string ogf_path =  binary_dir + ":" +
				binary_dir + "/binaries/bin:" +
				binary_dir + "/binaries/lib:" +
				project_root + "/lib:" +
				project_root + "/src/bin/graphite/lib:" +
				project_root + "/src/bin/gomgen/lib:" +
				project_root + "/src/bin/gml/lib";

	String::split_string(ogf_path, ':', ogf_path_) ;
#endif //WIN32
    }


    bool FileManager::find_file(std::string& file_name, bool verbose) const {
        if(file_name.length() == 0) {
            return false ;
        }
        if(can_read_file(file_name)) {
            return true ;
        }
        for(
            std::vector<std::string>::const_iterator it = ogf_path_.begin() ;
            it != ogf_path_.end() ; it++
        ) {
#ifdef WIN32
            std::string cur_file_name = *it + "\\" + file_name ;
#else
            std::string cur_file_name = *it + "/" + file_name ;
#endif // WIN32
				if(can_read_file(cur_file_name)) {
                file_name = cur_file_name ;
                return true ;
            }
        }
        if(verbose) {
            Logger::err("FileManager") << "did not find file:" 
                                       << file_name << std::endl ;
        }
        return false ;
    }

    bool FileManager::find_binary_file(
        std::string& file_name, bool verbose
    ) const {
        for(
            std::vector<std::string>::const_iterator it = ogf_path_.begin() ;
            it != ogf_path_.end() ; it++
        ) {
#ifdef WIN32
            std::string cur_file_name = 
                *it + "\\" + file_name ;
#else
            std::string cur_file_name = 
                *it + "/binaries/" + file_name ;
#endif
           if(can_read_file(cur_file_name)) {
                file_name = cur_file_name ;
                return true ;
            }
        }
        if(verbose) {
            Logger::err("FileManager") << "did not find binary file:" 
                                       << file_name << std::endl ;
        }
        return false ;
    }

// Fabien 18/11/2003
    std::string FileManager::dll_prefix() const {
#ifdef WIN32
        return "" ;
#else
        return "lib" ;
#endif
    }

// Fabien 18/11/2003
    std::string FileManager::dll_extension() const {
#ifdef WIN32
        return ".dll" ;
#else
        return ".so" ;
#endif
    }


    bool FileManager::can_read_file(const std::string& file_name) const {
        std::ifstream test_stream(file_name.c_str()) ;
		return test_stream.is_open() ;
    }

    FileManager* FileManager::instance() {
        return instance_ ;
    }

    std::string FileManager::base_name(const std::string& path) const {
        return FileSystem::base_name(path) ;
    }

    std::string FileManager::dir_name(const std::string& path) const {
        return FileSystem::dir_name(path) ;
    }

    std::string FileManager::extension(const std::string& path) const {
        return FileSystem::extension(path) ;
    }


//_________________________________________________________

}

