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
 
 
 

#include <OGF/basic/types/types.h>
#include <math.h>
#include <ctype.h>

#ifdef WIN32
#include <float.h> // for ::_isnan()
#else
#include <stdlib.h>
#endif


namespace OGF {

    namespace Numeric {
        const float big_float = 1e10f ;
        const float small_float = 1e-10f ;
        const double big_double = 1e20 ;
        const double small_double = 1e-20 ;

//   Note: do not add global scole (::isnan) for calling
// this function, since it is a macro in some implementations. 

        bool is_nan(float32 x) {
#ifdef WIN32
            return (_isnan(x) != 0) || (_finite(x) == 0) ;
#else
            return isnan(x) || !finite(x);
#endif
        }
    
        bool is_nan(float64 x) {
#ifdef WIN32
            return (_isnan(x) != 0) || (_finite(x) == 0) ;
#else
            return isnan(x) || !finite(x);
#endif
        }    

        int32 random_int32() {
#ifdef WIN32
            return rand() ;
#else
            return random() ;
#endif        
        }

        float32 random_float32() {
#ifdef WIN32
            return float(rand()) / float(RAND_MAX);
#else
            return float(drand48()) ;
#endif        
        }

        float64 random_float64() {
#ifdef WIN32
            return double(rand()) / double(RAND_MAX);
#else
            return double(drand48()) ;
#endif        
        }


    }

    namespace String {

        void split_string(
            const std::string& in, 
            char separator,
            std::vector<std::string>& out,
            bool skip_empty_fields
        ) {
            int length = in.length() ;
            int start = 0 ;
            int end = 0 ;
            
            while(start < length) {
                
                end = in.find(separator,start) ;
                if(end < 0) {
                    end = length ;
                }
                
                if(!skip_empty_fields || (end - start > 0)) {
                    out.push_back(in.substr(start, end - start)) ;
                }
                start = end + 1 ;
            }
        }
        

        void join_strings(
            const std::vector<std::string>& in,
            char separator,
            std::string& out
        ) {
            out = "" ;
            for(unsigned int i=0; i<in.size(); i++) {
                if(out.length() != 0) {
                    out += separator ;
                }
                out += in[i] ;
            }
        }

        void join_strings(
            const std::vector<std::string>& in,
            const std::string& separator,
            std::string& out
        ) {
            out = "" ;
            for(unsigned int i=0; i<in.size(); i++) {
                if(out.length() != 0) {
                    out += separator ;
                }
                out += in[i] ;
            }
        }

        std::string join_strings(
            const std::vector<std::string>& in,
            char separator
        ) {
            std::string result ;
            join_strings(in, separator, result) ;
            return result ;
        }

        std::string join_strings(
            const std::vector<std::string>& in,
            const std::string& separator
        ) {
            std::string result ;
            join_strings(in, separator, result) ;
            return result ;
        }

        void to_lowercase(std::string& in) {
            for(unsigned int i=0; i<in.length(); i++) {
                if(isupper(in[i])) {
                    in[i] = tolower(in[i]) ;
                }
            }
        }

        void to_uppercase(std::string& in) {
            for(unsigned int i=0; i<in.length(); i++) {
                if(islower(in[i])) {
                    in[i] = toupper(in[i]) ;
                }
            }
        }        

        std::string quote(const std::string& s, char quotes) {
            return char_to_string(quotes) + s + char_to_string(quotes) ;
        }


        bool string_starts_with(const std::string& haystack, const std::string& needle) {
            unsigned int l1 = haystack.length() ;
            unsigned int l2 = needle.length() ;
            if(l2 > l1) { return false ; }
            return (haystack.substr(0, l2) == needle) ;
        }
        
        bool string_ends_with(const std::string& haystack, const std::string& needle) {
            unsigned int l1 = haystack.length() ;
            unsigned int l2 = needle.length() ;
            if(l2 > l1) { return false ; }
            return (haystack.substr(l1 - l2, l2) == needle) ;
        }


    }

//____________________________________________________________________________

}
