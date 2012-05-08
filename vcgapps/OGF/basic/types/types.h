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
 
 
 

#ifndef __GNG_BASIC_TYPES_TYPES__
#define __GNG_BASIC_TYPES_TYPES__

#include <OGF/basic/common/common.h>
#include <string>
#include <vector>
#include <cmath>

#include <string.h>

#ifdef WIN32

#include <windows.h>
#ifdef min
#undef min
#endif
#ifdef max
#undef max
#endif

#else

#include <unistd.h>

#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace OGF {


//____________________________________________________________________________


/*
 * A Namespace gathering typedefs for memory management.
 */

    namespace Memory {

        typedef unsigned char  byte ;
        typedef unsigned char  word8 ;
        typedef unsigned short word16 ;
        typedef unsigned int   word32 ;

        typedef byte* pointer ;

        inline void clear(void* addr, size_t size) {
            ::memset(addr, 0, size) ;
        }
        
        inline void copy(void* to, const void* from, size_t size) {
            ::memcpy(to, from, size) ;
        }

    } 

//_______________________________________________________________________

/**
 * A namespace gathering typedefs
 * corresponding to numbers. These types
 * names have the form (u)int<size> or float<size>,
 * where the (optional) u denotes an unsigned type,
 * and the size is in bits.
 */

    namespace Numeric {

        typedef char          int8 ;
        typedef short         int16 ;
        typedef int           int32 ;

        typedef void*         pointer;

#ifdef WIN32
        typedef __int64 int64 ;
#else
        typedef long long int int64 ;	
#endif
		
        typedef unsigned char  uint8 ;
        typedef unsigned short uint16 ;
        typedef unsigned int   uint32 ;

#ifdef WIN32
        typedef unsigned __int64 uint64 ;
#else
        typedef unsigned long long int uint64 ;
#endif    

        typedef float  float32 ;
        typedef double float64 ;
   
        typedef float coord ;
        typedef float angle ;

        extern BASIC_API const float big_float ;
        extern BASIC_API const float small_float ;
        extern BASIC_API const double big_double ;
        extern BASIC_API const double small_double ;

        BASIC_API bool is_nan(float32 x) ;
        BASIC_API bool is_nan(float64 x) ;

        int32 BASIC_API random_int32() ;
        float32 BASIC_API random_float32() ;
        float64 BASIC_API random_float64() ;
    } 
    
//_______________________________________________________________________

    namespace String {

        void BASIC_API split_string(
            const std::string& in, 
            char separator,
            std::vector<std::string>& out,
            bool skip_empty_fields = true
        ) ;

        void BASIC_API join_strings(
            const std::vector<std::string>& in,
            char separator,
            std::string& out
        ) ;

        void BASIC_API join_strings(
            const std::vector<std::string>& in,
            const std::string& separator,
            std::string& out
        ) ;


        std::string BASIC_API join_strings(
            const std::vector<std::string>& in,
            char separator
        ) ;

        std::string BASIC_API join_strings(
            const std::vector<std::string>& in,
            const std::string& separator
        ) ;


        void BASIC_API to_lowercase(std::string& in) ;
        void BASIC_API to_uppercase(std::string& in) ;

        inline std::string char_to_string(char c) {
            char s[2] ;
            s[0] = c ;
            s[1] = '\0' ;
            return std::string(s) ;
        }

        std::string BASIC_API quote(const std::string& s, char quotes = '\"') ;

        bool BASIC_API string_starts_with(const std::string& haystack, const std::string& needle) ;
        bool BASIC_API string_ends_with(const std::string& haystack, const std::string& needle) ;
    } 

//_______________________________________________________________________

#define nil 0

//_______________________________________________________________________

    template <class T> inline T ogf_max(T x1, T x2) {
        return x1 > x2 ? x1 : x2;
    }

    template <class T> inline T ogf_min(T x1, T x2) {
        return x1 < x2 ? x1 : x2;
    }

    enum Sign { NEGATIVE=-1, ZERO=0, POSITIVE=1 } ;

    template <class T> inline Sign ogf_sgn(T x) {
        return (x > 0) ? POSITIVE : (
            (x < 0) ? NEGATIVE : ZERO
        );
    }

    template <class T> inline T ogf_abs(T x)  {
        return (x > 0) ? x : -x;
    }

    template <class T> inline T ogf_sqr(T x)  {
        return x*x;
    }

    template <class T> inline void ogf_clamp(T& x, T min, T max) {
        if(x < min) {
            x = min ;
        } else if(x > max) {
            x = max ;
        }
    }


    template <class T> inline void ogf_swap(T& x, T& y) {
        T z = x ;
        x = y ;
        y = z ;
    }

//____________________________________________________________________________

}

#endif
