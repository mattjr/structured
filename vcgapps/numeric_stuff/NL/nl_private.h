/*
 *  OpenNL: Numerical Library
 *  Copyright (C) 2004 Bruno Levy
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

#ifndef __NL_PRIVATE__
#define __NL_PRIVATE__

#include <numeric_stuff/NL/nl.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>


/*******************************************************************************************************/
/*** Assertion checks ***/
/*******************************************************************************************************/
void nl_assertion_failed(char* cond, char* file, int line) ;
void nl_range_assertion_failed(
    double x, double min_val, double max_val, char* file, int line
) ;
void nl_should_not_have_reached(char* file, int line) ;

#define nl_assert(x) {                                        \
    if(!(x)) {                                                  \
        nl_assertion_failed(#x,__FILE__, __LINE__) ;          \
    }                                                           \
} 

#define nl_range_assert(x,min_val,max_val) {                  \
    if(((x) < (min_val)) || ((x) > (max_val))) {                \
        nl_range_assertion_failed(x, min_val, max_val,        \
            __FILE__, __LINE__                                  \
        ) ;                                                     \
    }                                                           \
}

#define nl_assert_not_reached {                               \
    nl_should_not_have_reached(__FILE__, __LINE__) ;          \
}

#ifdef NL_DEBUG
#define nl_debug_assert(x) nl_assert(x)
#define nl_debug_range_assert(x,min_val,max_val) nl_range_assert(x,min_val,max_val)
#else
#define nl_debug_assert(x) 
#define nl_debug_range_assert(x,min_val,max_val) 
#endif

#ifdef NL_PARANOID
#define nl_parano_assert(x) nl_assert(x)
#define nl_parano_range_assert(x,min_val,max_val) nl_range_assert(x,min_val,max_val)
#else
#define nl_parano_assert(x) 
#define nl_parano_range_assert(x,min_val,max_val) 
#endif

/*******************************************************************************************************/
/*** Error reporting ***/
/*******************************************************************************************************/

void nlError(char* function, char* message) ;
void nlWarning(char* function, char* message) ;

/*******************************************************************************************************/
/*** OS ***/
/*******************************************************************************************************/

NLdouble nlCurrentTime()  ;

/************************************************************************************/
/* classic macros */

#ifndef MIN
#define MIN(x,y) (((x) < (y)) ? (x) : (y)) 
#endif

#ifndef MAX
#define MAX(x,y) (((x) > (y)) ? (x) : (y)) 
#endif

/************************************************************************************/
/* Memory management */
/************************************************************************************/

#define NL_NEW(T)                               (T*)(calloc(1, sizeof(T))) 
#define NL_NEW_ARRAY(T,NB)           (T*)(calloc((NB),sizeof(T))) 
#define NL_RENEW_ARRAY(T,x,NB)   (T*)(realloc(x,(NB)*sizeof(T))) 
#define NL_DELETE(x)                          free(x); x = NULL 
#define NL_DELETE_ARRAY(x)            free(x); x = NULL

#define NL_CLEAR(x)                       memset(x, 0, sizeof(*x)) 
#define NL_CLEAR_ARRAY(x,NB)   memset(x, 0, (NB)*sizeof(*x)) 

#endif
