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


#ifndef __nl_h__
#define __nl_h__

#define NL_USE_SUPERLU
#define NL_USE_TAUCS

#ifdef __cplusplus
extern "C" {
#endif

#define NL_VERSION_0_0 1

#define NLAPI
#define NLAPIENTRY

#ifdef NL_USE_SUPERLU_MT
#ifndef NL_USE_SUPERLU
#define NL_USE_SUPERLU
#endif
#endif

/*
 *
 * Datatypes
 *
 */

typedef unsigned int	NLenum;
typedef unsigned char	NLboolean;
typedef unsigned int	NLbitfield;
typedef void		NLvoid;
typedef signed char	NLbyte;		/* 1-byte signed */
typedef short		NLshort;	/* 2-byte signed */
typedef int		NLint;		/* 4-byte signed */
typedef unsigned char	NLubyte;	/* 1-byte unsigned */
typedef unsigned short	NLushort;	/* 2-byte unsigned */
typedef unsigned int	NLuint;		/* 4-byte unsigned */
typedef int		NLsizei;	/* 4-byte signed */
typedef float		NLfloat;	/* single precision float */
typedef double		NLdouble;	/* double precision float */

typedef void* NLContext ;

/*
 *
 * Constants
 *
 */

#define NL_FALSE   0x0
#define NL_TRUE    0x1

/* Primitives */

#define NL_SYSTEM  0x0
#define NL_MATRIX  0x1
#define NL_ROW     0x2


/* Solver Parameters */

#define NL_SOLVER           0x100
#define NL_NB_VARIABLES     0x101
#define NL_LEAST_SQUARES    0x102
#define NL_MAX_ITERATIONS   0x103
#define NL_THRESHOLD        0x104
#define NL_OMEGA            0x105
#define NL_SYMMETRIC        0x106
#define NL_USED_ITERATIONS  0x107
#define NL_ERROR            0x108
#define NL_INNER_ITERATIONS 0x109
#define NL_ELAPSED_TIME     0x10a
#define NL_PRECONDITIONER   0x10b
#define NL_MATRIX_STORE 0x10c
#define NL_RESIDUAL 0x10d

/* Matrix store */

#define NL_MATRIX_STORE_DEFAULT     0
#define NL_MATRIX_STORE_ROWS           1
#define NL_MATRIX_STORE_COLUMNS    2
#define NL_MATRIX_STORE_SYMMETRIC 4

/* Solvers */

#define NL_CG                    0x200
#define NL_BICGSTAB              0x201
#define NL_GMRES                 0x202
#define NL_SUPERLU_EXT           0x210
#define NL_PERM_SUPERLU_EXT      0x211
#define NL_SYMMETRIC_SUPERLU_EXT 0x212
#define NL_TAUCS_EXT             0x213
   
#define NL_SOLVER_USER 0x220

/* Preconditioners */

#define NL_PRECOND_NONE       0x000
#define NL_PRECOND_JACOBI     0x300
#define NL_PRECOND_SSOR       0x301
#define NL_PRECOND_USER       0x303

/* Enable / Disable */

#define NL_NORMALIZE_ROWS  0x400

/* Row parameters */

#define NL_RIGHT_HAND_SIDE 0x500
#define NL_ROW_SCALING     0x501

/* Functions */

#define NL_FUNC_SOLVER                  0x600
#define NL_FUNC_MATRIX                   0x601
#define NL_FUNC_PRECONDITIONER 0x602

/*
 * Contexts
 */
    NLAPI NLContext NLAPIENTRY nlNewContext() ;
    NLAPI void NLAPIENTRY nlDeleteContext(NLContext context) ;
    NLAPI void NLAPIENTRY nlMakeCurrent(NLContext context) ;
    NLAPI NLContext NLAPIENTRY nlGetCurrent() ;
    NLAPI NLboolean NLAPIENTRY nlInitExtension(char* extension) ;

/*
 * State set/get
 */

    NLAPI void NLAPIENTRY nlSolverParameterd(NLenum pname, NLdouble param) ;
    NLAPI void NLAPIENTRY nlSolverParameteri(NLenum pname, NLint param) ;

    NLAPI void NLAPIENTRY nlRowParameterd(NLenum pname, NLdouble param) ;
    NLAPI void NLAPIENTRY nlRowParameteri(NLenum pname, NLint param) ;

    NLAPI void NLAPIENTRY nlGetBooleanv(NLenum pname, NLboolean* params) ;
    NLAPI void NLAPIENTRY nlGetDoublev(NLenum pname, NLdouble* params) ;
    NLAPI void NLAPIENTRY nlGetIntergerv(NLenum pname, NLint* params) ;

    NLAPI void NLAPIENTRY nlEnable(NLenum pname) ;
    NLAPI void NLAPIENTRY nlDisable(NLenum pname) ;
    NLAPI NLboolean nlIsEnabled(NLenum pname) ;

/*
 * Functions
 */
    NLAPI void NLAPIENTRY nlSetFunction(NLenum pname, void* param) ;
    NLAPI void NLAPIENTRY nlGetFunction(NLenum pname, void** param) ;

/*
 * Variables
 */
    NLAPI void NLAPIENTRY nlSetVariable(NLuint index, NLdouble value) ;
    NLAPI NLdouble NLAPIENTRY nlGetVariable(NLuint index) ;
    NLAPI void NLAPIENTRY nlLockVariable(NLuint index) ;
    NLAPI void NLAPIENTRY nlUnlockVariable(NLuint index) ;
    NLAPI NLboolean NLAPIENTRY nlVariableIsLocked(NLuint index) ;

/*
 * Begin/End
 */

    NLAPI void NLAPIENTRY nlBegin(NLenum primitive) ;
    NLAPI void NLAPIENTRY nlEnd(NLenum primitive) ;
    NLAPI void NLAPIENTRY nlCoefficient(NLuint index, NLdouble value) ;

/*
 * Solve
 */

    NLAPI NLboolean NLAPIENTRY nlSolve() ;
    


#ifdef __cplusplus
}
#endif

#endif
