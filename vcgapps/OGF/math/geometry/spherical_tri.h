/***************************************************************************
 * SphTri.h                                                                 *
 *                                                                          *
 * This file defines the SphericalTriangle class definition, which          *
 * supports member functions for Monte Carlo sampling, point containment,   *
 * and other basic operations on spherical triangles.                       *
 *                                                                          *
 *   Changes:                                                               *
 *     07/06/2004  levy  Maked it use Graphite's Points and Vectors         *
 *     01/01/2000  arvo  Added New_{Alpha,Beta,Gamma} methods.              *
 *     12/30/1999  arvo  Added VecIrrad method for "Vector Irradiance".     *
 *     04/08/1995  arvo  Further optimized sampling algorithm.              *
 *     10/11/1994  arvo  Added analytic sampling algorithm.                 *
 *     06/14/1994  arvo  Initial implementation.                            *
 *                                                                          *
 *--------------------------------------------------------------------------*
 * Copyright (C) 1995, 2000, James Arvo                                     *
 *                                                                          *
 * This program is free software; you can redistribute it and/or modify it  *
 * under the terms of the GNU General Public License as published by the    *
 * Free Software Foundation.  See http://www.fsf.org/copyleft/gpl.html      *
 *                                                                          *
 * This program is distributed in the hope that it will be useful, but      *
 * WITHOUT EXPRESS OR IMPLIED WARRANTY of merchantability or fitness for    *
 * any particular purpose.  See the GNU General Public License for more     *
 * details.                                                                 *
 *                                                                          *
 ***************************************************************************/
#ifndef __SPHTRI_INCLUDED__
#define __SPHTRI_INCLUDED__

#include <OGF/math/common/common.h>
#include <OGF/math/geometry/types.h>
#include <iostream>

namespace OGF {

/*
 *  The (Oblique) Spherical Triangle ABC.  Edge lengths (segments of great 
 *  circles) are a, b, and c.  The (dihedral) angles are Alpha, Beta, and Gamma.
 *
 *                      B
 *                      o
 *                     / \
 *                    /   \
 *                   /Beta \
 *                  /       \
 *               c /         \ a
 *                /           \ 
 *               /             \
 *              /               \
 *             /                 \
 *            /                   \
 *           /Alpha          Gamma \
 *          o-----------------------o
 *         A            b            C
 *
 */

    inline Vector3d Unit(const Vector3d& in) {
        Vector3d result = in ;
        result.normalize() ;
        return result ;
    }


    class SphericalTriangle {

    public: // methods
        SphericalTriangle() { Init(); }
        SphericalTriangle( const SphericalTriangle &T ) { *this = T; }
        SphericalTriangle( const Vector3d &, const Vector3d &, const Vector3d & );
        SphericalTriangle & operator()( 
            const Vector3d &, const Vector3d &, const Vector3d & 
        ) ;
        ~SphericalTriangle( ) {}
        void   operator=( const SphericalTriangle &T ) { *this = T; }
        /** Const-Jacobian map from square. */
        Vector3d   Chart    ( double x, double y ) const;  
        /** Get 2D coords of a point. */
        Point2d   Coord    ( const Vector3d &P    ) const;  
        int    Orient( ) const { return orient; }
        int    Inside( const Vector3d & ) const;
        double  SolidAngle() const { return area; }
        double  SignedSolidAngle() const { return -orient * area; } // CC is pos.
        Vector3d A()  const { return A_ ; }
        Vector3d B()  const { return B_ ; }
        Vector3d C()  const { return C_ ; }
        double  a()        const { return a_       ; }
        double  b()        const { return b_       ; }
        double  c()        const { return c_       ; }
        double  Cos_a()    const { return cos_a    ; }
        double  Cos_b()    const { return cos_b    ; }
        double  Cos_c()    const { return cos_c    ; }
        double  Alpha()    const { return alpha    ; }
        double  Beta ()    const { return beta     ; }
        double  Gamma()    const { return gamma    ; }
        double  CosAlpha() const { return cos_alpha; }
        double  CosBeta () const { return cos_beta ; }
        double  CosGamma() const { return cos_gamma; }
        Vector3d VecIrrad() const; // Returns the vector irradiance.
        SphericalTriangle Dual() const;
        SphericalTriangle New_Alpha( double alpha ) const;
        SphericalTriangle New_Beta ( double beta  ) const;
        SphericalTriangle New_Gamma( double gamma ) const;

    private: // methods
        void Init( );
        void Init( const Vector3d &A, const Vector3d &B, const Vector3d &C );

    private: // data
        Vector3d  A_, B_, C_, U;   // The vertices (and a temp vector).
        double a_, b_, c_;          // The edge lengths.
        double alpha, beta, gamma;  // The angles.
        double cos_a, cos_b, cos_c;
        double cos_alpha, cos_beta, cos_gamma;
        double area;
        double sin_alpha, product;  // Used in sampling algorithm.
        int   orient;              // Orientation.
    } ;

    inline double CosDihedralAngle( 
        const Vector3d &A, const Vector3d &B, const Vector3d &C 
    ) {
        double x = Unit(A ^ B) * Unit(C ^ B ) ;
        if( x < -1.0 ) x = -1.0;
        if( x >  1.0 ) x =  1.0;
        return x;
    }

    inline double DihedralAngle( 
        const Vector3d &A, const Vector3d &B, const Vector3d &C 
    ) {
        return acos( CosDihedralAngle( A, B, C ) );
    }

}

#endif




