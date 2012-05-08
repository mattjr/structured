/***************************************************************************
 * SphTri.C                                                                 *
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

#include <OGF/math/geometry/spherical_tri.h>

namespace OGF {

    // Remove component parallel to B.
    inline Vector3d operator/( 
        const Vector3d &A, const Vector3d &B 
    ) {
        Vector3d C ;  // Cumbersome due to compiler falure.
        double x = B.norm2() ;
        if( x > 0.0 ) {
            C = A - (( A * B ) / x) * B ; 
        } else {
            C = A ;
        }
        return C;
    }


    inline double Triple( const Vector3d &A, const Vector3d &B, const Vector3d &C ) {
        return ( A ^ B ) * C;
    }

    inline double ArcCos( double x ) {
        double y = 0.0 ;
        if( -1.0 <= x && x <= 1.0 ) y = acos( x );
        else if( x >  1.0 ) y = 0.0;
        else if( x < -1.0 ) y = M_PI ;
        return y;
    }

    


/*-------------------------------------------------------------------------*
 * Constructor                                                             *
 *                                                                         *
 * Construct a spherical triangle from three (non-zero) vectors.  The      *
 * vectors needn't be of unit length.                                      *
 *                                                                         *
 *-------------------------------------------------------------------------*/
    SphericalTriangle::SphericalTriangle( 
        const Vector3d &A0, const Vector3d &B0, const Vector3d &C0 
    ) {
        Init( A0, B0, C0 );
    }

/*-------------------------------------------------------------------------*
 * Init                                                                    *
 *                                                                         *
 * Construct the spherical triange from three vertices.  Assume that the   *
 * sphere is centered at the origin.  The vectors A, B, and C need not     *
 * be normalized.                                                          *
 *                                                                         *
 *-------------------------------------------------------------------------*/
    void SphericalTriangle::Init( 
        const Vector3d &A0, const Vector3d &B0, const Vector3d &C0 
    )
    {
        // Normalize the three vectors -- these are the vertices.

        A_ = Unit( A0 ) ;
        B_ = Unit( B0 ) ;
        C_ = Unit( C0 ) ;

        // Compute and save the cosines of the edge lengths.

        cos_a = B_ * C_;
        cos_b = A_ * C_;
        cos_c = A_ * B_;

        // Compute and save the edge lengths.

        a_ = ArcCos( cos_a );
        b_ = ArcCos( cos_b );
        c_ = ArcCos( cos_c );

        // Compute the cosines of the internal (i.e. dihedral) angles.

        cos_alpha = CosDihedralAngle( C_, A_, B_ );
        cos_beta  = CosDihedralAngle( A_, B_, C_ );
        cos_gamma = CosDihedralAngle( A_, C_, B_ );

        // Compute the (dihedral) angles.

        alpha = ArcCos( cos_alpha );
        beta  = ArcCos( cos_beta  );
        gamma = ArcCos( cos_gamma );

        // Compute the solid angle of the spherical triangle.

        area = alpha + beta + gamma - M_PI ;

        // Compute the orientation of the triangle.

		double mix = A_ * ( B_ ^ C_ ) ;
		orient = (mix > 0) ? 1 : ((mix < 0) ? -1 : 0) ;

        // Initialize three variables that are used for sampling the triangle.

        U         = Unit( C_ / A_ );  // In plane of AC orthogonal to A.
        sin_alpha = sin( alpha );
        product   = sin_alpha * cos_c;
    }

/*-------------------------------------------------------------------------*
 * Init                                                                    *
 *                                                                         *
 * Initialize all fields.  Create a null spherical triangle.               *
 *                                                                         *
 *-------------------------------------------------------------------------*/
    void SphericalTriangle::Init()
    {
        a_ = 0;  A_ = Vector3d(0,0,0);  cos_alpha = 0;  cos_a = 0;  alpha = 0;  
        b_ = 0;  B_ = Vector3d(0,0,0);  cos_beta  = 0;  cos_b = 0;  beta  = 0;  
        c_ = 0;  C_ = Vector3d(0,0,0);  cos_gamma = 0;  cos_c = 0;  gamma = 0;  
        area      = 0;
        orient    = 0;
        sin_alpha = 0;
        product   = 0;
        U         = Vector3d(0,0,0);
    }

/*-------------------------------------------------------------------------*
 * "( A, B, C )" operator.                                                 *
 *                                                                         *
 * Construct the spherical triange from three vertices.  Assume that the   *
 * sphere is centered at the origin.  The vectors A, B, and C need not     *
 * be normalized.                                                          *
 *                                                                         *
 *-------------------------------------------------------------------------*/
    SphericalTriangle & SphericalTriangle::operator()( 
        const Vector3d &A0, 
        const Vector3d &B0, 
        const Vector3d &C0 )
    {
        Init( A0, B0, C0 );
        return *this;
    }

/*-------------------------------------------------------------------------*
 * Inside                                                                  *
 *                                                                         *
 * Determine if the vector W is inside the triangle.  W need not be a      *
 * unit vector                                                             *
 *                                                                         *
 *-------------------------------------------------------------------------*/
    int SphericalTriangle::Inside( const Vector3d &W ) const
    {
        Vector3d Z = double(Orient()) * W ;
        if( Z * ( A() ^ B() ) < 0.0 ) return 0;
        if( Z * ( B() ^ C() ) < 0.0 ) return 0;
        if( Z * ( C() ^ A() ) < 0.0 ) return 0;
        return 1;
    }

/*-------------------------------------------------------------------------*
 * Chart                                                                   *
 *                                                                         *
 * Generate samples from the current spherical triangle.  If x1 and x2 are *
 * random variables uniformly distributed over [0,1], then the returned    *
 * points are uniformly distributed over the solid angle.                  *
 *                                                                         *
 *-------------------------------------------------------------------------*/
    Vector3d SphericalTriangle::Chart( double x1, double x2 ) const
    {
        // Use one random variable to select the area of the sub-triangle.
        // Save the sine and cosine of the angle phi.

        register double phi = x1 * area - Alpha();
        register double s   = sin( phi );
        register double t   = cos( phi );

        // Compute the pair (u,v) that determines the new angle beta.

        register double u = t - cos_alpha;
        register double v = s + product  ;  // sin_alpha * cos_c

        // Compute the cosine of the new edge b.

        double q = ( cos_alpha * ( v * t - u * s ) - v ) / 
            ( sin_alpha * ( u * t + v * s )     );

        // Compute the third vertex of the sub-triangle.

        Vector3d C_new = q * A() + ::sqrt( 1.0 - q * q ) * U;

        // Use the other random variable to select the height z.

        double z = 1.0 - x2 * ( 1.0 - C_new * B() );

        // Construct the corresponding point on the sphere.

        Vector3d D = C_new / B();  // Remove B component of C_new.
        return z * B() + ::sqrt( ( 1.0 - z * z ) / ( D * D ) ) * D;
    }

/*-------------------------------------------------------------------------*
 * Coord                                                                   *
 *                                                                         *
 * Compute the two coordinates (x1,x2) corresponding to a point in the     *
 * spherical triangle.  This is the inverse of "Chart".                    *
 *                                                                         *
 *-------------------------------------------------------------------------*/
    Point2d SphericalTriangle::Coord( const Vector3d &P1 ) const
    {
        Vector3d P = Unit( P1 );

        // Compute the new C vertex, which lies on the arc defined by B-P
        // and the arc defined by A-C.

        Vector3d C_new = Unit( ( B() ^ P ) ^ ( C() ^ A() ) );

        // Adjust the sign of C_new.  Make sure it's on the arc between A and C.

        if( C_new * ( A() + C() ) < 0.0 ) C_new = -1.0 * C_new;

        // Compute x1, the area of the sub-triangle over the original area.

        double cos_beta  = CosDihedralAngle( A(), B(), C_new  );
        double cos_gamma = CosDihedralAngle( A(), C_new , B() );
        double sub_area  = Alpha() + acos( cos_beta ) + acos( cos_gamma ) - M_PI;
        double x1        = sub_area / SolidAngle();

        // Now compute the second coordinate using the new C vertex.

        double z  = P * B();
        double x2 = ( 1.0 - z ) / ( 1.0 - C_new * B() );

        if( x1 < 0.0 ) x1 = 0.0;  if( x1 > 1.0 ) x1 = 1.0;
        if( x2 < 0.0 ) x2 = 0.0;  if( x2 > 1.0 ) x2 = 1.0;
        return Point2d( x1, x2 );
    }

/*-------------------------------------------------------------------------*
 * Dual                                                                    *
 *                                                                         *
 * Construct the dual triangle of the current triangle, which is another   *
 * spherical triangle.                                                     *
 *                                                                         *
 *-------------------------------------------------------------------------*/
    SphericalTriangle SphericalTriangle::Dual() const
    {
        Vector3d dual_A = B() ^ C();  if( dual_A * A() < 0.0 ) dual_A = -1.0 * dual_A ;
        Vector3d dual_B = A() ^ C();  if( dual_B * B() < 0.0 ) dual_B = -1.0 * dual_B ;
        Vector3d dual_C = A() ^ B();  if( dual_C * C() < 0.0 ) dual_C = -1.0 * dual_C ;
        return SphericalTriangle( dual_A, dual_B, dual_C );
    }

/*-------------------------------------------------------------------------*
 * VecIrrad                                                                *
 *                                                                         *
 * Return the "vector irradiance" due to a light source of unit brightness *
 * whose spherical projection is this spherical triangle.  The negative of *
 * this vector dotted with the surface normal gives the (scalar)           *
 * irradiance at the origin.                                               *
 *                                                                         *
 *-------------------------------------------------------------------------*/
    Vector3d SphericalTriangle::VecIrrad() const
    {
        Vector3d Phi =
            a() * Unit( B() ^ C() ) +
            b() * Unit( C() ^ A() ) +
            c() * Unit( A() ^ B() ) ;
        if( Orient() ) Phi = -1.0 * Phi ;
        return Phi;    
    }

/*-------------------------------------------------------------------------*
 * New_Alpha                                                               *
 *                                                                         *
 * Returns a new spherical triangle derived from the original one by       *
 * moving the "C" vertex along the edge "BC" until the new "alpha" angle   *
 * equals the given argument.                                              *
 *                                                                         *
 *-------------------------------------------------------------------------*/
    SphericalTriangle SphericalTriangle::New_Alpha( double alpha ) const
    {
        Vector3d V1( A() ), V2( B() ), V3( C() );
        Vector3d E1 = Unit( V2 ^ V1 );
        Vector3d E2 = E1 ^ V1;
        Vector3d G  = ( cos(alpha) * E1 ) + ( sin(alpha) * E2 );
        Vector3d D  = Unit( V3 / V2 );
        Vector3d C2 = ((G * D) * V2) - ((G * V2) * D);
        if( Triple( V1, V2, C2 ) > 0.0 ) C2 = -1.0 * C2 ;
        return SphericalTriangle( V1, V2, C2 );
    }

}

