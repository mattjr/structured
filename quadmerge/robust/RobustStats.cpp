//============================================================================
//
//  This file is part of GPSTk, the GPS Toolkit.
//
//  The GPSTk is free software; you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License as published
//  by the Free Software Foundation; either version 2.1 of the License, or
//  any later version.
//
//  The GPSTk is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public
//  License along with GPSTk; if not, write to the Free Software Foundation,
//  Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//  
//  Copyright 2004, The University of Texas at Austin
//
//============================================================================

/**
 * @file RobustStats.cpp
 * Namespace Robust includes basic robust statistical computations, including median,
 * median average deviation, quartiles and m-estimate, as well as implementation of
 * of stem-and-leaf plots, quantile plots and robust least squares estimation of a
 * polynomial.
 * Reference: Mason, Gunst and Hess, "Statistical Design and
 *            Analysis of Experiments," Wiley, New York, 1989.
 */
 
//------------------------------------------------------------------------------------
// GPSTk includes
#include "Exception.hpp"
#include "StringUtils.hpp"
#include "Matrix.hpp"
#include "RobustStats.hpp"

//------------------------------------------------------------------------------------
// moved to RobustStats.hpp as macros
//const double gpstk::Robust::TuningT=1.5;      // or 1.345;       // or 1.5
//const double gpstk::Robust::TuningA=0.778;    // or 0.67;        // or 0.778
//const double gpstk::Robust::TuningE=0.6745;

//------------------------------------------------------------------------------------
using namespace std;
using namespace gpstk;
using namespace StringUtils;

//------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------

template <typename T>
void Robust::QuantilePlot(T *yd, long nd, T *xd)
   throw(Exception)
{
   if(!xd || nd<2 || !yd) {
      Exception e("Invalid input");
      GPSTK_THROW(e);
   }

   double f;
   for(int i=0; i<nd; i++) {
      f = double(8*i+5)/double(8*nd+2);         // f(i) = i-3/8 / n+1/4, i=1,n
      xd[i] = 4.91*(::pow(f,0.14) - ::pow(1-f,0.14));
   }

}  // end QuantilePlot


int Robust::RobustPolyFit(double *xd, const double *td, int nd,
                          int N, double *c, double *w)
   throw(Exception)
{
   try {
      if(!xd || !td || !c || nd < 2) {
         Exception e("Invalid input");
         GPSTK_THROW(e);
      }

      int i,j,niter;
      double x0=xd[0],t0=td[0],mad,median,conv,conv_limit=::sqrt(double(nd))*1.e-3;
      Matrix<double> PT,P(nd,N,1.0),Cov;
      Vector<double> Wts(nd,1.0), Coeff(N,0.0), D(nd), Res, ResCopy;

      // build the data vector and the (constant) partials matrix
      for(i=0; i<nd; i++) {
         D(i) = xd[i]-x0;
         for(j=1; j<N; j++)
            P(i,j) = P(i,j-1)*(td[i]-t0);
      }

      // iterate until weights don't change
      niter = 0;
      while(1) {
         // compute partials transpose multiplied by 'weight matrix'=diag(squared wts)
         PT = transpose(P);
         for(i=0; i<N; i++)
            for(j=0; j<nd; j++)
               PT(i,j) *= Wts(j)*Wts(j);
         Cov = PT * P;        // information matrix

         // solve
         try { Cov = inverse(Cov); }
         catch(Exception& e) { return -1; }
         Coeff = Cov * PT * D;

         // compute residuals
         ResCopy = Res = D - P*Coeff;

         // compute median and MAD. NB Median() will sort the vector...
         mad = MedianAbsoluteDeviation(&(ResCopy[0]),ResCopy.size(),median);

         // recompute weights
         Vector<double> OldWts(Wts);
         for(i=0; i<nd; i++) {
            if(Res(i) < -RobustTuningT*mad)
               Wts(i) = -RobustTuningT*mad/Res(i);
            else if(Res(i) > RobustTuningT*mad)
               Wts(i) = RobustTuningT*mad/Res(i);
            else
               Wts(i) = 1.0;
         }

         // test for convergence
         if(++niter > 20) return -2;
         conv = RMS(OldWts - Wts);
         if(conv > 1.) return -3;
         if(niter > 2 && conv < conv_limit) break;
      }

      // copy out weights, residuals and solution
      for(i=0; i<N; i++) c[i] = Coeff(i);
      //c[0] += x0;
      for(i=0; i<nd; i++) {
         xd[i] = Res(i);
         if(w) w[i] = Wts(i);
      }

      return 0;
   }
   catch(Exception& e) { GPSTK_RETHROW(e); }
}

//------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------
