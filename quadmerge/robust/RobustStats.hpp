

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
 * @file RobustStats.hpp
 * Namespace Robust includes basic robust statistical computations, including median,
 * median average deviation, quartiles and m-estimate, as well as implementation of
 * of stem-and-leaf plots, quantile plots and robust least squares estimation of a
 * polynomial.
 * Reference: Mason, Gunst and Hess, "Statistical Design and
 *            Analysis of Experiments," Wiley, New York, 1989.
 */
 
#ifndef GPSTK_ROBUSTSTATS_HPP
#define GPSTK_ROBUSTSTATS_HPP

//------------------------------------------------------------------------------------
// system includes
#include <string>

// GPSTk
#include "Exception.hpp"
#include "StringUtils.hpp"
namespace gpstk {
//------------------------------------------------------------------------------------
#define ABSOLUTE(x) ((x) < T() ? -(x) : (x))
/// tuning constant used in M-estimate and Robust least squares (SRIFilter.cpp)
#define RobustTuningT (1.5)         // or 1.345
/// tuning constant used in robust estimate of variance
#define RobustTuningA (0.778)       // or 0.67
/// tuning constant used in MAD
#define RobustTuningE (0.6745)

//------------------------------------------------------------------------------------
   /** @addtogroup math */
   //@{

   //--------------------------------------------------------------------------------
   // quick sort, for use by robust statistics routines

   /// Comparison function for sorting.
   /// Default comparision function int comp(T a, T b) returns
   /// 1 if a > b, -1 if a < b, and 0 if a==b. A user defined comparison
   /// function may be passed as a calling argument to the sort routines.
   /// @param a and b objects of type T to be compared
   /// @return 1 if a > b, -1 if a < b, or 0 if a==b.
   template <typename T>
   int Qsort_compare(const T& a, const T& b) {
      if(a > b) return 1;
      else if(a < b) return -1;
      else return 0;
   }
   
   /// Insert sort. operator>() and operator<() must be defined for T,
   /// and a user comparison function comp(T,T) may be passed to override
   /// the default Qsort_compare().
   /// @param sa is the array of type T to be sorted.
   /// @param na length of the array to be sorted.
   template <typename T>
   void insert(T *sa,
               int na,
               int (*comp)(const T&, const T&) = gpstk::Qsort_compare)
   {
      int i,j;
      T stemp;
      for(i=1; i < na; i=i+1) { // insert the i-th element into the sorted array
         stemp = sa[i];
         j = i - 1;             // find where it goes
         while((j >= 0) && (comp(stemp,sa[j]) < 0)) { 
            sa[j+1] = sa[j];
            j = j - 1;
         }
         sa[j+1] = stemp;
      }
   }  // end insert sort

   /// Quick sort in memory, with insert sort for small arrays.
   /// operator>() and operator<() must be defined for T,
   /// and a user comparison function comp(T,T) may be passed to
   /// override the default Qsort_compare().
   /// @param sa is the array of type T to be sorted.
   /// @param na length of the array to be sorted.
   /// @param comp (optional) the comparison function to be used.
   template <typename T>
   void QSort(T *sa,
              int na,
              int (*comp)(const T&, const T&) = gpstk::Qsort_compare)
   {
      int i,j,nr;
      T stemp, spart;
      while(1) { 
         if(na < 8) {                     // use insert sort for small arrays
            insert(sa,na,comp);
            return;
         }
         spart = sa[na/2];                // pick middle element as pivot
         i = -1; 
         j = na;
         while(1) {
            do {                          // find first element to move right
               i = i + 1;
            } while(comp(sa[i],spart) < 0);
            do {                          // find first element to move left
               j = j - 1;
            } while(comp(sa[j], spart) > 0);
                                          // if the boundaries have met,
                                          // through paritioning,
            if(i >= j) break;
                                          // swap i and j elements
            stemp = sa[i];
            sa[i] = sa[j];
            sa[j] = stemp;
         }
         nr = na - i;
         if(i < (na/2)) {                 // now sort each partition
            QSort(sa, i, comp);           // sort left side 
            sa = &sa[i];                  // sort right side here
            na = nr;                      // memsort(&(sa[i]),nr,comp);
         }
         else { 
            QSort(&(sa[i]), nr, comp);    // sort right side
            na = i;
         }
      }
   }  // end QSort
   
   /// Insert sort one vector, keeping a second parallel.
   /// See the single-vector version of insert.
   /// @param sa is the array of type T to be sorted.
   /// @param pa is the array of type S to be kept parallel to the first.
   /// @param na length of the array to be sorted.
   template <typename T, typename S>
   void insert(T *sa,
               S *pa,
               int na,
               int (*comp)(const T&, const T&) = gpstk::Qsort_compare)
   {
      int i,j;
      T stemp;
      S ptemp;
      for(i=1; i < na; i=i+1) { // insert the i-th element into the sorted array
         stemp = sa[i];
         ptemp = pa[i];
         j = i - 1;             // find where it goes
         while((j >= 0) && (comp(stemp,sa[j]) < 0)) { 
            sa[j+1] = sa[j];
            pa[j+1] = pa[j];
            j = j - 1;
         }
         sa[j+1] = stemp;
         pa[j+1] = ptemp;
      }
   }  // end insert sort

   /// Quick sort of one vector, keeping another parallel.
   /// See the single-vector version of QSort.
   /// @param sa is the array of type T to be sorted.
   /// @param pa is the array of type S to be kept parallel to the first.
   /// @param na length of the array to be sorted.
   /// @param comp (optional) the comparison function to be used.
   template <typename T, typename S>
   void QSort(T *sa,
              S *pa,
              int na,
              int (*comp)(const T&, const T&) = gpstk::Qsort_compare)
   {
      int i,j,nr;
      T stemp, spart;
      S ptemp, ppart;
      while(1) { 
         if(na < 8) {                     // use insert sort for small arrays
            insert(sa,pa,na,comp);
            return;
         }
         spart = sa[na/2];                // pick middle element as pivot
         ppart = pa[na/2];
         i = -1; 
         j = na;
         while(1) {
            do {                          // find first element to move right
               i = i + 1;
            } while(comp(sa[i],spart) < 0);
            do {                          // find first element to move left
               j = j - 1;
            } while(comp(sa[j], spart) > 0);
                                          // if the boundaries have met,
                                          // through paritioning,
            if(i >= j) break;
                                          // swap i and j elements
            stemp = sa[i]; ptemp = pa[i];
            sa[i] = sa[j]; pa[i] = pa[j];
            sa[j] = stemp; pa[j] = ptemp;
         }
         nr = na - i;
         if(i < (na/2)) {                 // now sort each partition
            QSort(sa, pa, i, comp);       // sort left side 
            sa = &sa[i];                  // sort right side here
            pa = &pa[i];
            na = nr;                      // memsort(&(sa[i]),nr,comp);
         }
         else { 
            QSort(&(sa[i]), &(pa[i]), nr, comp);    // sort right side
            na = i;
         }
      }
   }  // end QSort
   
   //--------------------------------------------------------------------------------
   /// Robust statistics.
   namespace Robust
   {
   /// Compute median of an array of length nd;
   /// array xd is returned sorted, unless save_flag is true.
   /// @param xd         array of data.
   /// @param nd         length of array xd.
   /// @param save_flag  if true (default), array xd will NOT be trashed.
   /// @return median of the data in array xd.
   template <typename T>
   T Median(T *xd, const int nd, bool save_flag=true)
      throw(Exception)
   {
      if(!xd || nd < 2) {
         Exception e("Invalid input");
         GPSTK_THROW(e);
      }

      try {
         int i;
         T med, *save;

         if(save_flag) {
            save = new T[nd];
            if(!save) {
               Exception e("Could not allocate temporary array");
               GPSTK_THROW(e);
            }
            for(i=0; i<nd; i++) save[i]=xd[i];
         }

         QSort(xd,nd);

         if(nd%2)
            med = xd[(nd+1)/2-1];
         else
            med = (xd[nd/2-1]+xd[nd/2])/T(2);

            // restore original data from temporary
         if(save_flag) {
            for(i=0; i<nd; i++) xd[i]=save[i];
            delete[] save;
         }

         return med;
      }
      catch(Exception& e) { GPSTK_RETHROW(e); }

   }  // end Median

   /// Compute the quartiles Q1 and Q3 of an array of length nd.
   /// Array is assumed sorted in ascending order.
   /// Quartile are values such that one fourth of the
   /// samples are larger (smaller) than Q3(Q1).
   /// @param xd array of data.
   /// @param nd length of array xd.
   /// @param Q1 (output) first quartile of data in array xd.
   /// @param Q3 (output) third quartile of data in array xd.
   template <typename T>
   void Quartiles(const T *xd, const int nd, double& Q1, double& Q3)
      throw(Exception)
   {
      if(!xd || nd < 2) {
         Exception e("Invalid input");
         GPSTK_THROW(e);
      }

      int q;
      if(nd % 2) q = (nd+1)/2;
      else       q = nd/2;

      if(q % 2) {
         Q1 = xd[(q+1)/2-1];
         Q3 = xd[nd-(q+1)/2];
      }
      else {
         Q1 = (xd[q/2-1]+xd[q/2])/T(2);
         Q3 = (xd[nd-q/2]+xd[nd-q/2-1])/T(2);
      }
   }  // end Quartiles

   /// Compute the median absolute deviation of a double array of length nd,
   /// as well as the median (M = Median(xd,nd));
   /// NB this routine will trash the array xd unless save_flag is true (default).
   /// @param xd array of data (input).
   /// @param nd length of array xd (input).
   /// @param M median of data in array xd (output).
   /// @param save_flag if true (default), array xd will NOT be trashed.
   /// @return median absolute deviation of data in array xd.
   template <typename T>
   T MedianAbsoluteDeviation(T *xd, int nd, T& M, bool save_flag=true)
      throw(Exception)
   {
      int i;
      T mad, *save;

      if(!xd || nd < 2) {
         Exception e("Invalid input");
         GPSTK_THROW(e);
      }

         // store data in a temporary array
      if(save_flag) {
         save = new T[nd];
         if(!save) {
            Exception e("Could not allocate temporary array");
            GPSTK_THROW(e);
         }
         for(i=0; i<nd; i++) save[i]=xd[i];
      }

         // get the median (don't care if xd gets sorted...)
      M = Median(xd, nd, false);

         // compute xd=abs(xd-M)
      for(i=0; i<nd; i++) xd[i] = ABSOLUTE(xd[i]-M);

         // sort xd in ascending order
      QSort(xd, nd);

         // find median and normalize to get mad
      mad = Median(xd, nd, false) / T(RobustTuningE);

         // restore original data from temporary
      if(save_flag) {
         for(i=0; i<nd; i++) xd[i]=save[i];
         delete[] save;
      }

      return mad;

   }  // end MedianAbsoluteDeviation

      /// Compute the median absolute deviation of a double array of length nd;
      /// see MedianAbsoluteDeviation().
   template <typename T>
   T MAD(T *xd, int nd, T& M, bool save_flag=true)
      throw(Exception)
   { return MedianAbsoluteDeviation(xd,nd,M,save_flag); }

   /// Compute the m-estimate. Iteratively determine the m-estimate, which
   /// is a measure of mean or median, but is less sensitive to outliers.
   /// M is the median (M=Median(xd,nd)), and MAD is the
   /// median absolute deviation (MAD=MedianAbsoluteDeviation(xd,nd,M)).
   /// Optionally, a pointer to an array, which will contain the weights
   /// on output, may be provided.
   /// @param xd input array of data.
   /// @param nd input length of array xd.
   /// @param M input median of data in array xd.
   /// @param MAD input median absolute deviation of data in array xd.
   /// @param w output array of length nd to contain weights on output.
   /// @return m-estimate of data in array xd.
   template <typename T>
   T MEstimate(T *xd, int nd, const T& M, const T& MAD, T *w=NULL)
      throw(Exception)
   {
      try {
         T tv, m, mold, sum, sumw, *wt, weight;
         T tol=0.000001;
         int i, n, N=10;      // N is the max number of iterations

         if(!xd || nd < 2) {
            Exception e("Invalid input");
            GPSTK_THROW(e);
         }

         tv = T(RobustTuningT)*MAD;
         n = 0;
         m = M;
         do {
            mold = m;
            n++;
            sum = sumw = T();
            for(i=0; i<nd; i++) {
               if(w) wt=&w[i];
               else wt=&weight;
               *wt = T(1);
               if(xd[i]<m-tv)      *wt = -tv/(xd[i]-m);
               else if(xd[i]>m+tv) *wt =  tv/(xd[i]-m);
               sumw += (*wt);
               sum += (*wt)*xd[i];
            }
            m = sum / sumw;

         } while(T(ABSOLUTE((m-mold)/m)) > tol && n < N);

         return m;
      }
      catch(Exception& e) { GPSTK_RETHROW(e); }

   }  // end MEstimate

   /// Fit a polynomial of degree n to data xd, with independent variable td,
   /// using robust techniques. The post-fit residuals are returned in the data
   /// vector, and the computed weights in the result may be output as well.
   /// Specifically, the equation describing the fit is
   /// c0 + c[1]*t(j) + c[2]*t(j)*t(j) + ... c[n-1]*pow(t(j),n-1) = xd[j],
   /// where the zero-th coefficient and the independent variable are debiased
   /// by the first value; i.e. c0 = c[0]+xd[0] and t(j) = td[i]-td[0].
   /// Specifically, to evaluate the polynomial at t, eval = f(t), do the following.
   /// xd0 = xd[0];
   /// Robust::RobustPolyFit(xd,td,nd,n,c);
   /// eval = xd0+c[0]; tt = 1.0;
   /// for(j=1; j<nd; j++) { tt *= (t-td[0]); eval += c[j]*tt; }
   /// @param xd (input) array of data, of length nd; contains residuals on output.
   /// @param td (input) array of independent variable, length nd (parallel to xd).
   /// @param nd (input) length of arrays xd and td.
   /// @param n (input) degree of polynomial and dimension of coefficient array.
   /// @param c (output) array of coefficients (dimension n).
   /// @param w (output, if non-null) array of length nd to contain weights.
   /// @return 0 for success, -1 for singular problem, -2 failure to converge.
   int RobustPolyFit(double *xd, const double *td, int nd,
                     int n, double *c, double *w=NULL)
      throw(Exception);

   /// Print 'stem and leaf' plot of the data in the double array xd of length nd,
   /// with an optional message, on the given output ostream. It is assumed that
   /// the input array xd is sorted in ascending order.
   /// @param os ostream on which to write output.
   /// @param xd array of data.
   /// @param nd length of array xd.
   /// @param msg string containing optional message for output.
     /*  template <typename T>
    void StemLeafPlot(std::ostream& os, T *xd, long nd,
         std::string msg=std::string(""))
	 throw(Exception);*/
inline long Stem(double x, double& scale) { return (long(x/scale)); }

     template <typename T>
     void StemLeafPlot(std::ostream& os, T *xd, long nd, std::string msg)
       throw(Exception){
   long stem, l, nout=0, s, sM, sQ1, sQ3, sOH, sOL;
   int i, sign, pos, k, leaf;
   unsigned len=0, kk;
   char c;
   double x,scale;
   std::string buf;

   if(!xd || nd < 2) GPSTK_THROW(Exception("Invalid input"));

      // find range 
   double range = xd[nd-1] - xd[0];                   // max - min
   if(range < 0.0)
      GPSTK_THROW(Exception("Array is not sorted"));
   if(range == 0.0)
      range = xd[0]; //GPSTK_THROW(Exception("Array has zero range"));

      // find scale
   scale = 0.0;
   short nscale=0;       // scale = 1*10^nscale
   if(range >= 10.0)
      sign = 1;
   else if(range < 1.0)
      sign = -1;
   else
      scale = 1.0;

   if(!scale) do {
      nscale += sign;
   } while(range*::pow(10.0,-nscale) < 1.0 || range*::pow(10.0,-nscale) >= 10.0);

   scale = ::pow(10.0,nscale);

   double M=Robust::Median(xd,nd);
   double Q1,Q3;
   Quartiles<T>(xd,nd,Q1,Q3);
      // outlier limits
   double OH=2.5*Q3-1.5*Q1;         // outlier high limit
   double OL=2.5*Q1-1.5*Q3;         // outlier low limit ('oh L' not 'zero L')

      // starting stem=stem(min=xd[0]), and stem step==scale
   i = 1+short((range/scale)+0.5);              // number of stems
   if(xd[0]*xd[nd-1] < 0.0) i++;                // add one stem for zero
   if(nd > 8 && i < 8 && xd[nd-1] != xd[0]) {   // fudge so #stems is big enough
      scale /= 10.0;
      nscale--;
   }

      // find length of stem for printing
   buf = StringUtils::asString<long>(::abs(Stem(xd[0],scale)));
   len = buf.size();
   buf = StringUtils::asString<long>(::abs(Stem(xd[nd-1],scale)));
   if(len < buf.size()) len=buf.size();
   buf = StringUtils::asString<long>(::abs(Stem(M,scale)));
   if(len < buf.size()) len=buf.size();

      // loop through data, adding stems and leaves to plot
   bool start=true;
   if(xd[0] < 0.0) pos=-1; else pos=1;
   sM = Stem(M,scale);
   sQ1 = Stem(Q1,scale);
   sQ3 = Stem(Q3,scale);
   sOH = Stem(OH,scale);
   sOL = Stem(OL,scale);
   for(l=0; l<nd; l++) {
         // current: stem=s,pos; data=stem,sign(xd[l])
      if(xd[l]>OH || xd[l]<OL) nout++;                   // count outliers
      sign = 1;
      if(xd[l] < 0.0) sign=-1;
      stem = Stem(fabs(xd[l]),scale);
      x = 10*fabs(xd[l]/scale-sign*stem);
      leaf = short(x + 0.5);
      if(leaf == 10) {
         stem++;
         leaf=0;
      }
      stem = sign*stem;

      // print it
      if(start || s!=stem || (s==0 && pos*sign<0.0)) {
         // Change of stem -> print
         if(start) {                                        // first time through
            os << "Stem and Leaf Plot (scale ";             // print scale
            i=nscale;
            if(nscale < 0) {
               os << "0.";
               i++;
               k = 1;
            }
            else {
               os << "1";
               k = -1;
            }
            while(i != 0) {
               os << "0";
               i += k;
            }
            if(nscale < 0)
               os << "1";
            else
               os << ".0";
            os << ", " << nd << "pts) : ";                  // print npts
            if(msg.size() > 0) os << msg;                   // and message
            s = stem - 1;                                   // save for later
            start = false;
         }

         while(s < stem || (s==0 && pos*sign<0)) { // also print stems without leaves
            if(s != 0) s++;
            else if(pos < 0) pos = 1;
            else s++;

               // print the new line with stem s
            os << "\n";
            buf = StringUtils::asString<long>(s < 0 ? -s : s); // abs(stem)

            //for(kk=buf.size(); kk<len; kk++) os << " ";
            //if(s<0) c='-'; else if(s>0) c='+'; else if(pos>0)c='+'; else c='-';
            //os << c;
            //os << buf << " ";

            if(s<0) c='-';                                     // sign of stem
            else if(s>0) c='+';
            else if(pos>0)c='+';
            else c='-';
            os << c;

            for(kk=buf.size(); kk<len; kk++) os << " ";        // pad out to length
            os << buf << " ";                                  // stem/axis space

               // now print either |, M (median), Q (quartiles), or >< (outliers)
            k=0;

               // if s==sM
            if(s==sM && (s!=0 || pos*M>0.0)) {
               os << "M";                       // marks the median
               k++;
            }

            if((s == sQ3 && (s != 0 || pos*Q3 < 0.0)) ||
                  (s == sQ1 && (s != 0 || pos*Q1 < 0.0) )) {
               os << "Q";                       // marks a quartile
               k++;
            }

            if((s < sOL || (s == 0 && sOL == 0 && (pos == -1 && OL > 0.0))) ||
               (s == sOL && (s != 0 || pos*OL > 0.0))) {
               os << "<";                       // marks an outlier (small)
               k++;
            }
            else if((s > sOH || (s == 0 && sOH == 0 && (pos == 1 && OH < 0.0))) ||
               (s == sOH && (s != 0 || pos*OH > 0.0))) {
               os << ">";                       // marks an outlier (big)
               k++;
            }

            if(k == 0) {
               os << "|";                       // marks a regular point
               k++;
            }

            while(k < 3) {
               os << " ";
               k++;
            }
         }
      }     // end change stem

         // print leaf
      os << leaf;
   }

   os << "\nEND Stem and Leaf Plot (there are " << nout << " outliers.)\n";

}  // end StemLeafPlot


   /// Generate data for a quantile-quantile plot. Given an array of data yd,
   /// of length nd (sorted in ascending order), and another array xd of the
   /// same length, fill the xd array with data such that (xd,yd) give a
   /// quantile-quantile plot. The distribution of yd is a normal distribution
   /// to the extent that this plot is a straight line, with y-intercept and
   /// slope identified with mean and standard deviation, respectively, of the
   /// distribution.
   /// @param yd array of data, sorted in ascending order.
   /// @param nd length of array xd.
   /// @param xd array of length nd containing quantiles of yd on output.
template <typename T>   void QuantilePlot(T *yd, long nd, T *xd)
      throw(Exception);

   }  // end Robust namespace

   //@}

}  // end namespace gpstk

#undef ABSOLUTE

#endif
