/*
 *  OGF/Graphite: Geometry and Graphics Programming Library +
 *  Utilities Copyright (C) 2000-2005 INRIA - Project ALICE
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
 *  Contact: Bruno Levy - levy@loria.fr
 *
 *     Project ALICE
 *     LORIA, INRIA Lorraine, 
 *     Campus Scientifique, BP 239
 *     54506 VANDOEUVRE LES NANCY CEDEX 
 *     FRANCE
 *
 *  Note that the GNU General Public License does not permit incorporating
 *  the Software into proprietary programs. 
 *
 * As an exception to the GPL, Graphite can be linked with the following (non-GPL) libraries:
 *     Qt, SuperLU, WildMagic and CGAL
 */
 

#include <OGF/math/geometry/polygon3d.h>

namespace OGF {

    namespace Geom {

        void barycentric_coords(
            const Polygon3d& P, const Point3d& p, std::vector<double>& bary
        ) {
            bary.clear() ;
            for(unsigned int i=0; i<P.size(); i++) { bary.push_back(0) ; }

            bool degenerate = false;
            double epsilon = 1.0e-5;
            double total = 0.0;
            unsigned int size = P.size();

            for (unsigned int i = 0; i < size; i++)  {
                unsigned int j = i + 1;
                if (j >= size) { j = 0; }
                unsigned int k = j + 1;
                if (k >= size) { k = 0; }
                
                const Point3d& pA = P[i];
                const Point3d& pB = P[j];
                const Point3d& pC = P[k];
		
                Vector3d vectBA = pA - pB;
                Vector3d vectBP = p - pB;
                Vector3d vectBC = pC - pB;
                double dotABP = (vectBP * vectBA);
                double crossABP = (vectBP ^ vectBA).norm();
                double dotPBC = (vectBC * vectBP);
                double crossPBC = (vectBC ^ vectBP).norm();
		
                degenerate |= crossABP < dotABP*epsilon;
                degenerate |= crossPBC < dotPBC*epsilon;
                degenerate |= vectBP.norm2() < epsilon*epsilon;
		
                // cotangent of angles
                double cotABP = dotABP/crossABP;
                double cotPBC = dotPBC/crossPBC;
                double factor = (cotABP+cotPBC)/vectBP.norm2();
                bary[j] = factor;
                total += factor;
            }
            
            if (false && degenerate) {
                //degenerate case, we will have to resort to another formula
                // in this case the computation is slower - n^2 instead of n
                total = 0.0;
                for (unsigned int i = 0; i < size; i++) {
                    unsigned int j = i + 1;
                    if (j >= size) { j = 0; }
                    unsigned int k = j + 1;
                    if (k >= size) { k = 0; }
                    
                    const Point3d& pA = P[i];
                    const Point3d& pB = P[j];
                    const Point3d& pC = P[k];
		    
                    double factor = (
                        ((pA-Origin()) ^ (pB-Origin())) +
                        ((pB-Origin()) ^ (pC-Origin())) +
                        ((pC-Origin()) ^ (pA-Origin()))
                    ).norm();
                    for (unsigned int l = k; l != i; l++) {
                        if (l >= size) { l = 0 ; }
                        unsigned int m = l + 1;
                        if (m >= size) { m = 0; }
                        
                        const Point3d& pD = P[l];
                        const Point3d& pE = P[m];
                        double area = (
                            ((pD-Origin()) ^ (pE-Origin())) +
                            ((pE-Origin()) ^ (p-Origin())) +
                            ((p-Origin()) ^ (pD-Origin()))
                        ).norm();
                        factor *= area;
                    }
		    
                    // there are actually 2 factors 0.5 to retrieve the real area,
                    // but they are transparently taken care of in the normalization
                    bary[j] = factor;
                    total += factor;
                }
            }

            // normalize the coordinates and we are done
            for (unsigned int n = 0; n < size; n++)  {
                bary[n] = bary[n]/total;
            }
        }
    }

}

