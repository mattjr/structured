/*	
 * File: Math.h
 * Project: DUtils library
 * Author: Dorian Galvez-Lopez
 * Date: April 2010
 * Description: some math functions
 *
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#pragma once
#ifndef __D_MATH__
#define __D_MATH__

#include <vector>
#include <algorithm>
#include <numeric>
#include <cmath>
#include <opencv/cv.h>

namespace DUtils {

	class Math {

	public:

		/* Returns the mean of a population
		 * @param v
		 */
		template <class T>
		static double Mean(const std::vector<T> &v)
		{
			if(v.empty())
				return 0;
			else{
				double sum = 0;
				typename std::vector<T>::const_iterator it;
				for(it = v.begin(); it != v.end(); it++){
					sum += *it;
				}
				return sum/double(v.size());
			}
		}

		/* Returns the standard deviation of a population
		 * @param v
		 * @param mean (optional): the mean of the population
		 */
		template <class T>
		static double Stdev(const std::vector<T> &v)
		{
			return Math::Stdev<T>(v, Math::Mean<T>(v));
		}

		template <class T>
		static double Stdev(const std::vector<T> &v, double mean)
		{
			if(v.size() <= 1)
				return 0;
			else{
				// stdev = sqrt( Sum{ (x_i - mean)^2 } / (N-1) )
				double sum = 0;
				typename std::vector<T>::const_iterator it;
				for(it = v.begin(); it != v.end(); it++){
					sum += pow(*it - mean, 2);
				}
				return sqrt(sum/double(v.size()-1));
			}
		}

	};

}

#endif

