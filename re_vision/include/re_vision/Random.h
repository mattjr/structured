/*	
 * File: Random.h
 * Project: DUtils library
 * Author: Dorian Galvez-Lopez
 * Date: April 2010
 * Description: manages pseudo-random numbers
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
#ifndef __D_RANDOM__
#define __D_RANDOM__

#include <cstdlib>

namespace DUtils {

class Random
{
public:
	/**
	 * Sets the random number seed to the current time
	 */
	static void SeedRand();
	
	/**
	 * Sets the random number seed to the current time only the first
	 * time this function is called
	 */
	static void SeedRandOnce();

	/** 
	 * Sets the given random number seed
	 * @param seed
	 */
	static void SeedRand(int seed);

	/** 
	 * Sets the given random number seed only the first time this function 
	 * is called
	 * @param seed
	 */
	static void SeedRandOnce(int seed);

	/**
	 * Returns a random number in the range [0..1]
	 * @return random T number in [0..1]
	 */
	template <class T>
	static T RandomValue(){
		return (T)rand()/(T)RAND_MAX;
	}

	/**
	 * Returns a random number in the range [min..max]
	 * @param min
	 * @param max
	 * @return random T number in [min..max]
	 */
	template <class T>
	static T RandomValue(T min, T max){
		return Random::RandomValue<T>() * (max - min) + min;
	}

	/**
	 * Returns a random int in the range [min..max]
	 * @param min
	 * @param max
	 * @return random int in [min..max]
	 */
	static int RandomInt(int min, int max);
	
	/** 
	 * Returns a random number from a gaussian distribution
	 * @param mean
	 * @param sigma standard deviation
	 */
	template <class T>
	static T RandomGaussianValue(T mean, T sigma)
	{
	  // Box-Muller transformation
    T x1, x2, w, y1;
    
    do {
      x1 = (T)2. * RandomValue<T>() - (T)1.;
      x2 = (T)2. * RandomValue<T>() - (T)1.;
      w = x1 * x1 + x2 * x2;
    } while ( w >= (T)1. || w == (T)0. );

    w = sqrt( ((T)-2.0 * log( w ) ) / w );
    y1 = x1 * w;

    return( mean + y1 * sigma );
	}

private:

  /// If SeedRandOnce() has already been called
  static bool m_seeded_current_time;
  
  /// If SeedRandOnce(int) has already been called
  static bool m_seeded_int;

};

}

#endif

