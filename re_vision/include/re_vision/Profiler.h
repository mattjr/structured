/*	
 * File: Profiler.h
 * Project: DUtils library
 * Author: Dorian Galvez-Lopez
 * Date: September 14, 2010
 * Description: class for profiling code
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
#ifndef __D_PROFILER__
#define __D_PROFILER__

#include <map>
#include <vector>
#include <string>

#include "Timestamp.h"

namespace DUtils {

#define PROFILE_S(cmd, s) \
  { DUtils::Timestamp t_begin, t_end; \
    t_begin.setToCurrentTime(); \
    cmd; \
    t_end.setToCurrentTime(); \
    std::cout << s << " - elapsed time: " \
      << DUtils::Timestamp::Format(t_end - t_begin) \
      << std::endl; \
  }

#define PROFILE(cmd) PROFILE_S(cmd, "")

class Profiler
{
public:

  Profiler(): m_last_profile(""){}
  virtual ~Profiler(){}
  
  /**
   * Starts profiling the given item. If it was already been profiled, 
   * the last call to profile with that item is ignored
   * @param name name of item to profile
   */
  void profile(const std::string &name = "");
  
  /**
   * Does the same as Profiler::stop, but multiplies the elapsed time by
   * the given scale factor
   * @param scale stored_duration = actual_duration * scale
   * @param name item name
   */
  void stopAndScale(double scale, const std::string &name = "");
  
  /**
   * Stops profiling the given item or the last one if this is not provided.
   * Adds the elapsed time to the sum of this item profile time
   * @param name item name
   */
  inline void stop(const std::string &name = "")
  {
    stopAndScale(1.0, name);
  }

  double getMeanTime(const std::string &name = "") const ;
  double getStdevTime(const std::string &name = "") const ;
  double getMinTime(const std::string &name = "") const ;
  double getMaxTime(const std::string &name = "") const ;
  void getTime(std::vector<double> &time, const std::string &name = "") const;

protected:
  
  std::map<std::string, std::vector<double> > m_profiles;
  std::map<std::string, Timestamp> m_start_points;
  std::string m_last_profile;
  
};

}

#endif
