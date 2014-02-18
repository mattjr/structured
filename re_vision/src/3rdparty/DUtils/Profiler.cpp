/*	
 * File: Profile.cpp
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

#include <map>
#include <vector>
#include <string>
#include <algorithm>

#include "Timestamp.h"
#include "Profiler.h"
#include "Math.hpp"

using namespace DUtils;
using namespace std;

// ---------------------------------------------------------------------------

void Profiler::profile(const std::string &name)
{
  Timestamp t;
  m_last_profile = name;
  pair<std::map<std::string, Timestamp>::iterator, bool> res =
    m_start_points.insert(make_pair(name, t));
  res.first->second.setToCurrentTime();
}

// ---------------------------------------------------------------------------

void Profiler::stopAndScale(double scale, const std::string &name)
{
  Timestamp t;
  t.setToCurrentTime();
  
  string s = (name.empty() ? m_last_profile : name);
  
  std::map<std::string, Timestamp>::iterator it = m_start_points.find(s);
  
  if(it != m_start_points.end())
  {
    double duration = (t - it->second) * scale;
    m_start_points.erase(it);
    
    pair<std::map<std::string, std::vector<double> >::iterator,bool> pdit;
    pdit.first = m_profiles.find(s);
    if(pdit.first == m_profiles.end())
    {
      pdit = m_profiles.insert(make_pair(s, vector<double>()));
    }
    pdit.first->second.push_back(duration);
  }
}

// ---------------------------------------------------------------------------

double Profiler::getMeanTime(const std::string &name) const
{
  std::map<std::string, std::vector<double> >::const_iterator it =
    m_profiles.find(name);
  
  if(it != m_profiles.end())
  {
    return Math::Mean<double>(it->second);
  }
  else return 0;
}

// ---------------------------------------------------------------------------

double Profiler::getStdevTime(const std::string &name) const
{
  std::map<std::string, std::vector<double> >::const_iterator it =
    m_profiles.find(name);

  if(it != m_profiles.end())
  {
    return Math::Stdev<double>(it->second);
  }
  else return 0;
}
  
// ---------------------------------------------------------------------------
  
double Profiler::getMinTime(const std::string &name) const
{
  std::map<std::string, std::vector<double> >::const_iterator it =
    m_profiles.find(name);
  
  if(it != m_profiles.end())
  {
    return *std::min_element(it->second.begin(), it->second.end());
  }
  else return 0;
}
  
// ---------------------------------------------------------------------------
  
double Profiler::getMaxTime(const std::string &name) const
{
  std::map<std::string, std::vector<double> >::const_iterator it =
    m_profiles.find(name);
  
  if(it != m_profiles.end())
  {
    return *std::max_element(it->second.begin(), it->second.end());
  }
  else return 0;
}
  
// ---------------------------------------------------------------------------
  
void
Profiler::getTime(std::vector<double> &time, const std::string &name) const
{
  std::map<std::string, std::vector<double> >::const_iterator it =
    m_profiles.find(name);
  
  if(it != m_profiles.end())
  {
    time = it->second;
  }
  else time.clear();
}
  
// ---------------------------------------------------------------------------


