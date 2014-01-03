/*	
 * File: STL.h
 * Project: DUtils library
 * Author: Dorian Galvez-Lopez
 * Date: November 2010
 * Description: STL-related functions
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
#ifndef __D_STL__
#define __D_STL__

#include <vector>
#include <algorithm>

namespace DUtils {

class STL {
public:

  /**
   * Removes from data the items of given indices
   * @param data (in/out) vector to remove items from
   * @param indices indices of items to remove
   * @param preserve_order if true, the items kept in data are in the same
   *    order as given. If false, the order may be different; this makes 
   *    this operation faster
   */
  template<class T>
  static void removeIndices(std::vector<T> &data, 
    const std::vector<unsigned int> &indices, bool preserve_order = true);
  
  /**
   * Removes from data the items of given indices, but can modify the index vector
   * @param data (in/out) vector to remove items from
   * @param indices indices of items to remove
   * @param preserve_order if true, the items kept in data are in the same
   *    order as given. If false, the order may be different; this makes 
   *    this operation faster
   */
  template<class T>
  static void removeIndices(std::vector<T> &data, 
    std::vector<unsigned int> &indices, bool preserve_order = true);

};

// ---------------------------------------------------------------------------

template<class T>
void STL::removeIndices(std::vector<T> &data, 
  const std::vector<unsigned int> &indices, bool preserve_order)
{
  if(indices.empty()) return;
  
  std::vector<unsigned int> copied_indices = indices;
  STL::removeIndices(data, copied_indices, preserve_order);
}

// ---------------------------------------------------------------------------

template<class T>
void STL::removeIndices(std::vector<T> &data, 
  std::vector<unsigned int> &indices, bool preserve_order)
{
  if(indices.empty()) return;
  
  // sort the index entries    
  std::sort(indices.begin(), indices.end()); // ascending order
  
  // remove those indices that exceed the data vector length
  {
    int i_idx = (int)indices.size() - 1;
    while( indices[i_idx] >= data.size() ) i_idx--;
    indices.resize(i_idx+1);
  }
    
  // make sure there are no repeated indices
  {
    const std::vector<unsigned int>::iterator last =
      std::unique(indices.begin(), indices.end());
    indices.erase(last, indices.end());
  }
    
  // go
  if(preserve_order)
  {
    // remove indices in descending order, grouping when possible
    int i_idx = (int)indices.size() - 1;
    while(i_idx >= 0)
    {
      int j_idx = i_idx - 1;
      while(j_idx >= 0 && ((int)(indices[i_idx] - indices[j_idx]) == i_idx - j_idx))
      {
        j_idx--;
      }
      data.erase(data.begin() + indices[j_idx + 1], 
        data.begin() + indices[i_idx] + 1);
      i_idx = j_idx;
    }
    
  }
  else
  { 
    // swap with the last items
    int nremoved = 0;
    
    const typename std::vector<T>::iterator first = data.begin();
    const typename std::vector<T>::iterator last = data.end()-1;
  
    int i_idx = (int)indices.size() - 1;
    
    // exception case: removing items are at the end of the vector
    while(i_idx >= 0 && 
      (indices.size() - i_idx == data.size() - indices[i_idx]))
    {
      i_idx--;
      nremoved++;
    }
        
    while(i_idx >= 0)
    {
      int j_idx = i_idx - 1;
      while(j_idx >= 0 && ((int)(indices[i_idx] - indices[j_idx]) == i_idx - j_idx))
      {
        j_idx--;
      }
      
      int nremoving = i_idx - j_idx;
      
      const typename std::vector<T>::iterator cpy_end = last - nremoved + 1;
      const typename std::vector<T>::iterator cpy_src = cpy_end - 
        MIN( nremoving, (int)data.size()-1 - nremoved - (int)indices[i_idx] );
      const typename std::vector<T>::iterator trg = first + indices[j_idx + 1];
            
      std::copy( cpy_src, cpy_end, trg );
      
      nremoved += nremoving;
      i_idx = j_idx;
    }
    
    data.resize(data.size() - nremoved);

    // v2, presumedly slower
#if 0
    std::vector<unsigned int>::reverse_iterator rit;
    for(rit = indices.rbegin(); rit != indices.rend(); ++rit)
    {
      if(*rit < data.size())
      {
        *(first + *rit) = *(last - nremoved);
        nremoved++;
      }
    }
    data.resize(data.size() - nremoved);
#endif
  }
  
}

// ---------------------------------------------------------------------------

}

#endif

