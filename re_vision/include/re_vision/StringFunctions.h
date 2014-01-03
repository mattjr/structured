/*
 * File: StringFunctions.h
 * Author: Dorian Galvez-Lopez
 * Date: December 2010
 * Description: string functions
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

#ifndef __D_STRING__
#define __D_STRING__

#include <string>
#include <vector>

namespace DUtils {

class StringFunctions
{
public:

  /**
   * Splits the given string into single tokens
   * @param s string
   * @param tokens returned tokens (no empty tokens are returned)
   * @param delims delimitation characters
   */
  static void split(const std::string &s, std::vector<std::string> &tokens,
    const std::string &delims = " \t\n");

};

}

#endif

