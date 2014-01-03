/*
 * File: Timestamp.h
 * Author: Dorian Galvez-Lopez
 * Date: March 2009
 * Description: timestamping functions
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

#ifndef __D_TIMESTAMP__
#define __D_TIMESTAMP__

#include <iostream>
using namespace std;

namespace DUtils {

class Timestamp
{
public:
  enum tOptions
  {
    NONE = 0,
    CURRENT_TIME = 0x1
  };
  
public:
	Timestamp(Timestamp::tOptions option = NONE);
	~Timestamp(void);

	/* Sets this instance to the current time
	 */
	void setToCurrentTime();

	/* Sets the timestamp from seconds and microseconds
	 * @param secs: seconds
	 * @param usecs: microseconds
	 */
	inline void setTime(unsigned long secs, unsigned long usecs){
		m_secs = secs;
		m_usecs = usecs;
	}

	/* Sets the timestamp from a string with the time in seconds
	 * @param stime: string such as "1235603336.036609"
	 */
	void setTime(const string &stime);

	/* Returns this timestamp as the number of seconds in (long) float format
	 */
	double getFloatTime() const;

	/* Returns this timestamp as the number of seconds in fixed length string format
	 */
	string getStringTime() const;

	/* Returns the difference in seconds between this timestamp (greater) and t (smaller)
	 * If the order is swapped, a negative number is returned
	 * @param t: timestamp to subtract from this timestamp
	 * @return difference in seconds
	 */
	double operator- (const Timestamp &t) const;

	/* Returns a copy of this timestamp + s seconds
	 * @param s: seconds
	 */
	Timestamp operator+ (double s) const;

	/* Returns a copy of this timestamp - s seconds
	 * @param s: seconds
	 */
	Timestamp operator- (double s) const;

	/* Returns whether this timestamp is at the future of t
	 * @param t
	 */
	bool operator> (const Timestamp &t) const;

	/* Returns whether this timestamp is at the future of (or is the same as) t
	 * @param t
	 */
	bool operator>= (const Timestamp &t) const;

	/* Returns whether this timestamp and t represent the same instant
	 * @param t
	 */
	bool operator== (const Timestamp &t) const;

	/* Returns whether this timestamp is at the past of t
	 * @param t
	 */
	bool operator< (const Timestamp &t) const;

	/* Returns whether this timestamp is at the past of (or is the same as) t
	 * @param t
	 */
	bool operator<= (const Timestamp &t) const;

  /**
   * Returns the timestamp in a human-readable string
   * @param machine_friendly if true, the returned string is formatted
   *   to yyyymmdd_hhmmss, without weekday or spaces
   * @note This has not been tested under Windows
   * @note The timestamp is truncated to seconds
   */
  string Format(bool machine_friendly = false) const;

	/* Returns a string version of the elapsed time in seconds, with the format
	 * xd hh:mm:ss, hh:mm:ss, mm:ss or s.us
	 * @param s: elapsed seconds (given by getFloatTime) to format
	 */
	static string Format(double s);
	

protected:
	unsigned long m_secs;	// seconds
	unsigned long m_usecs;	// microseconds
};

}

#endif

