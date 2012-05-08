/*
 *  OGF/Graphite: Geometry and Graphics Programming Library + Utilities
 *  Copyright (C) 2000 Bruno Levy
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
 *  Contact: Bruno Levy
 *
 *     levy@loria.fr
 *
 *     ISA Project
 *     LORIA, INRIA Lorraine, 
 *     Campus Scientifique, BP 239
 *     54506 VANDOEUVRE LES NANCY CEDEX 
 *     FRANCE
 *
 *  Note that the GNU General Public License does not permit incorporating
 *  the Software into proprietary programs. 
 */
 
#ifndef __OGF_BASIC_IO_LINE_STREAM__
#define __OGF_BASIC_IO_LINE_STREAM__

#include <iostream>
#include <sstream>
#include <OGF/basic/debug/assert.h>


namespace OGF {

    class LineInputStream {
    public:
        LineInputStream(
            std::istream& in
        ) : in_(in), line_in_(nil) {   }
        ~LineInputStream() {
            delete line_in_ ; line_in_ = nil ;
        }
        bool eof() const { return in_.eof() ; }
        bool eol() const { return line_in_ == nil || line_in_->eof() ; }
        bool ok() const { return in_ != NULL; }

        void get_line() {
            in_.getline(buffer_, 65536) ;
            bool check_multiline = true ;
            int total_length = 65536 ;
            char* ptr = buffer_ ;

            // If the line ends with a backslash, append
            // the next line to the current line.
            while(check_multiline) {
                int L = strlen(ptr) ;
                total_length -= L ;
                ptr = ptr + L - 2;
                if(*ptr == '\\' && total_length > 0) {
                    *ptr = ' ' ;
                    ptr++ ;
                    in_.getline(ptr, total_length) ;
                } else {
                    check_multiline = false ;
                }
            }

            if(total_length < 0) {
                Logger::err("LineInputStream") << "MultiLine longer than 65536 bytes" << std::endl ;
            }

            delete line_in_ ; line_in_ = new std::istringstream(buffer_) ;
        }

        std::istream& line() { 
            ogf_assert(line_in_ != nil) ;
            return *line_in_ ; 
        }

        const char *current_line() const {
            return buffer_;
        }

        template <class T> LineInputStream& operator>>(T& param) {
            *line_in_ >> param;
            return *this;
        }

    private:
        std::istream& in_ ;
        std::istringstream* line_in_ ;
        char buffer_[65536] ;
    } ;
}

#endif
