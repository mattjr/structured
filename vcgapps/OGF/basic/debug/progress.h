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
 

#ifndef __OGF_BASIC_DEBUG_PROGRESS__
#define __OGF_BASIC_DEBUG_PROGRESS__

#include <OGF/basic/common/common.h>
#include <OGF/basic/types/types.h>

namespace OGF {
    
//_________________________________________________________

    class ProgressClient ;
    
    /**
     * For internal use, client code do not need to use this one.
     */
    class BASIC_API Progress {
    public:
        Progress() ;
        virtual void notify(int new_val) ;
        virtual ~Progress() ;
        static Progress* instance() ;
        void set_client(ProgressClient* c) { client_ = c ; }
        void push() ;
        void pop() ;
        void cancel()            { canceled_ = true ;  }
        void clear_canceled()    { canceled_ = false ; }
        bool is_canceled() const { return canceled_ ;  }
    private:
        static Progress* instance_ ;
        ProgressClient* client_ ;
        int level_ ;
        bool canceled_ ;
    } ;

//_________________________________________________________

    /**
     * For internal use, client code do not need to use this one.
     */
    class BASIC_API ProgressClient {
    public:
        virtual void notify_progress(int new_val) ;
        virtual ~ProgressClient() ;
    } ;

//_________________________________________________________

    class BASIC_API ProgressLogger {
    public:
        ProgressLogger(int max_val = 100, const std::string& task_name = "", bool quiet = false) ;
        ~ProgressLogger() ;
        void notify(int new_val) ;
        void next() ;
        bool is_canceled() const {
            return Progress::instance()->is_canceled() ;
        }

    protected:
        void update() ;

    private:
        int max_val_ ;
        std::string task_name_ ;
        int cur_val_ ;
        int cur_percent_ ;
        bool quiet_ ;
    } ;

//_________________________________________________________

}
#endif

