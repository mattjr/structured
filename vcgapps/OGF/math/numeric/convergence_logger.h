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
 

#ifndef __OGF_MATH_NUMERIC_CONVERGENCE_LOGGER__
#define __OGF_MATH_NUMERIC_CONVERGENCE_LOGGER__

#include <OGF/math/common/common.h>
#include <OGF/basic/os/stopwatch.h>
#include <fstream>
#include <string>

namespace OGF {

//_________________________________________________________

    /**
     * Outputs an iteration and time log, for testing iterative 
     * solvers.
     */
    
    class MATH_API ConvergenceLogger {
    public:
            
        /**
         * @param base_name will be used to create the log files
         *   and to output the messages.
         * @param verbose if not set, no log file is created, just
         *   the timings are logged.
         */
        ConvergenceLogger(
            const std::string& base_name, bool verbose = false, bool quiet = false
        ) ;

        ~ConvergenceLogger() ;

        /**
         * Computing the residual for logging may consume
         * additional time. This function makes it possible
         * to ignore this additional time in the logs.
         */
        void begin_bias() ;

        /**
         * This function should be called at the end of a
         * computation done just for logging.
         */
        void end_bias() ;

        /**
         * Outputs a sample in the log.
         */
        void log(int iteration, double residual) ;

    private:
        std::string base_name_ ;
        bool verbose_ ;
        bool quiet_ ;
        SystemStopwatch total_time_ ;
        SystemStopwatch* bias_timer_ ;
        double bias_ ;
        std::ofstream* iter_output_ ;
        std::ofstream* time_output_ ;
        int iteration_ ;
    } ;
//_________________________________________________________

}

#endif

