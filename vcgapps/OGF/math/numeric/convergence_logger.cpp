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
 

#include <OGF/math/numeric/convergence_logger.h>
#include <OGF/basic/types/types.h>
#include <OGF/basic/debug/logger.h>
#include <iostream>
#include <math.h>

namespace OGF {

//_________________________________________________________

    ConvergenceLogger::ConvergenceLogger(
        const std::string& base_name, bool verbose, bool quiet
    ) {
        verbose_ = verbose ;
        quiet_ = quiet ;
        base_name_ = base_name ;
        bias_timer_ = nil ;
        bias_ = 0 ;
        iter_output_ = nil ;
        time_output_ = nil ;
        iteration_ = 0 ;
        if(verbose_) {
            std::string iter_file_name = base_name + "_i.dat" ;
            std::string time_file_name = base_name + "_t.dat" ;

            iter_output_ = new std::ofstream(iter_file_name.c_str()) ;
            time_output_ = new std::ofstream(time_file_name.c_str()) ;
            if(!*iter_output_) {
                Logger::err("ConvergenceLogger")
                    << "Error opening \'" << iter_file_name << "\'"  
                    << std::endl ;
                delete iter_output_ ;
                iter_output_ = nil ;
            }
            if(!*time_output_) {
                Logger::err("ConvergenceLogger")
                    << "Error opening \'" << time_file_name << "\'"  
                    << std::endl ;
                delete time_output_ ;
                time_output_ = nil ;
            }
        }
        if(!quiet_) {
            Logger::out(base_name_) << "begin task" << std::endl ;
        }
    }

    ConvergenceLogger::~ConvergenceLogger() {
        if(!quiet_) {
            Logger::out(base_name_)
                << "completed task: " 
                << "elapsed time="
                << total_time_.elapsed_user_time()
                << " used iterations="
                << iteration_ 
                << std::endl ;
        }
        delete iter_output_ ;
        delete time_output_ ;
    }
        
    void ConvergenceLogger::begin_bias() {
        bias_timer_ = new SystemStopwatch ;
    }
    
    void ConvergenceLogger::end_bias() {
        bias_ += bias_timer_-> elapsed_user_time() ;
        delete bias_timer_ ;
        bias_timer_ = nil ;
    }
        
    void ConvergenceLogger::log(int iteration, double residual) {
        iteration_ = iteration ;
        if(verbose_) {
            double conv = -log10(residual) ;            
            double elapsed_time = total_time_.elapsed_user_time() - bias_ ;
            if(iter_output_ != nil) {
                *iter_output_ << iteration << " " << conv << std::endl ;
            }
            if(time_output_ != nil) {
                *time_output_ << elapsed_time << " " << conv << std::endl ;
            }
        }
    }
    
//_________________________________________________________

}

