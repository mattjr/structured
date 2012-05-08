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
 

#ifndef __OGF_BASIC_OS_PROCESS__
#define __OGF_BASIC_OS_PROCESS__

#include <OGF/basic/common/common.h>
#include <OGF/basic/types/counted.h>
#include <OGF/basic/types/smart_pointer.h>

namespace OGF {

    /**
     * Client code will derive from this class.
     */
    class BASIC_API Thread : public Counted {
    public:
        virtual void run() = 0 ;
        virtual ~Thread() ;
    } ;
    typedef SmartPointer<Thread> Thread_var ;


    /**
     * Class to manipulate a group of user threads.
     */
    template <class THREAD> class ThreadGroup : public std::vector<Thread_var> {
    public:
        typedef std::vector<Thread_var> baseclass ;

	ThreadGroup() {	}

        THREAD* operator[](unsigned int i) {
            ogf_debug_assert(i < size()) ;
            Thread* result = baseclass::operator[](i) ;
            return static_cast<THREAD*>(result) ;
        }
    } ;


    /**
     * Wrapper for multithreading API, for internal use only.
     */
    class BASIC_API ThreadManager {
    public:
        virtual void run_threads(std::vector<Thread_var>& threads) = 0 ;
        virtual unsigned int maximum_concurrent_threads() = 0 ;
        virtual ~ThreadManager() ;
    } ;

    /**
     * Does not use multithreading, just a place-holder
     * (default ThreadManager).
     */
    class BASIC_API MonoThreadingThreadManager : public ThreadManager {
    public:
        virtual void run_threads(std::vector<Thread_var>& threads) ;        
        virtual unsigned int maximum_concurrent_threads() ;
    } ;

    namespace Process {
        void BASIC_API initialize() ;
        void BASIC_API terminate() ;

        /** Terminates the current process. */
        void BASIC_API brute_force_kill() ;

        /** 
         * The maximum number of threads that can be running simultaneously,
         * typically number of cores if multithreading is supported, else 1.
         */
        unsigned int BASIC_API maximum_concurrent_threads() ;

        /**
         * Runs a set of threads simultaneously, and waits for completion
         * of all of them.
         */
        void BASIC_API run_threads(std::vector<Thread_var>& threads) ;
        
        unsigned int BASIC_API number_of_cores() ;
        void BASIC_API set_thread_manager(ThreadManager* thread_manager) ;

	/**
	 * Checks whether multiple threads are running.
	 */
	bool BASIC_API is_running_threads() ;

        /**
         * If FPE is enabled, then floating point exceptions raise a signal,
         * else they generate NaNs.
         */
        void BASIC_API enable_FPE() ;

        /**
         * If FPE is enabled, then floating point exceptions raise a signal,
         * else they generate NaNs.
         */
        void BASIC_API disable_FPE() ;


	void BASIC_API enable_multithreading() ;
	void BASIC_API disable_multithreading() ;

        /**
	 * Outputs the stack trace to the terminal.
	 * (not implemented yet under Windows)
	 */ 
	void BASIC_API show_stack_trace() ;
       
        /**
	 * Called at initialization.
	 */ 
        void BASIC_API install_signal_handlers() ;
    } ;
}

#endif
