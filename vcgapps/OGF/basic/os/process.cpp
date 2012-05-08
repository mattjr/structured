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
 
#include <OGF/basic/os/process.h>
#include <OGF/basic/os/environment.h>

#ifdef WIN32
#include <windows.h>
#else
#include <unistd.h>
#include <fenv.h> 
#include <sys/types.h>
#include <unistd.h>
#include <signal.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <execinfo.h>
#endif

#include <sstream>

namespace OGF {

    Thread::~Thread() {
    }

    ThreadManager::~ThreadManager() {
    }

    void MonoThreadingThreadManager::run_threads(std::vector<Thread_var>& threads) {
        for(unsigned int i=0; i<threads.size(); i++) {
            threads[i]->run() ;
        }
    }

    unsigned int MonoThreadingThreadManager::maximum_concurrent_threads() {
        return 1 ;
    }

    namespace Process {

        static ThreadManager* thread_manager_ = nil;

        void initialize() {
            set_thread_manager(new MonoThreadingThreadManager) ;
	    std::ostringstream out ;
	    out << number_of_cores() << std::ends ;
	    Environment::instance()->set_value("nb_cores", out.str()) ;
        } 

        void terminate() {
// TODO: we got a (small) memory leak here, but it crashes
//  when I uncomment this line.            
//         set_thread_manager(nil) ;
        }

        void set_thread_manager(ThreadManager* thread_manager) {
            delete thread_manager_ ;
            thread_manager_ = thread_manager ;
        }


	static int running_threads_invocations_ = 0 ;

        void run_threads(std::vector<Thread_var>& threads) {
	    running_threads_invocations_++ ;
            thread_manager_->run_threads(threads) ;
	    running_threads_invocations_-- ;
        }

	bool is_running_threads() {
	    return (running_threads_invocations_ > 0) ;
	}

	static bool graphite_multithreading = true ;

	void enable_multithreading() {
	    graphite_multithreading = true ;
	    Logger::out("Process") << "Enabled multithreading" << std::endl ;
	    Logger::out("Process") << "Number of cores = " << number_of_cores() << std::endl ;
	    Logger::out("Process") << "Max. concurrent threads = " << maximum_concurrent_threads() << std::endl ;
	    if(maximum_concurrent_threads() == 1) {
		Logger::warn("Process") << "Processor is not a multicore or missing multithreading manager (e.g. cgal2graphite plugin)"
					<< std::endl ;
	    }
	}

	void disable_multithreading() {
	    graphite_multithreading = false ;
	    Logger::out("Process") << "Disabled multithreading" << std::endl ;
	}

        unsigned int maximum_concurrent_threads() {
            return 
		graphite_multithreading ?
		   thread_manager_->maximum_concurrent_threads() 
		: 1 ;
        }



#ifdef WIN32

        void brute_force_kill() {
            // Get the pid of this process
            DWORD processId = GetCurrentProcessId();
            
            // then modify its privileges to allow full acces
            HANDLE hHandle;
            
            hHandle = ::OpenProcess(PROCESS_QUERY_INFORMATION,0,processId);
            HANDLE tokHandle;
            OpenProcessToken( hHandle,TOKEN_ALL_ACCESS, &tokHandle);
            
            TOKEN_PRIVILEGES tp;
            LUID luid;
            LookupPrivilegeValue( 
                NULL,            // lookup privilege on local system
                SE_DEBUG_NAME,   // privilege to lookup 
                &luid );
            
            tp.PrivilegeCount = 1;
            tp.Privileges[0].Luid = luid;
            tp.Privileges[0].Attributes = SE_PRIVILEGE_ENABLED;
            // Enable the privilege.
            
            AdjustTokenPrivileges(
                tokHandle, 
                FALSE, 
                &tp, 
                sizeof(TOKEN_PRIVILEGES), 
                (PTOKEN_PRIVILEGES) NULL, 
                (PDWORD) NULL
            ) ; 
    
            if(hHandle == NULL){
                DWORD err = GetLastError();
            }

            // kill the process in a quite brutal way...
            HANDLE hHandle2 = ::OpenProcess(PROCESS_ALL_ACCESS,0,processId);
            DWORD dwExitCode = 0;
            // we don't need to know the current state of the process : it is STILL_ACTIVE (259)
            // and we want this termination to look normal (exit with code 0)
            //::GetExitCodeProcess(hHandle2,&dwExitCode); 
            ::TerminateProcess(hHandle2,dwExitCode);
        }

        

#else

        void brute_force_kill() {
            exit(-1) ;
        }

#endif

        unsigned int number_of_cores() {
            static unsigned int result = 0 ;
            if(result == 0) {
#ifdef WIN32
                SYSTEM_INFO si ;
                GetSystemInfo(&si) ;
                result = si.dwNumberOfProcessors ;
#else
                result = sysconf(_SC_NPROCESSORS_ONLN) ;
#endif                
            }
            return result ;
        }


        void enable_FPE() {
#ifdef WIN32
	    Logger::warn("Process") << "FPE control not implemented under Windows" << std::endl ;
#else
            int excepts= 0
              //| FE_INEXACT           // inexact result
                | FE_DIVBYZERO         // division by zero
              //| FE_UNDERFLOW         // result not representable due to underflow
                | FE_OVERFLOW          // result not representable due to overflow
                | FE_INVALID           // invalid operation
                ;
            feenableexcept(excepts);
	    Logger::out("Process") << "Enabled FPE" << std::endl ;
#endif
        }

        void disable_FPE() {
#ifdef WIN32
	    Logger::warn("Process") << "FPE control not implemented under Windows" << std::endl ;
#else
            feenableexcept(0);
	    Logger::out("Process") << "Disabled FPE" << std::endl ;
#endif
        }


        
	void show_stack_trace() {
#ifdef WIN32
	    Logger::err("StackTrace") << "Not implemented in Graphite for Windows" << std::endl ;
#else
	    Logger::err("StackTrace") << "==================== Stack trace ========================" << std::endl ;
	    const int size = 200 ;
	    static void* buffer[size] ;
	    size_t nb_pointers = backtrace(buffer, size) ;
	    Logger::err("StackTrace") << nb_pointers << " stack frames" << std::endl ;
	    int fd = ::open("/tmp/stacktrace.txt", O_WRONLY | O_CREAT | O_TRUNC, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH) ;
	    backtrace_symbols_fd(buffer, nb_pointers, fd) ;
	    ::close(fd) ;
	    system("cat /tmp/stacktrace.txt | sed -e \'sx/.*/xx' | /usr/bin/c++filt") ;
	    Logger::err("StackTrace") << "=========================================================" << std::endl ;
#endif
	}

       
        static void graphite_signal_handler(int sig) {
	    Logger::err("SignalHandler") << "Caught " << sig << std::endl ;
	    show_stack_trace() ;
	    abort() ;
	}
       
       
        void install_signal_handlers()  {
#ifdef WIN32
#else	   
	    signal(SIGFPE,  graphite_signal_handler) ;
	    signal(SIGSEGV, graphite_signal_handler) ;
   	    signal(SIGBUS,  graphite_signal_handler) ;
	    signal(SIGILL,  graphite_signal_handler) ;
#endif	   
	}
       
    }
}

