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
 

#ifndef __OGF_BASIC_DEBUG_LOGGER__
#define __OGF_BASIC_DEBUG_LOGGER__

#include <OGF/basic/common/common.h>
#include <OGF/basic/os/environment.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <set>


namespace OGF {

//_________________________________________________________

    class Logger ;
    class LoggerStream ;

	class BASIC_API LoggerStreamBuf : public std::stringbuf {
		public:
			LoggerStreamBuf(LoggerStream* loggerStream) : loggerStream_(loggerStream){
			}
			
		private:
			int sync();
			LoggerStream* loggerStream_ ;
	};


    class BASIC_API LoggerStream : public std::ostream {
    public:
        
        LoggerStream(Logger* logger);
        virtual ~LoggerStream() ;
        
    protected:
        void notify(std::string& str);
    private:
        
        Logger* logger_ ;
        
        friend class LoggerStreamBuf;
    } ;
    

//_________________________________________________________

    class BASIC_API LoggerClient {
    public:
        virtual void out_message(const std::string& value) = 0 ;
        virtual void warn_message(const std::string& value) = 0 ;
        virtual void err_message(const std::string& value) = 0 ;
        virtual void status_message(const std::string& value) = 0 ;
        virtual ~LoggerClient() ;
    } ;

	class BASIC_API CoutLogger : public LoggerClient {
	public:	
		CoutLogger();
		~CoutLogger();
		void out_message(const std::string& value);
		void warn_message(const std::string& value);
		void err_message(const std::string& value);
		void status_message(const std::string& value);
    } ;

	class BASIC_API FileLogger : public LoggerClient {
	public:
		FileLogger();
		FileLogger(std::string& file_name);
		~FileLogger();

		void out_message(const std::string& value);
		void warn_message(const std::string& value);
		void err_message(const std::string& value);
		void status_message(const std::string& value);
	
	protected:
		void set_file_name(std::string& value);

	private:
		std::string log_file_name_ ;
        std::ostream* log_file_ ;
	} ;


    /**
     * Implements a generic logging mechanism. Logging
     * can be activated/deactivated for an individual
     * feature, identified by its name (usually a 
     * class name). The environment variable
     * OGF_LOG is a colon-separated list of features
     * to log, the special item '*' (or 'Everything')
     * mean that everything should be logged. An item
     * starting with a dash means that the corresponding
     * feature should not be logged. For instance, 
     * '*:-TexCoordGenerator' means that everything except
     * TexCoordGenerator should be logged. The environment
     * variable OGF_LOG_FILE enable messages to be redirected
     * to a file. Client code should use the static functions
     * Logger::out(), Logger::err() and Logger::warn(), with
     * a string corresponding to the name of the class.
     */
    
    class BASIC_API Logger : public Environment {
    public:
        static void initialize() ;
        static void terminate() ;
        Logger() ;
        ~Logger() ;

        /** 
         * used to issue information messages. 
         * Example: <pre> 
         Logger::out("feature_name") << "initialized" << endl ; 
         * </pre>
         */
        static LoggerStream& out(const std::string& feature) ;

        /** 
         * used to issue errors 
         * Example: <pre> 
         *   Logger::out("feature_name") << "problem with args" << endl ; 
         * </pre>
         */
        static LoggerStream& err(const std::string& feature) ;

        /** 
         * used to issue warnings 
         * Example: <pre> 
         *   Logger::out("feature_name") << "strange value" << endl ; 
         * </pre>
         */
        static LoggerStream& warn(const std::string& feature) ;

        /** 
         * used to modify message in status bar 
         * Example: <pre> 
         *   Logger::status() << "Hyperdrive activated" << endl ; 
         * </pre>
         */
        static LoggerStream& status() ;

        virtual bool set_value(const std::string& name, const std::string& value)  ;
        virtual bool resolve(const std::string& name, std::string& value) const ;

		void register_client(LoggerClient* c);
		void unregister_client(LoggerClient* c);
		bool is_client(LoggerClient* c);

        static Logger* instance() { return instance_ ; }

    protected:
        //void flush_stream(LoggerStream* s) ;
        void notify(LoggerStream* from, std::string& message);

    protected:
        LoggerStream& out_stream(const std::string& feature) ;
        LoggerStream& err_stream(const std::string& feature) ;
        LoggerStream& warn_stream(const std::string& feature) ;
        LoggerStream& status_stream() ;

		void notify_out(std::string& message);
		void notify_warn(std::string& message);
		void notify_err(std::string& message);
		void notify_status(std::string& message);

    private:
        static Logger* instance_ ;

		//default clients (std::cout and file).
		LoggerClient* default_client_;
		LoggerClient* file_client_;


        LoggerStream out_ ;
        LoggerStream warn_ ;
        LoggerStream err_ ;
        LoggerStream status_ ;

		// features we want or don't want to log (only applies to 'out').
        std::set<std::string> log_features_ ;
        std::set<std::string> log_features_exclude_ ;
        bool log_everything_ ;
		std::string log_file_name_ ;
        
		std::string current_feature_ ;

		std::set<LoggerClient*> clients; // list of registered clients (observers)

        friend class LoggerStream ;
    } ;
    
    
//_________________________________________________________

}
#endif

