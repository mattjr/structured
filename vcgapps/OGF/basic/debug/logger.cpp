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
 

#include <OGF/basic/debug/logger.h>
#include <OGF/basic/types/types.h>
#include <OGF/basic/debug/assert.h>

#include <stdlib.h>

/* 
Disables the warning caused by passing 'this' as an argument while
construction is not finished (in LoggerStream ctor).
As LoggerStreamBuf only stores the pointer for later use, so we can
ignore the fact that 'this' is not completly formed yet.
*/
#pragma warning(disable:4355) 


namespace OGF {

//_________________________________________________________

	int LoggerStreamBuf::sync(){
		std::string str(this->str());
		loggerStream_->notify(str);
		this->str("");
		return 0;
	}

//_________________________________________________________


	LoggerStream::LoggerStream(Logger* logger)
		: std::ostream(new LoggerStreamBuf(this)), logger_(logger) {
    }
	
	LoggerStream::~LoggerStream() {
		std::streambuf * buf = rdbuf();
		delete buf;

    }

	void LoggerStream::notify(std::string& str){
		logger_->notify(this, str);
	}

    
//_________________________________________________________

    LoggerClient::~LoggerClient() {
    }

//_________________________________________________________

	CoutLogger::CoutLogger(){
	}

	CoutLogger::~CoutLogger(){
		Logger::instance()->unregister_client(this);
	}

	void CoutLogger::out_message(const std::string& value){ std::cout << value << std::flush; }
	void CoutLogger::warn_message(const std::string& value){ std::cout << value << std::flush; }
	void CoutLogger::err_message(const std::string& value){ std::cout << value << std::flush; }
	void CoutLogger::status_message(const std::string& value){ }

//_________________________________________________________

	FileLogger::FileLogger() : log_file_(nil){
		std::string file_name ;
		if( false ){ // TODO if we can get the file name from environment, set it
			set_file_name( file_name ) ;
		}
	}

	FileLogger::FileLogger(std::string& file_name) : log_file_(nil){
		set_file_name( file_name ) ;
	}

	FileLogger::~FileLogger(){
		Logger::instance()->unregister_client(this);
		delete log_file_ ;
		log_file_ = nil ; 
	}

	void FileLogger::set_file_name(std::string& value){
		log_file_name_ = value ;
		if (log_file_ != nil) {
			delete log_file_ ;
			log_file_ = nil ;
		}
        if(log_file_name_.length() != 0) {
            log_file_ = new std::ofstream(log_file_name_.c_str()) ;
        }
	}

	void FileLogger::out_message(const std::string& value){
		if (log_file_ != nil) {
			*log_file_ << value << std::flush ;
		}
	}
	
	void FileLogger::warn_message(const std::string& value){
		if (log_file_ != nil) {
			*log_file_ << value << std::flush ;
		}
	}
	
	void FileLogger::err_message(const std::string& value){
		if (log_file_ != nil) {
			*log_file_ << value << std::flush ;
		}
	}
	
	void FileLogger::status_message(const std::string& value){
	}


//_________________________________________________________

    Logger* Logger::instance_ = nil ;

    void Logger::initialize() {
        instance_ = new Logger() ;
        Environment::instance()->add_environment(instance_) ;
        Logger::out("Logger") << "initialized" << std::endl ;
    }
    
    void Logger::terminate() {
        Logger::out("Logger") << "terminating" << std::endl ;
        delete instance_ ;
        instance_ = nil ;
    }
 
    bool Logger::set_value(const std::string& name, const std::string& value) {

        if(name == "log_file_name") {
			log_file_name_ = value;
			file_client_ = new FileLogger(log_file_name_);
			register_client(file_client_);
			return true ;
        }

        if(name == "log_features") {
            std::vector<std::string> features ;
            String::split_string(value,';',features) ;
            log_features_.clear();
            for(unsigned int i=0; i<features.size(); i++) {
                log_features_.insert(features[i]) ;
            }
            log_everything_ = (features.size() == 1 && features[0] == "*") ;
            notify_observers(name) ;
            return true ;
        }

        if(name == "log_features_exclude") {
            std::vector<std::string> features ;
            String::split_string(value,';',features) ;
            log_features_exclude_.clear();
            for(unsigned int i=0; i<features.size(); i++) {
                log_features_exclude_.insert(features[i]) ;
            }
            notify_observers(name) ;
            return true ;
        }

        return false ;
    }

    bool Logger::resolve(const std::string& name, std::string& value) const {

        if(name == "log_file_name") {
            value = log_file_name_ ;
            return true ;
        }

        if(name == "log_features") {
            value = "" ;
            for(
                std::set<std::string>::const_iterator it = log_features_.begin(); 
                it != log_features_.end(); it++
            ) {
                if(value.length() != 0) {
                    value += ';' ;
                }
                value += *it ;
            }
            return true ;
        }

        if(name == "log_features_exclude") {
            value = "" ;
            for(
                std::set<std::string>::const_iterator 
                    it = log_features_exclude_.begin(); 
                it != log_features_exclude_.end(); it++
            ) {
                if(value.length() != 0) {
                    value += ';' ;
                }
                value += *it ;
            }
            return true ;
        }

        return false ;
    }

	
	void Logger::register_client(LoggerClient* c){
		clients.insert(c);
	}
	
	void Logger::unregister_client(LoggerClient* c){
		clients.erase(c);
	}

	bool Logger::is_client(LoggerClient* c){
		return clients.find(c) != clients.end();
	}


    Logger::Logger() : out_(this), warn_(this), err_(this), status_(this) {
        log_everything_ = false ;
        if(::getenv("OGF_LOG") != nil && (
               ::getenv("OGF_LOG") == std::string("*") ||
               ::getenv("OGF_LOG") == std::string("everything")               
           )
        ) {
            log_everything_ = true ;
        }

		// add a default client printing stuff to std::cout
		default_client_ = new CoutLogger(); 
		register_client(default_client_ );
		file_client_ = nil ;
    }
    
    Logger::~Logger() {
		delete default_client_;
		default_client_ = nil;
		
		if(file_client_ != nil) {
			delete file_client_;
			file_client_ = nil;
		}
    }

    LoggerStream& Logger::out(const std::string& feature) {
        ogf_assert(instance_ != nil) ;
        return instance_->out_stream(feature) ;
    }

    LoggerStream& Logger::err(const std::string& feature) {
        ogf_assert(instance_ != nil) ;
        return instance_->err_stream(feature) ;
    }

    LoggerStream& Logger::warn(const std::string& feature) {
        ogf_assert(instance_ != nil) ;
        return instance_->warn_stream(feature) ;
    }

    LoggerStream& Logger::status() {
        ogf_assert(instance_ != nil) ;
        return instance_->status_stream() ;
    }

    LoggerStream& Logger::out_stream(const std::string& feature) {
        current_feature_ = feature ;
        return out_ ;
    }

    LoggerStream& Logger::err_stream(const std::string& feature) {
        current_feature_ = feature ;
        return err_ ;
    }

    LoggerStream& Logger::warn_stream(const std::string& feature) {
        current_feature_ = feature ;
        return warn_ ;
    }

    LoggerStream& Logger::status_stream() {
        return status_ ;
    }


	void Logger::notify_out(std::string& message){
		if( 
			(log_everything_ && log_features_exclude_.find(current_feature_)
				== log_features_exclude_.end() )
			|| (log_features_.find(current_feature_) != log_features_.end())
          ) {
          	std::set<LoggerClient*>::iterator it;
			for (it = clients.begin(); it != clients.end(); it++){
				(*it)->out_message( "[" + current_feature_ + "] " + "" + message );
			}
        } 
	}
	void Logger::notify_warn(std::string& message){
		std::set<LoggerClient*>::iterator it;
		for (it = clients.begin(); it != clients.end(); it++){
			(*it)->warn_message( "[" + current_feature_ + "] " + "Warning: " + message );
			(*it)->status_message( std::string("Warning: " + message) );
		}
	}
	void Logger::notify_err(std::string& message){
		std::set<LoggerClient*>::iterator it;
		for (it = clients.begin(); it != clients.end(); it++){
			(*it)->err_message( "[" + current_feature_ + "] " + "Error: " + message );
			(*it)->status_message( std::string("Error: " + message) );
		}
	}
	void Logger::notify_status(std::string& message){
		std::set<LoggerClient*>::iterator it;
		for (it = clients.begin(); it != clients.end(); it++){
			(*it)->status_message( std::string(message) );
		}
	}



	void Logger::notify(LoggerStream* s, std::string& message) {
		if(s == &out_) {
            notify_out(message);
        } else if (s == &warn_) {
            notify_warn(message);
        } else if (s == &err_) {
            notify_err(message);
        } else if(s == &status_) {
            notify_status(message);
        } else {
            ogf_assert(false) ;
        }
    }
 	
//_________________________________________________________

}

