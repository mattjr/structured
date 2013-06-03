/* * structured - Tools for the Generation and Visualization of Large-scale
 * Three-dimensional Reconstructions from Image Data. This software includes
 * source code from other projects, which is subject to different licensing,
 * see COPYING for details. If this project is used for research see COPYING
 * for making the appropriate citations.
 * Copyright (C) 2013 Matthew Johnson-Roberson <mattkjr@gmail.com>
 *
 * This file is part of structured.
 *
 * structured is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * structured is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with structured.  If not, see <http://www.gnu.org/licenses/>.
 */

//!
//! \file auv_config_file.hpp
//!
//! This file contains the definition of the the Config_File class.
//!
//! \author Ian Mahon
//!
//! \date 13-07-03
//!

#ifndef CONFIG_FILE_HPP
#define CONFIG_FILE_HPP

#include <fstream>
#include <string>
#include <map>
#include <stdexcept>


// FIXME: Transition code to new interface that throws exceptions rather
//        than printing warnings and returning a default value.
#define ENABLE_CONFIG_FILE_OBSOLETE_INTERFACE 1


// Default values for each type. These are returned from the get_value 
// functions when there is no value associated with a key.
#if ENABLE_CONFIG_FILE_OBSOLETE_INTERFACE
   #define DEFAULT_BOOL          false
   #define DEFAULT_INT           0
   #define DEFAULT_UNSIGNED_CHAR 0
   #define DEFAULT_UNSIGNED_INT  0
   #define DEFAULT_DOUBLE        0.0
   #define DEFAULT_STRING        ""
#endif



class Config_File_Exception : public std::runtime_error
{
public:
   Config_File_Exception( const std::string &error );
};


class Config_File_Parse_Exception : public Config_File_Exception
{
public:
   Config_File_Parse_Exception( const std::string &error );
};


class Config_File_Missing_Value_Exception : public Config_File_Exception
{
public:
   Config_File_Missing_Value_Exception( const std::string &error );
};


//!
//! A class to access configuration values stored in a file.
//!
//! A Config_File object is initialised by providing the name of a configuration
//! file to the constructor. The configuration options will be read from the
//! file and stored in the Config_File object.
//!
//! An STL map is used to associate configuration options and their values.
//!
//! Both option names (keys) and values are stored as strings. Values are
//! converted to and from other types (int, double etc.) when a call to a 
//! get_value or set_value function is performed.
//!
class Config_File
{
public:

   //! Create a Config_File object and load the configuration values from a file
   Config_File( const std::string &file_name );


   // Load a value. 
   // A Config_File Missing_Value_Exception is thrown if no value exists
   // A Config_File_Parse_Exception is thrown if the value could not be parsed
   bool         get_bool  ( const std::string &key ) const;
   int          get_int   ( const std::string &key ) const;
   unsigned int get_uint  ( const std::string &key ) const;
   double       get_double( const std::string &key ) const;
   std::string  get_string( const std::string &key ) const;

   // Load a value. The default value is used if no value for the key exists
   // A Config_File_Parse_Exception is thrown if the value could not be parsed
   bool         get_bool  ( const std::string &key, bool               default_value  ) const;
   int          get_int   ( const std::string &key, int                default_value  ) const;
   unsigned int get_uint  ( const std::string &key, unsigned int       default_value  ) const;
   double       get_double( const std::string &key, double             default_value  ) const;
   std::string  get_string( const std::string &key, const std::string &default_value ) const;


   // Set a value
   void set( const std::string &key, bool               value );
   void set( const std::string &key, int                value );
   void set( const std::string &key, unsigned int       value );
   void set( const std::string &key, double             value );
   void set( const std::string &key, const std::string &value );
   void set( const std::string &key, const char        *value );
   

   //--- Obsolete Interface (Don't Use) ---//
#if ENABLE_CONFIG_FILE_OBSOLETE_INTERFACE

   //! Get the value of type bool associated with a key
   bool get_value( const std::string &key, bool &value, 
                   bool default_value=DEFAULT_BOOL ) const;

   //! Get the value of type int associated with a key
   bool get_value( const std::string &key, int &value, 
                  int default_value=DEFAULT_INT ) const;

   //! Get the value of type unsigned int associated with a key
   bool get_value( const std::string &key, unsigned int &value, 
                   unsigned int default_value=DEFAULT_INT ) const;

   //! Get the value of type float associated with a key
   bool get_value( const std::string &key, float &value, 
                   float default_value=DEFAULT_DOUBLE ) const;

   //! Get the value of type double associated with a key
   bool get_value( const std::string &key, double &value, 
                   double default_value=DEFAULT_DOUBLE ) const;

   //! Get the value of type string associated with a key
   bool get_value( const std::string &key, std::string &value, 
                   const std::string &default_value=DEFAULT_STRING ) const;


   //! Set the value associated with a key
   void set_value( const std::string &key, bool value );

   //! Set the value associated with a key
   void set_value( const std::string &key, int value );

   //! Set the value associated with a key
   void set_value( const std::string &key, double value );

   //! Set the value associated with a key
   void set_value( const std::string &key, const std::string &value );

   //! Set the value associated with a key
   void set_value( const std::string &key, const char *value );

#endif // ENABLE_CONFIG_FILE_OBSOLETE_INTERFACE


private:

   //! Parse the next token from a string containing a line from the 
   //! configuration file. Tokens are seperated by whitespace except for string 
   //! tokens that start and end with quotation characters.
   //!
   //! The file name and line number parameters are required to enable useful
   //! error messages to be printed.
   bool get_next_string( std::string       &current_line, 
                         std::string       &next,
                         const std::string &file_name,
                         unsigned int       line_number ) const;

   //! Parse a config file, storing the names and values of options in the map
   void parse_file( std::ifstream     &in_file,
                    const std::string &file_name );

   //! Get the value associated with a key in its string format
   std::string get_string_value( const std::string &key ) const;

   //! A mapping of configuration options (keys) to configuration values
   std::map<std::string,std::string> values;
};



#endif  // !CONFIG_FILE_HPP
