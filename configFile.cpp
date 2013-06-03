//
// structured - Tools for the Generation and Visualization of Large-scale
// Three-dimensional Reconstructions from Image Data. This software includes
// source code from other projects, which is subject to different licensing,
// see COPYING for details. If this project is used for research see COPYING
// for making the appropriate citations.
// Copyright (C) 2013 Matthew Johnson-Roberson <mattkjr@gmail.com>
//
// This file is part of structured.
//
// structured is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// structured is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with structured.  If not, see <http://www.gnu.org/licenses/>.
//

//!
//! \file auv_config_file.cpp
//!
//! This file contains the implementation of the the Config_File class.
//!
//! \author Ian Mahon
//!
//! \date 13-07-03
//!

#include "configFile.h"

#include <iostream>
#include <sstream>
#include <cstring>
#include <cstdlib>
#include <cerrno>

using namespace std;

// Comment token. Lines starting with this token will be ignored.
#define COMMENT_CHAR '#'


//----------------------------------------------------------------------------//
//   Exceptions                                                               //
//----------------------------------------------------------------------------//

Config_File_Exception::Config_File_Exception( const string &error )
   : runtime_error( error )
{

}

Config_File_Parse_Exception::Config_File_Parse_Exception( const string &error )
   : Config_File_Exception( error )
{

}

Config_File_Missing_Value_Exception::Config_File_Missing_Value_Exception
   ( const string &error )
   : Config_File_Exception( error )
{

}


//----------------------------------------------------------------------------//
//   Config_File Constructor                                                  //
//----------------------------------------------------------------------------//

Config_File::Config_File( const string &file_name )
{
   ifstream in_file( file_name.c_str( ) );
   if( !in_file )
   {
      stringstream ss;
      ss << "Unable to open configuration file '" << file_name << "'";
      throw Config_File_Exception( ss.str() );
   }
   parse_file( in_file, file_name );
} 


//----------------------------------------------------------------------------//
//   Get Functions                                                            //
//----------------------------------------------------------------------------//


bool Config_File::get_bool( const string &key ) const
{
   string value_string = get_string_value( key );
   
   if( strcasecmp( value_string.c_str( ), "true" ) == 0 ||
       strcasecmp( value_string.c_str( ), "t"    ) == 0 ||
       strcasecmp( value_string.c_str( ), "yes"  ) == 0 || 
       strcasecmp( value_string.c_str( ), "y"    ) == 0 ||
       strcasecmp( value_string.c_str( ), "on"   ) == 0      )
   {
      return true;
   }          
   else if( strcasecmp( value_string.c_str( ), "false" ) == 0 ||
            strcasecmp( value_string.c_str( ), "f"     ) == 0 ||
            strcasecmp( value_string.c_str( ), "no"    ) == 0 ||
            strcasecmp( value_string.c_str( ), "n"     ) == 0 ||
            strcasecmp( value_string.c_str( ), "off"   ) == 0    )
   {
      return false;
   }
   else
   {
      stringstream ss;
      ss << "Invalid configuration value for boolean option '"
         << key << "'" << endl;
      throw Config_File_Parse_Exception( ss.str() );
   }          
}


int Config_File::get_int( const string &key ) const
{
   string value_string = get_string_value( key );

   errno = 0;
   char *endptr = &value_string[value_string.size()];
   int value = strtol( &value_string[0], &endptr, 10 );
   if( errno != 0 || (*endptr)!='\0' )
   {
      stringstream ss;
      ss << "Invalid configuration value for integer option '"
         << key << "'" << endl;
      throw Config_File_Parse_Exception( ss.str() );
   }
   return value;
} 


unsigned int Config_File::get_uint( const string &key ) const
{
   string value_string = get_string_value( key );

   errno = 0;
   char *endptr = &value_string[value_string.size()];
   unsigned int value = strtoul( &value_string[0], &endptr, 10 );
   if( errno != 0 || (*endptr)!=0 )
   {
      stringstream ss;
      ss << "Invalid configuration value for integer option '"
         << key << "'" << endl;
      throw Config_File_Parse_Exception( ss.str() );
   }
   return value;
} 

double Config_File::get_double( const string &key ) const
{
   string value_string = get_string_value( key );

   char *endptr = &value_string[value_string.size()];
   double value = strtod( &value_string[0], &endptr );
   if( (*endptr)!=0 )
   {
      stringstream ss;
      ss << "Invalid configuration value for double option '"
         << key << "'" << endl;
      throw Config_File_Parse_Exception( ss.str() );
   }
   return value;
}


string Config_File::get_string( const string &key ) const
{
   string value_string = get_string_value( key );

   // strings should be in quotation marks - ignore them
   if( value_string[0] != '"' ||
       value_string[value_string.size( )-1] != '"' )
   {
      stringstream ss;
      ss << "Invalid configuration value for string option '"
         << key << "'" << endl;
      throw Config_File_Parse_Exception( ss.str() );
   }
   return value_string.substr( 1, value_string.size( )-2 );
}


//----------------------------------------------------------------------------//
//   Get Functions with a Default Value                                       //
//----------------------------------------------------------------------------//


bool Config_File::get_bool( const string &key, bool default_value ) const
{
   try
   {
      return get_bool( key );
   }
   catch( Config_File_Missing_Value_Exception &e )
   {
      return default_value;
   }
}


int Config_File::get_int( const string &key, int default_value ) const
{
   try
   {
      return get_int( key );
   }
   catch( Config_File_Missing_Value_Exception &e )
   {
      return default_value;
   }
}


unsigned int 
Config_File::get_uint( const string &key, unsigned int default_value ) const
{
   try
   {
      return get_uint( key );
   }
   catch( Config_File_Missing_Value_Exception &e )
   {
      return default_value;
   }
}


double Config_File::get_double( const string &key, double default_value ) const
{
   try
   {
      return get_double( key );
   }
   catch( Config_File_Missing_Value_Exception &e )
   {
      return default_value;
   }
}


string 
Config_File::get_string( const string &key, const string &default_value ) const
{
   try
   {
      return get_string( key );
   }
   catch( Config_File_Missing_Value_Exception &e )
   {
      return default_value;
   }
}


//----------------------------------------------------------------------------//
//   Set Functions                                                            //
//----------------------------------------------------------------------------//

void Config_File::set( const string &key, bool value )
{
   stringstream ss;
   if( value )
      ss << "True";
   else
      ss << "False";

   values[key] = ss.str( );
}

void Config_File::set( const string &key, int value )
{
   stringstream ss;
   ss << value;
   values[key] = ss.str( );
}

void Config_File::set( const string &key, unsigned int value )
{
   stringstream ss;
   ss << value;
   values[key] = ss.str( );
}

void Config_File::set( const string &key, double value )
{
   stringstream ss;
   ss << value;
   values[key] = ss.str( );
}

void Config_File::set( const string &key, const string &value )
{
   stringstream ss;
   ss << "\"" << value << "\"";
   values[key] = ss.str( );
}

void Config_File::set( const string &key, const char *value )
{
   stringstream ss;
   ss << "\"" << value << "\"";
   values[key] = ss.str( );
}


//----------------------------------------------------------------------------//
//   Obsolete Functions (Don't Use)                                           //
//----------------------------------------------------------------------------//

#if ENABLE_CONFIG_FILE_OBSOLETE_INTERFACE


bool Config_File::get_value( const string &key, bool &value, 
                             bool default_value ) const
{
   try
   {
      value = get_bool( key );
      return true;
   }
   catch( Config_File_Exception &e )
   {
      cerr << "WARNING - " << e.what() << endl;
      value = default_value;
      return false;
   }
}

bool Config_File::get_value( const string &key, int &value,
                             int default_value ) const
{
   try
   {
      value = get_int( key );
      return true;
   }
   catch( Config_File_Exception &e )
   {
      cerr << "WARNING - " << e.what() << endl;
      value = default_value;
      return false;
   }
} 

bool Config_File::get_value( const string &key, unsigned int &value, 
                             unsigned int default_value ) const
{
   try
   {
      value = get_uint( key );
      return true;
   }
   catch( Config_File_Exception &e )
   {
      cerr << "WARNING - " << e.what() << endl;
      value = default_value;
      return false;
   }
} 

bool Config_File::get_value( const string &key, float &value, 
                             float default_value ) const
{
   try
   {
      value = get_double( key );
      return true;
   }
   catch( Config_File_Exception &e )
   {
      cerr << "WARNING - " << e.what() << endl;
      value = default_value;
      return false;
   }
}

bool Config_File::get_value( const string &key, double &value, 
                             double default_value ) const
{
   try
   {
      value = get_double( key );
      return true;
   }
   catch( Config_File_Exception &e )
   {
      cerr << "WARNING - " << e.what() << endl;
      value = default_value;
      return false;
   }
}

bool Config_File::get_value( const string &key, string &value, 
                             const string &default_value ) const
{
   try
   {
      value = get_string( key );
      return true;
   }
   catch( Config_File_Exception &e )
   {
      cerr << "WARNING - " << e.what() << endl;
      value = default_value;
      return false;
   }
}



void Config_File::set_value( const string &key, bool value )
{
   set( key, value );
}

void Config_File::set_value( const string &key, int value )
{
   set( key, value );
}

void Config_File::set_value( const string &key, double value )
{
   set( key, value );
}

void Config_File::set_value( const string &key, const string &value )
{
   set( key, value );
}

void Config_File::set_value( const string &key, const char *value )
{
   set( key, value );
}

#endif // ENABLE_CONFIG_FILE_OBSOLETE_INTERFACE


//----------------------------------------------------------------------------//
//   Private Helper Functions                                                 //
//----------------------------------------------------------------------------//


//
// Parse the next token from a string containing a line from the configuration
// file. Tokens are seperated by whitespace except for string tokens that
// start and end with quotation characters.
//
// The file name and line number parameters are required to enable useful
// error messages to be printed.
//
bool Config_File::get_next_string( string       &current_line, 
                                   string       &next,
                                   const string &file_name,
                                   unsigned int  line_number ) const
{
   bool string_value = false;
   stringstream next_string;
   unsigned int index;


   // Ignore leading whitespace
   index = 0;
   while( index < current_line.size( ) && isblank( current_line[index] ) )
   {
      index++;
   }
   if( index == current_line.size( ) )
      return false;

   
   // Check if we are reading a string value. If we are, then the next string
   // ends when we rhe next quotation mark, not the next whitespace character.
   if( current_line[index] == '\"' )
      string_value = true;


   // Keep appending chars until we come to the end of the string or the end
   // of the line.
   unsigned int start_index = index;
   index++;
   while( index < current_line.size( ) && 
          ( (string_value && current_line[index] != '\"' ) || 
            (!string_value && !isblank( current_line[index] ) ) ) )
   {
      index++;
   }


   // If we are reading a string value, make sure the closing quotation
   // mark was found before the end of the line.
   if( string_value && index >= current_line.size( ) )
   {
      stringstream ss;
      ss << "Unterminated string on line " << line_number << " of file "
          << file_name << endl;
      throw Config_File_Parse_Exception( ss.str() );
   }
   

   // We want to include the closing quotation mark for string values
   if( string_value )
      index++;


   // Update the returned string value and removed the used chars from the
   // current line.
   next = current_line.substr( start_index, index-start_index );
   if( index+1 < current_line.size( ) )
      current_line = current_line.substr( index, current_line.size( )-index );
   else
      current_line.clear( );
   return true;
}



// Parse a config file, storing the names and values of options in
// the map datastructure.
//
// The file_name parameter is used to provide useful error messages.
void Config_File::parse_file( ifstream     &in_file,
                              const string &file_name )
{
   unsigned int line_number = 1;
   while( !in_file.eof( ) )
   {
      // Read the next line
      string current_line;
      getline( in_file, current_line );

      // Read the key from the current line. Ignore blank lines and comments
      string key;
      if( get_next_string( current_line, key, file_name, line_number ) &&
          key[0] != COMMENT_CHAR )
      {
         // Read the value from the current line
         string value;     
         if( !get_next_string(current_line,value,file_name,line_number) ||
              value[0]==COMMENT_CHAR )
         {
            stringstream ss;
            ss << "Missing value for " << key << " on line " << line_number
               << " of file " << file_name << endl;
            throw Config_File_Parse_Exception( ss.str() );   
         }
         
         // Check if there is data in the line that haven't been parsed
         string junk;
         if( get_next_string(current_line,junk,file_name,line_number) &&
             junk[0]!=COMMENT_CHAR )
         {
            stringstream ss;
            ss << "Excess data at end of line " << line_number
               << " of file " << file_name;
            throw Config_File_Parse_Exception( ss.str() );   
         }

         // Add the config option to the map
         values[key] = value;
      }
      line_number++;
   }
}


// Get the value associated with a key in its string format
string Config_File::get_string_value( const string &key ) const
{
   map<string,string>::const_iterator itr = values.find( key );
   if( itr==values.end() )
   {
      stringstream err;
      err << "No configuration value for option '" << key << "'";
      throw Config_File_Missing_Value_Exception( err.str() );
   }
   return itr->second;
}

