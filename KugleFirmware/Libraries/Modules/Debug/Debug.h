/* Copyright (C) 2018 Thomas Jespersen, TKJ Electronics. All rights reserved.
 *
 * This program is free software: you can redistribute it and/or modify  
 * it under the terms of the GNU General Public License as published by  
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but 
 * WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU 
 * General Public License for more details. 
 *
 * Contact information
 * ------------------------------------------
 * Thomas Jespersen, TKJ Electronics
 * Web      :  http://www.tkjelectronics.com
 * e-mail   :  thomasj@tkjelectronics.com
 * ------------------------------------------
 */
 
#ifndef MODULES_DEBUG_H
#define MODULES_DEBUG_H

#ifdef __cplusplus // for C++ usage

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cstring>
#include <string>

#define DEBUG(msg)	Debug::Message("DEBUG: ", __func__, msg)
#define ERROR(msg)	{Debug::Message("ERROR: ", __func__, msg); Debug::ErrorHandler();}

#define MAX_DEBUG_TEXT_LENGTH	250

class Debug
{

	public:
		Debug(void * com);
		~Debug();
	
		static void Message(const char * type, const char * functionName, const char * msg);
		static void Message(std::string type, const char * functionName, std::string msg);
		static void Message(const char * functionName, const char * msg);
		static void Message(const char * functionName, std::string msg);
		static void Message(const char * msg);
		static void Message(std::string msg);
		static void print(const char * msg);
		static void printf( const char *msgFmt, ... );
		static void ErrorHandler();

	private:
		void * com_; // LSPC object pointer


	public:
		static Debug * debugHandle;

};

#else  // for C usage

void Error_Handler(void);

#endif
	
	
#endif
