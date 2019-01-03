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
 
#include "Debug.h"
 
// Necessary to export for compiler such that the Error_Handler function can be called by C code
extern "C" __EXPORT void Error_Handler(void);

Debug::Debug()
{
	
}

Debug::~Debug()
{
	
}

void Debug::Message(const char * msg)
{
	
}

void Debug::Message(std::string msg)
{
	
}

void Debug::Message(const char * functionName, const char * msg)
{
	
}

void Debug::Message(const char * functionName, std::string msg)
{
	
}

void Debug::Message(const char * type, const char * functionName, const char * msg)
{

}

void Debug::Message(std::string type, const char * functionName, std::string msg)
{

}

void Debug::ErrorHandler()
{
	uint32_t i;
	while (1)
	{
		i++;
	}
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
	Debug::ErrorHandler();
}
