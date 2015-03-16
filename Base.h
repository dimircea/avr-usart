/*
 * Define a collections of methods and utilities
 * which are to be used with most applications.
 *
 * @file Base.h
 * @version 1.0
 * @created 31.01.2015 12:19:44
 * @author: Mircea Diaconescu
 */ 



#ifndef BASE_H_
#define BASE_H_

#define F_CPU 9216000

#include <stdlib.h>
#include <stdint.h>
#include <avr/pgmspace.h>

extern unsigned int __bss_end;
extern unsigned int __heap_start;
extern void *__brkval;

void* operator new( size_t size);
void operator delete( void* ptr);
uint16_t getFreeMCUMemory();
char* getPMData( const char pmD[], size_t length = 0);

#endif