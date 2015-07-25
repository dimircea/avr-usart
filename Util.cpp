/*
 * Util.c
 *
 * Created: 02.02.2015 18:08:48
 *  Author: Mircea Diaconescu
 */ 
#include "Util.h"
/************************************************************************/
/* @operator                                                           */
/* Allow the usage of the "new" C++ operator                            */
/************************************************************************/
void * operator new( size_t size) {
  return malloc( size);
};

/************************************************************************/
/* @operator                                                           */
/* Allow the usage of the "delete" C++ operator                         */
/************************************************************************/
void operator delete( void * ptr) {
  free( ptr);
};

/************************************************************************/
/* @method                                                              */
/* Calculate the current MCU free memory value (in bytes)               */
/* @return the number of free RAM bytes available                       */
/************************************************************************/
uint16_t getFreeMCUMemory() {
  uint16_t free_memory;
  if ( (uint16_t)__brkval == 0)
  return ( ( (uint16_t)&free_memory) - ( (uint16_t)&__bss_end));
  else
  return ( ( (uint16_t)&free_memory) - ( (uint16_t)__brkval));
};

/************************************************************************/
/* @method                                                              */
/* Extract data from PROGMEM and return a RAM pointer to it.            */
/* NOTE: use 'free' within the caller code to deallocate memory         */
/*       after the result of this method is not needed anymore!         */
/* @param pmD                                                           */
/*          the PROGMEM data                                            */
/* @param length                                                        */
/*          optional (auto-computed if not provided) parameter that     */
/*          specifies the number of bytes to load from PROGMEM          */
/* @return the pointer to the RAM data                                  */
/************************************************************************/
char* getPMData( const char pmData[], size_t length) {
  char c = 0;
  const char* data = pmData;
  uint8_t len = length;
  // compute the length (number of bytes to load) if not 
  // specified by the function 'length' parameter
  if ( len == 0) {
    // read the data from PROGMEM for length computation
    while ( 0 != ( c = pgm_read_byte(data++))) {
      len++;
    }
    data = pmData;
  }  
  // load the data from PROGMEM and store it in RAM
  char* ramData = (char*)malloc( len + 1);
  while ( 0 != ( c = pgm_read_byte(data++))) {
    *(ramData) = c;
    ramData++;
  }
  *(ramData) = '\0';
  return (ramData - len);
};