/**
 * Define the USART/Serial communication.
 *
 * @file Usart.h
 * @version 1.0
 * @created 27.01.2015 17:37:21
 * @author: Mircea Diaconescu
 */ 

#include "Base.h"
#include <avr/interrupt.h>

#ifndef USART_H_
#define USART_H_

extern "C" void USART_TX_vect(void) __attribute__ ((signal));
extern "C" void USART_RX_vect(void) __attribute__ ((signal));
extern "C" void USART_UDRE_vect(void) __attribute__ ((signal));

class Usart {
  public:
	  Usart( const uint16_t rxBuffLen);
    ~Usart();
    void setBaud( const unsigned long baud);
    void writeByte( uint8_t ub);
	  void writeBytes( uint8_t* data, const uint16_t dataLen);
    void write( const char* data);
    void write( long num);
    void write( unsigned long num);
    void write( int num);
    void write( unsigned int num);
    void writeFromPM( const char pmData[]);
    uint16_t getMaxLength();
    uint16_t getRxLostBytesNr();
	  uint8_t read( bool removeReadByte = true);
    uint16_t available();
    bool clear();
    bool find( const char* data, bool removeReadByte = true);
    bool findFromPM( const char pmData[], bool removeReadByte = true);
    // allow USART_RX_vect to access private members of this class
    friend void USART_RX_vect( void);
  private:
    volatile uint8_t* rxBuffStart;
    volatile uint8_t* rxBuffEnd;
    volatile uint8_t* rxDataStart;
    volatile uint8_t* rxDataTStart;
    volatile uint8_t* rxDataEnd;
    volatile uint16_t rxLength;
    volatile uint16_t rxLostBytesNr;
    volatile uint16_t rxMaxLength;
    unsigned long baud;
};
extern Usart USART;
#endif