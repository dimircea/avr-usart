/**
 * Implement the Usart class.
 *
 * @file Usart.cpp
 * @version 1.0 
 * @created 29.01.2015 12:27:39
 * @author: Mircea Diaconescu
 */ 

#include "Usart.h"

/************************************************************************/
/* @constructor                                                         */
/* Create a Usart object                                                */
/* @param rxBuffMaxLen                                                  */
/*          the maximum size (number of bytes) stored by the RX buffer  */
/************************************************************************/
Usart::Usart( const uint16_t rxBuffMaxLen) {
  this->rxBuffStart = (uint8_t*)malloc( rxBuffMaxLen);
  this->rxBuffEnd = this->rxBuffStart + rxBuffMaxLen;
  this->rxDataStart = this->rxBuffStart;
  this->rxDataTStart = this->rxBuffStart;
  this->rxDataEnd = this->rxBuffStart;
  this->rxLength = 0;
  this->rxLostBytesNr = 0;
  this->rxMaxLength = rxBuffMaxLen;
  // default: 9600Kbps = 1200KBps = 1.2KB/ms = 0.834ms/KB
  this->setBaud( 9600);
};

/************************************************************************/
/* @destructor                                                          */
/* Destroy the Usart object by freeing up the allocated memory          */
/************************************************************************/
 
Usart::~Usart() {
  delete this->rxBuffStart; 
};

/************************************************************************/
/* @method                                                              */
/* Check if any data is stored in the RX buffer                         */
/* @return the number of bytes available in the RX buffer (0 if empty)  */
/************************************************************************/
uint16_t Usart::available() {
  return this->rxLength;
};

/************************************************************************/
/* @method                                                              */
/* Clear the RX buffer - reset buffer pointers to their initial state   */
/* @return true so it can be used in logical expressions with ease      */
/************************************************************************/
bool Usart::clear() {
  bool txEnabled =  (UCSR0B & (1 << TXEN0)) > 0;
  // disable USART data receiving until clearing the buffer
  if ( txEnabled) {
    UCSR0B &= ~(1 << TXEN0);
  }
  this->rxDataStart = this->rxBuffStart;
  this->rxDataTStart = this->rxBuffStart;
  this->rxDataEnd = this->rxBuffStart;
  this->rxLength = 0;
  this->rxLostBytesNr = 0;
  // if TX was enabled when entering this method
  // then enable it back after cleared the buffer
  if ( txEnabled) {
    UCSR0B |= (1 << TXEN0);
  }
  return true;
};

/************************************************************************/
/* @method                                                              */
/* Get the maximum length of the RX buffer                              */
/* @return the maximum number of bytes stored by the RX buffer          */
/************************************************************************/
uint16_t Usart::getMaxLength() {
  return this->rxMaxLength;
};

/************************************************************************/
/* @method                                                              */
/* Get the number of bytes which were lost because TX buffer overrun    */
/* The number of bytes are computed from the last time when the buffer  */
/* was cleared (by calling the 'clear' method.                          */
/* NOTE: the value is represented as a 16 bits unsigned integer!        */
/* @return the number of RX bytes lost because buffer overrun           */
/************************************************************************/
uint16_t Usart::getRxLostBytesNr() {
  return this->rxLostBytesNr;
};

/************************************************************************/
/* @method                                                              */
/* Set communication baud rate against the CPU/MCU frequency            */
/* @param baud                                                          */
/*          the baud rate (in bps, e.g. 9600)                           */
/* @param fCpu                                                          */
/*          the CPU/MCU frequency in Hz (e.g. 16000000 for 16MHz)       */
/************************************************************************/
void Usart::setBaud( const unsigned long baud) {
  // According with the data-sheet, for Asynchronous mode, the baud rate
  // is computed as follows: BAUD = Fosc / ( 16 * ( UBRRn + 1))
  // so UBRRn = (Fosc / 16 * BAUD) - 1, where Fosc represents
  // the current speed of the MCU ( i.e. the Crystal/Oscillator frequency in Hz)
  uint8_t bRate = F_CPU/16/baud -1;
  this->baud = baud;
  // set the USART High register byte
  UBRR0H = ( bRate >> 8);
  // set the USART Low register byte
  UBRR0L =  bRate;
  // set USART transmission speed: normal speed ( U2X0 = 0);
  UCSR0A &= ~(1 << U2X0);
  // enable USART transmission
  UCSR0B |= (1 << TXEN0);
  // enable USART reception & USART_RX_vect interrupt
  UCSR0B |= (1 << RXEN0) | (1 << RXCIE0);
  // set asynchronous USART mode (UMSEL00 = 0; UMSEL01 = 0;)
  UCSR0C &= ~(1 << UMSEL00) | ~(1 << UMSEL01);
  // set data transmission mode: 8-bit (UCSZ00 = 1; UCSZ01 = 1; UCSZ02 = 0;)
  UCSR0C |=  (1 << UCSZ00) | (1 << UCSZ01);
  UCSR0B &= ~(1 << UCSZ02); // UCSZ02 bit is located in UCSR0B registry!
  // set data transmission mode: one stop bit ( USBS0 = 0;)
  UCSR0C &= ~( 1 << USBS0);
  // set data transmission mode: no parity ( UPM00 = 0; UPM01 = 0;)
  UCSR0C &= ~(1 << UPM00) | ~(1 << UPM01);
  // enable interrupts
  sei();
};

/************************************************************************/
/* @method                                                              */
/* Read the next byte from the RX buffer                                */
/* NOTE: call this method only after checking that the buffer is not    */
/*       empty, i.e., by calling the 'available' method, otherwise the  */
/*       returned value will be 0!                                      */
/* @param removeReadByte                                                */
/*          flag allowing to specify if the byte is deleted from buffer */
/* @return the next byte from the buffer                                */
/************************************************************************/
uint8_t Usart::read( bool removeReadByte) {
  uint8_t data = 0;
  // return 0 if the buffer is empty
  if ( this->rxLength == 0) {
    return 0;
  }
  if ( removeReadByte) {
    data = *(this->rxDataStart);
    this->rxDataStart++;
    if ( this->rxDataStart == this->rxBuffEnd) {
      this->rxDataStart = this->rxBuffStart;
    }
    this->rxLength--;
  } else {
    data = *(this->rxDataTStart);
    this->rxDataTStart++;
    if ( this->rxDataTStart == this->rxBuffEnd) {
      this->rxDataTStart = this->rxBuffStart;
    }
  }
  return data;
};

/************************************************************************/
/* @method                                                              */
/* Write a new byte in the UDR0 register, for being transmitted         */
/* @param ub                                                            */
/*          the unsigned byte to write                                  */
/************************************************************************/
void Usart::writeByte( uint8_t ub) {
  // Wait for URD0 register to be ready for the next data byte.
  // The URDE0 bit from the UCSR0A registry is set when USART 
  // data register is empty and new data can be transmitted
 	loop_until_bit_is_set( UCSR0A, UDRE0);
  // write the next data byte
  UDR0 = ub;
};

/************************************************************************/
/* @method                                                              */
/* Write a string to USART where the string source is PROGMEM           */
/* @param pmData                                                        */
/*          the PROGMEM source                                          */
/************************************************************************/
void Usart::writeFromPM( const char pmData[]) {
  char c = 0;
  while ( 0 != ( c = pgm_read_byte(pmData++))) {
    this->writeByte( c);
  } 
};
  
/************************************************************************/
/* @method                                                              */
/* Write a set of unsigned bytes value to USART output.                 */
/* @param data                                                          */
/*          the pointer referencing the set of bytes to write           */
/* @param dataLen                                                       */
/*          the length (number of bytes) of the data to be transmitted  */
/************************************************************************/
void Usart::writeBytes( uint8_t* data, const uint16_t dataLen) {
  for ( uint16_t i = 0; i < dataLen; i++) {
    this->writeByte( *(data++));
  }
};

/************************************************************************/
/* @method                                                              */
/* Write a string ( '\0' terminated char array/pointer) to USART.       */
/* Maximum allowed length for the string depends on the memory size!    */
/* @param data                                                          */
/*          the pointer referencing the string to write                 */
/************************************************************************/
void Usart::write( const char* data) {
  for ( const char* s = data; *s; ++s) {
    this->writeByte( *s);
  }
};

/************************************************************************/
/* @method                                                              */
/* Write a signed long number to USART.                                 */
/* @param num                                                           */
/*          the number to write                                         */
/************************************************************************/
void Usart::write( long num) {
  if ( num < 0) {
    this->writeByte( '-');
  }
  this->write( ( unsigned long) num);
}

/************************************************************************/
/* @method                                                              */
/* Write an unsigned long number to USART.                              */
/* @param num                                                           */
/*          the number to write                                         */
/************************************************************************/
void Usart::write( unsigned long num) {
  // max digits number for signed/unsigned long is 10
  char digits[10];
  uint8_t len = 0;
  do {
    digits[len] = '0' + ( num % 10);
    len++;
  } while ( num /= 10);
  while ( len > 0) {
     this->writeByte( digits[--len]);  
  }
}

/************************************************************************/
/* @method                                                              */
/* Write a signed integer number to USART.                              */
/* @param num                                                           */
/*          the number to write                                         */
/************************************************************************/
void Usart::write( int num) {
  this->write((long) num);
}

/************************************************************************/
/* @method                                                              */
/* Write an unsigned integer number to USART.                           */
/* @param num                                                           */
/*          the number to write                                         */
/************************************************************************/
void Usart::write( unsigned int num) {
  this->write(( unsigned long) num);
}

/************************************************************************/
/* @method                                                              */
/* Find data in the USART buffer                                        */
/* This method clears the USART buffer up to, and including the data,   */
/* if the data is found, otherwise clears all the data from the buffer  */ 
/* NOTE: the maximum data length is 255!                                */
/*                                                                      */
/* @param data                                                          */
/*          the data to search for in the buffer                        */
/* @return true if data was found, false otherwise                      */
/************************************************************************/
bool Usart::find( const char* data, bool removeReadByte) {
  uint8_t pos = 0, len = 0, avl = 0;
  const char* s;
  for ( s = data; *s; ++s, len++); 
  // null search data..
  if ( *data == 0) {
    return false;
  }
  // searching the TX buffer as long as it contains data
  avl = this->available();
  this->rxDataTStart = this->rxDataStart;
  while ( avl > 0) {
    // char/byte match on current position
    if ( *(data + pos) == this->read( removeReadByte)) {
      pos++;
    } 
    // no match for current position
    else {
      pos = 0;
    }
    // full data match!
    if ( pos == len) {
      return true;
    }
    avl--;
  }
  // no match if this point was reached
  return false;
};

/************************************************************************/
/* @method                                                              */
/* Find data in the USART buffer - the data to find is extracted        */
/* from PROGMEM ( program memory area)                                  */
/* This method clears the USART buffer up to, and including the data,   */
/* if the data is found, otherwise clears all the data from the buffer  */
/* NOTE: the maximum data length is 255!                                */
/*                                                                      */
/* @param pmData                                                        */
/*          the PROGMEM data to search for in the buffer                */
/* @return true if data was found, false otherwise                      */
/************************************************************************/
bool Usart::findFromPM( const char pmData[], bool removeReadByte) {
  char c = 0;
  const char* data = pmData;
  char* ramData = nullptr;
  uint8_t len = 0;
  bool result = false;
  // read the data from PROGMEM for length computation
  while ( 0 != ( c = pgm_read_byte(data++))) {
    len++;
  }  
  // load the data from PROGMEM and store it in RAM
  ramData = (char*)malloc( len + 1);
  data = pmData;
  while ( 0 != ( c = pgm_read_byte(data++))) {
    *(ramData) = c;
    ramData++;
  } 
  *(ramData) = '\0';
  ramData -= len;
  result = this->find( ramData, removeReadByte);
  free( ramData); 
  return result;
};

/************************************************************************/
/* @interrupt                                                           */
/* Define the interrupt vector for data reception                       */
/************************************************************************/
ISR(USART_RX_vect) {
  *(USART.rxDataEnd) = UDR0;
  USART.rxDataEnd++;
  if ( USART.rxDataEnd == USART.rxBuffEnd) {
    USART.rxDataEnd = USART.rxBuffStart;
  }
  if ( USART.rxLength == USART.rxMaxLength) {
    USART.rxLostBytesNr++;
    if ( ( USART.rxDataStart + 1) == USART.rxBuffEnd) {
      USART.rxDataStart = USART.rxBuffStart;
    } else {
      USART.rxDataStart++;
    }
  } else {
    USART.rxLength++;
  }
};

// define the global USART object
// used to read/write via USART/Serial
// default RX buffer size is 256 bytes
Usart USART( 256);