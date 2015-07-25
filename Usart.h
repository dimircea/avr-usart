/**
 * Define the USART/Serial communication.
 *
 * @file Usart.h
 * @version 1.0
 * @created 27.01.2015 17:37:21
 * @author: Mircea Diaconescu
 */ 

#include "Util.h"
#include <avr/interrupt.h>

#ifndef USART_H_
#define USART_H_

#if defined(UBRRH)
  #define HAS_USART
#endif
#if defined(UBRR0H)
  #define HAS_USART0
#endif
#if defined(UBRR1H)
  #define HAS_USART1
#endif
#if defined(UBRR2H)
  #define HAS_USART2
#endif
#if defined(UBRR3H)
  #define HAS_USART3
#endif

/****************************************************************/
/* Enumeration of communication modes supported by USART        */
/****************************************************************/
enum class UsartModeEL {
  ASYNC_NORMAL = 0,
  ASYNC_DOUBLE_SPEED = 1,
  SYNC_MASTER = 2
};

/****************************************************************/
/* Enumeration defining communication parity modes              */
/****************************************************************/
enum class UsartParityEL {
  NONE = 0,
  EVEN = 2,
  ODD = 3
};

/****************************************************************/
/* Enumeration defining communication stop bits                 */
/****************************************************************/
enum class UsartStopBitEL {
  ONE = 1,
  TWO = 2
};

/****************************************************************/
/* Enumeration defining communication frame length              */
/****************************************************************/
enum class UsartFrameLengthEL {
  FIVE_BITS = 5,
  SIX_BITS = 6,
  SEVEN_BITS = 7,
  EIGHT_BITS = 8,
  NINE_BITS = 9
};

/****************************************************************/
/* Define the Usart class implementing the USART communication  */
/****************************************************************/
class Usart {
  public:
    Usart( uint8_t portNo);
    ~Usart();
    void setFrameLength( UsartFrameLengthEL frameLength);
    UsartFrameLengthEL getFrameLength();
    void setStopBit( UsartStopBitEL stopBit);
    UsartStopBitEL getStopBit();
    void setParity( UsartParityEL parity);
    UsartParityEL getParity();
    void begin( const uint32_t baud, uint16_t rxBuffMaxLen, UsartModeEL mode = UsartModeEL::ASYNC_DOUBLE_SPEED);
    inline void begin( const uint32_t baud, UsartModeEL mode = UsartModeEL::ASYNC_DOUBLE_SPEED) { begin( baud, 64, mode);};
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
    // friend operation which deals with USARTn_RX_vector(s) interrupts
    // using friend operation allows to access private fields of this class
    friend void rxVector( uint8_t udr, Usart usart);
  private:
    // Define the ring buffer pointer.
    // Must be volatile because write by interrupts.
    volatile uint8_t* rxBuffStart;
    volatile uint8_t* rxBuffEnd;
    volatile uint8_t* rxDataStart;
    volatile uint8_t* rxDataTStart;
    volatile uint8_t* rxDataEnd;
    // Current number of bytes stored in the buffer.
    // Must be volatile because write by interrupts.
    volatile uint16_t rxLength;
    // The number of bytes lost from the buffer because 
    // it was not read and the buffer received more data
    // that it can store without overriding existing data.
    // Must be volatile because write by interrupts.
    volatile uint16_t rxLostBytesNr;
    // maximum receive buffer length
    uint16_t rxMaxLength;
    // communication baud rate in bits per second (e.g., 115200bps = 115.2kbps)
    uint32_t baud;
    // the communication port number, 0 to 3
    uint8_t portNo;    
    
    /** MCU used registry */
    volatile uint8_t *udr;
    volatile uint8_t *ucsra;
    volatile uint8_t *ucsrb;
    volatile uint8_t *ucsrc;
    volatile uint8_t *ubrrh;
    volatile uint8_t *ubrrl;
    /* MCU dependent registry bits (positions) */
    uint8_t txen;
    uint8_t rxen;
    uint8_t udre;
    uint8_t u2x;
    uint8_t rxcie;
    uint8_t usbs;
    uint8_t upm0;
    uint8_t upm1;
    uint8_t ucsz0;
    uint8_t ucsz1;
    uint8_t ucsz2;
    uint8_t umsel0;
    uint8_t umsel1;
};

#if defined(HAS_USART) || defined(HAS_USART0) 
  extern Usart USART;
#endif
#if defined(HAS_USART1)
  extern Usart USART1;
#endif
#if defined(HAS_USART2)
  extern Usart USART2;
#endif
#if defined(HAS_USART3)
  extern Usart USART3;
#endif
#endif