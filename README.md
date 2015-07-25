# avr-usart

An improved USART library for Arduino and ATmega MCUs supporting UART/USART communication.
It has a flash footprint between 2476 to 2946 bytes, depending on the number of used UART/USART ports.
It has a RAM usage of 56 to 194 bytes, without the buffer (the buffer is adjustable), depending on the number of used UART/USART ports.

Advantages
========
 - allows the usage of up to 4 UART/USART ports named USART, USART1, USART2 and USART3
 - the buffer can be adjusted (now uses dynamic memory allocation, but a version using stack comes soon)
 - allows to set frame length, stop bits number and parity
 - allows to send data stored with PROGMEM (data stored in program memory which allows to save RAM)
 - allows control over the buffer with the decision of when to delete the data which is read
 - works with Arduino IDE (v1.6.5+) but also with AvrStudio (v6.1+)
 
Supported MCUs and Arduino boards
========
 - ATmega328P and Arduino UNO v3
 - ATmega2560 and Arduino MEGA2560
 - currently under test... Arduino DUE

How to use it
========
This library requires C++11. To use it with the Arduino IDE, a compiler flag needs to be added. Navigate to your `arduino\hardware\arduino\avr` folder and edit the `platform.txt` file by adding the `-std=gnu++11` flag at the end of the line `compiler.cpp.flags=...`. Then save the file, restart Arduino IDE and you are ready to go.

If Arduino IDE is not used, the F_CPU constant needs to be defined. It specifies the MCU frequency (actually the crystal/oscilator frequency) in Hertz. For example, with a 16MHz oscillator/crystal this is defined as:
```
#define F_CPU 16000000L
```

You may use this site [AVR Baud Rate Calculator](http://wormfood.net/avrbaudcalc.php) to have an overview of the frequencies versus baud rates compatibility, since not all baud rates works well with all possible MCU speeds.

The global `USART/USARTn`, n = { ,, 2, 3}, objects allows you to use the USART communication. The following methods are available:

* `void setBaud( const unsigned long baud)` - set the Communication Baud Rate
* `void writeByte( uint8_t ub)` - write a single byte of data
* `void writeBytes( uint8_t* data, const uint16_t dataLen)` - write a set of bytes
* `void write( const char* data)` - write a String ( char pointer not String object!)
* `void writeFromPM( const char pmData[])` - write a String data which was stored in PROGMEM;
* `void write( long num)` - write a signed long integer number
* `void write( unsigned long num)` - write an unsigned long integer number
* `void write( int num)` - write a signed long integer number
* `void write( unsigned int num)` - write an unsigned integer number
* `uint16_t getMaxLength()` - returns the buffer max length
* `uint16_t getRxLostBytesNr()` - the number of lost bytes ( modulo 65535) from the last call of the `clear` method
* `uint8_t read( bool removeReadByte = true)` - read one byte from the RX buffer and remove it from the buffer if `removeReadbyte` is set to `true` (default = true)
* `uint16_t available()` - return the number of bytes available in the RX buffer
* `bool clear()` - clear the RX buffer
* `bool find( const char* data, bool removeReadByte = true)` - find a string in the RX buffer and remove all data until the found string ( or the entire buffer if not found) if the `removeReadbyte parameter` is set to `true` (default = true)
* `bool findFromPM( const char pmData[], bool removeReadByte = true)` - find a string (which was stored in PROGMEM) in the RX buffer and remove all data until the found string ( or the entire buffer if not found) if the `removeReadbyte parameter` is set to `true` (default = true);

There is some room for optimisation, but actually the footprint in both, Flash and RAM memory is a better than the Arduino Serial library and enhanced features are provided also (no critics to Arduino Serial library).

Arduino example code
========
```
#include "Usart.h"

void setup() {
  // start communication at a baud rate of 115200
  USART.begin( 115200);   
}

void loop() {
  // write some data
  USART.write( "Hello I am here, now send me the data!");
  
  // some delay waiting for data/response
  _delay_ms( 25);
  
  // read the response data
  while ( USART.available() > 0 ) {
    // here do something with the received data...
  }
}
```


AvrStudion example code
========
```
#include "Usart.h"

int main(void) { 
  // start communication at a baud rate of 115200
  USART.begin( 115200);   
  
  // write some data
  USART.write( "Hello I am here, now send me the data!");
  
  // some delay waiting for data/response
  _delay_ms( 25);
  
  // read the response data
  while ( USART.available() > 0 ) {
    // here do something with the received data...
  }
}
```
License
========
This project and its source code is released under [GPL](http://www.gnu.org/copyleft/gpl.html) license.
