# avr-usart

While Arduino is a very nice Hardware and Software pack, sometimes it may not be the best solution for your project. Arduino libraries have to consider different hadrware compatibility issues, and some of them are just old and far from being optimal which may be an issue when the only available RAM is 1 or 2KBytes.
Also, sometime Arduino is a bit of overhead and you may like to only use the MCU, crystal and a few passive components in your project instead of the entire Arduino board (for size, energy consumption or simplicity reasons). Yes, sure, you can still burn the Arduino Bootloader, but why to lose aditional Flash space.

The `avr-usart` is an implementation of USART communication for AVR cips. Now it supports only one communication port (USART0), but it can and in time it will be expanded to support additional ones (as for example to be used with ATmega2560 MCUs which have more than one UART port). It allows to use PROGMEM data (data stored in program memory which allows to save RAM) by using special built-in methods (the methods that have their name ending with `FromPM`).

How to use it
========

There is a file named `base.h` which contains some utility methods and also defines the `F_CPU` required constant which determine the frequency at which the MCU runs (the internal or external crystal/oscilator frequency). This is required by the code to determine USART communication parameters and also is used by the AVR libraries (e.g. for `_delay_ms( time)`).
Open the file and replace the default value with the one corresponding to your hardware setup. 
NOTE: not every AVR MCU frequency is suitable to work with any USART Baud Rate. You may use this site [AVR Baud Rate Calculator](http://wormfood.net/avrbaudcalc.php) to have an overview of the frequencies versus baud rates compatibility.

The global `USART` object (uses a 256 Bytes buffer and 9600bps baud rate) allows you to use the USART communication. The following methods are available:
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

There is room for optimisation, but actually the footprint in both, Flash and RAM memory is a lot better than the Arduino Serial library (no critics to Arduino Serial library).

Example code
========
First edit the `base.h` file and set `F_CPU` to the correct value for your case. Remember, not every USART Baud Rate works with every MCU frequency, so first check: [AVR Baud Rate Calculator](http://wormfood.net/avrbaudcalc.php).

Then use the following C/C++ code: 

```
#include "Usart.h"

int main(void) { 
  // set Baud Rate to 115200 - default is 9600
  USART.setBaud( 115200);   
  
  // write some data
  USART.write( "Hello I am here, now send me the data!");
  
  // some delay waiting for data/response
  _delay_ms( 25);
  
  // read the response data
  while ( this->usart->available() > 0 ) {
    // here do something with the received data...
  }
}
```
Compile the code and burn it to your MCU (e.g. use AVR Studio). And that is all, enjoy and have fun!
