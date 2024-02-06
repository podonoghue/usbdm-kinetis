/*
 * @file console.cpp (180.ARM_Peripherals/Startup_Code/console.cpp)
 *
 *  Created on: 14/04/2013
 *      Author: pgo
 */

#include "derivative.h"
#include "system.h"
#include "pin_mapping.h"
#include "console.h"

 /*
  * *****************************
  * *** DO NOT EDIT THIS FILE ***
  * *****************************
  *
  * This file is generated automatically.
  * Any manual changes will be lost.
  */

namespace USBDM {

#if USE_CONSOLE

/**
 * Print simple log message to console
 *
 * @param msg Message to print
 */
void log_error(const char *msg) {
   (void)msg;
   console.WRITELN(msg);
}

/**
 * @addtogroup CONSOLE_Group Console
 * @brief Console serial interface
 * @{
 */

// Console instance
Console console;

   /*
    * Initialises the Console
    */
   extern "C"
   void console_initialise() {
      console.initialise();
      console.setBaudRate(defaultBaudRate);
      console.setEcho();
      console.configureAllPins();
   }
   


/*
 * Set Console baud rate
 *
 * @param baudRate - the baud rate to use
 */
extern "C"
void console_setBaudRate(int baudRate = defaultBaudRate) {
   console.setBaudRate(UartBaudRate(baudRate));
}

/*
 * Transmits a single character to Console
 *
 * @param ch - character to send
 */
extern "C"
void console_txChar(int ch) {
   console.write((char)ch);
}

/*
 * Receives a single character from Console (blocking)
 *
 * @return - character received
 */
extern "C"
int console_rxChar(void) {
   return console.readChar();
}

/**
 * @}
 */
#else

Console console;

#endif /* USE_CONSOLE */

} // End namespace USBDM
