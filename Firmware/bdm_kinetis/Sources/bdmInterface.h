/*
 * BdmInterface.h
 *
 *  Created on: 12 Feb 2016
 *      Author: podonoghue
 */

#ifndef PROJECT_HEADERS_BDMINTERFACE_H_
#define PROJECT_HEADERS_BDMINTERFACE_H_
#include "system.h"
#include "derivative.h"
#include "hardware.h"
#include "delay.h"

/**
 * Represents a bidirectional signal e.g. 74LVC1T45 buffer
 * Assumes 2 signals: output/input, direction (0=>in, 1=>out)
 * It is desirable that there be a 1K series resistor on inOut
 *
 * @tparam inOut  GPIO connected to receiver input
 * @tparam dir    GPIO connected to direction control
 */
template<class inOut, class dir>
class Lvc1t45 {
private:

public:
   /**
    * Initialise Transceiver
    *
    * Initial state:
    *    Input
    */
   static void initialise() {
      // Set pin as input
      inOut::setInput();

      // Direction low => input
      dir::low();
      dir::setOutput();
   }
   /**
    * Drive signal high\n
    * Assumes series resistor for any I/O drive overlap
    */
   static void high() {
      inOut::high();
      inOut::setOutput();
      dir::high();
   }
   /**
    * Drive signal high
    *
    * @note Assumes driver already enabled
    */
   static void _high() {
      inOut::high();
   }
   /**
    * Drive signal low
    */
   static void low() {
      inOut::low();
      inOut::setOutput();
      dir::high();
   }
   /**
    * Drive signal low
    *
    * @note Assumes driver already enabled
    */
   static void _low() {
      inOut::low();
   }
   /**
    * Disable Transceiver (high-impedance)\n
    * Actually sets to input
    */
   static void highZ() {
      dir::low();
      inOut::setInput();
   }
   /**
    * Read value from receiver
    *
    * @return value on pin
    */
   static bool read() {
      dir::low();
      inOut::setInput();
      return inOut::read();
   }
   /**
    * Read value from receiver
    *
    * @return value on pin
    *
    * @note Assumes already set as input
    */
   static bool _read() {
      return inOut::read();
   }
};

#endif /* PROJECT_HEADERS_BDMINTERFACE_H_ */
