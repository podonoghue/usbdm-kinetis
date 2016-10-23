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
 * It is desirable that there be a 1K series resistor on Data
 *
 * @tparam Data  GPIO connected to transceiver input/output
 * @tparam Direction    GPIO connected to direction control
 */
template<class Data, class Direction>
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
      Data::setInput();

      // Direction low => input
      Direction::low();
      Direction::setOutput();
   }
   /**
    * Drive signal high\n
    * Assumes series resistor for any I/O drive overlap
    */
   static void high() {
      Data::high();
      Data::setOutput();
      Direction::high();
   }
   /**
    * Drive signal high
    *
    * @note Assumes driver already enabled
    */
   static void _high() {
      Data::high();
   }
   /**
    * Drive signal low
    */
   static void low() {
      Data::low();
      Data::setOutput();
      Direction::high();
   }
   /**
    * Drive signal low
    *
    * @note Assumes driver already enabled
    */
   static void _low() {
      Data::low();
   }
   /**
    * Disable Transceiver (high-impedance)\n
    * Actually sets to input
    */
   static void highZ() {
      Direction::low();
      Data::setInput();
   }
   /**
    * Read value from receiver
    *
    * @return value on pin
    */
   static bool read() {
      Direction::low();
      Data::setInput();
      return Data::read();
   }
   /**
    * Read value from receiver
    *
    * @return value on pin
    *
    * @note Assumes already set as input
    */
   static bool _read() {
      return Data::read();
   }
};

#endif /* PROJECT_HEADERS_BDMINTERFACE_H_ */
