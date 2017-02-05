/*
 * ResetInterface.h
 *
 *  Created on: 22Dec.,2016
 *      Author: podonoghue
 */

#ifndef SOURCES_RESETINTERFACE_H_
#define SOURCES_RESETINTERFACE_H_

#include "USBDM_MK.h"
#include "hardware.h"

/**
 * 3-State I/O for reset signal
 */
class ResetInterface {

private:
   using Direction = USBDM::GpioC<0>;
   using Data      = USBDM::GpioC<1>;

   static void callback(uint32_t status) {
      if ((Data::MASK & status) != 0) {
         Debug::toggle();
      }
   }

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
      Data::setIrq(USBDM::PinIrqFalling);
      Data::setCallback(callback);
      Data::enableNvicInterrupts(true);
   }
   /**
    * Drive signal high\n
    * Assumes series resistor for any I/O drive overlap
    */
   static void high() {
      Data::high();
      Data::setOut();
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
      Data::setOut();
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
      Data::setIn();
   }
   /**
    * Read value from receiver
    *
    * @return value on pin
    */
   static bool read() {
      Direction::low();
      Data::setIn();
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
   /**
    * Check if receiver input is low
    *
    * @return true if input is low
    */
   static bool isLow() {
      return !Data::read();
   }
   /**
    * Check if receiver input is high
    *
    * @return true if input is high
    */
   static bool isHigh() {
      return Data::read();
   }

};

#endif /* SOURCES_RESETINTERFACE_H_ */
