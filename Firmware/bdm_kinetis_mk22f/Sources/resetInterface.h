/*
 * ResetInterface.h
 *
 *  Created on: 22Dec.,2016
 *      Author: podonoghue
 */

#ifndef SOURCES_RESETINTERFACE_H_
#define SOURCES_RESETINTERFACE_H_

#include "hardware.h"

/**
 * Reset signal
 * Bidirection control
 * Reset event detection
 */
class ResetInterface {

private:

   // Controls direction of transceiver
   using Direction = USBDM::Reset_Dir; //USBDM::GpioC<0>;

   // Transceiver data (in/out)
   using Data      = USBDM::Reset_IO; // USBDM::GpioC<1>;

   static bool  fResetActivity;

   /**
    * Callback used to monitor reset events
    *
    * @param status
    */
   static void callback(uint32_t status) {

      // Check if RESET pin event and pin is low
      if ((Data::BITMASK & status) && isLow()) {
         fResetActivity = true;
      }
   }

public:
   /**
    * Initialise Transceiver
    *
    * Initial state:
    *    Reset signal Input/HighZ
    *    Reset monitoring enabled
    */
   static void initialise() {

      // Enable pins
      Data::setInput();
      Direction::setOutput();

      // Initially target reset is not driven
      highZ();

      // IRQ on falling edge - reset detection
      Data::setPinAction(USBDM::PinAction_IrqFalling);
      Data::setPinCallback(callback);
      Data::enableNvicInterrupts(USBDM::NvicPriority_Normal);

      fResetActivity = false;
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
    * Drive signal high
    */
   static void high() {
      Data::high();
      Data::setOut();
      Direction::high();
   }
   /**
    * Disable Transceiver (high-impedance)\n
    * Actually sets to input
    */
   static void highZ() {
      // Pulse RST=H (speed-up)
      Data::high();
      Data::setOut();
      Direction::high();
      // 3-state
      Direction::low();
      Data::setIn();
   }

   /**
    * Read value from receiver
    *
    * @return value on pin or driven value if output
    *
    * @note Assumes already set as input or returns driven value
    */
   static bool read() {
      return Data::read();
   }

   /**
    * Check if receiver input is low
    *
    * @return true if input is low
    *
    * @note Assumes already set as input or returns driven value
    */
   static bool isLow() {
      return !Data::read();
   }

   /**
    * Check if receiver input is high
    *
    * @return true if input is high
    *
    * @note Assumes already set as input or returns driven value
    */
   static bool isHigh() {
      return Data::read();
   }

   /**
    * Check and clear reset activity flag
    *
    * @return True  Reset has been active since last polled
    * @return False Reset has not been active since last polled
    */
   static bool resetActivity() {
      bool temp = fResetActivity;
      fResetActivity = false;
      return temp;
   }
};

#endif /* SOURCES_RESETINTERFACE_H_ */
