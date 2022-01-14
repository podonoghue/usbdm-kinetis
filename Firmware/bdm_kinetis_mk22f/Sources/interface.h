/*
 * interface.h
 *
 *  Created on: 21 May 2021
 *      Author: peter
 */

#ifndef SOURCES_INTERFACE_H_
#define SOURCES_INTERFACE_H_
#include "hardware.h"
#include "smc.h"

/**
 * GPIO for Activity LED
 */
class UsbLed : public USBDM::Usb_Led { // public USBDM::GpioD<7, USBDM::ActiveLow> {
public:
   /** Initialise activity LED */
   static void initialise() {
     setOutput();
   }
};

/**
 * GPIO for Debug pin
 */
class Debug : public USBDM::TPa1 { // USBDM::GpioB<0, USBDM::ActiveHigh> {
public:
   /** Initialise debug pin */
   static void initialise() {
      setOutput();
   }
};

/**
 * GPIO controlling some interface signals (SWD, UART-TX)
 */
class InterfaceEnable : public USBDM::Swd_Enable { // public USBDM::GpioC<4, USBDM::ActiveHigh> {
public:
   /** Initialise pin as output driving inactive level */
   static void initialise() {
     setOutput();
   }
      // Todo - fix after removing const on typedefs
   /** Initialise pin as output driving inactive level */
   static void on() {
      USBDM::Swd_Enable::on();
   }
};

/**
 * Hardware ID interface
 *
 * This uses the ADC to measure an external voltage divider
 */
class HardwareId : public USBDM::Identify { // USBDM::Adc0::Channel<19> {
public:
   /**
    * Get hardware ID
    *
    * @return Value in range 0-15 reflecting hardware ID strapping resistors
    *
    * @note Assumes ADC enabled elsewhere
    *
    * Resistor Ratios:
    * ID   R10/R13
    * ===============
    *  0 = 100K/0R or open/0R
    *  1 = 100K/7.15K
    *  2 = 100K/15.4K
    *  3 = 100K/24.9K
    *  4 = 100K/36.5K
    *  5 = 100K/49.9K
    *  6 = 100K/66.5K
    *  7 = 100K/86.6K
    *  8 = 86.6K/100K
    *  9 = 66.5K/100K
    * 10 = 49.9K/100K
    * 11 = 36.5K/100K
    * 12 = 24.9K/100K
    * 13 = 15.4K/100K
    * 14 = 7.15R/100K
    * 15 = 0R/100K or 0R/open
    */
   static int getId() {
      // Todo - fix after removing const on typedefs
      USBDM::Identify::OwningAdc::setResolution(USBDM::AdcResolution_8bit_se);
      return round(USBDM::Identify::readAnalogue()/17.0);
   }
};

#endif /* SOURCES_INTERFACE_H_ */
