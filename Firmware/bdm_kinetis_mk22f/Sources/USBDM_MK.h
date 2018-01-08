/*! @file
    @brief This file contains hardware specific information and configuration.

    USBDM_MK20D5 - USBDM-SWD for bare MK20 chip on small proto-board

    Supports Kinetis targets \n

    @note DO NOT CHANGE THIS FILE \n
    If you need to create another configuration make a copy of this file
    under a new name and change Configure.h appropriately.
*/
#ifndef _USBDM_MK_H_
#define _USBDM_MK_H_

#include "utilities.h"
#include "hardware.h"

/**
 * GPIO for Activity LED
 */
class UsbLed : public USBDM::GpioD<7> {
public:
   /** Initialise activity LED */
   static void initialise() {
      setOutput();
      off();
   }
   /** Turn on activity LED */
   static void on() {
      high();
   }
   /** Turn off activity LED */
   static void off() {
      low();
   }
};

/**
 * GPIO for Debug pin
 */
class Debug : public USBDM::GpioB<0> {
public:
   /** Initialise debug pin */
   static void initialise() {
      setOutput();
      low();
   }
};

/**
 * Hardware ID interface
 *
 * This uses the ADC to measure an external voltage divider
 */
class HardwareId : USBDM::Adc0Channel<12> {
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
      setResolution(USBDM::AdcResolution_8bit_se);
      return round(Adc0Channel::readAnalogue()/17.0);
   }
};

/**
 * GPIO controlling some interface signals (SWD, UART-TX)
 */
using InterfaceEnable = USBDM::GpioC<4>;

#include <swd.h>

//==========================================================================================
// USB Serial Number
#ifdef UNIQUE_ID
#define SERIAL_NO           "USBDM-MK-%lu"
#else
#define SERIAL_NO           "USBDM-MK-0001"
#endif
#define PRODUCT_DESCRIPTION "USBDM ARM-SWD for MK"
#define MANUFACTURER        "pgo"

//==========================================================================================
// Capabilities of the hardware - used to enable/disable appropriate code
//
#define HW_CAPABILITY       (CAP_RST_OUT|CAP_RST_IN|CAP_CDC|CAP_SWD_HW|CAP_BDM|CAP_SWD_HW|CAP_CORE_REGS|CAP_VDDCONTROL|CAP_VDDSENSE)
#define TARGET_CAPABILITY   (CAP_RST               |CAP_CDC|CAP_HCS08|CAP_HCS12|CAP_S12Z|CAP_CFV1|CAP_ARM_SWD|CAP_VDDCONTROL)

#define CPU  MK20D5

#define VERSION_HW  (HW_ARM+TARGET_HARDWARE)

#endif // _USBDM_MK_H_

