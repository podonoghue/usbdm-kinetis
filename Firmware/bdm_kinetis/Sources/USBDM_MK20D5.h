/*! @file
    @brief This file contains hardware specific information and configuration.
    
    USBDM_MK20D5 - USBDM-SWD for bare MK20 chip on small proto-board
             
    Supports Kinetis targets \n

    @note DO NOT CHANGE THIS FILE \n
    If you need to create another configuration make a copy of this file
    under a new name and change Configure.h appropriately.
*/
#ifndef _CONFIGURE_H_
#define _CONFIGURE_H_

#include "transceiver.h"
#include "utilities.h"

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
 * GPIO for Target Vdd LED
 */
class TVddLed : public USBDM::GpioB<3> {
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
 * GPIO for Power LED\n
 * Dummy class
 */
class PowerLed {
public:
   /** Initialise power LED */
   static void initialise() {
   }
   /** Turn on power LED */
   static void on() {
   }
   /** Turn off power LED */
   static void off() {
   }
};

/**
 * ADC channel for target Vdd measurement
 */
class TargetVdd : USBDM::Adc0Channel<12> {
private:
   /**
    * 2:1 voltage divider on input
    */
   static constexpr int   externalDivider = 2;
   /**
    * Conversion factor for ADC reading to input voltage\n
    * 3.3V range, 8 bit conversion, voltage divider on input
    * V = ADCValue * scaleFactor
    */
   static constexpr float scaleFactor = (externalDivider*3.3)/((1<<8)-1);

   /**
    * Minimum working input voltage as an ADC reading \n
    * 1.5 V as ADC reading
    */
   static constexpr int   threshold = (int)(1.5/scaleFactor);

public:
   /**
    * Initialise target Vdd measurement
    */
   static void initialise() {
      enable();
      setResolution(USBDM::resolution_8bit_se);
   }
   /**
    * Read target Vdd
    *
    * @return Target Vdd as an integer in the range 0-255 => 0-5V
    */
   static int readRawVoltage() {
      return round(readAnalogue()*(externalDivider*3.3/5));
   }
   /**
    * Read target Vdd
    *
    * @return Target Vdd in volts as a float
    */
   static float readVoltage() {
      return readAnalogue()*scaleFactor;
   }
   /**
    * Check if target Vdd is present
    */
   static bool vddOK() {
      return readAnalogue()>threshold;
   }
};

/**
 * GPIO controlling some SWD related signals
 */
using Swd_enable = USBDM::GpioC<4>;

/**
 * 3-State I/O for reset signal
 */
using Reset = Lvc1t45<USBDM::GpioC<1>, USBDM::GpioC<0>>;

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
#ifdef SDA_POWER
#define HW_CAPABILITY       (CAP_RST_OUT|CAP_CDC|CAP_SWD_HW|CAP_BDM|CAP_RST_IN|CAP_SWD_HW|CAP_CORE_REGS|CAP_VDDCONTROL)
#define TARGET_CAPABILITY   (CAP_RST    |CAP_CDC|CAP_HCS08|CAP_ARM_SWD|CAP_VDDCONTROL)
#else
#define HW_CAPABILITY       (CAP_RST_OUT|CAP_CDC|CAP_SWD_HW|CAP_BDM|CAP_RST_IN|CAP_SWD_HW|CAP_CORE_REGS|CAP_VDDSENSE)
#define TARGET_CAPABILITY   (CAP_RST    |CAP_CDC|CAP_HCS08|CAP_ARM_SWD)
#endif

#define CPU  MK20D5

#endif // _CONFIGURE_H_

