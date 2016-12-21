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
 * GPIO for Debug pin
 */
class Debug : public USBDM::GpioB<1> {
public:
   /** Initialise debug pin */
   static void initialise() {
      setOutput();
      low();
   }
};

/**
 * GPIO controlling some interface signals (SWD, UART-TX)
 */
using InterfaceEnable = USBDM::GpioC<4>;

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
#define HW_CAPABILITY       (CAP_RST_OUT|CAP_RST_IN|CAP_CDC|CAP_SWD_HW|CAP_BDM|CAP_SWD_HW|CAP_CORE_REGS|CAP_VDDCONTROL|CAP_VDDSENSE)
#define TARGET_CAPABILITY   (CAP_RST               |CAP_CDC|CAP_HCS08|CAP_HCS12|CAP_S12Z|CAP_CFV1|CAP_ARM_SWD|CAP_VDDCONTROL)

#define CPU  MK20D5

#endif // _CONFIGURE_H_

