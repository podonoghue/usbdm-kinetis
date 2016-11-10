/*
 ============================================================================
 * @file    main.cpp (180.ARM_Peripherals)
 * @brief   Basic C++ demo using GPIO class
 *
 *  Created on: 10/1/2016
 *      Author: podonoghue
 ============================================================================
 */
#include <stdio.h>
#include "system.h"
#include "derivative.h"
#include "hardware.h"
#include "delay.h"
#include "usb.h"

namespace USBDM {
extern void idleLoop();
};

int main() {
   PRINTF("SystemBusClock  = %ld\n", SystemBusClock);
   PRINTF("SystemCoreClock = %ld\n", SystemCoreClock);

   USBDM::UsbImplementation::initialise();
   USBDM::idleLoop();
   for(;;) {
   }
   return 0;
}
