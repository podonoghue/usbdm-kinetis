/**
 * @file      hardware.cpp (generated from MK20D5.usbdmHardware)
 * @version   1.2.0
 * @brief     Pin declarations for FRDM_K20D5
 *
 * *****************************
 * *** DO NOT EDIT THIS FILE ***
 * *****************************
 *
 * This file is generated automatically.
 * Any manual changes will be lost.
 */

#include "hardware.h"

namespace USBDM {

/**
 * Used to configure pin-mapping before 1st use of peripherals
 */
void mapAllPins() {
      enablePortClocks(PORTA_CLOCK_MASK|PORTB_CLOCK_MASK|PORTC_CLOCK_MASK|PORTD_CLOCK_MASK|PORTE_CLOCK_MASK);

      ((PORT_Type *)PORTA_BasePtr)->GPCHR = 0x0000UL|PORT_GPCHR_GPWE(0x000CUL);
      ((PORT_Type *)PORTA_BasePtr)->GPCLR = 0x0100UL|PORT_GPCLR_GPWE(0x3036UL);
      ((PORT_Type *)PORTA_BasePtr)->GPCLR = 0x0700UL|PORT_GPCLR_GPWE(0x0009UL);
      ((PORT_Type *)PORTB_BasePtr)->GPCHR = 0x0000UL|PORT_GPCHR_GPWE(0x000CUL);
      ((PORT_Type *)PORTB_BasePtr)->GPCLR = 0x0100UL|PORT_GPCLR_GPWE(0x000FUL);
      ((PORT_Type *)PORTB_BasePtr)->GPCHR = 0x0300UL|PORT_GPCHR_GPWE(0x0003UL);
      ((PORT_Type *)PORTC_BasePtr)->GPCLR = 0x0000UL|PORT_GPCLR_GPWE(0x0003UL);
      ((PORT_Type *)PORTC_BasePtr)->GPCLR = 0x0100UL|PORT_GPCLR_GPWE(0x0FFCUL);
      ((PORT_Type *)PORTD_BasePtr)->GPCLR = 0x0000UL|PORT_GPCLR_GPWE(0x0060UL);
      ((PORT_Type *)PORTD_BasePtr)->GPCLR = 0x0100UL|PORT_GPCLR_GPWE(0x0011UL);
      ((PORT_Type *)PORTD_BasePtr)->GPCLR = 0x0200UL|PORT_GPCLR_GPWE(0x000EUL);
      ((PORT_Type *)PORTE_BasePtr)->GPCLR = 0x0100UL|PORT_GPCLR_GPWE(0x0003UL);
}

} // End namespace USBDM

