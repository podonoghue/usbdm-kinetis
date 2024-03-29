/**
 * @file      hardware.cpp (generated from MK20D5.usbdmHardware)
 * @version   1.3.0
 * @brief     Pin initialisation for MK20DX128VLF5
 *
 * *****************************
 * *** DO NOT EDIT THIS FILE ***
 * *****************************
 *
 * This file is generated automatically.
 * Any manual changes will be lost.
 */

#include "hardware.h"
// /HARDWARE/Includes not found

/**
 * Namespace enclosing USBDM classes
 */
namespace USBDM {

/**
 * @addtogroup USBDM_Group USBDM Peripheral Interface
 * @brief Hardware Peripheral Interface and library
 * @{
 */

/**
 * Startup code for C++ classes
 */
extern "C" void __attribute__((constructor)) cpp_initialise() {
   if constexpr (MapAllPinsOnStartup) {
      mapAllPins();
   }
}

#ifdef PORT_PCR_MUX
// /HARDWARE_CPP/Definitions not found

/**
 * Map all configured pins to peripheral signals.
 *
 * PCRs of allocated pins are set according to settings in Configure.usbdmProject
 *
 * @note Only the lower 16-bits of the PCR registers are initialised
 */
void mapAllPins() {
#if true


#endif

   enablePortClocks(USBDM::PORTA_CLOCK_MASK|USBDM::PORTB_CLOCK_MASK|USBDM::PORTC_CLOCK_MASK|USBDM::PORTD_CLOCK_MASK|USBDM::PORTE_CLOCK_MASK);
   PORTA->GPCLR = ForceLockedPins|0x8200UL|PORT_GPCLR_GPWE(0x0006UL);
   PORTA->GPCLR = ForceLockedPins|0x8700UL|PORT_GPCLR_GPWE(0x0009UL);
   PORTB->GPCLR = ForceLockedPins|0x8100UL|PORT_GPCLR_GPWE(0x0003UL);
   PORTD->GPCLR = ForceLockedPins|0x8100UL|PORT_GPCLR_GPWE(0x0080UL);

   if constexpr (ForceLockoutUnbondedPins) {
      PORTA->GPCLR = PinLock_Locked |0x0000UL|PORT_GPCLR_GPWE(0xCFC0UL); // Lockout unavailable pins
      PORTA->GPCHR = PinLock_Locked |0x0000UL|PORT_GPCHR_GPWE(0xFFF3UL); // Lockout unavailable pins
      PORTB->GPCLR = PinLock_Locked |0x0000UL|PORT_GPCLR_GPWE(0xFFF0UL); // Lockout unavailable pins
      PORTB->GPCHR = PinLock_Locked |0x0000UL|PORT_GPCHR_GPWE(0xFFF0UL); // Lockout unavailable pins
      PORTC->GPCLR = PinLock_Locked |0x0000UL|PORT_GPCLR_GPWE(0xF000UL); // Lockout unavailable pins
      PORTC->GPCHR = PinLock_Locked |0x0000UL|PORT_GPCHR_GPWE(0xFFFFUL); // Lockout unavailable pins
      PORTD->GPCLR = PinLock_Locked |0x0000UL|PORT_GPCLR_GPWE(0xFF00UL); // Lockout unavailable pins
      PORTD->GPCHR = PinLock_Locked |0x0000UL|PORT_GPCHR_GPWE(0xFFFFUL); // Lockout unavailable pins
      PORTE->GPCLR = PinLock_Locked |0x0000UL|PORT_GPCLR_GPWE(0xFFFCUL); // Lockout unavailable pins
      PORTE->GPCHR = PinLock_Locked |0x0000UL|PORT_GPCHR_GPWE(0xFFFFUL); // Lockout unavailable pins
   }

}
#endif 
/**
 * End group USBDM_Group
 * @}
 */
/*
 *  Static objects
 */


} // End namespace USBDM

