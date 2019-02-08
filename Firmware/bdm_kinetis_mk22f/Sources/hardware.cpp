/**
 * @file      hardware.cpp (generated from MK22F51212.usbdmHardware)
 * @version   1.2.0
 * @brief     Pin initialisation for MK22FN512VLH12
 *
 * *****************************
 * *** DO NOT EDIT THIS FILE ***
 * *****************************
 *
 * This file is generated automatically.
 * Any manual changes will be lost.
 */

#include "hardware.h"

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
 * Used to configure pin-mapping before 1st use of peripherals
 */
void mapAllPins() {
#ifdef PCC_PCCn_CGC_MASK
      PCC->PCC_PORTA = PCC_PCCn_CGC_MASK;
      PCC->PCC_PORTB = PCC_PCCn_CGC_MASK;
      PCC->PCC_PORTC = PCC_PCCn_CGC_MASK;
      PCC->PCC_PORTD = PCC_PCCn_CGC_MASK;
      PCC->PCC_PORTE = PCC_PCCn_CGC_MASK;
#else
      enablePortClocks(PORTA_CLOCK_MASK|PORTB_CLOCK_MASK|PORTC_CLOCK_MASK|PORTD_CLOCK_MASK|PORTE_CLOCK_MASK);
#endif
      PORTA->GPCLR = 0x0200UL|PORT_GPCLR_GPWE(0x0006UL);
      PORTA->GPCLR = 0x0700UL|PORT_GPCLR_GPWE(0x0009UL);
      PORTB->GPCLR = 0x0000UL|PORT_GPCLR_GPWE(0x0004UL);
      PORTB->GPCLR = 0x0100UL|PORT_GPCLR_GPWE(0x0009UL);
      PORTC->GPCLR = 0x0000UL|PORT_GPCLR_GPWE(0x0080UL);
      PORTC->GPCLR = 0x0100UL|PORT_GPCLR_GPWE(0x0013UL);
      PORTC->GPCLR = 0x0200UL|PORT_GPCLR_GPWE(0x0064UL);
      PORTC->GPCLR = 0x0400UL|PORT_GPCLR_GPWE(0x0008UL);
      PORTD->GPCLR = 0x0100UL|PORT_GPCLR_GPWE(0x0083UL);
      PORTD->GPCLR = 0x0200UL|PORT_GPCLR_GPWE(0x0008UL);
      PORTD->GPCLR = 0x0400UL|PORT_GPCLR_GPWE(0x0050UL);
      PORTE->GPCLR = 0x0300UL|PORT_GPCLR_GPWE(0x0003UL);
}
/** 
 * End group USBDM_Group
 * @}
 */

} // End namespace USBDM
