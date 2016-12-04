/**
 * @file     bdmCommon.h
 * @brief    Low power timer interface
 *
 * @version  V4.12.1.80
 * @date     13 April 2016
 */
#ifndef _BDMCOMMON_H_
#define _BDMCOMMON_H_

#include <stdint.h>

//================================================================================
//  Timer Usage:
//
//   TPMx-CHa - BDM_IN pin, SYNC measuring, ACKN detection (IC rising & falling edges)
//   TPMx-CHb - ACKN & SYNC Timeouts (Output compare)
//
//================================================================================

/*
 * Initialises the timers, input captures and interrupts
 */
USBDM_ErrorCode   setTarget(TargetType_t target);
USBDM_ErrorCode   checkTargetVdd(void);
void              suspend(void);
USBDM_ErrorCode   cycleTargetVddOn(uint8_t mode);
USBDM_ErrorCode   cycleTargetVdd(uint8_t mode);
uint16_t          targetVddMeasure(void);
USBDM_ErrorCode   setTargetVdd( void );  // Low-level - bdm_cycleTargetVddOn() preferred
USBDM_ErrorCode   clearStatus(void);

#endif // _BDMCOMMON_H_
