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

// Timer constants, 24MHz ticks
#define TIMER_FREQ               (OSC_FREQ/2)
#define TIMER_MICROSECOND(x)     (((x)*(TIMER_FREQ/1000))/1000UL)  // Timer ticks in 1 us

// General Time intervals
#define VDD_RISE_TIMEus    2000U // us - minimum time to allow for controlled target Vdd rise
#define BKGD_WAITus        2000U // us - time to hold BKGD pin low after reset pin rise for special modes (allowance made for slow Reset rise)
#define RESET_SETTLEms        3U // ms - time to wait for signals to settle in us, this should be longer than the soft reset time
#define RESET_RECOVERYms     10U // ms - how long to wait after reset before new commands are allowed

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
