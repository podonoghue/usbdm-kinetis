/*! \file
    \brief USBDM - low level BDM interface.

   \verbatim
   This software was modified from \e TBLCF software
   This software was modified from \e TBDML software

   USBDM
   Copyright (C) 2007  Peter O'Donoghue

   Turbo BDM Light
   Copyright (C) 2005  Daniel Malik

   Turbo BDM Light ColdFire
   Copyright (C) 2005  Daniel Malik

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
   \endverbatim

   \verbatim
   Change History
   +============================================================================================
   | 3  Dec 2016 | Ported to Kinetis                                                - V4.121.160
   +============================================================================================
   \endverbatim
 */

#include <stdint.h>
#include "system.h"
#include "derivative.h"
#include "hardware.h"
#include "delay.h"
#include "USBDM_MK20D5.h"
#include "targetDefines.h"
#include "cmdProcessingHCS.h"
#include "bdm.h"
#include "bdmCommon.h"

namespace Bdm {

//constexpr unsigned BDM_SYNC_REQms       =      1U; //!< ms - length of the longest possible SYNC REQUEST pulse (128 BDM cycles @ 400kHz = 320us plus some extra time)
//constexpr unsigned SYNC_TIMEOUT_us       =    460U; //!< us - longest time for the target to completed a SYNC pulse (16+128+margin cycles @ 400kHz = 375us)
//constexpr unsigned ACKN_TIMEOUTus       =   2500U; //!< us - longest time after which the target should produce ACKN pulse (150 cycles @ 400kHz = 375us)
//constexpr unsigned SOFT_RESETus         =  10000U; //!< us - longest time needed for soft reset of the BDM interface (512 BDM cycles @ 400kHz = 1280us)
//constexpr unsigned RESET_LENGTHms       =    100U; //!< ms - time of RESET assertion
//constexpr unsigned RESET_INITIAL_WAITms =     10U; //!< ms - max time to wait for the RESET pin to come high after release
///** Time to set up Timer - This varies with optimisation! */
//static constexpr unsigned TMR_SETUP_TIME = 40;//15;


/** Max time to wait for the RESET pin to go high after release */
constexpr unsigned RESET_RELEASE_WAIT_ms = 300;

/** Width of sync pulse (us), 128 target-cycle @ 128kHz (<65536/FTMClock) */
static constexpr unsigned SYNC_WIDTH_us = 1000;

/** Time to wait for end of target sync pulse response (us), (<65536/FTMClock) */
static constexpr unsigned SYNC_TIMEOUT_us = 1200;

/** Longest time after which the target should produce ACKN pulse (150 cycles @ 400kHz = 375us) */
static constexpr unsigned ACKN_TIMEOUT_us = 2500;

/** Length of high drive before 3-state i.e. speed-up pulse (ticks) */
static constexpr unsigned SPEEDUP_PULSE_WIDTH_ticks = 1;


/** FTM to use */
using FtmInfo = USBDM::Ftm0Info;
/** FTM channel for BKGD out D6(ch6) */
constexpr int bkgdOutChannel = 6;
/** FTM channel for BKGD enable C3(ch2) */
constexpr int bkgdEnChannel = 2;
/** FTM channel for BKGD in D4(ch4) */
constexpr int bkgdInChannel = 4;

/* Make sure pins have been configured for FTM */
USBDM::CheckSignal<FtmInfo, bkgdOutChannel> x;
USBDM::CheckSignal<FtmInfo, bkgdEnChannel>  y;
USBDM::CheckSignal<FtmInfo, bkgdInChannel>  z;

/** Pointer to hardware */
static constexpr volatile FTM_Type *ftm = reinterpret_cast<volatile FTM_Type*>(FtmInfo::ftm);

/** Pointer to clock register */
static constexpr volatile uint32_t *clockReg  = reinterpret_cast<volatile uint32_t*>(FtmInfo::clockReg);

/**
 * Disable FTM control of BKGD
 */
inline
static void disablePins() {
   ftm->SWOCTRL =
         (0<<(bkgdEnChannel+8)) |(1<<bkgdEnChannel)|  // Force low (disable buffer)
         (1<<(bkgdOutChannel+8))|(1<<bkgdOutChannel); // Force high
}

/**
 * Set BKGD pin state
 *
 * @param pins Pin control mask
 */
void setBkgd(PinLevelMasks_t pins) {

   uint16_t value = 0;
   switch ((pins&PIN_BKGD_MASK)) {
      default:
      case PIN_BKGD_3STATE :
         value =
               (0<<(bkgdEnChannel+8)) |(1<<bkgdEnChannel)|  // Force low (disable buffer)
               (1<<(bkgdOutChannel+8))|(1<<bkgdOutChannel); // Force high
         break;
      case PIN_BKGD_LOW :
         value =
               (1<<(bkgdEnChannel+8)) |(1<<bkgdEnChannel)|  // Force high (enable buffer)
               (0<<(bkgdOutChannel+8))|(1<<bkgdOutChannel); // Force low
         break;
      case PIN_BKGD_HIGH :
         value =
               (1<<(bkgdEnChannel+8)) |(1<<bkgdEnChannel)|  // Force high (enable buffer)
               (0<<(bkgdOutChannel+8))|(1<<bkgdOutChannel); // Force high
         break;
   }
   ftm->SWOCTRL = value;
}

/**
 * Enable FTM control from BKGD
 */
inline
static void enablePins() {
   ftm->SWOCTRL = 0;
}

inline
static void enableFtmClock() {
   ftm->SC =
         FTM_SC_CPWMS(0)| // Left-Aligned
         FTM_SC_CLKS(1)|  // Clock source = SystemBusClock
         FTM_SC_TOIE(0)|  // Timer Overflow Interrupt disabled
         FTM_SC_PS(0);    // Prescale = /1
}

inline
static void disableFtmClock() {
   ftm->SC = 0;
}

///** PCR for BKGD in pin used by timer */
//using BkgdInPcr        = USBDM::PcrTable_T<FtmInfo, bkgdInChannel>;
//
///** PCR for BKGD enable pin used by timer */
//using BkgdEnPcr        = USBDM::PcrTable_T<FtmInfo, bkgdEnChannel>;
//
///** PCR for BKGD in pin used by timer */
//using BkgdOutPcr       = USBDM::PcrTable_T<FtmInfo, bkgdOutChannel>;
//
/** GPIO for BKGD in pin */
using BkgdIn           = USBDM::GpioTable_T<FtmInfo, bkgdInChannel>;

///** GPIO for BKGD enable pin used by timer */
//using BkgdEn = USBDM::GpioTable_T<FtmInfo, bkgdEnChannel>;
//
///** GPIO for BKGD in pin used by timer */
//using BkgdOut = USBDM::GpioTable_T<FtmInfo, bkgdOutChannel>;
//
///** Transceiver for BKGD */
//using Bkgd = Lvc1t45<BkgdEn, BkgdOut>;

/** Measured Target SYNC width (Ticks) */
static unsigned targetSyncWidth;

/** Calculated time for target '1' bit (Ticks) */
static unsigned oneBitTime;

/** Calculated time for target '0' bit (Ticks) */
static unsigned zeroBitTime;

/** Calculated time for to sample target response (Ticks) */
static unsigned sampleBitTime;

/** Calculated time for target minimum bit period (Ticks) */
static unsigned minPeriod;

/** Calculated time for 64 target clock cycles - for ACKN (us) */
static unsigned targetClocks64TimeUS;

/** Calculated time for 150 target clock cycles - for ACKN (us) */
static unsigned targetClocks150TimeUS;

///**
// * Return the maximum of two values
// *
// *  @param a One value
// *  @param b Other value
// *
// *  @return Max value
// */
//static int max(int a, int b) {
//   return (a>b)?a:b;
//}

/**
 * Converts a time in microseconds to number of ticks
 *
 * @param [in] time Time in microseconds
 *
 * @return Time in ticks
 *
 * @note Assumes prescale has been chosen as a appropriate value. No range checking.
 */
static uint32_t convertMicrosecondsToTicks(int time) {
   // Assumes the FTM will be used with SystemBusClock & /1
   long t = ((uint64_t)time*SystemBusClock)/1000000;

   assert((long)(uint32_t)t == t);
   assert(t != 0);

   // Calculate period
   return (uint32_t)t;
}
/**
 * Converts ticks to time in microseconds
 *
 * @param [in] time  Time in ticks
 *
 * @return Time in microseconds
 *
 * @note Assumes prescale has been chosen as a appropriate value. No range checking.
 */
static uint32_t convertTicksToMicroseconds(int time) {
   long t = ((uint64_t)time*1000000)/FtmInfo::getClockFrequency();

   assert((long)(uint32_t)t == t);
   assert(t != 0);

   return t;
}

/**
 * Initialise interface
 */
void initialise() {

   // Enable clock to timer
   *clockReg  |= FtmInfo::clockMask;

   // Extended features
   ftm->MODE     = FTM_MODE_INIT_MASK|FTM_MODE_FTMEN_MASK|FTM_MODE_WPDIS_MASK;

   // Debug mode
   ftm->CONF     = FTM_CONF_BDMMODE(2);

   // Clear s register changes have immediate effect
   disableFtmClock();

   // Common registers
   ftm->CNTIN    = 0;
   ftm->CNT      = 0;
   ftm->MOD      = (uint32_t)-1;

   enableFtmClock();

   ftm->OUTINIT =
         (0<<bkgdEnChannel)|  // Initialise low (disable buffer)
         (1<<bkgdOutChannel); // Initialise high

   Reset::initialise();

   disablePins();
   ftm->CONTROLS[bkgdEnChannel].CnSC  = USBDM::ftm_outputCompareClear;
   ftm->CONTROLS[bkgdOutChannel].CnSC = USBDM::ftm_outputCompareSet;

   // Switch pins to FTM
   FtmInfo::initPCRs(PORT_PCR_DSE_MASK|PORT_PCR_PE_MASK); // DS+PDN
}

/**
 * Disables interface
 *
 * Note: Reset is not affected
 */
void disable() {
   FtmInfo::clearPCRs();
}

/**
 *  Set sync length
 *
 *  @param [in] syncLength Sync length in timer ticks to set
 */
void setSyncLength(uint16_t syncLength) {
   constexpr int SYNC_RESPONSE_CYCLES = 128;    // Number of target clock cycles in the SYNC response
   constexpr int HOST_1_CYCLES_LOW    = 4;      // Number of target clock cycles low for a 1
   constexpr int HOST_0_CYCLES_LOW    = 13;     // Number of target clock cycles low for a 0
   constexpr int HOST_SAMPLE_CYCLE    = 10;     // Number of target clock cycles for sample
   constexpr int HOST_MIN_CYCLES      = 16;     // Minimum number of target clock cycles for entire transfer

   targetSyncWidth   = syncLength;

   // Calculate communication parameters
   oneBitTime        = ((syncLength*HOST_1_CYCLES_LOW)+(SYNC_RESPONSE_CYCLES-1))/SYNC_RESPONSE_CYCLES;  // Ticks (round up)
   zeroBitTime       = ((syncLength*HOST_0_CYCLES_LOW)+(SYNC_RESPONSE_CYCLES/2))/SYNC_RESPONSE_CYCLES;  // Ticks (round)
   sampleBitTime     = ((syncLength*HOST_SAMPLE_CYCLE)+(SYNC_RESPONSE_CYCLES/2))/SYNC_RESPONSE_CYCLES;  // Ticks (round)
   minPeriod         = ((syncLength*HOST_MIN_CYCLES)+(SYNC_RESPONSE_CYCLES-1))/SYNC_RESPONSE_CYCLES;    // Ticks (round up)

   targetClocks64TimeUS   = convertTicksToMicroseconds((syncLength*64)/SYNC_RESPONSE_CYCLES);  // us
   targetClocks150TimeUS  = convertTicksToMicroseconds((syncLength*150)/SYNC_RESPONSE_CYCLES); // us
}

/**
 * Determine connection speed using sync pulse
 *
 * @param [out] syncLength Sync length in timer ticks (48MHz)
 *
 * @return Error code, BDM_RC_OK indicates success
 */
USBDM_ErrorCode sync(uint16_t &syncLength) {

   /** Time to set up Timer - This varies with optimisation! */
   static constexpr unsigned TMR_SETUP_TIME = 40;

   /* SYNC pulse width */
   const uint32_t syncPulseWidthInTicks = convertMicrosecondsToTicks(SYNC_WIDTH_us);

   disableFtmClock();

   ftm->COMBINE =
         FTM_COMBINE_COMBINE0_MASK<<(bkgdEnChannel*4)|
         FTM_COMBINE_COMBINE0_MASK<<(bkgdOutChannel*4)|
         FTM_COMBINE_DECAPEN0_MASK<<(bkgdInChannel*4);

   // Positive pulse for buffer enable, 2nd edge delayed for speed-up pulse (bkgdEnChannel, bkgdEnChannel+1)
   ftm->CONTROLS[bkgdEnChannel].CnSC    = USBDM::ftm_CombinePositivePulse;
   ftm->CONTROLS[bkgdEnChannel].CnV     = TMR_SETUP_TIME;
   ftm->CONTROLS[bkgdEnChannel+1].CnV   = TMR_SETUP_TIME+syncPulseWidthInTicks+SPEEDUP_PULSE_WIDTH_ticks;

   // Negative pulse for BKGD out (bkgdOutChannel, bkgdOutChannel+1)
   ftm->CONTROLS[bkgdOutChannel].CnSC   = USBDM::ftm_CombineNegativePulse;
   ftm->CONTROLS[bkgdOutChannel].CnV    = TMR_SETUP_TIME;
   ftm->CONTROLS[bkgdOutChannel+1].CnV  = TMR_SETUP_TIME+syncPulseWidthInTicks;

   // Enable dual capture on BKGD in (bkgdInChannel, bkgdInChannel+1)
   ftm->CONTROLS[bkgdInChannel].CnSC    = USBDM::ftm_dualEdgeCaptureOneShotFallingEdge;
   ftm->CONTROLS[bkgdInChannel+1].CnSC  = USBDM::ftm_inputCaptureRisingEdge;

   // Release force pin control
   enablePins();

   ftm->CNT = 0;

   enableFtmClock();

   // Clear channel flags
   ftm->STATUS &= ~(
         (1<<bkgdEnChannel) |(1<<(bkgdEnChannel+1))|
         (1<<bkgdOutChannel)|(1<<(bkgdOutChannel+1))|
         (1<<bkgdInChannel) |(1<<(bkgdInChannel+1)));

   static auto pollPulseStart = [] {
         return ((ftm->CONTROLS[bkgdOutChannel].CnSC&FTM_CnSC_CHF_MASK) != 0);
   };
   // Wait for start of pulse
   USBDM::waitUS(TMR_SETUP_TIME, pollPulseStart);

   // Trigger dual-edge capture on BKGD_In
   ftm->COMBINE |= FTM_COMBINE_DECAP0_MASK<<(bkgdInChannel*4);

   static auto pollDecap = [] {
         return ((ftm->COMBINE&(FTM_COMBINE_DECAP0_MASK<<(bkgdInChannel*4))) == 0);
   };
   // Wait for dual-edge & capture
   bool success = USBDM::waitUS(SYNC_TIMEOUT_us, pollDecap);

   volatile uint16_t e1 = ftm->CONTROLS[bkgdInChannel].CnV;
   volatile uint16_t e2 = ftm->CONTROLS[bkgdInChannel+1].CnV;

   // Release force pin control
   disablePins();

   // Disable dual-edge capture (in case timeout)
   ftm->COMBINE = 0;

   if (success) {
      syncLength = e2 - e1;
      setSyncLength(syncLength);
      return BDM_RC_OK;
   }
   return BDM_RC_SYNC_TIMEOUT;
}

/**
 *  Depending on ACKN mode this function:       \n
 *    - Waits for ACKN pulse with timeout.
 *       OR
 *    - Busy waits for 64 target CPU clocks
 *
 *  @return
 *    \ref BDM_RC_OK           => Success \n
 *    \ref BDM_RC_ACK_TIMEOUT  => No ACKN detected [timeout]
 */
USBDM_ErrorCode acknowledgeOrWait64(void) {
   if (cable_status.ackn==ACKN) {
      // Wait for pin capture or timeout
      enableInterrupts();
      static auto fn = [] {
            return ((ftm->CONTROLS[bkgdInChannel].CnSC&FTM_CnSC_CHF_MASK) != 0) ||
                   ((ftm->CONTROLS[bkgdInChannel+1].CnSC&FTM_CnSC_CHF_MASK) != 0);
      };
      USBDM::waitMS(ACKN_TIMEOUT_us, fn);
      if ((ftm->CONTROLS[bkgdInChannel].CnSC&FTM_CnSC_CHF_MASK) == 0) {
         // No ACKN - Return timeout error
         return BDM_RC_ACK_TIMEOUT;
      }
   }
   else {
      USBDM::waitUS(targetClocks64TimeUS);
   }
   return BDM_RC_OK;
}

/**
 *  Depending on ACKN mode this function:       \n
 *    - Waits for ACKN pulse with timeout.
 *       OR
 *    - Busy waits for 150 target CPU clocks
 *
 *  @return
 *    \ref BDM_RC_OK           => Success \n
 *    \ref BDM_RC_ACK_TIMEOUT  => No ACKN detected [timeout]
 */
USBDM_ErrorCode acknowledgeOrWait150(void) {
   if (cable_status.ackn==ACKN) {
      // Wait for pin capture or timeout
      enableInterrupts();
      static auto fn = [] {
            return ((ftm->CONTROLS[bkgdInChannel].CnSC&FTM_CnSC_CHF_MASK) != 0) ||
                   ((ftm->CONTROLS[bkgdInChannel+1].CnSC&FTM_CnSC_CHF_MASK) != 0);
      };
      USBDM::waitMS(ACKN_TIMEOUT_us, fn);
      if ((ftm->CONTROLS[bkgdInChannel].CnSC&FTM_CnSC_CHF_MASK) == 0) {
         // No ACKN - Return timeout error
         return BDM_RC_ACK_TIMEOUT;
      }
   }
   else {
      USBDM::waitUS(targetClocks150TimeUS);
   }
   return BDM_RC_OK;
}

/**
 * Receive an value over BDM interface
 *
 * @param [in]  length Number of bits to receive
 * @param [out] data   Data received
 */
USBDM_ErrorCode rx(int length, unsigned &data) {

   /** Time to set up Timer - This varies with optimisation! */
   static constexpr unsigned TMR_SETUP_TIME = 40;

   disableFtmClock();

   ftm->COMBINE =
         FTM_COMBINE_COMBINE0_MASK<<(bkgdEnChannel*4)|
         FTM_COMBINE_COMBINE0_MASK<<(bkgdOutChannel*4);

   // Positive pulse for buffer enable
   ftm->CONTROLS[bkgdEnChannel].CnSC    = FTM_CnSC_ELS(2);
   ftm->CONTROLS[bkgdEnChannel].CnV     = TMR_SETUP_TIME;
   ftm->CONTROLS[bkgdEnChannel+1].CnV   = TMR_SETUP_TIME+oneBitTime-SPEEDUP_PULSE_WIDTH_ticks;

   // Negative pulse for BKGD out
   ftm->CONTROLS[bkgdOutChannel].CnSC   = FTM_CnSC_ELS(1);
   ftm->CONTROLS[bkgdOutChannel].CnV    = TMR_SETUP_TIME;
   ftm->CONTROLS[bkgdOutChannel+1].CnV  = TMR_SETUP_TIME+oneBitTime;

   // Capture rising edge of BKGD in
   ftm->CONTROLS[bkgdInChannel].CnSC    = USBDM::ftm_inputCaptureRisingEdge;

   enableFtmClock();

   bool success = true;
   unsigned value = 0;
   while (length-->0) {
      ftm->CNT = 0;

      // Clear channel flags
      ftm->STATUS &= ~(
            (1<<bkgdEnChannel) |(1<<(bkgdEnChannel+1))|
            (1<<bkgdOutChannel)|(1<<(bkgdOutChannel+1))|
            (1<<bkgdInChannel));

      // Wait until end of bit
      do {
         __asm("nop");
      } while (ftm->CNT < 2*minPeriod);

      // Should have captured a rising edge from target (or float?)
      success = (ftm->CONTROLS[bkgdInChannel].CnSC & FTM_CnSC_CHF_MASK) != 0;

      volatile uint16_t eventTime = ftm->CONTROLS[bkgdInChannel].CnV;
      value = (value<<1)|((eventTime>(TMR_SETUP_TIME+sampleBitTime))?0:1);
   }
   data = value;
   if (!success) {
      return BDM_RC_BKGD_TIMEOUT;
   }
   return BDM_RC_OK;
}

/**
 * Receive an 8-bit value over BDM interface
 *
 * @param [out] data   Data received
 */
inline
USBDM_ErrorCode rx8(uint8_t *data) {
   unsigned value;
   USBDM_ErrorCode rc = rx(8, value);
   *data = (uint8_t)value;
   return rc;
}

/**
 * Receive an 16-bit value over BDM interface
 *
 * @param [out] data   Data received
 */
inline
USBDM_ErrorCode rx16(uint8_t *data) {
   unsigned value;
   USBDM_ErrorCode rc = rx(16, value);
   unpack16BE(value, data);
   return rc;
}

/**
 * Receive an 32-bit value over BDM interface
 *
 * @param [out] data   Data received
 */
inline
USBDM_ErrorCode rx32(uint8_t *data) {
   unsigned value;
   USBDM_ErrorCode rc = rx(32, value);
   unpack32BE(value, data);
   return rc;
}

inline
void txInit() {
   disableFtmClock();
   ftm->SYNCONF = FTM_SYNCONF_SYNCMODE(1)|FTM_SYNCONF_SWWRBUF(1);
   disableInterrupts();
   enablePins();
}

inline
void txComplete() {
   disableFtmClock();
   enableInterrupts();
   disablePins();
}

/**
 * Transmit a value over BDM interface
 *
 * @param [in]  length Number of bits to transmit
 * @param [in]  data   Data value to transmit
 *
 * @return Error code, BDM_RC_OK indicates success
 */
USBDM_ErrorCode tx(int length, unsigned data) {

   /* Time to set up Timer - This varies with optimisation! */
   static constexpr unsigned TMR_SETUP_TIME = 40;
   uint32_t mask = (1U<<(length-1));

   ftm->COMBINE =
         FTM_COMBINE_SYNCEN0_MASK<<(bkgdEnChannel*4)|
         FTM_COMBINE_COMBINE0_MASK<<(bkgdEnChannel*4)|
         FTM_COMBINE_SYNCEN0_MASK<<(bkgdOutChannel*4)|
         FTM_COMBINE_COMBINE0_MASK<<(bkgdOutChannel*4);

   // Positive pulse for buffer enable
   ftm->CONTROLS[bkgdEnChannel].CnSC    = USBDM::ftm_CombinePositivePulse;
   ftm->CONTROLS[bkgdEnChannel].CnV     = TMR_SETUP_TIME;

   // Negative pulse for BKGD out
   ftm->CONTROLS[bkgdOutChannel].CnSC   = USBDM::ftm_CombineNegativePulse;
   ftm->CONTROLS[bkgdOutChannel].CnV    = TMR_SETUP_TIME;

   // Data sample capture rising edge of BKGD in
   ftm->CONTROLS[bkgdInChannel].CnSC    = USBDM::ftm_inputCaptureRisingEdge;

   // ACKN timeout
   ftm->CONTROLS[bkgdInChannel+1].CnSC  = USBDM::ftm_outputCompare;
   ftm->CONTROLS[bkgdInChannel+1].CnV   = TMR_SETUP_TIME+ACKN_TIMEOUT_us;

   while (mask>0) {
      int width;
      if (data&mask) {
         width = TMR_SETUP_TIME+oneBitTime;
      }
      else {
         width = TMR_SETUP_TIME+zeroBitTime;
      }
      mask >>= 1;
      disableFtmClock();
      ftm->CNT = 0;
      ftm->CONTROLS[bkgdOutChannel+1].CnV  = width;
      ftm->CONTROLS[bkgdEnChannel+1].CnV   = width+SPEEDUP_PULSE_WIDTH_ticks;
      ftm->SYNC     = FTM_SYNC_SWSYNC(1);

      enableFtmClock();

      // Wait until end of bit
      do {
         __asm("nop");
      } while (ftm->CNT < TMR_SETUP_TIME+minPeriod);
   }
   // Clear channel flags
   ftm->STATUS &= ~(
         (1<<bkgdEnChannel) |(1<<(bkgdEnChannel+1))|
         (1<<bkgdOutChannel)|(1<<(bkgdOutChannel+1))|
         (1<<bkgdInChannel)|(1<<(bkgdInChannel+1)));

   return BDM_RC_OK;
}

inline
USBDM_ErrorCode tx8(uint8_t data) {
   return tx(8, data);
}

inline
USBDM_ErrorCode tx16(uint16_t data) {
   return tx(16, data);
}

inline
USBDM_ErrorCode tx24(uint32_t data) {
   return tx(24, data);
}

inline
USBDM_ErrorCode tx32(uint32_t data) {
   return tx(32, data);
}

/**
 * *************************************************************************
 */
//============================================================
// The following commands DO NOT expect an ACK & do not delay
// Interrupts are left disabled!
//

/**
 * Write command byte, truncated sequence
 *
 * @param cmd command byte to write
 *
 * @note Interrupts are left disabled, no ACK is expected
 */
void cmd_0_0_T(uint8_t cmd) {
   txInit();
   tx8(cmd);
   //   txComplete();
}

/**
 *  Write command byte + parameter, truncated sequence
 *
 *  @param cmd         command byte to write
 *  @param parameter   byte parameter to write
 *
 * @note Interrupts are left disabled, no ACK is expected
 */
void cmd_1B_0_T(uint8_t cmd, uint8_t parameter) {
   txInit();
   tx8(cmd);
   tx8(parameter);
   //   txComplete();
}

/**
 *  Special for Software Reset HCS08, truncated sequence
 *
 *  @param cmd         command byte to write
 *  @param parameter1  word parameter to write
 *  @param parameter2  byte parameter to write
 *
 * @note Interrupts are left disabled, no ACK is expected
 */
void cmd_1W1B_0_T(uint8_t cmd, uint16_t parameter1, uint8_t parameter2) {
   txInit();
   tx8(cmd);
   tx16(parameter1);
   tx8(parameter2);
   //   txComplete();
}

//============================================================
// The following commands DO NOT expect an ACK & do not delay
//

/**
 *  Write cmd without ACK (HCS08/RS08/CFV1)
 *
 *  @param cmd command byte to write
 *
 *  @note No ACK is expected
 */
void cmd_0_0_NOACK(uint8_t cmd) {
   txInit();
   tx8(cmd);
   txComplete();
}

/**
 * Write cmd & read byte without ACK (HCS08)
 *
 *  @param cmd command byte to write
 *  @param result word read
 *
 *  @note No ACK is expected
 */
void cmd_0_1B_NOACK(uint8_t cmd, uint8_t *result) {
   txInit();
   tx8(cmd);
   rx8(result);
   txComplete();
}

/**
 *  Write cmd & byte without ACK
 *
 *  @param cmd       command byte to write
 *  @param parameter byte to write
 *
 *  @note No ACK is expected
 */
void cmd_1B_0_NOACK(uint8_t cmd, uint8_t parameter) {
   txInit();
   tx8(cmd);
   tx8(parameter);
   txComplete();
}

/**
 *  Write cmd & read word without ACK (HCS08)
 *
 *  @param cmd    command byte to write
 *  @param result word read
 *
 *  @note No ACK is expected
 */
void cmd_0_1W_NOACK(uint8_t cmd, uint8_t *result) {
   txInit();
   tx8(cmd);
   rx16(result);
   txComplete();
}

/**
 *  Write cmd & word without ACK
 *
 *  @param cmd       command byte to write
 *  @param parameter word to write
 *
 *  @note No ACK is expected
 */
void cmd_1W_0_NOACK(uint8_t cmd, uint16_t parameter) {
   txInit();
   tx8(cmd);
   tx16(parameter);
   txComplete();
}

/**
 *  Write cmd, word & read word without ACK
 *
 *  @param cmd        command byte to write
 *  @param parameter  word to write
 *  @param result     word pointer for read (status+data byte)
 *
 *  @note no ACK is expected
 */
void cmd_1W_1W_NOACK(uint8_t cmd, uint16_t parameter, uint8_t result[2]) {
   txInit();
   tx8(cmd);
   tx16(parameter);
   rx16(result);
   txComplete();
}

/**
 *  Write cmd, word, byte & read byte without ACK
 *
 *  @param cmd        command byte to write
 *  @param parameter  word to write
 *  @param value      byte to write
 *  @param status     byte pointer for read
 *
 *  @note no ACK is expected
 */
void cmd_1W1B_1B_NOACK(uint8_t cmd, uint16_t parameter, uint8_t value, uint8_t *status) {
   txInit();
   tx8(cmd);
   tx16(parameter);
   tx8(value);
   rx8(status);
   txComplete();
}

//====================================================================
// The following DO expect an ACK or wait at end of the transmit phase

/**
 *  Write cmd
 *
 *  @param cmd command byte to write
 *
 *  @note ACK is expected
 */
USBDM_ErrorCode cmd_0_0(uint8_t cmd) {
   txInit();
   tx8(cmd);
   USBDM_ErrorCode rc = acknowledgeOrWait64();
   txComplete();
   return rc;
}

/**
 *  Write cmd, read byte
 *
 *  @param cmd        command byte to write
 *  @param result     byte read
 *
 *  @note ACK is expected
 */
USBDM_ErrorCode cmd_0_1B(uint8_t cmd, uint8_t *result) {
   txInit();
   tx8(cmd);
   USBDM_ErrorCode rc = acknowledgeOrWait64();
   rx8(result);
   txComplete();
   return rc;
}

/**
 *  Write cmd & read word
 *
 *  @param cmd    command byte to write
 *  @param result word read
 *
 *  @note ACK is expected
 */
USBDM_ErrorCode cmd_0_1W(uint8_t cmd, uint8_t *result) {
   txInit();
   tx8(cmd);
   USBDM_ErrorCode rc = acknowledgeOrWait64();
   rx16(result);
   txComplete();
   return rc;
}

/**
 *  Write cmd & read longword
 *
 *  @param cmd    command byte to write
 *  @param result longword read
 *
 *  @note ACK is expected
 */
USBDM_ErrorCode cmd_0_1L(uint8_t cmd, uint8_t result[4]) {
   txInit();
   tx8(cmd);
   USBDM_ErrorCode rc = acknowledgeOrWait64();
   rx32(result);
   return rc;
}

/**
 *  Write cmd & byte
 *
 *  @param cmd        command byte to write
 *  @param parameter  byte to write
 *
 *  @note ACK is expected
 */
USBDM_ErrorCode cmd_1B_0(uint8_t cmd, uint8_t parameter) {
   txInit();
   tx8(cmd);
   tx8(parameter);
   USBDM_ErrorCode rc = acknowledgeOrWait64();
   txComplete();
   return rc;
}

/**
 *  Write cmd & word
 *
 *  @param cmd       command byte to write
 *  @param parameter word to write
 *
 *  @note ACK is expected
 */
USBDM_ErrorCode cmd_1W_0(uint8_t cmd, uint16_t parameter) {
   txInit();
   tx8(cmd);
   tx16(parameter);
   USBDM_ErrorCode rc = acknowledgeOrWait64();
   txComplete();
   return rc;
}

/**
 *  Write cmd & longword
 *
 *  @param cmd       command byte to write
 *  @param parameter longword to write
 *
 *  @note ACK is expected
 */
USBDM_ErrorCode cmd_1L_0(uint8_t cmd, uint32_t parameter) {
   txInit();
   tx8(cmd);
   tx32(parameter);
   USBDM_ErrorCode rc = acknowledgeOrWait64();
   txComplete();
   return rc;
}

/**
 *  Write cmd, word & read byte (read word but return byte - HC/S12(x))
 *
 *  @param cmd       command byte to write
 *  @param parameter word to write
 *  @param result    byte read
 *
 *  @note ACK is expected
 */
USBDM_ErrorCode cmd_1W_1WB(uint8_t cmd, uint16_t parameter, uint8_t *result) {
   txInit();
   tx8(cmd);
   tx16(parameter);
   USBDM_ErrorCode rc = acknowledgeOrWait150();
   if ((parameter)&0x0001) {
      (void)rx8(result);
      rx8(result);
   } else {
      rx8(result);
      uint8_t dummy;
      rx8(&dummy);
   }
   txComplete();
   return rc;
}

/**
 *  Write cmd & 2 words
 *
 *  @param cmd       command byte to write
 *  @param parameter1 word to write
 *  @param parameter2 word to write
 *
 *  @note ACK is expected
 */
USBDM_ErrorCode cmd_2W_0(uint8_t cmd, uint16_t parameter1, uint16_t parameter2) {
   txInit();
   tx8(cmd);
   tx16(parameter1);
   tx16(parameter2);
   USBDM_ErrorCode rc = acknowledgeOrWait150();
   txComplete();
   return rc;
}

/**
 *  Write cmd, word & read word
 *
 *  @param cmd        command byte to write
 *  @param parameter  word to write
 *  @param result     word read
 *
 *  @note ACK is expected
 */
USBDM_ErrorCode cmd_1W_1W(uint8_t cmd, uint16_t parameter, uint8_t result[2]) {
   txInit();
   tx8(cmd);
   tx16(parameter);
   USBDM_ErrorCode rc = acknowledgeOrWait150();
   rx16(result);
   txComplete();
   return rc;
}

/**
 *  Write cmd, word, byte & read byte
 *
 *  @param cmd        command byte to write
 *  @param parameter  word to write
 *  @param value      byte to write
 *  @param status     byte pointer for read
 *
 *  @note ACK is expected
 */
USBDM_ErrorCode cmd_1W1B_1B(uint8_t cmd, uint16_t parameter, uint8_t value, uint8_t *status) {
   txInit();
   tx8(cmd);
   tx16(parameter);
   tx8(value);
   USBDM_ErrorCode rc = acknowledgeOrWait64();
   rx8(status);
   txComplete();
   return rc;
}
/**
 *  Write cmd, word and a byte
 *  (sends 2 words, the byte in both high and low byte of the 16-bit value)
 *
 *  @param cmd        command byte to write
 *  @param parameter1 word to write
 *  @param parameter2 bye to write
 *
 *  @note ACK is expected
 */
USBDM_ErrorCode cmd_2WB_0(uint8_t cmd, uint16_t parameter1, uint8_t parameter2) {
   txInit();
   tx8(cmd);
   tx16(parameter1);
   tx8(parameter2);
   tx8(parameter2);
   USBDM_ErrorCode rc = acknowledgeOrWait150();
   txComplete();
   return rc;
}

/**
 *  Write cmd, word & read byte
 *
 *  @param cmd        command byte to write
 *  @param parameter  word to write
 *  @param result     byte read
 *
 *  @note ACK is expected
 */
USBDM_ErrorCode cmd_1W_1B(uint8_t cmd, uint16_t parameter, uint8_t *result) {
   USBDM_ErrorCode rc;
   txInit();
   tx8(cmd);
   tx16(parameter);
   rc = acknowledgeOrWait64();
   rx8(result);
   txComplete();
   return rc;
}

/**
 *  Write cmd, word & byte
 *
 *  @param cmd         command byte to write
 *  @param parameter1  word to write
 *  @param parameter2  byte to write
 *
 *  @note ACK is expected
 */
USBDM_ErrorCode cmd_1W1B_0(uint8_t cmd, uint16_t parameter1, uint8_t parameter2) {
   USBDM_ErrorCode rc;
   txInit();
   tx8(cmd);
   tx16(parameter1);
   tx8(parameter2);
   rc = acknowledgeOrWait150();
   txComplete();
   return rc;
}

/**
 *  Write cmd, 24-bit value & byte
 *
 *  @param cmd    command byte to write
 *  @param addr   24-bit value to write
 *  @param value  byte to write
 *
 *  @note ACK is expected
 */
USBDM_ErrorCode cmd_1A1B_0(uint8_t cmd, uint32_t addr, uint8_t value) {
   USBDM_ErrorCode rc;
   txInit();
   tx8(cmd);
   tx24(addr);
   tx8(value);
   rc = acknowledgeOrWait150();
   txComplete();
   return rc;
}

/**
 *  Write cmd, 24-bit value & word
 *
 *  @param cmd    command byte to write
 *  @param addr   24-bit value to write
 *  @param value  word to write
 *
 *  @note ACK is expected
 */
USBDM_ErrorCode cmd_1A1W_0(uint8_t cmd, uint32_t addr, uint16_t value) {
   USBDM_ErrorCode rc;
   txInit();
   tx8(cmd);
   tx24(addr);
   tx16(value);
   rc = acknowledgeOrWait150();
   txComplete();
   return rc;
}

/**
 *  Write cmd, 24-bit value & longword
 *
 *  @param cmd    command byte to write
 *  @param addr   24-bit value to write
 *  @param value  ptr to longword to write
 *
 *  @note ACK is expected
 */
USBDM_ErrorCode cmd_1A1L_0(uint8_t cmd, uint32_t addr, uint32_t value) {
   USBDM_ErrorCode rc;
   txInit();
   tx8(cmd);
   tx24(addr);
   tx32(value);
   rc = acknowledgeOrWait150();
   txComplete();
   return rc;
}

/**
 *  Write cmd, 24-bit value & read byte
 *
 *  @param cmd     command byte to write
 *  @param addr    24-bit value to write
 *  @param result  ptr to longword to read
 *
 *  @note ACK is expected
 */
USBDM_ErrorCode cmd_1A_1B(uint8_t cmd, uint32_t addr, uint8_t *result) {
   USBDM_ErrorCode rc;
   txInit();
   tx8(cmd);
   tx24(addr);
   rc = acknowledgeOrWait150();
   rx8(result);
   txComplete();
   return rc;
}

/**
 *  Write Target BDM control register
 *
 *  Depending on target architecture this writes to \n
 *   - BDCSC,  \n
 *   - BDMSTS, \n
 *   - XCSR.
 *
 *  @param value => value to write
 *
 *  @return BDM_RC_OK              Success
 *  @return BDM_RC_ILLEGAL_COMMAND Command not available for this target
 */
USBDM_ErrorCode writeBDMControl(uint8_t value) {

   switch (cable_status.target_type) {
#if TARGET_CAPABILITY & CAP_S12Z
      case T_HCS12Z  :
         BDMZ12_CMD_WRITE_BDCCSR((value<<8)|0xFF);
         break;
#endif
      case T_HC12:
         BDM12_CMD_BDWRITEB(HC12_BDMSTS,value);
         break;
      case T_HCS08:
      case T_RS08:
         BDM08_CMD_WRITECONTROL(value);
         break;
      case T_CFV1:
         BDMCF_CMD_WRITE_XCSR(value);
         break;
      default:
         // Don't know how to write status on this one!
         return BDM_RC_ILLEGAL_COMMAND;
   }
   return BDM_RC_OK;
}

/**
 *  Read Target BDM status
 *
 *  Depending on target architecture this reads from
 *  - BDCCSR
 *  - BDCSC,
 *  - BDMSTS,
 *  - XCSR.
 *
 * @param status Status value read
 *
 * @return BDM_RC_OK              Success
 * @return BDM_RC_ILLEGAL_COMMAND Command not available for this target
 */
USBDM_ErrorCode readBDMStatus(uint8_t *status) {

   switch (cable_status.target_type) {
#if TARGET_CAPABILITY & CAP_S12Z
      case T_HCS12Z  : {
         uint16_t temp;
         BDMZ12_CMD_READ_BDCCSR(&temp);
         *status = temp>>8;
      }
      break;
#endif
      case T_HC12:
         BDM12_CMD_BDREADB(HC12_BDMSTS,status);
         break;
      case T_HCS08:
      case T_RS08:
         BDM08_CMD_READSTATUS(status);
         break;
      case T_CFV1:
         BDMCF_CMD_READ_XCSR(status);
         break;
      default:
         // Don't know how to check status on this one!
         return BDM_RC_ILLEGAL_COMMAND;
   }
   return BDM_RC_OK;
}

/**
 * Attempts to enable ACKN mode on target BDM interface.\n
 * It is not an error if it fails as some targets do not support this.
 */
void enableACKNMode(void) {
   USBDM_ErrorCode rc;

   // Switch ACKN on
   cable_status.ackn = ACKN;

   // Send the ACK enable command to the target
   if ((cable_status.target_type==T_CFV1)||(cable_status.target_type==T_S12Z)) {
      rc = BDMCF_CMD_ACK_ENABLE();
   }
   else {
      rc = BDM_CMD_ACK_ENABLE();
//      rc = BDM_CMD_ACK_DISABLE();
   }
   // If ACKN fails turn off ACKN (RS08 or early HCS12 target)
   if (rc == BDM_RC_ACK_TIMEOUT) {
      cable_status.ackn = WAIT;
   }
}

/**
 *  Tries to connect to target - doesn't try other strategies such as reset.
 *  This function does a basic connect sequence and tries to enable ACKN.
 *  It doesn't configure the BDM registers on the target.
 *
 * @return BDM_RC_OK                  => success
 * @return BDM_RC_VDD_NOT_PRESENT     => no target power present
 * @return BDM_RC_RESET_TIMEOUT_RISE  => RESET signal timeout - remained low
 * @return BDM_RC_BKGD_TIMEOUT        => BKGD signal timeout - remained low
 * @return BDM_RC_OK                  => other failures
 */
USBDM_ErrorCode physicalConnect(void) {
   USBDM_ErrorCode rc;

   // Assume we know nothing about connection technique & speed
   cable_status.speed = SPEED_NO_INFO;

   // Target has power?
   rc = checkTargetVdd();
   if (rc != BDM_RC_OK) {
      return rc;
   }
   // Wait with timeout until both RESET and BKGD  are high
   if (bdm_option.useResetSignal) {
      if (Reset::isLow()) {
         // TODO May take a while
         //         setBDMBusy();
         static auto fn = [] { return Reset::isHigh() && BkgdIn::isHigh(); };
         USBDM::waitMS(RESET_RELEASE_WAIT_ms, fn);
      }
      if (Reset::isLow()) {
         // RESET timeout
         return(BDM_RC_RESET_TIMEOUT_RISE);
      }
   }
   // Wait with timeout until BKGD is high
   USBDM::waitMS(RESET_RELEASE_WAIT_ms, BkgdIn::isHigh);
   if (BkgdIn::isLow()) {
      // BKGD timeout
      return(BDM_RC_BKGD_TIMEOUT);
   }
   // Try SYNC method
   uint16_t syncLength;
   rc = sync(syncLength);
   if (rc != BDM_RC_OK) {
      // try again
      rc = sync(syncLength);
   }
   if (rc == BDM_RC_OK) {
      // Speed determined by SYNC method
      cable_status.speed = SPEED_SYNC;
   }
   //TODO bdmHC12_alt_speed_detect()
   //   else if ((bdm_option.guessSpeed) &&          // Try alternative method if enabled
   //       (cable_status.target_type == T_HC12)) { // and HC12 target
   //      rc = bdmHC12_alt_speed_detect();     // Try alternative method (guessing!)
   //   }
   if (rc == BDM_RC_OK) {
      enableACKNMode();  // Try the ACKN feature
   }
   return rc;
}

/**
 *  If BDM mode is not enabled in target yet, enable it so it can be made active
 *
 *  @return BDM_RC_OK             Success
 *  @return BDM_RC_UNKNOWN_TARGET Unknown target
 *  @return BDM_RC_BDM_EN_FAILED  Enabling BDM failed (target not connected or wrong speed ?)
 */
USBDM_ErrorCode enableBDM() {
   uint8_t bdm_sts;
   USBDM_ErrorCode rc;

   rc = readBDMStatus(&bdm_sts); // Get current status
   if (rc != BDM_RC_OK) {
      return rc;
   }
   if (cable_status.target_type==T_CFV1) {
      // CFV1
      if ((bdm_sts & CFV1_XCSR_ENBDM) == 0) {
         // Try to enable BDM
         bdm_sts = (bdm_sts & CFV1_XCSR_CLKSW)|CFV1_XCSR_ENBDM;
         writeBDMControl(bdm_sts);
         rc = readBDMStatus(&bdm_sts); // Get current status
      }
      // (bdm_sts==0xFF) often indicates BKGD pin is disabled etc
      if ((bdm_sts==0xFF) || (bdm_sts & CFV1_XCSR_ENBDM) == 0) {
         return BDM_RC_BDM_EN_FAILED;
      }
   }
   else {
      // RS08/HCS12/HCS08
      if ((bdm_sts & HC12_BDMSTS_ENBDM) == 0) {
         // Try to enable BDM
         bdm_sts |= HC12_BDMSTS_ENBDM;
         writeBDMControl(bdm_sts);
         rc = readBDMStatus(&bdm_sts); // Get current status
      }
      // (bdm_sts==0xFF) often indicates BKGD pin is disabled etc.
      if ((bdm_sts==0xFF) || (bdm_sts & HC12_BDMSTS_ENBDM) == 0) {
         return BDM_RC_BDM_EN_FAILED;
      }
   }
   return rc;
}

/**
 *  Connect to target
 *
 *  This function may cycle the target power in attempting to connect. \n
 *  It enables BDM on the target if connection is successful.
 *
 *  @return
 *  @return BDM_RC_OK                 Success
 *  @return BDM_RC_VDD_NOT_PRESENT    No target power present
 *  @return BDM_RC_RESET_TIMEOUT_RISE RESET signal timeout - remained low
 *  @return BDM_RC_BKGD_TIMEOUT       BKGD signal timeout - remained low
 *  @return BDM_RC_OK                 Other failures
 */
USBDM_ErrorCode connect(void) {
   USBDM_ErrorCode rc;

   if (cable_status.speed != SPEED_USER_SUPPLIED) {
      rc = physicalConnect();
      if ((rc != BDM_RC_OK) && bdm_option.cycleVddOnConnect) {
         // No connection to target - cycle power if allowed
         (void)cycleTargetVdd(RESET_SPECIAL); // Ignore errors
         rc = physicalConnect(); // Try connect again
      }
      if (rc != BDM_RC_OK) {
         return rc;
      }
   }
   // Try to enable BDM
   return enableBDM();
}

/**
 *  Resets the target using BDM commands
 *
 *  @note
 *     Not all targets support reset using BDM commands
 *
 *  @param mode\n
 *     - RESET_SPECIAL => Reset to special mode,
 *     - RESET_NORMAL  => Reset to normal mode
 *
 *  @return BDM_RC_OK                 Success
 *  @return BDM_RC_BKGD_TIMEOUT       BKGD pin stuck low
 *  @return BDM_RC_RESET_TIMEOUT_RISE RESET pin stuck low
 *  @return BDM_RC_ILLEGAL_PARAMS     Reset mode not supported for this type of target
 *  @return BDM_RC_UNKNOWN_TARGET     Don't know how to reset this type of target
 */
USBDM_ErrorCode softwareReset(uint8_t mode) {

   if (cable_status.target_type == T_HC12) {
      // HC12 doesn't have software reset
      return BDM_RC_ILLEGAL_PARAMS;
   }
   mode &= RESET_MODE_MASK;

   // Make sure of connection
   connect();

   // Make sure Active background mode (in case target is stopped!)
   if (halt() != BDM_RC_OK) {
      connect();
   }
   switch (cable_status.target_type) {
      case T_HCS08:
         BDM08_CMD_RESET(bdm_option.SBDFRaddress, HCS_SBDFR_BDFR);
         break;
      case T_RS08:
         BDMRS08_CMD_RESET();
         break;
      case T_CFV1:
         // Force reset (& set up to halt in BDM mode on various errors)
         if (mode == RESET_SPECIAL) {
            cmd_1B_0_T(_BDMCF_WRITE_CSR2_BYTE,
                  CFV1_CSR2_BDFR|CFV1_CSR2_COPHR|CFV1_CSR2_IOPHR|CFV1_CSR2_IADHR|CFV1_CSR2_BFHBR);
         }
         else {
            cmd_1B_0_T(_BDMCF_WRITE_CSR2_BYTE,
                  CFV1_CSR2_BDFR|CFV1_CSR2_COPHR|CFV1_CSR2_IOPHR|CFV1_CSR2_IADHR);
         }
         break;
      default:
         return BDM_RC_UNKNOWN_TARGET; // Don't know how to reset this one!
   }
   if (mode == RESET_SPECIAL) {
      // Special mode - need BKGD held low out of reset
      setBkgd(PIN_BKGD_LOW);
   }
   enableInterrupts();

#if (DEBUG&RESET_DEBUG)
   DEBUG_PIN     = 0;
   DEBUG_PIN     = 1;
#endif

   USBDM::waitMS(RESET_SETTLEms);   // Wait for target to start reset (and possibly assert reset)

#if (HW_CAPABILITY&CAP_RST_IO)
   if (bdm_option.useResetSignal) {
      // Wait with timeout until RESET is high (may be held low by processor)
      bdm_WaitForResetRise();
      // Assume RESET risen - check later after cleanup
   }
#endif // (HW_CAPABILITY&CAP_RST_IO)

#if (DEBUG&RESET_DEBUG)
   DEBUG_PIN   = 1;
   DEBUG_PIN   = 0;
#endif

   if (mode == RESET_SPECIAL) {
      // Special mode - release BKGD
      USBDM::waitUS(BKGD_WAITus);      // Wait for BKGD assertion time after reset rise
      disablePins();
   }

#if (DEBUG&RESET_DEBUG)
   DEBUG_PIN   = 0;
   DEBUG_PIN   = 1;
#endif

   // Wait recovery time before allowing anything else to happen on the BDM
   USBDM::waitMS(RESET_RECOVERYms);   // Wait for Target to start up after reset

   disablePins();      // Place interface in idle state

#if (DEBUG&RESET_DEBUG)
   DEBUG_PIN   = 1;
   DEBUG_PIN   = 0;
#endif

   //  bdm_halt();  // For RS08?
   return BDM_RC_OK;
}

/**
 *  Resets the target
 *
 *  Resets the target using any of the following as needed:
 *  - Vdd power cycle - POR,
 *  - Software BDM command or
 *  - Hardware reset.
 *
 *  @param mode
 *     - \ref RESET_SPECIAL => Reset to special mode,
 *     - \ref RESET_NORMAL  => Reset to normal mode
 *
 *  @return BDM_RC_OK  => Success \n
 *  @return BDM_RC_OK  => various errors
 */
USBDM_ErrorCode targetReset( uint8_t mode ) {
   USBDM_ErrorCode rc = BDM_RC_OK;

   // Power-cycle-reset - applies to all chips
   if (bdm_option.cycleVddOnReset) {
      // TODO cycleTargetVdd()
      //      rc = cycleTargetVdd(mode);
   }
   if (rc != BDM_RC_OK) {
      return rc;
   }
   // Software (BDM Command) reset - HCS08, RS08 & Coldfire
   rc = softwareReset(mode);

#if (HW_CAPABILITY&CAP_RST_IO)
   // Hardware (RESET pin) reset
   // HC12s and some HCS08s/RS08s & CFv1 (may not result in BDM mode) support this
   if ((rc != BDM_RC_OK) && bdm_option.useResetSignal)
      rc = hardwareReset(mode);
#endif //(HW_CAPABILITY&CAP_RST_IO)

   return rc;
}

/**
 *  Halts the processor - places in background mode
 */
USBDM_ErrorCode halt(void) {
   if ((cable_status.target_type==T_CFV1)||(cable_status.target_type==T_S12Z))
      return BDMCF_CMD_BACKGROUND();
   else
      return BDM_CMD_BACKGROUND();
}

/**
 * Commences full-speed execution on the target
 */
USBDM_ErrorCode go(void) {
   if (cable_status.target_type == T_CFV1) {
      // Clear Single-step mode
      uint32_t csr;
      BDMCF_CMD_READ_DREG(CFV1_CSR, csr);
      csr &= ~CFV1_CSR_SSM;
      BDMCF_CMD_WRITE_DREG(CFV1_CSR, csr);
      return BDMCF_CMD_GO();
   }
   else {
      return BDM_CMD_GO();
   }
}

/**
 *  Executes a single instruction on the target
 */
USBDM_ErrorCode step(void) {
   if (cable_status.target_type == T_CFV1) {
      // Set Single-step mode
      uint32_t csr;
      BDMCF_CMD_READ_DREG(CFV1_CSR, csr);
      csr |= CFV1_CSR_SSM;
      BDMCF_CMD_WRITE_DREG(CFV1_CSR, csr);
      return BDMCF_CMD_GO();
   }
#if TARGET_CAPABILITY & CAP_S12Z
   else if (cable_status.target_type == T_HCS12Z  ) {
      return BDMZ12_CMD_TRACE1();
   }
#endif
   else {
      return BDM_CMD_TRACE1();
   }
}

}; // End namespace Bdm
