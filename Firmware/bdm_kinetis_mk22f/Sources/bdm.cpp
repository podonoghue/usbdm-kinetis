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

#include <interfaceCommon.h>
#include <stdint.h>
#include "ftm.h"
#include "delay.h"
#include "utilities.h"
#include "configure.h"
#include "system.h"
#include "derivative.h"
#include "hardware.h"
#include "resetInterface.h"
#include "cmdProcessingHCS.h"
#include "bdm.h"
#include "targetDefines.h"

namespace Bdm {

using namespace USBDM;

static USBDM_ErrorCode hc12_alt_speed_detect();

/** Time to hold BKGD pin low after reset pin rise for special modes */
static constexpr unsigned BKGD_WAIT_us = 10;

/** Time to wait for RESET out signal to apply. This should be longer than the soft reset time (30 busy cycles) */
static constexpr unsigned RESET_OUT_TIME_us = 20;

/** How long to wait after reset before new commands are allowed */
static constexpr unsigned RESET_RECOVERY_ms = 30;

/** Max time to wait for the RESET pin to go high after release */
static constexpr unsigned RESET_RELEASE_WAIT_ms = 300;

/** Width of sync pulse (us), 128 target-cycle @ 128kHz (<65536/FTMClock) */
static constexpr unsigned SYNC_WIDTH_us = 1000;

/** Time to wait for end of target sync pulse response (us), (<65536/FTMClock) */
static constexpr unsigned SYNC_TIMEOUT_us = 1200;

/** Longest time after which the target should produce ACKN pulse (150 cycles @ 400kHz = 375us) */
static constexpr unsigned ACKN_TIMEOUT_us = 2500;

/** Length of high drive before 3-state i.e. speed-up pulse (ticks) */
static constexpr unsigned SPEEDUP_PULSE_WIDTH_ticks = 1;

/** FTM to use */
using FtmInfo = Ftm0Info;

/** FTM channel for BKGD out D6(ch6) */
constexpr int bkgdOutChannel = 6;

/** FTM channel for BKGD enable C3(ch2) */
constexpr int bkgdEnChannel = 2;

/** FTM channel for BKGD in D4(ch4) */
constexpr int bkgdInChannel = 4;

/* Make sure pins have been configured for FTM operation */
CheckSignalMapping<FtmInfo, bkgdOutChannel> bkgdOutChannel_chk;
CheckSignalMapping<FtmInfo, bkgdEnChannel>  bkgdEnChannel_chk;
CheckSignalMapping<FtmInfo, bkgdInChannel>  bkgdInChannel_chk;

/** GPIO for SWD-DIN pin */
using bkgdInGpio = GpioTable_T<FtmInfo, bkgdInChannel, ActiveHigh>;

/** Pointer to hardware */
/** Get pointer to FTM hardware as struct */
static constexpr HardwarePtr<FTM_Type> ftm = FtmInfo::baseAddress;

/**
 * Create mask to force control of BKGD output via FTM->SWOCTRL
 *
 * @param enable     Whether to enable buffer (enabled/3-state)
 * @param level      What level to force on pin if enabled
 *
 * @return Required mask
 */
consteval uint32_t SwoCtrlMask(bool enable, bool level) {
   return (enable<<(bkgdEnChannel+8)) |(1<<bkgdEnChannel) | (level<<(bkgdOutChannel+8)) |(1<<bkgdOutChannel);
}

/**
 * Create mask to force control of BKGD output via FTM->SWOCTRL
 *
 * @param pins PinLevelMasks_t control value
 *
 * @return Required mask
 */
constexpr uint32_t SwoCtrlMask(PinLevelMasks_t pins) {

   switch(PinLevelMasks_t(pins&PIN_BKGD_MASK)) {
      default:
      case PIN_BKGD_NC :
         return 0;
      case PIN_BKGD_3STATE :
         return SwoCtrlMask(false, true); // BKGD 3-state
      case PIN_BKGD_LOW :
         return SwoCtrlMask(true, false); // BKGD low
      case PIN_BKGD_HIGH :
         return SwoCtrlMask(true, true);  // BKGD high
   }
}
/**
 * Disable FTM control of BKGD
 */
inline
static void disablePins() {
   ftm->SWOCTRL = SwoCtrlMask(PIN_BKGD_3STATE); // Disable buffer + Force pin high
}

/**
 * Enable FTM control of BKGD
 */
inline
static void enablePins() {
   ftm->SWOCTRL = 0; // Release pin control
}

/**
 * Set pin state
 *
 * @param pins Pin control mask
 *
 * @note Only handles BKGD functions as others (such as reset) are assumed handled in common code
 */
void setPinState(PinLevelMasks_t pins) {
   PinLevelMasks_t selection = PinLevelMasks_t(pins&PIN_BKGD_MASK);
   if (selection == PIN_BKGD_NC) {
      return;
   }
   ftm->SWOCTRL = SwoCtrlMask(selection);
}

/**
 * Get pin status
 *
 * return Status from this interface
 */
PinLevelMasks_t getPinState() {
   return static_cast<PinLevelMasks_t>(bkgdInGpio::isHigh()?PIN_BKGD_HIGH:PIN_BKGD_LOW);
}

inline
static void enableFtmCounter() {
//   ftm->SC =
//        FtmMode_LeftAlign|
//        FtmClockSource_System|
//        FtmPrescale_1;
   ftm->SC =
         FTM_SC_CPWMS(0)| // Left-Aligned
         FTM_SC_CLKS(1)|  // Clock source = SystemBusClock
         FTM_SC_TOIE(0)|  // Timer Overflow Interrupt disabled
         FTM_SC_PS(0);    // Prescale = /1
}

inline
static void disableFtmCounter() {
   ftm->SC = 0;
}

///** PCR for BKGD in pin used by timer */
//using BkgdInPcr        = PcrTable_T<FtmInfo, bkgdInChannel>;
//
///** PCR for BKGD enable pin used by timer */
//using BkgdEnPcr        = PcrTable_T<FtmInfo, bkgdEnChannel>;
//
///** PCR for BKGD in pin used by timer */
//using BkgdOutPcr       = PcrTable_T<FtmInfo, bkgdOutChannel>;
//
/** GPIO for BKGD in pin */
using BkgdIn           = GpioTable_T<FtmInfo, bkgdInChannel, ActiveHigh>;

///** GPIO for BKGD enable pin used by timer */
//using BkgdEn = GpioTable_T<FtmInfo, bkgdEnChannel>;
//
///** GPIO for BKGD in pin used by timer */
//using BkgdOut = GpioTable_T<FtmInfo, bkgdOutChannel>;
//
///** Transceiver for BKGD */
//using Bkgd = Lvc1t45<BkgdEn, BkgdOut>;

/** Measured Target SYNC width (Timer Ticks) */
static unsigned targetSyncWidth;

/** Calculated time for target '1' bit (Timer Ticks) */
static unsigned oneBitTime;

/** Calculated time for target '0' bit (Timer Ticks) */
static unsigned zeroBitTime;

/** Calculated time for to sample target response (Timer Ticks) */
static unsigned sampleBitTime;

/** Calculated time for target minimum bit period (Timer Ticks) */
static unsigned minPeriod;

/** Calculated time for 64 target clock cycles - for ACKN (us) */
static unsigned targetClocks64TimeUS;

/** Calculated time for 150 target clock cycles - for ACKN (us) */
static unsigned targetClocks150TimeUS;

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

   usbdm_assert((long)(uint32_t)t == t, "Interval too large");
   usbdm_assert(t != 0, "Interval truncated 0");

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
   long t = ((uint64_t)time*1000000)/FtmInfo::getInputClockFrequency();

   usbdm_assert((long)(uint32_t)t == t, "Interval too large");
   usbdm_assert(t != 0, "Interval truncated 0");

   return t;
}

/**
 *  Interrupt callback function servicing the interrupt from Vdd changes
 *  This routine has several purposes:
 *   - Triggers POR into Debug mode on RS08/HCS08/CFV1 targets \n
 */
void targetVddSense(VddState) {
   // TODO Bdm::targetVddSense()
}

/**
 * Initialise interface
 */
void initialise() {

   // Enable clock to timer
   FtmInfo::enableClock();

   // Extended features
   ftm->MODE     = FTM_MODE_INIT_MASK|FTM_MODE_FTMEN_MASK|FTM_MODE_WPDIS_MASK;

   // Debug mode
   ftm->CONF     = FTM_CONF_BDMMODE(2);

   // Clear s register changes have immediate effect
   disableFtmCounter();

   // Common registers
   ftm->CNTIN    = 0;
   ftm->CNT      = 0;
   ftm->MOD      = (uint32_t)-1;

   enableFtmCounter();

   ftm->OUTINIT =
         (0<<bkgdEnChannel)|  // Initialise low (disable buffer)
         (1<<bkgdOutChannel); // Initialise high

   ResetInterface::initialise();

   enableFtmCounter();

   disablePins();
   ftm->CONTROLS[bkgdEnChannel].CnSC  = FtmChMode_OutputCompareClear;
   ftm->CONTROLS[bkgdOutChannel].CnSC = FtmChMode_OutputCompareSet;

   // Switch pins to FTM
   FtmInfo::initPCRs();
}

/**
 * Disables BDM interface
 *
 * Note: Reset is not affected
 */
void disableInterface() {
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

   cable_status.sync_length = syncLength;
}

/**
 *  Determine connection speed using sync pulse
 *
 *  @param [out] syncLength Sync length in timer ticks (48MHz)
 *
 *  @return Error code, BDM_RC_OK indicates success
 */
USBDM_ErrorCode sync(uint16_t &syncLength) {

   /** Time to set up Timer - This varies with optimisation! */
   static constexpr unsigned TMR_SETUP_TIME = 40;

   /* SYNC pulse width */
   const uint32_t syncPulseWidthInTicks = convertMicrosecondsToTicks(SYNC_WIDTH_us);

   disableFtmCounter();

   ftm->COMBINE =
         FTM_COMBINE_COMBINE0_MASK<<(bkgdEnChannel*4)|
         FTM_COMBINE_COMBINE0_MASK<<(bkgdOutChannel*4)|
         FTM_COMBINE_DECAPEN0_MASK<<(bkgdInChannel*4);

   // Positive pulse for buffer enable, 2nd edge delayed for speed-up pulse (bkgdEnChannel, bkgdEnChannel+1)
   ftm->CONTROLS[bkgdEnChannel].CnSC    = FtmChMode_CombinePositivePulse;
   ftm->CONTROLS[bkgdEnChannel].CnV     = TMR_SETUP_TIME;
   ftm->CONTROLS[bkgdEnChannel+1].CnV   = TMR_SETUP_TIME+syncPulseWidthInTicks+SPEEDUP_PULSE_WIDTH_ticks;

   // Negative pulse for BKGD out (bkgdOutChannel, bkgdOutChannel+1)
   ftm->CONTROLS[bkgdOutChannel].CnSC   = FtmChMode_CombineNegativePulse;
   ftm->CONTROLS[bkgdOutChannel].CnV    = TMR_SETUP_TIME;
   ftm->CONTROLS[bkgdOutChannel+1].CnV  = TMR_SETUP_TIME+syncPulseWidthInTicks;

   // Enable dual capture on BKGD in (bkgdInChannel, bkgdInChannel+1)
   ftm->CONTROLS[bkgdInChannel].CnSC    = FtmChMode_DualEdgeCaptureOneShotFallingEdge;
   ftm->CONTROLS[bkgdInChannel+1].CnSC  = FtmChMode_InputCaptureRisingEdge;

   // Release force pin control
   enablePins();

   // Start counter from 0
   ftm->CNT = 0;
   enableFtmCounter();

   // Clear channel flags
   ftm->STATUS = ftm->STATUS & ~(
         (1<<bkgdEnChannel) |(1<<(bkgdEnChannel+1))|
         (1<<bkgdOutChannel)|(1<<(bkgdOutChannel+1))|
         (1<<bkgdInChannel) |(1<<(bkgdInChannel+1)));

   static auto pollPulseStart = [] {
         return ((ftm->CONTROLS[bkgdOutChannel].CnSC&FTM_CnSC_CHF_MASK) != 0);
   };
   // Wait for start of pulse
   waitUS(2*TMR_SETUP_TIME, pollPulseStart);

   // Enable dual-edge capture on BKGD_In
   ftm->COMBINE = ftm->COMBINE | FTM_COMBINE_DECAP0_MASK<<(bkgdInChannel*4);

   static auto pollDecap = [] {
         return ((ftm->COMBINE&(FTM_COMBINE_DECAP0_MASK<<(bkgdInChannel*4))) == 0);
   };
   // Wait for dual-edge capture
   bool success = waitUS(SYNC_TIMEOUT_us, pollDecap);

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
 *  @return BDM_RC_OK          Success
 *  @return BDM_RC_ACK_TIMEOUT No ACKN detected [timeout]
 */
USBDM_ErrorCode acknowledgeOrWait64() {
   if (cable_status.ackn==ACKN) {
      // Wait for pin capture or timeout
      static auto fn = [] {
            return ((ftm->CONTROLS[bkgdInChannel].CnSC&FTM_CnSC_CHF_MASK) != 0) ||
                   ((ftm->CONTROLS[bkgdInChannel+1].CnSC&FTM_CnSC_CHF_MASK) != 0);
      };
      waitMS(ACKN_TIMEOUT_us, fn);
      if ((ftm->CONTROLS[bkgdInChannel].CnSC&FTM_CnSC_CHF_MASK) == 0) {
         // No ACKN - Return timeout error
         return BDM_RC_ACK_TIMEOUT;
      }
   }
   else {
      waitUS(targetClocks64TimeUS);
   }
   return BDM_RC_OK;
}

/**
 *  Depending on ACKN mode this function:       \n
 *    - Waits for ACKN pulse with timeout.
 *       OR
 *    - Busy waits for 150 target CPU clocks
 *
 *  @return BDM_RC_OK          Success
 *  @return BDM_RC_ACK_TIMEOUT No ACKN detected [timeout]
 */
USBDM_ErrorCode acknowledgeOrWait150() {
   if (cable_status.ackn==ACKN) {
      // Wait for pin capture or timeout
      static auto fn = [] {
            return ((ftm->CONTROLS[bkgdInChannel].CnSC&FTM_CnSC_CHF_MASK) != 0) ||
                   ((ftm->CONTROLS[bkgdInChannel+1].CnSC&FTM_CnSC_CHF_MASK) != 0);
      };
      waitMS(ACKN_TIMEOUT_us, fn);
      if ((ftm->CONTROLS[bkgdInChannel].CnSC&FTM_CnSC_CHF_MASK) == 0) {
         // No ACKN - Return timeout error
         return BDM_RC_ACK_TIMEOUT;
      }
   }
   else {
      waitUS(targetClocks150TimeUS);
   }
   return BDM_RC_OK;
}

/**
 * Receive an value over BDM interface
 *
 * @param [in]  length Number of bits to receive
 * @param [out] data   Data received
 *
 * @return BDM_RC_OK => Success, error otherwise
 *
 * @note FTM use:\n
 *    bkgdEnChannel,bkgdEnChannel+1   = Positive pulse for buffer enable   \n
 *    bkgdOutChannel,bkgdOutChannel+1 = Negative pulse for BKGD out        \n
 *    bkgdInChannel                   = Sampling of data bit from target
 */
USBDM_ErrorCode rx(int length, unsigned &data) {

   /** Time to set up Timer - This varies with optimisation! */
   static constexpr unsigned TMR_SETUP_TIME = 20;

   disableFtmCounter();

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
   ftm->CONTROLS[bkgdInChannel].CnSC    = FtmChMode_InputCaptureRisingEdge;

   enableFtmCounter();

   bool success = true;
   unsigned value = 0;
   while (length-->0) {

      // Restart counter
      ftm->CNT = 0;

      // Clear channel flags
      ftm->STATUS = ftm->STATUS & ~(
            (1<<bkgdEnChannel) |(1<<(bkgdEnChannel+1))|
            (1<<bkgdOutChannel)|(1<<(bkgdOutChannel+1))|
            (1<<bkgdInChannel));

      // Wait until end of bit
      do {
      } while (ftm->CNT <= TMR_SETUP_TIME+minPeriod);

      // Should have captured a rising edge from target
      success = success && ((ftm->CONTROLS[bkgdInChannel].CnSC & FTM_CnSC_CHF_MASK) != 0);

      // Use time of rise to determine bit value
      value = (value<<1)|((ftm->CONTROLS[bkgdInChannel].CnV>(TMR_SETUP_TIME+sampleBitTime))?0:1);
   }
   if (!success) {
      return BDM_RC_BKGD_TIMEOUT;
   }
   data = value;
   return BDM_RC_OK;
}

/**
 * Receive an 8-bit value over BDM interface
 *
 * @param [out] data   Data received
 *
 * @return BDM_RC_OK => Success, error otherwise
 */
inline
USBDM_ErrorCode rx8(uint8_t *data) {
   unsigned value = 0;
   USBDM_ErrorCode rc = rx(8, value);
   *data = (uint8_t)value;
   return rc;
}

/**
 * Receive an 16-bit value over BDM interface
 *
 * @param [out] data   Data received
 *
 * @return BDM_RC_OK => Success, error otherwise
 */
inline
USBDM_ErrorCode rx16(uint8_t *data) {
   unsigned value = 0;
   USBDM_ErrorCode rc = rx(16, value);
   unpack16BE(value, data);
   return rc;
}

/**
 * Receive an 32-bit value over BDM interface
 *
 * @param [out] data   Data received
 *
 * @return BDM_RC_OK => Success, error otherwise
 */
inline
USBDM_ErrorCode rx32(uint8_t *data) {
   unsigned value = 0;
   USBDM_ErrorCode rc = rx(32, value);
   unpack32BE(value, data);
   return rc;
}

/**
 * Set up for transmission
 */
inline
void transactionStart() {
   ftm->SYNCONF = FTM_SYNCONF_SYNCMODE(1)|FTM_SYNCONF_SWWRBUF(1);

   __disable_irq();
   enablePins();
}

/**
 * End transmission phase
 */
inline
void transactionComplete() {
//   disableFtmCounter();
   disablePins();
   __enable_irq();
}

/**
 * Transmit a value over BDM interface
 *
 * @param [in]  length Number of bits to transmit
 * @param [in]  data   Data value to transmit
 *
 * @return Error code, BDM_RC_OK indicates success
 *
 * @note FTM use:\n
 *    bkgdEnChannel,bkgdEnChannel+1   = Positive pulse for buffer enable   \n
 *    bkgdOutChannel,bkgdOutChannel+1 = Negative pulse for BKGD out, width modified by data 0/1 \n
 *    bkgdInChannel                   = ACKN capture   \n
 *    bkgdInChannel+1                 = ACKN timeout   \n
 * bkgdInChannel,bkgdInChannel+1 are left setup for ACKN.  ACKN is not waited for.
 */
USBDM_ErrorCode tx(int length, unsigned data) {

   /* Time to set up Timer - This varies with optimisation! */
   static constexpr unsigned TMR_SETUP_TIME = 20;
   uint32_t mask = (1U<<(length-1));

   ftm->COMBINE =
         FTM_COMBINE_SYNCEN0_MASK<<(bkgdEnChannel*4)|
         FTM_COMBINE_COMBINE0_MASK<<(bkgdEnChannel*4)|
         FTM_COMBINE_SYNCEN0_MASK<<(bkgdOutChannel*4)|
         FTM_COMBINE_COMBINE0_MASK<<(bkgdOutChannel*4);

   // Disable so immediate effect
   disableFtmCounter();

   // Positive pulse for buffer enable
   ftm->CONTROLS[bkgdEnChannel].CnSC    = FtmChMode_CombinePositivePulse;
   ftm->CONTROLS[bkgdEnChannel].CnV     = TMR_SETUP_TIME;

   // Negative pulse for BKGD out
   ftm->CONTROLS[bkgdOutChannel].CnSC   = FtmChMode_CombineNegativePulse;
   ftm->CONTROLS[bkgdOutChannel].CnV    = TMR_SETUP_TIME;

   // Data sample capture rising edge of BKGD in
   ftm->CONTROLS[bkgdInChannel].CnSC    = FtmChMode_InputCaptureRisingEdge;

   // ACKN timeout
   ftm->CONTROLS[bkgdInChannel+1].CnSC  = FtmChMode_OutputCompare;
   ftm->CONTROLS[bkgdInChannel+1].CnV   = TMR_SETUP_TIME+ACKN_TIMEOUT_us;

   // Maximum length of a bit
   const uint16_t maxBitTime = TMR_SETUP_TIME+minPeriod;
   while (mask>0) {
      int width;
      if (data&mask) {
         width = TMR_SETUP_TIME+oneBitTime;
      }
      else {
         width = TMR_SETUP_TIME+zeroBitTime;
      }
      mask >>= 1;

      disableFtmCounter();
      ftm->CNT = 0;
      ftm->CONTROLS[bkgdOutChannel+1].CnV  = width;
      ftm->CONTROLS[bkgdEnChannel+1].CnV   = width+SPEEDUP_PULSE_WIDTH_ticks;
      ftm->SYNC = FTM_SYNC_SWSYNC(1);

      enableFtmCounter();

      // Wait until end of bit
      do {
      } while (ftm->CNT < maxBitTime);
   }
   // Clear channel flags for ACKN pulse
   ftm->STATUS = ftm->STATUS & ~(
         (1<<bkgdEnChannel) |(1<<(bkgdEnChannel+1))|
         (1<<bkgdOutChannel)|(1<<(bkgdOutChannel+1))|
         (1<<bkgdInChannel)|(1<<(bkgdInChannel+1)));

   return BDM_RC_OK;
}

/**
 * Transmit an 8-bit value over BDM interface
 *
 * @param [in]  data   Data value to transmit
 *
 * @return Error code, BDM_RC_OK indicates success
 */
inline
USBDM_ErrorCode tx8(uint8_t data) {
   return tx(8, data);
}

/**
 * Transmit an 16-bit value over BDM interface
 *
 * @param [in]  data   Data value to transmit
 *
 * @return Error code, BDM_RC_OK indicates success
 */
inline
USBDM_ErrorCode tx16(uint16_t data) {
   return tx(16, data);
}

/**
 * Transmit an 24-bit value over BDM interface
 *
 * @param [in]  data   Data value to transmit
 *
 * @return Error code, BDM_RC_OK indicates success
 */
inline
USBDM_ErrorCode tx24(uint32_t data) {
   return tx(24, data);
}

/**
 * Transmit an 32-bit value over BDM interface
 *
 * @param [in]  data   Data value to transmit
 *
 * @return Error code, BDM_RC_OK indicates success
 */
inline
USBDM_ErrorCode tx32(uint32_t data) {
   return tx(32, data);
}

/*
 * ***************************************************************
 * The following commands DO NOT expect an ACK & do not delay
 * Interrupts are left disabled!
 * ***************************************************************
 */

/**
 * Write command byte, truncated sequence
 *
 * @param cmd command byte to write
 *
 * @note Interrupts are left disabled, no ACK is expected
 */
void cmd_0_0_T(uint8_t cmd) {
   transactionStart();
   tx8(cmd);
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
   transactionStart();
   tx8(cmd);
   tx8(parameter);
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
   transactionStart();
   tx8(cmd);
   tx16(parameter1);
   tx8(parameter2);
}

/*
 * ***************************************************************
 * The following commands DO NOT expect an ACK & do not delay
 * ***************************************************************
 */

/**
 *  Write cmd without ACK (HCS08/RS08/CFV1)
 *
 *  @param cmd command byte to write
 *
 *  @note No ACK is expected
 */
void cmd_0_0_NOACK(uint8_t cmd) {
   transactionStart();
   tx8(cmd);
   transactionComplete();
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
   transactionStart();
   tx8(cmd);
   rx8(result);
   transactionComplete();
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
   transactionStart();
   tx8(cmd);
   tx8(parameter);
   transactionComplete();
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
   transactionStart();
   tx8(cmd);
   rx16(result);
   transactionComplete();
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
   transactionStart();
   tx8(cmd);
   tx16(parameter);
   transactionComplete();
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
   transactionStart();
   tx8(cmd);
   tx16(parameter);
   rx16(result);
   transactionComplete();
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
   transactionStart();
   tx8(cmd);
   tx16(parameter);
   tx8(value);
   rx8(status);
   transactionComplete();
}

/*
 * *****************************************************************************
 * The following commands DO expect an ACK or wait at end of the transmit phase
 * *****************************************************************************
 */

/**
 *  Write cmd
 *
 *  @param cmd command byte to write
 *
 *  @return BDM_RC_OK            Success
 *  @return BDM_RC_ACK_TIMEOUT   Missing ACKN
 *
 *  @note ACK is expected
 */
USBDM_ErrorCode cmd_0_0(uint8_t cmd) {
   transactionStart();
   tx8(cmd);
   USBDM_ErrorCode rc = acknowledgeOrWait64();
   transactionComplete();
   return rc;
}

/**
 *  Write cmd, read byte
 *
 *  @param cmd        command byte to write
 *  @param result     byte read
 *
 *  @return BDM_RC_OK            Success
 *  @return BDM_RC_ACK_TIMEOUT   Missing ACKN
 *
 *  @note ACK is expected
 */
USBDM_ErrorCode cmd_0_1B(uint8_t cmd, uint8_t *result) {
   transactionStart();
   tx8(cmd);
   USBDM_ErrorCode rc = acknowledgeOrWait64();
   rx8(result);
   transactionComplete();
   return rc;
}

/**
 *  Write cmd & read word
 *
 *  @param cmd    command byte to write
 *  @param result word read
 *
 *  @return BDM_RC_OK            Success
 *  @return BDM_RC_ACK_TIMEOUT   Missing ACKN
 *
 *  @note ACK is expected
 */
USBDM_ErrorCode cmd_0_1W(uint8_t cmd, uint8_t *result) {
   transactionStart();
   tx8(cmd);
   USBDM_ErrorCode rc = acknowledgeOrWait64();
   rx16(result);
   transactionComplete();
   return rc;
}

/**
 *  Write cmd & read longword
 *
 *  @param cmd    command byte to write
 *  @param result longword read
 *
 *  @return BDM_RC_OK            Success
 *  @return BDM_RC_ACK_TIMEOUT   Missing ACKN
 *
 *  @note ACK is expected
 */
USBDM_ErrorCode cmd_0_1L(uint8_t cmd, uint8_t result[4]) {
   transactionStart();
   tx8(cmd);
   USBDM_ErrorCode rc = acknowledgeOrWait64();
   rx32(result);
   transactionComplete();
   return rc;
}

/**
 *  Write cmd & byte
 *
 *  @param cmd        command byte to write
 *  @param parameter  byte to write
 *
 *  @return BDM_RC_OK            Success
 *  @return BDM_RC_ACK_TIMEOUT   Missing ACKN
 *
 *  @note ACK is expected
 */
USBDM_ErrorCode cmd_1B_0(uint8_t cmd, uint8_t parameter) {
   transactionStart();
   tx8(cmd);
   tx8(parameter);
   USBDM_ErrorCode rc = acknowledgeOrWait64();
   transactionComplete();
   return rc;
}

/**
 *  Write cmd & word
 *
 *  @param cmd       command byte to write
 *  @param parameter word to write
 *
 *  @return BDM_RC_OK            Success
 *  @return BDM_RC_ACK_TIMEOUT   Missing ACKN
 *
 *  @note ACK is expected
 */
USBDM_ErrorCode cmd_1W_0(uint8_t cmd, uint16_t parameter) {
   transactionStart();
   tx8(cmd);
   tx16(parameter);
   USBDM_ErrorCode rc = acknowledgeOrWait64();
   transactionComplete();
   return rc;
}

/**
 *  Write cmd & longword
 *
 *  @param cmd       command byte to write
 *  @param parameter longword to write
 *
 *  @return BDM_RC_OK            Success
 *  @return BDM_RC_ACK_TIMEOUT   Missing ACKN
 *
 *  @note ACK is expected
 */
USBDM_ErrorCode cmd_1L_0(uint8_t cmd, uint32_t parameter) {
   transactionStart();
   tx8(cmd);
   tx32(parameter);
   USBDM_ErrorCode rc = acknowledgeOrWait64();
   transactionComplete();
   return rc;
}

/**
 *  Write cmd, word & read byte (read word but return byte - HC/S12(x))
 *
 *  @param cmd       command byte to write
 *  @param parameter word to write
 *  @param result    byte read
 *
 *  @return BDM_RC_OK            Success
 *  @return BDM_RC_ACK_TIMEOUT   Missing ACKN
 *
 *  @note ACK is expected
 */
USBDM_ErrorCode cmd_1W_1WB(uint8_t cmd, uint16_t parameter, uint8_t *result) {
   transactionStart();
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
   transactionComplete();
   return rc;
}

/**
 *  Write cmd & 2 words
 *
 *  @param cmd       command byte to write
 *  @param parameter1 word to write
 *  @param parameter2 word to write
 *
 *  @return BDM_RC_OK            Success
 *  @return BDM_RC_ACK_TIMEOUT   Missing ACKN
 *
 *  @note ACK is expected
 */
USBDM_ErrorCode cmd_2W_0(uint8_t cmd, uint16_t parameter1, uint16_t parameter2) {
   transactionStart();
   tx8(cmd);
   tx16(parameter1);
   tx16(parameter2);
   USBDM_ErrorCode rc = acknowledgeOrWait150();
   transactionComplete();
   return rc;
}

/**
 *  Write cmd, word & read word
 *
 *  @param cmd        command byte to write
 *  @param parameter  word to write
 *  @param result     word read
 *
 *  @return BDM_RC_OK            Success
 *  @return BDM_RC_ACK_TIMEOUT   Missing ACKN
 *
 *  @note ACK is expected
 */
USBDM_ErrorCode cmd_1W_1W(uint8_t cmd, uint16_t parameter, uint8_t result[2]) {
   transactionStart();
   tx8(cmd);
   tx16(parameter);
   USBDM_ErrorCode rc = acknowledgeOrWait150();
   rx16(result);
   transactionComplete();
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
 *  @return BDM_RC_OK            Success
 *  @return BDM_RC_ACK_TIMEOUT   Missing ACKN
 *
 *  @note ACK is expected
 */
USBDM_ErrorCode cmd_1W1B_1B(uint8_t cmd, uint16_t parameter, uint8_t value, uint8_t *status) {
   transactionStart();
   tx8(cmd);
   tx16(parameter);
   tx8(value);
   USBDM_ErrorCode rc = acknowledgeOrWait64();
   rx8(status);
   transactionComplete();
   return rc;
}
/**
 *  Write cmd, word and a byte
 *  (sends 2 words, the byte in both high and low byte of the 16-bit value)
 *
 *  @param cmd        command byte to write
 *  @param parameter1 word to write
 *  @param parameter2 byte to write
 *
 *  @return BDM_RC_OK            Success
 *  @return BDM_RC_ACK_TIMEOUT   Missing ACKN
 *
 *  @note ACK is expected
 */
USBDM_ErrorCode cmd_2WB_0(uint8_t cmd, uint16_t parameter1, uint8_t parameter2) {
   transactionStart();
   tx8(cmd);
   tx16(parameter1);
   tx8(parameter2);
   tx8(parameter2);
   USBDM_ErrorCode rc = acknowledgeOrWait150();
   transactionComplete();
   return rc;
}

/**
 *  Write cmd, word & read byte
 *
 *  @param cmd        command byte to write
 *  @param parameter  word to write
 *  @param result     byte read
 *
 *  @return BDM_RC_OK            Success
 *  @return BDM_RC_ACK_TIMEOUT   Missing ACKN
 *
 *  @note ACK is expected
 */
USBDM_ErrorCode cmd_1W_1B(uint8_t cmd, uint16_t parameter, uint8_t *result) {
   transactionStart();
   tx8(cmd);
   tx16(parameter);
   USBDM_ErrorCode rc = acknowledgeOrWait64();
   rx8(result);
   transactionComplete();
   return rc;
}

/**
 *  Write cmd, word & byte
 *
 *  @param cmd         command byte to write
 *  @param parameter1  word to write
 *  @param parameter2  byte to write
 *
 *  @return BDM_RC_OK            Success
 *  @return BDM_RC_ACK_TIMEOUT   Missing ACKN
 *
 *  @note ACK is expected
 */
USBDM_ErrorCode cmd_1W1B_0(uint8_t cmd, uint16_t parameter1, uint8_t parameter2) {
   transactionStart();
   tx8(cmd);
   tx16(parameter1);
   tx8(parameter2);
   USBDM_ErrorCode rc = acknowledgeOrWait150();
   transactionComplete();
   return rc;
}

/**
 *  Write cmd, 24-bit value & byte
 *
 *  @param cmd    command byte to write
 *  @param addr   24-bit value to write
 *  @param value  byte to write
 *
 *  @return BDM_RC_OK            Success
 *  @return BDM_RC_ACK_TIMEOUT   Missing ACKN
 *
 *  @note ACK is expected
 */
USBDM_ErrorCode cmd_1A1B_0(uint8_t cmd, uint32_t addr, uint8_t value) {
   transactionStart();
   tx8(cmd);
   tx24(addr);
   tx8(value);
   USBDM_ErrorCode rc = acknowledgeOrWait150();
   transactionComplete();
   return rc;
}

/**
 *  Write cmd, 24-bit value & word
 *
 *  @param cmd    command byte to write
 *  @param addr   24-bit value to write
 *  @param value  word to write
 *
 *  @return BDM_RC_OK            Success
 *  @return BDM_RC_ACK_TIMEOUT   Missing ACKN
 *
 *  @note ACK is expected
 */
USBDM_ErrorCode cmd_1A1W_0(uint8_t cmd, uint32_t addr, uint16_t value) {
   transactionStart();
   tx8(cmd);
   tx24(addr);
   tx16(value);
   USBDM_ErrorCode rc = acknowledgeOrWait150();
   transactionComplete();
   return rc;
}

/**
 *  Write cmd, 24-bit value & longword
 *
 *  @param cmd    command byte to write
 *  @param addr   24-bit value to write
 *  @param value  ptr to longword to write
 *
 *  @return BDM_RC_OK            Success
 *  @return BDM_RC_ACK_TIMEOUT   Missing ACKN
 *
 *  @note ACK is expected
 */
USBDM_ErrorCode cmd_1A1L_0(uint8_t cmd, uint32_t addr, uint32_t value) {
   transactionStart();
   tx8(cmd);
   tx24(addr);
   tx32(value);
   USBDM_ErrorCode rc = acknowledgeOrWait150();
   transactionComplete();
   return rc;
}

/**
 *  Write cmd, 24-bit value & read byte
 *
 *  @param cmd     command byte to write
 *  @param addr    24-bit value to write
 *  @param result  ptr to read location
 *
 *  @return BDM_RC_OK            Success
 *  @return BDM_RC_ACK_TIMEOUT   Missing ACKN
 *
 *  @note ACK is expected
 */
USBDM_ErrorCode cmd_1A_1B(uint8_t cmd, uint32_t addr, uint8_t *result) {
   transactionStart();
   tx8(cmd);
   tx24(addr);
   USBDM_ErrorCode rc = acknowledgeOrWait150();
   rx8(result);
   transactionComplete();
   return rc;
}

/**
 *  Write cmd, 24-bit value & read 16-bit value
 *
 *  @param cmd     command byte to write
 *  @param addr    24-bit value to write
 *  @param result  pointer to read location
 *
 *  @return BDM_RC_OK            Success
 *  @return BDM_RC_ACK_TIMEOUT   Missing ACKN
 *
 *  @note ACK is expected
 */
USBDM_ErrorCode cmd_1A_1W(uint8_t cmd, uint32_t addr, uint8_t *result) {
   transactionStart();
   tx8(cmd);
   tx24(addr);
   USBDM_ErrorCode rc = acknowledgeOrWait150();
   rx16(result);
   transactionComplete();
   return rc;
}

/**
 *  Write cmd, 24-bit value & read 32-bit value
 *
 *  @param cmd     command byte to write
 *  @param addr    24-bit value to write
 *  @param result  pointer to read location
 *
 *  @return BDM_RC_OK            Success
 *  @return BDM_RC_ACK_TIMEOUT   Missing ACKN
 *
 *  @note ACK is expected
 */
USBDM_ErrorCode cmd_1A_1L(uint8_t cmd, uint32_t addr, uint8_t *result) {
   transactionStart();
   tx8(cmd);
   tx24(addr);
   USBDM_ErrorCode rc = acknowledgeOrWait150();
   rx32(result);
   transactionComplete();
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
      case T_S12Z  :
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
      case T_S12Z  : {
         uint8_t temp[2];
         Bdm::BDMZ12_CMD_READ_BDCCSR(temp);
         *status = temp[0];
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
void enableACKNMode() {
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
 *  It does wait for BKGD and optionally RESET to be inactive
 *
 * @return BDM_RC_OK                  => Success
 * @return BDM_RC_VDD_NOT_PRESENT     => No target power present
 * @return BDM_RC_RESET_TIMEOUT_RISE  => RESET signal timeout - remained low
 * @return BDM_RC_BKGD_TIMEOUT        => BKGD signal timeout - remained low
 * @return != BDM_RC_OK               => Other failures
 */
USBDM_ErrorCode physicalConnect() {
   USBDM_ErrorCode rc;

   // Assume we know nothing about connection technique & speed
   cable_status.speed = SPEED_NO_INFO;

   // Target has power?
   rc = checkTargetVdd();
   if (rc != BDM_RC_OK) {
      return rc;
   }
   if (bdm_option.useResetSignal) {
      // Wait with timeout until RESET is high
      if (ResetInterface::isLow()) {
         // TODO This may take a while
         //         setBDMBusy();
         if (!waitMS(RESET_RELEASE_WAIT_ms, ResetInterface::isHigh)) {
            // RESET timeout
            return(BDM_RC_RESET_TIMEOUT_RISE);
         }
      }
   }
   // Wait with timeout until BKGD is high
   if (!waitMS(RESET_RELEASE_WAIT_ms, BkgdIn::isHigh)) {
      // BKGD timeout
      return(BDM_RC_BKGD_TIMEOUT);
   }
   // Try SYNC method
   uint16_t syncLength;
   rc = sync(syncLength);
   if (rc != BDM_RC_OK) {
      // Try again
      rc = sync(syncLength);
   }
   if (rc == BDM_RC_OK) {
      // Speed determined by SYNC method
      cable_status.speed = SPEED_SYNC;
   }
   else if ((bdm_option.guessSpeed) && (cable_status.target_type == T_HC12)) {
      // Try alternative method if enabled and HC12 target
      rc = hc12_alt_speed_detect();
   }
   if (rc == BDM_RC_OK) {
      // Try the ACKN feature
      enableACKNMode();
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

   // Get current status
   rc = readBDMStatus(&bdm_sts);
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
USBDM_ErrorCode connect() {
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
      setPinState(PIN_BKGD_LOW);
   }
   __enable_irq();

   // Wait for target to start internal reset (and possibly usbdm_assert reset output)
   waitUS(RESET_OUT_TIME_us);

   if (bdm_option.useResetSignal) {
      // Wait with timeout until RESET is high
      if (!waitMS(RESET_RELEASE_WAIT_ms, ResetInterface::isHigh)) {
         // RESET timeout
         return(BDM_RC_RESET_TIMEOUT_RISE);
      }
   }
   else {
      // Allow time for RESET to rise
      waitMS(RESET_RELEASE_WAIT_ms);
   }
   if (mode == RESET_SPECIAL) {
      // Wait for BKGD assertion time after reset rise
      waitUS(BKGD_WAIT_us);
      // Special mode - release BKGD
      setPinState(PIN_BKGD_3STATE);
   }
   // Wait recovery time before allowing anything else to happen on the BDM
   waitMS(RESET_RECOVERY_ms);

   // Place interface in idle state
   disablePins();

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
USBDM_ErrorCode halt() {
   if ((cable_status.target_type==T_CFV1)||(cable_status.target_type==T_S12Z))
      return BDMCF_CMD_BACKGROUND();
   else
      return BDM_CMD_BACKGROUND();
}

/**
 * Commences full-speed execution on the target
 */
USBDM_ErrorCode go() {
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
USBDM_ErrorCode step() {
   if (cable_status.target_type == T_CFV1) {
      // Set Single-step mode
      uint32_t csr;
      BDMCF_CMD_READ_DREG(CFV1_CSR, csr);
      csr |= CFV1_CSR_SSM;
      BDMCF_CMD_WRITE_DREG(CFV1_CSR, csr);
      return BDMCF_CMD_GO();
   }
#if TARGET_CAPABILITY & CAP_S12Z
   else if (cable_status.target_type == T_S12Z  ) {
      return BDMZ12_CMD_TRACE1();
   }
#endif
   else {
      return BDM_CMD_TRACE1();
   }
}

 /**
  *  Confirm communication at given Sync value.
  *  Only works on HC12 (and maybe only 1 of 'em!)
  *
  *  @return
  *    == \ref BDM_RC_OK  => Success \n
  *    != \ref BDM_RC_OK  => Various errors
  */
USBDM_ErrorCode hc12confirmSpeed(unsigned syncLength) {
   // PARTID retrieved from HCS12
   // Used to quickly confirm target connection speed and avoid needless probing
   static uint16_t partid = 0xFA50;

   setSyncLength(syncLength);

   // Assume probing failed
   USBDM_ErrorCode rc = BDM_RC_BDM_EN_FAILED;
   uint8_t probe[2];
   uint16_t newPartid;

   do {
      // Check if we can read a previously retrieved PARTID.
      // If so, assume still connected and avoid further target probing
      // This should be the usual case
      if (BDM12_CMD_READW(HCS12_PARTID, probe) != BDM_RC_OK) {
         // Can't even read PartID - give up entirely
         break;
      }
      newPartid = pack16BE(probe);
      if (newPartid == partid) {
         // PartID unchanged assume connected at this speed
         return BDM_RC_OK;
      }

      do {
         // This method works for secured or unsecured devices
         // in special mode that have a common Flash type.
         // BUT - it may upset flash programming if done at wrong time

         // Flash controller register to access
         static constexpr uint16_t FDATA_ADDR = 0x10A;

         // Set FDATA to 0xAA55 & read back
         if ((BDM12_CMD_WRITEW(FDATA_ADDR, 0xAA55) != BDM_RC_OK) ||
               (BDM12_CMD_READW(FDATA_ADDR, probe) != BDM_RC_OK) ||
               (pack16BE(probe) != 0xAA55)) {
            break;
         }
         // Set FDATA to 0x55AA & read back
         if ((BDM12_CMD_WRITEW(FDATA_ADDR, 0x55AA) != BDM_RC_OK) ||
               (BDM12_CMD_READW(FDATA_ADDR, probe) != BDM_RC_OK) ||
               (pack16BE(probe) != 0x55AA)) {
            break;
         }
         // Update partID
         partid = newPartid;

         // Success!
         return BDM_RC_OK;

      } while (false);

      do {
         uint8_t originalValue;
         // This method works for unsecured devices
         // in special or non-special modes
         // BUT - it may upset CCR in some (unlikely?) cases

         // Get current BDMCCR
         if (BDM12_CMD_BDREADB(HC12_BDMCCR,&originalValue) != BDM_RC_OK) {
            break;
         }
         // Set BDMCCR to 0xAA & read back
         if ((BDM12_CMD_BDWRITEB(HC12_BDMCCR, 0xAA) != BDM_RC_OK) ||
               (BDM12_CMD_BDREADB(HC12_BDMCCR, probe) != BDM_RC_OK) ||
               (probe[0] != 0xAA)) {
            break;
         }
         // Set BDMCCR to 0x55 & read back
         if ((BDM12_CMD_BDWRITEB(HC12_BDMCCR, 0x55) != BDM_RC_OK) ||
               (BDM12_CMD_BDREADB(HC12_BDMCCR, probe) != BDM_RC_OK) ||
               (probe[0] != 0x55)) {
            break;
         }
         // Restore BDMCCR
         BDM12_CMD_BDWRITEB(HC12_BDMCCR, originalValue);

         // Update partID
         partid = newPartid;

         // Success!
         return BDM_RC_OK;
      } while (false);
   } while (false);

   cable_status.sync_length  = 1;
   cable_status.ackn         = WAIT;    // Clear indication of ACKN feature
   return rc;
}
/**
 * Converts a frequency to the expected sync value from a HCS target
 *
 * @param frequency
 *
 * @return Sync value in microseconds
 */
constexpr uint32_t convertFrequencyToSyncValue(uint32_t frequency) {
   return (125*1000000)/frequency;
}

/**  Attempt to determine target speed by trial and error
 *
 *  Basic process used to check for communication is:
 *    -  Attempt to modify the BDM Status register [BDMSTS] or BDM CCR Save Register [BDMCCR]
 *
 *  The above is attempted for a range of 'nice' frequencies and then every Tx driver frequency. \n
 *  To improve performance the last two successful frequencies are remembered.  This covers the \n
 *  common case of alternating between two frequencies [reset & clock configured] with a minimum \n
 *  number of probes.
 */
static USBDM_ErrorCode hc12_alt_speed_detect() {
static const uint32_t typicalSpeeds[] = {
      // Table of 'nice' BDM speeds to try
      8000000,
      16000000,
   0
   };
static uint16_t lastGuess1 = convertFrequencyToSyncValue(8000000);  // Used to remember last 2 guesses
static uint16_t lastGuess2 = convertFrequencyToSyncValue(16000000); // Common situation to change between 2 speeds (reset,running)
int sub;
uint16_t currentGuess;
USBDM_ErrorCode  rc;

   // Try last used speed #1
   if (hc12confirmSpeed(lastGuess1) == BDM_RC_OK) {
      cable_status.speed = SPEED_GUESSED;  // Speed found by trial and error
      return BDM_RC_OK;
   }
   // Try last used speed #2
   currentGuess = lastGuess2;
   rc = hc12confirmSpeed(lastGuess2);
   if (rc != BDM_RC_OK) {
      // TODO This may take a while
//      setBDMBusy();
   }
   // Try some likely numbers!
   for (sub=0; typicalSpeeds[sub]>0; sub++) {
      rc = hc12confirmSpeed(lastGuess1);
      if (rc == BDM_RC_OK) {
         break;
      }
      currentGuess = convertFrequencyToSyncValue(typicalSpeeds[sub]);
      rc           = hc12confirmSpeed(currentGuess);
      }
   if (rc == BDM_RC_OK) {
      // Update speed cache (LRU)
      lastGuess2         = lastGuess1;
      lastGuess1         = currentGuess;
      cable_status.speed = SPEED_GUESSED;  // Speed found by trial and error
      return BDM_RC_OK;
   }
   return rc;
}

}; // End namespace Bdm
