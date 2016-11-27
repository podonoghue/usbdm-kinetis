/*
 * bdm.h
 *
 *  Created on: 26Nov.,2016
 *      Author: podonoghue
 */

#ifndef SOURCES_BDM_H_
#define SOURCES_BDM_H_

#include "system.h"
#include "derivative.h"
#include "hardware.h"
#include "delay.h"
#include "USBDM_MK20D5.h"

// D6 = C6, C2 = C3
//using Bkgd = Lvc1t45<USBDM::GpioD<6>, USBDM::GpioC<3>>;

/**
 * Class representing a low-level interface to a RS08/HCS08/HCS12 background debug module
 *
 * @tparam Info               Information class for FTM
 * @tparam bkgdOutChannel     Channel to use for BKGD output to driver
 * @tparam bkgdEnChannel      Channel to use for BKGD output to driver enable
 * @tparam bkgdInChannel      Channel to use for BKGD input from receiver
 */
template <class Info, uint8_t bkgdOutChannel, uint8_t bkgdEnChannel, uint8_t bkgdInChannel>
class BackgroundDebug_T :
   public
   USBDM::CheckSignal<Info, bkgdOutChannel>,
   USBDM::CheckSignal<Info, bkgdEnChannel>,
   USBDM::CheckSignal<Info, bkgdInChannel> {

protected:
   /** How long to wait for target response per bit (us) */
   static constexpr uint BIT_TIME_OUT            = 100;

   /** Width of sync pulse (us), 128 target-cycle @ 128kHz (<65536/FTMClock) */
   static constexpr uint SYNC_WIDTH              = 1000;

   /** Time to wait for end of target sync pulse response (us), (<65536/FTMClock) */
   static constexpr uint SYNC_TIMEOUT            = 1200;

   /** Time to set up Timer - This varies with optimisation! */
   static constexpr uint TMR_SETUP_TIME          = 40;//15;

   /** Length of high drive before 3-state i.e. speed-up pulse (ticks) */
   static constexpr uint SPEEDUP_PULSE_WIDTH_IN_TICKS        = 1;

   /** Pointer to hardware */
   static constexpr volatile FTM_Type *tmr       = reinterpret_cast<volatile FTM_Type*>(Info::ftm);

   /** Pointer to clock register */
   static constexpr volatile uint32_t *clockReg  = reinterpret_cast<volatile uint32_t*>(Info::clockReg);

   /** Mask for BKGD output channel */
   static constexpr uint32_t BKGD_MASK           = (1<<bkgdInChannel);

   /** PCR for BKGD enable pin used by timer */
   using BkgdEn = USBDM::GpioTable_T<Info, bkgdEnChannel>;

   /** PCR for BKGD in pin used by timer */
   using BkgdOut = USBDM::GpioTable_T<Info, bkgdOutChannel>;

   using Bkgd = Lvc1t45<BkgdEn, BkgdOut>;

//   /** PCR for BKGD in pin used by timer */
//   using BkgdInPcr        = USBDM::PcrTable_T<Info, bkgdInChannel>;
//
//   /** PCR for BKGD enable pin used by timer */
//   using BkgdEnPcr        = USBDM::PcrTable_T<Info, bkgdEnChannel>;
//
//   /** PCR for BKGD in pin used by timer */
//   using BkgdOutPcr       = USBDM::PcrTable_T<Info, bkgdOutChannel>;
//
//   /** GPIO for BKGD in pin
//   using BkgdIn           = USBDM::GpioTable_T<Info, bkgdInChannel>;
//

   /** Measured Target SYNC width (Ticks) */
   static uint targetSyncWidth;

   /** Calculated time for target '1' bit (Ticks) */
   static uint oneBitTime;

   /** Calculated time for target '0' bit (Ticks) */
   static uint zeroBitTime;

   /** Calculated time for to sample target response (Ticks) */
   static uint sampleBitTime;

   /** Calculated time for target minimum bit period (Ticks) */
   static uint minPeriod;

   /** Calculated time for recovery after a '1' Tx (Ticks) */
   static uint oneRecoveryTime;

   /** Calculated time for recovery after a '0' Tx (Ticks) */
   static uint zeroRecoveryTime;

private:
   /**
    * Return the maximum of two values
    *
    *  @param a One value
    *  @param b Other vaue
    *
    *  @return Max value
    */
   static int max(int a, int b) {
      return (a>b)?a:b;
   }

public:
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
      long t = ((uint64_t)time*Info::getClockFrequency())/1000000;

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
      long t = ((uint64_t)time*1000000)/Info::getClockFrequency();

      assert((long)(uint32_t)t == t);
      assert(t != 0);

      return t;
   }

public:
   /**
    * Enable interface
    */
   static void initialise() {

      // Enable clock to timer
      *clockReg  |= Info::clockMask;

      // Extended features
      tmr->MODE     = FTM_MODE_INIT_MASK|FTM_MODE_FTMEN_MASK|FTM_MODE_WPDIS_MASK;

      // Debug mode
      tmr->CONF     = FTM_CONF_BDMMODE(2);

      // Clear s register changes have immediate effect
      tmr->SC = 0;

      // Common registers
      tmr->CNTIN    = 0;
      tmr->CNT      = 0;
      tmr->MOD      = (uint32_t)-1;

      tmr->SC       =
         FTM_SC_CPWMS(0)| // Left-Aligned
         FTM_SC_CLKS(1)|  // Clock source = system
         FTM_SC_TOIE(0)|  // Timer Overflow Interrupt disabled
         FTM_SC_PS(0);    // Prescale = /1

      tmr->OUTINIT =
            (0<<bkgdEnChannel)|  // Initialise low (disable buffer)
            (1<<bkgdOutChannel); // Initialise high

      Reset::initialise();

      disablePins();
      tmr->CONTROLS[bkgdEnChannel].CnSC  = USBDM::ftm_outputCompareClear;
      tmr->CONTROLS[bkgdOutChannel].CnSC = USBDM::ftm_outputCompareSet;

      // Switch pins to FTM
      Info::initPCRs(PORT_PCR_DSE_MASK|PORT_PCR_PE_MASK); // DS+PDN
   }

   static void disablePins() {
      tmr->SWOCTRL =
            (0<<(bkgdEnChannel+8)) |(1<<bkgdEnChannel)|  // Force low (disable buffer)
            (1<<(bkgdOutChannel+8))|(1<<bkgdOutChannel); // Force high
   }

   static void enablePins() {
      tmr->SWOCTRL = 0;
   }

   /**
    * Determine connection speed using sync pulse
    *
    * @param [out] syncLength Sync length in timer ticks
    *
    * @return Error code, BDM_RC_OK indicates success
    */
   static USBDM_ErrorCode sync(uint16_t &syncLength) {

      /* SYNC pulse width */
      const uint16_t syncPulseWidthInTicks = convertMicrosecondsToTicks(SYNC_WIDTH);

      static_assert((bkgdInChannel&1)==0, "BKGD Channel must be even");

      // Use Dual Edge on BKGD in
      tmr->COMBINE |= FTM_COMBINE_DECAPEN0_MASK<<(bkgdInChannel*4);

      // Enable dual capture on bkgdInChannel, bkgdInChannel+1
      tmr->CONTROLS[bkgdInChannel].CnSC    = USBDM::ftm_dualEdgeCaptureOneShotFallingEdge;
      tmr->CONTROLS[bkgdInChannel+1].CnSC  = USBDM::ftm_inputCaptureRisingEdge;

      // Set up to drive BKGD low (o-low, en=high)
      tmr->CONTROLS[bkgdOutChannel].CnSC   = USBDM::ftm_outputCompareClear;
      tmr->CONTROLS[bkgdEnChannel].CnSC    = USBDM::ftm_outputCompareSet;

      // Schedule event in the near future
      // This must allow enough time to complete configuration before events
      uint16_t eventTime = tmr->CNT+100;
      tmr->CONTROLS[bkgdOutChannel].CnV    = eventTime;
      tmr->CONTROLS[bkgdEnChannel].CnV     = eventTime;

      // Clear channel flags
      tmr->STATUS &= ~((1<<bkgdInChannel)|(1<<(bkgdInChannel+1))|(1<<bkgdOutChannel)|(1<<bkgdEnChannel));

      // Release force pin control
      enablePins();

      // Wait for event
      while ((tmr->CONTROLS[bkgdOutChannel].CnSC&FTM_CnSC_CHF_MASK) == 0) {
         __asm__("nop");
      }

      // Trigger dual-edge capture on BKGD_In
      tmr->COMBINE |= FTM_COMBINE_DECAP0_MASK<<(bkgdInChannel*4);

      // Drive BKGD high then 3-state
      tmr->CONTROLS[bkgdOutChannel].CnSC   = USBDM::ftm_outputCompareSet;
      tmr->CONTROLS[bkgdEnChannel].CnSC    = USBDM::ftm_outputCompareClear;
      eventTime += syncPulseWidthInTicks;
      tmr->CONTROLS[bkgdOutChannel].CnV    = eventTime;
      tmr->CONTROLS[bkgdEnChannel].CnV     = eventTime+SPEEDUP_PULSE_WIDTH_IN_TICKS;

      static auto pollBkgdIn2Timer = [] {
            return ((tmr->CONTROLS[bkgdInChannel+1].CnSC & FTM_CnSC_CHF_MASK) != 0);
      };
      // Wait for dual-edge & capture
      bool success = USBDM::waitUS(SYNC_TIMEOUT, pollBkgdIn2Timer);

      // Clear dual-edge (in case timeout)
      tmr->COMBINE &= ~(FTM_COMBINE_DECAP0_MASK<<(bkgdInChannel*4));

      uint16_t rv1 = tmr->CONTROLS[bkgdInChannel].CnV;
      uint16_t rv2 = tmr->CONTROLS[bkgdInChannel+1].CnV;

      disablePins();

      if (success) {
         syncLength = targetSyncWidth;
         return BDM_RC_OK;
      }
      return BDM_RC_SYNC_TIMEOUT;
   }

   static void setSyncLength(uint16_t syncLength) {
      constexpr int SYNC_RESPONSE_CYCLES = 128;    // Number of target clock cycles in the SYNC response
      constexpr int HOST_1_CYCLES_LOW    = 4;      // Number of target clock cycles low for a 1
      constexpr int HOST_0_CYCLES_LOW    = 13;     // Number of target clock cycles low for a 0
      constexpr int HOST_MIN_CYCLES      = 16;     // Minimum number of target clock cycles for entire transfer
      constexpr int HOST_SAMPLE_CYCLE    = 10;     // Number of target clock cycles for sample

      targetSyncWidth   = syncLength;

      // Calculate communication parameters
      zeroBitTime       = ((syncLength*HOST_1_CYCLES_LOW)+(SYNC_RESPONSE_CYCLES/2))/SYNC_RESPONSE_CYCLES;  // Ticks (round)
      oneBitTime        = ((syncLength*HOST_0_CYCLES_LOW)+(SYNC_RESPONSE_CYCLES-1))/SYNC_RESPONSE_CYCLES;  // Ticks (round up)
      sampleBitTime     = ((syncLength*HOST_SAMPLE_CYCLE)+(SYNC_RESPONSE_CYCLES/2))/SYNC_RESPONSE_CYCLES;  // Ticks (round)
      minPeriod         = ((syncLength*HOST_MIN_CYCLES)+(SYNC_RESPONSE_CYCLES-1))/SYNC_RESPONSE_CYCLES;    // Ticks (round up)

      zeroRecoveryTime  = max(TMR_SETUP_TIME, minPeriod-zeroBitTime);   // Ticks
      oneRecoveryTime   = max(TMR_SETUP_TIME, minPeriod-oneBitTime);    // Ticks
   }

   static USBDM_ErrorCode rxDualEdgePulse(uint8_t *p) {

      /** Time to set up Timer - This varies with optimisation! */
      static constexpr uint TMR_SETUP_TIME = 40;

      Reset::low();

      tmr->SC = 0;

      enablePins();
      tmr->COMBINE |=
            FTM_COMBINE_COMBINE0_MASK<<(bkgdEnChannel*4)|
            FTM_COMBINE_COMBINE0_MASK<<(bkgdOutChannel*4);

      // Positive pulse for buffer enable
      tmr->CONTROLS[bkgdEnChannel].CnSC    = FTM_CnSC_ELS(2);
      tmr->CONTROLS[bkgdEnChannel].CnV     = TMR_SETUP_TIME;
      tmr->CONTROLS[bkgdEnChannel+1].CnV   = TMR_SETUP_TIME+zeroBitTime-SPEEDUP_PULSE_WIDTH_IN_TICKS;

      // Negative pulse for BKGD out
      tmr->CONTROLS[bkgdOutChannel].CnSC   = FTM_CnSC_ELS(1);
      tmr->CONTROLS[bkgdOutChannel].CnV    = TMR_SETUP_TIME;
      tmr->CONTROLS[bkgdOutChannel+1].CnV  = TMR_SETUP_TIME+zeroBitTime;

      // Capture rising edge of BKGD in
      tmr->CONTROLS[bkgdInChannel].CnSC    = USBDM::ftm_inputCaptureRisingEdge;

      tmr->SC =
            FTM_SC_CPWMS(0)| // Left-Aligned
            FTM_SC_CLKS(1)|  // Clock source = system
            FTM_SC_TOIE(0)|  // Timer Overflow Interrupt disabled
            FTM_SC_PS(0);    // Prescale = /1

      bool success;
      for (int bitNum=0; bitNum<8; bitNum++) {
         tmr->CNT = 0;

         // Clear channel flags
         tmr->STATUS &= ~(
               (1<<bkgdEnChannel) |(1<<(bkgdEnChannel+1))|
               (1<<bkgdOutChannel)|(1<<(bkgdOutChannel+1))|
               (1<<bkgdInChannel));

         // Wait until end of bit
         do {
            __asm("nop");
         } while (tmr->CNT < 2*minPeriod);

         // Should have captured a rising edge from target (or float?)
         success = (tmr->CONTROLS[bkgdInChannel].CnSC & FTM_CnSC_CHF_MASK) != 0;

         volatile uint16_t eventTime = tmr->CONTROLS[bkgdInChannel].CnV;
         *p = (eventTime>sampleBitTime)?1:0;
      }
      disablePins();
      Reset::high();
      if (!success) {
         return BDM_RC_BKGD_TIMEOUT;
      }

      return BDM_RC_OK;
   }

   static USBDM_ErrorCode txDualEdgePulse(uint8_t data) {

      /** Time to set up Timer - This varies with optimisation! */
      static constexpr uint TMR_SETUP_TIME = 40;

      Reset::low();

      tmr->SC = 0;
      tmr->SYNCONF |= FTM_SYNCONF_SYNCMODE(1)|FTM_SYNCONF_SWWRBUF(1);

      enablePins();
      tmr->COMBINE |=
            FTM_COMBINE_SYNCEN0_MASK<<(bkgdEnChannel*4)|
            FTM_COMBINE_COMBINE0_MASK<<(bkgdEnChannel*4)|
            FTM_COMBINE_SYNCEN0_MASK<<(bkgdOutChannel*4)|
            FTM_COMBINE_COMBINE0_MASK<<(bkgdOutChannel*4);

      // Positive pulse for buffer enable
      tmr->CONTROLS[bkgdEnChannel].CnSC    = FTM_CnSC_ELS(2);
      tmr->CONTROLS[bkgdEnChannel].CnV     = TMR_SETUP_TIME;
      tmr->CONTROLS[bkgdEnChannel+1].CnV   = TMR_SETUP_TIME+zeroBitTime+SPEEDUP_PULSE_WIDTH_IN_TICKS;

      // Negative pulse for BKGD out
      tmr->CONTROLS[bkgdOutChannel].CnSC   = FTM_CnSC_ELS(1);
      tmr->CONTROLS[bkgdOutChannel].CnV    = TMR_SETUP_TIME;
      tmr->CONTROLS[bkgdOutChannel+1].CnV  = TMR_SETUP_TIME+zeroBitTime;

      tmr->SC =
            FTM_SC_CPWMS(0)| // Left-Aligned
            FTM_SC_CLKS(1)|  // Clock source = system
            FTM_SC_TOIE(0)|  // Timer Overflow Interrupt disabled
            FTM_SC_PS(0);    // Prescale = /1

      bool success;
      for (int bitNum=0; bitNum<8; bitNum++) {
         int width;
         if (data&1) {
            width = TMR_SETUP_TIME+oneBitTime;
         }
         else {
            width = TMR_SETUP_TIME+zeroBitTime;
         }
         data >>= 1;
         tmr->CNT = 0;
         tmr->CONTROLS[bkgdOutChannel+1].CnV  = width;
         tmr->CONTROLS[bkgdEnChannel+1].CnV   = width+SPEEDUP_PULSE_WIDTH_IN_TICKS;
         tmr->SYNC     = FTM_SYNC_SWSYNC(1);

         // Clear channel flags
         tmr->STATUS &= ~(
               (1<<bkgdEnChannel) |(1<<(bkgdEnChannel+1))|
               (1<<bkgdOutChannel)|(1<<(bkgdOutChannel+1)));

         // Wait until end of bit
         do {
            __asm("nop");
         } while (tmr->CNT < 2*minPeriod);

         __asm("nop");
      }
      disablePins();
      Reset::high();
      return BDM_RC_OK;
   }

   /**
    * Transmit a 8-bit value
    *
    * @param data Value to transmit
    */
   static void tx(uint32_t data) {

      enablePins();

      (void)tmr->CONTROLS[bkgdInChannel].CnSC;
      (void)tmr->CONTROLS[bkgdInChannel+1].CnSC;

      uint bitTime;
      uint recoveryTime;

      Reset::low();
      for (uint32_t bitMask=(1<<7); bitMask!=0; bitMask>>=1) {
         if (data&bitMask) {
            bitTime      = oneBitTime;
            recoveryTime = oneRecoveryTime;
         }
         else {
            bitTime      = zeroBitTime;
            recoveryTime = zeroRecoveryTime;
         }
         // Drive BKGD low for 0/1 bit time followed by high
         tmr->CONTROLS[bkgdEnChannel].CnSC    = USBDM::ftm_outputCompareClear;
         tmr->CONTROLS[bkgdOutChannel].CnSC   = USBDM::ftm_outputCompareSet;
         uint32_t eventTime = tmr->CNT+TMR_SETUP_TIME;
         tmr->CONTROLS[bkgdEnChannel].CnV     = eventTime;
         tmr->CONTROLS[bkgdOutChannel].CnV    = eventTime+bitTime;
         // Wait for event
         while ((tmr->CONTROLS[bkgdOutChannel].CnSC&FTM_CnSC_CHF_MASK) == 0) {
            __asm__("nop");
         }
         // Set up for next (BKGD highZ but ready to drive low)
         tmr->CONTROLS[bkgdEnChannel].CnSC    = USBDM::ftm_outputCompareSet;
         tmr->CONTROLS[bkgdOutChannel].CnSC   = USBDM::ftm_outputCompareClear;
         eventTime = tmr->CNT+recoveryTime;
         tmr->CONTROLS[bkgdEnChannel].CnV     = eventTime;
         tmr->CONTROLS[bkgdOutChannel].CnV    = eventTime+1;
         // Wait for event
         while ((tmr->CONTROLS[bkgdOutChannel].CnSC&FTM_CnSC_CHF_MASK) == 0) {
            __asm__("nop");
         }
      }
      Reset::high();

      disablePins();
   }
   /**
     * Transmit a 8-bit value
     *
     * @param data Value to transmit
     */
    static void rx(uint32_t data) {

       Bkgd::highZ();

       // Switch control to FTM
       Info::initPCRs(PORT_PCR_DSE_MASK|PORT_PCR_PE_MASK); // DS+PUD

       (void)tmr->CONTROLS[bkgdInChannel].CnSC;
       (void)tmr->CONTROLS[bkgdInChannel+1].CnSC;

       uint bitTime;
       uint recoveryTime;

       Reset::low();
       for (uint32_t bitMask=(1<<7); bitMask!=0; bitMask>>=1) {
          bitTime      = zeroBitTime;
          recoveryTime = zeroRecoveryTime;
          // Drive BKGD low for 0/1 bit time followed by high
          tmr->CONTROLS[bkgdEnChannel].CnSC    = USBDM::ftm_outputCompareClear;
          tmr->CONTROLS[bkgdOutChannel].CnSC   = USBDM::ftm_outputCompareSet;
          uint32_t eventTime = tmr->CNT+TMR_SETUP_TIME;
          tmr->CONTROLS[bkgdEnChannel].CnV     = eventTime;
          tmr->CONTROLS[bkgdOutChannel].CnV    = eventTime+bitTime;
          // Wait for event
          while ((tmr->CONTROLS[bkgdOutChannel].CnSC&FTM_CnSC_CHF_MASK) == 0) {
             __asm__("nop");
          }
          // Set up for next (BKGD highZ but ready to drive low)
          tmr->CONTROLS[bkgdEnChannel].CnSC    = USBDM::ftm_outputCompareSet;
          tmr->CONTROLS[bkgdOutChannel].CnSC   = USBDM::ftm_outputCompareClear;
          eventTime = tmr->CNT+recoveryTime;
          tmr->CONTROLS[bkgdEnChannel].CnV     = eventTime;
          tmr->CONTROLS[bkgdOutChannel].CnV    = eventTime+1;
          // Wait for event
          while ((tmr->CONTROLS[bkgdOutChannel].CnSC&FTM_CnSC_CHF_MASK) == 0) {
             __asm__("nop");
          }
       }
       Reset::high();
    }

   /**
    *
    * @param lowTime Length of low pulse that initiates a bit reception
    *
    * @return ??
    */
   static uint16_t rx2(uint16_t lowTime) {

      Bkgd::highZ();

      // Switch control to FTM
      Info::initPCRs(PORT_PCR_DSE_MASK|PORT_PCR_PE_MASK); // DS+PUD

      // Enable dual capture on bkgdInChannel, bkgdInChannel+1
      (void)tmr->CONTROLS[bkgdInChannel].CnSC;
      (void)tmr->CONTROLS[bkgdInChannel+1].CnSC;
      tmr->CONTROLS[bkgdInChannel].CnSC    = USBDM::ftm_dualEdgeCaptureOneShotRisingEdge;
      tmr->CONTROLS[bkgdInChannel+1].CnSC  = USBDM::ftm_inputCaptureFallingEdge;

      // Drive BKGD low
      tmr->CONTROLS[bkgdOutChannel].CnSC   = USBDM::ftm_outputCompareClear;
      tmr->CONTROLS[bkgdEnChannel].CnSC    = USBDM::ftm_outputCompareClear;
      uint16_t now = tmr->CNT;
      tmr->CONTROLS[bkgdOutChannel].CnV    = now+20;
      tmr->CONTROLS[bkgdEnChannel].CnV     = now+20;
      // Wait for event
      while ((tmr->CONTROLS[bkgdOutChannel].CnSC&FTM_CnSC_CHF_MASK) == 0) {
         __asm__("nop");
      }
      // Trigger dual-edge capture on BKGD_In
      tmr->COMBINE |= FTM_COMBINE_DECAP0_MASK<<(bkgdInChannel*4);

      // Drive BKGD high then 3-state
      tmr->CONTROLS[bkgdOutChannel].CnSC   = USBDM::ftm_outputCompareSet;
      tmr->CONTROLS[bkgdEnChannel].CnSC    = USBDM::ftm_outputCompareSet;
      now = tmr->CNT;
      tmr->CONTROLS[bkgdOutChannel].CnV    = now+lowTime;
      tmr->CONTROLS[bkgdEnChannel].CnV     = now+lowTime+SPEEDUP_PULSE_WIDTH_IN_TICKS;

      // Wait for dual-edge capture
      bool success = USBDM::waitUS(BIT_TIME_OUT, pollBkgdIn2Timer);

      uint16_t rv1 = tmr->CONTROLS[bkgdInChannel].CnV;
      uint16_t rv2 = tmr->CONTROLS[bkgdInChannel+1].CnV;

      if (!success) {
         return 0;
      }
      else {
         return (uint16_t)(rv2-rv1);
      }
   }

   static uint16_t tryIt() {
      return  rx(50);
   }

   /**
    * Pulse BKGD signal continuously LOW-HIGH-LOW-3STATE
    */
   static void testBkgd() {
      for(;;) {
         Bkgd::low();
         USBDM::waitUS(10);
         Bkgd::high();
         USBDM::waitUS(1);
         Bkgd::low();
         USBDM::waitUS(1);
         Bkgd::highZ();
         USBDM::waitUS(1);
      }
   }

private:
   /* Polls the BKGD in timer channel
    *
    * @return true Channel event has occurred
    */
   static bool pollTimer() {
      return ((tmr->CONTROLS[bkgdInChannel].CnSC & FTM_CnSC_CHF_MASK) != 0);
   }
   /* Polls the BKGD in 2nd timer channel (dual-edge mode)
    *
    * @return true Channel event has occurred
    */
   static bool pollBkgdIn2Timer() {
      return ((tmr->CONTROLS[bkgdInChannel+1].CnSC & FTM_CnSC_CHF_MASK) != 0);
   }
//   /* Polls the BKGD input low
//    *
//    * @return BKGD pin value
//    */
//   static bool pollBkgdInLow() {
//      return (!BkgdIn::read());
//   }
};

/** Measured Target SYNC width (Ticks) */
template <class info, uint8_t bkgdOutChannel, uint8_t bkgdEnChannel, uint8_t bkgdInChannel>
uint BackgroundDebug_T<info, bkgdOutChannel,bkgdEnChannel, bkgdInChannel>::targetSyncWidth;

/** Calculated time for target '1' bit (Ticks) */
template <class info, uint8_t bkgdOutChannel, uint8_t bkgdEnChannel, uint8_t bkgdInChannel>
uint BackgroundDebug_T<info, bkgdOutChannel,bkgdEnChannel, bkgdInChannel>::oneBitTime;

/** Calculated time for target '0' bit (Ticks) */
template <class info, uint8_t bkgdOutChannel, uint8_t bkgdEnChannel, uint8_t bkgdInChannel>
uint BackgroundDebug_T<info, bkgdOutChannel,bkgdEnChannel, bkgdInChannel>::zeroBitTime;

/** Calculated time for target '0' bit (Ticks) */
template <class info, uint8_t bkgdOutChannel, uint8_t bkgdEnChannel, uint8_t bkgdInChannel>
uint BackgroundDebug_T<info, bkgdOutChannel,bkgdEnChannel, bkgdInChannel>::sampleBitTime;

/** Calculated time for target minimum bit period (Ticks) */
template <class info, uint8_t bkgdOutChannel, uint8_t bkgdEnChannel, uint8_t bkgdInChannel>
uint BackgroundDebug_T<info, bkgdOutChannel,bkgdEnChannel, bkgdInChannel>::minPeriod;

/** Calculated time for recovery after a '1' Tx (Ticks) */
template <class info, uint8_t bkgdOutChannel, uint8_t bkgdEnChannel, uint8_t bkgdInChannel>
uint BackgroundDebug_T<info, bkgdOutChannel,bkgdEnChannel, bkgdInChannel>::oneRecoveryTime;

/** Calculated time for recovery after a '0' Tx (Ticks) */
template <class info, uint8_t bkgdOutChannel, uint8_t bkgdEnChannel, uint8_t bkgdInChannel>
uint BackgroundDebug_T<info, bkgdOutChannel,bkgdEnChannel, bkgdInChannel>::zeroRecoveryTime;

/**
 * bkgdOutChannel  D6(ch6)
 * bkgdEnChannel   C3(ch2)
 * bkgdInChannel   D4(ch4)
 */
using Bdm = BackgroundDebug_T<USBDM::Ftm0Info, 6, 2, 4>;

#endif /* SOURCES_BDM_H_ */
