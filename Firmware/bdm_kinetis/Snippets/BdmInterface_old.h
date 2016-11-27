/*
 * BdmInterface.h
 *
 *  Created on: 12 Feb 2016
 *      Author: podonoghue
 */

#ifndef PROJECT_HEADERS_BDMINTERFACE_H_
#define PROJECT_HEADERS_BDMINTERFACE_H_
#include "system.h"
#include "derivative.h"
#include "pin_mapping.h"
#include "delay.h"

/**
 * Represents a bidirectional signal e.g. 74LVC1T45 buffer
 * Assumes 2 signals: output/input, direction (0=>in, 1=>out)
 *
 * @tparam inOut  GPIO connected to receiver input
 * @tparam dir    GPIO connected to direction control
 */
template<class inOut, class dir>
class Lvc1t45 {
private:

public:
   /**
    * Initialise Transceiver
    *
    * Initial state:
    *    Input
    */
   static void initialise() {
      // Set pin as input
      inOut::setInput();

      // Direction low => input
      dir::low();
      dir::setOutput();
   }
   /**
    * Drive signal high\n
    * Assumes series resistor for any I/O drive overlap
    */
   static void high() {
      inOut::high();
      inOut::setOutput();
      dir::high();
   }
   /**
    * Drive signal high
    *
    * @note Assumes driver already enabled
    */
   static void _high() {
      inOut::high();
   }
   /**
    * Drive signal low
    */
   static void low() {
      inOut::low();
      inOut::setOutput();
      dir::high();
   }
   /**
    * Drive signal low
    *
    * @note Assumes driver already enabled
    */
   static void _low() {
      inOut::low();
   }
   /**
    * Disable Transceiver (high-impedance)\n
    * Actually sets to input
    */
   static void highZ() {
      (void)read();
   }
   /**
    * Read value from receiver
    *
    * @return value on pin
    */
   static bool read() {
      inOut::setInput();
      dir::low();
      return inOut::read();
   }
   /**
    * Read value from receiver
    *
    * @return value on pin
    *
    * @note Assumes already set as input
    */
   static bool _read() {
      return inOut::read();
   }
};

/**
 * Represents a bidirectional signal with 3-state control\n
 * e.g. 2 x 74LV125 buffers
 * Assumes 3 signals: output, input, 3-state enable
 *
 * @tparam out    GPIO connected to driver input
 * @tparam in     GPIO connected to receiver output
 * @tparam enable GPIO connected to driver 3-state control
 */
template<class out, class enable, class in>
class Transceiver {
public:
   /**
    * Initialise Transceiver
    *
    * Initial state:
    *    High-Z
    */
   static void initialise() {
      // Enable high => disable driver
      enable::high();
      enable::setOutput();
      // Output high (but has no effect)
      out::high();
      out::setOutput();
      // Set as input
      in::setInput();
   }
   /**
    * Drive signal high
    */
   static void high() {
      out::high();
      enable::low();
   }
   /**
    * Drive signal high
    *
    * @note Assumes driver already enabled
    */
   static void _high() {
      out::high();
   }
   /**
    * Drive signal low
    */
   static void low() {
      out::low();
      enable::low();
   }
   /**
    * Drive signal low
    *
    * @note Assumes driver already enabled
    */
   static void _low() {
      out::low();
   }
   /**
    * Disable Transceiver (high-impedance)
    */
   static void highZ() {
      enable::high();
   }
   /**
    * Read value from receiver
    *
    * @return value on pin
    */
   static bool read() {
      enable::high();
      return in::read();
   }
   /**
    * Read value from receiver
    *
    * @return value on pin
    *
    * @note Assumes already set as input
    */
   static bool _read() {
      return in::read();
   }
};

/**
 * Represents a signal with 3-state control\n
 * e.g. 74LV125 buffer, 74LVC1T45
 * Assumes 2 signals: output to Transceiver, enable to Transceiver
 *
 * @tparam out       GPIO connected to driver input
 * @tparam enable    GPIO connected to driver 3-state control (or DIR)
 * @tparam oeActive  Polarity of OE signal (true=>high to enable)
 */
template<class out, class enable, bool oeActive=true>
class Driver {
public:
   /**
    * Initialise Driver
    */
   static void initialise() {
      // Enable high => disable driver
      enable::write(!oeActive);
      enable::setOutput();
      // Output high (but has no effect)
      out::high();
      out::setOutput();
   }
   /**
    * Drive signal high
    */
   static void high() {
      out::high();
      enable::write(oeActive);
   }
   /**
    * Drive signal high
    *
    * @note Assumes driver already enabled
    */
   static void _high() {
      out::high();
   }
   /**
    * Drive signal low
    */
   static void low() {
      out::low();
      enable::write(oeActive);
   }
   /**
    * Drive signal low
    *
    * @note Assumes driver already enabled
    */
   static void _low() {
      enable::low();
   }
   /**
    * Disable Transceiver (high-impedance)
    */
   static void highZ() {
      enable::write(!oeActive);
   }
};
/**
 * Represents an Open-collector signal\n
 * e.g. 74LV125 buffer
 * Assumes 1 signal: enable to Transceiver (and output to Transceiver)
 *
 * @tparam out    GPIO connected to driver input
 * @tparam enable GPIO connected to driver 3-state control
 */
template<class out>
class DriverOC {
public:
   /**
    * Initialise Driver
    */
   static void initialise() {
      // Output high (actually 3-state)
      out::high();
      out::setOutput();
   }
   /**
    * Drive signal high (actually 3-state)
    */
   static void high() {
      out::high();
   }
   /**
    * Drive signal high (actually 3-state)
    */
   static void _high() {
      out::high();
   }
   /**
    * Drive signal low
    */
   static void low() {
      out::low();
   }
   /**
    * Drive signal low
    */
   static void _low() {
      out::low();
   }
   /**
    * Disable Transceiver (high-impedance)
    */
   static void highZ() {
      out::high();
   }
};

//using Reset = Driver<USBDM::GpioC<0>, USBDM::GpioC<1>, 1>;
using Bkgd  = Transceiver<USBDM::GpioA<5>, USBDM::GpioD<4>, USBDM::GpioA<1>>;

/**
 * Class representing a low-level interface to a RS08/HCS08/HCS12 debug module
 *
 * @tparam Info               Information class for FTM
 * @tparam bkgdOutChannel     Channel to use for BKGD output to driver
 * @tparam bkgdEnChannel      Channel to use for BKGD output to driver enable
 * @tparam bkgdInChannel      Channel to use for BKGD input from receiver
 */
template <class Info, uint8_t bkgdOutChannel, uint8_t bkgdEnChannel, uint8_t bkgdInChannel>
class BackgroundDebug_T : public USBDM::CheckSignal<Info, bkgdOutChannel>, USBDM::CheckSignal<Info, bkgdEnChannel>, USBDM::CheckSignal<Info, bkgdInChannel> {

protected:
   /** How long to wait for target response per bit (us) */
   static constexpr uint BIT_TIME_OUT            = 100;

   /** Length of high drive before 3-state i.e. speed-up pulse (ticks) */
   static constexpr uint SPEEDUP_PULSE_TIME      = 1;

   /** Width of sync pulse (us), 128 target-cycle @ 128kHz (<65536/FTMClock) */
   static constexpr uint SYNC_WIDTH              = 1000;

   /** Time to wait for end of target sync pulse response (us), (<65536/FTMClock) */
   static constexpr uint SYNC_TIMEOUT            = 1200;

   /** Time to set up Timer - This varies with optimisation! */
   static constexpr uint TMR_SETUP_TIME          = 15;

   /** Pointer to hardware */
   static constexpr volatile FTM_Type *tmr       = reinterpret_cast<volatile FTM_Type*>(Info::basePtr);

   /** Pointer to clock register */
   static constexpr volatile uint32_t *clockReg  = reinterpret_cast<volatile uint32_t*>(Info::clockReg);

   /** Mask for BKGD output channel */
   static constexpr uint32_t BKGD_MASK           = (1<<bkgdInChannel);

   /** PCR for BKGD in pin used by timer */
   using BkgdInPcr        = USBDM::PcrTable_T<Info, bkgdInChannel>;

   /** PCR for BKGD enable pin used by timer */
   using BkgdEnPcr        = USBDM::PcrTable_T<Info, bkgdEnChannel>;

   /** PCR for BKGD in pin used by timer */
   using BkgdOutPcr       = USBDM::PcrTable_T<Info, bkgdOutChannel>;

//   /** GPIO for BKGD in pin
//   using BkgdIn           = USBDM::GpioTable_T<Info, bkgdInChannel>;
//

   /** Measured Target SYNC width (Ticks) */
   static uint targetSyncWidth;

   /** Calculated time for target '1' bit (Ticks) */
   static uint oneBitTime;

   /** Calculated time for target '0' bit (Ticks) */
   static uint zeroBitTime;

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
    * Calculates the tick rate of the timer in Ticks per second (Hz)
    *
    * @return Tick rate in Hz
    */
   static uint32_t getTickRate() {
      return SystemBusClock/(1<<(tmr->SC&FTM_SC_PS_MASK));
   }
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
      // Calculate period
      return ((uint64_t)time*getTickRate())/1000000;
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
      // Calculate period
      return ((uint64_t)time*1000000)/getTickRate();
   }

public:
   /**
    * Enable interface
    */
   static void enable() {

      // Enable clock to timer
      *clockReg  |= Info::clockMask;

      tmr->MODE     = FTM_MODE_INIT_MASK|FTM_MODE_FTMEN_MASK|FTM_MODE_WPDIS_MASK;
      tmr->CONF     = FTM_CONF_BDMMODE(2);

      // Common registers
      tmr->CNTIN    = 0;
      tmr->CNT      = 0;
      tmr->MOD      = (uint32_t)-1;

      // Left aligned PWM without CPWMS selected
      tmr->SC       = Info::SC;

      // Use Dual Edge on BKGD in
      tmr->COMBINE |= FTM_COMBINE_DECAPEN0_MASK<<(bkgdInChannel*4);

      Reset::initialise();
      Bkgd::initialise();
   }

   /**
    * Determine connection speed using sync pulse
    *
    * @param [out] syncLength Sync length in timer ticks
    *
    * @return Indicates success or failure.
    */
   static bool sync(uint16_t &syncLength) {

      const uint16_t syncPulseWidthInTicks = convertMicrosecondsToTicks(SYNC_WIDTH);
      const uint16_t syncTimeoutInTicks    = convertMicrosecondsToTicks(SYNC_TIMEOUT);

      Bkgd::highZ();

      // Switch control to FTM
      BkgdEnPcr::setPCR();
      BkgdOutPcr::setPCR();
      BkgdInPcr::setPCR();

      // Clear any flags
      (void)tmr->CONTROLS[bkgdInChannel].CnSC;
      (void)tmr->CONTROLS[bkgdInChannel+1].CnSC;

      // Enable dual capture on bkgdInChannel, bkgdInChannel+1
      tmr->CONTROLS[bkgdInChannel].CnSC    = USBDM::ftm_dualEdgeCaptureOneShotFallingEdge;
      tmr->CONTROLS[bkgdInChannel+1].CnSC  = USBDM::ftm_inputCaptureRisingEdge;

      // Drive BKGD low
      tmr->CONTROLS[bkgdOutChannel].CnSC   = USBDM::ftm_outputCompareClear;
      tmr->CONTROLS[bkgdEnChannel].CnSC    = USBDM::ftm_outputCompareClear;
      uint16_t eventTime = tmr->CNT+20;
      tmr->CONTROLS[bkgdOutChannel].CnV    = eventTime;
      tmr->CONTROLS[bkgdEnChannel].CnV     = eventTime;
      // Wait for event
      while ((tmr->CONTROLS[bkgdOutChannel].CnSC&FTM_CnSC_CHF_MASK) == 0) {
         __asm__("nop");
      }
      // Trigger dual-edge capture on BKGD_In
      tmr->COMBINE |= FTM_COMBINE_DECAP0_MASK<<(bkgdInChannel*4);

      // Drive BKGD high then 3-state
      tmr->CONTROLS[bkgdOutChannel].CnSC   = USBDM::ftm_outputCompareSet;
      tmr->CONTROLS[bkgdEnChannel].CnSC    = USBDM::ftm_outputCompareSet;
      eventTime += syncPulseWidthInTicks;
      tmr->CONTROLS[bkgdOutChannel].CnV    = eventTime;
      tmr->CONTROLS[bkgdEnChannel].CnV     = eventTime+SPEEDUP_PULSE_TIME;

      // Wait for dual-edge & capture
      bool success = USBDM::waitUS(syncTimeoutInTicks, pollBkgdIn2Timer);

      // Clear dual-edge (in case timeout)
      tmr->COMBINE &= ~(FTM_COMBINE_DECAP0_MASK<<(bkgdInChannel*4));

      uint16_t rv1 = tmr->CONTROLS[bkgdInChannel].CnV;
      uint16_t rv2 = tmr->CONTROLS[bkgdInChannel+1].CnV;

      if (success) {
         targetSyncWidth   = (uint16_t)(rv2-rv1);

         zeroBitTime       = ((targetSyncWidth*4)+64)/128;     // Ticks (round)
         oneBitTime        = ((targetSyncWidth*13)+127)/128;   // Ticks (round up)
         minPeriod         = ((targetSyncWidth*16)+127)/128;   // Ticks (round up)
         zeroRecoveryTime  = max(TMR_SETUP_TIME, minPeriod-zeroBitTime);   // Ticks
         oneRecoveryTime   = max(TMR_SETUP_TIME, minPeriod-oneBitTime);    // Ticks

         syncLength       = targetSyncWidth;
         return true;
      }
      else {
         return false;
      }

   }
   /**
    * Transmit a 8-bit value
    *
    * @param data Value to transmit
    */
   static void tx(uint32_t data) {

      // Switch control to FTM
      BkgdEnPcr::setPCR();
      BkgdOutPcr::setPCR();
      BkgdInPcr::setPCR();

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
   }
   /**
     * Transmit a 8-bit value
     *
     * @param data Value to transmit
     */
    static void rx(uint32_t data) {

       Bkgd::highZ();

       // Switch control to FTM
       BkgdEnPcr::setPCR();
       BkgdOutPcr::setPCR();
       BkgdInPcr::setPCR();

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
      BkgdEnPcr::setPCR();
      BkgdOutPcr::setPCR();
      BkgdInPcr::setPCR();

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
      tmr->CONTROLS[bkgdEnChannel].CnV     = now+lowTime+SPEEDUP_PULSE_TIME;

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

   /**
    * Pulse RESET signal continuously LOW-3STATE
    */
   static void testReset() {
      for(;;) {
         Reset::low();
         USBDM::waitUS(1000);
         Reset::highZ();
         USBDM::waitUS(5000);
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

/** Calculated time for target minimum bit period (Ticks) */
template <class info, uint8_t bkgdOutChannel, uint8_t bkgdEnChannel, uint8_t bkgdInChannel>
uint BackgroundDebug_T<info, bkgdOutChannel,bkgdEnChannel, bkgdInChannel>::minPeriod;

/** Calculated time for recovery after a '1' Tx (Ticks) */
template <class info, uint8_t bkgdOutChannel, uint8_t bkgdEnChannel, uint8_t bkgdInChannel>
uint BackgroundDebug_T<info, bkgdOutChannel,bkgdEnChannel, bkgdInChannel>::oneRecoveryTime;

/** Calculated time for recovery after a '0' Tx (Ticks) */
template <class info, uint8_t bkgdOutChannel, uint8_t bkgdEnChannel, uint8_t bkgdInChannel>
uint BackgroundDebug_T<info, bkgdOutChannel,bkgdEnChannel, bkgdInChannel>::zeroRecoveryTime;

using BackgroundDebugInterface = BackgroundDebug_T<USBDM::Ftm0Info, 6, 2, 4>;

/**
 * Determine connection speed using sync pulse
 *
 * @param [out] syncLength Sync length in timer ticks
 *
 * @return Indicates success or failure.
 */
//   static void txByte(uint8_t byte) {
//      uint16_t data[] = {convertMicrosecondsToTicks(10), convertMicrosecondsToTicks(15), convertMicrosecondsToTicks(20), convertMicrosecondsToTicks(30)};
//      USBDM::DMAChannel::SingleTransferInfo info = {data, sizeof(data), &FTM0->CONTROLS[0].CnV, 0};
//      using DMAChannel0 = USBDM::DMAChannel_T<USBDM::DmaInfo, 0, USBDM::DMA0_SLOT_FTM0_Ch_0+bkgdOutChannel>;
//
//      DMAChannel0::configure(&info);
//      BkgdOutPcr::setPCR();
//
//      // Schedule falling edge event Now+20 ticks
//      tmr->CONTROLS[bkgdOutChannel].CnSC = USBDM::ftm_outputCompareToggle|FTM_CnSC_DMA_MASK;
//      tmr->CONTROLS[bkgdOutChannel].CnV  = tmr->CNT+20;
//
//      DMAChannel0::waitUntilComplete();
//      __asm__("nop");

//      // Enable BKGD pin so it can be driven low
//      BkgdEnableGpio::low();
//
//      // 1st read to enable clear status
//      (void)tmr->CONTROLS[bkgdChannel].CnSC;
//
//      // Schedule falling edge event Now+20 ticks
//      tmr->CONTROLS[bkgdChannel].CnSC = ftm_outputCompareClear;
//      tmr->CONTROLS[bkgdChannel].CnV  = tmr->CNT+20;
//      // Wait for event
//      while ((tmr->CONTROLS[bkgdChannel].CnSC & FTM_CnSC_CHF_MASK) == 0) {
//      }
//      BkgdEnableGpio::low();
//   }

//static void tryIt2() {
//   constexpr uint base  = 100; //
//   constexpr uint width = 18;  // 24=500ns, 18=375ns, 15 fails
//   const struct {
//      uint32_t CnSC;
//      uint32_t CnV;
//   } data[] = {
//      {FTM_CnSC_CHIE_MASK|FTM_CnSC_DMA_MASK|USBDM::ftm_outputCompareClear, 10, },
//      {FTM_CnSC_CHIE_MASK|FTM_CnSC_DMA_MASK|USBDM::ftm_outputCompareClear, (base), },
//      {FTM_CnSC_CHIE_MASK|FTM_CnSC_DMA_MASK|USBDM::ftm_outputCompareSet,   (base+width), },
//      {FTM_CnSC_CHIE_MASK|FTM_CnSC_DMA_MASK|USBDM::ftm_outputCompareClear, (base+width+width), },
//      {FTM_CnSC_CHIE_MASK|FTM_CnSC_DMA_MASK|USBDM::ftm_outputCompareSet,   (base+width+width+2*width),},
//      {0,                                                                  0, },
//   };
//   using DmaChannel = USBDM::DmaChannel_T<USBDM::DmaInfo, 0, USBDM::DMA0_SLOT_FTM0_Ch_0+bkgdOutChannel>;
//
//   Bkgd::high();
//   BkgdOutPcr::setPCR();
//
//   static const USBDM::DmaChannel::MultipleTransferInfo info = {
//      /* sourceAddress        */ data,
//      /* destinationAddress   */ &tmr->CONTROLS[bkgdOutChannel],
//      /* nBytes               */ DMA_NBYTES_MLOFFYES_DMLOE_MASK|
//                                 DMA_NBYTES_MLOFFYES_MLOFF(-sizeof(data[0]))|
//                                 DMA_NBYTES_MLOFFYES_NBYTES(sizeof(data[0])),
//      /* attributes           */ DMA_ATTR_SMOD(0)|
//                                 DMA_ATTR_SSIZE(DmaChannel::getAttrSize(4))|
//                                 DMA_ATTR_DMOD(0)|
//                                 DMA_ATTR_DSIZE(DmaChannel::getAttrSize(4)),
//      /* sourceOffset         */ 4,
//      /* destinationOffset    */ 4,
//      /* numberOfTransactions */ sizeof(data)/sizeof(data[0]),
//   };
////      tmr->CONTROLS[bkgdOutChannel].CnSC = FTM_CnSC_CHIE_MASK|FTM_CnSC_DMA_MASK|USBDM::ftm_outputCompare;
//   DmaChannel::configure(&info);
//   Reset::low();
//   tmr->CNT = 0;
//   DmaChannel::enableRequests();
//   DmaChannel::waitUntilComplete();
//   DmaChannel::disableRequests();
//   Reset::high();
//   __asm__("nop");
//}

#endif /* PROJECT_HEADERS_BDMINTERFACE_H_ */
