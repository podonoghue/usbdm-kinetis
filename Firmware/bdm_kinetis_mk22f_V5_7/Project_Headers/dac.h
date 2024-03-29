/**
 * @file     dac.h (180.ARM_Peripherals/Project_Headers/dac.h)
 * @brief    Abstraction layer for DAC interface
 *
 * @version  V4.12.1.240
 * @date     28/10/2018
 */

#ifndef HEADERS_DAC_H_
#define HEADERS_DAC_H_
 /*
 * *****************************
 * *** DO NOT EDIT THIS FILE ***
 * *****************************
 *
 * This file is generated automatically.
 * Any manual changes will be lost.
 */
#include "derivative.h"
#include "pin_mapping.h"

namespace USBDM {

/**
 * @addtogroup DAC_Group DAC, Digital-to-Analogue Converter
 * @brief Pins used for Digital-to-Analogue Converter
 * @{
 */

/**
 * Template class representing a Digital to Analogue Converter
 *
 * @tparam info      Information class for DAC
 *
 * @code
 * using dac = Dac_T<Dac0Info>;
 *
 *  dac::configure();
 *
 * @endcode
 */
template<class Info>
class DacBase_T : public Info {

protected:

   /** Class to static check output is mapped to a pin - Assumes existence */
   template<int dacOutput> class CheckOutputIsMapped {

      // Check mapping - no need to check existence
      static constexpr bool Test1 = (Info::info[dacOutput].gpioBit >= 0);

      static_assert(Test1, "DAC output is not mapped to a pin - Modify Configure.usbdm");

   public:
      /** Dummy function to allow convenient in-line checking */
      static constexpr void check() {}
   };

public:
   /**
    * Type definition for DAC interrupt call back
    */
   typedef typename Info::CallbackFunction CallbackFunction;

protected:
   /**
    * Callback to catch unhandled interrupt
    */
   static void unhandledCallback(uint8_t) {
      setAndCheckErrorCode(E_NO_HANDLER);
   }

   /** Callback function for ISR */
   static CallbackFunction sCallback;

public:
   /**
    * Hardware instance pointer
    *
    * @return Reference to CMT hardware
    */
   static constexpr HardwarePtr<DAC_Type> dac = Info::baseAddress;

   /** Get base address of DAC hardware as uint32_t */
   static constexpr uint32_t dacBase() { return Info::baseAddress; }

   /** Get base address of DAC.DATA register as uint32_t */
   static constexpr uint32_t dacData() { return dacBase() + offsetof(DAC_Type, DATA[0]); }

   /** Get base address of DAC.DATA[index] register as uint32_t */
   static constexpr uint32_t dacData(unsigned index) { return dacBase() + offsetof(DAC_Type, DATA) + index*sizeof(DAC_Type::DATA[0]); }

   /**
    * IRQ handler
    */
   static void irqHandler() {
      // Call handler
      sCallback(getAndClearStatus());
   }

// Template _mapPinsOption.xml

   /**
    * Configures all mapped pins associated with DAC
    *
    * @note Locked pins will be unaffected
    */
   static void configureAllPins() {
   
      // Configure pins if selected and not already locked
      if constexpr (Info::mapPinsOnEnable) {
         Info::initPCRs();
      }
   }

   /**
    * Disabled all mapped pins associated with DAC
    *
    * @note Only the lower 16-bits of the PCR registers are modified
    *
    * @note Locked pins will be unaffected
    */
   static void disableAllPins() {
   
      // Disable pins if selected and not already locked
      if constexpr (Info::mapPinsOnEnable) {
         Info::clearPCRs();
      }
   }

   /**
    * Basic enable of DAC
    * Includes enabling clock and configuring all mapped pins if mapPinsOnEnable is selected in configuration
    */
   static void enable() {
      Info::enableClock();
      configureAllPins();
   }

   /**
    * Disables the clock to DAC and all mapped pins
    */
   static void disable() {
      disableNvicInterrupts();
      dac->C0 = DAC_C0_DACEN(0);
      disableAllPins();
      Info::disableClock();
   }
// End Template _mapPinsOption.xml

   /**
    * Wrapper to allow the use of a class member as a callback function
    * @note Only usable with static objects.
    *
    * @tparam T         Type of the object containing the callback member function
    * @tparam callback  Member function pointer
    * @tparam object    Object containing the member function
    *
    * @return  Pointer to a function suitable for the use as a callback
    *
    * @code
    * class AClass {
    * public:
    *    int y;
    *
    *    // Member function used as callback
    *    // This function must match CallbackFunction
    *    void callback() {
    *       ...;
    *    }
    * };
    * ...
    * // Instance of class containing callback member function
    * static AClass aClass;
    * ...
    * // Wrap member function
    * auto fn = Dac::wrapCallback<AClass, &AClass::callback, aClass>();
    * // Use as callback
    * Dac::setCallback(fn);
    * @endcode
    */
   template<class T, void(T::*callback)(uint8_t), T &object>
   static CallbackFunction wrapCallback() {
      static CallbackFunction fn = [](uint8_t status) {
         (object.*callback)(status);
      };
      return fn;
   }

   /**
    * Wrapper to allow the use of a class member as a callback function
    * @note There is a considerable space and time overhead to using this method
    *
    * @tparam T         Type of the object containing the callback member function
    * @tparam callback  Member function pointer
    * @tparam object    Object containing the member function
    *
    * @return  Pointer to a function suitable for the use as a callback
    *
    * @code
    * class AClass {
    * public:
    *    int y;
    *
    *    // Member function used as callback
    *    // This function must match CallbackFunction
    *    void callback() {
    *       ...;
    *    }
    * };
    * ...
    * // Instance of class containing callback member function
    * AClass aClass;
    * ...
    * // Wrap member function
    * auto fn = Pit::wrapCallback<AClass, &AClass::callback>(aClass);
    * // Use as callback
    * Dac::setCallback(fn);
    * @endcode
    */
   template<class T, void(T::*callback)(uint8_t)>
   static CallbackFunction wrapCallback(T &object) {
      static T &obj = object;
      static CallbackFunction fn = [](uint8_t status) {
         (obj.*callback)(status);
      };
      return fn;
   }

   /**
    * Set callback function
    *
    * @param[in] callback Callback function to execute on interrupt.\n
    *                     Use nullptr to remove callback.
    */
   static void setCallback(CallbackFunction callback) {
      static_assert(Info::irqHandlerInstalled, "DAC not configured for interrupts");
      if (callback == nullptr) {
         callback = unhandledCallback;
      }
      sCallback = callback;
   }

   /**
    * Set DAC Buffer Watermark Interrupt Enable
    *
    * @param dacWatermarkIrq     Control whether an interrupt is generated when SR.DACBFWMF is set i.e.
    *        when the DAC buffer read pointer has reached the watermark level.
    * @param dacReadPtrTopIrq    Control whether an interrupt is generated when SR.DACBFRPTF is set i.e.
    *        when the DAC buffer read pointer is zero.
    * @param dacReadPtrBottomIrq Control whether an interrupt is generated when SR.DACBFRPBF is set i.e. 
    *        when the DAC buffer read pointer is equal to buffer upper limit (C2.DACBFUP)
    */
   void setActions(
         DacWatermarkIrq     dacWatermarkIrq,
         DacReadPtrTopIrq    dacReadPtrTopIrq    = DacReadPtrTopIrq_Disabled,
         DacReadPtrBottomIrq dacReadPtrBottomIrq = DacReadPtrBottomIrq_Disabled) {
   
      dac->C0 = (dac->C0&~(DAC_C0_DACBWIEN_MASK|DAC_C0_DACBTIEN_MASK|DAC_C0_DACBBIEN_MASK)) |
                  dacWatermarkIrq|dacReadPtrTopIrq|dacReadPtrBottomIrq;
   }
   
   /**
    * Configure DAC from values specified in init
   
    * @param init Class containing initialisation values
    */
   static void configure(const typename Info::Init &init) {
   
      enable();
   
      if constexpr (Info::irqHandlerInstalled) {
         // Only set call-back if feature enabled
         setCallback(init.callback);
         enableNvicInterrupts(init.irqlevel);
      }
   
      dac->C0 = init.c0;
      dac->C1 = init.c1;
      dac->C2 = init.c2;
   }
   
   /**
    * Configure DAC with default settings
    */
   static void defaultConfigure() {
   
      configure(Info::DefaultInitValue);
   }



   /**
    * Configure DAC.
    * Interrupts are initially disabled.
    *
    * @param dacReferenceSelect     Reference Select
    * @param dacPower               Power control
    * @param dacTriggerSelect       Trigger Select
    */
   static void configure(
         DacReferenceSelect dacReferenceSelect = DacReferenceSelect_Vdda,
         DacPower           dacPower           = DacPower_Low,
         DacTriggerSelect   dacTriggerSelect   = DacTriggerSelect_Software) {

      enable();
      dac->C0 = DAC_C0_DACEN_MASK|dacReferenceSelect|dacTriggerSelect|dacPower;
   }

#ifdef DAC_C1_DACBFWM
   /**
    *  Configure DAC buffer operation
    *
    * @param dacBufferMode  Select if buffer is used and how the buffer pointer changes.
    * @param dacWaterMark   Selects water mark level for buffer.
    */
   static void configureBuffer(
         DacBufferMode dacBufferMode  = DacBufferMode_Disabled,
         DacWaterMark  dacWaterMark   = DacWaterMark_Normal_1
          ) {
      dac->C1 =
            (dac->C1&~(DAC_C1_DACBFEN_MASK|DAC_C1_DACBFMD_MASK|DAC_C1_DACBFWM_MASK))|
            dacBufferMode|dacWaterMark;
   }
#else
   /**
    *  Configure DAC buffer operation
    *
    * @param dacBufferMode  Select if buffer is used and how the buffer pointer changes.
    */
   static void configureBuffer(
         DacBufferMode dacBufferMode  = DacBufferMode_Disabled
          ) {
      dac->C1 =
            (dac->C1&~(DAC_C1_DACBFEN_MASK|DAC_C1_DACBFMD_MASK))|
            dacBufferMode;
   }
#endif

   /**
    * Get output range of DAC
    *
    * @return Range of DAC e.g. 2^12-1
    */
   static constexpr unsigned getRange() {
      return DAC_DATA_DATA_MASK;
   }

   /**
    * Get size of ADC buffer
    *
    * @return size in entries
    */
   static constexpr unsigned getBufferSize() {
      return sizeofArray(dac->DATA);
   }

   /**
    * Used to modify the FIFO read and write pointers in FIFO mode.
    *
    * @param writePtr  Write pointer
    * @param readPtr   Read pointer
    */
   static void setFifoPointers(uint8_t writePtr, uint8_t readPtr) {
      usbdm_assert(readPtr<getBufferSize(), "Illegal read pointer");
      usbdm_assert(writePtr<getBufferSize(),"Illegal write pointer");
      dac->C2 = DAC_C2_DACBFRP(readPtr)|DAC_C2_DACBFUP(writePtr);
   }

   /**
    * Clear (empty) FIFO.
    */
   static void clearFifo() {
      dac->C2 = DAC_C2_DACBFRP(0)|DAC_C2_DACBFUP(0);
   }

   /**
    * Set buffer upper limit in buffered modes.
    *
    * @param limit Upper limit for buffer index (inclusive)
    */
   static void setBufferLimit(uint8_t limit) {
      usbdm_assert(limit<getBufferSize(),"Illegal limit value");
      dac->C2 = (dac->C2&~DAC_C2_DACBFUP_MASK)|DAC_C2_DACBFUP(limit);
   }

   /**
    * Set buffer write index in buffered modes.
    *
    * @param index Write index (0..N)
    */
   static void setBufferWritePointer(uint8_t index) {
      usbdm_assert(index<getBufferSize(),"Illegal write index");
      dac->C2 = (dac->C2&~DAC_C2_DACBFRP_MASK)|DAC_C2_DACBFRP(index);
   }

   /**
    * Enable DMA mode
    */
   static void enableDma() {
      dac->C1 = dac->C1 | DAC_C1_DMAEN_MASK;
   }

   /**
    * Disable DMA mode
    */
   static void disableDma() {
      dac->C1 = dac->C1 & ~DAC_C1_DMAEN_MASK;
   }

   /**
    * Do a software trigger on the DAC.
    * If DAC software trigger is selected and buffer is enabled then
    * the buffer read pointer will be advanced once.
    */
   static void softwareTrigger() {
      dac->C0 = dac->C0 | DAC_C0_DACSWTRG_MASK;
   }

   /**
    * Enable interrupts in NVIC
    */
   static void enableNvicInterrupts() {
      NVIC_EnableIRQ(Info::irqNums[0]);
   }

   /**
    * Enable and set priority of interrupts in NVIC
    * Any pending NVIC interrupts are first cleared.
    *
    * @param[in]  nvicPriority  Interrupt priority
    */
   static void enableNvicInterrupts(NvicPriority nvicPriority) {
      enableNvicInterrupt(Info::irqNums[0], nvicPriority);
   }

   /**
    * Disable interrupts in NVIC
    */
   static void disableNvicInterrupts() {
      NVIC_DisableIRQ(Info::irqNums[0]);
   }

   /**
    * Enable DAC output pin as output.
    * Configures all Pin Control Register (PCR) values
    */
   static void setOutput() {

      CheckOutputIsMapped<Info::outputPin>::check();

      using Pcr = PcrTable_T<Info, Info::outputPin>;

      // Enable and map pin to CMP_OUT
      Pcr::setPCR();
   }

   /**
    * Get DAC status
    *
    * @return DAC status value see DacStatus
    */
   static uint8_t getStatus() {
      return dac->SR;
   }

   /**
    * Get and clear DAC status
    *
    * @return DAC status value see DacStatus
    */
   static uint8_t getAndClearStatus() {
      // Get status
      uint8_t status = dac->SR;
      // Clear set flags
      dac->SR = ~status;
      // return original status
      return status;
   }
   /**
    *   Disable the DAC
    */
   static void finalise() {
      // Enable timer
      dac->C0 = 0;
      dac->C1 = 0;
      Info::disableClock();
   }
   /**
    * Write output value for non-buffered mode or write value to FIFO for FIFO mode.
    *
    * @param value 12-bit value to write to DAC or FIFO
    */
   static void writeValue(uint16_t value) {
      dac->DATA[0] = DAC_DATA_DATA(value);
   }

   /**
    * Write DAC value to buffer (for buffered non-FIFO modes).
    *
    * @param index Index value for output buffer
    * @param value 12-bit value to write to DAC output buffer
    */
   static void writeValue(unsigned index, uint16_t value) {
      usbdm_assert(index<getBufferSize(), "Buffer index out of range");
      dac->DATA[index] = DAC_DATA_DATA(value);
   }

};

/**
 * Callback table for programmatically set handlers
 */
template<class Info> typename Info::CallbackFunction DacBase_T<Info>::sCallback =  DacBase_T<Info>::unhandledCallback;

   /**
    * Class representing DAC0
    */
   class Dac0 : public DacBase_T<Dac0Info> {};
   /**
    * Class representing DAC1
    */
   class Dac1 : public DacBase_T<Dac1Info> {};

/**
 * @}
 */
} // End namespace USBDM

#endif /* HEADERS_DAC_H_ */
