/*
 * TargetVddInterface.h
 *
 *  Created on: 21Dec.,2016
 *      Author: podonoghue
 */

#ifndef PROJECT_HEADERS_TARGETVDDINTERFACE_H_
#define PROJECT_HEADERS_TARGETVDDINTERFACE_H_

#include <math.h>
#include "hardware.h"
#include "cmp.h"
#include "console.h"
#include "commands.h"

/**
 * State of VDD control interface
 */
enum VddState {
   VddState_None,         //!< Vdd Off
   VddState_Internal,     //!< Vdd Internal
   VddState_External,     //!< Vdd External
   VddState_Overloaded,   //!< Internal Vdd overloaded & off)
};

/**
 * Low-level interface to Target Vdd (Vbdm) control and sensing
 */
class TargetVddInterface {

private:
   /**
    * Represents the 2:1 resistor voltage divider on input
    */
   static constexpr int externalDivider = 2;

   /**
    * Vref of Comparator and ADC
    */
   static constexpr float Vref = 3.3f;

   /**
    * Minimum working input 5V voltage as an ADC reading \n
    * 5V-5% as ADC reading i.e. 0-255
    */
   static constexpr int onMinThreshold5VAdc = (int)((4.75/(externalDivider*Vref))*(USBDM::Adc::getSingleEndedMaximum(USBDM::AdcResolution_8bit_se)));

   /**
    * Maximum working input 5V voltage as an ADC reading \n
    * 5V-5% as ADC reading i.e. 0-255
    */
   static constexpr int onMaxThreshold5VAdc = (int)((5.25/(externalDivider*Vref))*(USBDM::Adc::getSingleEndedMaximum(USBDM::AdcResolution_8bit_se)));

   /**
    * Minimum working input 3.3V voltage as an ADC reading \n
    * 3.3-5% as ADC reading i.e. 0-255
    */
   static constexpr int onMinThreshold3V3Adc = (int)((3.153/(externalDivider*Vref))*(USBDM::Adc::getSingleEndedMaximum(USBDM::AdcResolution_8bit_se)));

   /**
    * Maximum working input 3.3V voltage as an ADC reading \n
    * 3.3+5% as ADC reading i.e. 0-255
    */
   static constexpr int onMaxThreshold3V3Adc = (int)((3.47/(externalDivider*Vref))*(USBDM::Adc::getSingleEndedMaximum(USBDM::AdcResolution_8bit_se)));

   /**
    * Minimum working input 1.5V voltage as an ADC reading \n
    * 1.5-5% as ADC reading i.e. 0-255
    */
   static constexpr int onMinThreshold1V5Adc = (int)((1.4/(externalDivider*Vref))*(USBDM::Adc::getSingleEndedMaximum(USBDM::AdcResolution_8bit_se)));

   /**
    * Minimum working voltage as an DAC value. \n
    * 1.5V as DAC value i.e. 0-63
    */
   static constexpr int onThresholdDac = (int)((1.5/(externalDivider*Vref))*((1<<6)-1));

   /**
    * Minimum working voltage as an DAC value. \n
    * 0.8V as DAC value i.e. 0-63
    */
   static constexpr int powerOnResetThresholdDac = (int)((0.8/(externalDivider*Vref))*((1<<6)-1));

   /**
    * Minimum POR input voltage as an ADC reading. \n
    * This is the voltage needed to ensure a power-on-reset of the target. \n
    * 0.8 V as ADC reading
    */
   static constexpr int powerOnResetThresholdAdc = (int)((0.8/(externalDivider*Vref))*(USBDM::Adc::getSingleEndedMaximum(USBDM::AdcResolution_8bit_se)));

   /**
    * GPIO for Target Vdd enable pin
    */
   using Control = USBDM::TVdd_Enable; // USBDM::GpioD<1, USBDM::ActiveHigh>;

   /**
    * GPIO for Target Vdd LED
    */
   using Led = USBDM::TVdd_Led; // USBDM::GpioB<3, USBDM::ActiveHigh>;

   /**
    * ADC channel for Target Vdd measurement
    */
   using VddMeasure = USBDM::TVdd_Measure; // USBDM::Adc0::Channel<12>;

   /**
    * Comparator to monitor Vdd level
    */
   using VddMonitor = USBDM::TVdd_Monitor; // USBDM::Cmp0;

   /**
    * GPIO used to monitor power switch error indicator
    */
   using VddPowerFaultMonitor = USBDM::TVdd_Fault; // USBDM::GpioD<0, USBDM::ActiveHigh>;

   /**
    * Callback for Vdd changes
    */
   static void (*fCallback)(VddState);

   /**
    * Target Vdd state
    */
   static VddState vddState;

   /**
    * Dummy routine used if callback is not set
    */
   static void nullCallback(VddState) {
#ifdef DEBUG_BUILD
      __BKPT();
#endif
   }

public:
   /**
    * Monitors Target Vdd (Vbdm) level via comparator
    */
   static void vddMonitorCallback(const USBDM::CmpStatus &status) {
      if (status.event == USBDM::CmpEventId_Falling) {
         // Falling edge
         switch(vddState) {
            case VddState_Overloaded:
            case VddState_None:
               break;
            case VddState_External:
               // External power removed
               vddState = VddState_None;
               break;
            case VddState_Internal:
               // Fault (overload) detected
               vddState = VddState_Overloaded;
               break;
         }
         // In case Vdd overload
         Control::off();
         Led::off();
      }
      if (status.event == USBDM::CmpEventId_Rising) {
         // Rising edge
         switch(vddState) {
            case VddState_Internal:
               break;
            case VddState_Overloaded:
            case VddState_External:
            case VddState_None:
               // External power supplied
               vddState = VddState_External;
               break;
         }
         Led::on();
      }
      // Notify callback
      fCallback(vddState);
   }

   /**
    * Monitors Target Vdd (Vbdm) power switch overload
    * GPIO falling edge IRQ
    *
    * @param status Bit mask for entire port
    */
   static void powerFaultCallback(uint32_t status) {

      if ((VddPowerFaultMonitor::BITMASK & status) != 0) {

         // In case Vdd overload
         Control::off();

         // Fault (overload) detected
         vddState = VddState_Overloaded;

         // Notify callback
         fCallback(vddState);
      }
   }

   /**
    * Set callback to execute on Target Vdd (Vbdm) changes
    *
    * @param[in] callback Callback for target Vdd state changes (may be null)
    */
   static void setCallback(void (*callback)(VddState)) {
      if (callback == nullptr) {
         callback = nullCallback;
      }
      fCallback = callback;
   }

   /**
    * Initialise Vdd control and measurement interface
    *
    * @param[in] callback Callback for target Vdd state changes (may be null)
    */
   static void initialise(void (*callback)(VddState)) {
      using namespace USBDM;

      Control::setOutput();

      // Do default calibration for 8-bits
      VddMeasure::OwningAdc::configure(
            AdcResolution_8bit_se,
            AdcClockSource_Bus,
            AdcSample_4);
      VddMeasure::OwningAdc::calibrate();

      Led::setOutput(
            PinDriveStrength_High,
            PinDriveMode_PushPull,
            PinSlewRate_Slow);

      setCallback(callback);

      VddMonitor::setCallback(vddMonitorCallback);
      VddMonitor::configure(
            CmpPower_HighSpeed,
            CmpHysteresis_Level_2,
            CmpPolarity_Normal);
      VddMonitor::configureDac(
            powerOnResetThresholdDac,
            CmpDacrefSel_Vdd);
      VddMonitor::selectInputs(TVdd_Mon::plusPin, TVdd_DacRef::minusPin);
      VddMonitor::enableInterrupts(CmpEvent_OnEither);
      VddMonitor::enableNvicInterrupts(NvicPriority_Normal);

      VddPowerFaultMonitor::setPinCallback(powerFaultCallback);
      VddPowerFaultMonitor::setInput(
            PinPull_Up,
            PinAction_IrqFalling,
            PinFilter_Passive);
      VddPowerFaultMonitor::enableNvicPinInterrupts(NvicPriority_High);

      vddState = VddState_None;
      if (isVddOK_3V3()) {
         vddState = VddState_External;
      }
   }

   /**
    * Turn on Target Vdd
    *
    * @note This has no effect if in error state
    */
   static void vddOn() {
      if (vddState == VddState_Overloaded) {
         return;
      }
      vddState  = VddState_Internal;
      Control::on();
   }

   /**
    * Turn on Target Vdd @ 3.3V\n
    * Dummy routine as voltage level controlled by physical link
    *
    * @note This has no effect if in error state
    */
   static void vdd3V3On() {
      vddOn();
   }

   /**
    * Turn on Target Vdd @ 5V\n
    * Dummy routine as voltage level controlled by physical link
    *
    * @note This has no effect if in error state
    */
   static void vdd5VOn() {
      vddOn();
   }

   /**
    * Turn off Target Vdd
    *
    * @note - Will reset error state
    */
   static void vddOff() {
      Control::off();
      vddState  = VddState_None;
   }

   /**
    * Read target Vdd
    *
    * @return Target Vdd as an integer in the range 0-255 => 0-5V
    */
   static int readRawVoltage() {
      VddMeasure::OwningAdc::setResolution(USBDM::AdcResolution_8bit_se);
      return round(VddMeasure::readAnalogue()*externalDivider*5/Vref);
   }

   /**
    * Read target Vdd
    *
    * @return Target Vdd in volts as a float
    */
   static float readVoltage() {
      VddMeasure::OwningAdc::setResolution(USBDM::AdcResolution_8bit_se);
      return VddMeasure::readAnalogue()*Vref*externalDivider/((1<<8)-1);
   }

   /**
    * Check if target Vdd is present in acceptable range. \n
    * Also updates Target Vdd LED
    *
    * @param voltage Target voltage to check as ADC value (8-resolution)
    *
    * @return true  => Target Vdd in given range
    * @return false => Target Vdd not in given or interface in error state (overload etc.)
    */
   static bool isVddOK(int vmin, int vmax) {
      if (vddState == VddState_Overloaded) {
         return false;
      }
      VddMeasure::OwningAdc::setResolution(USBDM::AdcResolution_8bit_se);
      int value = VddMeasure::readAnalogue();
      if (value>=vmin) {
         Led::on();
         return (value<=vmax);
      }
      else {
         Led::off();
         return false;
      }
   }
   /**
    * Check if target Vdd is present \n
    * Also updates Target Vdd LED
    *
    * @return true  => Target Vdd =  ~3V3
    * @return false => Target Vdd != ~3V3
    */
   static bool isVddOK_3V3() {
      return isVddOK(onMinThreshold3V3Adc, onMaxThreshold3V3Adc);
   }

   /**
    * Check if target Vdd is present \n
    * Also updates Target Vdd LED
    *
    * @return true  => Target Vdd =  ~5V
    * @return false => Target Vdd != ~5V
    */
   static bool isVddOK_5V() {
      return isVddOK(onMinThreshold5VAdc, onMaxThreshold5VAdc);
   }

   /**
    * Check if target Vdd is present \n
    * Also updates Target Vdd LED
    *
    * @return true  => Target Vdd >= ~1.5V
    * @return false => Target Vdd <  ~1.5V
    */
   static bool isVddPresent() {
      VddMeasure::OwningAdc::setResolution(USBDM::AdcResolution_8bit_se);
      if (VddMeasure::readAnalogue()<onMinThreshold1V5Adc) {
         Led::off();
         return false;
      }
      else {
         Led::on();
         return true;
      }
   }

   /**
    * Check if target Vdd has fallen to POR level\n
    * Also updates Target Vdd LED
    */
   static bool isVddLow() {
      VddMeasure::OwningAdc::setResolution(USBDM::AdcResolution_8bit_se);
      if (VddMeasure::readAnalogue()<powerOnResetThresholdAdc) {
         Led::off();
         return true;
      }
      else {
         Led::on();
         return false;
      }
   }

   /**
    * Clear VDD change flag
    */
   static void clearVddChangeFlag() {
      VddMonitor::clearInterruptFlags();
   }

//   /**
//    * Get Vdd state
//    *
//    * @return Vdd state as VddState_None, VddState_Internal, VddState_External or VddState_Overloaded
//    */
//   static VddState getState() {
//      return vddState;
//   }

   /**
    * Update Vdd state
    *
    * @return Vdd state as VddState_None, VddState_Internal, VddState_External or VddState_Overloaded
    */
   static VddState checkVddState() {
      switch(vddState) {
         case VddState_Overloaded    :
            // No change - requires Vdd to be turned off to clear
            break;

         case VddState_Internal :
            if (!isVddOK_3V3() && !isVddOK_5V()) {
               // In case Vdd overload
               Control::off();
               // Power should be present in expected range!
               vddState = VddState_Overloaded;
            }
            break;

         case VddState_External :
         case VddState_None     :
            if (isVddOK_3V3()) {
               vddState = VddState_External;
            }
            else {
               vddState = VddState_None;
            }
            break;
      }
      return vddState;
   }
};

#endif /* PROJECT_HEADERS_TARGETVDDINTERFACE_H_ */
