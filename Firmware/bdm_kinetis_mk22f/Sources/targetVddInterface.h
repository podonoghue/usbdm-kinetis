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

/**
 * State of VDD control interface
 */
enum VddState {
   VddState_None,       //!< Vdd Off
   VddState_Internal,   //!< Vdd Internal
   VddState_External,   //!< Vdd External
   VddState_Error,      //!< Vdd in Error (overloaded & off)
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
   static constexpr int onThreshold5VAdc = (int)((4.75/(externalDivider*Vref))*((1<<8)-1));

   /**
    * Minimum working input 3.3V voltage as an ADC reading \n
    * 3.3-5% as ADC reading i.e. 0-255
    */
   static constexpr int onThreshold3V3Adc = (int)((3.15/(externalDivider*Vref))*((1<<8)-1));

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
   static constexpr int powerOnResetThresholdAdc = (int)((0.8/(externalDivider*Vref))*((1<<8)-1));

   /**
    * GPIO for Target Vdd enable pin
    */
   using Control = USBDM::GpioD<1, USBDM::ActiveHigh>;

   /**
    * GPIO for Target Vdd LED
    */
   using Led = USBDM::GpioB<3, USBDM::ActiveHigh>;

   /**
    * ADC channel for Target Vdd measurement
    */
   using VddMeasure = USBDM::Adc0::Channel<12>;

   /**
    * Comparator to monitor Vdd level
    */
   using VddMonitor = USBDM::Cmp0;

   /**
    * GPIO used to monitor power switch error indicator
    */
   using VddPowerFaultMonitor = USBDM::GpioD<0, USBDM::ActiveHigh>;

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
   static void vddMonitorCallback(USBDM::CmpStatus status) {
      if (status.event == USBDM::CmpEvent_Falling) {
         // Falling edge
         switch(vddState) {
            case VddState_Error:
            case VddState_None:
               break;
            case VddState_External:
               // External power removed
               vddState = VddState_None;
               break;
            case VddState_Internal:
               // Fault (overload) detected
               vddState = VddState_Error;
               break;
         }
         // In case Vdd overload
         Control::off();
         Led::off();
      }
      if (status.event == USBDM::CmpEvent_Rising) {
         // Rising edge
         switch(vddState) {
            case VddState_Error:
            case VddState_External:
            case VddState_Internal:
               break;
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

      if ((VddPowerFaultMonitor::MASK & status) != 0) {

         // In case Vdd overload
         Control::off();

         // Fault (overload) detected
         vddState = VddState_Error;

         // Notify callback
         fCallback(vddState);
      }
   }

   /**
    * Set callback to execute on Target Vdd (Vbdm) changes
    *
    * @param[in] callback Callback to execute
    */
   static void setCallback(void (*callback)(VddState)) {
      if (callback == nullptr) {
         callback = nullCallback;
      }
      fCallback = callback;
   }

   /**
    * Initialise Vdd control and measurement interface
    */
   static void initialise(void (*callback)(VddState)) {
      Control::setOutput();

      // Do default calibration for 8-bits
      VddMeasure::Adc::configure(USBDM::AdcResolution_8bit_se);
      VddMeasure::Adc::calibrate();

      Led::setOutput(
            USBDM::PinDriveStrength_High,
            USBDM::PinDriveMode_PushPull,
            USBDM::PinSlewRate_Slow);

      fCallback = callback;

      VddMonitor::setCallback(vddMonitorCallback);
      VddMonitor::configure(
            USBDM::CmpPower_HighSpeed,
            USBDM::CmpHysteresis_2,
            USBDM::CmpPolarity_Noninverted);
      VddMonitor::configureDac(
            powerOnResetThresholdDac,
            USBDM::CmpDacSource_Vdda);
      VddMonitor::selectInputs(USBDM::Cmp0Input_1, USBDM::Cmp0Input_Cmp0Dac);
      VddMonitor::enableInterrupts(USBDM::CmpInterrupt_Both);
      VddMonitor::enableNvicInterrupts(true);

      VddPowerFaultMonitor::setCallback(powerFaultCallback);
      VddPowerFaultMonitor::setInput(
            USBDM::PinPull_Up,
            USBDM::PinAction_IrqFalling,
            USBDM::PinFilter_Passive);
      VddPowerFaultMonitor::enableNvicInterrupts();

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
      if (vddState == VddState_Error) {
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
      if (isVddOK_3V3()) {
         vddState = VddState_External;
      }
   }

   /**
    * Read target Vdd
    *
    * @return Target Vdd as an integer in the range 0-255 => 0-5V
    */
   static int readRawVoltage() {
      VddMeasure::Adc::setResolution(USBDM::AdcResolution_8bit_se);
      return round(VddMeasure::readAnalogue()*externalDivider*5/Vref);
   }

   /**
    * Read target Vdd
    *
    * @return Target Vdd in volts as a float
    */
   static float readVoltage() {
      VddMeasure::Adc::setResolution(USBDM::AdcResolution_8bit_se);
      return VddMeasure::readAnalogue()*Vref*externalDivider/((1<<8)-1);
   }

   /**
    * Check if target Vdd is present. \n
    * Also updates Target Vdd LED
    *
    * @param voltage Target voltage to check as ADC value (8-resolution)
    *
    * @return true  => Target Vdd >= voltage
    * @return false => Target Vdd < voltage
    */
   static bool isVddOK(int voltage) {
      if (vddState == VddState_Error) {
         return false;
      }
      VddMeasure::Adc::setResolution(USBDM::AdcResolution_8bit_se);
      int value = VddMeasure::readAnalogue();
      if (value>voltage) {
         Led::on();
         return true;
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
    * @return true  => Target Vdd >= 3V3
    * @return false => Target Vdd < 3V3
    */
   static bool isVddOK_3V3() {
      return isVddOK(onThreshold3V3Adc);
   }

   /**
    * Check if target Vdd is present \n
    * Also updates Target Vdd LED
    *
    * @return true  => Target Vdd >= 5V
    * @return false => Target Vdd < 5V
    */
   static bool isVddOK_5V() {
      return isVddOK(onThreshold5VAdc);
   }

   /**
    * Check if target Vdd has fallen to POR level\n
    * Also updates Target Vdd LED
    */
   static bool isVddLow() {
      VddMeasure::Adc::setResolution(USBDM::AdcResolution_8bit_se);
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
//    * @return Vdd state as VddState_None, VddState_Internal, VddState_External or VddState_Error
//    */
//   static VddState getState() {
//      return vddState;
//   }

   /**
    * Update Vdd state
    *
    * @return Vdd state as VddState_None, VddState_Internal, VddState_External or VddState_Error
    */
   static VddState checkVddState() {
      switch(vddState) {
         case VddState_Error    :
            // No change - requires Vdd to be turned off to clear
            break;

         case VddState_Internal :
            if (!isVddOK_3V3()) {
               // Power should be present!
               vddState = VddState_Error;
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
