/*
 * TargetVddInterface.h
 *
 *  Created on: 21Dec.,2016
 *      Author: podonoghue
 */

#ifndef PROJECT_HEADERS_TARGETVDDINTERFACE_H_
#define PROJECT_HEADERS_TARGETVDDINTERFACE_H_

#include "math.h"
#include "hardware.h"

/**
 * Low-level interface to Vdd control and sensing
 */
class TargetVddInterface {
private:
   /**
    * Represents the 2:1 voltage divider on input
    */
   static constexpr int externalDivider = 2;

   /**
    * Conversion factor for ADC reading to input voltage\n
    * 3.3V range, 8 bit conversion, voltage divider on input
    * V = ADCValue * scaleFactor
    */
   static constexpr float scaleFactor = (externalDivider*3.3)/((1<<8)-1);

   /**
    * Minimum working input voltage as an ADC reading \n
    * 1.5 V as ADC reading
    */
   static constexpr int onThreshold = (int)(1.5/scaleFactor);

   /**
    * Minimum working input voltage as an ADC reading \n
    * 0.5 V as ADC reading
    */
   static constexpr int offThreshold = (int)(0.5/scaleFactor);


private:
   /**
    * GPIO for Target Vdd enable pin
    */
   using Control = USBDM::GpioD<1>;

   /**
    * GPIO for Target Vdd LED
    */
   using Led = USBDM::GpioB<3>;

   /**
    * ADC channel for target Vdd measurement
    */
   using VddSense = USBDM::Adc0Channel<12>;

public:
   static void initialise() {
      Control::setOutput();
      vddOff();
      Led::setOutput();
      ledOff();
      VddSense::enable();
      VddSense::setResolution(USBDM::resolution_8bit_se);
      }

   /**
    * Turn on Target Vdd
    */
   static void vddOn() {
      Control::high();
   }

   /**
    * Turn on Target Vdd @ 3.3V\n
    * Dummy routine as voltage level controlled by physical link
    */
   static void vdd3V3On() {
      vddOn();
   }

   /**
    * Turn on Target Vdd @ 5V\n
    * Dummy routine as voltage level controlled by physical link
    */
   static void vdd5On() {
      vddOn();
   }

   /**
    * Turn off Target Vdd
    */
   static void vddOff() {
      Control::low();
   }

   /** Turn on TVdd LED */
   static void ledOn() {
      Led::high();
   }

   /** Turn off Tvdd LED */
   static void ledOff() {
      Led::low();
   }

   /**
    * Read target Vdd
    *
    * @return Target Vdd as an integer in the range 0-255 => 0-5V
    */
   static int readRawVoltage() {
      return round(VddSense::readAnalogue()*(externalDivider*3.3/5));
   }

   /**
    * Read target Vdd
    *
    * @return Target Vdd in volts as a float
    */
   static float readVoltage() {
      return VddSense::readAnalogue()*scaleFactor;
   }

   /**
    * Check if target Vdd is present\n
    * Also updates Target Vdd LED
    */
   static bool isVddOK() {
      if (VddSense::readAnalogue()>onThreshold) {
         ledOn();
         return true;
      }
      else {
         ledOff();
         return false;
      }
   }
   /**
    * Check if target Vdd is low\n
    * Also updates Target Vdd LED
    */
   static bool isVddLow() {
      if (VddSense::readAnalogue()<offThreshold) {
         ledOn();
         return true;
      }
      else {
         ledOff();
         return false;
      }
   }

   /**
    * Clear VDD change flag
    */
   static void clearVddChangeFlag() {
      // TODO Vdd Sensing
   }

   /**
    * Clear VDD change flag
    */
   static void enableVddChangeSense(bool enable) {
      // TODO Vdd Sensing
      (void)enable;
   }
};

#endif /* PROJECT_HEADERS_TARGETVDDINTERFACE_H_ */
