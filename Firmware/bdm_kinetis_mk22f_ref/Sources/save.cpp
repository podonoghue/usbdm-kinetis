/*
 * save.cpp
 *
 *  Created on: 19Oct.,2016
 *      Author: podonoghue
 */


//static void tryIt() {
////   BkgdOut::low();
////   USBDM::waitUS(20);
////   BkgdOut::highZ();
////   USBDM::waitUS(20);
////   BkgdOut::low();
////   USBDM::waitUS(20);
////   BkgdOut::high();
//
//   Reset::low();
//   USBDM::waitUS(20);
//   Reset::highZ();
//   USBDM::waitUS(20);
//   Reset::low();
//   USBDM::waitUS(20);
//   Reset::high();
//}

//static void tryIt() {
//   static const uint32_t source[] = {1,2,3,4,5,6,7,8};
//   static uint32_t destination[16];
//   using DMAChannel = USBDM::DMAChannel_T<USBDM::DmaInfo, 0>;
//
//   static const USBDM::DMAChannel::SingleTransferInfo info = {
//      /* sourceAddress      */ source,
//      /* destinationAddress */ destination,
//      /* nBytes             */ sizeof(source),
//      /* attributes         */ DMA_ATTR_SMOD(0)|
//                               DMA_ATTR_SSIZE(DMAChannel::getAttrSize(sizeof(source[0])))|
//                               DMA_ATTR_DMOD(0)|
//                               DMA_ATTR_DSIZE(DMAChannel::getAttrSize(sizeof(destination[0]))),
//      /* sourceOffset       */ sizeof(source[0]),
//      /* destinationOffset  */ sizeof(destination[0]),
//   };
//   DMAChannel::configure(&info);
//   DMAChannel::waitUntilComplete();
//   __asm__("nop");
//}

//static void tryIt() {
//   uint32_t data[] = {1000, 20000, 30000, 4000};
//   using DMAChannel = USBDM::DMAChannel_T<USBDM::DmaInfo, 0, USBDM::DMA0_SLOT_FTM0_Ch_0>;
//
//   static const USBDM::DMAChannel::MultipleTransferInfo info = {
//      /* sourceAddress        */ data,
//      /* destinationAddress   */ &FTM0->CONTROLS[0].CnV,
//      /* nBytes               */ sizeof(data[0]),
//      /* attributes           */ DMA_ATTR_SMOD(0)|
//                                 DMA_ATTR_SSIZE(DMAChannel::getAttrSize(sizeof(data[0])))|
//                                 DMA_ATTR_DMOD(0)|
//                                 DMA_ATTR_DSIZE(DMAChannel::getAttrSize(sizeof(data[0]))),
//      /* sourceOffset         */ sizeof(source[0]),
//      /* destinationOffset    */ sizeof(source[0]),
//      /* numberOfTransactions */ sizeof(data)/sizeof(data[0]),
//   };
//   DMAChannel::configure(&info);
//   DMAChannel::waitUntilComplete();
//   __asm__("nop");
//}

//static void tryIt() {
//   struct {
//      uint32_t CnSC;
//      uint32_t CnV;
//   } source[] = {
//         {FTM_CnSC_},
//   };
//   uint32_t data[] = {1000, 20000, 30000, 4000};
//   USBDM::DMAChannel::Information info = {data, sizeof(data), &FTM0->CONTROLS[0].CnV, 0};
//   DMAChannel0::configure(&info);
//   DMAChannel0::waitUntilComplete();
//   __asm__("nop");
//}


//int main() {
//   SIM->SCGC5 = -1;
//
//   using Led = USBDM::Gpio_T<USBDM::GpioBInfo, 3>;
//
//   Led::setOutput();
//   Led::low();
//
//   BackgroundDebugInterface::enable();
//   Reset::high();
//   waitUS(10000);
//
//   //   BackgroundDebugInterface::testReset();
//   //   BackgroundDebugInterface::testBkgd();
//
//   uint16_t syncLength;
//   if (BackgroundDebugInterface::sync(syncLength)) {
//      float syncPeriod = ((float)syncLength*1000000)/BackgroundDebugInterface::getTickRate();
//      float frequency  = 128/syncPeriod;
//      PRINTF("OK: Sync = %d ticks, %3.1f us, bus freq %3.2f MHz\n", syncLength, syncPeriod, frequency);
//   }
//   else {
//      PRINTF("Sync failed\n");
//   }
//   for(;;) {
//      for(uint32_t i=0xE0; i<0xFF; i++) {
//         BackgroundDebugInterface::rx(i);
//         USBDM::waitUS(1000);
//      }
//   }
//   //   uint16_t syncLength;
//   //
//   //   Reset::low();
//   //   Bkgd::low();
//   //   waitMS(10000);
//   //   Reset::high();
//   //
//   //   for(;;) {
//   //      BackgroundDebugInterface::txByte(0);
//   //   }
//   //   for(;;) {
//   //      if (BackgroundDebugInterface::sync(syncLength)) {
//   //         float syncPeriod = ((float)syncLength*1000000)/BackgroundDebugInterface::getTickRate();
//   //         PRINTF("OK: Sync = %d ticks, %3.1f us\n", syncLength, syncPeriod);
//   ////         PRINTF("OK: Sync = %d ticks\n", syncLength);
//   //      }
//   //      else {
//   //         PRINTF("Sync failed\n");
//   //      }
//   ////      USBDM::waitMS(100);
//   //   }
//   return 0;
//}




