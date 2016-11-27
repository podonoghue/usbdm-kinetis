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

namespace Bdm {
/**
 * Initialise BDM interface
 */
void initialise();

/**
 *  Set sync length
 *
 *  @param [in] syncLength Sync length in timer ticks to set
 */
void setSyncLength(uint16_t syncLength);

/**
 * Determine connection speed using sync pulse
 *
 * @param [out] syncLength Sync length in timer ticks
 *
 * @return Error code, BDM_RC_OK indicates success
 */
USBDM_ErrorCode sync(uint16_t &syncLength);

/**
 * Receive an 8-bit value over BDM interface
 *
 * @param [out] data Where to place data received
 */
USBDM_ErrorCode rxDualEdgePulse(uint8_t *data);

/**
 * Transmit an 8-bit value over BDM interface
 *
 * @param [in] data Data value to transmit
 */
USBDM_ErrorCode txDualEdgePulse(uint8_t data);

}; // End namespace Bdm

#endif /* SOURCES_BDM_H_ */
