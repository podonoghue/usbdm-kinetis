/*
 * bdm.h
 *
 *  Created on: 26Nov.,2016
 *      Author: podonoghue
 */

#ifndef SOURCES_H_
#define SOURCES_H_

#include "system.h"
#include "derivative.h"
#include "hardware.h"
#include "delay.h"
#include "USBDM_MK20D5.h"

namespace Bdm {
/**
 * Initialise interface
 */
void initialise();

/**
 * Disables interface
 *
 * Note: Reset is not affected
 */
void disable();

/**
 * Set pin state
 *
 * @param pins Pin control mask
 */
void setPinState(PinLevelMasks_t pins);

/**
 * Get pin state
 *
 * @param [INOUT] status Updated with pin status from this interface
 */
void getPinState(PinLevelMasks_t &status);

/**
 * Determine connection speed using sync pulse
 *
 * @param [out] syncLength Sync length in timer ticks
 *
 * @return Error code, BDM_RC_OK indicates success
 */
USBDM_ErrorCode sync(uint16_t &syncLength);

/**
 * Determine connection speed using sync pulse
 *
 * @return Error code, BDM_RC_OK indicates success
 */
inline USBDM_ErrorCode sync() {
   uint16_t syncLength;
   return sync(syncLength);
}

/**
 *  Set sync length
 *
 *  @param [in] syncLength Sync length in timer ticks to set
 */
void setSyncLength(uint16_t syncLength);

/**
 *  Depending on ACKN mode this function:       \n
 *    - Waits for ACKN pulse with timeout.
 *       OR
 *    - Busy waits for 64 target CPU clocks
 *
 *  @return BDM_RC_OK          Success \n
 *  @return BDM_RC_ACK_TIMEOUT No ACKN detected [timeout]
 */
USBDM_ErrorCode acknowledgeOrWait64(void);

/**
 *  Depending on ACKN mode this function:       \n
 *    - Waits for ACKN pulse with timeout.
 *       OR
 *    - Busy waits for 150 target CPU clocks
 *
 *  @return BDM_RC_OK          Success \n
 *  @return BDM_RC_ACK_TIMEOUT No ACKN detected [timeout]
 */
USBDM_ErrorCode acknowledgeOrWait150(void);

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
USBDM_ErrorCode physicalConnect(void);

/**
 *  Connect to target
 *
 *  This function may cycle the target power in attempting to connect. \n
 *  It enables BDM on the target if connection is successful.
 *
 *  @return
 *     == \ref BDM_RC_OK                  Success  \n
 *     == \ref BDM_RC_VDD_NOT_PRESENT     No target power present \n
 *     == \ref BDM_RC_RESET_TIMEOUT_RISE  RESET signal timeout - remained low \n
 *     == \ref BDM_RC_BKGD_TIMEOUT        BKGD signal timeout - remained low \n
 *     != \ref BDM_RC_OK                  Other failures \n
 */
USBDM_ErrorCode connect(void);

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
USBDM_ErrorCode targetReset( uint8_t mode );

/**
 *  Halts the processor - places in background mode
 */
USBDM_ErrorCode halt(void);

/**
 *  Executes a single instruction on the target
 */
USBDM_ErrorCode step(void);

/**
 * Commences full-speed execution on the target
 */
USBDM_ErrorCode go(void);

/**
 *  Read Target BDM status
 *
 *  Depending on target architecture this reads from
 *  - BDCCSR
 *  - BDCSC,
 *  - BDMSTS,
 *  - XCSR.
 *
 * @param bdm_sts value read
 *
 * @return BDM_RC_OK               Success
 * @return BDM_RC_ILLEGAL_COMMAND  Command not available for this target
 */
USBDM_ErrorCode readBDMStatus(uint8_t *bdm_sts);
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
 *  @return BDM_RC_OK               Success
 *  @return BDM_RC_ILLEGAL_COMMAND  Command not available for this target
 */
USBDM_ErrorCode writeBDMControl(uint8_t value);

/**
 * Attempts to enable ACKN mode on target BDM interface.\n
 * It is not an error if it fails as some targets do not support this.
 */
void enableACKNMode(void);

/**
 *  If BDM mode is not enabled in target yet, enable it so it can be made active
 *
 *  @return BDM_RC_OK              Success
 *  @return BDM_RC_UNKNOWN_TARGET  Unknown target
 *  @return BDM_RC_BDM_EN_FAILED   Enabling BDM failed (target not connected or wrong speed ?)
 */
USBDM_ErrorCode enableBDM();

/**
 *  Resets the target using BDM commands
 *
 *  @note
 *     Not all targets support reset using BDM commands
 *
 *  @param mode\n
 *     - RESET_SPECIAL Reset to special mode,
 *     - RESET_NORMAL  Reset to normal mode
 *
 *  @return BDM_RC_OK                 Success \n
 *  @return BDM_RC_BKGD_TIMEOUT       BKGD pin stuck low \n
 *  @return BDM_RC_RESET_TIMEOUT_RISE RESET pin stuck low \n
 *  @return BDM_RC_UNKNOWN_TARGET     Don't know how to reset this type of target! \n
 */
USBDM_ErrorCode softwareReset(uint8_t mode);

/**
 *  Halts the processor - places in background mode
 */
USBDM_ErrorCode halt(void);

/**
 * Commences full-speed execution on the target
 */
USBDM_ErrorCode go(void);

/**
 * Write command byte, truncated sequence
 *
 * @param cmd command byte to write
 *
 * @note Interrupts are left disabled, no ACK is expected
 */
void cmd_0_0_T(uint8_t cmd);
/**
 *  Write command byte + parameter, truncated sequence
 *
 *  @param cmd         Command byte to write
 *  @param parameter   Byte parameter to write
 *
 * @note Interrupts are left disabled, no ACK is expected
 */
void cmd_1B_0_T(uint8_t cmd, uint8_t parameter);

/**
 *  Special for Software Reset HCS08, truncated sequence
 *
 *  @param cmd         Command byte to write
 *  @param parameter1  Word parameter to write
 *  @param parameter2  Byte parameter to write
 *
 * @note Interrupts are left disabled, no ACK is expected
 */
void cmd_1W1B_0_T(uint8_t cmd, uint16_t parameter1, uint8_t parameter2);

/**
 *  Write command without ACK (HCS08/RS08/CFV1)
 *
 *  @param cmd command byte to write
 *
 *  @note No ACK is expected
 */
void cmd_0_0_NOACK(uint8_t cmd);

/**
 * Write command & read byte without ACK (HCS08)
 *
 *  @param cmd command byte to write
 *  @param result word read
 *
 *  @note No ACK is expected
 */
void cmd_0_1B_NOACK(uint8_t cmd, uint8_t *result);

/**
 *  Write command & byte without ACK
 *
 *  @param cmd       command byte to write
 *  @param parameter byte to write
 *
 *  @note No ACK is expected
 */
void cmd_1B_0_NOACK(uint8_t cmd, uint8_t parameter);

/**
 *  Write command & read word without ACK (HCS08)
 *
 *  @param cmd    command byte to write
 *  @param result word read
 *
 *  @note No ACK is expected
 */
void cmd_0_1W_NOACK(uint8_t cmd, uint8_t result[2]);

/**
 *  Write command & word without ACK
 *
 *  @param cmd       command byte to write
 *  @param parameter word to write
 *
 *  @note No ACK is expected
 */
void cmd_1W_0_NOACK(uint8_t cmd, uint16_t parameter);

/**
 *  Write command, word & read word without ACK
 *
 *  @param cmd        command byte to write
 *  @param parameter  word to write
 *  @param result     word pointer for read (status+data byte)
 *
 *  @note no ACK is expected
 */
void cmd_1W_1W_NOACK(uint8_t cmd, uint16_t parameter, uint8_t result[2]);

/**
 *  Write command, word, byte & read byte without ACK
 *
 *  @param cmd        command byte to write
 *  @param parameter  word to write
 *  @param value      byte to write
 *  @param status     byte pointer for read
 *
 *  @note no ACK is expected
 */
void cmd_1W1B_1B_NOACK(uint8_t cmd, uint16_t parameter, uint8_t value, uint8_t *status);

/**
 *  Write command
 *
 *  @param cmd command byte to write
 *
 *  @note ACK is expected
 */
USBDM_ErrorCode cmd_0_0(uint8_t cmd);

/**
 *  Write command, read byte
 *
 *  @param cmd        command byte to write
 *  @param result     byte read
 *
 *  @note ACK is expected
 */
USBDM_ErrorCode cmd_0_1B(uint8_t cmd, uint8_t *result);

/**
 *  Write command & read word
 *
 *  @param cmd    command byte to write
 *  @param result word read
 *
 *  @note ACK is expected
 */
USBDM_ErrorCode cmd_0_1W(uint8_t cmd, uint8_t result[2]);

/**
 *  Write command & read longword
 *
 *  @param cmd    command byte to write
 *  @param result longword read
 *
 *  @note ACK is expected
 */
USBDM_ErrorCode cmd_0_1L(uint8_t cmd, uint8_t result[4]);

/**
 *  Write command & byte
 *
 *  @param cmd        command byte to write
 *  @param parameter  byte to write
 *
 *  @note ACK is expected
 */
USBDM_ErrorCode cmd_1B_0(uint8_t cmd, uint8_t parameter);

/**
 *  Write command & word
 *
 *  @param cmd       command byte to write
 *  @param parameter word to write
 *
 *  @note ACK is expected
 */
USBDM_ErrorCode cmd_1W_0(uint8_t cmd, uint16_t parameter);

/**
 *  Write command & longword
 *
 *  @param cmd       command byte to write
 *  @param parameter longword to write
 *
 *  @note ACK is expected
 */
USBDM_ErrorCode cmd_1L_0(uint8_t cmd, uint32_t parameter);

/**
 *  Write command, word & read byte (read word but return byte - HC/S12(x))
 *
 *  @param cmd       command byte to write
 *  @param parameter word to write
 *  @param result    byte read
 *
 *  @note ACK is expected
 */
USBDM_ErrorCode cmd_1W_1WB(uint8_t cmd, uint16_t parameter, uint8_t *result);

/**
 *  Write cmd & 2 words
 *
 *  @param cmd       command byte to write
 *  @param parameter1 word to write
 *  @param parameter2 word to write
 *
 *  @note ACK is expected
 */
USBDM_ErrorCode cmd_2W_0(uint8_t cmd, uint16_t parameter1, uint16_t parameter2);

/**
 *  Write command, word & read word
 *
 *  @param cmd        command byte to write
 *  @param parameter  word to write
 *  @param result     word read
 *
 *  @note ACK is expected
 */
USBDM_ErrorCode cmd_1W_1W(uint8_t cmd, uint16_t parameter, uint8_t result[2]);

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
USBDM_ErrorCode cmd_1W1B_1B(uint8_t cmd, uint16_t parameter, uint8_t value, uint8_t *status);

/**
 *  Write command, word and a byte
 *  (sends 2 words, the byte in both high and low byte of the 16-bit value)
 *
 *  @param cmd        command byte to write
 *  @param parameter1 word to write
 *  @param parameter2 bye to write
 *
 *  @note ACK is expected
 */
USBDM_ErrorCode cmd_2WB_0(uint8_t cmd, uint16_t parameter1, uint8_t parameter2);

/**
 *  Write cmd, word & read byte
 *
 *  @param cmd        command byte to write
 *  @param parameter  word to write
 *  @param result     byte read
 *
 *  @note ACK is expected
 */
USBDM_ErrorCode cmd_1W_1B(uint8_t cmd, uint16_t parameter, uint8_t *result);

/**
 *  Write command, word & byte
 *
 *  @param cmd         command byte to write
 *  @param parameter1  word to write
 *  @param parameter2  byte to write
 *
 *  @note ACK is expected
 */
USBDM_ErrorCode cmd_1W1B_0(uint8_t cmd, uint16_t parameter1, uint8_t parameter2);

/**
 *  Write cmd, 24-bit value & byte
 *
 *  @param cmd    command byte to write
 *  @param addr   24-bit value to write
 *  @param value  byte to write
 *
 *  @note ACK is expected
 */
USBDM_ErrorCode cmd_1A1B_0(uint8_t cmd, uint32_t addr, uint8_t value);

/**
 *  Write command, 24-bit value & word
 *
 *  @param cmd    command byte to write
 *  @param addr   24-bit value to write
 *  @param value  word to write
 *
 *  @note ACK is expected
 */
USBDM_ErrorCode cmd_1A1W_0(uint8_t cmd, uint32_t addr, uint16_t value);

/**
 *  Write command, 24-bit value & longword
 *
 *  @param cmd    command byte to write
 *  @param addr   24-bit value to write
 *  @param value  ptr to longword to write
 *
 *  @note ACK is expected
 */
USBDM_ErrorCode cmd_1A1L_0(uint8_t cmd, uint32_t addr, uint32_t value);

/**
 *  Write command, 24-bit value & read byte
 *
 *  @param cmd     command byte to write
 *  @param addr    24-bit value to write
 *  @param result  pointer to read location
 *
 *  @note ACK is expected
 */
USBDM_ErrorCode cmd_1A_1B(uint8_t cmd, uint32_t addr, uint8_t *result);

/**
 *  Write cmd, 24-bit value & read 16-bit value
 *
 *  @param cmd     command byte to write
 *  @param addr    24-bit value to write
 *  @param result  pointer to read location
 *
 *  @return Error code, BDM_RC_OK indicates success
 *
 *  @note ACK is expected
 */
USBDM_ErrorCode cmd_1A_1W(uint8_t cmd, uint32_t addr, uint8_t *result);

/**
 *  Write cmd, 24-bit value & read 16-bit value
 *
 *  @param cmd     command byte to write
 *  @param addr    24-bit value to write
 *  @param result  pointer to read location
 *
 *  @return Error code, BDM_RC_OK indicates success
 *
 *  @note ACK is expected
 */
USBDM_ErrorCode cmd_1A_1L(uint8_t cmd, uint32_t addr, uint8_t *result);

/**
 *  Confirm communication at given Sync value.
 *  Only works on HC12 (and maybe only 1 of 'em!)
 *
 *  @return
 *    == \ref BDM_RC_OK  => Success \n
 *    != \ref BDM_RC_OK  => Various errors
 */
USBDM_ErrorCode hc12confirmSpeed(unsigned syncLength);

}; // End namespace Bdm

#include <bdmMacros.h>

#endif /* SOURCES_H_ */
