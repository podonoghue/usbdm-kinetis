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
 * Determine connection speed using sync pulse
 *
 * @param [out] syncLength Sync length in timer ticks
 *
 * @return Error code, BDM_RC_OK indicates success
 */
USBDM_ErrorCode sync(uint16_t &syncLength);

/**
 *  Set sync length
 *
 *  @param [in] syncLength Sync length in timer ticks to set
 */
void setSyncLength(uint16_t syncLength);

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
 *  @param cmd         command byte to write
 *  @param parameter   byte parameter to write
 *
 * @note Interrupts are left disabled, no ACK is expected
 */
void cmd_1B_0_T(uint8_t cmd, uint8_t parameter);

/**
 *  Special for Software Reset HCS08, truncated sequence
 *
 *  @param cmd         command byte to write
 *  @param parameter1  word parameter to write
 *  @param parameter2  byte parameter to write
 *
 * @note Interrupts are left disabled, no ACK is expected
 */
void cmd_1W1B_0_T(uint8_t cmd, uint16_t parameter1, uint8_t parameter2);

/**
 *  Write cmd without ACK (HCS08/RS08/CFV1)
 *
 *  @param cmd command byte to write
 *
 *  @note No ACK is expected
 */
void cmd_0_0_NOACK(uint8_t cmd);

/**
 * Write cmd & read byte without ACK (HCS08)
 *
 *  @param cmd command byte to write
 *  @param result word read
 *
 *  @note No ACK is expected
 */
void cmd_0_1B_NOACK(uint8_t cmd, uint8_t *result);

/**
 *  Write cmd & byte without ACK
 *
 *  @param cmd       command byte to write
 *  @param parameter byte to write
 *
 *  @note No ACK is expected
 */
void cmd_1B_0_NOACK(uint8_t cmd, uint8_t parameter);

/**
 *  Write cmd & read word without ACK (HCS08)
 *
 *  @param cmd    command byte to write
 *  @param result word read
 *
 *  @note No ACK is expected
 */
void cmd_0_1W_NOACK(uint8_t cmd, uint16_t *result);

/**
 *  Write cmd & word without ACK
 *
 *  @param cmd       command byte to write
 *  @param parameter word to write
 *
 *  @note No ACK is expected
 */
void cmd_1W_0_NOACK(uint8_t cmd, uint16_t parameter);

/**
 *  Write cmd, word & read word without ACK
 *
 *  @param cmd        command byte to write
 *  @param parameter  word to write
 *  @param result     word pointer for read (status+data byte)
 *
 *  @note no ACK is expected
 */
void cmd_1W_1W_NOACK(uint8_t cmd, uint16_t parameter, uint16_t *result);

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
void cmd_1W1B_1B_NOACK(uint8_t cmd, uint16_t parameter, uint8_t value, uint8_t *status);

/**
 *  Write cmd
 *
 *  @param cmd command byte to write
 *
 *  @note ACK is expected
 */
uint8_t cmd_0_0(uint8_t cmd);

/**
 *  Write cmd, read byte
 *
 *  @param cmd        command byte to write
 *  @param result     byte read
 *
 *  @note ACK is expected
 */
uint8_t cmd_0_1B(uint8_t cmd, uint8_t *result);

/**
 *  Write cmd & read word
 *
 *  @param cmd    command byte to write
 *  @param result word read
 *
 *  @note ACK is expected
 */
uint8_t cmd_0_1W(uint8_t cmd, uint16_t *result);

/**
 *  Write cmd & read longword
 *
 *  @param cmd    command byte to write
 *  @param result longword read
 *
 *  @note ACK is expected
 */
uint8_t cmd_0_1L(uint8_t cmd, uint32_t *result);

/**
 *  Write cmd & byte
 *
 *  @param cmd        command byte to write
 *  @param parameter  byte to write
 *
 *  @note ACK is expected
 */
uint8_t cmd_1B_0(uint8_t cmd, uint8_t parameter);

/**
 *  Write cmd & word
 *
 *  @param cmd       command byte to write
 *  @param parameter word to write
 *
 *  @note ACK is expected
 */
uint8_t cmd_1W_0(uint8_t cmd, uint16_t parameter);

/**
 *  Write cmd & longword
 *
 *  @param cmd       command byte to write
 *  @param parameter longword to write
 *
 *  @note ACK is expected
 */
uint8_t cmd_1L_0(uint8_t cmd, uint32_t parameter);

/**
 *  Write cmd, word & read byte (read word but return byte - HC/S12(x))
 *
 *  @param cmd       command byte to write
 *  @param parameter word to write
 *  @param result    byte read
 *
 *  @note ACK is expected
 */
uint8_t cmd_1W_1WB(uint8_t cmd, uint16_t parameter, uint8_t *result);

/**
 *  Write cmd & 2 words
 *
 *  @param cmd       command byte to write
 *  @param parameter1 word to write
 *  @param parameter2 word to write
 *
 *  @note ACK is expected
 */
uint8_t cmd_2W_0(uint8_t cmd, uint16_t parameter1, uint16_t parameter2);

/**
 *  Write cmd, word & read word
 *
 *  @param cmd        command byte to write
 *  @param parameter  word to write
 *  @param result     word read
 *
 *  @note ACK is expected
 */
uint8_t cmd_1W_1W(uint8_t cmd, uint16_t parameter, uint16_t *result);

/**
 *  Write cmd, word and a byte
 *  (sends 2 words, the byte in both high and low byte of the 16-bit value)
 *
 *  @param cmd        command byte to write
 *  @param parameter1 word to write
 *  @param parameter2 bye to write
 *
 *  @note ACK is expected
 */
uint8_t cmd_2WB_0(uint8_t cmd, uint16_t parameter1, uint8_t parameter2);

/**
 *  Write cmd, word & read byte
 *
 *  @param cmd        command byte to write
 *  @param parameter  word to write
 *  @param result     byte read
 *
 *  @note ACK is expected
 */
uint8_t cmd_1W_1B(uint8_t cmd, uint16_t parameter, uint8_t *result);

/**
 *  Write cmd, word & byte
 *
 *  @param cmd         command byte to write
 *  @param parameter1  word to write
 *  @param parameter2  byte to write
 *
 *  @note ACK is expected
 */
uint8_t cmd_1W1B_0(uint8_t cmd, uint16_t parameter1, uint8_t parameter2);

/**
 *  Write cmd, 24-bit value & byte
 *
 *  @param cmd    command byte to write
 *  @param addr   24-bit value to write
 *  @param value  byte to write
 *
 *  @note ACK is expected
 */
uint8_t cmd_1A1B_0(uint8_t cmd, uint32_t addr, uint8_t value);

/**
 *  Write cmd, 24-bit value & word
 *
 *  @param cmd    command byte to write
 *  @param addr   24-bit value to write
 *  @param value  word to write
 *
 *  @note ACK is expected
 */
uint8_t cmd_1A1W_0(uint8_t cmd, uint32_t addr, uint16_t value);

/**
 *  Write cmd, 24-bit value & longword
 *
 *  @param cmd    command byte to write
 *  @param addr   24-bit value to write
 *  @param value  ptr to longword to write
 *
 *  @note ACK is expected
 */
uint8_t cmd_1A1L_0(uint8_t cmd, uint32_t addr, uint32_t *value);

/**
 *  Write cmd, 24-bit value & read byte
 *
 *  @param cmd     command byte to write
 *  @param addr    24-bit value to write
 *  @param result  ptr to longword to read
 *
 *  @note ACK is expected
 */
uint8_t cmd_1A_1B(uint8_t cmd, uint32_t addr, uint8_t *result);

}; // End namespace Bdm

#endif /* SOURCES_H_ */
