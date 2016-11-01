/*
 * SCI.h
 *
 *  Created on: 29/09/2010
 *      Author: PODonoghue
 */

#ifndef SCI_H_
#define SCI_H_

#include <stdint.h>
#include "usb_defs.h"

// Interrupt handlers
void cdc_txHandler(void);
void cdc_rxHandler(void);
//void cdcErrorHandler(void);

// SCI Tx Buffer 
bool cdc_putTxBuffer(char *source, uint8_t size);
uint8_t cdc_txBufferIsFree(void);

// SCI Rx
uint8_t cdc_setRxBuffer(char *buffer);
uint8_t cdc_rxBufferItemCount(void);
void checkUsbCdcRxData(void);

void cdc_setLineCoding(const LineCodingStructure *lineCodingStructure);
const LineCodingStructure *cdc_getLineCoding(void);
void cdc_setControlLineState(uint8_t value);
void cdc_sendBreak(uint16_t length);

#define SERIAL_STATE_CHANGE (1<<7)
uint8_t cdc_getSerialState(void);

#endif /* SCI_H_ */
