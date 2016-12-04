/**
 * @file     CmdProcessingHCS.h
 * @brief    HCS08/12 Command Processing
 *
 * @version  V4.12.1.80
 * @date     13 April 2016
 */
#ifndef CMDPROCESSINGHCS_H_
#define CMDPROCESSINGHCS_H_

#include "commands.h"
#include "cmdProcessing.h"
#include "bdm.h"

namespace Hcs {
USBDM_ErrorCode f_CMD_CONNECT(void);
USBDM_ErrorCode f_CMD_SET_SPEED(void);
USBDM_ErrorCode f_CMD_GET_SPEED(void);

USBDM_ErrorCode f_CMD_READ_STATUS_REG(void);
USBDM_ErrorCode f_CMD_WRITE_CONTROL_REG(void);

USBDM_ErrorCode f_CMD_RESET(void);
USBDM_ErrorCode f_CMD_STEP(void);
USBDM_ErrorCode f_CMD_GO(void);
USBDM_ErrorCode f_CMD_HALT(void);
}; // End namespace Bdm

namespace Hcs08 {
USBDM_ErrorCode f_CMD_WRITE_MEM(void);
USBDM_ErrorCode f_CMD_READ_MEM(void);

USBDM_ErrorCode f_CMD_WRITE_REG(void);
USBDM_ErrorCode f_CMD_READ_REG(void);

USBDM_ErrorCode f_CMD_WRITE_BKPT(void);
USBDM_ErrorCode f_CMD_READ_BKPT(void);
}; // End namespace Hcs08

namespace Hcs12 {

USBDM_ErrorCode f_CMD_WRITE_MEM(void);
USBDM_ErrorCode f_CMD_READ_MEM(void);

USBDM_ErrorCode f_CMD_WRITE_REG(void);
USBDM_ErrorCode f_CMD_READ_REG(void);

USBDM_ErrorCode f_CMD_WRITE_BD(void);
USBDM_ErrorCode f_CMD_READ_BD(void);
}; // End namespace Hcs12

namespace S12z {
USBDM_ErrorCode f_CMD_CUSTOM_COMMAND(void);

}; // End namespace S12z

namespace Cfv1 {
USBDM_ErrorCode f_CMD_WRITE_MEM(void);
USBDM_ErrorCode f_CMD_READ_MEM(void);

USBDM_ErrorCode f_CMD_WRITE_REG(void);
USBDM_ErrorCode f_CMD_READ_REG(void);
USBDM_ErrorCode f_CMD_WRITE_DREG(void);
USBDM_ErrorCode f_CMD_READ_DREG(void);
USBDM_ErrorCode f_CMD_WRITE_CREG(void);
USBDM_ErrorCode f_CMD_READ_CREG(void);

USBDM_ErrorCode f_CMD_WRITE_CSR2(void);
USBDM_ErrorCode f_CMD_READ_CSR2(void);
USBDM_ErrorCode f_CMD_WRITE_CSR3(void);
USBDM_ErrorCode f_CMD_READ_CSR3(void);

USBDM_ErrorCode f_CMD_READ_ALL_CORE_REGS(void);
}; // End namespace Cfv1

#endif /* CMDPROCESSINGHCS_H_ */
