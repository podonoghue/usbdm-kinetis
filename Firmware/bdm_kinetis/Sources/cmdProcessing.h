/*! \file
    \brief Main command procedure for executing BDM commands received over the USB.
*/    
#ifndef _CMDPROCESSING_H_
#define _CMDPROCESSING_H_
#include <stdint.h>
#include "commands.h"

//! Buffer for USB command in, result out
extern uint8_t   commandBuffer[MAX_COMMAND_SIZE+4];

//! Size of command return result
extern int       returnSize;

extern void             commandLoop(void);
extern USBDM_ErrorCode  compatibleCommandExec(void);
extern USBDM_ErrorCode  optionalReconnect(uint8_t when);

//! Target status
typedef struct {
   TargetType_t        target_type:8;  //!< Target type \ref TargetType_t
   AcknMode_t          ackn:8;         //!< Target supports ACKN see \ref AcknMode_t
   ResetMode_t         reset:8;        //!< Target has been reset, see \ref ResetMode_T
   SpeedMode_t         speed:8;        //!< Target speed determination method, see \ref SpeedMode_t
   TargetVddState_t    power:8;        //!< Target Vdd state
   TargetVppSelect_t   flashState:8;   //!< State of RS08 Flash programming,  see \ref FlashState_t
   uint16_t            sync_length;    //!< Length of the target SYNC pulse in 60MHz ticks
   uint16_t            wait150_cnt;    //!< Time for 150 BDM cycles in bus cycles of the MCU divided by N
   uint16_t            wait64_cnt;     //!< Time for 64 BDM cycles in bus cycles of the MCU divided by N
   uint8_t             bdmpprValue;    //!< BDMPPR value for HCS12
} CableStatus_t;

//! Target interface options
typedef struct {
   uint8_t  cycleVddOnReset:1;      //!< Cycle target Power  when resetting
   uint8_t  cycleVddOnConnect:1;    //!< Cycle target Power if connection problems (when resetting?)
   uint8_t  leaveTargetPowered:1;   //!< Leave target power on exit
   uint8_t  guessSpeed:1;           //!< Guess speed for target w/o ACKN
   uint8_t  useResetSignal:1;       //!< Use RESET signal on BDM interface
   uint8_t  targetVdd;              //!< Target Vdd (off, 3.3V or 5V)
   uint8_t  useAltBDMClock;         //!< Use alternative BDM clock source in target (HCS08)
   uint8_t  autoReconnect;          //!< Automatically re-connect method (for speed change)
   uint16_t SBDFRaddress;           //!< Address of HCS08_SBDFR register
   uint8_t  reserved[3];
} BDM_Option_t;

extern CableStatus_t cable_status;  // Status of the BDM interface
extern BDM_Option_t  bdm_option;    // Options for cable operation

#endif
