/*
 BitzOS (BOS) V0.4.0 - Copyright (C) 2017-2025 Hexabitz
 All rights reserved

 File Name     : motor.h
 Description   : This file contains all the functions prototypes for motor drivers.
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MOTOR_H
#define __MOTOR_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

   
/* boolean for false condition */
#ifndef FALSE
#define FALSE (0)
#endif
/* boolean for true condition */
#ifndef TRUE
#define TRUE  (1)
#endif

/* Device_Direction_Options Device Direction Options */
/* Direction options */
typedef enum {
  Backward = 0,
  Forward = 1,
  UNKNOW_DIR = ((uint8_t)0xFF)
} motorDir_t;

/* Device_Action_Options Device Action Options */
/* Action options */
typedef enum {
  ACTION_RESET = ((uint8_t)0x00),
  ACTION_COPY  = ((uint8_t)0x08)
} motorAction_t;

/* Device_States Device States */
/* Device states */
typedef enum {
  ACCELERATING       = 0,
  DECELERATINGTOSTOP = 1,  
  DECELERATING       = 2, 
  STEADY             = 3,
  INDEX_ACCEL        = 4,
  INDEX_RUN          = 5,
  INDEX_DECEL        = 6,
  INDEX_DWELL        = 7,
  INACTIVE           = 8,
  STANDBY            = 9,
  STANDBYTOINACTIVE  = 10
} motorState_t;

/* Device_Step_mode Device Step mode */
 /* Stepping options */
typedef enum {
  STEP_MODE_FULL   = ((uint8_t)0x00), 
  STEP_MODE_HALF   = ((uint8_t)0x01),
  STEP_MODE_1_4    = ((uint8_t)0x02),
  STEP_MODE_1_8    = ((uint8_t)0x03),
  STEP_MODE_1_16   = ((uint8_t)0x04),
  STEP_MODE_1_32   = ((uint8_t)0x05),
  STEP_MODE_1_64   = ((uint8_t)0x06),
  STEP_MODE_1_128  = ((uint8_t)0x07),
  STEP_MODE_1_256  = ((uint8_t)0x08),
  STEP_MODE_UNKNOW = ((uint8_t)0xFE),
  STEP_MODE_WAVE   = ((uint8_t)0xFF)  
} motorStepMode_t;
  
/* Decay_mode Decay mode */
/* Decay Mode */
typedef enum {
  SLOW_DECAY = 0,
  FAST_DECAY = 1,
  UNKNOW_DECAY = ((uint8_t)0xFF)
} motorDecayMode_t;
  
/* Stop_mode Stop mode */
/* Stop mode */
typedef enum {
	HOLD_MODE = 0,
	HIZ_MODE = 1,
	STANDBY_MODE = 2,
	UNKNOW_STOP_MODE = ((uint8_t) 0xFF)
} motorStopMode_t;

/* Torque_mode Torque mode */
/* Torque mode */
typedef enum {
	ACC_TORQUE = 0,
	DEC_TORQUE = 1,
	RUN_TORQUE = 2,
	HOLD_TORQUE = 3,
	CURRENT_TORQUE = 4,
	UNKNOW_TORQUE = ((uint8_t) 0xFF)
} motorTorqueMode_t;
    
/* Dual_Full_Bridge_Configuration Dual Full Bridge Configuration */
/* Dual full bridge configurations for brush DC motors */
typedef enum {
  PARALLELING_NONE___1_BIDIR_MOTOR_BRIDGE_A__1_BIDIR_MOTOR_BRIDGE_B = 0,
  PARALLELING_NONE___1_BIDIR_MOTOR_BRIDGE_A__2_UNDIR_MOTOR_BRIDGE_B = 1,
  PARALLELING_NONE___2_UNDIR_MOTOR_BRIDGE_A__1_BIDIR_MOTOR_BRIDGE_B = 2,
  PARALLELING_NONE___2_UNDIR_MOTOR_BRIDGE_A__2_UNDIR_MOTOR_BRIDGE_B = 3,
  PARALLELING_IN1A_IN2A__1_UNDIR_MOTOR_BRIDGE_A__1_BIDIR_MOTOR_BRIDGE_B = 4,
  PARALLELING_IN1A_IN2A__1_UNDIR_MOTOR_BRIDGE_A__2_UNDIR_MOTOR_BRIDGE_B = 5,
  PARALLELING_IN1B_IN2B__1_BIDIR_MOTOR_BRIDGE_A__1_UNDIR_MOTOR_BRIDGE_B = 6,
  PARALLELING_IN1B_IN2B__2_UNDIR_MOTOR_BRIDGE_A__1_UNDIR_MOTOR_BRIDGE_B = 7,
  PARALLELING_IN1A_IN2A__IN1B_IN2B__1_UNDIR_MOTOR_BRIDGE_A__1_UNDIR_MOTOR_BRIDGE_B = 8,
  PARALLELING_IN1A_IN2A__IN1B_IN2B__1_BIDIR_MOTOR = 9,
  PARALLELING_IN1A_IN1B__IN2A_IN2B__1_UNDIR_MOTOR_BRIDGE_1A__1_UNDIR_MOTOR_BRIDGE_2A = 10,
  PARALLELING_IN1A_IN1B__IN2A_IN2B__1_BIDIR_MOTOR = 11,
  PARALLELING_ALL_WITH_IN1A___1_UNDIR_MOTOR = 12,
  PARALLELING_END_ENUM = 13 
} dualFullBridgeConfig_t;


#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H */

 /***************** (C) COPYRIGHT HEXABITZ ***** END OF FILE ****************/
