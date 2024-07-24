/*
 * LSM303AGR_APIs.h
 * Description: LSM6DS3TR-C Accelerometer and magnetometer unit APIs driver header file.
 *  Created on: Jul 24, 2024
 *      Author: Adel Faki @ Hexabitz
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 Hexabitz.
 * All rights reserved.
 *
 ******************************************************************************
 */
#ifndef LSM303AGR_APIS_H_
#define LSM303AGR_APIS_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "LSM303AGR_MAG.h"
#include "LSM303AGR_ACC.h"
#include "H0BR4_i2c.h"

/* Exported macros -----------------------------------------------------------*/
#define LSM303AGR_MAG_SENSITIVITY_FOR_FS_50G  1.5  /**< Sensitivity value for 16 gauss full scale [mgauss/LSB] */
#define I2CHandle                   hi2c2

/* Exported types ------------------------------------------------------------*/
typedef enum {
	LSM303AGR_OK =0,
	LSM303AGR_ERR,
} LSM303AGR_Status;

/* Exported functions  ---------------------------------------------*/

LSM303AGR_Status LSM303MagInit(void);
LSM303AGR_Status LSM303SampleMagMGauss(int *magX,int *magY,int *magZ);
LSM303AGR_Status LSM303SampleMagRaw(int16_t *magX,int16_t *magY,int16_t *magZ);


#endif /* LSM303AGR_APIS_H_ */
