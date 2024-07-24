/*
 * LSM6DS3TR_C_APIs.h
 * Description: LSM6DS3TR-C Accelerometer and Gyroscope unit APIs driver header file.
 *  Created on: Jul 23, 2024
 *      Author: Adel Faki @ Hexabitz
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 Hexabitz.
 * All rights reserved.
 *
 ******************************************************************************
 */
#ifndef LSM6DS3TR_C_APIS_H_
#define LSM6DS3TR_C_APIS_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "lsm6ds3tr-c_reg.h"
#include "H0BR4_i2c.h"
//#include "i2c.h"

/* Exported macros -----------------------------------------------------------*/
#define I2CHandle                   hi2c2
#define celsiusToFahrenheit(c)		(((c * 9.0) / 5) + 32)

/* Exported types ------------------------------------------------------------*/
typedef enum {
	LSM6DS3TR_C_OK = 0,
	LSM6DS3TR_C_ERR,
}LSM6DS3TR_C_Status;


/* Exported functions  ---------------------------------------------*/
LSM6DS3TR_C_Status LSM6DS3TR_C_Init(void);

LSM6DS3TR_C_Status LSM6DS3TR_C_SampleGyroDPS(float *gyroX, float *gyroY, float *gyroZ);
LSM6DS3TR_C_Status LSM6DS3TR_C_SampleGyroRaw(int16_t *gyroX, int16_t *gyroY, int16_t *gyroZ);

LSM6DS3TR_C_Status LSM6DS3TR_C_SampleAccG(float *accX, float *accY, float *accZ);
LSM6DS3TR_C_Status LSM6DS3TR_C_SampleAccRaw(int16_t *accX, int16_t *accY, int16_t *accZ);

LSM6DS3TR_C_Status LSM6DS3TR_C_SampleTempCelsius(float *temp);
LSM6DS3TR_C_Status LSM6DS3TR_C_SampleTempFahrenheit(float *temp);

#endif /* LSM6DS3TR_C_APIS_H_ */
/************************ (C) COPYRIGHT Hexabitz *****END OF FILE****/
