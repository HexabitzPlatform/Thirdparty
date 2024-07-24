/*
 * LSM303AGR_APIs.C
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

#include "LSM303AGR_APIS.h"




/* Platform Exported Functions ********************************************/
uint8_t LSM303AGR_ACC_I2C_Write(void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite);
uint8_t LSM303AGR_ACC_I2C_Read(void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead);

/* Local functions prototypes ********************************************/
LSM303AGR_Status LSM303MagEnable(void);

/**************************************************************************/
/* Platform Exported Functions ********************************************/
/**************************************************************************/
uint8_t LSM303AGR_ACC_I2C_Write(void *handle,uint8_t WriteAddr,uint8_t *pBuffer,uint16_t nBytesToWrite){
	if(HAL_I2C_Mem_Write(handle,LSM303AGR_ACC_I2C_ADDRESS,WriteAddr,sizeof(WriteAddr),pBuffer,nBytesToWrite,100) != HAL_OK){
		return 1;
	}
	return 0;
}

/**********************************************************************/

uint8_t LSM303AGR_ACC_I2C_Read(void *handle,uint8_t ReadAddr,uint8_t *pBuffer,uint16_t nBytesToRead){
	if(HAL_I2C_Mem_Read(handle,LSM303AGR_ACC_I2C_ADDRESS,ReadAddr,sizeof(ReadAddr),pBuffer,nBytesToRead,100) != HAL_OK){
		return 1;
	}
	return 0;
}

/**********************************************************************/

uint8_t LSM303AGR_MAG_I2C_Write(void *handle,uint8_t WriteAddr,uint8_t *pBuffer,uint16_t nBytesToWrite){
	if(HAL_I2C_Mem_Write(handle,LSM303AGR_MAG_I2C_ADDRESS,WriteAddr,sizeof(WriteAddr),pBuffer,nBytesToWrite,100) != HAL_OK){
		return 1;
	}
	return 0;
}

/**********************************************************************/

uint8_t LSM303AGR_MAG_I2C_Read(void *handle,uint8_t ReadAddr,uint8_t *pBuffer,uint16_t nBytesToRead){
	if(HAL_I2C_Mem_Read(handle,LSM303AGR_MAG_I2C_ADDRESS,ReadAddr,sizeof(ReadAddr),pBuffer,nBytesToRead,100) != HAL_OK){
		return 1;
	}
	return 0;
}

/**************************************************************************/
/* Local Functions  *******************************************************/
/**************************************************************************/

LSM303AGR_Status LSM303MagEnable(void){
	if(LSM303AGR_MAG_W_MD(&I2CHandle,LSM303AGR_MAG_MD_CONTINUOS_MODE) != LSM303AGR_OK)
		return LSM303AGR_ERR;

	return LSM303AGR_OK;
}


/**************************************************************************/
/* Exported functions  ****************************************************/
/**************************************************************************/

LSM303AGR_Status LSM303MagInit(void){

	// Check the Sensor
	uint8_t who_am_i =0x00;

	if(LSM303AGR_MAG_R_WHO_AM_I(&hi2c2,&who_am_i) != LSM303AGR_OK)
		return LSM303AGR_ERR;

	if(who_am_i != LSM303AGR_MAG_WHO_AM_I)
		return LSM303AGR_ERR;

	// Operating Mode: Power Down
	if(LSM303AGR_MAG_W_MD(&I2CHandle,LSM303AGR_MAG_MD_IDLE1_MODE) != LSM303AGR_OK)
		return LSM303AGR_ERR;

	// Enable Block Data Update
	if(LSM303AGR_MAG_W_BDU(&I2CHandle,LSM303AGR_MAG_BDU_ENABLED) != LSM303AGR_OK)
		return LSM303AGR_ERR;

	// TODO: Change the default ODR
	if(LSM303AGR_MAG_W_ODR(&I2CHandle,LSM303AGR_MAG_ODR_10Hz) != LSM303AGR_OK)
		return LSM303AGR_ERR;

	// Self Test Disabled
	if(LSM303AGR_MAG_W_ST(&I2CHandle,LSM303AGR_MAG_ST_DISABLED) != LSM303AGR_OK)
		return LSM303AGR_ERR;

	return LSM303MagEnable();
}

/*-----------------------------------------------------------*/

LSM303AGR_Status LSM303SampleMagRaw(int16_t *magX,int16_t *magY,int16_t *magZ){
	int16_t *pData;
	uint8_t data[6];

//	memset(data,0,sizeof(data));

	if(LSM303AGR_MAG_Get_Raw_Magnetic(&I2CHandle,data) != LSM303AGR_OK)
		return LSM303AGR_ERR;

	pData =(int16_t* )data;
	*magX =pData[0];
	*magY =pData[1];
	*magZ =pData[2];

	return LSM303AGR_OK;
}

/*-----------------------------------------------------------*/

LSM303AGR_Status LSM303SampleMagMGauss(int *magX,int *magY,int *magZ){

	int16_t rawMagX, rawMagY, rawMagZ;

	/* Read raw data from LSM303AGR output register. */
	if((LSM303SampleMagRaw(&rawMagX,&rawMagY,&rawMagZ)) != LSM303AGR_OK)
		return LSM303AGR_ERR;

	/* Set the raw data. */
	*magX =rawMagX * (float )LSM303AGR_MAG_SENSITIVITY_FOR_FS_50G;
	*magY =rawMagY * (float )LSM303AGR_MAG_SENSITIVITY_FOR_FS_50G;
	*magZ =rawMagZ * (float )LSM303AGR_MAG_SENSITIVITY_FOR_FS_50G;
	return LSM303AGR_OK;
}

/*-----------------------------------------------------------*/
