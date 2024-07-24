/*
 * LSM6DS3TR_C_APIs.h
 * Description: LSM6DS3TR-C Accelerometer and Gyroscope unit APIs driver source file.
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

#include "LSM6DS3TR_C_APIS.h"

/* Exported Type's instance  ---------------------------------------------*/
stmdev_ctx_t dev_ctx;

/* Platform Exported Functions ********************************************/
int32_t I2Cwrite(void *handle, uint8_t reg, const uint8_t *bufp,uint16_t len);
int32_t I2Cread(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
void delay(uint32_t ms);


/* Local functions prototypes ********************************************/
LSM6DS3TR_C_Status LSM6DS3TR_C_Enable(void);
LSM6DS3TR_C_Status LSM6DS3TR_C_SetupGyro(void);
LSM6DS3TR_C_Status LSM6DS3TR_C_SetupAcc(void);


/**************************************************************************/
/* Platform Exported Functions ********************************************/
/**************************************************************************/
int32_t WriteI2C(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len) {

	HAL_I2C_Mem_Write(handle, LSM6DS3TR_C_I2C_ADD_L, reg, sizeof(reg),(uint8_t*) bufp, len, 100);

	return 0;
}

/**********************************************************************/

int32_t ReadI2C(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len){

	  HAL_I2C_Mem_Read(handle, LSM6DS3TR_C_I2C_ADD_L, reg, sizeof(reg), bufp, len, 100);

    return 0;
}

/**********************************************************************/

void delay(uint32_t ms){

	HAL_Delay(ms);
}

/**************************************************************************/
/* Local Functions  *******************************************************/
/**************************************************************************/

LSM6DS3TR_C_Status LSM6DS3TR_C_Enable(void) {

	uint8_t who_am_i = 0;

	/* Check WHO_AM_I Register */
	if (lsm6ds3tr_c_device_id_get(&dev_ctx, &who_am_i) != LSM6DS3TR_C_OK)
		return LSM6DS3TR_C_ERR;

	if (who_am_i != LSM6DS3TR_C_ID)
		return LSM6DS3TR_C_ERR;

//	lsm6ds3tr_c_reset_set(&dev_ctx, 1);

	return LSM6DS3TR_C_OK;
}

/**********************************************************************/

LSM6DS3TR_C_Status LSM6DS3TR_C_SetupGyro(void) {

	/* Gyroscope ODR Init */
	if (lsm6ds3tr_c_gy_data_rate_set(&dev_ctx, LSM6DS3TR_C_GY_ODR_12Hz5) != LSM6DS3TR_C_OK)
		return LSM6DS3TR_C_ERR;

	/* Gyroscope FS Init */
	if (lsm6ds3tr_c_gy_full_scale_set(&dev_ctx, LSM6DS3TR_C_2000dps) != LSM6DS3TR_C_OK)
		return LSM6DS3TR_C_ERR;

	/* Gyroscope - filtering chain */
	/* when initializing the low pass filter of the gyroscope
	 * the sensitivity is getting low while the noise resistance is
	 * getting high.
	 * so: it depends on the application
	 * keep in mind: when initializing the low pass filter of the gyroscope
	 * in order to de-initialze it you need to reset the the IC configuration by
	 * un-commenting "lsm6ds3tr_c_reset_set' function in "LSM6DS3TR_C_Enable" function  */
//	if (lsm6ds3tr_c_gy_band_pass_set(&dev_ctx, LSM6DS3TR_C_HP_16mHz_LP1_LIGHT) != LSM6DS3TR_C_OK)
//		return LSM6DS3TR_C_ERR;

	return LSM6DS3TR_C_OK;
}

/**********************************************************************/

LSM6DS3TR_C_Status LSM6DS3TR_C_SetupAcc(void) {

	/* Accelerometer ODR Init */
	if (lsm6ds3tr_c_xl_data_rate_set(&dev_ctx, LSM6DS3TR_C_XL_ODR_12Hz5) != LSM6DS3TR_C_OK)
		return LSM6DS3TR_C_ERR;

	/* Accelerometer FS Init */
	if (lsm6ds3tr_c_xl_full_scale_set(&dev_ctx, LSM6DS3TR_C_2g) != LSM6DS3TR_C_OK)
		return LSM6DS3TR_C_ERR;

	/* Accelerometer - analog filter */
	if (lsm6ds3tr_c_xl_filter_analog_set(&dev_ctx, LSM6DS3TR_C_XL_ANA_BW_400Hz) != LSM6DS3TR_C_OK)
		return LSM6DS3TR_C_ERR;

	/* Accelerometer - LPF1 path ( LPF2 not used )*/
	if (lsm6ds3tr_c_xl_lp1_bandwidth_set(&dev_ctx, LSM6DS3TR_C_XL_LP1_ODR_DIV_4) != LSM6DS3TR_C_OK)
		return LSM6DS3TR_C_ERR;

	/* Accelerometer - LPF1 + LPF2 path */
	if (lsm6ds3tr_c_xl_lp2_bandwidth_set(&dev_ctx, LSM6DS3TR_C_XL_LOW_NOISE_LP_ODR_DIV_100) != LSM6DS3TR_C_OK)
		return LSM6DS3TR_C_ERR;

	return LSM6DS3TR_C_OK;

}

/**************************************************************************/
/* Exported functions  ****************************************************/
/**************************************************************************/

LSM6DS3TR_C_Status LSM6DS3TR_C_Init(void) {

	dev_ctx.write_reg = WriteI2C;
	dev_ctx.read_reg = ReadI2C;
	dev_ctx.mdelay = delay;
	dev_ctx.handle = &I2CHandle;

	if (( LSM6DS3TR_C_Enable()) != LSM6DS3TR_C_OK)
		return LSM6DS3TR_C_ERR;

	if (( LSM6DS3TR_C_SetupGyro()) != LSM6DS3TR_C_OK)
		return LSM6DS3TR_C_ERR;

	if (( LSM6DS3TR_C_SetupAcc()) != LSM6DS3TR_C_OK)
		return LSM6DS3TR_C_ERR;

	return LSM6DS3TR_C_OK;
}

/**********************************************************************/

LSM6DS3TR_C_Status LSM6DS3TR_C_SampleGyroRaw(int16_t *gyroX, int16_t *gyroY, int16_t *gyroZ) {
	int16_t buff[3];

	if (lsm6ds3tr_c_angular_rate_raw_get(&dev_ctx, buff) != LSM6DS3TR_C_OK)
		return LSM6DS3TR_C_ERR;

	*gyroX = buff[0];
	*gyroY = buff[1];
	*gyroZ = buff[2];

	return LSM6DS3TR_C_OK;
}

/**********************************************************************/

LSM6DS3TR_C_Status LSM6DS3TR_C_SampleGyroDPS(float *gyroX, float *gyroY, float *gyroZ) {
	uint8_t GyroFullScale;
	int16_t buff[3];

	if (lsm6ds3tr_c_angular_rate_raw_get(&dev_ctx, buff) != LSM6DS3TR_C_OK)
		return LSM6DS3TR_C_ERR;

	if (lsm6ds3tr_c_gy_full_scale_get(&dev_ctx, &GyroFullScale) != LSM6DS3TR_C_OK)
		return LSM6DS3TR_C_ERR;

	switch (GyroFullScale) {
	case LSM6DS3TR_C_250dps:
		*gyroX = lsm6ds3tr_c_from_fs250dps_to_mdps(buff[0]) / 1000;
		*gyroY = lsm6ds3tr_c_from_fs250dps_to_mdps(buff[1]) / 1000;
		*gyroZ = lsm6ds3tr_c_from_fs250dps_to_mdps(buff[2]) / 1000;
		break;

	case LSM6DS3TR_C_125dps:
		*gyroX = lsm6ds3tr_c_from_fs125dps_to_mdps(buff[0]) / 1000;
		*gyroY = lsm6ds3tr_c_from_fs125dps_to_mdps(buff[1]) / 1000;
		*gyroZ = lsm6ds3tr_c_from_fs125dps_to_mdps(buff[2]) / 1000;
		break;

	case LSM6DS3TR_C_500dps:
		*gyroX = lsm6ds3tr_c_from_fs500dps_to_mdps(buff[0]) / 1000;
		*gyroY = lsm6ds3tr_c_from_fs500dps_to_mdps(buff[1]) / 1000;
		*gyroZ = lsm6ds3tr_c_from_fs500dps_to_mdps(buff[2]) / 1000;
		break;

	case LSM6DS3TR_C_1000dps:
		*gyroX = lsm6ds3tr_c_from_fs1000dps_to_mdps(buff[0]) / 1000;
		*gyroY = lsm6ds3tr_c_from_fs1000dps_to_mdps(buff[1]) / 1000;
		*gyroZ = lsm6ds3tr_c_from_fs1000dps_to_mdps(buff[2]) / 1000;
		break;

	case LSM6DS3TR_C_2000dps:
		*gyroX = lsm6ds3tr_c_from_fs2000dps_to_mdps(buff[0]) / 1000;
		*gyroY = lsm6ds3tr_c_from_fs2000dps_to_mdps(buff[1]) / 1000;
		*gyroZ = lsm6ds3tr_c_from_fs2000dps_to_mdps(buff[2]) / 1000;
		break;

	}

	return LSM6DS3TR_C_OK;
}

/**********************************************************************/

LSM6DS3TR_C_Status LSM6DS3TR_C_SampleAccRaw(int16_t *accX,int16_t *accY,int16_t *accZ){

	int16_t buff[3];

	if (lsm6ds3tr_c_acceleration_raw_get(&dev_ctx, buff) != LSM6DS3TR_C_OK)
		return LSM6DS3TR_C_ERR;

	*accX = buff[0];
	*accY = buff[1];
	*accZ = buff[2];

	return LSM6DS3TR_C_OK;
 }

/**********************************************************************/

LSM6DS3TR_C_Status LSM6DS3TR_C_SampleAccG(float *accX,float *accY,float *accZ){
	uint8_t AccFullScale;
	int16_t buff[3];

	if (lsm6ds3tr_c_acceleration_raw_get(&dev_ctx, buff) != LSM6DS3TR_C_OK)
		return LSM6DS3TR_C_ERR;

	if (lsm6ds3tr_c_xl_full_scale_get(&dev_ctx,&AccFullScale) != LSM6DS3TR_C_OK)
		return LSM6DS3TR_C_ERR;

	switch (AccFullScale) {
	case LSM6DS3TR_C_2g:
		*accX = lsm6ds3tr_c_from_fs2g_to_mg(buff[0]) / 1000;
		*accY = lsm6ds3tr_c_from_fs2g_to_mg(buff[1]) / 1000;
		*accZ = lsm6ds3tr_c_from_fs2g_to_mg(buff[2]) / 1000;
		break;

	case LSM6DS3TR_C_16g:
		*accX = lsm6ds3tr_c_from_fs16g_to_mg(buff[0]) / 1000;
		*accY = lsm6ds3tr_c_from_fs16g_to_mg(buff[1]) / 1000;
		*accZ = lsm6ds3tr_c_from_fs16g_to_mg(buff[2]) / 1000;
		break;

	case LSM6DS3TR_C_4g:
		*accX = lsm6ds3tr_c_from_fs4g_to_mg(buff[0]) / 1000;
		*accY = lsm6ds3tr_c_from_fs4g_to_mg(buff[1]) / 1000;
		*accZ = lsm6ds3tr_c_from_fs4g_to_mg(buff[2]) / 1000;
		break;

	case LSM6DS3TR_C_8g:
		*accX = lsm6ds3tr_c_from_fs8g_to_mg(buff[0]) / 1000;
		*accY = lsm6ds3tr_c_from_fs8g_to_mg(buff[1]) / 1000;
		*accZ = lsm6ds3tr_c_from_fs8g_to_mg(buff[2]) / 1000;
		break;

	}

	return LSM6DS3TR_C_OK;
}

/**********************************************************************/

LSM6DS3TR_C_Status LSM6DS3TR_C_SampleTempCelsius(float *temp) {

	int16_t buff;

	if (lsm6ds3tr_c_temperature_raw_get(&dev_ctx, &buff) != LSM6DS3TR_C_OK)
		return LSM6DS3TR_C_ERR;

	*temp = lsm6ds3tr_c_from_lsb_to_celsius(buff);

	return LSM6DS3TR_C_OK;
}

/**********************************************************************/

LSM6DS3TR_C_Status LSM6DS3TR_C_SampleTempFahrenheit(float *temp) {

	float celsius = 0.0f;

	if (LSM6DS3TR_C_SampleTempCelsius(&celsius) != LSM6DS3TR_C_OK)
		return LSM6DS3TR_C_ERR;

	*temp = celsiusToFahrenheit(celsius);

	return LSM6DS3TR_C_OK;
}

/**********************************************************************/
/************************ (C) COPYRIGHT Hexabitz *****END OF FILE****/
