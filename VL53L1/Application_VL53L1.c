/*
 * Bracelet_IR_ToF.h
 *  Description: Library file for ToF IR sensor
 *  Created on: Month 16, 8, 2021
 *      Author: Abd Alrhman Hammal @ Hexabitz
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2021 Hexabitz.
 * All rights reserved.
 *
 ******************************************************************************
 */

#include "Application_VL53L1.h"
int status;
volatile int IntCount_ToF;
/* ---------------------------------------------------------------------
 |							 Bracelet  APIs	 					        |
 -----------------------------------------------------------------------
 */
Status_TypeDef SetGPIOsPin(GPIO_HANDLE *GPIOx, uint16_t Pin)
{
	Status_TypeDef Status;

	if (NULL!=GPIOx)
	{
		HAL_GPIO_WritePin(GPIOx, Pin, GPIO_PIN_SET);
		Status=STATUS_OK;
	}
	else
		Status=STATUS_ERR;

	return Status;
}

/*
 * set a GPIO pin form a port low
 */
Status_TypeDef ResetGPIOsPin(GPIO_HANDLE *GPIOx, uint16_t Pin)
{
	Status_TypeDef Status;

	if (NULL!=GPIOx)
	{
		HAL_GPIO_WritePin(GPIOx, Pin, GPIO_PIN_RESET);
		Status=STATUS_OK;
	}
	else
		Status=STATUS_ERR;

	return Status;
}


Status_TypeDef tofModeMeasurement(VL53L1_DEV Dev, VL53L1_PresetModes PresetMode,
		VL53L1_DistanceModes DistanceMode, VL53L1_InterruptMode InterruptMode,
		dynamicZone_s userDynamicZone_s, ToF_Structure *ToFStructure) {

	Status_TypeDef Status;
	VL53L1_MultiRangingData_t MultiRangingData;
	VL53L1_MultiRangingData_t *pMultiRangingData = &MultiRangingData;
	static uint8_t NewDataReady = 0;
	uint32_t MeasurementTimingBudgetMicroSeconds;
	uint8_t SeqEnabled;
	int no_of_object_found = 0, j;
	int16_t Range = 5000;
	uint8_t proxiStatus = 1;
	static uint8_t firstTimeInit = 1;
	static VL53L1_PresetModes oldPresetMode = 0;
	/* Must IRsensorInit() call before tofModeMeasurement(...)  */
	/* VL53L1_SetPresetMode function is mandatory to be called even if default PresetMode is the VL53L1_PRESETMODE_RANGING */
	if (oldPresetMode != PresetMode) {
		status = VL53L1_StopMeasurement(Dev);
		status = VL53L1_SetPresetMode(Dev, PresetMode);
		//status = VL53L1_SetOutputMode(Dev, VL53L1_OUTPUTMODE_STRONGEST);/*no need/*
		status = VL53L1_SetDistanceMode(Dev, DistanceMode);
		if (PresetMode != VL53L1_PRESETMODE_MULTIZONES_SCANNING) {
			status = ROIExample(Dev);
		}

		switch (PresetMode) {

		case VL53L1_PRESETMODE_RANGING:
			if (status == VL53L1_ERROR_NONE) {
				status = TimingBudgetExample(Dev);
				status = VL53L1_SetMeasurementTimingBudgetMicroSeconds(Dev,
						45000);
			}
			break;
		case VL53L1_PRESETMODE_AUTONOMOUS:
		case VL53L1_PRESETMODE_LOWPOWER_AUTONOMOUS:
			if (status == VL53L1_ERROR_NONE) {
				status = VL53L1_SetInterMeasurementPeriodMilliSeconds(Dev, 500);
			}
			if (status == VL53L1_ERROR_NONE) {
				status = VL53L1_GetMeasurementTimingBudgetMicroSeconds(Dev,
						&MeasurementTimingBudgetMicroSeconds);
			}
			if (status == VL53L1_ERROR_NONE) {
				status = VL53L1_SetMeasurementTimingBudgetMicroSeconds(Dev,
						45000);
				status = VL53L1_GetMeasurementTimingBudgetMicroSeconds(Dev,
						&MeasurementTimingBudgetMicroSeconds);
			}

			status = VL53L1_SetSequenceStepEnable(Dev,
			VL53L1_SEQUENCESTEP_MM1, 0);
			status = VL53L1_SetSequenceStepEnable(Dev,
			VL53L1_SEQUENCESTEP_MM2, 0);
			/* get all step flags
			 */
			for (int i = 0; i < VL53L1_SEQUENCESTEP_NUMBER_OF_ITEMS; i++) {
				status = VL53L1_GetSequenceStepEnable(Dev, i, &SeqEnabled);
			}

			break;

		case VL53L1_PRESETMODE_PROXY_RANGING_MODE:

			if (status == VL53L1_ERROR_NONE) {
				status = VL53L1_SetMeasurementTimingBudgetMicroSeconds(Dev,
						16000);
			}
			if (status == VL53L1_ERROR_NONE) {
				status = VL53L1_SetInterMeasurementPeriodMilliSeconds(Dev, 16);
			}
			break;

		case VL53L1_PRESETMODE_MULTIZONES_SCANNING:

			if (status == VL53L1_ERROR_NONE)
				status = MultiROIExample(Dev);

			if (status == VL53L1_ERROR_NONE)
				status = TimingBudgetExample(Dev);
			break;

		}

		status = VL53L1_StartMeasurement(Dev);
		HAL_Delay(1);
		/*********************************/
		/* Very first ranging measurement completion interrupt must be ignored */
		ss: status = VL53L1_GetMeasurementDataReady(Dev, &NewDataReady);
		if ((!status) && (NewDataReady != 0)) {
			//status = VL53L1_GetMultiRangingData(Dev, pMultiRangingData);
		} else {
			HAL_Delay(2);
			goto ss;
		}
		oldPresetMode = PresetMode;
		status = VL53L1_ClearInterruptAndStartMeasurement(Dev);
	}

	/***************MULTIZONE OR RANGING Mode Dynamic Zones***************/

	uint8_t dynamicRangingZone_v = userDynamicZone_s.dynamicRangingZone_user;
	uint8_t dynamicMultiZone_v = userDynamicZone_s.dynamicMultiZone_user;
	static uint8_t endZone = 0;

	if (dynamicRangingZone_v
			== DYNAMIC_ZONE_ON&& PresetMode==VL53L1_PRESETMODE_RANGING) {
		if (STATUS_OK != ROIDynamicRangingZone(Dev, ToFStructure))
			return STATUS_ERR;

		if (STATUS_OK != VL53L1_StartMeasurement(Dev))
			return STATUS_ERR;

		//status = VL53L1_WaitMeasurementDataReady(Dev);
		NewDataReady = 0;
		while (NewDataReady == 0)
			VL53L1_GetMeasurementDataReady(Dev, &NewDataReady);

		if ((!status) && (NewDataReady != 0))
			status = VL53L1_ClearInterruptAndStartMeasurement(Dev);
	}

	if (dynamicMultiZone_v == DYNAMIC_MZONE_ON
			&& PresetMode == VL53L1_PRESETMODE_MULTIZONES_SCANNING
			&& endZone == 0)/*must not change zones until complete all regions*/
			{

		if (STATUS_OK != ROIDynamicMultiZone(Dev, ToFStructure))
			return STATUS_ERR;

		if (STATUS_OK != VL53L1_StartMeasurement(Dev))
			return STATUS_ERR;

		//status = VL53L1_WaitMeasurementDataReady(Dev);
		NewDataReady = 0;
		while (NewDataReady == 0)
			VL53L1_GetMeasurementDataReady(Dev, &NewDataReady);

		if ((!status) && (NewDataReady != 0))
			status = VL53L1_ClearInterruptAndStartMeasurement(Dev);
	}

	/* Starting measuring loop  */

	if (InterruptMode) { /* Interrupt mode */
		_WAITFORINT();
		if (IntCount_ToF != 0) {
			IntCount_ToF = 0;
			/*returns values for only one region yet next calling will change the region automatically*/
			if (firstTimeInit == 1) {
				HAL_Delay(100);
				firstTimeInit = 0;
			}
			status = VL53L1_GetMultiRangingData(Dev, pMultiRangingData);
			no_of_object_found = pMultiRangingData->NumberOfObjectsFound;
			if (no_of_object_found <= 0)
				no_of_object_found = 0;
			if (no_of_object_found > 4)
				no_of_object_found = MAX_NUMBER_OBJECT;
			ToFStructure->tofNumberofObject = no_of_object_found;
			ToFStructure->tofStreamCount = pMultiRangingData->StreamCount;
			for (j = 0; j < no_of_object_found; j++) {
				ToFStructure->ObjectNumber[j].tofTargetNumber = j + 1;
				ToFStructure->ObjectNumber[j].tofROINumber =
						pMultiRangingData->RoiNumber;
				ToFStructure->ObjectNumber[j].tofRoiStatus =
						pMultiRangingData->RoiStatus;
				ToFStructure->ObjectNumber[j].tofState =
						pMultiRangingData->RangeData[j].RangeStatus;
				ToFStructure->ObjectNumber[j].tofDistanceMm =
						pMultiRangingData->RangeData[j].RangeMilliMeter;
				ToFStructure->ObjectNumber[j].tofReflectanceTarget =
						pMultiRangingData->RangeData[j].SignalRateRtnMegaCps
								/ 65536.0;
				ToFStructure->ObjectNumber[j].tofLightAmbient =
						pMultiRangingData->RangeData[j].AmbientRateRtnMegaCps
								/ 65536.0;
				ToFStructure->ObjectNumber[j].tofRangeMinMilliMeter =
						pMultiRangingData->RangeData[j].RangeMinMilliMeter;
				ToFStructure->ObjectNumber[j].tofRangeMaxMilliMeter =
						pMultiRangingData->RangeData[j].RangeMaxMilliMeter;

				ToFStructure->currentZoneScan = pMultiRangingData->RoiNumber;
				/**********for proximity **************/
				if (VL53L1_PRESETMODE_PROXY_RANGING_MODE == PresetMode) {

					int16_t Range = 5000; /* arbitrary set no object range value to 5 meters */
					uint8_t ValidRange = 0, RngSta = 0;

					/* by default VL53L1_GetMultiRangingData gives objects from the nearest to the farest */
					RngSta = MultiRangingData.RangeData[j].RangeStatus;
					ValidRange =
							(RngSta
									== VL53L1_RANGESTATUS_TARGET_PRESENT_LACK_OF_SIGNAL);
					ValidRange = ValidRange
							|| (RngSta == VL53L1_RANGESTATUS_RANGE_VALID);
					if (ValidRange) {
						Range = MultiRangingData.RangeData[j].RangeMilliMeter;
					}

					if (ValidRange) {
						/* We found one object, let's check its distance*/
						if (Range >= LowThres && Range < HighThres)
							ToFStructure->ObjectNumber[j].objectInProximity = 1;
						if (Range > HighThres)
							ToFStructure->ObjectNumber[j].objectInProximity = 0;
					} else
						ToFStructure->ObjectNumber[j].objectInProximity = 0;
				}

				/****************************/
			}
//				if (status == 0) {
//					status = VL53L1_ClearInterruptAndStartMeasurement(Dev);
//				}
		}
	}

	else { /* pulling mode  */
		NewDataReady = 0;
		retryForDataReadyLable: status = VL53L1_GetMeasurementDataReady(Dev,
				&NewDataReady);
		HAL_Delay(1);

		if ((!status) && (NewDataReady != 0)) {
			NewDataReady = 0;
			if (firstTimeInit == 1) {
				HAL_Delay(100);
				firstTimeInit = 0;
			}
			status = VL53L1_GetMultiRangingData(Dev, pMultiRangingData);
			no_of_object_found = pMultiRangingData->NumberOfObjectsFound;
			if (no_of_object_found > 4)
				no_of_object_found = MAX_NUMBER_OBJECT;
			ToFStructure->tofNumberofObject = no_of_object_found;
			ToFStructure->tofStreamCount = pMultiRangingData->StreamCount;
			for (j = 0; j < no_of_object_found; j++) {
				ToFStructure->ObjectNumber[j].tofTargetNumber = j + 1;
				ToFStructure->ObjectNumber[j].tofROINumber =
						pMultiRangingData->RoiNumber;
				ToFStructure->ObjectNumber[j].tofRoiStatus =
						pMultiRangingData->RoiStatus;
				ToFStructure->ObjectNumber[j].tofState =
						pMultiRangingData->RangeData[j].RangeStatus;
				ToFStructure->ObjectNumber[j].tofDistanceMm =
						pMultiRangingData->RangeData[j].RangeMilliMeter;
				ToFStructure->ObjectNumber[j].tofReflectanceTarget =
						pMultiRangingData->RangeData[j].SignalRateRtnMegaCps
								/ 65536.0;
				ToFStructure->ObjectNumber[j].tofLightAmbient =
						pMultiRangingData->RangeData[j].AmbientRateRtnMegaCps
								/ 65536.0;
				ToFStructure->ObjectNumber[j].tofRangeMinMilliMeter =
						pMultiRangingData->RangeData[j].RangeMinMilliMeter;
				ToFStructure->ObjectNumber[j].tofRangeMaxMilliMeter =
						pMultiRangingData->RangeData[j].RangeMaxMilliMeter;
				ToFStructure->currentZoneScan = pMultiRangingData->RoiNumber;
				/**********for proximity **************/
				if (VL53L1_PRESETMODE_PROXY_RANGING_MODE == PresetMode) {

					Range = MultiRangingData.RangeData[j].RangeMilliMeter;
					proxiStatus = pMultiRangingData->RangeData[j].RangeStatus;
					if (Range >= LowThres && Range < HighThres)
						ToFStructure->ObjectNumber[j].objectInProximity = 1;
					else if (Range > HighThres)
						ToFStructure->ObjectNumber[j].objectInProximity = 0;
					if (proxiStatus)
						ToFStructure->ObjectNumber[j].objectInProximity = 0;
				}

			}

		} else if ((!status) && (NewDataReady == 0))
			goto retryForDataReadyLable;
	}

	if (status == 0) {
		if (pMultiRangingData->RoiNumber
				< (ToFStructure->userToFRoiConfig.NumberOfRoi - 1)) {
			endZone = 1;
		} else {
			endZone = 0;
		}

	}
	status = VL53L1_ClearInterruptAndStartMeasurement(Dev);
	HAL_Delay(2);
	if (status == VL53L1_ERROR_NONE)
		Status = STATUS_OK;
	else
		Status = STATUS_ERR;
	return Status;
}

/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

Status_TypeDef IRSensorInit(VL53L1_DEV Dev) {

	Dev->I2cHandle = &HANDLER_ToF_I2C;
	Dev->I2cDevAddr = ToF_SENSOR_I2C_ADDRESS;

	ResetGPIOsPin(TOF_XSHUT_GPIO_Port, TOF_XSHUT_Pin);
	HAL_Delay(2);
	SetGPIOsPin(TOF_XSHUT_GPIO_Port, TOF_XSHUT_Pin);
    HAL_Delay(2);
	if (VL53L1_ERROR_NONE != VL53L1_WaitDeviceBooted(Dev))
		return STATUS_ERR;
	if (VL53L1_ERROR_NONE != VL53L1_DataInit(Dev))
		return STATUS_ERR;
	if (VL53L1_ERROR_NONE != VL53L1_StaticInit(Dev))
		return STATUS_ERR;

	return STATUS_OK;
}

/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
Status_TypeDef IRSensorCalibration(VL53L1_DEV Dev) {

	VL53L1_Dev_t dev;
	Dev = &dev;
	VL53L1_CalibrationData_t *pCalibrationData = { 0 };
	/*
	 * Run reference SPAD characterization
	 */
	if (VL53L1_ERROR_NONE != VL53L1_PerformRefSpadManagement(Dev))
		return STATUS_ERR;

	/*
	 * Example for standard offset calibration
	 * value 0x50000 for a 5% reflective target (16.16 fixed point format)
	 * The target shall be located at 140 mm from the device
	 */
	if (VL53L1_ERROR_NONE != VL53L1_GetCalibrationData(Dev, pCalibrationData))
		return STATUS_ERR;
	if (VL53L1_ERROR_NONE != VL53L1_SetOffsetCalibrationMode(Dev, VL53L1_OFFSETCALIBRATIONMODE_STANDARD))
		return STATUS_ERR;
	if (VL53L1_ERROR_NONE != VL53L1_SetXTalkCompensationEnable(Dev, 1)) /*enable*/
		return STATUS_ERR;
	if (VL53L1_ERROR_NONE != VL53L1_PerformOffsetCalibration(Dev, 140, 0x50000U)) /*enable*/
		return STATUS_ERR;

	/*
	 * Example for standard crosstalsk calibration
	 * Assuming there is no target lower than 80 cm from the device
	 */

	if (VL53L1_ERROR_NONE != VL53L1_PerformXTalkCalibration(Dev, VL53L1_XTALKCALIBRATIONMODE_NO_TARGET))
		return STATUS_ERR;

	/*
	 * Initialize configuration data structures for the
	 * given preset mode. Does *not* apply the settings
	 * to the device just initializes the data structures
	 */
	if (VL53L1_ERROR_NONE != VL53L1_SetCalibrationData(Dev, pCalibrationData))
		return STATUS_ERR;
	if (VL53L1_ERROR_NONE != VL53L1_SetXTalkCompensationEnable(Dev, 0)) /*disable*/
		return STATUS_ERR;

	return STATUS_OK;
}
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

Status_TypeDef IRSensorUnit() {

	return STATUS_OK;
}

/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
Status_TypeDef IRSensorDeactivate() {
	ResetGPIOsPin(TOF_XSHUT_GPIO_Port, TOF_XSHUT_Pin);
	return STATUS_OK;
}

/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
Status_TypeDef IRSensorActivate() {
	SetGPIOsPin(TOF_XSHUT_GPIO_Port, TOF_XSHUT_Pin);
	return STATUS_OK;
}

/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

VL53L1_Error ROIExample(VL53L1_DEV Dev) {
	VL53L1_Error status = VL53L1_ERROR_NONE;
	uint8_t MaxNumberOfROI;
	VL53L1_RoiConfig_t RoiConfig;

	status = VL53L1_GetMaxNumberOfROI(Dev, &MaxNumberOfROI);

	if (status == VL53L1_ERROR_NONE) {
		RoiConfig.NumberOfRoi = 1;
		RoiConfig.UserRois[0].TopLeftX = 0;
		RoiConfig.UserRois[0].TopLeftY = 15;
		RoiConfig.UserRois[0].BotRightX = 15;
		RoiConfig.UserRois[0].BotRightY = 0;
		status = VL53L1_SetROI(Dev, &RoiConfig);
	}

	return status;

}

/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
VL53L1_Error MultiROIExample(VL53L1_DEV Dev) {
	VL53L1_Error status = VL53L1_ERROR_NONE;
	uint8_t MaxNumberOfROI;
	VL53L1_RoiConfig_t RoiConfig;
	status = VL53L1_GetMaxNumberOfROI(Dev, &MaxNumberOfROI);

	if (status == VL53L1_ERROR_NONE) {
		RoiConfig.NumberOfRoi = 4;
		RoiConfig.UserRois[0].TopLeftX = 0;
		RoiConfig.UserRois[0].TopLeftY = 15;
		RoiConfig.UserRois[0].BotRightX = 7;
		RoiConfig.UserRois[0].BotRightY = 0;

		RoiConfig.UserRois[1].TopLeftX = 7;
		RoiConfig.UserRois[1].TopLeftY = 15;
		RoiConfig.UserRois[1].BotRightX = 15;
		RoiConfig.UserRois[1].BotRightY = 0;

		RoiConfig.UserRois[2].TopLeftX = 6;
		RoiConfig.UserRois[2].TopLeftY = 13;
		RoiConfig.UserRois[2].BotRightX = 13;
		RoiConfig.UserRois[2].BotRightY = 6;

		RoiConfig.UserRois[3].TopLeftX = 0;
		RoiConfig.UserRois[3].TopLeftY = 4;
		RoiConfig.UserRois[3].BotRightX = 4;
		RoiConfig.UserRois[3].BotRightY = 0;
		status = VL53L1_SetROI(Dev, &RoiConfig);
	}

	return status;

}

/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

VL53L1_Error TimingBudgetExample(VL53L1_DEV Dev) {
	VL53L1_Error status = VL53L1_ERROR_NONE;

	uint32_t MeasurementTimingBudgetMicroSeconds = 12000;

	status = VL53L1_GetMeasurementTimingBudgetMicroSeconds(Dev,
			&MeasurementTimingBudgetMicroSeconds);

	if (status == VL53L1_ERROR_NONE) {
		status = VL53L1_SetMeasurementTimingBudgetMicroSeconds(Dev,
				MeasurementTimingBudgetMicroSeconds + 5000);
	}

	if (status == VL53L1_ERROR_NONE) {
		status = VL53L1_GetMeasurementTimingBudgetMicroSeconds(Dev,
				&MeasurementTimingBudgetMicroSeconds);
	}

	return status;

}

/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//{
//  if (GPIO_Pin==INT_ToF_Pin)
//  {
//	  IntCount_ToF++;
//	  if(IntCount_ToF>10) IntCount_ToF=1;
//  }
//}

/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

Status_TypeDef ROIDynamicRangingZone(VL53L1_DEV Dev,
		ToF_Structure *ToFStructure) {
	Status_TypeDef Status;
	VL53L1_RoiConfig_t RoiConfig;

	if (STATUS_OK != VL53L1_StopMeasurement(Dev))
		return STATUS_ERR;
	ToFStructure->userToFRoiConfig.NumberOfRoi = 1; /*sould be only one*/

	RoiConfig.NumberOfRoi = ToFStructure->userToFRoiConfig.NumberOfRoi;
	RoiConfig.UserRois[0].TopLeftX =
			ToFStructure->userToFRoiConfig.UserRois[0].TopLeftX;
	RoiConfig.UserRois[0].TopLeftY =
			ToFStructure->userToFRoiConfig.UserRois[0].TopLeftY;
	RoiConfig.UserRois[0].BotRightX =
			ToFStructure->userToFRoiConfig.UserRois[0].BotRightX;
	RoiConfig.UserRois[0].BotRightY =
			ToFStructure->userToFRoiConfig.UserRois[0].BotRightY;

	if (VL53L1_SetROI(Dev, &RoiConfig) != VL53L1_ERROR_NONE)
		return STATUS_ERR;
	else
		Status = STATUS_OK;
	return Status;
}

/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

Status_TypeDef ROIDynamicMultiZone(VL53L1_DEV Dev, ToF_Structure *ToFStructure) {
	Status_TypeDef Status;
	VL53L1_RoiConfig_t RoiConfig;
	uint8_t numForcntRoi = 0;

	if (STATUS_OK != VL53L1_StopMeasurement(Dev))
		return STATUS_ERR;

	numForcntRoi = ToFStructure->userToFRoiConfig.NumberOfRoi;
	RoiConfig.NumberOfRoi = numForcntRoi;

	for (uint8_t zoneCnt = 0; zoneCnt < numForcntRoi; zoneCnt++) {
		RoiConfig.UserRois[zoneCnt].TopLeftX =
				ToFStructure->userToFRoiConfig.UserRois[zoneCnt].TopLeftX;
		RoiConfig.UserRois[zoneCnt].TopLeftY =
				ToFStructure->userToFRoiConfig.UserRois[zoneCnt].TopLeftY;
		RoiConfig.UserRois[zoneCnt].BotRightX =
				ToFStructure->userToFRoiConfig.UserRois[zoneCnt].BotRightX;
		RoiConfig.UserRois[zoneCnt].BotRightY =
				ToFStructure->userToFRoiConfig.UserRois[zoneCnt].BotRightY;
	}
	if (VL53L1_SetROI(Dev, &RoiConfig) != VL53L1_ERROR_NONE)
		return STATUS_ERR;
	else
		Status = STATUS_OK;
	return Status;
}

/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

Status_TypeDef IRSensorObjectsPosXYZ(VL53L1_DEV Dev,
		ToF_Objects_Pos_s *ToF_Objects_Pos) {
	/*configuration of structure*/
	Status_TypeDef Status = STATUS_OK;

	ToF_Structure ToFStructureXYZ;
	ToF_Structure *pToFStructureXYZ = &ToFStructureXYZ;
	/*
	 * Ranging mode to detect more than one object then Multizone mode to determinate the position of those objects;
	 */

	VL53L1_PresetModes PresetModeXYZ = VL53L1_PRESETMODE_RANGING;
	VL53L1_DistanceModes DistanceModeXYZ = VL53L1_DISTANCEMODE_LONG;
	VL53L1_InterruptMode InterruptModeXYZ = INTERRUPT_DISABLE;
	dynamicZone_s userDynamicZone_sXYZ;
	userDynamicZone_sXYZ.dynamicMultiZone_user = DYNAMIC_MZONE_ON;
	userDynamicZone_sXYZ.dynamicRangingZone_user = DYNAMIC_ZONE_OFF;
	const uint8_t firstRegionShift = 0, secondRegionShift = 4;
	const uint8_t thirdRegionShift = 8, forthRegionShift = 12;
	const uint8_t sameObjectThreshold = 100;
	int16_t minThresoldSameObj = 0;
	int16_t maxThresoldSameObj = 0;
	const uint8_t spacingThreshold = 10;
	int8_t centerOfRegion[16][2] = { { 6, 6 }, { 2, 6 }, { 2, 2 }, { 6, 2 }, {
			-2, 6 }, { -6, 6 }, { -6, 2 }, { -2, 2 }, { -2, -2 }, { -6, -2 }, {
			-6, -6 }, { -2, -6 }, { 6, -2 }, { 2, -2 }, { 2, -6 }, { 6, -6 } };
	uint8_t regionPos[4][4] = { { 5, 4, 1, 0 }, { 6, 7, 2, 3 },
			{ 9, 8, 13, 12 }, { 10, 11, 14, 15 } };/*region position in array index to make vicinity detected process easer*/
	/*local variables*/
	uint8_t numOfObjectInSpace = 0; /*counting number of object in all space*/

	int16_t arrayRegionObjectDis[16][2] = { { -1, -1 }, { -1, -1 }, { -1, -1 },
			{ -1, -1 }, { -1, -1 }, { -1, -1 }, { -1, -1 }, { -1, -1 },
			{ -1, -1 }, { -1, -1 }, { -1, -1 }, { -1, -1 }, { -1, -1 },
			{ -1, -1 }, { -1, -1 }, { -1, -1 } }; /*arrays	first column object number, yet -1 if not*/
	uint8_t numOfRepetingObject[4] = { 0, 0, 0, 0 }; /*			second column depth for this region if detected, yet -1 if not*/
	uint8_t sumOfObjectSubSpaces = 0, kObjectvicinity = 0, vecinityLock = 0,
			indexOfSameObjectDis[] = { };
	uint8_t RegionShift = 0;
	float sumX[4] = { 0, 0, 0, 0 };
	float sumY[4] = { 0, 0, 0, 0 };
	float stepSizeXY[4] = { 0, 0, 0, 0 };

	//uint8_t objectScaned=0,firstEchoObjects=0,secondEchoObject=0;
	//uint8_t stepSizeMM=0;

	//ToFStructureXYZ->userToFRoiConfig.NumberOfRoi=1;
	/* First: start first scan and retry until streamCount has got increased, now we have #of objects in space*/
	/* Process of getting Z measurement*/
	if (Status != STATUS_OK) {
		if (IRSensorInit(Dev) != STATUS_OK)
			return STATUS_ERR;
	}
	if (allRegionToF(Dev, pToFStructureXYZ) != STATUS_OK) /* one region -> one measurement */
		return STATUS_ERR;

	do {
		if (STATUS_OK
				!= tofModeMeasurement(Dev, PresetModeXYZ, DistanceModeXYZ,
						InterruptModeXYZ, userDynamicZone_sXYZ,
						pToFStructureXYZ))
			return STATUS_ERR;
		numOfObjectInSpace = ToFStructureXYZ.tofNumberofObject; /*number of object in all space*/
	} while (numOfObjectInSpace == 0);

	ToF_Objects_Pos->numObjectSpace = numOfObjectInSpace;
	for (uint8_t resetCnt = 3; resetCnt >= numOfObjectInSpace; resetCnt--) {
		ToF_Objects_Pos->objectZspace[resetCnt] = 0;
	}
	/*store Z axis measurements (depth value)*/
	for (uint8_t CNTobjectDepth = 0; CNTobjectDepth < numOfObjectInSpace;
			CNTobjectDepth++) {
		ToF_Objects_Pos->objectZspace[CNTobjectDepth] =
				ToFStructureXYZ.ObjectNumber[CNTobjectDepth].tofDistanceMm;
		stepSizeXY[CNTobjectDepth] = CONST_OF_STEP
				* ToFStructureXYZ.ObjectNumber[CNTobjectDepth].tofDistanceMm;
	}

	/*discussing if there are any objects have same depth to avoid merge them*/
	for (uint8_t cntSameDepth = 0; cntSameDepth < numOfObjectInSpace - 1;
			cntSameDepth++)
		for (uint8_t nextCntSameDepth = cntSameDepth + 1;
				nextCntSameDepth < numOfObjectInSpace; nextCntSameDepth++)
			if (ToF_Objects_Pos->objectZspace[cntSameDepth] - spacingThreshold
					<= ToF_Objects_Pos->objectZspace[nextCntSameDepth]
					&& ToF_Objects_Pos->objectZspace[cntSameDepth]
							+ spacingThreshold
							>= ToF_Objects_Pos->objectZspace[nextCntSameDepth]) {
				vecinityLock = 1;
				indexOfSameObjectDis[kObjectvicinity] = cntSameDepth;
				kObjectvicinity++;
				indexOfSameObjectDis[kObjectvicinity] = nextCntSameDepth;
			}

	/*Second: start scanning subSpaces*/
	/* Process of getting XY measurement*/
	PresetModeXYZ = VL53L1_PRESETMODE_MULTIZONES_SCANNING;
	for (uint8_t cntFourTimes = 1; cntFourTimes <= 4; cntFourTimes++) {
		/*determine which for region of interest*/
		switch (cntFourTimes) {
		/*first subSpace upperRight*/
		case 1:
			if (upperRightsubSpacesRegionToF(Dev, pToFStructureXYZ)
					!= STATUS_OK) /* four region -> four measurement */
				return STATUS_ERR;
			RegionShift = firstRegionShift;
			break;
			/*second subSpace upperLeft*/
		case 2:
			if (upperLeftsubSpacesRegionToF(Dev, pToFStructureXYZ) != STATUS_OK) /* four region -> four measurement */
				return STATUS_ERR;
			RegionShift = secondRegionShift;
			break;
			/*third subSpace lowerLeft*/
		case 3:
			if (lowerLeftsubSpacesRegionToF(Dev, pToFStructureXYZ) != STATUS_OK) /* four region -> four measurement */
				return STATUS_ERR;
			RegionShift = thirdRegionShift;
			break;
			/*fourth subSpace lowerRight*/
		case 4:
			if (lowerRightsubSpacesRegionToF(Dev, pToFStructureXYZ)
					!= STATUS_OK) /* four region -> four measurement */
				return STATUS_ERR;
			RegionShift = forthRegionShift;
			break;
		}
		/*Now we have four regions and four measurement (it well be repeated four times)*/
		for (uint8_t regionCNT = 0; regionCNT < 4; regionCNT++) /*fore region in xy coordinate */
		{
			if (STATUS_OK
					!= tofModeMeasurement(Dev, PresetModeXYZ, DistanceModeXYZ,
							InterruptModeXYZ, userDynamicZone_sXYZ,
							pToFStructureXYZ))
				return STATUS_ERR;

			if (ToFStructureXYZ.tofNumberofObject != 0) {
				sumOfObjectSubSpaces++;
				arrayRegionObjectDis[regionCNT + RegionShift][1] =
						ToFStructureXYZ.ObjectNumber[0].tofDistanceMm; /*only one object might be detected*/
				if (arrayRegionObjectDis[regionCNT + RegionShift][1] <= 0)
					arrayRegionObjectDis[regionCNT + RegionShift][0] = -1;

				minThresoldSameObj =
						ToFStructureXYZ.ObjectNumber[0].tofDistanceMm
								- sameObjectThreshold;
				maxThresoldSameObj =
						ToFStructureXYZ.ObjectNumber[0].tofDistanceMm
								+ sameObjectThreshold;

				for (uint8_t CntOR = 0; CntOR < numOfObjectInSpace; CntOR++) {

					if ((minThresoldSameObj
							<= ToF_Objects_Pos->objectZspace[CntOR])
							&& (maxThresoldSameObj
									>= ToF_Objects_Pos->objectZspace[CntOR])) {
						arrayRegionObjectDis[regionCNT + RegionShift][0] =
								CntOR;

					}

				}
			} else {
				arrayRegionObjectDis[regionCNT + RegionShift][0] = -1;
				arrayRegionObjectDis[regionCNT + RegionShift][1] = -1;
			}
		}
	}

	/*if there are tow objects on same depth*/
	/*then it should not merge it with another object ,so we discuss the vicinity.
	 * by modifying arrayRegionObjectDis[number of object for this region][0]*/

	if (vecinityLock == 1) /*to avoid losing an object*/
	{

		uint8_t iistart = 0, iiend = 0, jjstart = 0, jjend = 0;

		for (uint8_t iForVicinity = 0; iForVicinity < 4; iForVicinity++)
			for (uint8_t jForVicinity = 0; jForVicinity < 4; jForVicinity++) {
				for (uint8_t kForVicinity = 0; kForVicinity <= kObjectvicinity;
						kForVicinity++) {
					if (ToF_Objects_Pos->objectZspace[indexOfSameObjectDis[kForVicinity]]
							== arrayRegionObjectDis[regionPos[iForVicinity][jForVicinity]][1]) {
						// modifying the object number in arrayRegionObjectDis[region][0] array if there is same distance at vicinity
						//so make all vicinity have same object number
						if (iForVicinity == 0) {
							iistart = iForVicinity;
							iiend = iForVicinity + 1;
						} else if (iForVicinity == 3) {
							iistart = iForVicinity - 1;
							iiend = iForVicinity;
						} else {
							iistart = iForVicinity - 1;
							iiend = iForVicinity + 1;
						}
						if (jForVicinity == 0) {
							jjstart = jForVicinity;
							jjend = jForVicinity + 1;
						} else if (jForVicinity == 3) {
							jjstart = jForVicinity - 1;
							jjend = jForVicinity;
						} else {
							jjstart = jForVicinity - 1;
							jjend = jForVicinity + 1;
						}

						for (uint8_t CNTiVicinity = iistart;
								CNTiVicinity < iiend; CNTiVicinity++)
							for (uint8_t CNTjVicinity = jjstart;
									CNTjVicinity < jjend; CNTjVicinity++) {
								if (-1
										!= arrayRegionObjectDis[regionPos[iistart][jjstart]][0])
									arrayRegionObjectDis[regionPos[iistart][jjstart]][0] =
											indexOfSameObjectDis[kForVicinity];
							}
					}
				}

			}
	}
	/*Discuss the condition of having an extra object*/

	/* calculate xPos and yPos using stepSize and depth*/
	for (uint8_t cntProcessedRegion = 0; cntProcessedRegion < 16;
			cntProcessedRegion++) {
		if (0 == arrayRegionObjectDis[cntProcessedRegion][0])/*first object addressed as 0*/
		{
			sumOfObjectSubSpaces++;
			numOfRepetingObject[0]++;
			sumX[0] = sumX[0] + centerOfRegion[cntProcessedRegion][0];
			sumY[0] = sumY[0] + centerOfRegion[cntProcessedRegion][1];

		} else if (1 == arrayRegionObjectDis[cntProcessedRegion][0])/*second object addressed as 1*/
		{
			sumOfObjectSubSpaces++;
			numOfRepetingObject[1]++;
			sumX[1] = sumX[1] + centerOfRegion[cntProcessedRegion][0];
			sumY[1] = sumY[1] + centerOfRegion[cntProcessedRegion][1];

		} else if (2 == arrayRegionObjectDis[cntProcessedRegion][0])/*third object addressed as 2*/
		{
			sumOfObjectSubSpaces++;
			numOfRepetingObject[2]++;
			sumX[2] = sumX[2] + centerOfRegion[cntProcessedRegion][0];
			sumY[2] = sumY[2] + centerOfRegion[cntProcessedRegion][1];

		} else if (3 == arrayRegionObjectDis[cntProcessedRegion][0])/*fourth object addressed as 3*/
		{
			sumOfObjectSubSpaces++;
			numOfRepetingObject[3]++;
			sumX[3] = sumX[3] + centerOfRegion[cntProcessedRegion][0];
			sumY[3] = sumY[3] + centerOfRegion[cntProcessedRegion][1];

		}
		uint8_t iposRegion = 0, jposRegion = 0;
		iposRegion = cntProcessedRegion / 4;
		jposRegion = cntProcessedRegion % 4;
		ToF_Objects_Pos->objectPosRegion[iposRegion][jposRegion] =
				arrayRegionObjectDis[regionPos[iposRegion][jposRegion]][0];
	}
	for (uint8_t iii = 0; iii < 4; iii++) {
		if (numOfRepetingObject[iii] != 0) {
			ToF_Objects_Pos->objectXspace[iii] = (sumX[iii]
					/ numOfRepetingObject[iii]) * stepSizeXY[iii];
			ToF_Objects_Pos->objectYspace[iii] = (sumY[iii]
					/ numOfRepetingObject[iii]) * stepSizeXY[iii];
		} else {
			ToF_Objects_Pos->objectXspace[iii] = 0;
			ToF_Objects_Pos->objectYspace[iii] = 0;
		}
		switch (numOfRepetingObject[iii]) {
		case 0:
			ToF_Objects_Pos->objectSize_t[iii] = NO_OBJECT;
			break;
		case 1:
			ToF_Objects_Pos->objectSize_t[iii] = SMALL;
			break;
		case 2:
			ToF_Objects_Pos->objectSize_t[iii] = MEDUIM;
			break;
		case 3:
			ToF_Objects_Pos->objectSize_t[iii] = BIG;
			break;
		default:
			ToF_Objects_Pos->objectSize_t[iii] = HUGE;
			break;
		}

	}

	return Status;
}

/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

Status_TypeDef allRegionToF(VL53L1_DEV Dev, ToF_Structure *ToFStructure) {

	Status_TypeDef Status = STATUS_OK;
	ToFStructure->userToFRoiConfig.NumberOfRoi = 1;
	ToFStructure->userToFRoiConfig.UserRois[0].BotRightX = 15;
	ToFStructure->userToFRoiConfig.UserRois[0].BotRightY = 0;
	ToFStructure->userToFRoiConfig.UserRois[0].TopLeftX = 0;
	ToFStructure->userToFRoiConfig.UserRois[0].TopLeftY = 15;
	return Status;
}

Status_TypeDef subSpacesRegionToF(VL53L1_DEV Dev, ToF_Structure *ToFStructure) {

	Status_TypeDef Status = STATUS_OK;
	ToFStructure->userToFRoiConfig.NumberOfRoi = 4;
	/*upper right supSpace*/
	ToFStructure->userToFRoiConfig.UserRois[0].BotRightX = 15;
	ToFStructure->userToFRoiConfig.UserRois[0].BotRightY = 8;
	ToFStructure->userToFRoiConfig.UserRois[0].TopLeftX = 8;
	ToFStructure->userToFRoiConfig.UserRois[0].TopLeftY = 15;
	/*upper left supSpace*/
	ToFStructure->userToFRoiConfig.UserRois[1].BotRightX = 7;
	ToFStructure->userToFRoiConfig.UserRois[1].BotRightY = 8;
	ToFStructure->userToFRoiConfig.UserRois[1].TopLeftX = 0;
	ToFStructure->userToFRoiConfig.UserRois[1].TopLeftY = 15;
	/*lower left supSpace*/
	ToFStructure->userToFRoiConfig.UserRois[2].BotRightX = 7;
	ToFStructure->userToFRoiConfig.UserRois[2].BotRightY = 0;
	ToFStructure->userToFRoiConfig.UserRois[2].TopLeftX = 0;
	ToFStructure->userToFRoiConfig.UserRois[2].TopLeftY = 7;
	/*lower right supSpace*/
	ToFStructure->userToFRoiConfig.UserRois[3].BotRightX = 15;
	ToFStructure->userToFRoiConfig.UserRois[3].BotRightY = 0;
	ToFStructure->userToFRoiConfig.UserRois[3].TopLeftX = 8;
	ToFStructure->userToFRoiConfig.UserRois[3].TopLeftY = 7;

	return Status;
}

/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

Status_TypeDef upperRightsubSpacesRegionToF(VL53L1_DEV Dev,
		ToF_Structure *ToFStructure) {

	Status_TypeDef Status = STATUS_OK;
	ToFStructure->userToFRoiConfig.NumberOfRoi = 4;
	/*upper right form first supSpace*/
	ToFStructure->userToFRoiConfig.UserRois[0].BotRightX = 15;
	ToFStructure->userToFRoiConfig.UserRois[0].BotRightY = 12;
	ToFStructure->userToFRoiConfig.UserRois[0].TopLeftX = 12;
	ToFStructure->userToFRoiConfig.UserRois[0].TopLeftY = 15;
	/*upper left form first supSpace*/
	ToFStructure->userToFRoiConfig.UserRois[1].BotRightX = 11;
	ToFStructure->userToFRoiConfig.UserRois[1].BotRightY = 12;
	ToFStructure->userToFRoiConfig.UserRois[1].TopLeftX = 8;
	ToFStructure->userToFRoiConfig.UserRois[1].TopLeftY = 15;
	/*lower left form first supSpace*/
	ToFStructure->userToFRoiConfig.UserRois[2].BotRightX = 11;
	ToFStructure->userToFRoiConfig.UserRois[2].BotRightY = 8;
	ToFStructure->userToFRoiConfig.UserRois[2].TopLeftX = 8;
	ToFStructure->userToFRoiConfig.UserRois[2].TopLeftY = 11;
	/*lower right form first supSpace*/
	ToFStructure->userToFRoiConfig.UserRois[3].BotRightX = 15;
	ToFStructure->userToFRoiConfig.UserRois[3].BotRightY = 8;
	ToFStructure->userToFRoiConfig.UserRois[3].TopLeftX = 12;
	ToFStructure->userToFRoiConfig.UserRois[3].TopLeftY = 11;

	return Status;
}

/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

Status_TypeDef upperLeftsubSpacesRegionToF(VL53L1_DEV Dev,
		ToF_Structure *ToFStructure) {

	Status_TypeDef Status = STATUS_OK;
	ToFStructure->userToFRoiConfig.NumberOfRoi = 4;
	/*upper right form second supSpace*/
	ToFStructure->userToFRoiConfig.UserRois[0].BotRightX = 8;
	ToFStructure->userToFRoiConfig.UserRois[0].BotRightY = 12;
	ToFStructure->userToFRoiConfig.UserRois[0].TopLeftX = 4;
	ToFStructure->userToFRoiConfig.UserRois[0].TopLeftY = 15;
	/*upper left form second supSpace*/
	ToFStructure->userToFRoiConfig.UserRois[1].BotRightX = 3;
	ToFStructure->userToFRoiConfig.UserRois[1].BotRightY = 12;
	ToFStructure->userToFRoiConfig.UserRois[1].TopLeftX = 0;
	ToFStructure->userToFRoiConfig.UserRois[1].TopLeftY = 15;
	/*lower left form second supSpace*/
	ToFStructure->userToFRoiConfig.UserRois[2].BotRightX = 3;
	ToFStructure->userToFRoiConfig.UserRois[2].BotRightY = 8;
	ToFStructure->userToFRoiConfig.UserRois[2].TopLeftX = 0;
	ToFStructure->userToFRoiConfig.UserRois[2].TopLeftY = 11;
	/*lower right form second supSpace*/
	ToFStructure->userToFRoiConfig.UserRois[3].BotRightX = 7;
	ToFStructure->userToFRoiConfig.UserRois[3].BotRightY = 8;
	ToFStructure->userToFRoiConfig.UserRois[3].TopLeftX = 4;
	ToFStructure->userToFRoiConfig.UserRois[3].TopLeftY = 11;

	return Status;
}

/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

Status_TypeDef lowerLeftsubSpacesRegionToF(VL53L1_DEV Dev,
		ToF_Structure *ToFStructure) {

	Status_TypeDef Status = STATUS_OK;
	ToFStructure->userToFRoiConfig.NumberOfRoi = 4;
	/*upper right form second supSpace*/
	ToFStructure->userToFRoiConfig.UserRois[0].BotRightX = 7;
	ToFStructure->userToFRoiConfig.UserRois[0].BotRightY = 4;
	ToFStructure->userToFRoiConfig.UserRois[0].TopLeftX = 4;
	ToFStructure->userToFRoiConfig.UserRois[0].TopLeftY = 7;
	/*upper left form second supSpace*/
	ToFStructure->userToFRoiConfig.UserRois[1].BotRightX = 3;
	ToFStructure->userToFRoiConfig.UserRois[1].BotRightY = 4;
	ToFStructure->userToFRoiConfig.UserRois[1].TopLeftX = 0;
	ToFStructure->userToFRoiConfig.UserRois[1].TopLeftY = 7;
	/*lower left form second supSpace*/
	ToFStructure->userToFRoiConfig.UserRois[2].BotRightX = 3;
	ToFStructure->userToFRoiConfig.UserRois[2].BotRightY = 0;
	ToFStructure->userToFRoiConfig.UserRois[2].TopLeftX = 0;
	ToFStructure->userToFRoiConfig.UserRois[2].TopLeftY = 3;
	/*lower right form second supSpace*/
	ToFStructure->userToFRoiConfig.UserRois[3].BotRightX = 7;
	ToFStructure->userToFRoiConfig.UserRois[3].BotRightY = 0;
	ToFStructure->userToFRoiConfig.UserRois[3].TopLeftX = 4;
	ToFStructure->userToFRoiConfig.UserRois[3].TopLeftY = 3;

	return Status;
}

/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

Status_TypeDef lowerRightsubSpacesRegionToF(VL53L1_DEV Dev,
		ToF_Structure *ToFStructure) {

	Status_TypeDef Status = STATUS_OK;
	ToFStructure->userToFRoiConfig.NumberOfRoi = 4;
	/*upper right form second supSpace*/
	ToFStructure->userToFRoiConfig.UserRois[0].BotRightX = 15;
	ToFStructure->userToFRoiConfig.UserRois[0].BotRightY = 4;
	ToFStructure->userToFRoiConfig.UserRois[0].TopLeftX = 12;
	ToFStructure->userToFRoiConfig.UserRois[0].TopLeftY = 7;
	/*upper left form second supSpace*/
	ToFStructure->userToFRoiConfig.UserRois[1].BotRightX = 11;
	ToFStructure->userToFRoiConfig.UserRois[1].BotRightY = 4;
	ToFStructure->userToFRoiConfig.UserRois[1].TopLeftX = 8;
	ToFStructure->userToFRoiConfig.UserRois[1].TopLeftY = 7;
	/*lower left form second supSpace*/
	ToFStructure->userToFRoiConfig.UserRois[2].BotRightX = 11;
	ToFStructure->userToFRoiConfig.UserRois[2].BotRightY = 0;
	ToFStructure->userToFRoiConfig.UserRois[2].TopLeftX = 8;
	ToFStructure->userToFRoiConfig.UserRois[2].TopLeftY = 3;
	/*lower right form second supSpace*/
	ToFStructure->userToFRoiConfig.UserRois[3].BotRightX = 15;
	ToFStructure->userToFRoiConfig.UserRois[3].BotRightY = 0;
	ToFStructure->userToFRoiConfig.UserRois[3].TopLeftX = 12;
	ToFStructure->userToFRoiConfig.UserRois[3].TopLeftY = 3;

	return Status;
}

/************************ (C) COPYRIGHT Hexabitz *****END OF FILE****/
