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

#ifndef INC_BRACELET_IR_TOF_H_
#define INC_BRACELET_IR_TOF_H_


/* Includes */
#include "vl53l1_api.h"
#include "P08R7_i2c.h"

#define HANDLER_ToF_I2C          hi2c2

#define ToF_SENSOR_I2C_ADDRESS   0x52
#define _WAITFORINT() 	         __WFI()

#define TOF_PWR_Pin            GPIO_PIN_4
#define TOF_PWR_GPIO_Port      GPIOE
#define TOF_INT_Pin            GPIO_PIN_1
#define TOF_INT_GPIO_Port      GPIOB
#define TOF_INT_EXTI_IRQn      EXTI1_IRQn
#define TOF_XSHUT_Pin          GPIO_PIN_5
#define TOF_XSHUT_GPIO_Port    GPIOA


//int status;
typedef GPIO_TypeDef GPIO_HANDLE;

/*proximity mode threshold definitions*/
#define LowThres   0
#define HighThres  90

/* define for x,y,z and l,th,phi*/
#define TOF_FIELD_OF_VIEW	0.24            /* (distance/radius filed of view)/2*/
#define CONST_OF_STEP		0.0212132		/* TOFFILEDOFVIEW /8sqrt(2)  so step=CONSTOFSTEP*d */
#define MAX_OBJECT_REGIONS  4
#define MAX_NUMBER_OBJECT   4

/*ToF definitions */

typedef enum
{
	INTERRUPT_DISABLE=0,
	INTERRUPT_ENABLE=1,

}VL53L1_InterruptMode;

typedef struct {

	uint8_t tofTargetNumber;
	uint8_t tofROINumber;
	uint8_t tofRoiStatus;
	uint8_t tofState;
	uint16_t tofDistanceMm;
	uint32_t tofReflectanceTarget;
	uint32_t tofLightAmbient;
	int16_t tofRangeMaxMilliMeter;
	int16_t tofRangeMinMilliMeter;

	uint8_t objectInProximity;

} tofTatgetRangeData;

typedef enum{
	DYNAMIC_MZONE_OFF = 0U,
	DYNAMIC_MZONE_ON
}dynamicMultiZone;

typedef enum{
	DYNAMIC_ZONE_OFF = 0U,
	DYNAMIC_ZONE_ON
}dynamicRangingZone;


typedef struct {
	dynamicMultiZone dynamicMultiZone_user;
	dynamicRangingZone dynamicRangingZone_user;
} dynamicZone_s;

//#define VL53L1_MAX_USER_ZONES		5

typedef struct {

	uint8_t   TopLeftX;   /*!< Top Left x coordinate:  0-15 range */
	uint8_t   TopLeftY;   /*!< Top Left y coordinate:  0-15 range */
	uint8_t   BotRightX;  /*!< Bot Right x coordinate: 0-15 range */
	uint8_t   BotRightY;  /*!< Bot Right y coordinate: 0-15 range */

} ToF_UserRoi_t;


/** @brief Defines ROI configuration parameters
 *
 *  Support up a max of 16 zones, Each Zone has the same size
 *
 */
typedef struct {

	uint8_t          NumberOfRoi;   /*!< Number of Rois defined*/

	ToF_UserRoi_t    UserRois[VL53L1_MAX_USER_ZONES];
		/*!< List of Rois */

} ToF_RoiConfig_t;

typedef struct {

	uint8_t tofNumberofObject;
	uint8_t currentZoneScan;
	uint8_t tofStreamCount;
	tofTatgetRangeData ObjectNumber[MAX_NUMBER_OBJECT];
	ToF_RoiConfig_t	userToFRoiConfig;

} ToF_Structure;

/*spaces and subspaces numbers (regions)
_______________________________
|		|		|		|		|
|	05	|	04	|	01	|	00	|
|_______1______|A|______0_______|
|		|		|		|		|
|	06	|	07	|	02	|	03	|
|______|B|______|______|D|______|
|		|		|		|		|
|	09	|	08	|	13	|	12	|
|_______2______|C|______3_______|
|		|		|		|		|
|	10	|	11	|	14	|	15	|
|_______|_______|_______|_______|

*/
/*small
 * medium
 * beg
 * */
typedef enum {
	NO_OBJECT=0,
	SMALL=1,
	MEDUIM=2,
	BIG=3,
	HUGE=4,
}objectSize_t;

typedef struct {
	uint8_t numObjectSpace;
	int8_t objectPosRegion[MAX_NUMBER_OBJECT][MAX_OBJECT_REGIONS];
	objectSize_t  objectSize_t[MAX_NUMBER_OBJECT];
	float objectXspace[MAX_NUMBER_OBJECT];
	float objectYspace[MAX_NUMBER_OBJECT];
	float objectZspace[MAX_NUMBER_OBJECT];

} ToF_Objects_Pos_s;


/*
 * ToF Functions and descriptions
 *
 * @First param from 1->9 VL53L1_PresetModes
 * @Second param from 1-> VL53L1_DistanceModes
 * @@Third param
 * Distance mode defines how the distance criteria is defined:
• 0: below a certain distance (if the object distance is greater than the configured distance or no object is
found, there is no report).
• 1: beyond a certain distance
• 2: within a distance range (min/max)
• 3: out of the distance range (min/max)

 * @Forth param NumberOfMeasurements nomber of measurement
 * @fifth param 0 pulling || 1 interrupt mode
 *
 * Modifying Structure
 *
 * returns Status_TypeDef
 */
typedef enum {
	STATUS_OK =0,
	STATUS_ERR
			/* Battery charger/gauge error */
} Status_TypeDef;
Status_TypeDef tofModeMeasurement(VL53L1_DEV Dev, VL53L1_PresetModes PresetMode, VL53L1_DistanceModes DistanceMode,
			VL53L1_InterruptMode InterruptMode, dynamicZone_s userDynamicZone_s, ToF_Structure *ToFStructure);

Status_TypeDef IRSensorInit(VL53L1_DEV Dev);
Status_TypeDef IRSensorCalibration(VL53L1_DEV Dev);
Status_TypeDef IRSensorUnit();
Status_TypeDef IRSensorDeactivate();
Status_TypeDef IRSensorActivate();

Status_TypeDef ROIDynamicRangingZone(VL53L1_DEV Dev, ToF_Structure *ToFStructure);
Status_TypeDef ROIDynamicMultiZone(VL53L1_DEV Dev, ToF_Structure *ToFStructure);

VL53L1_Error TimingBudgetExample(VL53L1_DEV Dev);
VL53L1_Error ROIExample(VL53L1_DEV Dev);
VL53L1_Error MultiROIExample(VL53L1_DEV Dev);

Status_TypeDef IRSensorObjectsPosXYZ(VL53L1_DEV Dev, ToF_Objects_Pos_s *ToF_Objects_Pos);
Status_TypeDef allRegionToF(VL53L1_DEV Dev ,ToF_Structure *ToFStructure);
Status_TypeDef subSpacesRegionToF(VL53L1_DEV Dev ,ToF_Structure *ToFStructure);
Status_TypeDef upperRightsubSpacesRegionToF(VL53L1_DEV Dev ,ToF_Structure *ToFStructure);
Status_TypeDef upperLeftsubSpacesRegionToF(VL53L1_DEV Dev ,ToF_Structure *ToFStructure);
Status_TypeDef lowerLeftsubSpacesRegionToF(VL53L1_DEV Dev ,ToF_Structure *ToFStructure);
Status_TypeDef lowerRightsubSpacesRegionToF(VL53L1_DEV Dev ,ToF_Structure *ToFStructure);

#endif /* INC_BRACELET_IR_TOF_H_ */


/************************ (C) COPYRIGHT Hexabitz *****END OF FILE****/

