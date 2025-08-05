/*
 BitzOS (BOS) V0.4.0 - Copyright (C) 2017-2025 Hexabitz
 All rights reserved

 File Name     : APA102_LedMatrix.h
 Description   : APA102 Led Matrix Header file

 @attention: this driver requires to set up SPI

*/

/* Define to prevent recursive inclusion ***********************************/
#ifndef APA102_LEDMATRIX_H_
#define APA102_LEDMATRIX_H_

/* Includes ****************************************************************/
#include "BOS.h"

/* Exported Macros *********************************************************/
#define LED_FRAME_SIZE        	NUM_OF_MODULE_LEDMATRIX*64	/* define number of LEDs in the chain */

#define FALSE 					0
#define TRUE 					1
#define OUT_OF_RANGE			1	/* chosen LED does not exist */
#define RANGE_OK 				0	/* chosen LED exist */
#define LED_START_FRAME_SIZE 	4
#define LED_END_FRAME_SIZE 		4
#define INTINSITY_LED           10  /* maximum intensity 10 */

/* Type Definition *********************************************************/
typedef union {
	struct {				/* LSB */
		uint8_t RED    :8;	/* RED led intensity from 0 (off) to 255 (max) */
		uint8_t GREEN  :8; 	/* GREEN led intensity from 0 (off) to 255 (max) */
		uint8_t BLUE   :8; 	/* BLUE led intensity from 0 (off) to 255 (max) */
		uint8_t GLOBAL :5;  /* Global intensity for all LEDs from 0 (off) to 32 (max) */
		uint8_t INIT   :3;  /* Initialize, the led configuration starts with '111' */
	} FieldsIn;				/* All fields in a LED packet */

	struct {				/* LSB */
		uint8_t RED    :8;	/* RED led intensity from 0 (off) to 255 (max) */
		uint8_t GREEN  :8; 	/* GREEN led intensity from 0 (off) to 255 (max) */
		uint8_t BLUE   :8; 	/* BLUE led intensity from 0 (off) to 255 (max) */
		uint8_t CMD    :8;  /* Global intensity for all LEDs && frame start with */
	} FieldsOut;

	uint32_t data; 	/* RAW LED packet data */

} DigitalLedframe;	/* Frame of LED packets */

/***************************************************************************/
/***************************** General Functions ***************************/
/***************************************************************************/
void DigiLedInit();
void DigiLedSetRGB(uint8_t led, uint8_t red, uint8_t green, uint8_t blue, uint8_t intensity);
void DigiLedSetAllRGB(uint8_t red, uint8_t green, uint8_t blue, uint8_t intensity);
void DigiLedSetColor(uint8_t led, uint8_t color, uint8_t intensity);
void DigiLedSetAllColor(uint8_t color, uint8_t intensity);
void DigiLedSetLedOff(uint8_t led);
void DigiLedSetAllLedOff();
void DigiLedSetLedOn(uint8_t led, uint8_t intensity);
void DigiLedSetAllLedOn(uint8_t intensity);
void DigiLedUpdate(uint8_t forceUpdate);
void DigiLedRGBColorPickerMode(uint8_t color, uint16_t time, uint8_t intensity);
void DigiLedRGBSetColorSomeLed(uint8_t StartLed, uint8_t EndLed, uint8_t color, uint8_t intensity);
void DigiLedScrollMode(uint8_t baseColour, uint8_t scrollRow, uint8_t intensity, uint16_t scrollTime);
void DigiLedFlashMode(uint8_t baseColour, uint8_t flashColour, uint8_t intensity, uint16_t flashTime, uint16_t timeBetweenFlash);

uint8_t DigiLedGetFrameSize(void);
uint8_t DigiLedTestPosition(uint8_t led);

#endif /* APA102_LEDMATRIX_H_ */

/***************** (C) COPYRIGHT HEXABITZ ***** END OF FILE ****************/
