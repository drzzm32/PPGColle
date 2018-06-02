#ifndef __BATTERY_H_
#define __BATTERY_H_


#include "halinc.h"

#define BAT_REFV 3.3F
#define BAT_USBV 4.3F
#define BAT_LOWV 3.6F
#define BAT_CHGV 4.25F

typedef struct {
	ADC_HandleTypeDef* adc;
	GPIO_TypeDef*      CEPortGroup;
	uint16_t           CEPortIndex;
	float scale;
	uint16_t buffer[8];
} pBattery;

typedef struct {
	pBattery* p;
	void (*charge)(pBattery* p, uint8_t state);
	void (*refresh)(pBattery* p);
	float (*voltage)(pBattery* p);
} Battery;

Battery* BatteryInit(ADC_HandleTypeDef* adc, float scale,
		GPIO_TypeDef* CEPortGroup, uint16_t CEPortIndex);


#endif
