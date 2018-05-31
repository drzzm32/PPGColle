#ifndef __BATTERY_H_
#define __BATTERY_H_


#include "halinc.h"

#define BAT_REFV 3.3F

typedef struct {
	ADC_HandleTypeDef* adc;
	float scale;
	uint16_t buffer[8];
} pBattery;

typedef struct {
	pBattery* p;
	void (*refresh)(pBattery* p);
	float (*voltage)(pBattery* p);
} Battery;

Battery* BatteryInit(ADC_HandleTypeDef* adc, float scale);


#endif
