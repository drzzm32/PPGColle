#include "./Include/battery.h"

#include "nsio.h"

#include <stdlib.h>
#include <string.h>

void _battery_charge(pBattery* p, uint8_t state) {
	HAL_GPIO_WritePin(p->CEPortGroup, p->CEPortIndex, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void _battery_refresh(pBattery* p) {
	for (uint8_t i = 0; i < 8; i++) {
		HAL_ADC_Start(p->adc);
		HAL_ADC_PollForConversion(p->adc, 5);
		p->buffer[i] = HAL_ADC_GetValue(p->adc);
	}
}

float _battery_voltage(pBattery* p) {
	HAL_ADC_Start(p->adc);
	HAL_ADC_PollForConversion(p->adc, 5);
	uint16_t data = HAL_ADC_GetValue(p->adc);
	ASHL(p->buffer);
	p->buffer[7] = data;

	float result = 0.0F;
	for (uint8_t i = 0; i < 8; i++)
		result += (float) p->buffer[i];
	result /= 8.0F;

	return result / 4095.0F * BAT_REFV * p->scale;
}

Battery* BatteryInit(ADC_HandleTypeDef* adc, float scale,
		GPIO_TypeDef* CEPortGroup, uint16_t CEPortIndex) {
	pBattery* p = malloc(sizeof(pBattery));
	p->adc = adc;
	p->scale = scale;
	p->CEPortGroup = CEPortGroup;
	p->CEPortIndex = CEPortIndex;

	Battery* c = malloc(sizeof(Battery));
	c->p = p;

	c->charge = &_battery_charge;
	c->refresh = &_battery_refresh;
	c->voltage = &_battery_voltage;

	return c;
}
