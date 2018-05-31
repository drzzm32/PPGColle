#include "./Include/battery.h"

#include <string.h>

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
	memmove(p->buffer[0], p->buffer[1], sizeof(uint16_t) * 7);
	p->buffer[7] = data;

	float result = 0.0F;
	for (uint8_t i = 0; i < 8; i++)
		result += (float) p->buffer[i];
	result /= 8.0F;

	return result / 4095.0F * BAT_REFV * p->scale;
}

Battery* BatteryInit(ADC_HandleTypeDef* adc, float scale) {
	pBattery* p = malloc(sizeof(pBattery));
	p->adc = adc;
	p->scale = scale;

	Battery* c = malloc(sizeof(Battery));
	c->p = p;

	c->refresh = &_battery_refresh;
	c->voltage = &_battery_voltage;

	return c;
}
