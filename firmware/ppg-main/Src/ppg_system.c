#include "ppg_system.h"

#include <string.h>

extern BLE* ble;
extern MPUnit* mpu;
extern RGBOLED* dev;
extern Battery* bat;
extern MAX30102* max30102;
extern uint32_t red, ir;

void DataUpload();
void BLEStateCheck();
void BLEDataReceive();
void BatteryVoltCheck();
void DataPackInfoShow();

Work workList[] = {
	{ &DataUpload },
	{ &BLEStateCheck },
	{ &BLEDataReceive },
	{ &BatteryVoltCheck },
	{ &DataPackInfoShow },
	{ 0 }
};

DataPack pack = {
	.hour = 12,
	.minute = 34,
	.heart = 75,
	.SpO2 = 98.25,
	.breath = 18,
	.phone = 2,
	.message = 5,
	.weather = PPG_WEATHER_SUNNY,
	.control = 0x80
};

uint8_t CTRL() { return pack.control & 0x01; }
uint8_t DEBUG() { return pack.control & 0x80; }

volatile uint32_t tookTime;
volatile uint32_t tick = 0;
#define defineWork(period) if (tick % period == 0)
static uint32_t times[32] = { 0 };

void systemWork() {
	if (tick == 0) {
		dev->bitmaps(dev->p, 0, 0, 128, 128, getBackground());
		HAL_GPIO_TogglePin(LED_A_GPIO_Port, LED_A_Pin);
	}

	tookTime = HAL_GetTick();

	for (uint8_t i = 0; workList[i].run != 0; i++)
		workList[i].run();

	tookTime = HAL_GetTick() - tookTime;

	ASHL(times);
	times[15] = tookTime;
	float result = 0.0F;
	for (uint8_t i = 0; i < 16; i++)
		result += (float) times[i];
	result /= 16.0F;
	if (DEBUG()) dev->printf(dev->p, 0, 118, "%1.2f", result);

	tick += 1;
	HAL_GPIO_TogglePin(LED_A_GPIO_Port, LED_A_Pin);
	HAL_GPIO_TogglePin(LED_B_GPIO_Port, LED_B_Pin);
}

void DataUpload() {
	defineWork(2) {
		if (max30102->chkint(max30102->p)) {
			max30102->fifo(max30102->p, &red, &ir);
			if (ble->state(ble->p) && CTRL())
				ble->ppg(ble->p, red, ir);
		}
	} else defineWork(3) {
		float acc = mpu->accsum(mpu->p);
		if (ble->state(ble->p) && CTRL())
			ble->acc(ble->p, acc);
	}
}

void BLEStateCheck() {
	defineWork(10) {
		dev->icon(dev->p, 0, 0, PPG_TOP_WIDTH, PPG_TOP_HEIGHT,
				getTopIcon(ble->state(ble->p) ? PPG_TOP_BT_OK : PPG_TOP_BT));
		dev->icon(dev->p, 20, 0, PPG_TOP_WIDTH, PPG_TOP_HEIGHT,
				getTopIcon(ble->led(ble->p) ? PPG_TOP_LED_ON : PPG_TOP_LED));
	}
}

void BLEDataReceive() {
	defineWork(5) {
		if (ble->state(ble->p))
			ble->pull(ble->p, &pack);
	}
}

void BatteryVoltCheck() {
	defineWork(20) {
		float volt = bat->voltage(bat->p);
		if (volt > BAT_USBV)
			dev->icon(dev->p, 87, 0, PPG_TOP_WIDTH, PPG_TOP_HEIGHT, getTopIcon(PPG_TOP_USB));
		else
			dev->icon(dev->p, 87, 0, PPG_TOP_WIDTH, PPG_TOP_HEIGHT, getBlank());

		if (volt > BAT_CHGV)
			dev->icon(dev->p, 107, 0, PPG_TOP_WIDTH, PPG_TOP_HEIGHT, getTopIcon(PPG_TOP_BAT_CHG));
		else if (volt < BAT_LOWV)
			dev->icon(dev->p, 107, 0, PPG_TOP_WIDTH, PPG_TOP_HEIGHT, getTopIcon(PPG_TOP_BAT_LOW));
		else
			dev->icon(dev->p, 107, 0, PPG_TOP_WIDTH, PPG_TOP_HEIGHT, getTopIcon(PPG_TOP_BAT));

		dev->colorf(dev->p, 0xFF9800);
		dev->printfc(dev->p, 20, "%1.2fV", bat->voltage(bat->p));
		dev->colorf(dev->p, 0xFFFFFF);
	}
}

void DataPackInfoShow() {
	defineWork(10) {
		dev->font(dev->p, RGBBig);
		dev->printfc(dev->p, 4, "%2d:%2d", pack.hour, pack.minute);
		dev->font(dev->p, RGBSmall);

		uint32_t color = 0x4CAF50;
		if (pack.heart > 120) color = 0xF44336;
		else if (pack.heart > 100) color = 0xFFEB3B;
		else if (pack.heart < 60) color = 0x2196F3;
		dev->colorf(dev->p, color);
		dev->font(dev->p, RGBBig); dev->scale(dev->p, 2);
		dev->printf(dev->p, 4, 48, "%3d", pack.heart);
		dev->font(dev->p, RGBSmall); dev->scale(dev->p, 1);
		dev->colorf(dev->p, 0xFFFFFF);

		dev->font(dev->p, RGBBig);
		dev->printf(dev->p, 56, 36, "%2.1f", pack.SpO2);
		dev->printf(dev->p, 64, 56, "%3d", pack.breath);

		if (pack.phone > 99) dev->printfcp(dev->p, 20, 120, "99+");
		else dev->printfcp(dev->p, 20, 120, "%d", pack.phone);
		if (pack.message > 99) dev->printfcp(dev->p, 108, 120, "99+");
		else dev->printfcp(dev->p, 108, 120, "%d", pack.message);
		dev->font(dev->p, RGBSmall);

		dev->iconc(dev->p, 64, 102, PPG_WEATHER_WIDTH, PPG_WEATHER_HEIGHT,
				getWeatherIcon(pack.weather));
	}
}

#undef defineWork
