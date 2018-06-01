#include "ppg_system.h"

#include <string.h>

extern BLE* ble;
extern MPUnit* mpu;
extern RGBOLED* dev;
extern Battery* bat;
extern MAX30102* max30102;
extern uint32_t red, ir;

void PPGUpload();
void BLEStateCheck();
void BLEDataReceive();
void BatteryVoltCheck();

Work workList[] = {
	{ &PPGUpload },
	{ &BLEStateCheck },
	//{ &BLEDataReceive },
	{ &BatteryVoltCheck },
	{ 0 }
};

volatile uint32_t tookTime;
volatile uint32_t tick = 0;
#define defineWork(period) if (tick % period == 0)

static uint32_t times[8] = { 0 };

void systemWork() {
	if (tick == 0) {
		dev->bitmaps(dev->p, 0, 0, 128, 128, getBackground());
		HAL_GPIO_TogglePin(LED_A_GPIO_Port, LED_A_Pin);
	}

	tookTime = HAL_GetTick();

	for (uint8_t i = 0; workList[i].run != 0; i++)
		workList[i].run();

	tookTime = HAL_GetTick() - tookTime;

	memmove(times, times + 1, 7);
	times[7] = tookTime;
	float result = 0.0F;
	for (uint8_t i = 0; i < 8; i++)
		result += (float) times[i];
	result /= 8.0F;
	dev->printf(dev->p, 0, 118, "dt: %1.2f ms", result);

	tick += 1;
	HAL_GPIO_TogglePin(LED_A_GPIO_Port, LED_A_Pin);
	HAL_GPIO_TogglePin(LED_B_GPIO_Port, LED_B_Pin);
}

void PPGUpload() {
	defineWork(1) {
		if (max30102->chkint(max30102->p)) {
			max30102->fifo(max30102->p, &red, &ir);
			if (ble->state(ble->p))
				ble->ppg(ble->p, red, ir);
		}
	}
}

void BLEStateCheck() {
	defineWork(10) {
		dev->icon(dev->p, 0, 0, PPG_TOP_WIDTH, PPG_TOP_HEIGHT, getTopIcon(ble->state(ble->p) ? PPG_TOP_BT_OK : PPG_TOP_BT));
		dev->icon(dev->p, 20, 0, PPG_TOP_WIDTH, PPG_TOP_HEIGHT, getTopIcon(ble->led(ble->p) ? PPG_TOP_LED_ON : PPG_TOP_LED));
	}
}

static char rxBuf[32];
void BLEDataReceive() {
	defineWork(5) {
		memset(rxBuf, 0, 32);
		if (ble->state(ble->p)) {
			ble->read(ble->p, rxBuf, 32);
			if (strlen(rxBuf) > 0)
				print("%s\n", rxBuf);
		}
	}
}

void BatteryVoltCheck() {
	defineWork(10) {
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
		dev->printfc(dev->p, 12, "%1.2fV", bat->voltage(bat->p));
		dev->colorf(dev->p, 0xFFFFFF);
	}
}

#undef defineWork
