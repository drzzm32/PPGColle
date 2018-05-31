#include "ppg_system.h"

#include "cmsis_os.h"

#include <string.h>

extern UART_HandleTypeDef huart1;

extern BLE* ble;
extern MPUnit* mpu;
extern RGBOLED* dev;
extern Battery* bat;
extern MAX30102* max30102;
extern uint32_t red, ir;

osThreadId hBLEStateCheck;
osThreadId hBatteryVoltCheck;

void BLEStateCheck(void const * argument);
void BatteryVoltCheck(void const * argument);

void registerThreads() {
	osThreadDef(bleCheck, BLEStateCheck, osPriorityNormal, 0, 128);
	hBLEStateCheck = osThreadCreate(osThread(bleCheck), NULL);

	osThreadDef(batCheck, BatteryVoltCheck, osPriorityNormal, 0, 128);
	hBatteryVoltCheck = osThreadCreate(osThread(batCheck), NULL);
}

char rxBuf[32]; char txBuf[32];
void BLEStateCheck(void const * argument) {
	while (1) {
		HAL_GPIO_WritePin(LED_A_GPIO_Port, LED_A_Pin, ble->led(ble->p));
		HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, ble->state(ble->p));

		if (ble->state(ble->p)) {
			max30102->waitint(max30102->p);
			max30102->fifo(max30102->p, &red, &ir);
			sprintf(txBuf, "%d,%d", red, ir);
			HAL_UART_Transmit(&huart1, (uint8_t*) txBuf, strlen(txBuf), 100);

			HAL_UART_Receive(&huart1, (uint8_t*) rxBuf, 32, 100);
			if (strlen(rxBuf) > 0)
				print("%s\n", rxBuf);

			memset(rxBuf, 0, 32);
			memset(txBuf, 0, 32);
		} else {
			osDelay(100);
		}
	}
}

void BatteryVoltCheck(void const * argument) {
	while (1) {
		dev->colorf(dev->p, 0xFF9800);
		dev->printf(dev->p, 96, 0, "%1.2fV", bat->voltage(bat->p));
		dev->colorf(dev->p, 0xFFFFFF);
		osDelay(500);
	}
}
