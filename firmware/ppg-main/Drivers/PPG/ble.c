#include "./Include/ble.h"

#include "./Include/nsio.h"
#include <stdlib.h>
#include <string.h>

void _ble_reset(pBLE* p) {
	HAL_GPIO_WritePin(p->RSTPortGroup, p->RSTPortIndex, GPIO_PIN_RESET);
	HAL_Delay(50);
	HAL_GPIO_WritePin(p->RSTPortGroup, p->RSTPortIndex, GPIO_PIN_SET);
}

uint8_t _ble_state(pBLE* p) {
	return HAL_GPIO_ReadPin(p->STEPortGroup, p->STEPortIndex);
}

uint8_t _ble_led(pBLE* p) {
	return HAL_GPIO_ReadPin(p->LEDPortGroup, p->LEDPortIndex);
}

#define _len BLE_BUFSIZ
void _write(UART_HandleTypeDef* uart, char* tag, char* data) {
	print("[%s] %s\n", tag, data);
	char buf[_len] = "\0";
	strcat(buf, data); strcat(buf, "\r\n");
	HAL_UART_Transmit(uart, (uint8_t*) buf, strlen(buf), 100);

	memset(buf, 0, _len);
	HAL_UART_Receive(uart, (uint8_t*) buf, _len, 100);

	for (uint8_t i = 0; i < _len; i++)
		if (buf[i] == '\r') buf[i] = '\n';
	print("[%s] %s", tag, buf);
}
#undef _len

void _ble_write(pBLE* p, char* data) {
	_write(p->huart, "BLE", data);
}

void _ble_read(pBLE* p, char* data, size_t len) {
	HAL_UART_Receive(p->huart, (uint8_t*) data, len, 50);
}

void _ble_ppg(pBLE* p, int red, int ir) {
	char txBuf[32];
	sprintf(txBuf, "P%d,%d", red, ir);
	HAL_UART_Transmit(p->huart, (uint8_t*) txBuf, strlen(txBuf), 50);
}

void _ble_acc(pBLE* p, float acc) {
	char txBuf[32];
	sprintf(txBuf, "A%1.4f", acc);
	HAL_UART_Transmit(p->huart, (uint8_t*) txBuf, strlen(txBuf), 50);
}

void _ble_pull(pBLE* p, DataPack* pack) {
	/*	Total: 13 bytes

	0							0xFF
	1							0xFE
	2	uint8_t hour;			byte (0x00 ~ 0x17)
	3	uint8_t minute;			byte (0x00 ~ 0x3B)

	4	uint8_t heart;			byte (0x00 ~ 0xFF)
	5	float SpO2;				byte (0x00 ~ 0xFF) <- (int) float_data
	6							byte (0x00 ~ 0xFF) <- (int) ((float_data - ((int) float_data)) * 100)
	7	uint8_t breath;			byte (0x00 ~ 0xFF)

	8	uint8_t phone;			byte (0x00 ~ 0xFF)
	9	uint8_t message;		byte (0x00 ~ 0xFF)

	10	uint8_t weather;		byte (0x00 ~ 0x05)
			WEATHER_SUNNY	0
			WEATHER_CLOUDY	1
			WEATHER_FOG		2
			WEATHER_PCLOUDY	3
			WEATHER_RAINY	4
			WEATHER_WINDY	5

	11	uint8_t control;		byte (0x00: stop, 0x01: start)

	12		check sum			byte (0x00 ~ 0xFF) <- sum(0 : 11)

	*/

	if (pack == 0) return;

	uint8_t buf[BLE_PACKBUF]; uint8_t ptr = 0;
	HAL_UART_Receive(p->huart, buf, BLE_PACKBUF, 50);

	while (buf[ptr] != 0xFF && buf[ptr + 1] != 0xFE) {
		if (ptr < BLE_PACKBUF - 2) ptr += 1;
		else break;
	}

	if (ptr <= BLE_PACKSIZ) {
		if (buf[ptr] != 0xFF && buf[ptr + 1] != 0xFE)
			return;

		uint8_t chksum = 0;
		for (uint8_t i = 0; i < BLE_PACKSIZ - 1; i++)
			chksum += buf[ptr + i];
		if (chksum != buf[ptr + BLE_PACKSIZ - 1]) return;

		pack->hour =	buf[ptr + 2];
		pack->minute =	buf[ptr + 3];

		pack->heart =	buf[ptr + 4];
		pack->SpO2 =	buf[ptr + 5] +
						buf[ptr + 6] * 0.01F;
		pack->breath =	buf[ptr + 7];

		pack->phone =	buf[ptr + 8];
		pack->message =	buf[ptr + 9];

		pack->weather =	buf[ptr + 10];

		pack->control = buf[ptr + 11];

		if (pack->hour > 24) pack->hour = 0;
		if (pack->minute > 60) pack->minute = 0;
		if (pack->weather > 5) pack->weather = 0;

		HAL_UART_Transmit(p->huart, "OK", 2, 50);
	}
}

BLE* BLEInit(UART_HandleTypeDef* huart,
		GPIO_TypeDef* RSTPortGroup, uint16_t RSTPortIndex,
		GPIO_TypeDef* STEPortGroup, uint16_t STEPortIndex,
		GPIO_TypeDef* LEDPortGroup, uint16_t LEDPortIndex) {
    pBLE* p = malloc(sizeof(pBLE));
    p->huart = huart;
    p->RSTPortGroup = RSTPortGroup;
    p->RSTPortIndex = RSTPortIndex;
    p->STEPortGroup = STEPortGroup;
	p->STEPortIndex = STEPortIndex;
	p->LEDPortGroup = LEDPortGroup;
	p->LEDPortIndex = LEDPortIndex;

    BLE* c = malloc(sizeof(BLE));
    c->p = p;

    c->reset = &_ble_reset;
    c->state = &_ble_state;
    c->led = &_ble_led;
    c->write = &_ble_write;
    c->read = &_ble_read;

    c->ppg = &_ble_ppg;
    c->acc = &_ble_acc;
    c->pull = &_ble_pull;

    return c;
}
