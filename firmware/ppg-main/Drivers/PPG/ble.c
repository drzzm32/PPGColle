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

    return c;
}
