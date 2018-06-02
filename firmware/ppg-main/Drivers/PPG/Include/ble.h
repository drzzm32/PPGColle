#ifndef __BLE_H_
#define __BLE_H_


#include "halinc.h"

#define BLE_BUFSIZ 32
#define BLE_PACKSIZ 13
#define BLE_PACKBUF (BLE_PACKSIZ * 2)

typedef struct {
	UART_HandleTypeDef* huart;
	GPIO_TypeDef*       RSTPortGroup;
	uint16_t            RSTPortIndex;
	GPIO_TypeDef*       STEPortGroup;
	uint16_t            STEPortIndex;
	GPIO_TypeDef*       LEDPortGroup;
	uint16_t            LEDPortIndex;
} pBLE;

typedef struct {
	uint8_t hour;
	uint8_t minute;

	uint8_t heart;
	float SpO2;
	uint8_t breath;

	uint8_t phone;
	uint8_t message;

	uint8_t weather;

	uint8_t control;
} DataPack;

typedef struct {
	pBLE* p;
	void (*reset)(pBLE* p);
	uint8_t (*state)(pBLE* p);
	uint8_t (*led)(pBLE* p);
	void (*write)(pBLE* p, char* data);
	void (*read)(pBLE* p, char* data, size_t len);

	void (*ppg)(pBLE* p, int red, int ir);
	void (*acc)(pBLE* p, float acc);
	void (*pull)(pBLE* p, DataPack* pack);
} BLE;

BLE* BLEInit(UART_HandleTypeDef* huart,
		GPIO_TypeDef* RSTPortGroup, uint16_t RSTPortIndex,
		GPIO_TypeDef* STEPortGroup, uint16_t STEPortIndex,
		GPIO_TypeDef* LEDPortGroup, uint16_t LEDPortIndex);


#endif
