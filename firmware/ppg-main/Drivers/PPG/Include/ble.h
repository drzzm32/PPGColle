#ifndef __BLE_H_
#define __BLE_H_


#include "halinc.h"

#define BLE_BUFSIZ 32

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
	pBLE* p;
	void (*reset)(pBLE* p);
	uint8_t (*state)(pBLE* p);
	uint8_t (*led)(pBLE* p);
	void (*write)(pBLE* p, char* data);
	void (*read)(pBLE* p, char* data, size_t len);

	void (*ppg)(pBLE* p, int red, int ir);
	void (*acc)(pBLE* p, float acc);

} BLE;

BLE* BLEInit(UART_HandleTypeDef* huart,
		GPIO_TypeDef* RSTPortGroup, uint16_t RSTPortIndex,
		GPIO_TypeDef* STEPortGroup, uint16_t STEPortIndex,
		GPIO_TypeDef* LEDPortGroup, uint16_t LEDPortIndex);


#endif
