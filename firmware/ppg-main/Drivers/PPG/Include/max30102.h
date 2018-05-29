#ifndef __MAX30102_H_
#define __MAX30102_H_


#include "halinc.h"

//#define MAX_USE_PRIVATE_FUN
#define MAX_COM_DELAY 100

#define MAX_30102_ADDR 0xAE

#define MAX_INTR_STATUS_1 0x00
#define MAX_INTR_STATUS_2 0x01
#define MAX_INTR_ENABLE_1 0x02
#define MAX_INTR_ENABLE_2 0x03
#define MAX_FIFO_WR_PTR 0x04
#define MAX_OVF_COUNTER 0x05
#define MAX_FIFO_RD_PTR 0x06
#define MAX_FIFO_DATA 0x07
#define MAX_FIFO_CONFIG 0x08
#define MAX_MODE_CONFIG 0x09
#define MAX_SPO2_CONFIG 0x0A
#define MAX_LED1_PA 0x0C
#define MAX_LED2_PA 0x0D
#define MAX_PILOT_PA 0x10
#define MAX_MULTI_LED_CTRL1 0x11
#define MAX_MULTI_LED_CTRL2 0x12
#define MAX_TEMP_INTR 0x1F
#define MAX_TEMP_FRAC 0x20
#define MAX_TEMP_CONFIG 0x21
#define MAX_PROX_INT_THRESH 0x30
#define MAX_REV_ID 0xFE
#define MAX_PART_ID 0xFF

typedef struct {
	I2C_HandleTypeDef* i2c;
	GPIO_TypeDef* INTPortGroup;
	uint16_t INTPortIndex;
	uint8_t readAddr;
	uint8_t writeAddr;
} pMAX30102;

typedef struct {
	pMAX30102* p;
#ifdef MAX_USE_PRIVATE_FUN
	void (*send)(pMAX30102* p, uint8_t reg, uint8_t byte);
	uint8_t (*read)(pMAX30102* p, uint8_t reg);
	void (*reads)(pMAX30102* p, uint8_t reg, uint8_t* data, uint8_t len);
#endif
	void (*init)(pMAX30102* p);
	void (*reset)(pMAX30102* p);
	void (*clrint)(pMAX30102* p);
	void (*waitint)(pMAX30102* p);
	void (*fifo)(pMAX30102* p, uint32_t* red, uint32_t* ir);
} MAX30102;

MAX30102* MAX30102Init(I2C_HandleTypeDef* pi2c, uint8_t pIICAddr,
		GPIO_TypeDef* INTPortGroup, uint16_t INTPortIndex);


#endif
