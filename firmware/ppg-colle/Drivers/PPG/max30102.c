#include "./Include/max30102.h"

#include <stdlib.h>

void _max_30102_send(pMAX30102* p, uint8_t reg, uint8_t byte) {
	uint8_t buf[] = { reg, byte };
	HAL_I2C_Master_Transmit(p->i2c, p->writeAddr, buf, 2, MAX_COM_DELAY);
}

uint8_t _max_30102_read(pMAX30102* p, uint8_t reg) {
	uint8_t data = 0;
	HAL_I2C_Master_Transmit(p->i2c, p->writeAddr, &reg, 1, MAX_COM_DELAY);
	HAL_I2C_Master_Receive(p->i2c, p->readAddr, &data, 1, MAX_COM_DELAY);
	return data;
}

void _max_30102_reads(pMAX30102* p, uint8_t reg, uint8_t* data, uint8_t len) {
	HAL_I2C_Master_Transmit(p->i2c, p->writeAddr, &reg, 1, MAX_COM_DELAY);
	HAL_I2C_Master_Receive(p->i2c, p->readAddr, data, len, MAX_COM_DELAY);
}

void _max_30102_init(pMAX30102* p) {
	_max_30102_send(p, MAX_INTR_ENABLE_1, 0xC0);
	_max_30102_send(p, MAX_INTR_ENABLE_2, 0x00);
	_max_30102_send(p, MAX_FIFO_WR_PTR, 0x00);
	_max_30102_send(p, MAX_OVF_COUNTER, 0x00);
	_max_30102_send(p, MAX_FIFO_RD_PTR, 0x00);
	_max_30102_send(p, MAX_FIFO_CONFIG, 0x4f);
	_max_30102_send(p, MAX_MODE_CONFIG, 0x03);
	_max_30102_send(p, MAX_SPO2_CONFIG, 0x27);
	_max_30102_send(p, MAX_LED1_PA, 0x24);
	_max_30102_send(p, MAX_LED2_PA, 0x24);
	_max_30102_send(p, MAX_PILOT_PA, 0x7f);
}

void _max_30102_reset(pMAX30102* p) {
	_max_30102_send(p, MAX_MODE_CONFIG, 0x40);
}

void _max_30102_clrint(pMAX30102* p) {
	_max_30102_read(p, MAX_INTR_STATUS_1);
}

void _max_30102_fifo(pMAX30102* p, uint32_t* red, uint32_t* ir) {
	uint8_t buffer[6];
	*red = 0; *ir = 0;

	_max_30102_read(p, MAX_INTR_STATUS_1);
	_max_30102_read(p, MAX_INTR_STATUS_2);
	_max_30102_reads(p, MAX_FIFO_DATA, buffer, 6);

	*red += ((uint32_t) buffer[0]) << 16;
	*red += ((uint32_t) buffer[1]) << 8;
	*red +=  (uint32_t) buffer[2];
	*red &= 0x03FFFF;

	*ir += ((uint32_t) buffer[3]) << 16;
	*ir += ((uint32_t) buffer[4]) << 8;
	*ir +=  (uint32_t) buffer[5];
	*ir &= 0x03FFFF;
}

MAX30102* MAX30102Init(I2C_HandleTypeDef* pi2c, uint8_t pIICAddr) {
	pMAX30102* p = malloc(sizeof(pMAX30102));
	p->i2c = pi2c;
	p->readAddr = pIICAddr | 0x01;
	p->writeAddr = pIICAddr & 0xFE;

	MAX30102* c = malloc(sizeof(MAX30102));
	c->p = p;
#ifdef MAX_USE_PRIVATE_FUN
	c->send = &_max_30102_send;
	c->read = &_max_30102_read;
	c->reads = &_max_30102_reads;
#endif
	c->init = &_max_30102_init;
	c->reset = &_max_30102_reset;
	c->clrint = &_max_30102_clrint;
	c->fifo = &_max_30102_fifo;

	return c;
}
