#include "./Include/mpu.h"

#include <stdlib.h>
#include <math.h>

void _mpu_reset(pMPU* p) {
	uint8_t buf[] = { 0xFF, 0xAA, 0x52 };
	HAL_UART_Transmit(p->huart, buf, sizeof(buf) / sizeof(buf[0]), 10);
}

void _mpu_acc(pMPU* p, float* x, float* y, float* z) {
	uint8_t buf[MPU_BUFSIZ]; uint8_t ptr = 0;
	HAL_UART_Receive(p->huart, buf, MPU_BUFSIZ, 20);

	while (buf[ptr] != 0x55 && buf[ptr + 1] != 0x51) {
		if (ptr < MPU_BUFSIZ - 2) ptr += 1;
		else break;
	}

	*x = *y = *z = 0.0F;

	if (ptr <= MPU_PAKSIZ) {
		if (buf[ptr] != 0x55 && buf[ptr + 1] != 0x51)
			return;

		uint8_t chksum = 0;
		for (uint8_t i = 0; i < MPU_PAKSIZ - 1; i++)
			chksum += buf[ptr + i];
		if (chksum != buf[ptr + MPU_PAKSIZ - 1]) return;

		short tmp;
		tmp = (short) (buf[ptr + 3] << 8 | buf[ptr + 2]);
		*x = tmp / 32768.0F * p->gravity;
		tmp = (short) (buf[ptr + 5] << 8 | buf[ptr + 4]);
		*y = tmp / 32768.0F * p->gravity;
		tmp = (short) (buf[ptr + 7] << 8 | buf[ptr + 6]);
		*z = tmp / 32768.0F * p->gravity;

		tmp = (short) (buf[ptr + 9] << 8 | buf[ptr + 8]);
		p->temper = tmp / 340.0F + 36.25F;
	}
}

float _mpu_accsum(pMPU* p) {
	float x, y, z;
	_mpu_acc(p, &x, &y, &z);
	return sqrt(x * x + y * y + z * z);
}

MPUnit* MPUInit(UART_HandleTypeDef* huart, float G) {
	pMPU* p = malloc(sizeof(pMPU));
    p->huart = huart;
    p->gravity = G;

    MPUnit* c = malloc(sizeof(MPUnit));
    c->p = p;

    c->reset = &_mpu_reset;
    c->acc = &_mpu_acc;
    c->accsum = &_mpu_accsum;

    return c;
}
