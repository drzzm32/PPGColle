#ifndef __MPU_H_
#define __MPU_H_


#include "halinc.h"

#define MPU_PAKSIZ 11
#define MPU_BUFSIZ (MPU_PAKSIZ * 2)

typedef struct {
	UART_HandleTypeDef* huart;
	float gravity;
	float temper;
} pMPU;

typedef struct {
	pMPU* p;
	void (*reset)(pMPU* p);
	void (*acc)(pMPU* p, float* x, float* y, float* z);
	float (*accsum)(pMPU* p);
} MPUnit;

MPUnit* MPUInit(UART_HandleTypeDef* huart, float G);


#endif
