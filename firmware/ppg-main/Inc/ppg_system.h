#ifndef __PPG_SYSTEM_H_
#define __PPG_SYSTEM_H_


#include "ppg_bios.h"

typedef struct {
	void (*run)();
} Work;

void systemWork();


#endif
