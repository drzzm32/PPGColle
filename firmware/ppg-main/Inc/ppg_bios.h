#ifndef __PPG_BIOS_H_
#define __PPG_BIOS_H_


#include "nsio.h"

#include "rgboled.h"
#include "asset.h"
#include "flash.h"
#include "logo.h"
#include "ble.h"
#include "mpu.h"
#include "max30102.h"
#include "battery.h"

#define VERSION "dev180602"
#define CHK_DELAY 1000

//#define USE_TESTFUNC

#ifdef USE_TESTFUNC
void extTest();
#endif
void hardwareInit();


#endif
