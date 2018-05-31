#include "ppg_bios.h"

#include "main.h"
#include "fatfs.h"
#include "usbd_core.h"
#include "usb_device.h"

#include <string.h>
#include <math.h>

extern ADC_HandleTypeDef hadc1;
extern I2C_HandleTypeDef hi2c1;
extern SPI_HandleTypeDef hspi2;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

RGBOLED* dev;
Flash* flash;
uint8_t flashOK = 0;
Battery* bat;

BLE* ble;
MPUnit* mpu;
MAX30102* max30102;
uint32_t red, ir;

FATFS fileSystem;
FIL testFile;
uint8_t testBuffer[16] = "Hello Gensokyo!\0";
UINT testBytes;
FRESULT res;

void extTest() {
	if (HAL_GPIO_ReadPin(B13_GPIO_Port, B13_Pin) == GPIO_PIN_RESET) {
		if (HAL_GPIO_ReadPin(B15_GPIO_Port, B15_Pin) == GPIO_PIN_RESET) {
			print("Erase flash...\n");
			flash->eraseAll(flash->p);
			while (flash->busy(flash->p));
			print("Erase done.\n");
		} else {
			print("Test flash...\n");
			for (uint8_t n = 0; n < 32; n++) {
				print("Test cycle: %d\n", n + 1);
				uint8_t buf[512]; uint32_t addr = rand() & 0xFFFE00;
				print("  Addr: 0x%X\n", addr);
				memset(buf, 0x32, 512);
				flash->write512byte(flash->p, addr, buf);
				memset(buf, 0x00, 512);
				flash->read512byte(flash->p, addr, buf);
				uint16_t diff = 0;
				for (uint16_t i = 0; i < 512; i++)
					if (buf[i] != 0x32) diff++;
				print("  Diff: %d\n", diff);
				HAL_Delay(1000);
			}
		}
	}

	if (HAL_GPIO_ReadPin(B12_GPIO_Port, B12_Pin) == GPIO_PIN_RESET) {
		f_mount(&fileSystem, USERPath, 1);
		if(f_mount(&fileSystem, USERPath, 1) == FR_OK) {
			uint8_t path[13] = "NYA_GAME.TXT";
			path[12] = '\0';
			res = f_open(&testFile, (char*)path, FA_WRITE | FA_CREATE_ALWAYS);
			res = f_write(&testFile, testBuffer, 16, &testBytes);
			res = f_close(&testFile);

			print("FATFS OK...\n");
		} else {
			print("FATFS Error\n");
		}
	}

	if (HAL_GPIO_ReadPin(B14_GPIO_Port, B14_Pin) == GPIO_PIN_RESET) {
		print("Init USB...\n");
		USBD_Start(&hUsbDeviceFS);
		print("USB initialized\n");
	}
}

void hardwareInit() {
	USBD_Stop(&hUsbDeviceFS);
	HAL_Delay(1000);

	dev = SoftRGBInit(
			OLED_SDA_GPIO_Port, OLED_SDA_Pin,
			OLED_SCL_GPIO_Port, OLED_SCL_Pin,
			OLED_DC_GPIO_Port, OLED_DC_Pin,
			OLED_CS_GPIO_Port, OLED_CS_Pin,
			OLED_RST_GPIO_Port, OLED_RST_Pin);

	dev->reset(dev->p);
	dev->init(dev->p);

	dev->colorb(dev->p, 0xFFFFFF);
	dev->colorf(dev->p, 0x000000);
	dev->clear(dev->p);
	dev->bitmapsc(dev->p, 63, 63, 64, 64, getLogo());

	HAL_Delay(3000);
	dev->colorb(dev->p, 0x000000);
	dev->colorf(dev->p, 0xFFFFFF);
	dev->clear(dev->p);

	if (HAL_GPIO_ReadPin(B15_GPIO_Port, B15_Pin) == GPIO_PIN_RESET) {
		dev->bitmap(dev->p, 0 ,0 ,128, 128, getImage());
		HAL_Delay(3000);
		while (HAL_GPIO_ReadPin(B15_GPIO_Port, B15_Pin) == GPIO_PIN_SET);
		dev->clear(dev->p);
	}

	//HAL_GPIO_WritePin(BAT_CE_GPIO_Port, BAT_CE_Pin, GPIO_PIN_RESET);

	print("==============\n");
	print("PPGColle v1.0\n");
	print("by drzzm32\n");
	print("==============\n");
	HAL_Delay(1000);

	print("Init Battery...\n");
	bat = BatteryInit(&hadc1, 2.0F);
	bat->refresh(bat->p);
	print("  Volt: %1.2f\n", bat->voltage(bat->p));
	print("\n");
	HAL_Delay(3000);

	print("Init MAX30102...\n");
	max30102 = MAX30102Init(
			&hi2c1, MAX_30102_ADDR,
			MAX_INT_GPIO_Port, MAX_INT_Pin);
	max30102->reset(max30102->p);
	max30102->clrint(max30102->p);
	max30102->init(max30102->p);
	max30102->waitint(max30102->p);
	max30102->fifo(max30102->p, &red, &ir);
	print("  Red: %d\n", red);
	print("  IR: %d\n", ir);
	print("\n");
	HAL_Delay(3000);

	print("Init MPU...\n");
	mpu = MPUInit(&huart2, 9.8F);
	mpu->reset(mpu->p);
	HAL_Delay(1000);
	float x = 0.0F, y = 0.0F, z = 0.0F;
	while (x == 0.0F) mpu->acc(mpu->p, &x, &y, &z);
	float sum = sqrt(x * x + y * y + z * z);
	print("  AccX: %f\n", x);
	print("  AccY: %f\n", y);
	print("  AccZ: %f\n", z);
	print("  AccSum: %f\n", sum);
	print("  Temper: %f\n", mpu->p->temper);
	print("\n");
	HAL_Delay(3000);

	print("Init flash...\n");
	flash = FlashInit(&hspi2, FLASH_CS_GPIO_Port, FLASH_CS_Pin, W25Q128);
	flashOK = flash->begin(flash->p);
	print("  State: %d\n", flashOK);
	print("  PID: 0x%X\n", flash->readPartID(flash->p));
	print("  UID: 0x%X\n", flash->readUniqueID(flash->p));
	print("\n");
	HAL_Delay(3000);

	print("Init BLE...\n");
	ble = BLEInit(&huart1,
			BLE_RST_GPIO_Port, BLE_RST_Pin,
			BLE_STE_GPIO_Port, BLE_STE_Pin,
			BLE_LED_GPIO_Port, BLE_LED_Pin);
	ble->reset(ble->p);
	HAL_Delay(500);
	ble->write(ble->p, "AT");
	HAL_Delay(500);
	ble->write(ble->p, "AT+NAME");
	HAL_Delay(500);
	ble->write(ble->p, "AT+ROLE");
	print("\n");

	print("Starting System...\n\n");
	HAL_Delay(1000);
	dev->clear(dev->p);
}
