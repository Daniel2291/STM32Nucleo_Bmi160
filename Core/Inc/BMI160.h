#ifndef BMI160_IMU_H
#define BMI160_IMU_H

#include "stm32f4xx_hal.h"

/* Register defines */
#define BMI_CHIP_ID 		    0x00
#define BMI_ACC_DATA 			0x12 // X direction LSB
#define BMI_TEMP_DATA 			0x21 // Temperature MSB
#define BMI_ACC_CONF 			0x40 // some low pass filtering
#define BMI_ACC_RANGE 			0x41


#define BMI_STATUS   	        0x1B  //11000000
#define BMI_INT_STATUS   	    0x1D // 00010000

#define BMI_INT_EN	   	        0x51
#define BMI_INT_OUT_CTRL 	   	0x53
#define BMI_INT_LATCH 	   	    0x54
#define BMI_INT1_INT2_MAP_DATA 	0x56
#define BMI_PMU_CONF 		    0x03
#define BMI_CMD         		0x7E

#define BMI_GYR_DATA			0x0C
#define	BMI_GYR_RANGE			0x43
#define	BMI_GYR_BANDWIDTH		0x42
#define SPI_ENABLE              0x70


typedef struct {                        //configure settings related to BMI160

	/* SPI */
	SPI_HandleTypeDef *spiHandle;       //represents the SPI (Serial Peripheral Interface) handle used by the STM32 microcontroller for communication with the IMU sensor. The SPI handle typically contains configuration parameters and runtime information for the SPI peripheral.
	GPIO_TypeDef 	  *csPinBank;    //represents the GPIO port bank (such as GPIOA, GPIOB, etc.) to which the chip select (CS) pin for the accelerometer (Acc) is connected. The chip select pin is used to enable communication with the accelerometer when the microcontroller wants to read or write data
	uint16_t 		   csPin;        //This member is an unsigned 16-bit integer representing the specific pin number within the GPIO port bank (csAccPinBank) that is used as the chip select pin for the accelerometer.


	/* DMA */
	uint8_t readingAcc;
	uint8_t readingGyr;
	uint8_t accTxBuf[8];

	uint8_t gyrTxBuf[8];
	uint8_t accRxBuf[8];
	volatile uint8_t gyrRxBuf[8];

	/* Conversion constants (raw to m/s^2 and raw to rad/s) */
	float accConversion;
	float gyrConversion;

	/* x-y-z measurements */
	float acc_mps2[3];
	float gyr_rps[3];


} BMI160;

/*
 *
 * INITIALISATION
 *
 */
uint8_t BMI160_Init(BMI160 *imu,
				 SPI_HandleTypeDef *spiHandle,
				 GPIO_TypeDef *csPinBank, uint16_t csPin);

/*
 *
 * LOW-LEVEL REGISTER FUNCTIONS
 *
 */
uint8_t BMI160_ReadRegister(BMI160 *imu, uint8_t regAddr, uint8_t *data);


uint8_t BMI160_WriteRegister(BMI160 *imu, uint8_t regAddr, uint8_t data);


/*
 *
 * POLLING
 *
 */
uint8_t BMI160_ReadAccelerometer(BMI160 *imu);
uint8_t BMI160_ReadGyroscope(BMI160 *imu);

/*
 *
 * DMA
 *
 */
uint8_t BMI160_ReadAccelerometerDMA(BMI160 *imu);
void 	BMI160_ReadAccelerometerDMA_Complete(BMI160 *imu);

uint8_t BMI160_ReadGyroscopeDMA(BMI160 *imu);
void 	BMI160_ReadGyroscopeDMA_Complete(BMI160 *imu);

#endif
