#include "BMI160.h"
#include "stm32f4xx_hal.h"

/*
 *
 * INITIALISATION
 *
 */
uint8_t BMI160_Init(BMI160 *imu,
				 SPI_HandleTypeDef *spiHandle,
				 GPIO_TypeDef *csPinBank, uint16_t csPin) {

	/* Store interface parameters in struct */
	imu->spiHandle 		= spiHandle;
	imu->csPinBank 	    = csPinBank;
	imu->csPin 		    = csPin;

	/* Clear DMA flags */
	imu->readingAcc = 0;
	imu->readingGyr = 0;

	uint8_t status = 0;

	/*
	 *
	 * ACCELEROMETER
	 *
	 */

	/* Sensor requires rising edge on CSB after start-up to activate SPI, not sure this is needed for bmi160, maybe better use register setting */
	HAL_GPIO_WritePin(imu->csPinBank, imu->csPin, GPIO_PIN_RESET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(imu->csPinBank, imu->csPin, GPIO_PIN_SET);
	HAL_Delay(100);


	/* Perform sensor soft reset */
	status += BMI160_WriteRegister(imu, BMI_CMD, 0xB6);
	HAL_Delay(50);

	/* Check chip ID */
	uint8_t chipID;
	status += BMI160_ReadRegister(imu, BMI_CHIP_ID, &chipID);

	if (chipID != 0xD1) {

		//return 0;

	}
	HAL_Delay(10);


	/* Configure accelerometer  */
	status += BMI160_WriteRegister(imu, BMI_ACC_CONF, 0b00101000); /* (no oversampling, ODR = 100 Hz, BW = 40 Hz) */
	HAL_Delay(10);

	status += BMI160_WriteRegister(imu, BMI_ACC_RANGE, 0b00000101); /* +- 4g range */
	HAL_Delay(10);


	status += BMI160_WriteRegister(imu, BMI_INT_STATUS, 0x10); // Flag DRDY data ready interrupt to be active
    HAL_Delay(10);


	status += BMI160_WriteRegister(imu, BMI_INT_EN, 0b00010000); // enable data ready interrupt
	HAL_Delay(10);


	status += BMI160_WriteRegister(imu, BMI_INT_OUT_CTRL, 0b10101010); /* INT2 disable, 0000,   INT1 = push-pull output, active low */
	HAL_Delay(10);

	status += BMI160_WriteRegister(imu, BMI_INT1_INT2_MAP_DATA, 0b10001000);
	HAL_Delay(10);


	/* Pre-compute accelerometer conversion constant (raw to m/s^2) */
	imu->accConversion =   (4.00f * 9.81f)/ 327268.0f; /* Datasheet page  */

	/* Set accelerometer TX buffer for DMA */
	imu->accTxBuf[0] = BMI_ACC_DATA | 0x80;

	/*
	 *
	 * GYROSCOPE
	 *
	 */



	/* Configure gyroscope */
	status += BMI160_WriteRegister(imu, BMI_GYR_RANGE, 0b00000010); /* +- 500 deg/s */
	HAL_Delay(10);

	status += BMI160_WriteRegister(imu, BMI_GYR_BANDWIDTH, 0b00101000); /* ODR = 800 Hz, Filter bandwidth = 254 Hz */
	HAL_Delay(10);

	/* Pre-compute gyroscope conversion constant (raw to rad/s) */
	imu->gyrConversion = (0.01745329251f * 500.0f) / 32768.0f; /*  */

	/* Set gyroscope TX buffer for DMA */
	imu->gyrTxBuf[0] = BMI_GYR_DATA | 0x80;




/*
	 *
	 * BOTH
	 *
	 */

/* Put sensor into active mode */
	//status += BMI160_WriteRegister(imu, BMI_PMU_CONF, 0b10010100);
	//HAL_Delay(10);


	status += BMI160_WriteRegister(imu, BMI_CMD, 0b00010001); //accelerometer in normal
	HAL_Delay(50);

	status += BMI160_WriteRegister(imu, BMI_CMD, 0b00010101); //gyro in normal
	HAL_Delay(50);

	return status;
}

/*
 *
 * LOW-LEVEL REGISTER FUNCTIONS
 *
 */

uint8_t BMI160_ReadRegister(BMI160 *imu, uint8_t regAddr, uint8_t *data) {

	uint8_t txBuf[2] = {regAddr | 0x80 };
	uint8_t rxBuf[2];

	HAL_GPIO_WritePin(imu->csPinBank, imu->csPin, GPIO_PIN_RESET);

	uint8_t status = (HAL_SPI_TransmitReceive(imu->spiHandle, txBuf, rxBuf, 2, HAL_MAX_DELAY) == HAL_OK);
	HAL_GPIO_WritePin(imu->csPinBank, imu->csPin, GPIO_PIN_SET);

	if (status == 1) {

		*data = rxBuf[1];

	}

	return status;

}

uint8_t BMI160_WriteRegister(BMI160 *imu, uint8_t regAddr, uint8_t data) {

	uint8_t txBuf[2] = {regAddr, data};

	HAL_GPIO_WritePin(imu->csPinBank, imu->csPin, GPIO_PIN_RESET);

	uint8_t status = (HAL_SPI_Transmit(imu->spiHandle, txBuf, 2, HAL_MAX_DELAY) == HAL_OK);
	//while(HAL_SPI_GetState(imu->spiHandle) != HAL_SPI_STATE_READY);
	HAL_GPIO_WritePin(imu->csPinBank, imu->csPin, GPIO_PIN_SET);

	return status;

}





/*
 *
 * POLLING
 *
 */
uint8_t BMI160_ReadAccelerometer(BMI160 *imu) {

	/* Read raw accelerometer data */
	uint8_t txBuf[8] = {(BMI_ACC_DATA | 0x80), 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; /* Register addr,, 6 bytes data */
	uint8_t rxBuf[8];

	HAL_GPIO_WritePin(imu->csPinBank, imu->csPin, GPIO_PIN_RESET);
	uint8_t status = (HAL_SPI_TransmitReceive(imu->spiHandle, txBuf, rxBuf, 8, HAL_MAX_DELAY) == HAL_OK);
	HAL_GPIO_WritePin(imu->csPinBank, imu->csPin, GPIO_PIN_SET);

	/* Form signed 16-bit integers */
	int16_t accX = (int16_t) ((rxBuf[2] << 8) | rxBuf[1]);
	int16_t accY = (int16_t) ((rxBuf[4] << 8) | rxBuf[3]);
	int16_t accZ = (int16_t) ((rxBuf[6] << 8) | rxBuf[5]);

	/* Convert to m/s^2 */
	imu->acc_mps2[0] = imu->accConversion * accX;
	imu->acc_mps2[1] = imu->accConversion * accY;
	imu->acc_mps2[2] = imu->accConversion * accZ;

	return status;

}

uint8_t BMI160_ReadGyroscope(BMI160 *imu) {

	/* Read raw gyroscope data */
	uint8_t txBuf[7] = {(BMI_GYR_DATA | 0x80), 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; /* Register addr, 6 bytes data */
	uint8_t rxBuf[7];

	HAL_GPIO_WritePin(imu->csPinBank, imu->csPin, GPIO_PIN_RESET);
	uint8_t status = (HAL_SPI_TransmitReceive(imu->spiHandle, txBuf, rxBuf, 7, HAL_MAX_DELAY) == HAL_OK);
	HAL_GPIO_WritePin(imu->csPinBank, imu->csPin, GPIO_PIN_SET);

	/* Form signed 16-bit integers */
	int16_t gyrX = (int16_t) ((rxBuf[2] << 8) | rxBuf[1]);
	int16_t gyrY = (int16_t) ((rxBuf[4] << 8) | rxBuf[3]);
	int16_t gyrZ = (int16_t) ((rxBuf[6] << 8) | rxBuf[5]);

	/* Convert to rad/s */
	imu->gyr_rps[0] = imu->gyrConversion * gyrX;
	imu->gyr_rps[1] = imu->gyrConversion * gyrY;
	imu->gyr_rps[2] = imu->gyrConversion * gyrZ;

	return status;

}

/*
 *
 * DMA
 *
 */
uint8_t BMI160_ReadAccelerometerDMA(BMI160 *imu) {


	HAL_GPIO_WritePin(imu->csPinBank, imu->csPin, GPIO_PIN_RESET);
	if(HAL_SPI_TransmitReceive_DMA(imu->spiHandle,(uint8_t *) imu->accTxBuf, imu->accRxBuf, 8) == HAL_OK){


		imu->readingAcc = 1;
		return 1;

	} else {

		HAL_GPIO_WritePin(imu->csPinBank, imu->csPin, GPIO_PIN_SET);
		return 0;

	}

}

void BMI160_ReadAccelerometerDMA_Complete(BMI160 *imu) {

	HAL_GPIO_WritePin(imu->csPinBank, imu->csPin, GPIO_PIN_SET);
	imu->readingAcc = 0;

	/* Form signed 16-bit integers */
	int16_t accX = (int16_t) ((imu->accRxBuf[2] << 8) | imu->accRxBuf[1]);
	int16_t accY = (int16_t) ((imu->accRxBuf[4] << 8) | imu->accRxBuf[3]);
	int16_t accZ = (int16_t) ((imu->accRxBuf[6] << 8) | imu->accRxBuf[5]);

	/* Convert to m/s^2 */
	imu->acc_mps2[0] = imu->accConversion * accX;
	imu->acc_mps2[1] = imu->accConversion * accY;
	imu->acc_mps2[2] = imu->accConversion * accZ;

}

uint8_t BMI160_ReadGyroscopeDMA(BMI160 *imu) {

	HAL_GPIO_WritePin(imu->csPinBank, imu->csPin, GPIO_PIN_RESET);
	if (HAL_SPI_TransmitReceive_DMA(imu->spiHandle, imu->gyrTxBuf, (uint8_t *) imu->gyrRxBuf, 7) == HAL_OK) {

		imu->readingGyr = 1;
		return 1;

	} else {

		HAL_GPIO_WritePin(imu->csPinBank, imu->csPin, GPIO_PIN_SET);
		return 0;

	}

}

void BMI160_ReadGyroscopeDMA_Complete(BMI160 *imu) {

	HAL_GPIO_WritePin(imu->csPinBank, imu->csPin, GPIO_PIN_SET);
	imu->readingGyr = 0;

	/* Form signed 16-bit integers */
	int16_t gyrX = (int16_t) ((imu->gyrRxBuf[2] << 8) | imu->gyrRxBuf[1]);
	int16_t gyrY = (int16_t) ((imu->gyrRxBuf[4] << 8) | imu->gyrRxBuf[3]);
	int16_t gyrZ = (int16_t) ((imu->gyrRxBuf[6] << 8) | imu->gyrRxBuf[5]);

	/* Convert to deg/s */
	imu->gyr_rps[0] = imu->gyrConversion * gyrX;
	imu->gyr_rps[1] = imu->gyrConversion * gyrY;
	imu->gyr_rps[2] = imu->gyrConversion * gyrZ;

}
