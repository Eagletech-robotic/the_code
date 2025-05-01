/*
 * sensors.c
 *
 *  Created on: 29 gen 2021
 *      Author: Raul Rosa
 */

#include "iot01A/sensors.h"
#include "stm32l4xx_hal.h"
#include <stdlib.h>

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi3;
extern I2C_HandleTypeDef hi2c2;

int T_C0 = 0;
int T_C1 = 0;
int16_t T_C0_lsb = 0;
int16_t T_C1_lsb = 0;
float m = 0;

uint8_t H_0 = 0;
uint8_t H_1 = 0;
int16_t H_0_lsb = 0;
int16_t H_1_lsb = 0;
float mh = 0;

void initLPS22hh() {
    uint8_t addressWrite = 0xba;
    uint8_t turnOn[] = {0x10, 0x20}; // The address of the register and the value of the register to turn on the sensor

    HAL_I2C_Master_Transmit(&hi2c2, addressWrite, turnOn, 2, 1);
}

void getPressure(float *pressure) {

    int lsb = 0;
    uint8_t addressWrite = 0xba;
    uint8_t addressRead = 0xbb;
    uint8_t pressXL[] = {0x28};
    uint8_t pressL[] = {0x29};
    uint8_t pressH[] = {0x2a};
    uint8_t data[2];

    HAL_I2C_Master_Transmit(&hi2c2, addressWrite, pressXL, 2, 1);
    HAL_I2C_Master_Transmit(&hi2c2, addressRead, data, 2, 1);
    lsb = data[0];

    HAL_I2C_Master_Transmit(&hi2c2, addressWrite, pressL, 2, 1);
    HAL_I2C_Master_Transmit(&hi2c2, addressRead, data, 2, 1);
    lsb |= data[0] << 8;

    HAL_I2C_Master_Transmit(&hi2c2, addressWrite, pressH, 2, 1);
    HAL_I2C_Master_Transmit(&hi2c2, addressRead, data, 2, 1);
    lsb |= data[0] << 16;

    if (lsb > 8388607) {
        lsb = lsb - 1;
        lsb = ~lsb;
    }

    *pressure = ((float)lsb) / ((float)(4096));
}

void initHTS221() {
    uint8_t addressWrite = 0xbe;
    uint8_t addressRead = 0xbf;
    uint8_t turnOn[] = {0x20, 0x81}; // The address of the register and the value of the register to turn on the sensor
    uint8_t data[2];

    HAL_I2C_Master_Transmit(&hi2c2, addressWrite, turnOn, 2, 1);

    uint8_t tempMinAddress[] = {0x32};
    uint8_t tempMaxAddress[] = {0x33};
    // reading low temperature calibration lsb
    HAL_I2C_Master_Transmit(&hi2c2, addressWrite, tempMinAddress, 1, 1);
    HAL_I2C_Master_Receive(&hi2c2, addressRead, data, 1, 1);
    T_C0 = data[0];

    // reading high temperature calibration lsb
    HAL_I2C_Master_Transmit(&hi2c2, addressWrite, tempMaxAddress, 1, 1);
    HAL_I2C_Master_Receive(&hi2c2, addressRead, data, 1, 1);
    T_C1 = data[0];

    uint8_t MSB_temp[] = {0x35};
    HAL_I2C_Master_Transmit(&hi2c2, addressWrite, MSB_temp, 1, 1);
    HAL_I2C_Master_Receive(&hi2c2, addressRead, data, 1, 1);

    T_C0 |= ((data[0] & 0x03) << 8);
    T_C1 |= (((data[0] & 0x0c) >> 2) << 8);

    T_C0 = T_C0 >> 3;
    T_C1 = T_C1 >> 3;

    uint8_t ADC0L[] = {0x3c};
    uint8_t ADC0H[] = {0x3d};
    // get the calibration adc min
    HAL_I2C_Master_Transmit(&hi2c2, addressWrite, ADC0L, 1, 1);
    HAL_I2C_Master_Receive(&hi2c2, addressRead, data, 1, 1);
    T_C0_lsb = data[0];

    HAL_I2C_Master_Transmit(&hi2c2, addressWrite, ADC0H, 1, 1);
    HAL_I2C_Master_Receive(&hi2c2, addressRead, data, 1, 1);
    T_C0_lsb |= data[0] << 8;

    uint8_t ADC1L[] = {0x3e};
    uint8_t ADC1H[] = {0x3f};
    // leggo temperatura
    HAL_I2C_Master_Transmit(&hi2c2, addressWrite, ADC1L, 1, 1);
    HAL_I2C_Master_Receive(&hi2c2, addressRead, data, 1, 1);
    T_C1_lsb = data[0];

    HAL_I2C_Master_Transmit(&hi2c2, addressWrite, ADC1H, 1, 1);
    HAL_I2C_Master_Receive(&hi2c2, addressRead, data, 1, 1);
    T_C1_lsb |= data[0] << 8;

    m = ((float)(T_C1 - T_C0)) / ((float)(T_C1_lsb - T_C0_lsb));

    uint8_t HumMinAddress[] = {0x30};
    uint8_t HumMaxAddress[] = {0x31};
    // reading low temperature calibration lsb
    HAL_I2C_Master_Transmit(&hi2c2, addressWrite, HumMinAddress, 1, 1);
    HAL_I2C_Master_Receive(&hi2c2, addressRead, data, 1, 1);
    H_0 = data[0];

    // reading high temperature calibration lsb
    HAL_I2C_Master_Transmit(&hi2c2, addressWrite, HumMaxAddress, 1, 1);
    HAL_I2C_Master_Receive(&hi2c2, addressRead, data, 1, 1);
    H_1 = data[0];

    H_0 = H_0 >> 1;
    H_1 = H_1 >> 1;

    ADC0L[0] = 0x36;
    ADC0H[0] = 0x37;
    // get the calibration adc min
    HAL_I2C_Master_Transmit(&hi2c2, addressWrite, ADC0L, 1, 1);
    HAL_I2C_Master_Receive(&hi2c2, addressRead, data, 1, 1);
    H_0_lsb = data[0];

    HAL_I2C_Master_Transmit(&hi2c2, addressWrite, ADC0H, 1, 1);
    HAL_I2C_Master_Receive(&hi2c2, addressRead, data, 1, 1);
    H_0_lsb |= data[0] << 8;

    ADC1L[0] = 0x3a;
    ADC1H[0] = 0x3b;
    // leggo temperatura
    HAL_I2C_Master_Transmit(&hi2c2, addressWrite, ADC1L, 1, 1);
    HAL_I2C_Master_Receive(&hi2c2, addressRead, data, 1, 1);
    H_1_lsb = data[0];

    HAL_I2C_Master_Transmit(&hi2c2, addressWrite, ADC1H, 1, 1);
    HAL_I2C_Master_Receive(&hi2c2, addressRead, data, 1, 1);
    H_1_lsb |= data[0] << 8;

    mh = ((float)(H_1 - H_0)) / ((float)(H_1_lsb - H_0_lsb));
}

void getHumidity(float *humidity) {

    uint8_t humL[] = {0x28};
    uint8_t humH[] = {0x29};
    uint8_t data[2];
    uint8_t addressWrite = 0xbe;
    uint8_t addressRead = 0xbf;
    int16_t hum;
    // reading temperature
    HAL_I2C_Master_Transmit(&hi2c2, addressWrite, humL, 1, 1);
    HAL_I2C_Master_Receive(&hi2c2, addressRead, data, 1, 1);
    hum = data[0];

    // high register
    HAL_I2C_Master_Transmit(&hi2c2, addressWrite, humH, 1, 1);
    HAL_I2C_Master_Receive(&hi2c2, addressRead, data, 1, 1);
    hum |= data[0] << 1;

    *humidity = H_0 + mh * hum;
}

void getTemperature(float *temperature) {

    uint8_t tempL[] = {0x2A};
    uint8_t tempH[] = {0x2B};
    uint8_t data[2];
    uint8_t addressWrite = 0xbe;
    uint8_t addressRead = 0xbf;
    int16_t temp;
    // reading temperature
    HAL_I2C_Master_Transmit(&hi2c2, addressWrite, tempL, 1, 1);
    HAL_I2C_Master_Receive(&hi2c2, addressRead, data, 1, 1);
    temp = data[0];

    // high register
    HAL_I2C_Master_Transmit(&hi2c2, addressWrite, tempH, 1, 1);
    HAL_I2C_Master_Receive(&hi2c2, addressRead, data, 1, 1);
    temp |= data[0] << 8;

    *temperature = T_C0 + m * temp;
}

// PC6
#define TOF_RESET_Pin GPIO_PIN_6
#define TOF_RESET_GPIO_Port GPIOC

void writereg(uint8_t reg, uint8_t value) {
	uint8_t addressWrite = 0x52;
	uint8_t turnOn[] = {0x00, 0x01};
	turnOn[0] = reg;
	turnOn[1] = value;
	HAL_I2C_Master_Transmit(&hi2c2, addressWrite, turnOn, 2, 1);
}

void writereg16(uint8_t reg, uint16_t value) {
	uint8_t addressWrite = 0x52;
	uint8_t turnOn[] = {0x00, 0x01, 0x02};
	turnOn[0] = reg;
	turnOn[1] = (value >> 8) & 0xFF;
	turnOn[2] = value & 0xFF;
	HAL_I2C_Master_Transmit(&hi2c2, addressWrite, turnOn, 2, 1);
}

uint16_t readreg(uint8_t reg) {
    uint8_t addressWrite = 0x52;
    uint8_t addressRead = 0x53;
    uint8_t resultAddress[] = {0x1e};
    resultAddress[0]=reg;
    uint8_t rawData[] = {0, 0};
    HAL_I2C_Master_Transmit(&hi2c2, addressWrite, resultAddress, 1, 1);
    HAL_I2C_Master_Receive(&hi2c2, addressRead, rawData, 2, 1);

    return (rawData[0] << 8) + rawData[1];
}

void startToF() {

	HAL_GPIO_WritePin(TOF_RESET_GPIO_Port, TOF_RESET_Pin, GPIO_PIN_SET);
	writereg(0x00, 0X1);

	// --- "tunning"
	writereg(0xFF, 0x01);
	writereg(0x00, 0x00);

	writereg(0xFF, 0x00);
	writereg(0x09, 0x00);
	writereg(0x10, 0x00);
	writereg(0x11, 0x00);

	writereg(0x24, 0x01);
	writereg(0x25, 0xFF);
	writereg(0x75, 0x00);

	writereg(0xFF, 0x01);
	writereg(0x4E, 0x2C);
	writereg(0x48, 0x00);
	writereg(0x30, 0x20);

	writereg(0xFF, 0x00);
	writereg(0x30, 0x09);
	writereg(0x54, 0x00);
	writereg(0x31, 0x04);
	writereg(0x32, 0x03);
	writereg(0x40, 0x83);
	writereg(0x46, 0x25);
	writereg(0x60, 0x00);
	writereg(0x27, 0x00);
	writereg(0x50, 0x06);
	writereg(0x51, 0x00);
	writereg(0x52, 0x96);
	writereg(0x56, 0x08);
	writereg(0x57, 0x30);
	writereg(0x61, 0x00);
	writereg(0x62, 0x00);
	writereg(0x64, 0x00);
	writereg(0x65, 0x00);
	writereg(0x66, 0xA0);

	writereg(0xFF, 0x01);
	writereg(0x22, 0x32);
	writereg(0x47, 0x14);
	writereg(0x49, 0xFF);
	writereg(0x4A, 0x00);

	writereg(0xFF, 0x00);
	writereg(0x7A, 0x0A);
	writereg(0x7B, 0x00);
	writereg(0x78, 0x21);

	writereg(0xFF, 0x01);
	writereg(0x23, 0x34);
	writereg(0x42, 0x00);
	writereg(0x44, 0xFF);
	writereg(0x45, 0x26);
	writereg(0x46, 0x05);
	writereg(0x40, 0x40);
	writereg(0x0E, 0x06);
	writereg(0x20, 0x1A);
	writereg(0x43, 0x40);

	writereg(0xFF, 0x00);
	writereg(0x34, 0x03);
	writereg(0x35, 0x44);

	writereg(0xFF, 0x01);
	writereg(0x31, 0x04);
	writereg(0x4B, 0x09);
	writereg(0x4C, 0x05);
	writereg(0x4D, 0x04);

	writereg(0xFF, 0x00);
	writereg(0x44, 0x00);
	writereg(0x45, 0x20);
	writereg(0x47, 0x08);
	writereg(0x48, 0x28);
	writereg(0x67, 0x00);
	writereg(0x70, 0x04);
	writereg(0x71, 0x01);
	writereg(0x72, 0xFE);
	writereg(0x76, 0x00);
	writereg(0x77, 0x00);

	writereg(0xFF, 0x01);
	writereg(0x0D, 0x01);

	writereg(0xFF, 0x00);
	writereg(0x80, 0x01);
	writereg(0x01, 0xF8);

	writereg(0xFF, 0x01);
	writereg(0x8E, 0x01);
	writereg(0x00, 0x01);
	writereg(0xFF, 0x00);
	writereg(0x80, 0x00);

	// augmentation du temps de mesure "pifométrique" car le vrai calcul est long
	uint8_t FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI        = 0x71;
	writereg16(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,500); //10000 2s
	// continuous back-to-back mode
	writereg(0x0, 0x02); // VL53L0X_REG_SYSRANGE_MODE_BACKTOBACK
}

void getDistance(int *distance) {
    *distance = readreg(0x1e) -20;

    if (*distance < 0) {
        *distance = 0;
    }
    if (*distance > 2000) {
        *distance = 2000;
    }
}

void init_accelerometer() {
    uint8_t addressWrite = 0xd4;
    uint8_t turnOn[] = {0x10, 0x10};
    // uint8_t turnOn[]={0x10,0x60};
    HAL_I2C_Master_Transmit(&hi2c2, addressWrite, turnOn, 2, 1);
}

void init_inertial() {
    uint8_t addressWrite = 0xd4;

    // Accelero
    uint8_t turnOn[] = {0x10, 0x60}; // 416 Hz high performance
    HAL_I2C_Master_Transmit(&hi2c2, addressWrite, turnOn, 2, 1);
    // gyro
    uint8_t turnOnGyro[] = {0x11, 0x60}; // 416 Hz high performance
    HAL_I2C_Master_Transmit(&hi2c2, addressWrite, turnOnGyro, 2, 1);
}

// acc en m.s
// gyro en rad/S ?
void getInertial6D(float *accx_, float *accy_, float *accz_, float *gyrox_, float *gyroy_, float *gyroz_) {

    uint8_t addressWrite = 0xd4;
    uint8_t addressRead = 0xd5;

    uint8_t ACCcmd[1];
    uint8_t ACCread[1];
    int16_t accx;
    int16_t accy;
    int16_t accz;
    int16_t gyrox;
    int16_t gyroy;
    int16_t gyroz;

    // ACC X
    ACCcmd[0] = 0x28;
    HAL_I2C_Master_Transmit(&hi2c2, addressWrite, ACCcmd, 1, 1);
    HAL_I2C_Master_Receive(&hi2c2, addressRead, ACCread, 1, 1);
    accx = ACCread[0];

    ACCcmd[0] = 0x29;
    HAL_I2C_Master_Transmit(&hi2c2, addressWrite, ACCcmd, 1, 1);
    HAL_I2C_Master_Receive(&hi2c2, addressRead, ACCread, 1, 1);
    accx |= ((ACCread[0]) << 8);

    // ACC Y

    ACCcmd[0] = 0x2a;
    HAL_I2C_Master_Transmit(&hi2c2, addressWrite, ACCcmd, 1, 1);
    HAL_I2C_Master_Receive(&hi2c2, addressRead, ACCread, 1, 1);
    accy = ACCread[0];

    ACCcmd[0] = 0x2b;
    HAL_I2C_Master_Transmit(&hi2c2, addressWrite, ACCcmd, 1, 1);
    HAL_I2C_Master_Receive(&hi2c2, addressRead, ACCread, 1, 1);
    accy |= ((ACCread[0]) << 8);

    // ACC Z

    ACCcmd[0] = 0x2c;
    HAL_I2C_Master_Transmit(&hi2c2, addressWrite, ACCcmd, 1, 1);
    HAL_I2C_Master_Receive(&hi2c2, addressRead, ACCread, 1, 1);
    accz = ACCread[0];

    ACCcmd[0] = 0x2d;
    HAL_I2C_Master_Transmit(&hi2c2, addressWrite, ACCcmd, 1, 1);
    HAL_I2C_Master_Receive(&hi2c2, addressRead, ACCread, 1, 1);
    accz |= ((ACCread[0]) << 8);

    *accx_ = accx / 16.0f;
    *accy_ = accy / 16.0f;
    *accz_ = accz / 16.0f;

    // GYRO X
    ACCcmd[0] = 0x22;
    HAL_I2C_Master_Transmit(&hi2c2, addressWrite, ACCcmd, 1, 1);
    HAL_I2C_Master_Receive(&hi2c2, addressRead, ACCread, 1, 1);
    gyrox = ACCread[0];

    ACCcmd[0] = 0x23;
    HAL_I2C_Master_Transmit(&hi2c2, addressWrite, ACCcmd, 1, 1);
    HAL_I2C_Master_Receive(&hi2c2, addressRead, ACCread, 1, 1);
    gyrox |= ((ACCread[0]) << 8);

    // GYRO Y

    ACCcmd[0] = 0x24;
    HAL_I2C_Master_Transmit(&hi2c2, addressWrite, ACCcmd, 1, 1);
    HAL_I2C_Master_Receive(&hi2c2, addressRead, ACCread, 1, 1);
    gyroy = ACCread[0];

    ACCcmd[0] = 0x25;
    HAL_I2C_Master_Transmit(&hi2c2, addressWrite, ACCcmd, 1, 1);
    HAL_I2C_Master_Receive(&hi2c2, addressRead, ACCread, 1, 1);
    gyroy |= ((ACCread[0]) << 8);

    // ACC Z

    ACCcmd[0] = 0x26;
    HAL_I2C_Master_Transmit(&hi2c2, addressWrite, ACCcmd, 1, 1);
    HAL_I2C_Master_Receive(&hi2c2, addressRead, ACCread, 1, 1);
    gyroz = ACCread[0];

    ACCcmd[0] = 0x27;
    HAL_I2C_Master_Transmit(&hi2c2, addressWrite, ACCcmd, 1, 1);
    HAL_I2C_Master_Receive(&hi2c2, addressRead, ACCread, 1, 1);
    gyroz |= ((ACCread[0]) << 8);

    *gyrox_ = gyrox / 1000.0f;
    *gyroy_ = gyroy / 1000.0f;
    *gyroz_ = gyroz / 1000.0f;
}

void getAxisAccelerometer(int16_t *accx, int16_t *accy, int16_t *accz) {

    uint8_t addressWrite = 0xd4;
    uint8_t addressRead = 0xd5;

    uint8_t ACCcmd[1];
    uint8_t ACCread[1];

    // ACC X
    ACCcmd[0] = 0x28;
    HAL_I2C_Master_Transmit(&hi2c2, addressWrite, ACCcmd, 1, 1);
    HAL_I2C_Master_Receive(&hi2c2, addressRead, ACCread, 1, 1);
    *accx = ACCread[0];

    ACCcmd[0] = 0x29;
    HAL_I2C_Master_Transmit(&hi2c2, addressWrite, ACCcmd, 1, 1);
    HAL_I2C_Master_Receive(&hi2c2, addressRead, ACCread, 1, 1);
    *accx |= ((ACCread[0]) << 8);

    // ACC Y

    ACCcmd[0] = 0x2a;
    HAL_I2C_Master_Transmit(&hi2c2, addressWrite, ACCcmd, 1, 1);
    HAL_I2C_Master_Receive(&hi2c2, addressRead, ACCread, 1, 1);
    *accy = ACCread[0];

    ACCcmd[0] = 0x2b;
    HAL_I2C_Master_Transmit(&hi2c2, addressWrite, ACCcmd, 1, 1);
    HAL_I2C_Master_Receive(&hi2c2, addressRead, ACCread, 1, 1);
    *accy |= ((ACCread[0]) << 8);

    // ACC Z

    ACCcmd[0] = 0x2c;
    HAL_I2C_Master_Transmit(&hi2c2, addressWrite, ACCcmd, 1, 1);
    HAL_I2C_Master_Receive(&hi2c2, addressRead, ACCread, 1, 1);
    *accz = ACCread[0];

    ACCcmd[0] = 0x2d;
    HAL_I2C_Master_Transmit(&hi2c2, addressWrite, ACCcmd, 1, 1);
    HAL_I2C_Master_Receive(&hi2c2, addressRead, ACCread, 1, 1);
    *accz |= ((ACCread[0]) << 8);

    *accx = *accx / 16;
    *accy = *accy / 16;
    *accz = *accz / 16;
}

int16_t getMicrophonedb(int analogValue) {

    if (analogValue <= 0) {
        analogValue = -analogValue;
    }

    uint8_t bit = getfirstValidBit(analogValue);

    return 47 + (bit * 3);
}

int getfirstValidBit(int absoluteValue) {
    uint8_t counter;
    counter = 0;

    while (absoluteValue > 0) {
        absoluteValue = absoluteValue >> 1;
        counter++;
    }
    return counter;
}

void mag_write(uint8_t reg, uint8_t data_) {
    uint8_t addressWrite = 0x3c;
    uint8_t data[2];
    data[0] = reg;
    data[1] = data_;
    HAL_I2C_Master_Transmit(&hi2c2, addressWrite, data, 2, 1);
}

// lis3mdl
void init_magnetometer() {

    //	 uint8_t addressWrite=0x3c;
    //	 uint8_t turnOn[]={0x22,0x00}; //Continuous-conversion mode
    //	 HAL_I2C_Master_Transmit(&hi2c2,addressWrite,turnOn,2,1);
    //
    //	 uint8_t turnOnxy[]={0x20,0x7E}; //ultrahigh-performance mode 155hz
    //	 HAL_I2C_Master_Transmit(&hi2c2,addressWrite,turnOnxy,2,1);
    //
    //	 uint8_t turnOnz[]={0x23,0xC}; //ultrahigh-performance mode en Z
    // 	 HAL_I2C_Master_Transmit(&hi2c2,addressWrite,turnOnz,2,1);

    mag_write(0x20, 0x7e); // 10hz UHP
    mag_write(0x21, 0x00); // 4Gauss
    mag_write(0x22, 0x00); // continuous mode
    mag_write(0x23, 0x0C); // UHP en Z
    mag_write(0x24, 0x40); // BDU
}

// l'unité retourné est en gauss (selon l'initiation de l'echelle utilisé)
// par défautl 2 Gauss pleine échelle sur 16 bit
// le champ magnétique terreste en France est de 0.5 gauss
void getAxisMagnetometer(float *magx_, float *magy_, float *magz_) {

    uint8_t addressWrite = 0x3c;
    uint8_t addressRead = 0x3d;

    uint8_t MAGcmd[1];
    uint8_t MAGread[1];
    int16_t magx;
    int16_t magy;
    int16_t magz;
    // MAG X
    MAGcmd[0] = 0x28;
    HAL_I2C_Master_Transmit(&hi2c2, addressWrite, MAGcmd, 1, 1);
    HAL_I2C_Master_Receive(&hi2c2, addressRead, MAGread, 1, 1);
    magx = MAGread[0];

    MAGcmd[0] = 0x29;
    HAL_I2C_Master_Transmit(&hi2c2, addressWrite, MAGcmd, 1, 1);
    HAL_I2C_Master_Receive(&hi2c2, addressRead, MAGread, 1, 1);
    magx |= ((MAGread[0]) << 8);

    // MAG Y
    MAGcmd[0] = 0x2a;
    HAL_I2C_Master_Transmit(&hi2c2, addressWrite, MAGcmd, 1, 1);
    HAL_I2C_Master_Receive(&hi2c2, addressRead, MAGread, 1, 1);
    magy = MAGread[0];

    MAGcmd[0] = 0x2b;
    HAL_I2C_Master_Transmit(&hi2c2, addressWrite, MAGcmd, 1, 1);
    HAL_I2C_Master_Receive(&hi2c2, addressRead, MAGread, 1, 1);
    magy |= ((MAGread[0]) << 8);

    // MAG Z
    MAGcmd[0] = 0x2c;
    HAL_I2C_Master_Transmit(&hi2c2, addressWrite, MAGcmd, 1, 1);
    HAL_I2C_Master_Receive(&hi2c2, addressRead, MAGread, 1, 1);
    magz = MAGread[0];

    MAGcmd[0] = 0x2d;
    HAL_I2C_Master_Transmit(&hi2c2, addressWrite, MAGcmd, 1, 1);
    HAL_I2C_Master_Receive(&hi2c2, addressRead, MAGread, 1, 1);
    magz |= ((MAGread[0]) << 8);

    //*magx_=magx/16535.0f;
    //*magy_=magy/16535.0f;
    //*magz_=magz/16535.0f;

    *magx_ = magx / 6842.0f;
    *magy_ = magy / 6842.0f;
    *magz_ = magz / 6842.0f;
}

// 22.46  ­ 0.46 0.20 -0.11  0.51
// 14.60  -0.39 0.12 -0.13  0.42
//  6.47   0.37 -0.3 -0.14  0.40

//
// void init_gyroscope(){
//
//		  uint8_t GYROcmd[2];
//		  uint8_t GYROread[2];
//		  GYROcmd[0]=0x00|0x11;
//		  GYROcmd[1]=0x12;
//		//  HAL_GPIO_WritePin(CS_LSM6DSOX_GPIO_Port,CS_LSM6DSOX_Pin,GPIO_PIN_RESET);
//		  HAL_SPI_TransmitReceive(&hspi1,GYROcmd,GYROread,2,1);
//		//  HAL_GPIO_WritePin(CS_LSM6DSOX_GPIO_Port,CS_LSM6DSOX_Pin,GPIO_PIN_SET);
//}
//
// void getAxisGyro(int16_t *gyrox, int16_t *gyroy, int16_t *gyroz){
//
//	  uint8_t GYROcmd[2];
//	  uint8_t GYROread[2];
//
//	  GYROcmd[0]=0x80|0x22;
//	  GYROcmd[1]=0x00;
////	  HAL_GPIO_WritePin(CS_LSM6DSOX_GPIO_Port,CS_LSM6DSOX_Pin,GPIO_PIN_RESET);
//	  HAL_SPI_TransmitReceive(&hspi1,GYROcmd,GYROread,2,1);
////	  HAL_GPIO_WritePin(CS_LSM6DSOX_GPIO_Port,CS_LSM6DSOX_Pin,GPIO_PIN_SET);
//
//	  *gyrox=GYROread[1];
//
//	  GYROcmd[0]=0x80|0x23;
//	  GYROcmd[1]=0x00;
////	  HAL_GPIO_WritePin(CS_LSM6DSOX_GPIO_Port,CS_LSM6DSOX_Pin,GPIO_PIN_RESET);
//	  HAL_SPI_TransmitReceive(&hspi1,GYROcmd,GYROread,2,1);
////	  HAL_GPIO_WritePin(CS_LSM6DSOX_GPIO_Port,CS_LSM6DSOX_Pin,GPIO_PIN_SET);
//
//	  *gyrox|=GYROread[1]<<8;
//
//	  *gyrox=*gyrox*4;
//
//	  GYROcmd[0]=0x80|0x24;
//	  GYROcmd[1]=0x00;
//	//  HAL_GPIO_WritePin(CS_LSM6DSOX_GPIO_Port,CS_LSM6DSOX_Pin,GPIO_PIN_RESET);
//	  HAL_SPI_TransmitReceive(&hspi1,GYROcmd,GYROread,2,1);
//	//  HAL_GPIO_WritePin(CS_LSM6DSOX_GPIO_Port,CS_LSM6DSOX_Pin,GPIO_PIN_SET);
//
//	  *gyroy=GYROread[1];
//
//
//
//	  GYROcmd[0]=0x80|0x25;
//	  GYROcmd[1]=0x00;
//	  //HAL_GPIO_WritePin(CS_LSM6DSOX_GPIO_Port,CS_LSM6DSOX_Pin,GPIO_PIN_RESET);
//	  HAL_SPI_TransmitReceive(&hspi1,GYROcmd,GYROread,2,1);
//	  //HAL_GPIO_WritePin(CS_LSM6DSOX_GPIO_Port,CS_LSM6DSOX_Pin,GPIO_PIN_SET);
//
//	  *gyroy|=GYROread[1]<<8;
//
//	  *gyroy=*gyroy*4;
//
//	  GYROcmd[0]=0x80|0x26;
//	  GYROcmd[1]=0x00;
//
//	  //HAL_GPIO_WritePin(CS_LSM6DSOX_GPIO_Port,CS_LSM6DSOX_Pin,GPIO_PIN_RESET);
//	  HAL_SPI_TransmitReceive(&hspi1,GYROcmd,GYROread,2,1);
//	  //HAL_GPIO_WritePin(CS_LSM6DSOX_GPIO_Port,CS_LSM6DSOX_Pin,GPIO_PIN_SET);
//
//	  *gyroz=GYROread[1];
//
//	  GYROcmd[0]=0x80|0x27;
//	  GYROcmd[1]=0x00;
//	  //HAL_GPIO_WritePin(CS_LSM6DSOX_GPIO_Port,CS_LSM6DSOX_Pin,GPIO_PIN_RESET);
//	  HAL_SPI_TransmitReceive(&hspi1,GYROcmd,GYROread,2,1);
//	  //HAL_GPIO_WritePin(CS_LSM6DSOX_GPIO_Port,CS_LSM6DSOX_Pin,GPIO_PIN_SET);
//
//	  *gyroz|=GYROread[1]<<8;
//
//	  *gyroz=*gyroz*4;
//
//	  *gyrox=*gyrox/1000;
//	  *gyroy=*gyroy/1000;
//	  *gyroz=*gyroz/1000;
//
//}
