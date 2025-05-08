#pragma once
/*
 * sensors.h
 *
 *  Created on: 29 gen 2021
 *      Author: UTPM9
 */

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

void init_accelerometer();

void getAxisAccelerometer(int16_t *accx, int16_t *accy, int16_t *accz);

int16_t getMicrophonedb(int analogValue);

// a trick to make the logbase2 of a number
int getfirstValidBit(int absoluteValue);

void init_magnetometer();

void getAxisMagnetometer(float *magx, float *magy, float *magz);

void init_gyroscope();

void getAxisGyro(int16_t *gyrox, int16_t *gyroy, int16_t *gyroz);

int8_t getMicrophone();

void initHTS221();

void getTemperature(float *temperature);

void getHumidity(float *humidity);

void initLPS22hh();

void getPressure(float *pressure);

void startToF();

void getDistance(int *distance);

void init_inertial();
void getInertial6D(float *accx_, float *accy_, float *accz_, float *gyrox_, float *gyroy_, float *gyroz_);

#ifdef __cplusplus
}
#endif
