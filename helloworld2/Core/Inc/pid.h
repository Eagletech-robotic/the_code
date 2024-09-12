#pragma once
/*
 * pid.h
 *
 *  Created on: Sep 12, 2024
 *      Author: nboulay
 */

typedef struct pid_t {
	float Input, Output, Setpoint;
	float ITerm, lastInput;
	float kp, ki, kd;
	int SampleTime_hz;
	float outMin, outMax;
	int isModeAuto;
	int isControllerDirectionDirect;
} pid_t;

// constructeur
void pid_init(pid_t *pid);
float pid_compute(pid_t * pid, float input); //fonction in_loop
void pid_tune(pid_t *pid,float Kp, float Ki, float Kd); // modif des paramètres
void pid_frequency(pid_t* pid,int f); // modification de la vitesse de sample, n'a pas raison de changer.
void pid_limits(pid_t* pid, float Min, float Max);
void pid_stop(pid_t* pid);
void pid_start(pid_t* pid);
void pid_SetControllerDirection(pid_t* pid, int Direction); // Si le capteur monte la commande doit monter ou l'inverse

