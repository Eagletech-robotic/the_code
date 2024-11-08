
// code issue de https://www.pm-robotix.eu/2022/01/19/ameliorer-vos-regulateurs-pid/#Direction-du-contr%C3%B4leur
/*working variables*/

//unsigned long lastTime;
//float Input, Output, Setpoint;
//float ITerm, lastInput;
//float kp, ki, kd;
//int SampleTime_hz = 1000; //1 sec
//float outMin, outMax;
//bool inAuto = false;

//#define MANUAL 0
//#define AUTOMATIC 1

//#define DIRECT 0
//#define REVERSE 1
//int controllerDirection = DIRECT;

#include "robotic/pid.h"
#include <stdio.h>  //printf

float clamp(float val, float Min, float Max) {
	float ret = val;

	if (val > Max) {
		ret = Max;
	} else if (val < Min) {
		ret = Min;
	}
	return ret;
}

void pid_print(pid_t* pid) {
	printf("%f %f %f\r\n", pid->kp, pid->ki , pid->kd);
	printf("%f : %f (%f)\r\n", pid->Setpoint, pid->Output , pid->Input);
	printf(" %f %f %f\r\n", pid->ITerm, pid->lastInput , pid->Input);
	printf(" [%f %f] %i %i\r\n", pid->outMin, pid->outMax,pid->isModeAuto, pid->isControllerDirectionDirect);
}

void Initialize(pid_t* pid)
{
   pid->lastInput = pid->Input;
   pid->ITerm = pid->Output;
   pid->ITerm = clamp(pid->ITerm, pid->outMin, pid->outMax);
}

void pid_init(pid_t *pid) {
	pid_t pid_ = {0};
	*pid = pid_;
	pid->SampleTime_hz = 1000;
	pid->isModeAuto = 1;
	pid->isControllerDirectionDirect = 1;
	Initialize(pid);
}

void pid_command(pid_t * pid, float setpoint) {
	pid->Setpoint = setpoint;
}

float pid_compute(pid_t * pid, float input) {

	if(!pid->isModeAuto) return 0.0; // attention, cette valeur n'a peut être aucun sens

	pid->Input = input;
	/*Compute all the working error variables*/
	float error = pid->Setpoint - pid->Input;
	printf("                             err=%f\r\n", error);
	pid->ITerm += (pid->ki * error);

	// anti windup
	pid->ITerm = clamp(pid->ITerm,pid->outMin,pid->outMax);

	float dInput = (pid->Input - pid->lastInput);

	/*Compute PID Output*/
	pid->Output = pid->kp * error + pid->ITerm - pid->kd * dInput;

	pid->Output = clamp(pid->Output,pid->outMin,pid->outMax);

	pid->lastInput = pid->Input;
	return pid->Output;
}

float pid_(pid_t * pid, float cmd, float input) {
	pid_command(pid,cmd);
	float regul = pid_compute(pid,input);
	return regul;
}

//void Compute(pid_t *pid)
//{
//   if(!pid->inAuto) return;
//   unsigned long now = millis();
//   int timeChange = (now - lastTime);
//   if(timeChange>=SampleTime)
//   {
//      /*Compute all the working error variables*/
//		pid_compute(pid);
//      /*Remember some variables for next time*/
//      lastTime = now;
//   }
//}

void pid_tune(pid_t *pid,float Kp, float Ki, float Kd)
{
   if (Kp<0 || Ki<0|| Kd<0) return;

  float SampleTimeInSec = ((float)pid->SampleTime_hz)/1000;
  float kp = Kp;
  float ki = Ki * SampleTimeInSec;
  float kd = Kd / SampleTimeInSec;

  if(!pid->isControllerDirectionDirect)
   {
      kp = (0 - kp);
      ki = (0 - ki);
      kd = (0 - kd);
   }
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
}

void pid_frequency(pid_t* pid,int f)
{
   if (f > 0)
   {
      float ratio  = (float)f / (float)pid->SampleTime_hz;
      pid->ki *= ratio;
      pid->kd /= ratio;
      pid->SampleTime_hz = (unsigned long)f;
   }
}

void pid_limits(pid_t* pid, float Min, float Max)
{
   if(Min > Max) return;
   pid->outMin = Min;
   pid->outMax = Max;
   pid->Output = clamp(pid->Output, Min, Max);
   pid->ITerm = clamp(pid->ITerm, Min, Max);
}

void pid_mode(pid_t* pid, int Mode)
{
    if(Mode)
    {  /*we just went from manual to auto*/
        Initialize(pid);
    }
    pid->isModeAuto = Mode;
}

void pid_stop(pid_t* pid) {
	pid_mode(pid, 0);
}

void pid_start(pid_t* pid) {
	pid_mode(pid, 1);
}

void pid_SetControllerDirection(pid_t* pid, int Direction)
{
   pid->isControllerDirectionDirect = Direction;
}

int pid_test() {
	pid_t pid;

	pid_init(&pid);
	pid_tune(&pid,1.0,1.0,1.0); // valeur bien trop grande
	pid_limits(&pid,-110.0,110.0);
	//inloop simulation
	pid_command(&pid,100.0);
	float sensor =1.0;
	float output;
	for(int i =0; i<1000;i++){
		output = pid_compute(&pid, sensor);

		// simulation de système
		if (output > sensor) {
			sensor +=1.0f;
		} else {
			sensor -=1.0f;
		}
		//pid_print(&pid);
		printf("%i %f\r\n",i,output);
	}
	pid_command(&pid,-10.0);
	for(int i =0; i<1000;i++){
		output = pid_compute(&pid, sensor);

		// simulation de système
		if (output > sensor) {
			sensor +=1.0f;
		} else {
			sensor -=1.0f;
		}
		printf("%i %f\r\n",i,output);
	}
	return output != 0.0; // oui, je sais...
}
