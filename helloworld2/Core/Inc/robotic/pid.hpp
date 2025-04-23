#pragma once

#ifdef __cplusplus
extern "C" {
#endif

typedef struct PID_t {
    float Input, Output, Setpoint;
    float ITerm, lastInput;
    float kp, ki, kd;
    int SampleTime_hz;
    float outMin, outMax;
    int isModeAuto;
    int isControllerDirectionDirect;
} PID_t;

void pid_init(PID_t *pid);                               // constructeur
float pid_compute(PID_t *pid, float input);              // fonction in_loop avec l'entrée capteur
void pid_command(PID_t *pid, float setpoint);            // mise à jour de la commande
void pid_tune(PID_t *pid, float Kp, float Ki, float Kd); // modif des paramètres
void pid_frequency(PID_t *pid, int f); // modification de la vitesse de sample, n'a pas raison de changer.
void pid_limits(
    PID_t *pid, float Min,
    float Max); // limite d'excursion de la sortie pour éviter les débordements inutiles des accumulateurs internes
void pid_stop(PID_t *pid);  // coupe les compteurs internes compute retourne zero
void pid_start(PID_t *pid); // reprise
void pid_SetControllerDirection(
    PID_t *pid, int Direction); // Si le capteur monte la commande doit monter ou l'inverse, dépend de l'application
float pid_(PID_t *pid, float cmd, float input);
int pid_test();

#ifdef __cplusplus
} // extern "C"
#endif