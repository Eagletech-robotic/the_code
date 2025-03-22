/*
 * falcon.c
 *
 *  Created on: Nov 3, 2024
 *      Author: nboulay
 */

#include "nicolas.h"

#include "iot01A/top_driver.h" // c'est la dépendance au interface, rien d'autre n'est authorisé à être inclus depuis iot01A
#include "robotic/carre.h"
#include <stdio.h>
#include "robotic/pid.h"
#include "iot01A/sensors.h"
#include "robotic/inertial.h"
#include "robotic/projection.h"
#include <math.h>

carre_t carre;

pid_t pid_diff;
pid_t pid_sum;

float curve(float v1, float v2) {
	return (v1-v2) / (v1+v2);
}

void autopilot_init(config_t * config) {
	pid_init(&pid_diff);
	pid_tune(&pid_diff, 0.00033  , 0.0000000, 0.000001); // avec ki =0 est le seul moyen d'avoir R inférieur à 1% d'erreur (autour de 12-15v), ki fait diverger
	pid_limits(&pid_diff, -2.0, 2.0);
	float f = 1000.0 / config->time_step_ms;
	pid_frequency(&pid_diff, f);

	pid_init(&pid_sum);
	pid_tune(&pid_sum, 0.0003 , 0.0000000, 0.000001); //ki est toujours trop instable au démarrage.
	pid_limits(&pid_sum, -2.0, 2.0);
	pid_frequency(&pid_sum, f);

}

// Fonction pour envoyer une valeur ITM
//void ITM_SendValue(uint32_t port, uint32_t value) {
//    if ((ITM->TCR & ITM_TCR_ITMENA_Msk) && // Vérifie que l'ITM est activé
//        (ITM->TER & (1UL << port))) {      // Vérifie que le port est activé
//        while (ITM->PORT[port].u32 == 0);  // Attendre que le port soit prêt
//        ITM->PORT[port].u32 = value;       // Écrire la valeur sur le port
//    }
//}
//
//// Fonction pour envoyer des flottants via SWV
//void ITM_SendFloat(uint32_t port, float value) {
//    uint32_t scaled_value = (uint32_t)(value * 1000); // Exemple : échelle de 3 décimales
//    ITM_SendValue(port, scaled_value);
//}

// pour éviter de créer une nouvelle structure et pour évite de perdre le flux d'information, il faut envoyer un ouput vierge
// et recopier les données utiles. Cela évite de se perdre dans les mise à jours de valeurs
void autopilot(config_t * config, input_t * input, float v1, float v2, output_t* ret)
{
	float sensor1 = input->encoder1;
	float sensor2 = input->encoder2;

	float regul_sum = pid_(&pid_sum, v1+v2, sensor1 + sensor2);
	float regul_diff = pid_(&pid_diff, v1-v2, sensor1 - sensor2);

	ret->vitesse1_ratio = (regul_sum + regul_diff) / 2.0;
	ret->vitesse2_ratio = (regul_sum - regul_diff) / 2.0;

	//float cmd_curve = curve(v1,v2);
	//float sensor_curve = curve(sensor1, sensor2);
	//printf("%.4f %.4f\r\n", cmd_curve, sensor_curve);
}

inertial_t inertial;
//int first = 1;
//float m_1[3];

//  doit appeler la fonction et gérer les IOS
void nicolas_top_step(config_t* config, input_t *input, output_t* output ) {
	//float gyrox,gyroy,gyroz;float accx,accy,accz;
	//float magx, magy, magz;
	//getInertial6D(&accx, &accy, &accz, &gyrox, &gyroy, &gyroz); //todo : à mettre dans input
	//getAxisMagnetometer(&magx, &magy, &magz);

	//float gyro = sqrtf(gyrox*gyrox+gyroy*gyroy+gyroz*gyroz);
	//int diffe = input->encoder1 -input->encoder2;
	//printf("%.2f   %d  %.2f\r\n", gyro, diffe, (1.0f*diffe)/gyro);

	//float M[3];
	//M[0] = magx;
	//M[1] = magy;
	//M[2] = magz;
	//projectOnPlane(config->g,M, M2D);

	//printf("%.2f %.2f %.2f  %.2f %.2f %.2f  %.2f %.2f\r\n", magx, magy, magz, accx,accy,accz, M2D[0], M2D[1]);
	//printf("%.2f %.2f\r\n", M2D[0], M2D[1]);

//	if (first) {
//		m_1[0] = M[0];
//		m_1[1] = M[1];
//		m_1[2] = M[2];
//		first=0;
//	} else {
//		float a = angleBetweenVectors(m_1,M);
//		printf("radian=%.3f \r\n",a);
//	}
	//float acc[3];
	//acc[0]=accx;
	//acc[1]=accy;
	//acc[2]=accz;
	//float a = computeHeading(M, acc);
//	printf("heading=%.3f \r\n",a);
	//stat_nr(a); //157 198
	//calibrate_nr(M);
	carre_in_loop(&carre, output);
	//carre_in_loop_with_heading(&carre, a, output);

	output_t ret;
	autopilot(config, input, output->vitesse1_ratio, output->vitesse2_ratio, &ret);
	output->vitesse1_ratio=ret.vitesse1_ratio;
	output->vitesse2_ratio=ret.vitesse2_ratio;

	//printf("%.2f %.2f %li %li\r\n", output->vitesse1_ratio, output->vitesse2_ratio, input->encoder1, input->encoder2);

	//float v_cg_i_ex ;
	//float v_cg_i_ey;

	//inertial_step(&inertial, input->encoder1, input->encoder2, &v_cg_i_ex, &v_cg_i_ey);
	//printf("x= %.4f y=%.4f theta=%.4f\r\n",inertial.p_cg_i_ex, inertial.p_cg_i_ey,inertial.theta_e);
	//float r = curve(input->encoder1,input->encoder2);
	//printf(" real=%f (cmd=%f err=%.4f%%)\r\n", r, r_, (r_-r)*100.0/r_);
	//printf("%.4f \r\n", input->tof_m);
}

// TODO :
// utiliser mag pour faire le carré
// faire une projection pour passer en 2D
// mesurer une différence d'angle entre 2 vecteur 2D (pour faire le carré et les virage à 90°)
// regarder la lib arduino stm32 pour l'iot pour récupérer le code + rapide -> trop merdique
// 2ms pour le mag, c'est long.

// les senseurs sont déjà initialisés
//void measure_g(float g[3]) {
//	float gyrox,gyroy,gyroz;float accx,accy,accz;
//	g[0]=0.0f;
//	g[1]=0.0f;
//	g[2]=0.0f;
//	for (int i=0;i<10;i++) {
//		//getInertial6D(&accx, &accy, &accz, &gyroz, &gyroy, &gyrox);
//		g[0]+=accx;
//		g[1]+=accy;
//		g[2]+=accy;
//	}
//	g[0] /=10.0;
//	g[1] /=10.0;
//	g[2] /=10.0;
//	//printf("[g] %.2f %.2f %.2f \r\n",g[0],g[1],g[2]);
//}

void nicolas_top_init(config_t* config) {
	config->time_step_ms = 4; // il faudrait 250hz, les get par I2C sont trop lent
	printf(":: %i\r\n",config->time_step_ms);
	carre_init(&carre, config->time_step_ms / 1000.0);
	autopilot_init(config);
	//init_inertial();
	inertial_init(&inertial, 0.0065/2, 0.33, config->time_step_ms, 500*4*36);
	//init_magnetometer();
	//init_inertial(); //sensors.c todo devrait être sensors_init
}

//TODO :
// virer le g dans input qui ne sert à rien
// accelero/gyro/magneto du code ? Ou changer de carte pour retenter le heading dans l'input ?
// VL53L0X dans l'input
// connectivité externe en bluetooth
// virer le code commenté
//Elec
// prévoir les commandes de servo + alim servo 6V
// Alim 5V de la carte
// Jack de départ
