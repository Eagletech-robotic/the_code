/*
 * falcon.c
 *
 *  Created on: Nov 3, 2024
 *      Author: nboulay
 */

#include "iot01A/top_driver.h" // c'est la dépendance au interface, rien d'autre n'est authorisé à être inclus depuis iot01A
#include "robotic/carre.h"
#include <stdio.h>
#include "robotic/pid.h"
#include "iot01A/sensors.h"

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

//  doit appeler la fonction et gérer les IOS
void top_step(config_t* config, input_t *input, output_t* output ) {
	float gyrox,gyroy,gyroz;float accx,accy,accz;
	getInertial6D(&accx, &accy, &accz, &gyroz, &gyroy, &gyrox);
	carre_in_loop(&carre, output);
	//const float ratio_speed_sensor = 0.0002;
	//output->vitesse1_ratio = 100.0 ;
	//output->vitesse2_ratio = 200.0 ; // àdroite dans le sens de la marche 3500 est 97% de la vitesse à 15V 100% à 12V à vide

	float r_ = curve(input->encoder1,input->encoder2); // la command

	output_t ret;
	autopilot(config, input, output->vitesse1_ratio, output->vitesse2_ratio, &ret);
	output->vitesse1_ratio=ret.vitesse1_ratio;
	output->vitesse2_ratio=ret.vitesse2_ratio;

	//printf("%.3f\r\n", output->vitesse1_ratio);
	float r = curve(input->encoder1,input->encoder2);

	printf(" real=%f (cmd=%f err=%.4f%%)\r\n", r, r_, (r_-r)*100.0/r_);
}


void top_init(config_t* config) {
	config->time_step_ms = 6;
	printf(":: %i\r\n",config->time_step_ms);
	carre_init(&carre, config->time_step_ms / 1000.0);
	autopilot_init(config);
	init_inertial();
}

