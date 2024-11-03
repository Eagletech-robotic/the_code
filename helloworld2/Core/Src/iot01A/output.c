
// fonction de génération des sorties.
//  le but est de pouvoir tester les autres fonctions indépendement de la cible
//  et de générer les sorties de façon fixe temporrellement à chaque cycle

#include "iot01A/output.h"
#include "iot01A/motor.h"

void output_init(output_t * output){
	output->vitesse1_ratio = 0;
	output->vitesse2_ratio = 0;
}

void output_set(output_t *output) {
	motorSet(output->vitesse1_ratio, output->vitesse2_ratio);
}

