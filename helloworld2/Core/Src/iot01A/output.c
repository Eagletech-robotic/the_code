
// fonction de génération des sorties.
//  le but est de pouvoir tester les autres fonctions indépendement de la cible
//  et de générer les sorties de façon fixe temporrellement à chaque cycle

#include "iot01A/output.h"
#include "iot01A/motor.h"

void outputInit(output_t * output){
	output->ratio2 = 0;
	output->ratio15 = 0;
}

void outputSet(output_t *output) {
	motorSet(output->ratio2, output->ratio15);
}

